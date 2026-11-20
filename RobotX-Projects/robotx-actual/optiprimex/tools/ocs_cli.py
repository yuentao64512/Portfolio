#!/usr/bin/env python3
"""ocs_cli: the operator console in a terminal (what the mission GUI will do, decision 22).

Publishes ui/cmd/<v> intents to the hub and prints the authority verdict and the vehicle acks; `state` prints the last
ocs/state. Every intent is first checked against the playbook (tools/playbook.py), which predicts a refusal and says
what to do first; the real verdict is still the OCS's and the vehicle's.

  ocs_cli help                 the usage patterns: which R / V state each intent needs, what it does, what comes next
  ocs_cli next                 what is allowed right now, and the obvious next step
  ocs_cli state | watch        the last ocs/state once | one line per second
  ocs_cli declare | enable_auto | disable_auto | start_task m4t0 | home | select_target green | save_point A [--lat --lon] | del_point A | hold | resume | skip (free run) | session start NAME / stop / result | abort | confirm_safe | end_run | reset
  ocs_cli start_task m0t0 | m0t1    the two no-motion test missions (S1 -> S2 -> S3 -> DONE); ocs_cli step advances m0t0
  ocs_cli goto --x 20 --y 5    ocs_cli inject assistance --lat 1.28111 --lon 103.85501 --cseq 3
  ocs_cli mission m4t0|m0t1|...   preselect the mission start_task / the RunStart trigger will run (ocs_cli mission off clears it)
  ocs_cli missions             the catalogue: code = wire task / tier / status  (m4t0 = Task 4 core, m0t1 = TEST_AUTO, +sim = fake FCU only)
  ocs_cli ready                macro: declare if needed, enable_auto if needed, wait for RunStart (R5)
  ocs_cli manual               macro: abort if a task runs, then disable_auto
  ocs_cli stop_reset           v0.1.54 macro STOP & RESET: abort -> disable_auto -> fcu_mode HOLD -> disarm -> reset (OCS R2, vehicles RESET); v0.1.59: reporting
                               left as it is (was report off); each step reported, the chain goes on past a refusal
  ocs_cli tui                  the full-screen bench console (op-tui)
  ocs_cli fence apply course|geofence [--points "LAT,LON;LAT,LON;..."]   v0.1.53: the USV's FCU geofence - the received course or the
                               OCS profile's geofence as one inclusion polygon, read back and compared; ocs_cli fence on|off
                               (MAV_CMD_DO_FENCE_ENABLE; off by default). The FENCE_* parameters are apply_params.py's presets.
  ocs_cli speed cruise|circle|gate|all K | reset   v0.1.94.3: the live speed knobs x0.5-2.0, any state, at once (cap x k, max 1.5 m/s)
  ocs_cli tune [KEY VALUE | save]   v0.1.49: a mission-manager knob (sway_v, line_v, ... `tune` lists them) in MANUAL / AUTO_HOLD;
                               save = a profile copy logs/launch/<stamp>/usv_tuned.yaml on the boat (MM_CONFIG_OVERRIDE loads it)

Options (--wait, --host, --port, --vehicle-id, --vehicle) may go before or after the sub-command.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
import threading
import time
import uuid
from typing import List, Optional, Tuple

import paho.mqtt.client as mqtt

from .. import TEAM_NS
from ..companion.state import utc_iso
from . import playbook

ALIASES = {"auto": "enable_auto", "task": "start_task", "confirm": "confirm_safe", "end": "end_run", "stop": "abort",
           "hold_on": "hold", "ls": "next"}


class OcsConsole:
    def __init__(self, host="127.0.0.1", port=1882, ns=TEAM_NS, vehicle_id="USV1", vehicle="usv", operator="cli"):
        self.host, self.port, self.ns, self.vehicle_id, self.vehicle, self.operator = host, port, ns, vehicle_id, vehicle, operator
        self.results: List[dict] = []
        self.acks: List[dict] = []
        self.state: Optional[dict] = None
        self.status: Optional[dict] = None
        self._evt = threading.Event()
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"ocs-cli-{uuid.uuid4().hex[:6]}", protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message
        self._connected = threading.Event()
        self.c.on_disconnect = lambda *a, **k: self._connected.clear()
        self.c.reconnect_delay_set(1, 2)

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        client.subscribe("ocs/intent_result", 1)
        client.subscribe("ocs/state", 0)
        client.subscribe(f"{self.ns}/ack/#", 1)
        client.subscribe(f"{self.ns}/status/{self.vehicle}", 0)
        self._connected.set()

    def _on_message(self, client, userdata, msg):
        try:
            env = json.loads(msg.payload)
        except Exception:
            return
        if msg.topic == "ocs/intent_result":
            b = env["body"]
            if isinstance(b, dict):
                b["_rx"] = time.monotonic()                  # v0.1.86.9: the press timing (when the OCS's verdict arrived)
            self.results.append(b)
            del self.results[:-500]
        elif msg.topic == "ocs/state":
            self.state = env["body"]
        elif msg.topic.endswith(f"/status/{self.vehicle}"):
            self.status = env["body"]
        else:
            if isinstance(env, dict):
                env["_rx"] = time.monotonic()
            self.acks.append(env)
            del self.acks[:-1000]                            # v0.1.86.9: a day of presses, bounded
        self._evt.set()

    def start(self):
        self.c.connect(self.host, self.port, keepalive=15)
        self.c.loop_start()
        time.sleep(0.3)

    def stop(self):
        self.c.loop_stop()
        self.c.disconnect()

    def connected(self, timeout_s: float = 10.0) -> bool:
        return self._connected.wait(timeout_s)

    # v0.1.86.9 (the venue analysis B1): `until` = "done" (the command line: wait for DONE / FAILED / REJECTED, as before) or
    # "accepted" (the pages' /cmd: return at the vehicle's first answer, ACCEPTED or REJECTED - ~100-200 ms; the final DONE / FAILED
    # is followed by the caller, see ocs_web.Web.command). The class default lets ocs_web switch every single-intent word at once
    # while the compound words (ready, manual, stop_reset) pass until="done" for their steps.
    default_until = "done"
    FINAL = ("DONE", "FAILED", "REJECTED")

    def intent(self, action: str, task: Optional[str] = None, args: Optional[dict] = None, wait_s: float = 3.0,
               until: Optional[str] = None) -> dict:
        until = until or self.default_until
        stop_on = self.FINAL + (("ACCEPTED",) if until == "accepted" else ())
        intent_id = str(uuid.uuid4())
        env = {"schema_version": 1, "msg_id": intent_id, "seq": 1, "sent_at": utc_iso(), "run_id": None, "declaration_seq": None,
               "vehicle_id": self.vehicle_id, "kind": "intent", "source": "ocs_cli",
               "body": {"action": action, "task": task, "args": args or {}, "operator": self.operator}}
        if not self._connected.wait(10.0):
            raise RuntimeError("ocs_cli: broker not reachable")
        t_send = time.monotonic()
        self.c.publish(f"{self.ns}/ui/cmd/{self.vehicle}", json.dumps(env), qos=1).wait_for_publish(2.0)
        t0 = time.monotonic()
        res = None
        acks: List[dict] = []
        while time.monotonic() - t0 < wait_s:
            self._evt.wait(0.1)
            self._evt.clear()
            res = res or next((r for r in self.results if r.get("intent_id") == intent_id), None)
            acks = [a for a in self.acks if a.get("intent_id") == intent_id]
            if res and (res["result"] == "refused" or not res.get("cmd_msg_id") or any(a["body"]["result"] in stop_on for a in acks)):
                break
        return {"intent_id": intent_id, "result": res, "acks": acks, "t_send": t_send, "until": until,
                "final": bool(res and (res["result"] == "refused" or not res.get("cmd_msg_id")
                                       or any(a["body"]["result"] in self.FINAL for a in acks)))}

    def follow(self, intent_id: str, wait_s: float) -> List[dict]:
        """v0.1.86.9: after an until="accepted" return, wait (polling, never the shared event) for the vehicle's final ack."""
        t0 = time.monotonic()
        while True:
            acks = [a for a in list(self.acks) if a.get("intent_id") == intent_id]
            if any(a["body"]["result"] in self.FINAL for a in acks) or time.monotonic() - t0 >= wait_s:
                return acks
            time.sleep(0.05)

    def wait_state(self, timeout_s: float = 3.0) -> Optional[dict]:
        t0 = time.monotonic()
        while self.state is None and time.monotonic() - t0 < timeout_s:
            time.sleep(0.1)
        return self.state

    def wait_for(self, pred, timeout_s: float) -> bool:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if self.state and pred(self.state):
                return True
            time.sleep(0.2)
        return False

    def vehicle_state(self) -> Optional[str]:
        """The vehicle's V state: status/<v> (2 Hz) when seen, else the 1 Hz copy inside ocs/state."""
        if self.status and self.status.get("state"):
            return self.status["state"]
        return ((self.state or {}).get("vehicles") or {}).get(self.vehicle_id, {}).get("state")


# ---- parser ---------------------------------------------------------------------------------------------------------
def _opts(with_defaults: bool) -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(add_help=False)
    d = (lambda v: v) if with_defaults else (lambda v: argparse.SUPPRESS)
    p.add_argument("--host", default=d("127.0.0.1"))
    p.add_argument("--port", type=int, default=d(1882))
    p.add_argument("--vehicle-id", default=d("USV1"))
    p.add_argument("--vehicle", default=d("usv"))
    p.add_argument("--wait", type=float, default=d(3.0), help="seconds to wait for the verdict and acks")
    p.add_argument("--dry-run", action="store_true", default=d(False), help="playbook check only, send nothing")
    return p


MOD_KEYS = ("start_mode", "look_lock", "scout_entry_s", "circle_exit_tangent", "squeeze", "lucky_pairs")     # v0.1.92: = cmd_authority.MOD_ARGS


def _mods_args(p) -> None:
    """v0.1.92 (COWORK_HANDOFF addendum 6 §6.3): the mission modifiers - m1t1 / m1t2 START_TASK args; not given = the boat's profile."""
    p.add_argument("--start-mode", dest="start_mode", choices=["advance360", "straight"],
                   help="v0.1.92: m1t1 / m1t2 start - advance360 (2 m ahead + a full turn on the spot, the default) | straight (the search at once)")
    p.add_argument("--look-lock", dest="look_lock", choices=["on", "off"], help="v0.1.92: m1t1 / m1t2 - stop, bow on the suspect before approaching it (default on)")
    p.add_argument("--scout-entry-s", dest="scout_entry_s", type=int, metavar="S", help="v0.1.92: m1t1 / m1t2 - the entry search budget, s (420 default | 300 for a 15-min slot)")
    p.add_argument("--exit-tangent", dest="circle_exit_tangent", choices=["on", "off"],
                   help="v0.1.92: m1t1 / m1t2 - leave the entry circle on the tangent to gate 1's run-in (default off)")
    p.add_argument("--squeeze", dest="squeeze", choices=["on", "off"],
                   help="v0.1.93.1: m1t1 / m1t2 - pass an unknown object near a gate on the wider gap beside it at 0.08 m/s (default off)")
    p.add_argument("--lucky-pairs", dest="lucky_pairs", choices=["one_known", "any"],
                   help="v0.1.93.1: with --lucky - one_known (a known red / green + an uncertain one, the default) | any (two unknowns too)")


def _orbit_args(p) -> None:
    """v0.1.30 (WP8c) m0t6: the orbit's centre and shape."""
    p.add_argument("--centre", "--center", dest="centre", help="m0t6: the saved point to orbit (or --lat / --lon)")
    p.add_argument("--lat", type=float, help="m0t6: the centre's latitude (the map's click)")
    p.add_argument("--lon", type=float, help="m0t6: the centre's longitude")
    p.add_argument("--radius", type=float, help="m0t6: radius, m (default orbit.r_m 3)")
    p.add_argument("--dir", choices=["cw", "ccw"], help="m0t6: direction (default cw)")
    p.add_argument("--turns", type=float, help="m0t6: full circles (default 1)")
    p.add_argument("--speed", type=float, help="m0t6: speed over ground, m/s (default 0.5)")
    p.add_argument("--budget", type=float, help="scout (v0.1.79): the time budget, s (default scout.budget_s 600), then home")
    p.add_argument("--pair", choices=("side", "aft", "both"), help="m0t10 (v0.1.85): the thruster pair of the yaw couple test")
    p.add_argument("--amp", type=float, help="m0t10: the frame amplitude (0..1, default 0.6)")
    p.add_argument("--duration", type=float, help="m0t8 / m0t9 (v0.1.79 / .80): how long the ground test watches, s (default 600)")


def split_names(text: Optional[str]) -> List[str]:
    """v0.1.30 (pool report F2): `--names "1 2 3"` reached the boat as one name - commas and spaces both separate."""
    return [n for n in re.split(r"[,\s]+", text or "") if n]


def check_names(names: List[str], known: Optional[set]) -> Optional[str]:
    """The names a mission would use against the saved points the OCS sees (the map digest's `points`); None = fine or unknown."""
    if not names or known is None:
        return None
    missing = [n for n in names if n not in known]
    if missing:
        return f"point{'s' if len(missing) > 1 else ''} {', '.join(missing)} not saved on the boat (saved: {', '.join(sorted(known)) or 'none'}) - save_point first"
    return None


def parse_points(text: str) -> List[List[float]]:
    """v0.1.53: "LAT,LON;LAT,LON;..." (or spaces between the pairs) -> [[lat, lon], ...]; ValueError on junk."""
    out = []
    for pair in [x for x in re.split(r"[;\s]+", text or "") if x]:
        a, b = pair.split(",")
        out.append([float(a), float(b)])
    return out


def build_parser() -> argparse.ArgumentParser:
    root = _opts(True)
    ap = argparse.ArgumentParser(prog="ocs_cli", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter, parents=[root])
    sub = ap.add_subparsers(dest="cmd", required=True)
    po = [_opts(False)]
    for name in ("help", "next", "state", "watch", "tui", "ready", "manual", "missions"):
        sub.add_parser(name, parents=po)
    p = sub.add_parser("declare", parents=po)            # v0.1.64: declare | declare uav on|off
    p.add_argument("uav", nargs="?", choices=("uav",), help="declare uav on|off: the UAV in the next declaration (off by default; m1t2 needs on)")
    p.add_argument("uav_state", nargs="?", choices=("on", "off"))
    p = sub.add_parser("uav", parents=po)                  # v0.1.64 (WP14): the UAV's words
    p.add_argument("op", choices=("ready", "task", "resend_course", "scan"), help="ready on|off · task N|none · resend_course · scan done")
    p.add_argument("value", nargs="?", help="on|off for ready; none|1|2|3|4 for task")
    p = sub.add_parser("usv", parents=po)                  # v0.1.82: the USV's PILOT READY (an OCS gate: declare refuses the USV without it)
    p.add_argument("op", choices=("ready",), help="ready on|off - the boat's crew says it is ready (the OCS's gate before DECLARE)")
    p.add_argument("value", choices=("on", "off"))
    sub.add_parser("enable_auto", parents=po, aliases=["auto"])
    sub.add_parser("disable_auto", parents=po)
    sub.add_parser("hold", parents=po)
    sub.add_parser("resume", parents=po)
    sub.add_parser("abort", parents=po, aliases=["stop"])
    sub.add_parser("confirm_safe", parents=po, aliases=["confirm"])
    sub.add_parser("end_run", parents=po, aliases=["end"])
    sub.add_parser("reset", parents=po)
    sub.add_parser("step", parents=po)
    sub.add_parser("skip", parents=po)                    # v0.1.68 (WP17 F4): free run only - end the current stage's behaviour now
    p = sub.add_parser("session", parents=po)             # v0.1.69 (WP25): the dry / wet sessions (OCS-side; nothing moves)
    p.add_argument("op", choices=("start", "stop", "result"))
    p.add_argument("name", nargs="?", help="buoys | dockface | windows | uavmap | cannon | collect")
    p.add_argument("--model", choices=("students", "cvt-v2-l"))
    p.add_argument("--tape", help="dockface: the tape distances, m (3,5,8)")
    p.add_argument("--path", choices=("broker", "udp"), help="uavmap: their broker (the boat's bridge) | JSON to the adapter's 14605")
    p.add_argument("--aim10", action="store_true", help="cannon: the aim on for 10 s (H4.3; bench only, with --confirm aim)")
    p.add_argument("--confirm", help="cannon --aim10: type aim")
    p.add_argument("--label", help="collect: the recording's / the points' name (letters, digits, _ -)")
    sub.add_parser("cancel_auto", parents=po)            # v0.1.43: during the 3-2-1 after RunStart
    sub.add_parser("hold_on_start", parents=po)
    sub.add_parser("stop_reset", parents=po)              # v0.1.54: the STOP & RESET macro (execute -> stop_reset())
    p = sub.add_parser("report", parents=po)              # v0.1.53: START / STOP REPORTING - heartbeats to RoboCommand before a declaration
    p.add_argument("state", choices=("on", "off"))
    p.add_argument("which", nargs="?", choices=("uav", "usv"), help="v0.1.64: report on|off uav - the UAV's heartbeats without declaring it; "
                   "v0.1.82: report on|off usv - the boat's heartbeats (the ▶ HEARTBEAT USV button)")
    p = sub.add_parser("mission", parents=po)
    p.add_argument("task", nargs="?", default=None, help="a mission code: m4t0 | goto | m0t0 | m0t1 (+sim) - or the old name T4 / TEST_AUTO; off; no argument = show the preselection")
    p.add_argument("--auto", action="store_true", help="start it by itself when RunStart is verified (once per run)")
    p.add_argument("--lucky", action="store_true", help="v0.1.90: m1t1 / m1t2 - uncertain-colour pairs inside the course as gates (a risk)")
    _mods_args(p)
    p.add_argument("--t4", choices=["off", "core", "advanced", "on"], help="v0.1.71: T4 as a tier - the declaration's T4: off = TIER_NONE, core = TIER_CORE (the default), advanced = TIER_ADVANCED (on = core, the v0.1.44 word)")
    p.add_argument("--planner", choices=["simple", "s6"], help="m1t1: the gate planner for this run (default: t1.planner of the usv profile)")
    p.add_argument("--names", help="m0t2 / m0t3: comma-separated saved point names (usv_ui points.json), e.g. EN,G1,G2,EX")
    p.add_argument("--bay", type=int, help="m3t1: dock in this bay (1..3 left to right facing the dock) instead of the GREEN one")
    p.add_argument("--step", action="store_true", help="stage stepping: the vehicle parks in AUTO_SAFE_HOLD at every stage boundary; `resume` enters the next stage")
    p.add_argument("--from", dest="from_stage", metavar="STAGE", help="start at an entry stage (m3t1: TO_LINE SWAY_TO_ABEAM TURN_IN ENTER AIM_FIRE UNDOCK; m1t1: SEARCH_ENTRY TRANSIT EXIT; m0t4: OBSERVE ALIGN APPROACH)")
    p.add_argument("--target-colour", choices=["red", "green", "blue"], help="m0t4: the face colour to approach (default: the GREEN rule, the vehicle picks)")
    p.add_argument("--scan-x", type=float, default=None, help="m0t4: scan point north, m")
    p.add_argument("--scan-y", type=float, default=None, help="m0t4: scan point east, m")
    p.add_argument("--expected-boxes", type=int, default=None, help="m0t4: faces to survey (default 3)")
    p.add_argument("--scan-hdg", type=float, default=None, help="m0t4: the search looks this way, deg (default: the heading on arrival)")
    p.add_argument("--strafe-side", choices=["starboard", "port"], help="m0t4: the search sways this way (default box_inspect.strafe_side)")
    _orbit_args(p)
    p = sub.add_parser("start_task", parents=po, aliases=["task"])
    p.add_argument("task", nargs="?", default=None, help="a mission code: m4t0 m1t1 (built), goto (bench), m0t0 m0t1 m0t2 (test), m1t2 m3t1 m0t3 (not built yet); +sim = fake FCU only; default = the preselected mission, else m4t0")
    p.add_argument("--lucky", action="store_true", help="v0.1.90: m1t1 / m1t2 - uncertain-colour pairs as gates")
    _mods_args(p)
    p.add_argument("--planner", choices=["simple", "s6"], help="m1t1: the gate planner for this run")
    p.add_argument("--names", help="m0t2 / m0t3: comma-separated saved point names")
    p.add_argument("--bay", type=int, help="m3t1: this bay instead of the GREEN one")
    p.add_argument("--step", action="store_true", help="park in AUTO_SAFE_HOLD at every stage boundary; `resume` goes on")
    p.add_argument("--from", dest="from_stage", metavar="STAGE", help="start at an entry stage (see `ocs_cli mission --help`)")
    p.add_argument("--target-colour", choices=["red", "green", "blue"], help="m0t4: the face colour to approach (default: the GREEN rule, the vehicle picks)")
    p.add_argument("--scan-x", type=float, default=None, help="m0t4: scan point north, m")
    p.add_argument("--scan-y", type=float, default=None, help="m0t4: scan point east, m")
    p.add_argument("--expected-boxes", type=int, default=None, help="m0t4: faces to survey (default 3)")
    p.add_argument("--scan-hdg", type=float, default=None, help="m0t4: the search looks this way, deg (default: the heading on arrival)")
    p.add_argument("--strafe-side", choices=["starboard", "port"], help="m0t4: the search sways this way (default box_inspect.strafe_side)")
    _orbit_args(p)
    p = sub.add_parser("select_target", parents=po)
    p.add_argument("colour", choices=["red", "green", "blue"], help="m0t4 at ASK_OCS: the face to approach (an override of the GREEN rule)")
    p = sub.add_parser("save_point", parents=po)          # v0.1.29 (WP8b): m0t2's points from the OCS
    p.add_argument("name", help="1-16 letters / digits / _ -; overwrites a point of that name")
    p.add_argument("--lat", type=float, help="with --lon: this position (the map's drop point); none = the boat's own fix")
    p.add_argument("--lon", type=float)
    p = sub.add_parser("del_point", parents=po)
    p.add_argument("name")
    sub.add_parser("clear_map", parents=po)              # v0.1.29.1: the boat forgets its detections
    p = sub.add_parser("tune", parents=po)                # v0.1.49 (WP10k-b): the mission manager's runtime knobs
    p.add_argument("key", nargs="?", default=None, help="a knob (companion/tuning: line_v sway_v enter_v standoff_m stop_m lead_step_m look_s "
                                                        "transit_v circle_v la_m scan_rate_dps), or save; none = list them")
    p.add_argument("value", nargs="?", default=None, help="the new value (within the knob's range)")
    p = sub.add_parser("speed", parents=po)               # v0.1.94.3 (Dmitrii 14 Nov): the live speed knobs, any state
    p.add_argument("knob", help="cruise | circle | gate | all | reset")
    p.add_argument("k", nargs="?", default=None, help="the multiplier, 0.5-2.0 (steps of 0.25 on the console)")
    p = sub.add_parser("fcu_mode", parents=po)            # v0.1.50: the FCU's mode from a page (Dmitrii 2026-11-06)
    p.add_argument("mode", help="MANUAL | HOLD | GUIDED | LOITER | AUTO | ... (an ArduPilot Rover mode name)")
    sub.add_parser("arm", parents=po)                     # v0.1.50: ARM / DISARM from a page; the FCU's pre-arm checks decide
    sub.add_parser("disarm", parents=po)
    p = sub.add_parser("station_keep", parents=po)       # v0.1.81 (WP37): the autonomous station keep - hold (neutral) | loiter (position hold)
    p.add_argument("mode", choices=("hold", "loiter"))
    p = sub.add_parser("rc6_policy", parents=po)          # v0.1.50 (not `rc6`: op-tui's plant word): latched = a later page command wins over a high RC6; hold = no
    p.add_argument("policy", choices=("latched", "hold"))
    p = sub.add_parser("fence", parents=po)               # v0.1.53: the USV's FCU geofence (Dmitrii 2026-11-06)
    p.add_argument("what", choices=("apply", "on", "off"), help="apply = upload + read back; on / off = MAV_CMD_DO_FENCE_ENABLE")
    p.add_argument("source", nargs="?", choices=("course", "geofence"), help="apply: the received course or the OCS profile's geofence")
    p.add_argument("--points", help="apply geofence: these corners instead of the profile's (the map sends what it shows), LAT,LON;LAT,LON;...")
    p = sub.add_parser("home", parents=po)                # v0.1.30: the go-home override
    p.add_argument("--speed", type=float, help="m/s (default home.speed_mps 0.3)")
    p = sub.add_parser("goto", parents=po)
    p.add_argument("--x", type=float); p.add_argument("--y", type=float)
    p.add_argument("--lat", type=float); p.add_argument("--lon", type=float); p.add_argument("--speed", type=float)
    p = sub.add_parser("inject", parents=po)
    p.add_argument("what", choices=["run_start", "course", "assistance", "keep_out", "all_clear", "moving", "confirm"])
    p.add_argument("--lat", type=float); p.add_argument("--lon", type=float); p.add_argument("--cseq", type=int)
    return ap


def parse(argv) -> argparse.Namespace:
    ns = build_parser().parse_args(argv)
    ns.cmd = ALIASES.get(ns.cmd, ns.cmd)
    return ns


# ---- execution ------------------------------------------------------------------------------------------------------
def intent_spec(a: argparse.Namespace) -> Tuple[str, Optional[str], dict, Optional[str]]:
    """(action, task, args, inject-what) for the intent sub-commands."""
    action, task, args, what = a.cmd.upper(), None, {}, None
    if a.cmd == "report":
        args = {"on": a.state == "on"}
        if getattr(a, "which", None):
            args["vehicle"] = a.which
    if a.cmd == "declare" and getattr(a, "uav", None):   # v0.1.64: declare uav on|off
        if getattr(a, "uav_state", None) is None:
            raise SystemExit("declare uav on|off")
        return "DECLARE_UAV", None, {"on": a.uav_state == "on"}, None
    if a.cmd == "usv":                                     # v0.1.82: usv ready on|off
        return "USV_READY", None, {"on": a.value == "on"}, None
    if a.cmd == "uav":
        args = {"op": a.op}
        if a.op == "ready":
            if a.value not in ("on", "off"):
                raise SystemExit("uav ready on|off")
            args["on"] = a.value == "on"
        elif a.op == "task":
            args["task"] = (a.value or "none").lower()
        elif a.op == "scan":                               # v0.1.66: `uav scan done` (m1t2: leave WAIT_UAV_MAP)
            if a.value != "done":
                raise SystemExit("uav scan done")
            args = {"op": "scan_done"}
        return "UAV", None, args, None
    if a.cmd == "session":                                # v0.1.69 (WP25)
        if a.op == "start" and not a.name:
            raise SystemExit("session start NAME (buoys | dockface | windows | uavmap | cannon | collect)")
        opts = {k: getattr(a, k) for k in ("model", "tape", "path", "confirm", "label") if getattr(a, k, None)}
        if getattr(a, "aim10", False):
            opts["aim10"] = "yes"
        return "SESSION", None, {"op": a.op, "name": a.name, "opts": opts}, None
    if a.cmd == "abort":
        action = "ABORT_TASK"
    elif a.cmd == "reset":
        action = "RESET_RUN"
    elif a.cmd == "mission":
        action = "MISSION"
        args = {"task": (a.task or "").upper(), "auto": bool(getattr(a, "auto", False))}
        if getattr(a, "t4", None):                     # v0.1.44: the T4 modifier ("on" / "off" - a string, so off survives the filter)
            args["t4"] = a.t4
        rr = playbook.resolve(a.task) if a.task and a.task.lower() not in ("off", "none") else None
        if rr:
            args["task"] = rr[1]
            if rr[2]:
                args["tier"] = rr[2]
            if rr[3]:
                args["sim"] = True
    elif a.cmd == "start_task":
        task = (a.task or "").upper() or None          # None = the OCS uses its preselected mission (else m4t0 / T4)
        rr = playbook.resolve(a.task) if a.task else None
        if rr:
            task = rr[1]
            if rr[2]:
                args["tier"] = rr[2]
            if rr[3]:
                args["sim"] = True
    if a.cmd in ("mission", "start_task"):
        if getattr(a, "lucky", False):                  # v0.1.90.2
            args["lucky"] = True
        for k in MOD_KEYS:                              # v0.1.92: the mission modifiers (cmd_authority normalises them)
            if getattr(a, k, None) is not None:
                args[k] = getattr(a, k)
        if getattr(a, "planner", None):
            args["planner"] = a.planner
        if getattr(a, "names", None):
            args["names"] = split_names(a.names)
        if getattr(a, "centre", None):                  # v0.1.30 m0t6: the centre is a saved point = names[0]
            args["names"] = [a.centre]
        if getattr(a, "lat", None) is not None and getattr(a, "lon", None) is not None:
            args["lat"], args["lon"] = a.lat, a.lon
        for flag, key in (("radius", "r_m"), ("dir", "direction"), ("turns", "turns"), ("speed", "speed_mps"), ("budget", "budget_s"),
                          ("duration", "duration_s")):
            if getattr(a, flag, None) is not None:
                args[key] = getattr(a, flag)
        if getattr(a, "bay", None):
            args["bay"] = a.bay
        if getattr(a, "pair", None):                    # v0.1.85: m0t10
            args["pair"] = a.pair
        if getattr(a, "amp", None) is not None:
            args["amp"] = a.amp
        if getattr(a, "step", False):
            args["step"] = True
        if getattr(a, "from_stage", None):
            args["from"] = a.from_stage.upper()
        if getattr(a, "target_colour", None):
            args["target_colour"] = a.target_colour
        if getattr(a, "scan_x", None) is not None:
            args["scan_x_n"] = a.scan_x
        if getattr(a, "scan_y", None) is not None:
            args["scan_y_e"] = a.scan_y
        if getattr(a, "expected_boxes", None) is not None:
            args["expected_boxes"] = a.expected_boxes
        if getattr(a, "scan_hdg", None) is not None:
            args["scan_hdg_deg"] = a.scan_hdg
        if getattr(a, "strafe_side", None):
            args["strafe_side"] = a.strafe_side
    if a.cmd == "select_target":
        action = "SELECT_TARGET"
        args = {"target_colour": a.colour}
    elif a.cmd == "clear_map":
        action = "CLEAR_MAP"
    elif a.cmd == "tune":
        action = "TUNE"
        if (a.key or "").lower() == "save":
            args, what = {"save": True}, "save"
        else:
            args = {"key": a.key, "value": a.value}
    elif a.cmd == "speed":
        action = "SPEED"
        args = {"reset": True} if a.knob.lower() == "reset" else {a.knob.lower(): a.k}
    elif a.cmd == "fcu_mode":
        action, args = "FCU_MODE", {"mode": a.mode.upper()}
    elif a.cmd in ("arm", "disarm"):
        action, args = "FCU_ARM", {"on": a.cmd == "arm"}
    elif a.cmd == "rc6_policy":
        action, args = "RC6", {"policy": a.policy}
    elif a.cmd == "station_keep":
        action, args = "STATION_KEEP", {"mode": a.mode}
    elif a.cmd == "fence":                              # v0.1.53
        if a.what == "apply":
            action, args = "FENCE_APPLY", {"source": a.source or ""}
            if a.points:
                args["points"] = parse_points(a.points)
        else:
            action, args = "FENCE_ENABLE", {"on": a.what == "on"}
    elif a.cmd == "home":
        action = "HOME"
        if a.speed is not None:
            args = {"speed_mps": a.speed}
    elif a.cmd in ("save_point", "del_point"):
        action = "SAVE_POINT" if a.cmd == "save_point" else "DELETE_POINT"
        args = {"name": a.name}
        if a.cmd == "save_point" and a.lat is not None and a.lon is not None:
            args.update(lat=a.lat, lon=a.lon)
    elif a.cmd == "goto":
        if a.lat is not None:
            args = {"lat": a.lat, "lon": a.lon}
        else:
            args = {"x_n": a.x if a.x is not None else 10.0, "y_e": a.y if a.y is not None else 0.0}
        if a.speed:
            args["speed_mps"] = a.speed
    elif a.cmd == "inject":
        wmap = {"assistance": "assistance_request", "keep_out": "keep_out_zone", "moving": "moving_object", "confirm": "readiness_confirm"}
        what = wmap.get(a.what, a.what)
        args = {"what": what}
        if a.lat is not None:
            args.update(lat=a.lat, lon=a.lon if a.lon is not None else 103.85501)
        if a.cseq is not None:
            args["command_seq"] = a.cseq
    return action, task, args, what


def send_intent(con: OcsConsole, action: str, task: Optional[str], args: dict, wait_s: float, what: Optional[str] = None,
                dry_run: bool = False, until: Optional[str] = None) -> Tuple[int, List[str]]:
    """Playbook check, then the real intent; returns (exit code, lines to print)."""
    lines: List[str] = []
    con.wait_state(2.0)
    chk_task = (task + "sim") if (task and args.get("sim")) else task      # the playbook predicts the sim guard too
    v = playbook.check(action, con.state, con.vehicle_id, task=chk_task, what=what, status=con.status)
    if not v.ok and action == "MISSION":     # v0.1.29: the OCS's own preselect, no playbook entry: no false alarm
        v = playbook.Verdict(action, True, sent_as=action)
    if not v.ok:
        lines.append(f"playbook: {action} would be refused: {v.why}" + (f"  ->  {v.hint}" if v.hint else ""))
        if dry_run:
            return 1, lines
        lines.append("sending anyway so the real verdict is shown")
    elif dry_run:
        lines.append(f"playbook: {action} looks allowed now (sent as {v.sent_as})")
        return 0, lines
    out = con.intent(action, task, args, wait_s=wait_s, until=until)
    con.last_out = dict(out, action=action, wait_s=wait_s)              # v0.1.86.9: ocs_web follows the final ack from here
    r = out["result"]
    if r is None:
        lines.append("no intent result (is ocs_app running on this broker?)")
        return 1, lines
    lines.append(f"intent {r['result']:8s} {r.get('reason', '')}".rstrip())
    last = "accepted" if r["result"] == "accepted" else "refused"
    for ack in out["acks"]:
        b = ack["body"]
        lines.append(f"  ack {b['result']:9s} state={b.get('state')} {b.get('reason', '')}".rstrip())
        last = b["result"]
    if r["result"] == "accepted" and r.get("cmd_msg_id") and out["acks"] and not out.get("final"):
        lines.append("  (accepted by the vehicle - its final answer follows)")
    if r["result"] == "accepted" and r.get("cmd_msg_id") and not out["acks"]:
        lines.append("  (no vehicle ack within --wait: is the mission manager up and bridged? try --wait 6)")
        last = "timeout"
    if last in ("refused", "REJECTED", "FAILED"):
        h = playbook.check(action, con.state, con.vehicle_id, task=chk_task, what=what, status=con.status)
        if h.hint:
            lines.append(f"  hint: {h.hint}")
        lines.append(f"  next: {playbook.suggest(con.state, con.vehicle_id)}")
        return 1, lines
    return 0, lines


def state_line(st: Optional[dict], vehicle_id: str) -> str:
    if not st:
        return "no ocs/state"
    veh = (st.get("vehicles") or {}).get(vehicle_id) or {}
    return (f"{st.get('r')} {st.get('r_state'):12s} run={st.get('run_id')} d={st.get('declaration_seq')} "
            f"stalled={'yes' if st.get('stalled') else 'no'} | {vehicle_id} {veh.get('state')} {veh.get('current_task')} "
            f"{veh.get('task_state') or ''} age={veh.get('age_s')} | judge {'up' if st.get('judge_connected') else 'DOWN'} "
            f"rate {st.get('msg_rate')}/5 hb {sum((st.get('heartbeats_sent') or {}).values())}")


def execute(con: OcsConsole, a: argparse.Namespace) -> Tuple[int, List[str]]:
    wait_s = getattr(a, "wait", 3.0)
    dry = getattr(a, "dry_run", False)
    if a.cmd == "help":
        return 0, playbook.help_lines()
    if a.cmd == "state":
        st = con.wait_state(wait_s)
        return (0, [json.dumps(st, indent=1)]) if st else (1, ["no ocs/state received (is ocs_app running?)"])
    if a.cmd == "next":
        st = con.wait_state(wait_s)
        if not st:
            return 1, ["no ocs/state received (is ocs_app running?)"]
        lines = [state_line(st, con.vehicle_id), ""]
        for v in playbook.allowed_now(st, con.vehicle_id, con.status):
            e = playbook.PLAYBOOK[v.action]
            mark = "ok " if v.ok else "-- "
            lines.append(f"{mark}{e.cli:16s} {'' if v.ok else v.why}" + (f"  ->  {v.hint}" if (not v.ok and v.hint) else ""))
        lines += ["", f"suggested next: {playbook.suggest(st, con.vehicle_id)}"]
        return 0, lines
    if a.cmd == "watch":
        try:
            while True:
                print(time.strftime("%H:%M:%S"), state_line(con.state, con.vehicle_id), flush=True)
                time.sleep(1.0)
        except KeyboardInterrupt:
            return 0, []
    if a.cmd == "ready":
        lines: List[str] = []
        con.wait_state(wait_s)
        if con.state and con.state.get("r") in ("R2", "R6") or (con.state and con.state.get("r") == "R4" and con.state.get("stalled")):
            c, l = send_intent(con, "DECLARE", None, {}, wait_s, until="done"); lines += ["> declare"] + l
            if c:
                return c, lines
            con.wait_for(lambda s: s.get("r") in ("R3", "R4", "R5"), 3)
        if con.vehicle_state() == "MANUAL":
            c, l = send_intent(con, "ENABLE_AUTO", None, {}, max(wait_s, 5.0), until="done"); lines += ["> enable_auto"] + l
            if c:
                return c, lines
        ok = con.wait_for(lambda s: s.get("r") == "R5", 15.0)
        # v0.1.59 (B3): on the judge wire R4 after 15 s is the normal wait for the course manager (minutes), not a failure
        waiting = (not ok and (con.state or {}).get("r") == "R4" and (con.state or {}).get("judge") == "protobuf"
                   and not (con.state or {}).get("stalled"))
        if ok:
            lines.append("R5 RUNNING: run " + str(con.state.get("run_id")))
        elif waiting:
            lines.append(f"R4 READY: declared seq {(con.state or {}).get('declaration_seq')}, waiting for the course manager's RunStart "
                         "(it may take minutes - RUN SEQUENCE step 4 counts)")
        else:
            lines.append("no RunStart within 15 s (judge refusing? inject run_start on the bench)")
        lines.append("next: " + playbook.suggest(con.state, con.vehicle_id))
        return (0 if ok or waiting else 1), lines
    if a.cmd == "missions":
        lines = ["code   wire task    tier      status   title"]
        for c in playbook.MISSION_ORDER:
            m = playbook.MISSIONS[c]
            lines.append(f"{c:6s} {m.task:12s} {m.tier:9s} {m.status:8s} {m.title}")
        lines.append("+sim (m1t1sim): the same mission, rejected by the vehicle unless usv_ui holds the fake FCU; old names (T4, TEST_AUTO) still work")
        return 0, lines
    if a.cmd == "mission" and not a.task:
        st = con.wait_state(wait_s)
        mi = (st or {}).get("mission") or {}
        code = mi.get("code") or playbook.code_of(mi.get("task"), mi.get("tier"), bool(mi.get("sim")))
        return 0, [f"mission: {code or 'none preselected'}" + (f"  ({mi.get('task')})" if mi.get("task") else "") +
                   ("  (auto-start on RunStart)" if mi.get("auto") else "") +
                   "   -> ocs_cli mission m4t0|goto|m0t0|m0t1 [--auto] | off   (ocs_cli missions lists the codes)"]
    if a.cmd == "tune" and (not a.key or (a.key.lower() != "save" and a.value is None)):
        from ..companion import tuning
        lines = ["knob           default  range          sets       what"] + [
            f"{k:14s} {d:<8g} {lo:g} .. {hi:g} {u:5s} {'/'.join(sc):10s} {tip}" for k, (sc, d, lo, hi, u, tip) in tuning.KNOBS.items()]
        lines += ["ocs_cli tune KEY VALUE (MANUAL / AUTO_HOLD only) · ocs_cli tune save · " + tuning.RO_NOTE + ": PWM limits, stern factors"]
        return (0 if not a.key else 1), lines
    if a.cmd == "manual":
        lines = []
        con.wait_state(wait_s)
        if con.vehicle_state() in ("AUTO_TASK", "AUTO_SAFE_HOLD"):
            c, l = send_intent(con, "ABORT_TASK", None, {}, wait_s, until="done"); lines += ["> abort"] + l
            if c:
                return c, lines
            t0 = time.monotonic()
            while con.vehicle_state() != "AUTO_HOLD" and time.monotonic() - t0 < 3.0:
                time.sleep(0.1)
        c, l = send_intent(con, "DISABLE_AUTO", None, {}, wait_s, until="done"); lines += ["> disable_auto"] + l
        return c, lines
    if a.cmd == "stop_reset":                           # v0.1.54
        return stop_reset(con, wait_s)
    if a.cmd == "fence" and a.what == "apply" and not a.source:
        return 1, ["usage: fence apply course|geofence   (fence on | fence off)"]
    try:
        action, task, args, what = intent_spec(a)
    except ValueError as exc:
        return 1, [f"bad --points: {exc} (LAT,LON;LAT,LON;...)"]
    if action in ("FENCE_APPLY", "FENCE_ENABLE"):
        wait_s = max(wait_s, 12.0)                     # v0.1.53: the FCU's answer (upload + read-back) takes a few seconds
    return send_intent(con, action, task, args, wait_s, what=what, dry_run=dry)


# ---- v0.1.54: STOP & RESET -------------------------------------------------------------------------------------------
TASK_STATES = ("AUTO_TASK", "AUTO_SAFE_HOLD")


def _wait_vehicle(con: OcsConsole, want, timeout_s: float) -> Optional[str]:
    t0 = time.monotonic()
    while con.vehicle_state() not in want and time.monotonic() - t0 < timeout_s:
        time.sleep(0.1)
    return con.vehicle_state()


def stop_reset(con: OcsConsole, wait_s: float = 3.0) -> Tuple[int, List[str]]:
    """Dmitrii 2026-11-06: "a better stop button, which will return vehicle in hold - disarmed state ... clear the current mission
    state, stop reporting, and then I could rerun the sequence". The steps (playbook.STOP_RESET_STEPS), in order, each one reported:
    cancel_auto (only in the 3-2-1) -> abort (only when a task runs) -> disable_auto (only from AUTO_HOLD) -> fcu_mode HOLD ->
    disarm -> reset (R2+: vehicles RESET, the OCS back to R2). v0.1.59 (B1): reporting is left as it is - on the judge wire the
    heartbeats go on (the boat is still in the course area; `report off` is its own, confirmed, word). A refused or not-applicable
    step never stops the chain: the FCU commands always go out. Exit 0 when HOLD and DISARM were taken."""
    lines: List[str] = ["STOP & RESET: abort -> disable_auto -> fcu_mode HOLD -> disarm -> reset"]
    res: List[Tuple[str, str]] = []
    con.wait_state(wait_s)

    def step(name: str, action: str, args: dict, w: float) -> int:
        try:
            c, out = send_intent(con, action, None, args, w, until="done")
        except Exception as exc:                        # the broker gone: say so, go on with the next step
            c, out = 1, [f"not sent: {exc}"]
        lines.append(f"> {name}")
        lines.extend(out)
        res.append((name, "ok" if c == 0 else "REFUSED / FAILED"))
        return c

    def skip(name: str, why: str) -> None:
        lines.append(f"> {name}: skipped - {why}")
        res.append((name, "n/a"))

    st = con.state or {}
    if ((st.get("mission") or {}).get("countdown")):
        step("cancel_auto", "CANCEL_AUTO", {}, wait_s)
    else:
        skip("cancel_auto", "no auto-start countdown")
    v = con.vehicle_state()
    if v in TASK_STATES:
        step("abort", "ABORT_TASK", {}, wait_s)
        v = _wait_vehicle(con, ("AUTO_HOLD",), 3.0)
    else:
        skip("abort", f"no task runs (vehicle {v or 'not heard'})")
    if v in ("AUTO_HOLD",) + TASK_STATES:
        step("disable_auto", "DISABLE_AUTO", {}, wait_s)
        v = _wait_vehicle(con, ("MANUAL",), 3.0)
    else:
        skip("disable_auto", f"vehicle {v or 'not heard'}: not in AUTO")
    crit = [step("fcu_mode HOLD", "FCU_MODE", {"mode": "HOLD"}, max(wait_s, 5.0)),
            step("disarm", "FCU_ARM", {"on": False}, max(wait_s, 5.0))]
    r = (con.state or {}).get("r")
    if r in ("R2", "R3", "R4", "R5", "R6"):
        if step("reset", "RESET_RUN", {}, wait_s) == 0:
            con.wait_for(lambda s: s.get("r") in ("R1", "R2"), 3.0)
    else:
        skip("reset", f"nothing to reset in {r or 'an unknown R'} (no course yet)")
    t0 = time.monotonic()                               # the result as the boat reports it (status/<v> is 2 Hz)
    while time.monotonic() - t0 < 2.0 and not (con.status and con.status.get("armed") is False and con.status.get("mode") == "HOLD"):
        time.sleep(0.1)
    s2, o2 = con.status or {}, con.state or {}
    hb = (o2.get("reporting") or {}).get("heartbeats")
    lines.append("steps: " + " · ".join(f"{n} {x}" for n, x in res))
    lines.append(f"now: OCS {o2.get('r') or '?'} {o2.get('r_state') or ''} · vehicle {s2.get('state') or '?'} · FCU {s2.get('mode') or '?'} "
                 f"{'?' if s2.get('armed') is None else ('ARMED' if s2.get('armed') else 'disarmed')} · heartbeats "
                 f"{'?' if hb is None else ('ON' if hb else 'off')}")
    lines.append("next: rerun the sequence - declare, arm, enable_auto")
    return (0 if not any(crit) else 1), lines


def main(argv=None) -> int:
    a = parse(argv)
    if a.cmd == "tui":
        from .bench_tui import main as tui_main
        return tui_main(["--host", a.host, "--port", str(a.port), "--vehicle-id", a.vehicle_id, "--vehicle", a.vehicle])
    if a.cmd == "help":
        print("\n".join(playbook.help_lines()))
        return 0
    con = OcsConsole(a.host, a.port, vehicle_id=a.vehicle_id, vehicle=a.vehicle)
    con.start()
    try:
        if not con.connected(5.0):
            print(f"broker {a.host}:{a.port} not reachable (is the bench up? make bench)")
            return 1
        code, lines = execute(con, a)
        for ln in lines:
            print(ln)
        return code
    finally:
        con.stop()


if __name__ == "__main__":
    sys.exit(main())
