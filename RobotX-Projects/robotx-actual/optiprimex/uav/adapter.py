"""uav_adapter: the UAV as a vehicle on our team bus, speaking the UAV team's §5 wire to their Orin (v0.1.64 WP14, D4-D5, D9-D13 of
docs/OVERNIGHT_2026-11-07_UAV.md).

A separate process (console `op-uav-adapter`, tmux window `uav`), started only when the launcher's UAV field is on; nothing else in the
stack knows whether it runs - ocs_app keys everything on `declare uav` and on this process's status / presence. It is the Orin-facing
half of the UAV team's `robocommand_bridge.py`; the RoboCommand-facing half (the declaration, the heartbeat, the reports, the course and
command forwards) is ocs_app's, the team's one RoboCommand client.

  hub (team bus)                                         UDP (his §5 wire, JSON datagrams)                         the Orin
  status/uav 2 Hz, event/uav/#, ack/uav   <--  adapter  <-- uav_telemetry 1 Hz, uav_report, robocommand_ack  <--  geofence_udp_listener
  cmd/uav/# (PREPARE, START_RUN, RC_FORWARD, ...)  -->      --> robocommand_hello 2 s, robocommand {course|command},     (port 14600)
                                                                fence_rejected, fence_change_result, bridge_notice, uav_report_ack

NOTHING HERE COMMANDS THE DRONE: no MAVLink, no mode, no arm, no mission - only his protocol messages and RoboCommand's own traffic
(MERGE_GUIDE §4 invariant 15; his listener and RunStart flight decide on the Orin, the pilot's RC wins).

The V-state for the UAV (D5): BOOT until the first telemetry; MANUAL when the FCU mode is not AUTO / GUIDED; AUTO_HOLD in AUTO / GUIDED with
no task; AUTO_TASK while a UAV task is current; KILLED never (no kill input we see). robot_state = his telemetry's `state` (STATE_AUTO in
AUTO or GUIDED - his rule). Telemetry older than 5 s: the state keeps its last value, health.telemetry_age_s grows, robot_state
STATE_UNKNOWN, and ocs_app's heartbeat publisher stops (his invariant 3)."""
from __future__ import annotations

import argparse
import dataclasses
import json
import logging
import os
import signal
import threading
import time
from typing import Callable, Dict, List, Optional

from ..companion import config as cfgmod
from ..companion.config import Config
from ..companion.mqtt import Bus
from ..companion.state import utc_iso
from ..ocs.geofence import MAX_CEILING_HAE_M, geofence_problems
from . import wire as W

log = logging.getLogger("optiprimex.uav")

AUTO_MODES = ("AUTO", "GUIDED")
TASK_OF = {"none": "TASK_NONE", "1": "TASK_SAFE_PASSAGE", "2": "TASK_INFRA_SURVEY_REPAIR", "3": "TASK_COORDINATED_LOGISTICS",
           "4": "TASK_DYNAMIC_INCIDENT"}          # his TASKS, in our status words (rc_wire.TASK_TO_RX maps them to the wire)
TASK_OF.update({v: v for v in list(TASK_OF.values())})


def uav_cfg(cfg: Config) -> dict:
    """The `uav:` block of the OCS profile (D6), with the defaults of the brief (D1: values from the profile, never hardcoded)."""
    u = dict((cfg.extra.get("ocs") or {}).get("uav") or {})
    u.setdefault("vehicle_id", "UAV1")
    u.setdefault("registration", "SGIT-53577")
    u.setdefault("pilot_callsign", "Calwin-U2332301")
    u.setdefault("topic", "uav")
    u.setdefault("telemetry_stale_s", W.TELEMETRY_STALE_S)
    u.setdefault("stale_stop_s", W.TELEMETRY_STALE_S)
    u.setdefault("declare_default", False)
    return u


def _ring(points) -> List[List[float]]:
    pts = [[float(p[0]), float(p[1])] for p in (points or [])]
    if pts and pts[0] != pts[-1]:
        pts.append(list(pts[0]))                   # D8: closed ring (close it if the listener does not)
    return pts


def course_rings(course: Optional[dict]):
    """(corners, transit) as (lat, lon) tuples from our course dict (boundary / uav_transit_area) or a forwarded RxCourse dict."""
    c = course or {}
    if "corners" in c:
        corners = [(float(p.get("latitude", 0.0)), float(p.get("longitude", 0.0))) for p in c.get("corners") or []]
        transit = [(float(p.get("latitude", 0.0)), float(p.get("longitude", 0.0))) for p in c.get("uav_transit_area") or []]
    else:
        def ll(p):
            return (float(p["lat"]), float(p["lon"])) if isinstance(p, dict) else (float(p[0]), float(p[1]))
        corners = [ll(p) for p in c.get("boundary") or []]
        transit = [ll(p) for p in c.get("uav_transit_area") or []]
    return corners, transit


def fence_check(fence, ceiling, course) -> List[str]:
    """geofence_problems() (his, verbatim) on the FCU's polygon against the course; [] = RoboCommand would accept it."""
    corners, transit = course_rings(course)
    if not corners:
        return ["no course from RoboCommand to check the geofence against"]
    if ceiling is None:
        return ["no FENCE_ALT_MAX from the drone yet"]
    pts = [(float(a), float(b)) for a, b in _ring(fence)]
    return geofence_problems(pts, corners, transit, float(ceiling), MAX_CEILING_HAE_M)


class UavAdapter:
    def __init__(self, cfg: Config, listener=("127.0.0.1", W.DEFAULT_LISTEN_PORT), bridge_port: int = W.DEFAULT_BRIDGE_PORT,
                 bind_host: str = "0.0.0.0", bus: Optional[Bus] = None, retry_for_s: float = 15.0, clock: Callable[[], float] = time.monotonic,
                 fcu_link: str = "real"):
        self.u = uav_cfg(cfg)
        self.vid, self.tok = self.u["vehicle_id"], self.u["topic"]
        # the OCS profile is read for the hub and the uav block; on the bus this process IS the UAV (envelopes, presence, the LWT)
        self.cfg = cfg = dataclasses.replace(cfg, vehicle_id=self.vid, vehicle=self.tok, node="uav_adapter")
        self.stale_s = float(self.u["telemetry_stale_s"])
        self.clock = clock
        self.fcu_link = fcu_link
        self.bus = bus or Bus(cfg, client_id=f"uav-adapter-{os.getpid()}", source="uav_adapter",
                              lwt_topic=cfg.team("status", self.tok),
                              lwt_body={"state": "OFFLINE", "current_task": "TASK_NONE", "vehicle_type": "UAV"})
        self.sender = W.BoundSender(tuple(listener), retry_for_s, bind_host, bridge_port)
        self.listener = tuple(listener)
        self.lock = threading.RLock()
        # what the Orin told us
        self.tel: Optional[dict] = None
        self.tel_t: Optional[float] = None
        self.tel_n = 0
        self.ever_airborne = False
        self.ever_armed = False
        self.alt_max: Optional[float] = None
        # the run, as ocs_app tells it
        self.pilot_ready = False
        self.task = "TASK_NONE"
        self.task_override: Optional[str] = None
        self.pending_task: Optional[str] = None        # D11: the preselected UAV mission's task, set once the RunStart forward is acked
        self.run_id: Optional[int] = None
        self.runstart_acked_run: Optional[int] = None
        self.state = "BOOT"
        self.ocs: dict = {}                            # ocs/state's `uav` block: declared? the declared fence / ceiling
        self.course: Optional[dict] = None             # the last forwarded RxCourse (dict) + its fingerprint
        self.course_fp: Optional[str] = None
        self.course_msg: Optional[str] = None          # the pending course send (the newest cancels it)
        self.course_acked_t: Optional[float] = None
        self.last_forward: Optional[dict] = None
        self.fence_events: Dict[str, dict] = {}        # fence_change id -> {t, held}
        self.fence_results: Dict[str, dict] = {}       # fence_change id -> the fence_change_result datagram (re-sent while asked)
        self.fence_waiting: set = set()
        self.alt_resend = False
        self.last_fence_result: Optional[dict] = None
        self.fence_rejected_t: Optional[str] = None
        self.seen_reports: List[str] = []
        self.reports_n = 0
        self.last_report: Optional[dict] = None
        self.events: List[dict] = []
        self._stop = threading.Event()
        self._threads: List[threading.Thread] = []

    # ---- start / stop -------------------------------------------------------------------------------------------------
    def start(self) -> None:
        ns = self.cfg.mqtt.ns
        self.bus.subscribe(f"{ns}/cmd/{self.tok}/#", self.on_cmd, qos=1)
        self.bus.subscribe("ocs/state", self.on_ocs_state, qos=0)
        self.bus.start()
        for name, fn in (("uav-wire", self._wire_loop), ("uav-status", self._status_loop)):
            t = threading.Thread(target=fn, name=name, daemon=True)
            t.start()
            self._threads.append(t)
        log.info("uav_adapter: %s (%s) <-> listener %s:%s, bound %s", self.vid, self.tok, self.listener[0], self.listener[1], self.sender.bound)

    def stop(self) -> None:
        self._stop.set()
        for t in self._threads:
            t.join(timeout=1.0)
        try:
            self.bus.publish(self.cfg.team("status", self.tok), "status",
                             {"state": "OFFLINE", "current_task": "TASK_NONE", "vehicle_type": "UAV"}, qos=0, timeout_s=0.5, vehicle_id=self.vid)
        except Exception:
            pass
        self.bus.stop()
        try:
            self.sender.sock.close()
        except OSError:
            pass

    def _wire_loop(self) -> None:
        last_hello = 0.0
        while not self._stop.is_set():
            try:
                self.sender.service(self.on_datagram)
                now = time.time()
                if now - last_hello >= W.HELLO_INTERVAL_S:
                    self.sender.send_once({"type": "robocommand_hello"})
                    last_hello = now
            except Exception:
                log.exception("uav wire")

    def _status_loop(self) -> None:
        while not self._stop.wait(0.5):              # 2 Hz (D2: the last fresh sample with its true age)
            try:
                self.publish_status()
            except Exception:
                log.exception("uav status")

    # ---- the Orin -> us ---------------------------------------------------------------------------------------------------
    def on_datagram(self, d: dict) -> None:
        t = d.get("type")
        if t == "uav_telemetry":
            self.on_telemetry(d)
        elif t == "uav_report":
            self.on_report(d)
        elif t == "uav_buoys":                                 # v0.1.66 (D20 alternative): their buoys as UDP when they run no broker
            self.relay_buoys({k: v for k, v in d.items() if k != "type"})

    # ---- v0.1.66 (D21): the relay path - the UAV's buoys onto the hub (the hub -> boat bridge lands them on local/usv/uav/...) --------
    def relay_buoys(self, msg: dict) -> None:
        if not isinstance(msg, dict) or not isinstance(msg.get("items"), list):
            return
        try:
            self.bus.publish_raw("uav/perception/buoys", json.dumps(msg), qos=0)
            self.buoys_relayed = getattr(self, "buoys_relayed", 0) + 1
        except Exception:
            log.exception("uav buoys relay")

    def start_buoy_relay(self, host: str, port: int) -> None:
        """UAV_BUOY_PATH relay with a broker on the UAV Orin: subscribe its uav/perception/buoys, republish on the hub."""
        import paho.mqtt.client as mqtt
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"uav-buoy-relay-{os.getpid()}")
        c.on_connect = lambda cl, u, fl, rc, pr=None: cl.subscribe("uav/perception/buoys", 0)
        def on_msg(cl, u, m):
            try:
                self.relay_buoys(json.loads(m.payload))
            except ValueError:
                pass
        c.on_message = on_msg
        c.reconnect_delay_set(1, 5)
        c.connect_async(host, int(port), keepalive=15)
        c.loop_start()
        self._relay_client = c
        log.info("uav buoys: relaying %s:%s uav/perception/buoys -> the hub", host, port)

    def on_telemetry(self, d: dict) -> None:
        with self.lock:
            self.tel, self.tel_t = d, self.clock()
            self.tel_n += 1
            if d.get("armed"):
                self.ever_armed = True
            if d.get("flight_phase") == "FLIGHT_PHASE_AIRBORNE":
                self.ever_airborne = True
            if d.get("fence_alt_max") is not None:
                v = float(d["fence_alt_max"])
                if self.alt_max is not None and abs(v - self.alt_max) > 0.01 and self.ocs.get("declared"):
                    self.alt_resend = True                     # D9: a ceiling change is treated as a fence change
                    self._event("FENCE_ALT_MAX %g -> %g m" % (self.alt_max, v))
                self.alt_max = v
            if not d.get("armed") and self.task != "TASK_NONE" and self.task_override is None and self.ever_armed:
                self.task = "TASK_NONE"                        # D11: the UAV mission's task ends when the drone disarms
            fc = d.get("fence_change")
        if fc:
            self.on_fence_change(fc, d)
        elif self.alt_resend:
            self.on_ceiling_change(d)

    def on_fence_change(self, change: dict, t: dict) -> None:
        cid = str(change.get("id"))
        if cid in self.fence_results:                          # the listener did not get the result: again (his rule)
            self.sender.send_once(self.fence_results[cid])
            return
        if t.get("armed"):
            if cid not in self.fence_waiting:
                self.fence_waiting.add(cid)
                self._event("fence changed while ARMED: the re-declaration waits until disarmed")
            return
        if cid in self.fence_events:
            return
        self.fence_events[cid] = {"t": self.clock()}
        self.alt_resend = False                                # this one carries the current FENCE_ALT_MAX
        self.pilot_ready = False                               # D10: a fence change clears the pilot's ready
        self.bus.publish(self.cfg.team("event", self.tok, "fence_change"), "event",
                         {"event": "fence_change", "id": cid, "vertices": _ring(change.get("vertices")), "ceiling_hae_m": self.alt_max},
                         qos=1, timeout_s=0.0, vehicle_id=self.vid)
        self._event(f"fence change {cid}: {len(change.get('vertices') or [])} points -> the OCS")

    def on_ceiling_change(self, t: dict) -> None:
        if t.get("armed"):
            return                                             # waits until disarmed (his rule)
        if len(t.get("fcu_fence") or []) < 3:
            self.alt_resend = False
            return
        self.alt_resend = False
        cid = "alt-%d" % int(time.time() * 1000)               # D9: a synthetic id
        self.pilot_ready = False                               # D10
        self.fence_events[cid] = {"t": self.clock(), "synthetic": True}
        self.bus.publish(self.cfg.team("event", self.tok, "fence_change"), "event",
                         {"event": "fence_change", "id": cid, "vertices": _ring(t.get("fcu_fence")), "ceiling_hae_m": self.alt_max,
                          "synthetic": True}, qos=1, timeout_s=0.0, vehicle_id=self.vid)

    def on_report(self, d: dict) -> None:
        """D13: a uav_report from the Orin -> validated (his UAV_REPORT_KINDS + RoboNation's schema) -> event/uav/<kind> for ocs_app, and
        uav_report_ack {id, ok, error} back. A bad report is answered ok: false and never published."""
        rid, kind = d.get("id"), d.get("kind")
        if rid in self.seen_reports:
            self.sender.send_once({"type": "uav_report_ack", "id": rid, "ok": True})
            return
        if kind not in W.UAV_REPORT_KINDS:
            self.sender.send_once({"type": "uav_report_ack", "id": rid, "ok": False, "error": f"unknown report '{kind}'"})
            return
        err = validate_report(kind, d.get("body") or {})
        if err:
            self.sender.send_once({"type": "uav_report_ack", "id": rid, "ok": False, "error": err[:200]})
            self._event(f"report {kind} REJECTED: {err[:80]}")
            return
        self.bus.publish(self.cfg.team("event", self.tok, kind), "event",
                         {"event": "uav_report", "kind": kind, "body": d.get("body") or {}, "id": rid}, qos=1, timeout_s=0.0, vehicle_id=self.vid)
        self.seen_reports = (self.seen_reports + [rid])[-500:]
        self.reports_n += 1
        self.last_report = {"kind": kind, "t": utc_iso()}
        self.sender.send_once({"type": "uav_report_ack", "id": rid, "ok": True})

    # ---- us -> the Orin: the commands ocs_app sends on cmd/uav/# --------------------------------------------------------------
    def _ack(self, env: dict, result: str, reason: str = "") -> None:
        self.bus.publish(self.cfg.team("ack", self.tok), "ack", {"ack_of": env["msg_id"], "result": result, "reason": reason,
                                                                  "state": self.state}, qos=1, timeout_s=0.0, vehicle_id=self.vid,
                         intent_id=env.get("intent_id"))

    def on_cmd(self, topic: str, env: dict) -> None:
        if env.get("kind") != "cmd":
            return
        b = env.get("body") or {}
        a, args = b.get("action"), dict(b.get("args") or {})
        if a == "PREPARE":
            self._ack(env, "ACCEPTED")
            why = self.prepare_problem(args.get("course"))
            self._ack(env, "DONE" if why is None else "REJECTED", why or "UAV ready: telemetry fresh, disarmed, fence valid, pilot ready")
        elif a == "START_RUN":
            with self.lock:
                self.run_id = args.get("run_id")
                self.pending_task = args.get("uav_task") or None
                if self.pending_task and self.runstart_acked_run is not None:
                    self.task = TASK_OF.get(str(self.pending_task), "TASK_NONE")
            self._ack(env, "DONE", f"run {self.run_id}")
        elif a in ("STOP_RUN", "RESET"):
            with self.lock:
                self.pilot_ready = False                       # D10: cleared by STOP_RUN / RESET
                self.run_id, self.pending_task, self.runstart_acked_run = None, None, None
                if self.task_override is None:
                    self.task = "TASK_NONE"
            self._ack(env, "DONE", "pilot ready cleared")
        elif a == "UAV_READY":
            self.pilot_ready = bool(args.get("on"))
            self._event("pilot ready " + ("ON" if self.pilot_ready else "off"))
            self._ack(env, "DONE", "pilot ready " + ("on" if self.pilot_ready else "off"))
        elif a == "UAV_TASK":
            t = str(args.get("task") or "none").lower()
            if t not in TASK_OF:
                self._ack(env, "REJECTED", "uav task none | 1 | 2 | 3 | 4")
                return
            with self.lock:
                self.task = TASK_OF[t]
                self.task_override = None if t == "none" else self.task
            self._ack(env, "DONE", f"current_task {self.task}")
        elif a == "FENCE_REJECTED":
            self._ack(env, "ACCEPTED")
            self.fence_rejected_t = utc_iso()
            self.sender.send({"type": "fence_rejected", "reason": str(args.get("reason") or "")}, "Run Request rejected - clear the fence",
                             on_ack=lambda: self._ack(env, "DONE", "the listener has fence_rejected"),
                             on_fail=lambda: self._ack(env, "FAILED", "the listener never acknowledged fence_rejected"))
        elif a == "FENCE_CHANGE_RESULT":
            cid = str(args.get("id"))
            res = {"type": "fence_change_result", "id": args.get("id"), "ok": bool(args.get("ok")), "text": str(args.get("text") or "")}
            if cid.startswith("alt-"):                         # a ceiling re-declaration: his bridge tells the GUI with a notice
                self.sender.send_once({"type": "bridge_notice", "ok": res["ok"], "text": res["text"]})
            else:
                self.fence_results[cid] = res
                self.sender.send_once(res)
            self.last_fence_result = dict(res, t=utc_iso())
            self._ack(env, "DONE")
        elif a == "BRIDGE_NOTICE":
            self.sender.send_once({"type": "bridge_notice", "ok": bool(args.get("ok", True)), "text": str(args.get("text") or "")})
            self._ack(env, "DONE")
        elif a == "RC_FORWARD":
            self.forward(env, str(args.get("kind")), args.get("message") or {}, args)
        elif a == "RESEND_COURSE":
            if self.course is None:
                self._ack(env, "REJECTED", "no RxCourse cached yet - wait for RoboCommand's course")
                return
            self._ack(env, "ACCEPTED")
            self._send_course(env, self.course, "course reapply")
        elif a in ("INCIDENT", "COURSE"):
            self._ack(env, "DONE", "forwarded with the RoboCommand traffic (cmd/uav/rc)")
        else:
            self._ack(env, "REJECTED", f"{a}: not a UAV command (nothing here commands the drone)")

    def forward(self, env: dict, kind: str, message: dict, args: dict) -> None:
        """D12: RoboCommand's course / command to the Orin, reliably; DONE on the listener's ack, FAILED on the 15 s give-up."""
        if kind == "course":
            self._ack(env, "ACCEPTED")
            fp = W.course_fingerprint(message)
            self.course, self.course_fp = message, fp
            self._send_course(env, message, f"course {message.get('course_id') or '?'}")
            return
        if kind != "command":
            self._ack(env, "REJECTED", f"rc kind {kind}: course | command")
            return
        self._ack(env, "ACCEPTED")
        which = str(args.get("which") or "command")
        label = f"command #{message.get('seq')} {which}"
        rec = {"kind": which, "seq": message.get("seq"), "t": utc_iso(), "acked": None}
        self.last_forward = rec

        def on_ack():
            rec["acked"] = utc_iso()
            if which == "run_start":
                with self.lock:
                    self.runstart_acked_run = self.run_id if self.run_id is not None else -1
                    if self.pending_task:
                        self.task = TASK_OF.get(str(self.pending_task), "TASK_NONE")
            self._ack(env, "DONE", f"the Orin has {label}")

        def on_fail():
            rec["acked"] = False
            self._ack(env, "FAILED", f"the Orin did not take {label} (no ack in {self.sender.retry_for_s:g} s)")
        self.sender.send({"type": "robocommand", "kind": "command", "message": message}, label, on_ack=on_ack, on_fail=on_fail)

    def _send_course(self, env: dict, message: dict, label: str) -> None:
        if self.course_msg is not None:
            self.sender.cancel(self.course_msg)                # the newest course wins (his rule)
        fp = W.course_fingerprint(message)

        def on_ack():
            if self.course_fp == fp:
                self.course_acked_t = self.clock()
                self.course_msg = None
            self._ack(env, "DONE", f"the Orin has {label} ({fp[:12]})")

        def on_fail():
            self._ack(env, "FAILED", f"the Orin did not take {label} (no ack in {self.sender.retry_for_s:g} s) - uav resend_course")
        self.course_msg = self.sender.send({"type": "robocommand", "kind": "course", "message": message}, label, on_ack=on_ack, on_fail=on_fail)

    def on_ocs_state(self, topic: str, env: dict) -> None:
        self.ocs = dict((env.get("body") or {}).get("uav") or {})

    # ---- D10: PREPARE ---------------------------------------------------------------------------------------------------
    def telemetry_age(self) -> Optional[float]:
        return None if self.tel_t is None else self.clock() - self.tel_t

    def prepare_problem(self, course: Optional[dict] = None) -> Optional[str]:
        """The first failing reason, or None = DONE (telemetry fresh < 5 s, disarmed, the FCU polygon passes geofence_problems() against the
        received course, pilot ready)."""
        with self.lock:
            age, t = self.telemetry_age(), dict(self.tel or {})
        if age is None:
            return "no UAV telemetry from the Orin (is the drone GUI / listener running?)"
        if age > self.stale_s:
            return f"UAV telemetry stale ({age:.0f} s > {self.stale_s:g} s)"
        if t.get("armed"):
            return "the drone is ARMED - a declaration is made only disarmed, on the ground"
        fence = t.get("fcu_fence") or []
        if len(fence) < 3:
            return "the FCU has no single polygon fence to declare"
        probs = fence_check(fence, t.get("fence_alt_max"), course or self.course)
        if probs:
            return "; ".join(probs)
        if not self.pilot_ready:
            return "waiting for the pilot: uav ready on"
        return None

    # ---- status/uav (D5) -------------------------------------------------------------------------------------------------
    def status_body(self) -> dict:
        with self.lock:
            t = dict(self.tel or {})
            age = self.telemetry_age()
            if t:
                mode = t.get("mode")
                if mode in AUTO_MODES:
                    self.state = "AUTO_TASK" if self.task != "TASK_NONE" else "AUTO_HOLD"
                else:
                    self.state = "MANUAL"
            fresh = age is not None and age <= self.stale_s
            fence = _ring(t.get("fcu_fence"))
            decl = self.ocs.get("declared_fence") or []
            dc = self.ocs.get("declared_ceiling_hae_m")
            matches = None
            if decl and fence:
                matches = (len(decl) == len(fence) and all(abs(a[0] - b[0]) < 1e-6 and abs(a[1] - b[1]) < 1e-6 for a, b in zip(decl, fence))
                           and (dc is None or self.alt_max is None or abs(float(dc) - float(self.alt_max)) < 0.01))
            return {"state": self.state, "robot_state": (t.get("state") or "STATE_UNKNOWN") if fresh else "STATE_UNKNOWN",
                    "current_task": self.task, "task_state": None, "mode": t.get("mode"), "armed": t.get("armed"),
                    "lat": t.get("latitude"), "lon": t.get("longitude"), "altitude_hae_m": t.get("altitude_hae_m"),
                    "altitude_source": t.get("altitude_source"), "flight_phase": t.get("flight_phase"),
                    "sog_mps": t.get("spd_mps"), "hdg_deg": t.get("heading_deg"), "roll_deg": t.get("roll_deg"), "pitch_deg": t.get("pitch_deg"),
                    "vehicle_type": "UAV", "uav_registration": self.u["registration"],
                    "health": {"telemetry_age_s": None if age is None else round(age, 2), "telemetry_n": self.tel_n,
                               "fence_ok": not fence_check(t.get("fcu_fence") or [], t.get("fence_alt_max"), self.course) if fence else False,
                               "pilot_ready": self.pilot_ready, "stale_s": self.stale_s},
                    "fcu_link": self.fcu_link,
                    "fence": {"points": fence, "points_n": max(0, len(fence) - 1), "ceiling_hae_m": self.alt_max, "matches_declared": matches,
                              "change_pending": bool(self.fence_waiting) or self.alt_resend, "last_result": self.last_fence_result,
                              "rejected_sent": self.fence_rejected_t},
                    "uav": {"vehicle_id": self.vid, "registration": self.u["registration"], "pilot_callsign": self.u["pilot_callsign"],
                            "declared": bool(self.ocs.get("declared"))},
                    "robocommand": {"course_fp": (self.course_fp or "")[:12] or None, "course_id": (self.course or {}).get("course_id"),
                                    "course_acked_age_s": None if self.course_acked_t is None else round(self.clock() - self.course_acked_t, 1),
                                    "last_forward": self.last_forward, "reports_n": self.reports_n, "last_report": self.last_report},
                    "flown": {"armed_seen": self.ever_armed, "airborne_seen": self.ever_airborne},
                    "listener": {"host": self.listener[0], "port": self.listener[1], "bound": list(self.sender.bound)},
                    "run": {"run_id": self.run_id}, "events": self.events[-8:]}

    def publish_status(self) -> None:
        self.bus.publish(self.cfg.team("status", self.tok), "status", self.status_body(), qos=0, timeout_s=0.0, vehicle_id=self.vid)

    def _event(self, text: str) -> None:
        self.events = (self.events + [{"t": utc_iso(), "text": text}])[-20:]
        log.info("uav: %s", text)


def validate_report(kind: str, body: dict) -> Optional[str]:
    """RoboNation's schema (our vendored rc_pb): the body must parse into RxReport.<kind>; None = fine, else the parser's message."""
    try:
        from google.protobuf import json_format

        from ..ocs.rc_pb import rx_reports_pb2 as REP
        json_format.ParseDict({"team_id": "SGIT", "vehicle_id": "X", "seq": 1, kind: body}, REP.RxReport())
        return None
    except Exception as e:                                     # ParseError, ValueError, TypeError
        return str(e) or e.__class__.__name__


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="optiprimex UAV adapter: the UAV team's §5 wire <-> our team bus (no drone control)")
    ap.add_argument("--config", required=True, help="the OCS profile (mqtt + the ocs.uav block)")
    ap.add_argument("--uav-host", default=os.environ.get("UAV_HOST", "127.0.0.1"), help="the Orin's listener (UAV_HOST)")
    ap.add_argument("--uav-port", type=int, default=int(os.environ.get("UAV_LISTEN_PORT", W.DEFAULT_LISTEN_PORT)))
    ap.add_argument("--bridge-port", type=int, default=int(os.environ.get("UAV_BRIDGE_PORT", W.DEFAULT_BRIDGE_PORT)))
    ap.add_argument("--bind", default=os.environ.get("UAV_BRIDGE_BIND", "0.0.0.0"))
    ap.add_argument("--retry-for", type=float, default=15.0)
    ap.add_argument("--fcu-link", choices=("real", "fake"), default=os.environ.get("UAV_FCU_LINK", "real"))
    ap.add_argument("--log-level", default="INFO")
    a = ap.parse_args(argv)
    logging.basicConfig(level=getattr(logging, a.log_level.upper()), format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = cfgmod.load(a.config)
    ad = UavAdapter(cfg, (a.uav_host, a.uav_port), a.bridge_port, a.bind, retry_for_s=a.retry_for, fcu_link=a.fcu_link)
    ad.start()
    path = os.environ.get("UAV_BUOY_PATH", "").strip().lower()       # v0.1.66 (D21): direct | relay | off
    if path == "relay" and os.environ.get("UAV_BROKER_HOST"):
        ad.start_buoy_relay(os.environ["UAV_BROKER_HOST"], int(os.environ.get("UAV_BROKER_PORT", "1883")))
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.is_set():
            time.sleep(0.2)
    finally:
        ad.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
