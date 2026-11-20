"""fault_timing: the fault-injection timing test of OCS_V2_SPEC §13 (v0.1.43) against a running simulator (`scripts/run_sim.sh pool`).

Eight faults are injected one at a time on a ready boat (handed over, AUTO_HOLD, a run); for each, the OCS page's /state.json is
polled at 5 Hz until the fault is NAMED where the operator looks without a click - the state word + why (the top bar), the
attention lines (the two most severe warnings under the bar) or the RUNSTART slot - and the FIRST ACTION is readable there.
Pass = both within 3 s. Then the fault is removed and the stack cleaned (RESET SIM).

  python -m optiprimex.tools.fault_timing [--page http://127.0.0.1:8096/] [--md FILE] [--only kill,rc6,...]

Faults: kill · rc6 (transmitter override) · rc_lost · gps (fix lost) · fcu (stream paused 4 s) · bridge (the boat broker paused 5 s -
the bridge side; SIGSTOP / SIGCONT by its PID, found by its port) · refusal (the fake judge refuses the declaration, silently) ·
perception (cv_publish suspended in its tmux window, Ctrl-Z / fg).
v0.1.65 (D18), with --uav (a stack started with --uav; the UAV declared + pilot ready in each setup): uav_tel (the UAV team's listener
SIGSTOPped 6 s by its PID), uav_fence (a fence outside the course - the adapter's event/uav/fence_change with those vertices, published on
the hub as the adapter would after a Mission Planner write), uav_adapter (the adapter stopped with C-c in its tmux window, restarted after).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
import urllib.request
from typing import Callable, Dict, List, Optional, Tuple

from .sim_day import Walk

LIMIT_S = 3.0


def page_state(page: str) -> dict:
    try:
        with urllib.request.urlopen(page.rstrip("/") + "/state.json", timeout=2) as r:
            return json.loads(r.read())
    except Exception:
        return {}


def visible(st: dict) -> List[str]:
    """What the operator reads without a click: the top bar (word, why, hint), the two attention lines, the RUNSTART slot's facts."""
    b = st.get("bar") or {}
    out = [f"{b.get('word', '')} {b.get('why', '')} {b.get('hint', '')}",
           f"FCU {b.get('holder')} {b.get('v')} {b.get('mode')} {'KILL' if b.get('kill') else ''} {'stale' if b.get('stale') else ''}"]
    w = sorted(st.get("warnings") or [], key=lambda x: x.get("style") != "bad")[:2]
    out += [x.get("text", "") for x in w]
    o = st.get("ocs") or {}
    if o.get("r") == "R4":
        out.append("waiting for the course manager · a refusal is silent: if nothing comes, ask the course manager")
    return out


# fault: (name it with this, the first action with this)
EXPECT: Dict[str, Tuple[str, str]] = {
    "kill": (r"KILL", r"confirm_safe|release|RE-ARM"),
    "rc6": (r"RC6|MANUAL|transmitter|takeover", r"transmitter|RC6|takeover|enable_auto|handover|RE-ARM"),
    "rc_lost": (r"RC LOST|receiver", r"transmitter|HOME|takeover"),
    "gps": (r"GPS|EKF", r"confirm_safe|takeover|wait"),
    "fcu": (r"FCU_LOST|FCU LINK|no heartbeat", r"takeover|UART|power|confirm_safe"),
    "bridge": (r"LINK LOST|bridge", r"radio"),
    "refusal": (r"waiting for (RunStart|the course manager)", r"ask the (staff|course manager)"),
    "perception": (r"PERCEPTION", r"CAMERA|observer"),
    "uav_tel": (r"UAV telemetry slow|UAV heartbeat stopped", r"listener|Orin|14600"),               # v0.1.65 (D18)
    "uav_fence": (r"FENCE CHANGED|DECLARATION REFUSED \(UAV\)", r"redraw|cleared|tell the judges|Mission Planner"),
    "uav_adapter": (r"UAV heartbeat stopped|no UAV adapter", r"start it|check the Orin|adapter"),
}
OUTSIDE_FENCE = [[1.2826, 103.8536], [1.2826, 103.8576], [1.2846, 103.8576], [1.2846, 103.8536]]   # ~200 m north of Course EXTRA


class Timing(Walk):
    def __init__(self, a):
        super().__init__(a)
        self.page = a.page
        self.results: List[dict] = []

    def broker_pid(self, port: int) -> Optional[int]:
        try:
            out = subprocess.run(["lsof", "-t", "-nP", f"-iTCP:{port}", "-sTCP:LISTEN"], capture_output=True, text=True, timeout=5).stdout.split()
            return int(out[0]) if out else None
        except (OSError, ValueError, subprocess.TimeoutExpired):
            return None

    def measure(self, name: str, inject: Callable[[], None], undo: Callable[[], None], setup: Optional[Callable[[], None]] = None) -> dict:
        self.clean()
        (setup or self.ready)()
        time.sleep(2.0)
        pat_name, pat_act = EXPECT[name]
        t0 = time.time()
        inject()
        t_name = t_act = None
        seen = ""
        while time.time() - t0 < 12.0:
            txt = " | ".join(visible(page_state(self.page)))
            dt = time.time() - t0
            if t_name is None and re.search(pat_name, txt, re.I):
                t_name, seen = dt, txt
            if t_name is not None and t_act is None and re.search(pat_act, txt, re.I):
                t_act, seen = dt, txt
            if t_name is not None and t_act is not None:
                break
            time.sleep(0.2)
        undo()
        r = {"fault": name, "named_s": round(t_name, 1) if t_name is not None else None, "action_s": round(t_act, 1) if t_act is not None else None,
             "ok": t_name is not None and t_act is not None and t_act <= LIMIT_S, "seen": seen[:240]}
        self.results.append(r)
        print(f"  {'ok ' if r['ok'] else 'BAD'} {name:10s} named {r['named_s']} s, action {r['action_s']} s - {r['seen'][:150]}", flush=True)
        time.sleep(2.0)
        return r

    def run(self, only: List[str]) -> List[dict]:
        sess = self.a.session
        pid = self.broker_pid(self.a.boat_port)
        steps = {
            "kill": (lambda: self.fcu(kill=True), lambda: (self.fcu(kill=False), self.cli("confirm_safe")), None),
            "rc6": (lambda: self.fcu(rc6=1900), lambda: self.fcu(rc6=1000), None),
            "rc_lost": (lambda: self.fcu(rc_lost=True), lambda: self.fcu(rc_lost=False), None),
            "gps": (lambda: self.fcu(gps_fix=0), lambda: self.fcu(gps_fix=3), None),
            "fcu": (lambda: self.sim("/fcu", {"pause_stream": 4.0}), lambda: time.sleep(4.0), None),
            "bridge": ((lambda: os.kill(pid, signal.SIGSTOP)) if pid else (lambda: None),
                       (lambda: (time.sleep(max(0.0, 5.0 - 0.0)), os.kill(pid, signal.SIGCONT))) if pid else (lambda: None), None),
            "refusal": (lambda: self.cli("declare", wait=3),
                        lambda: self.inject_judge({"kind": "sim_config", "refuse": False}),
                        lambda: (self.handover(), self.fcu(mode=4), self.cli("enable_auto", wait=5),
                                 self.inject_judge({"kind": "sim_config", "refuse": True}))),
            "perception": (lambda: subprocess.run(["tmux", "send-keys", "-t", f"{sess}:cvpub", "C-z"], timeout=5),
                           lambda: subprocess.run(["tmux", "send-keys", "-t", f"{sess}:cvpub", "fg", "C-m"], timeout=5), None),
        }
        if getattr(self.a, "uav", False):                              # v0.1.65 (D18): the UAV's three
            lpid = self.pid_of("geofence_udp_listener")
            uav_setup = lambda: (self.cli("declare", "uav", "on"), self.cli("uav", "ready", "on"), self.ready())
            steps.update({
                "uav_tel": ((lambda: os.kill(lpid, signal.SIGSTOP)) if lpid else (lambda: None),
                            (lambda: (time.sleep(6.0), os.kill(lpid, signal.SIGCONT))) if lpid else (lambda: None), uav_setup),
                "uav_fence": (lambda: self.uav_event({"event": "fence_change", "id": "ft-%d" % int(time.time()), "vertices": OUTSIDE_FENCE,
                                                      "ceiling_hae_m": 20.0}), lambda: None, uav_setup),
                "uav_adapter": (lambda: subprocess.run(["tmux", "send-keys", "-t", f"{sess}:uav", "C-c"], timeout=5),
                                lambda: (time.sleep(1.0), subprocess.run(["tmux", "send-keys", "-t", f"{sess}:uav", ". ./adapter.sh", "C-m"], timeout=5),
                                         time.sleep(4.0)), uav_setup),
            })
        for name, (inj, undo, setup) in steps.items():
            if only and name not in only:
                continue
            try:
                self.measure(name, inj, undo, setup)
            except Exception as exc:                                  # a broken injection is a finding, not the end
                self.results.append({"fault": name, "named_s": None, "action_s": None, "ok": False, "seen": f"crash {exc}"})
                if name == "bridge" and pid:
                    os.kill(pid, signal.SIGCONT)
        self.clean()
        return self.results

    @staticmethod
    def pid_of(word: str) -> Optional[int]:
        """The PID of our one process whose command line holds `word` (pgrep lists; we signal that PID only - never pkill -f)."""
        try:
            out = subprocess.run(["pgrep", "-f", word], capture_output=True, text=True, timeout=5).stdout.split()
            pids = [int(x) for x in out if int(x) != os.getpid()]
            return pids[0] if pids else None
        except (OSError, ValueError, subprocess.TimeoutExpired):
            return None

    def uav_event(self, body: dict) -> None:
        from ..companion.mqtt import Envelope
        env = Envelope.wrap("event", body, "UAV1", 1, source="fault_timing")
        subprocess.run(["mosquitto_pub", "-p", str(self.a.hub_port), "-q", "1", "-t", "team/optiprime/v1/event/uav/fence_change",
                        "-m", json.dumps(env)], timeout=5)

    def inject_judge(self, body: dict) -> None:
        subprocess.run(["mosquitto_pub", "-p", str(self.a.hub_port), "-t", "rc/sim/inject", "-m", json.dumps(body)], timeout=5)


def table(results: List[dict]) -> str:
    out = ["| fault | named (s) | first action readable (s) | ≤ 3 s | what the page showed |", "|---|---|---|---|---|"]
    for r in results:
        out.append(f"| {r['fault']} | {r['named_s']} | {r['action_s']} | {'yes' if r['ok'] else '**NO**'} | {r['seen'][:160].replace('|', '/')} |")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--page", default="http://127.0.0.1:8096/")
    ap.add_argument("--sim", default="http://127.0.0.1:8097")
    ap.add_argument("--hub-port", type=int, default=1882)
    ap.add_argument("--boat-port", type=int, default=1884)
    ap.add_argument("--session", default=os.environ.get("OPTIPRIMEX_SESSION", "optiprimex"))
    ap.add_argument("--only", default="")
    ap.add_argument("--md", default="")
    ap.add_argument("--uav", action="store_true", help="v0.1.65: + the UAV's three faults (a stack started with --uav)")
    a = ap.parse_args(argv)
    t = Timing(a)
    res = t.run([x for x in a.only.split(",") if x])
    txt = table(res)
    print("\n" + txt)
    if a.md:
        with open(a.md, "w") as f:
            f.write(f"# fault-injection timing - {time.strftime('%Y-%m-%d %H:%M')} - page {a.page}\n\n{txt}\n")
    return 0 if all(r["ok"] for r in res) else 1


if __name__ == "__main__":
    raise SystemExit(main())
