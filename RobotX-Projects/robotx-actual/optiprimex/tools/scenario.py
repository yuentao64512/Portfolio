#!/usr/bin/env python3
"""scenario: guided bench scenarios, one step at a time, with the reasoning printed before each step.

  op-scenario list
  op-scenario nominal | kill_in_task | rc_override | fcu_loss | incident_idle | wrong_order | stall  [--pause]

Each step says what is sent, why, and what should happen; then the observed state is checked and marked PASS / FAIL.
--pause waits for Enter before every step so you can watch the TUI (op-tui) in another window. Needs `make bench`
running (hub 1882, boat 1884, fake FCU control udp 14560); `stall` needs the judge started with --refuse.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from typing import Callable, List, Optional, Tuple

from . import ocs_cli, playbook

Step = Tuple[str, str, Callable]          # (kind, explanation, fn) ; fn(ctx) -> (ok, observed)


class Ctx:
    def __init__(self, a):
        self.a = a
        self.con = ocs_cli.OcsConsole(a.host, a.port, vehicle_id=a.vehicle_id, vehicle=a.vehicle, operator="scenario")

    def v(self) -> Optional[str]:
        return self.con.vehicle_state()

    def veh(self) -> dict:
        return ((self.con.state or {}).get("vehicles") or {}).get(self.a.vehicle_id) or {}

    def r(self) -> Optional[str]:
        return (self.con.state or {}).get("r")

    def plant(self, msg: dict) -> None:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.a.fcu_control))

    def cli(self, *argv: str, expect_fail: bool = False) -> Tuple[bool, str]:
        ns = ocs_cli.parse(list(argv))
        code, lines = ocs_cli.execute(self.con, ns)
        ok = (code != 0) if expect_fail else (code == 0)
        return ok, " | ".join(lines)

    def wait(self, pred: Callable[[], bool], timeout_s: float, what: str) -> Tuple[bool, str]:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if pred():
                return True, f"{what} after {time.monotonic() - t0:.1f} s"
            time.sleep(0.2)
        return False, f"{what} not seen within {timeout_s:.0f} s (R={self.r()} V={self.v()} task={self.veh().get('current_task')} {self.veh().get('task_state')})"


def S(kind: str, why: str, fn: Callable) -> Step:
    return (kind, why, fn)


def _ready(ctx: Ctx) -> Tuple[bool, str]:
    return ctx.cli("ready", "--wait", "6")


def _reset(ctx: Ctx) -> Tuple[bool, str]:
    """Known start: no run (R2/R6), vehicle MANUAL. Uses the same intents an operator would."""
    notes = []
    ctx.con.wait_state(3.0)
    if ctx.v() in ("AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD"):
        ok, out = ctx.cli("manual", "--wait", "6")
        notes.append("manual: " + ("ok" if ok else out))
    if ctx.v() in ("KILLED", "FAULT"):
        ctx.plant({"set": {"kill": False, "rc6": 1000, "ekf_ok": True}})
        time.sleep(1.0)
        ok, out = ctx.cli("confirm_safe", "--wait", "6")
        notes.append("confirm_safe: " + ("ok" if ok else out))
    if ctx.r() in ("R3", "R4", "R5"):
        ok, out = ctx.cli("end_run", "--wait", "6")
        notes.append("end_run: " + ("ok" if ok else out))
    ok, _ = ctx.wait(lambda: ctx.r() in ("R2", "R6") and ctx.v() == "MANUAL", 8, "reset")
    return ok, f"R={ctx.r()} V={ctx.v()} " + "; ".join(notes)


RESET = S("macro", "reset to a known start: abort + disable_auto if in auto, confirm_safe if killed/faulted, end_run if a run is open", _reset)


SCENARIOS = {
    "nominal": [
        RESET,
        S("intent", "declare: the OCS publishes the RunDeclaration and PREPARE; R2 -> R3; heartbeats start", lambda c: c.cli("declare")),
        S("wait", "the fake judge sends RunStart 2 s after a valid declaration; the OCS accepts it from R3 (judge owns the clock)",
          lambda c: c.wait(lambda: c.r() == "R5", 10, "R5 RUNNING")),
        S("intent", "enable_auto: vehicle guards (armed, EKF, PREPARE) then V2 -> V3; HOLD on the plant", lambda c: c.cli("enable_auto", "--wait", "6")),
        S("intent", "start_task T4: needs V3 and a verified run; T4 waits in LISTEN", lambda c: c.cli("start_task", "T4", "--wait", "6")),
        S("intent", "inject assistance: the bench publishes the AssistanceRequest the judge would; T4 acks within a tick and transits",
          lambda c: c.cli("inject", "assistance", "--lat", "1.28111", "--lon", "103.85501", "--cseq", "7")),
        S("wait", "TRANSIT: a GUIDED carrot to the request position; the plant moves at <= 1 m/s",
          lambda c: c.wait(lambda: c.veh().get("task_state") in ("TRANSIT", "READY", "WAIT_CONFIRM", "DONE"), 10, "T4 TRANSIT")),
        S("wait", "READY -> readiness report -> the fake judge confirms 1 s later -> DONE -> back to AUTO_HOLD, TASK_NONE",
          lambda c: c.wait(lambda: c.v() == "AUTO_HOLD" and c.veh().get("current_task") == "TASK_NONE", 90, "task done, AUTO_HOLD")),
        S("intent", "end_run: STOP_RUN to the vehicle, R5 -> R6; nothing from run 1 is valid afterwards", lambda c: c.cli("end_run", "--wait", "6")),
        S("wait", "R6 ENDED", lambda c: c.wait(lambda: c.r() == "R6", 5, "R6")),
    ],
    "kill_in_task": [
        RESET,
        S("macro", "ready: declare + enable_auto + wait for RunStart", _ready),
        S("intent", "goto a far point so a task is running when the kill comes", lambda c: c.cli("goto", "--x", "60", "--y", "0", "--wait", "6")),
        S("wait", "V4 AUTO_TASK (GOTO)", lambda c: c.wait(lambda: c.v() == "AUTO_TASK", 5, "AUTO_TASK")),
        S("plant", "assert the kill line on the fake FCU: precedence puts KILLED above everything within one tick; the task is aborted, no motion",
          lambda c: (c.plant({"set": {"kill": True}}) or True, "kill=True sent")),
        S("wait", "V1 KILLED, STATE_KILLED in the heartbeat", lambda c: c.wait(lambda: c.v() == "KILLED", 3, "KILLED")),
        S("plant", "release the kill: the vehicle must NOT leave KILLED on its own (ARCHITECTURE 3.2)", lambda c: (c.plant({"set": {"kill": False}}) or True, "kill=False sent")),
        S("wait", "still KILLED 2 s later", lambda c: (time.sleep(2.0) or c.v() == "KILLED", f"V={c.v()}")),
        S("intent", "enable_auto now is rejected: only CONFIRM_SAFE leaves V1", lambda c: c.cli("enable_auto", expect_fail=True)),
        S("intent", "confirm_safe: the operator's confirmation -> MANUAL (DONE ack when V2 is entered)", lambda c: c.cli("confirm_safe", "--wait", "6")),
        S("wait", "V2 MANUAL", lambda c: c.wait(lambda: c.v() == "MANUAL", 5, "MANUAL")),
        S("intent", "enable_auto again: the run is still R5, so the vehicle can take a task straight away", lambda c: c.cli("enable_auto", "--wait", "6")),
        S("intent", "end_run", lambda c: c.cli("end_run", "--wait", "6")),
    ],
    "rc_override": [
        RESET,
        S("macro", "ready", _ready),
        S("plant", "RC6 high: the safety operator's switch; V3/V4/V5 -> V2 at once, no software may object", lambda c: (c.plant({"set": {"rc6": 1900}}) or True, "rc6=1900")),
        S("wait", "V2 MANUAL", lambda c: c.wait(lambda: c.v() == "MANUAL", 3, "MANUAL")),
        S("plant", "RC6 low again: the vehicle stays MANUAL (no automatic return to auto)", lambda c: (c.plant({"set": {"rc6": 1000}}) or True, "rc6=1000")),
        S("wait", "still MANUAL 2 s later", lambda c: (time.sleep(2.0) or c.v() == "MANUAL", f"V={c.v()}")),
        S("intent", "enable_auto: the operator re-enables explicitly", lambda c: c.cli("enable_auto", "--wait", "6")),
        S("wait", "V3 AUTO_HOLD", lambda c: c.wait(lambda: c.v() == "AUTO_HOLD", 5, "AUTO_HOLD")),
        S("intent", "end_run", lambda c: c.cli("end_run", "--wait", "6")),
    ],
    "fcu_loss": [
        RESET,
        S("macro", "ready", _ready),
        S("plant", "pause the FCU stream 4 s: after timeouts.fcu_s (2 s) health raises FCU_LOST, a hard fault -> V6", lambda c: (c.plant({"pause_stream": 4.0}) or True, "pause_stream 4")),
        S("wait", "V6 FAULT", lambda c: c.wait(lambda: c.v() == "FAULT", 8, "FAULT")),
        S("wait", "stream back (fcu age < 1 s) but still FAULT: a fault needs an operator action, never straight back to auto",
          lambda c: c.wait(lambda: (c.con.status or {}).get("health", {}).get("fcu_age_s", 9) < 1.0 and c.v() == "FAULT", 10, "stream back, still FAULT")),
        S("intent", "confirm_safe -> MANUAL", lambda c: c.cli("confirm_safe", "--wait", "6")),
        S("wait", "V2 MANUAL", lambda c: c.wait(lambda: c.v() == "MANUAL", 5, "MANUAL")),
        S("intent", "end_run", lambda c: c.cli("end_run", "--wait", "6")),
    ],
    "incident_idle": [
        RESET,
        S("macro", "ready", _ready),
        S("intent", "inject assistance with NO task running: an AssistanceRequest in AUTO_HOLD auto-starts T4 (the ack must go out regardless)",
          lambda c: c.cli("inject", "assistance", "--lat", "1.28114", "--lon", "103.85509", "--cseq", "9")),
        S("wait", "TASK_DYNAMIC_INCIDENT in the heartbeat", lambda c: c.wait(lambda: c.veh().get("current_task") == "TASK_DYNAMIC_INCIDENT", 5, "T4 running")),
        S("wait", "chain completes (ack, transit, readiness, judge confirm, done)", lambda c: c.wait(lambda: c.v() == "AUTO_HOLD" and c.veh().get("current_task") == "TASK_NONE", 90, "done")),
        S("intent", "end_run", lambda c: c.cli("end_run", "--wait", "6")),
    ],
    "wrong_order": [
        RESET,
        S("intent", "start_task before anything: bench mode lets the OCS pass it, the VEHICLE rejects it (no PREPARE / not in AUTO) -- expected failure",
          lambda c: c.cli("start_task", "T4", expect_fail=True)),
        S("intent", "resume in MANUAL: nothing is held -- expected failure", lambda c: c.cli("resume", expect_fail=True)),
        S("intent", "confirm_safe in MANUAL: nothing to confirm -- expected failure", lambda c: c.cli("confirm_safe", expect_fail=True)),
        S("macro", "ready (declare + enable_auto + RunStart)", _ready),
        S("intent", "declare while RUNNING: the authority refuses (R5); end_run first -- expected failure", lambda c: c.cli("declare", expect_fail=True)),
        S("intent", "start_task T1: not built on the vehicle, rejected before the FSM moves -- expected failure", lambda c: c.cli("start_task", "T1", expect_fail=True)),
        S("intent", "goto, then disable_auto while the task runs: rejected, abort first -- expected failure",
          lambda c: (c.cli("goto", "--x", "40", "--y", "0", "--wait", "6")[0] and c.cli("disable_auto", expect_fail=True)[0], "goto accepted, disable_auto rejected")),
        S("macro", "manual macro: abort, then disable_auto, each a proper intent", lambda c: c.cli("manual", "--wait", "6")),
        S("wait", "V2 MANUAL", lambda c: c.wait(lambda: c.v() == "MANUAL", 5, "MANUAL")),
        S("intent", "end_run", lambda c: c.cli("end_run", "--wait", "6")),
    ],
    "stall": [
        RESET,
        S("intent", "declare with a judge that never answers (make bench BENCH_ARGS='--rc-args \"--refuse\"')", lambda c: c.cli("declare")),
        S("intent", "enable_auto -> R4 READY: all declared vehicles in AUTO, waiting for RunStart", lambda c: c.cli("enable_auto", "--wait", "6")),
        S("wait", "R4", lambda c: c.wait(lambda: c.r() == "R4", 5, "R4 READY")),
        S("intent", "start_task now: the OCS (bench mode) passes it, the vehicle rejects it: no START_RUN -- expected failure", lambda c: c.cli("start_task", "T4", expect_fail=True)),
        S("wait", "after stall_s (60 s in the profile) the OCS raises run_start_stall and ocs/state shows stalled=true (ADR-8)",
          lambda c: c.wait(lambda: bool((c.con.state or {}).get("stalled")), 75, "stalled")),
        S("intent", "declare again is the only re-arm (new declaration_seq)", lambda c: c.cli("declare")),
        S("intent", "end_run", lambda c: c.cli("end_run", "--wait", "6")),
    ],
}


def run(name: str, a) -> int:
    steps = SCENARIOS[name]
    ctx = Ctx(a)
    ctx.con.start()
    if not ctx.con.connected(5.0):
        print(f"broker {a.host}:{a.port} not reachable (make bench?)")
        return 2
    ctx.con.wait_state(3.0)
    print(f"scenario {name}: {len(steps)} steps; start R={ctx.r()} V={ctx.v()}  (next: {playbook.suggest(ctx.con.state, a.vehicle_id)})")
    fails = 0
    try:
        for i, (kind, why, fn) in enumerate(steps, 1):
            print(f"\n[{i}/{len(steps)}] {kind}: {why}")
            if a.pause:
                input("    Enter to run this step... ")
            ok, observed = fn(ctx)
            print(f"    {'PASS' if ok else 'FAIL'}: {observed}")
            if not ok:
                fails += 1
                if a.stop_on_fail:
                    break
            time.sleep(a.step_delay)
    finally:
        time.sleep(1.5)                       # let the 1 Hz ocs/state catch up before the summary
        print(f"\nend: R={ctx.r()} V={ctx.v()} task={ctx.veh().get('current_task')}  (next: {playbook.suggest(ctx.con.state, a.vehicle_id)})")
        ctx.con.stop()
    print(f"{name}: {len(steps) - fails} passed, {fails} failed")
    return 1 if fails else 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("name", nargs="?", default="list", help="scenario name or 'list'")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--fcu-control", type=int, default=14560)
    ap.add_argument("--pause", action="store_true", help="wait for Enter before each step")
    ap.add_argument("--step-delay", type=float, default=0.5)
    ap.add_argument("--stop-on-fail", action="store_true")
    a = ap.parse_args(argv)
    if a.name == "list":
        for n, steps in SCENARIOS.items():
            print(f"{n:14s} {len(steps):2d} steps  {steps[0][1][:80]}")
        return 0
    if a.name not in SCENARIOS:
        print(f"unknown scenario {a.name}; one of: {', '.join(SCENARIOS)}")
        return 2
    return run(a.name, a)


if __name__ == "__main__":
    sys.exit(main())
