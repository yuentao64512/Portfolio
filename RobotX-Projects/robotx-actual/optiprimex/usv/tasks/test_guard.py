"""m0t9 TEST_GUARD (v0.1.80, WP36; Dmitrii 2026-11-11: "test missions to check the safety guard on the ground first" - and the motors may
pulse): the boat stands still (HOLD) for `duration_s` while the ZED guard's verdict is logged on every change (clear -> SLOW 2.4 m ->
STOP 1.3 m, the bearing extent, the pixels, the IMU roll / pitch); on a STOP it does what the scout would do first - the BRAKE frame
(mode 20, surge -pulse for pulse_s: **props in the air or the boat on a stand**) - then HOLD; the 2 m back-off is only logged (never driven on
land). A board walked toward the lens: the STOP line's range is where the guard fires (brief §4.4: 1.5 +- 0.1 m); tilt the camera +-10 deg:
it must not move. `pulse: 0` = no motor at all. No run needed (a test task); the supervisor leaves it alone (guard_exempt)."""
from __future__ import annotations

import json
import os
import time
from typing import Any, Dict

from ...companion.fsm import Transition
from ...companion.state import Hold, RobotXTask, ThrusterFrame, utc_iso
from ..behaviours import StationKeep
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"duration_s": 600.0, "pulse": 0.4, "pulse_s": 1.5, "rearm_s": 5.0, "log_dir": "logs", "lost_s": 1.5}


class TestGuard(Task):
    name = "TEST_GUARD"
    robo_task = RobotXTask.NONE
    guard_exempt = True
    BEHAVIOURS = {"WATCH": StationKeep}
    SKIP_REFUSE = {"WATCH": "a timer"}

    def build_machine(self, on_transition, clock):
        return simple_machine("TEST_GUARD", ["WATCH"], [Transition("TIME", ["WATCH"], "DONE", reason="duration over")],
                              initial="WATCH", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update((self.cfg.extra or {}).get("test_guard", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        self.p = p
        self.t0 = ctx.b.now
        self.verdict = None
        self.pulse_until = -1e9
        self.last_stop = -1e9
        self.n_stop = 0
        self.lines = []
        self.log_path = os.path.join(str(p["log_dir"]), time.strftime("guard_%Y%m%d_%H%M%S.jsonl"))
        ctx.log(f"TEST_GUARD: standing still {p['duration_s']:.0f} s; walk a board toward the bow - "
                + (f"a STOP pulses the motors astern ({p['pulse']} for {p['pulse_s']} s): PROPS IN THE AIR" if p["pulse"] else "no motor (pulse 0)"))

    def _write(self, ctx: TaskContext, line: Dict[str, Any]) -> None:
        line = dict(line, t=utc_iso())
        self.lines.append(line)
        try:
            os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(line) + "\n")
        except OSError as ex:
            ctx.log(f"TEST_GUARD log write failed: {ex}")
        self.report(ctx, "guard_test", line)

    def step(self, ctx: TaskContext):
        now = ctx.b.now
        if not self.holding and self.m.state == "WATCH":
            g = getattr(ctx.b.map, "guard_last", None) or (None, None)
            msg, t = g
            if msg is None or t is None or now - t > float(self.p["lost_s"]):
                v = "lost"
            elif msg.get("clear", True):
                v = "clear"
            else:
                v = "STOP" if msg.get("zone") == "stop" else "SLOW"
            if v != self.verdict:
                self.verdict = v
                line = {"ev": "verdict", "verdict": v}
                if msg and not msg.get("clear", True):
                    line.update({k: msg.get(k) for k in ("range_m", "bearing_deg", "bearing_min_deg", "bearing_max_deg", "px", "frames")})
                if msg:
                    line.update({k: msg.get(k) for k in ("roll_deg", "pitch_deg", "imu", "h_m") if msg.get(k) is not None})
                self._write(ctx, line)
                ctx.log(f"guard test: {v}" + (f" at {msg.get('range_m')} m {msg.get('bearing_deg')} deg ({msg.get('px')} px)" if v in ("SLOW", "STOP") else ""))
                if v == "STOP" and now - self.last_stop > float(self.p["rearm_s"]):
                    self.last_stop, self.n_stop = now, self.n_stop + 1
                    self.pulse_until = now + float(self.p["pulse_s"]) if float(self.p["pulse"]) > 0 else now
                    self._write(ctx, {"ev": "would", "action": "brake" + (" (pulsed)" if float(self.p["pulse"]) > 0 else " (no motor)"),
                                      "then": "back off 2 m astern - not driven on the ground", "range_m": msg.get("range_m")})
            if ctx.b.now - self.t0 >= float(self.p["duration_s"]):
                self._write(ctx, {"ev": "summary", "stops": self.n_stop, "duration_s": round(now - self.t0, 1)})
                if self._beh is not None:
                    self._beh.cancel()
                    self._beh = None
                self.m.handle("TIME")
        out = super().step(ctx)
        if out.out is not None:
            if now < self.pulse_until:
                s = -float(self.p["pulse"])
                out.out.cmd = ThrusterFrame(s, 0.0, s, 0.0)             # the brake frame (mode 20) - the scout's first move on a STOP
            elif not isinstance(out.out.cmd, Hold):
                out.out.cmd = Hold()
            out.out.info["guard_test"] = {"verdict": self.verdict, "stops": self.n_stop, "pulsing": now < self.pulse_until}
        return out
