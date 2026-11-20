"""m0t10 TEST_YAW (v0.1.85, WP41 T0.5): the yaw couple test - how fast does the hull turn on each thruster pair? Run-free, bench / pool only
(test mode: refused on the course profile). The TEST_SWAY pattern: mode 20 frames straight to the FCU for `duration_s`, then HOLD.

  pair side  M2 = +amp, M4 = -amp (the side pair opposed: stern to starboard, bow to port = clockwise)
  pair aft   M1 = +amp, M3 = -amp (the aft pair differential)
  pair both  both at once
  dir -1     the mirror (counter-clockwise)

The measured yaw rate (from the FCU heading, unwrapped) is logged every second and as a summary (logs/yaw_<stamp>.jsonl + a `yaw_test`
event): the number the morning needs for `mode20.yaw_side_gain` and the FCU steering preset (docs/fcu/steering_rate.param).
  start_task m0t10 --pair side --amp 0.6 --duration 10        (AUTO_HOLD, test mode, props in the water - the boat turns)
"""
from __future__ import annotations

import json
import os
import time
from typing import Any, Dict, List, Tuple

from ...companion.fsm import Transition
from ...companion.state import Hold, RobotXTask, ThrusterFrame, utc_iso
from ..behaviours import StationKeep
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"pair": "side", "amp": 0.6, "duration_s": 10.0, "dir": 1, "settle_s": 2.0, "log_dir": "logs"}
PAIRS = ("side", "aft", "both")


def yaw_frame(pair: str, amp: float, sign: int = 1) -> ThrusterFrame:
    a = max(0.0, min(1.0, float(amp))) * (1 if sign >= 0 else -1)
    m1 = m2 = m3 = m4 = 0.0
    if pair in ("side", "both"):
        m2, m4 = a, -a
    if pair in ("aft", "both"):
        m1, m3 = a, -a
    return ThrusterFrame(m1, m2, m3, m4)


def mean_rate(samples: List[Tuple[float, float]]) -> float:
    """deg/s from (t, hdg_deg) samples, unwrapped (a least-squares slope)."""
    if len(samples) < 2:
        return 0.0
    un, prev, off = [], None, 0.0
    for t, h in samples:
        if prev is not None:
            d = h - prev
            if d > 180:
                off -= 360
            elif d < -180:
                off += 360
        prev = h
        un.append((t, h + off))
    n = len(un)
    mt = sum(t for t, _ in un) / n
    mh = sum(h for _, h in un) / n
    den = sum((t - mt) ** 2 for t, _ in un)
    return sum((t - mt) * (h - mh) for t, h in un) / den if den > 1e-9 else 0.0


class TestYaw(Task):
    __test__ = False                   # not a pytest class
    name = "TEST_YAW"
    robo_task = RobotXTask.NONE
    guard_exempt = True
    BEHAVIOURS = {"TURN": StationKeep}
    SKIP_REFUSE = {"TURN": "a timer"}

    def build_machine(self, on_transition, clock):
        return simple_machine("TEST_YAW", ["TURN"], [Transition("TIME", ["TURN"], "DONE", reason="duration over")],
                              initial="TURN", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update((self.cfg.extra or {}).get("test_yaw", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        if str((args or {}).get("direction") or "").lower() == "ccw":     # ocs_cli --dir ccw
            p["dir"] = -1
        self.p = p
        self.t0 = ctx.b.now
        self.samples: List[Tuple[float, float]] = []
        self.lines: List[Dict[str, Any]] = []
        self.next_line = ctx.b.now + 1.0
        self.frame = yaw_frame(str(p["pair"]), float(p["amp"]), int(p["dir"]))
        self.log_path = os.path.join(str(p["log_dir"]), time.strftime("yaw_%Y%m%d_%H%M%S.jsonl"))
        ctx.log(f"TEST_YAW: {p['pair']} pair at {float(p['amp']):.2f} for {float(p['duration_s']):.0f} s ({'cw' if int(p['dir']) >= 0 else 'ccw'}): "
                f"frame {self.frame.as_list()} - the boat turns")

    def precheck(self, ctx: TaskContext) -> str:
        if not self.cfg.guards.test_mode:
            return "test_yaw is a bench / pool test (test mode) - refused on the course profile"
        if str(self.p["pair"]) not in PAIRS:
            return f"pair {self.p['pair']!r}: side | aft | both"
        if not 0.0 < float(self.p["amp"]) <= 1.0 or not 0.0 < float(self.p["duration_s"]) <= 60.0:
            return "amp in (0, 1], duration_s in (0, 60]"
        return ""

    def _write(self, ctx: TaskContext, line: Dict[str, Any]) -> None:
        line = dict(line, t=utc_iso())
        self.lines.append(line)
        try:
            os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(line) + "\n")
        except OSError as ex:
            ctx.log(f"TEST_YAW log write failed: {ex}")
        self.report(ctx, "yaw_test", line)

    def step(self, ctx: TaskContext):
        now = ctx.b.now
        if not self.holding and self.m.state == "TURN":
            if ctx.b.pose is not None and now - self.t0 >= float(self.p["settle_s"]):
                self.samples.append((now, ctx.b.pose.hdg_deg))
            if now >= self.next_line:
                self.next_line = now + 1.0
                self._write(ctx, {"ev": "rate", "s": round(now - self.t0, 1), "hdg_deg": None if ctx.b.pose is None else round(ctx.b.pose.hdg_deg, 1),
                                  "rate_dps": round(mean_rate(self.samples[-25:]), 2)})
            if now - self.t0 >= float(self.p["duration_s"]):
                r = mean_rate(self.samples)
                self.summary = {"ev": "summary", "pair": self.p["pair"], "amp": float(self.p["amp"]), "dir": int(self.p["dir"]),
                                "rate_dps": round(r, 2), "samples": len(self.samples), "duration_s": round(now - self.t0, 1)}
                self._write(ctx, self.summary)
                ctx.log(f"TEST_YAW {self.p['pair']} at {float(self.p['amp']):.2f}: {r:+.2f} deg/s (after {float(self.p['settle_s']):.0f} s settle)")
                if self._beh is not None:
                    self._beh.cancel()
                    self._beh = None
                self.m.handle("TIME")
        out = super().step(ctx)
        if out.out is not None:
            out.out.cmd = self.frame if self.m.state == "TURN" else Hold()
            out.out.info["yaw_test"] = {"pair": self.p["pair"], "amp": self.p["amp"], "rate_dps": round(mean_rate(self.samples[-25:]), 2)}
        return out
