"""FollowPath(points, speed): GUIDED carrots along a local-frame polyline (usv_ui `pathto` in the task layer, ROADMAP WP2).

Progress is the nearest path point in a window ahead of the last one (monotonic); the carrot is the first point `la_m`
further along; within `end_r_m` of the last point the behaviour is done. Aborts on cross-track > corridor or timeout.
The adapter sends the carrot at most at 2 Hz (RoverAdapter.guided_goto). Decided 2026-11-02: TRANSIT by GUIDED carrots.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Tuple

from ...companion.state import GuidedTarget, Hold
from .base import Behaviour, BehaviourContext, BehaviourOut


class FollowPath(Behaviour):
    name = "follow_path"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        self.pts: List[Tuple[float, float]] = [tuple(p) for p in args["points"]]
        if len(self.pts) < 2:
            raise ValueError("follow_path needs at least two points")
        self.s = [0.0]
        for a, b in zip(self.pts, self.pts[1:]):
            self.s.append(self.s[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.speed = args.get("speed_mps")
        self.la = float(args.get("la_m", 2.0))
        self.end_r = float(args.get("end_r_m", 1.0))
        self.corridor = float(args.get("corridor_m", 4.0))
        self.timeout_s = float(args.get("timeout_s", max(30.0, 4.0 * self.s[-1] / max(0.2, float(self.speed or 0.5)) + 20.0)))
        self.idx = 0
        self.xte = 0.0
        self.carrot = self.pts[0]
        from .. import guidance as GD                     # v0.1.85 (WP41 T0.2d / e): LOS look-ahead along the polyline, closest-approach arrival
        self.uw = GD.underway(ctx.cfg)
        if self.uw:
            g = GD.params(ctx.cfg)
            self.la = max(self.la, float(g["los_m"]))
            self.arrival = GD.Arrival(self.end_r, float(g["arrive_extra_m"]))
            rt = GD.turn_radius(g, float(g["v_min_mps"]))       # the hull's turning circle: the corridor and the time allow for one turn round
            self.corridor = max(self.corridor, float(args.get("corridor_m", 4.0)) + rt)
            if "timeout_s" not in args:
                self.timeout_s += 360.0 / max(0.5, GD.hull_rate_dps(g, float(g["v_min_mps"])))

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        info = {"path": self.pts, "idx": self.idx}
        if ctx.pose is None or ctx.origin is None:
            return BehaviourOut(cmd=Hold(), info=info)
        if ctx.now - self.t_start > self.timeout_s:
            return BehaviourOut(cmd=Hold(), abort=f"follow_path timeout after {self.timeout_s:.0f} s", info=info)
        p = (ctx.pose.x_n, ctx.pose.y_e)
        n = len(self.pts)
        i1 = min(n - 1, self.idx + 40)
        best = min(range(self.idx, i1 + 1), key=lambda i: math.hypot(p[0] - self.pts[i][0], p[1] - self.pts[i][1]))
        self.idx = best
        j, k = min(best + 1, n - 1), best if best + 1 < n else best - 1
        a, b = self.pts[k], self.pts[j]
        dl = math.hypot(b[0] - a[0], b[1] - a[1])
        self.xte = ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / dl if dl > 1e-6 else 0.0
        info.update(idx=best, xte_m=round(self.xte, 2), remaining_m=round(self.s[-1] - self.s[best], 1))
        d_end = math.hypot(p[0] - self.pts[-1][0], p[1] - self.pts[-1][1])
        near_end = self.uw and self.s[-1] - self.s[best] <= self.end_r + 2.0 and self.arrival(d_end)
        if near_end or d_end <= self.end_r or (best >= n - 2 and d_end <= 2 * self.end_r):
            lat, lon = ctx.origin.from_local(*self.pts[-1])
            return BehaviourOut(cmd=GuidedTarget(lat, lon, self.speed), done=True, info=info)
        if abs(self.xte) > self.corridor:
            return BehaviourOut(cmd=Hold(), abort=f"follow_path off the path: xte {self.xte:+.1f} m > {self.corridor:.1f} m", info=info)
        ci = best
        while ci < n - 1 and self.s[ci] < self.s[best] + self.la:
            ci += 1
        self.carrot = self.pts[ci]
        info["carrot"] = self.carrot
        lat, lon = ctx.origin.from_local(*self.carrot)
        return BehaviourOut(cmd=GuidedTarget(lat, lon, self.speed), info=info)
