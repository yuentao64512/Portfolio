"""Goto(point, speed): one GUIDED target, arrive or settle, timeout (from pool_test2.goto).

v0.1.54: `via` (optional local points): GUIDED to each in turn (the next one within `via_r_m`), then to the point - the corners of the
way round the mapped course objects (gatepath.via_points, the 0.60 m clearance rule: T4's transit)."""
from __future__ import annotations

from typing import Any, Dict, Optional

from ...companion.frames import dist_m
from ...companion.state import GuidedTarget
from .base import Behaviour, BehaviourContext, BehaviourOut


class Goto(Behaviour):
    name = "goto"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        lim = ctx.cfg.limits
        if "lat" in args and "lon" in args:
            self.lat, self.lon = float(args["lat"]), float(args["lon"])
            if ctx.origin is not None:
                self.target_local = ctx.origin.to_local(self.lat, self.lon)
            else:
                self.target_local = None
        elif "x_n" in args and "y_e" in args and ctx.origin is not None:
            self.target_local = (float(args["x_n"]), float(args["y_e"]))
            self.lat, self.lon = ctx.origin.from_local(*self.target_local)
        else:
            raise ValueError("goto needs lat/lon or x_n/y_e with an origin")
        self.speed = args.get("speed_mps", None)
        self.arrive_r = float(args.get("arrive_r_m", lim.goto_arrive_r_m))
        self.settle_r = float(args.get("settle_r_m", lim.goto_settle_r_m))
        self.settle_gs = float(args.get("settle_gs_mps", lim.goto_settle_gs_mps))
        self.settle_s = float(args.get("settle_s", lim.goto_settle_s))
        self.timeout_s = float(args.get("timeout_s", lim.goto_timeout_s))
        self._settle_since: Optional[float] = None
        self.via = [tuple(p) for p in (args.get("via") or [])] if ctx.origin is not None else []
        self.via_r = float(args.get("via_r_m", 1.5))
        from .. import guidance as GD                     # v0.1.85 (WP41 T0.2e): closest-approach arrival under way
        self.arrival = GD.Arrival(self.arrive_r, float(GD.params(ctx.cfg)["arrive_extra_m"])) if GD.underway(ctx.cfg) else None

    def distance(self, ctx: BehaviourContext) -> Optional[float]:
        if ctx.pose is None or self.target_local is None:
            return None
        return dist_m((ctx.pose.x_n, ctx.pose.y_e), self.target_local)

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        d = self.distance(ctx)
        while self.via and ctx.pose is not None and dist_m((ctx.pose.x_n, ctx.pose.y_e), self.via[0]) <= self.via_r:
            self.via.pop(0)
        if self.via and ctx.origin is not None:
            vlat, vlon = ctx.origin.from_local(*self.via[0])
            out = BehaviourOut(cmd=GuidedTarget(vlat, vlon, self.speed), info={"dist_m": d, "via": len(self.via)})
            if ctx.now - self.t_start > self.timeout_s:
                out.abort = f"goto timeout after {self.timeout_s:.0f} s (dist {d})"
            return out
        out = BehaviourOut(cmd=GuidedTarget(self.lat, self.lon, self.speed), info={"dist_m": d})
        if ctx.now - self.t_start > self.timeout_s:
            out.abort = f"goto timeout after {self.timeout_s:.0f} s (dist {d})"
            return out
        if d is None:
            return out
        if d <= self.arrive_r or (self.arrival is not None and self.arrival(d)):
            out.done = True
            return out
        gs = ctx.pose.sog_mps if ctx.pose else 1e9
        if d <= self.settle_r and gs < self.settle_gs:
            self._settle_since = self._settle_since or ctx.now
            if ctx.now - self._settle_since >= self.settle_s:
                out.done = True
        else:
            self._settle_since = None
        return out
