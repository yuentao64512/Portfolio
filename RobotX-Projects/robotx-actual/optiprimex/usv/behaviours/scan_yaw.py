"""ScanYaw(until): hold position and turn on the spot until a predicate on the map is true (ROADMAP WP2).

GUIDED velocity target vx 0, yaw rate `rate_dps` (positive = clockwise). Done when `until(ctx)` is true; after `turns`
full turns without it the behaviour aborts with `reason` (the task decides: next stage, or a safe hold). Decided
2026-11-02: a missing gate = hold, then scan.

v0.1.39 (WP10h): `face=(x_n, y_e)` turns toward that point and holds facing it (yaw rate = clamp(1.5 x error)) instead of turning
round - the LOOK at a pending buoy; `max_s` then bounds it (abort with `reason`). The same GUIDED velocity target: no mode change.
"""
from __future__ import annotations

import math
from typing import Any, Callable, Dict

from ...companion.state import Hold, VelocityYawTarget
from .base import Behaviour, BehaviourContext, BehaviourOut


class ScanOrbit(Behaviour):
    """v0.1.85 (WP41 T0.3): the circle scan - forward at scan_v_mps 0.3 with the yaw rate at the cap (5 deg/s), the hull holding the circle it
    can (~5.4 m at 0.3 m/s on omni_usv), until `until(ctx)` or the heading has swept turns x 360 deg (abort with `reason`, as ScanYaw).
    The side (cw / ccw) is the one whose circle keeps clear of the mapped objects and inside the course polygon (args `obstacles`, `poly`)."""
    name = "scan_orbit"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        from .. import guidance as GD
        g = GD.params(ctx.cfg)
        self.until: Callable[[BehaviourContext], bool] = args["until"]
        self.turns = float(args.get("turns", 1.0))
        self.reason = str(args.get("reason", "scan found nothing"))
        self.v = float(g["scan_v_mps"])
        self.w = math.radians(float(g["yaw_rate_max_dps"]))
        self.R = max(float(g["scan_r_m"]), GD.turn_radius(g, self.v))
        self.max_s = float(args.get("max_s", g["scan_max_s"])) * self.turns
        self.swept, self.last_h, self.sign = 0.0, None, 1.0
        self.g, self.poly = g, args.get("poly")
        pose = ctx.pose
        if pose is not None:
            obst, poly = args.get("obstacles"), args.get("poly")
            if obst is None:
                from ..gatepath import planner as gp
                obst = gp.map_obstacles(ctx.map, 3)
            if poly is None:
                poly = GD.course_local(ctx.run, ctx.origin)

            def score(sign: float) -> float:
                h = math.radians(pose.hdg_deg + sign * 90.0)
                c = (pose.x_n + self.R * math.cos(h), pose.y_e + self.R * math.sin(h))
                ring = [(c[0] + self.R * math.cos(a * math.pi / 12), c[1] + self.R * math.sin(a * math.pi / 12)) for a in range(24)]
                clear = min([math.hypot(q[0] - o[0], q[1] - o[1]) - float(o[2] if len(o) > 2 else 0.25) for q in ring for o in obst] or [99.0])
                out = sum(1 for q in ring if poly and not GD.inside(q, poly))
                return clear - 5.0 * out
            self.sign = 1.0 if score(1.0) >= score(-1.0) else -1.0
            self.poly = poly

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        if self.until(ctx):
            return BehaviourOut(cmd=Hold(), done=True, info={"swept_deg": round(math.degrees(self.swept))})
        if ctx.pose is not None:
            h = math.radians(ctx.pose.hdg_deg)
            if self.last_h is not None:
                self.swept += abs(math.atan2(math.sin(h - self.last_h), math.cos(h - self.last_h)))
            self.last_h = h
        info = {"swept_deg": round(math.degrees(self.swept)), "scan": "orbit", "r_m": round(self.R, 1)}
        if self.swept >= 2 * math.pi * self.turns or ctx.now - self.t_start > self.max_s:
            return BehaviourOut(cmd=Hold(), abort=self.reason, info=info)
        v = self.v
        if ctx.pose is not None:                       # the under-way rules on the circle too: bow-clear (T0.2b), the boundary (T0.2c)
            from .. import guidance as GD
            from ..gatepath import planner as gp
            pos = (ctx.pose.x_n, ctx.pose.y_e)
            if GD.bow_blocked(pos, ctx.pose.hdg_deg, gp.map_obstacles(ctx.map, 3), self.g) is not None:
                v, info["why"] = 0.0, "bow-clear"
            ray = GD.ray_to_poly(pos, ctx.pose.hdg_deg, self.poly) if self.poly else None
            if self.poly and ray is None and not GD.inside(pos, self.poly):
                v, info["why"] = 0.0, "outside, heading away"
            elif ray is not None and ray < float(self.g["bnd_stop_m"]):
                v, info["why"] = 0.0, "boundary"
            elif ray is not None and ray < float(self.g["bnd_slow_m"]):
                v = min(v, float(self.g["bnd_slow_v_mps"]))
        return BehaviourOut(cmd=VelocityYawTarget(v, self.sign * self.w), info=info)


class ScanYaw(Behaviour):
    name = "scan_yaw"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        from .. import guidance as GD                 # v0.1.85: under the profile's scan_mode orbit the turn on the spot becomes ScanOrbit
        self._orbit = None
        if args.get("face") is None and not args.get("inplace") and GD.scan_mode(ctx.cfg) == "orbit":   # v0.1.92: inplace = on the spot
            self._orbit = ScanOrbit()
            self._orbit.start(args, ctx)
        self._ymax = math.radians(float(GD.params(ctx.cfg)["yaw_rate_max_dps"])) if GD.underway(ctx.cfg) else None
        self.until: Callable[[BehaviourContext], bool] = args["until"]
        self.rate = math.radians(float(args.get("rate_dps", 20.0)))
        self.turns = float(args.get("turns", 1.0))
        self.settle_s = float(args.get("settle_s", 1.0))
        self.reason = str(args.get("reason", "scan found nothing"))
        self.swept = 0.0
        self.last_h = None
        self.face = args.get("face")
        self.timeout_s = args.get("timeout_s")              # v0.1.92: the start's 360 on the spot (the hull turns ~2 deg/s: 180 s)
        self.max_s = float(args.get("max_s", 8.0))
        self.still_deg = float(args.get("still_deg") or 0.0)   # v0.1.94: facing within this -> yaw 0 (until 2x off): hold still to read a lamp
        self.still = False

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        if self._orbit is not None:
            return self._orbit.step(ctx)
        if self._ymax is not None and self.rate > self._ymax:
            self.rate = self._ymax                    # v0.1.85: under way every yaw command within yaw_rate_max_dps (the LOOK's face too)
        if self.until(ctx):
            return BehaviourOut(cmd=Hold(), done=True, info={"swept_deg": round(math.degrees(self.swept))})
        if ctx.pose is not None:
            h = math.radians(ctx.pose.hdg_deg)
            if self.last_h is not None:
                self.swept += abs(math.atan2(math.sin(h - self.last_h), math.cos(h - self.last_h)))
            self.last_h = h
        info = {"swept_deg": round(math.degrees(self.swept))}
        if self.face is not None:
            if ctx.now - self.t_start > self.max_s:
                return BehaviourOut(cmd=Hold(), abort=self.reason, info=info)
            if ctx.pose is None:
                return BehaviourOut(cmd=Hold(), info=info)
            want = math.atan2(self.face[1] - ctx.pose.y_e, self.face[0] - ctx.pose.x_n)
            err = math.atan2(math.sin(want - math.radians(ctx.pose.hdg_deg)), math.cos(want - math.radians(ctx.pose.hdg_deg)))
            if self.still_deg > 0:                      # v0.1.94 (13 Nov pm: CHECK_BLUE hunted round the target, the lamp never held)
                ed = abs(math.degrees(err))
                self.still = ed <= self.still_deg or (self.still and ed <= 2.0 * self.still_deg)
            yr = 0.0 if self.still else max(-self.rate, min(self.rate, 1.5 * err))
            return BehaviourOut(cmd=VelocityYawTarget(0.0, yr),
                                info=dict(info, face_err_deg=round(math.degrees(err)), still=self.still, hold_hdg=round(math.degrees(want) % 360.0, 1)))   # v0.1.55: the heading goal
        if self.swept >= 2 * math.pi * self.turns or ctx.now - self.t_start > (self.timeout_s or 2 * math.pi * self.turns / self.rate * 2.0 + 10.0):
            return BehaviourOut(cmd=Hold(), abort=self.reason, info=info)
        if ctx.now - self.t_start < self.settle_s:
            return BehaviourOut(cmd=Hold(), info=info)
        return BehaviourOut(cmd=VelocityYawTarget(0.0, self.rate), info=info)
