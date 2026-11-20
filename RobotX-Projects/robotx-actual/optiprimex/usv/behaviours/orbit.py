"""Orbit(centre, r, direction): one full circle round a point by GUIDED carrots (ROADMAP WP2; spec 6.4 circleWps / 6.7).

The carrot sits on the circle `la_m` of arc ahead of the boat's angle about the centre, in the circle's sense (cw =
compass bearing increasing). Winding = the boat's angle about the centre, accumulated while it is within `band_m` of the
circle (spec 6.7); done (latched) once the winding in the wanted sense reaches `turns` x 360 deg. Decided 2026-11-02:
r 3 m at 0.5 m/s (t1.circle_r_m, t1.circle_v).

v0.1.54: `approach` (optional points): first along them (the carrot `la_m` ahead on the polyline, as FollowPath) until the boat is
within 1 m of the circle - T1's path to the ring round the other course objects (the clearance rule) - then the circle as before.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

from ...companion.state import GuidedTarget, Hold
from .base import Behaviour, BehaviourContext, BehaviourOut


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Orbit(Behaviour):
    name = "orbit"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        self.c = (float(args["x_n"]), float(args["y_e"]))
        self.r = float(args.get("r_m", 3.0))
        self.sign = 1.0 if str(args.get("direction", "cw")).lower() == "cw" else -1.0
        self.speed = args.get("speed_mps", 0.5)
        self.approach_speed = args.get("approach_speed", self.speed)   # v0.1.92: ALIGN_ORBIT - to the ring faster than round it
        self.la = float(args.get("la_m", 2.0))
        self.v_max = float(args.get("speed_max") or 0.0)   # v0.1.94.2 (Dmitrii: "in slow mode we might drift away"): the drift boost's top
        self.boost, self.t_prev, self.xy_prev, self.v_gnd = 0.0, None, None, None
        self.band = float(args.get("band_m", 2.5))
        self.turns = float(args.get("turns", 1.0))
        ex = args.get("exit_toward")                  # v0.1.92: after the turns, circle on until the bow points here (<= one more turn)
        self.exit_to = (float(ex[0]), float(ex[1])) if ex else None
        self.exit_tol = float(args.get("exit_tol_deg", 25.0))
        from .. import guidance as GD                     # v0.1.85 (WP41): under way the circle is one the hull can hold - r >= v / omega(v)
        if GD.underway(ctx.cfg):
            g = GD.params(ctx.cfg)
            self.speed = min(float(self.speed or g["scan_v_mps"]), float(g["scan_v_mps"]))
            self.v_max = min(self.v_max, float(g["scan_v_mps"]))
            self.r = max(self.r, 1.1 * GD.turn_radius(g, max(self.speed, self.v_max) if args.get("r_for_max") else self.speed))
            self.la = max(self.la, 0.6 * self.r)
            self.band = max(self.band, 0.5 * self.r)
        self.timeout_s = float(args.get("timeout_s", 30.0 + 3.0 * 2 * math.pi * self.r * (self.turns + (1.0 if args.get("exit_toward") else 0.0))
                                        / max(0.2, float(self.speed or 0.5))))
        self.winding = 0.0
        self.last_a: Optional[float] = None
        self.approach = [tuple(p) for p in (args.get("approach") or [])]
        self.ai = 0

    def angle(self, x: float, y: float) -> float:
        return math.atan2(y - self.c[1], x - self.c[0])

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        info = {"centre": self.c, "r_m": self.r, "winding_deg": round(math.degrees(self.sign * self.winding), 0),
                "path": [(self.c[0] + self.r * math.cos(k * math.pi / 18), self.c[1] + self.r * math.sin(k * math.pi / 18)) for k in range(37)]}
        if ctx.pose is None or ctx.origin is None:
            return BehaviourOut(cmd=Hold(), info=info)
        if ctx.now - self.t_start > self.timeout_s:
            return BehaviourOut(cmd=Hold(), abort=f"orbit timeout ({math.degrees(self.sign * self.winding):.0f} of {360 * self.turns:.0f} deg)", info=info)
        x, y = ctx.pose.x_n, ctx.pose.y_e
        a = self.angle(x, y)
        rho = math.hypot(x - self.c[0], y - self.c[1])
        if self.approach:
            if abs(rho - self.r) < 1.0 or self.ai >= len(self.approach) - 1:
                self.approach = []                      # on the ring: circle from here
            else:
                n = len(self.approach)
                self.ai = min(range(self.ai, min(n, self.ai + 40)), key=lambda i: math.hypot(x - self.approach[i][0], y - self.approach[i][1]))
                ci, s = self.ai, 0.0
                while ci < n - 1 and s < self.la:
                    s += math.hypot(self.approach[ci + 1][0] - self.approach[ci][0], self.approach[ci + 1][1] - self.approach[ci][1])
                    ci += 1
                info["carrot"] = self.approach[ci]
                lat, lon = ctx.origin.from_local(*self.approach[ci])
                info["sub"] = "align orbit"
                return BehaviourOut(cmd=GuidedTarget(lat, lon, self.approach_speed), info=info)
        if self.last_a is not None and abs(rho - self.r) < self.band:
            self.winding += wrap_pi(a - self.last_a)
        self.last_a = a
        info["winding_deg"] = round(math.degrees(self.sign * self.winding), 0)
        if self.sign * self.winding >= 2 * math.pi * self.turns:
            if self.exit_to is None or self.sign * self.winding >= 2 * math.pi * (self.turns + 1.0):
                return BehaviourOut(cmd=Hold(), done=True, info=info)
            brg = math.degrees(math.atan2(self.exit_to[1] - y, self.exit_to[0] - x))
            if abs((brg - ctx.pose.hdg_deg + 180.0) % 360.0 - 180.0) <= self.exit_tol:
                info["exit"] = "on the tangent to the next leg"
                return BehaviourOut(cmd=Hold(), done=True, info=info)
        la = self.la if self.r >= 2.5 else min(self.la, max(1.3, 0.6 * self.r))   # v0.1.54: a small circle (the clearance rule) cuts less; >= 1.3 m: GUIDED stops within 1 m of its target
        ac = a + self.sign * la / self.r
        carrot = (self.c[0] + self.r * math.cos(ac), self.c[1] + self.r * math.sin(ac))
        info["carrot"] = carrot
        lat, lon = ctx.origin.from_local(*carrot)
        v = float(self.speed or 0.0)
        if self.v_max > v:                              # v0.1.94.2: the drift boost - over ground slower than 70 % of the asked speed
            dt = 0.0 if self.t_prev is None else max(0.0, min(1.0, ctx.now - self.t_prev))   # (a current, wind) -> up 0.05 m/s per s,
            if dt > 0 and self.xy_prev is not None:     # back down 0.03 m/s per s once it keeps up; never above speed_max. The speed over
                inst = math.hypot(x - self.xy_prev[0], y - self.xy_prev[1]) / dt      # ground from the pose itself (EMA ~2 s), not a SOG field
                self.v_gnd = inst if self.v_gnd is None else self.v_gnd + min(1.0, dt / 2.0) * (inst - self.v_gnd)
            if self.v_gnd is not None and ctx.now - self.t_start > 3.0:
                slow = self.v_gnd < 0.7 * (v + self.boost)
                self.boost = max(0.0, min(self.v_max - v, self.boost + (0.05 if slow else -0.03) * dt))
            v += self.boost
            info["v_mps"], info["boost_mps"] = round(v, 3), round(self.boost, 3)
        self.t_prev, self.xy_prev = ctx.now, (x, y)
        return BehaviourOut(cmd=GuidedTarget(lat, lon, v), info=info)
