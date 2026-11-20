"""SwayTransit(a, b, dir): sway along the line A -> B with the bow held orthogonal to it (usv_ui `swayto` in the task layer,
ROADMAP WP3). Mode-20 frames: heading hold (yaw), cross-track along the bow axis held by surge, the sway command tapered
over the last 2 m; dir +1 = starboard faces B (the bow points to the left of A -> B), -1 = port faces B. Phases: align
(heading within 10 deg for 1 s, 60 s max) then sway; aborts on cross-track > corridor or heading error > 60 deg.
"""
from __future__ import annotations

import math
from typing import Any, Dict

from ...companion.state import ThrusterFrame
from .base import Behaviour, BehaviourContext, BehaviourOut
from .mode20 import frame, wrap180, yaw_cmd


class SwayTransit(Behaviour):
    name = "sway_transit"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        self.a = tuple(args["a"])
        self.b = tuple(args["b"])
        self.dir = 1 if int(args.get("dir", 1)) >= 0 else -1
        self.v = float(args.get("v", 0.25))
        self.corridor = float(args.get("corridor_m", 3.0))
        self.end_r = float(args.get("end_r_m", 0.5))
        dn, de = self.b[0] - self.a[0], self.b[1] - self.a[1]
        self.length = math.hypot(dn, de)
        self.theta = math.degrees(math.atan2(de, dn)) % 360.0
        self.psi = (self.theta - 90.0 * self.dir) % 360.0
        self.phase = "align"
        self.t_phase = ctx.now
        self.aligned_since = None
        self.zero_until = ctx.now + 0.4                       # the firmware's zero-frame rule on entering mode 20

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        info = {"phase": self.phase, "path": [self.a, self.b], "hold_hdg": round(self.psi, 1)}
        if ctx.pose is None:
            return BehaviourOut(cmd=ThrusterFrame(), info=info)
        th = math.radians(self.theta)
        dn, de = math.cos(th), math.sin(th)
        bn, be = self.dir * math.sin(th), -self.dir * math.cos(th)    # bow direction
        pn, pe = ctx.pose.x_n - self.a[0], ctx.pose.y_e - self.a[1]
        s = pn * dn + pe * de
        c = pn * bn + pe * be                                  # + = ahead of the line (bow side)
        e = wrap180(self.psi - ctx.pose.hdg_deg)
        info.update(s_m=round(s, 2), c_m=round(c, 2), e_deg=round(e, 1), remaining_m=round(self.length - s, 2))
        g, amp = ctx.cfg.gains, ctx.cfg.limits.max_amp
        if abs(c) > self.corridor:
            return BehaviourOut(cmd=ThrusterFrame(), abort=f"sway: cross-track {c:+.1f} m > corridor {self.corridor:.1f} m", info=info)
        r = yaw_cmd(g, e)
        u = max(-g.sway_umax, min(g.sway_umax, -g.sway_kc * c))
        v = 0.0
        if self.phase == "align":
            if abs(e) < 10.0:
                self.aligned_since = self.aligned_since if self.aligned_since is not None else ctx.now
                if ctx.now - self.aligned_since > 1.0:
                    self.phase, self.t_phase = "sway", ctx.now
            else:
                self.aligned_since = None
            if ctx.now - self.t_phase > 60.0:
                return BehaviourOut(cmd=ThrusterFrame(), abort=f"sway: align timeout, heading error {e:.0f} deg", info=info)
        else:
            remaining = self.length - s
            if remaining <= self.end_r:
                return BehaviourOut(cmd=ThrusterFrame(), done=True, info=info)
            if abs(e) > 60.0 and ctx.now - self.t_phase > 5.0:
                return BehaviourOut(cmd=ThrusterFrame(), abort=f"sway: heading error {e:.0f} deg", info=info)
            if ctx.now - self.t_phase > 30.0 + 6.0 * self.length / max(0.05, self.v):
                return BehaviourOut(cmd=ThrusterFrame(), abort="sway: timeout", info=info)
            v = self.v * max(0.3, min(1.0, remaining / 2.0)) * self.dir
        if ctx.now < self.zero_until:
            return BehaviourOut(cmd=ThrusterFrame(), info=info)
        info["tau"] = {"surge": round(u, 3), "sway": round(v, 3), "yaw": round(r, 3)}   # v0.1.55 (D1): the commanded frame for mm/state cmd
        return BehaviourOut(cmd=frame(g, amp, u, v, r), info=info)
