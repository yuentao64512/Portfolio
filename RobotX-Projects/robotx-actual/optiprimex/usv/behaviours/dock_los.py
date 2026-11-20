"""DockLos: the guarded bow-in into a dock bay and the way out (usv_ui `dockto` in the task layer, ROADMAP WP3).

mode "align"  turn on the spot to the bow-in heading (yaw only), done after 1 s within 8 deg
mode "enter"  surge toward the face along the bay centreline, heading held, lateral offset corrected by sway, the surge
              tapered with the distance; done at `stop_m` off the face (decided 2026-11-03: 1 m)
mode "astern" back out along the centreline until `out_m` off the face (default: the standoff, 5 m)
mode "out"    v0.1.56 (D6/D7): out along the dock normal from any heading (T3's TO_LINE when it starts inside the standoff line) -
              the velocity `v` away from the dock in the body frame (astern when the bow is to the dock), the bow turned to the dock;
              no heading abort; done at `out_m` off the face
Aborts: off the centreline by more than `corridor_m`, heading error > 30 deg, timeout. Mode-20 frames.
"""
from __future__ import annotations

import math
from typing import Any, Dict

from ...companion.state import ThrusterFrame
from .base import Behaviour, BehaviourContext, BehaviourOut
from .mode20 import frame, wrap180, yaw_cmd


class DockLos(Behaviour):
    name = "dock_los"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        self.mode = str(args.get("mode", "enter"))
        self.face = tuple(args["face"])
        self.n = tuple(args["normal"])                       # unit normal of the dock line, pointing to the water
        self.psi = (math.degrees(math.atan2(-self.n[1], -self.n[0]))) % 360.0     # bow to the dock
        self.stop_m = float(args.get("stop_m", 1.0))
        self.out_m = float(args.get("out_m", 5.0))
        self.v = float(args.get("v", 0.15))
        self.corridor = float(args.get("corridor_m", 1.2))
        self.timeout_s = float(args.get("timeout_s", 90.0))
        self.since = None
        self.zero_until = ctx.now + 0.4

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        info = {"mode": self.mode, "hold_hdg": round(self.psi, 1)}
        if ctx.pose is None:
            return BehaviourOut(cmd=ThrusterFrame(), info=info)
        nn, ne = self.n
        dn, de = ctx.pose.x_n - self.face[0], ctx.pose.y_e - self.face[1]
        out = dn * nn + de * ne                               # metres off the face along the normal
        across = dn * ne + de * (-nn)                         # + = starboard of the centreline when facing the dock
        e = wrap180(self.psi - ctx.pose.hdg_deg)
        info.update(out_m=round(out, 2), across_m=round(across, 2), e_deg=round(e, 1), path=[(ctx.pose.x_n, ctx.pose.y_e), self.face])
        g, amp = ctx.cfg.gains, ctx.cfg.limits.max_amp
        if ctx.now - self.t_start > self.timeout_s:
            return BehaviourOut(cmd=ThrusterFrame(), abort=f"dock {self.mode}: timeout", info=info)
        r = yaw_cmd(g, e)
        if self.mode == "align":
            if abs(e) < 8.0:
                self.since = self.since if self.since is not None else ctx.now
                if ctx.now - self.since > 1.0:
                    return BehaviourOut(cmd=ThrusterFrame(), done=True, info=info)
            else:
                self.since = None
            u = v = 0.0
        else:
            if abs(across) > self.corridor:
                return BehaviourOut(cmd=ThrusterFrame(), abort=f"dock {self.mode}: {across:+.1f} m off the bay centreline", info=info)
            if abs(e) > 30.0 and self.mode != "out":
                return BehaviourOut(cmd=ThrusterFrame(), abort=f"dock {self.mode}: heading error {e:.0f} deg", info=info)
            v = max(-0.2, min(0.2, -0.4 * across))
            if self.mode == "enter":
                d = out - self.stop_m
                if d <= 0.15:
                    return BehaviourOut(cmd=ThrusterFrame(), done=True, info=info)
                u = max(0.0, min(self.v, 0.1 * d + 0.03))
            elif self.mode == "out":                          # v0.1.56: away from the dock along the normal, whatever the heading
                if out >= self.out_m:
                    return BehaviourOut(cmd=ThrusterFrame(), done=True, info=info)
                h = math.radians(ctx.pose.hdg_deg)
                u = self.v * (nn * math.cos(h) + ne * math.sin(h))
                v = max(-0.2, min(0.2, self.v * (-nn * math.sin(h) + ne * math.cos(h)) - 0.4 * across * math.cos(math.radians(e))))
            else:                                             # astern
                if out >= self.out_m:
                    return BehaviourOut(cmd=ThrusterFrame(), done=True, info=info)
                u = -self.v
        if ctx.now < self.zero_until:
            return BehaviourOut(cmd=ThrusterFrame(), info=info)
        info["tau"] = {"surge": round(u, 3), "sway": round(v, 3), "yaw": round(r, 3)}   # v0.1.55 (D1): the commanded frame for mm/state cmd
        return BehaviourOut(cmd=frame(g, amp, u, v, r), info=info)
