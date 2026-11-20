"""StationKeep: hold position (HOLD mode via the adapter); optional max duration. v0.1.81 (WP37): LOITER (a real position hold) when the
station keep in force is loiter (`station_keep hold|loiter`, the profile's station_keep.mode; hold by default)."""
from __future__ import annotations

from typing import Any, Dict

from ...companion.state import Hold, Loiter, station_keep_mode
from .base import Behaviour, BehaviourContext, BehaviourOut


class StationKeep(Behaviour):
    name = "station_keep"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        super().start(args, ctx)
        self.duration_s = args.get("duration_s")

    def step(self, ctx: BehaviourContext) -> BehaviourOut:
        out = BehaviourOut(cmd=Loiter() if station_keep_mode(ctx.cfg) == "loiter" else Hold())
        if self.duration_s is not None and ctx.now - self.t_start >= float(self.duration_s):
            out.done = True
        return out
