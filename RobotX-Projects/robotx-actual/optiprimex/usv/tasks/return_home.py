"""RETURN_HOME: the go-home override's task (v0.1.30, ROADMAP WP8c). GO -> DONE: one slow GUIDED leg to HOME, then AUTO_HOLD.

Started only by the HOME command (the OCS's master switch / `home`), never by START_TASK: the command handler aborts whatever runs and
starts this task without a run (decided 2026-11-04: any AUTO state, with or without a run; MANUAL refuses). HOME = the saved point
`HOME` in usv_ui's points.json, else the control origin - the handler resolves it and passes lat / lon. Our own task, not ArduPilot's
RTL (that uses the arming position and RTL_SPEED, a parameter). Decided: 0.3 m/s (home.speed_mps), arrive within 1.5 m, stop and
hold. A soft fault holds it in AUTO_SAFE_HOLD like any task (the transmitter is the backup).
"""
from __future__ import annotations

import math
from typing import Any, Dict

from ...companion.fsm import Transition
from ...companion.state import RobotXTask
from ..behaviours import Goto
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"speed_mps": 0.3, "arrive_r_m": 1.5}


class ReturnHome(Task):
    name = "RETURN_HOME"
    robo_task = RobotXTask.NONE
    BEHAVIOURS = {"GO": Goto}

    def build_machine(self, on_transition, clock):
        return simple_machine("RETURN_HOME", ["GO"], [Transition("ARRIVED", ["GO"], "DONE", reason="home reached")],
                              initial="GO", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update(self.cfg.extra.get("home", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        self.p = p

    def behaviour_args(self, sub: str, ctx: TaskContext) -> Dict[str, Any]:
        a = {"lat": float(self.args["lat"]), "lon": float(self.args["lon"]), "speed_mps": float(self.p["speed_mps"]),
             "arrive_r_m": float(self.p["arrive_r_m"]), "settle_r_m": float(self.p["arrive_r_m"]) + 1.0}
        o, pose = ctx.b.origin, ctx.b.pose
        if o is not None and pose is not None:                       # slow and maybe far: the timeout from the distance
            x, y = o.to_local(a["lat"], a["lon"])
            a["timeout_s"] = 60.0 + 2.5 * math.hypot(x - pose.x_n, y - pose.y_e) / max(0.1, a["speed_mps"])
        return a

    def on_behaviour_done(self, sub: str, ctx: TaskContext) -> None:
        self.m.handle("ARRIVED")
