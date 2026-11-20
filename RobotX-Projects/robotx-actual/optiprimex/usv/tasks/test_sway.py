"""TEST_SWAY (mission code m0t3): the pool-trial sway leg through the final architecture (ROADMAP WP3; usv_ui `swayto`).

LINE -> TO_START (FollowPath to A) -> SWAY (SwayTransit A -> B, mode 20) -> DONE. The line: START_TASK args.points (two
[lat, lon] / {lat, lon}), args.names (two saved points of `test_path.points_file`), else the dock line from the map: from the
boat's point on the standoff line (`t3.standoff_m`, v0.1.56: derived from the dock geometry unless set) to the farthest bay abeam, bow to the dock. With points
or names the starboard side faces B (usv_ui swayto); `args.dir: -1` = port faces B.
"""
from __future__ import annotations

import json
import math
import os
from typing import Any, Dict

from ...companion.fsm import Transition
from ...companion.state import RobotXTask
from ..behaviours import FollowPath, SwayTransit
from ..dockpath import dock_geometry, standoff_lines
from ..gatepath import planner as gp
from .base import Task, TaskContext, simple_machine


class TestSway(Task):
    name = "TEST_SWAY"
    robo_task = RobotXTask.NONE
    BEHAVIOURS = {"TO_START": FollowPath, "SWAY": SwayTransit}

    def build_machine(self, on_transition, clock):
        return simple_machine("TEST_SWAY", ["LINE", "TO_START", "SWAY"], [
            Transition("PLANNED", ["LINE"], "TO_START", reason="line ready"),
            Transition("AT_START", ["TO_START"], "SWAY", reason="at A"),
            Transition("ARRIVED", ["SWAY"], "DONE", reason="at B"),
        ], initial="LINE", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        self.t_plan = ctx.b.now
        self.a = self.b = None
        self.dir = int((args or {}).get("dir", 1))
        self.sway_v = float((args or {}).get("sway_v", (self.cfg.extra.get("t3", {}) or {}).get("sway_v", 0.25)))
        self.source = ""

    def plan(self, ctx: TaskContext) -> str:
        o, pose = ctx.b.origin, ctx.b.pose
        if o is None or pose is None:
            return "no position yet"
        pts = self.args.get("points")
        names = self.args.get("names")
        if pts or names:
            if names:
                path = (self.cfg.extra.get("test_path", {}) or {}).get("points_file", "usv_ui/points.json")
                if not os.path.exists(path):
                    return f"points file {path} not found"
                data = json.load(open(path))
                missing = [n for n in names[:2] if n not in data]
                if missing:
                    return f"point {missing[0]} not in {path}"
                pts = [[data[n]["lat"], data[n]["lon"]] for n in names[:2]]
                self.source = f"{names[0]} -> {names[1]} from {path}"
            else:
                self.source = "two points from START_TASK"
            ll = [(q["lat"], q["lon"]) if isinstance(q, dict) else (q[0], q[1]) for q in pts[:2]]
            if len(ll) < 2:
                return "two points needed"
            self.a, self.b = o.to_local(*ll[0]), o.to_local(*ll[1])
        else:
            g = dock_geometry(ctx.b.map, pose, ctx.b.now)
            if g is None or len(g.bays) < 2:
                return "no points given and no dock line in the map"
            sd = standoff_lines(self.cfg.extra.get("t3"), self.cfg.extra.get("t1"))[0]          # v0.1.56: T3's line (derived or the override)
            here = (pose.x_n, pose.y_e)
            self.a = g.on_standoff(here, sd)
            far = max(g.bays, key=lambda bay: math.hypot(g.abeam(bay, sd)[0] - self.a[0], g.abeam(bay, sd)[1] - self.a[1]))
            self.b = g.abeam(far, sd)
            theta = math.degrees(math.atan2(self.b[1] - self.a[1], self.b[0] - self.a[0])) % 360.0
            err = ((theta - 90.0) - g.bow_in_hdg + 180.0) % 360.0 - 180.0
            self.dir = 1 if abs(err) < 90.0 else -1
            self.source = f"the dock line ({len(g.bays)} bays) at {sd:.0f} m, to bay {far.number}"
        if math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1]) < 1.0:
            return "A and B less than 1 m apart"
        return ""

    def step(self, ctx: TaskContext):
        if self.m.state == "LINE" and not self.holding:
            why = self.plan(ctx)
            if why and ctx.b.now - self.t_plan < 8.0:
                return super().step(ctx)
            if why:
                self.m.handle("ABORT", reason=f"TEST_SWAY: {why}")
            else:
                ctx.log(f"TEST_SWAY line from {self.source}: {math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1]):.1f} m, dir {self.dir:+d}")
                self.m.handle("PLANNED")
                pose = ctx.b.pose
                if math.hypot(self.a[0] - pose.x_n, self.a[1] - pose.y_e) < 1.5:
                    self.m.handle("AT_START", reason="already at A")
        return super().step(ctx)

    def behaviour_args(self, sub: str, ctx: TaskContext) -> Dict[str, Any]:
        if sub == "TO_START":
            pose = ctx.b.pose
            return {"points": gp.resample([(pose.x_n, pose.y_e), self.a], 0.5), "speed_mps": 0.5, "la_m": 1.5, "end_r_m": 0.7}
        return {"a": self.a, "b": self.b, "dir": self.dir, "v": self.sway_v, "corridor_m": 2.0, "end_r_m": 0.4}

    def on_behaviour_done(self, sub: str, ctx: TaskContext) -> None:
        self.m.handle("AT_START" if sub == "TO_START" else "ARRIVED")
