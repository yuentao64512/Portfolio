"""TEST_ORBIT (mission code m0t6): orbit a point - the Orbit behaviour of m1t1 on its own (v0.1.30, ROADMAP WP8c).

APPROACH -> ORBIT -> DONE. The centre: START_TASK args.names [NAME] (a saved point of usv_ui's points.json, as m0t2), or lat / lon
(the OCS map's click), or x_n / y_e (local metres). APPROACH = GUIDED to the point of the circle nearest the boat; ORBIT = the
Orbit behaviour (GUIDED carrots `la_m` ahead on the circle, done after `turns` x 360 deg of winding in the wanted sense).
Options: r_m (3), direction cw | ccw (cw), turns (1), speed_mps (0.5); profile section `orbit:`. --step / --from APPROACH | ORBIT.
"""
from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, Optional, Tuple

from ...companion.fsm import Transition
from ...companion.state import RobotXTask
from ..behaviours import Goto, Orbit
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"r_m": 3.0, "direction": "cw", "turns": 1.0, "speed_mps": 0.5, "la_m": 2.0, "band_m": 2.5}


class TestOrbit(Task):
    name = "TEST_ORBIT"
    robo_task = RobotXTask.NONE
    BEHAVIOURS = {"APPROACH": Goto, "ORBIT": Orbit}
    UNDERWAY = True        # v0.1.85: the morning's water test W0 (1) - the law round a circle the hull can hold
    ENTRY_POINTS = {"APPROACH": "a centre", "ORBIT": "the boat within band_m of the circle (start the orbit where it is)"}

    def build_machine(self, on_transition, clock):
        return simple_machine("TEST_ORBIT", ["APPROACH", "ORBIT"], [
            Transition("ON_CIRCLE", ["APPROACH"], "ORBIT", reason="on the circle"),
            Transition("CIRCLED", ["ORBIT"], "DONE", reason="orbit done"),
        ], initial="APPROACH", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update(self.cfg.extra.get("orbit", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        p["direction"] = str(p["direction"]).lower()
        self.p = p

    def points_file(self) -> str:
        return str((self.cfg.extra.get("test_path", {}) or {}).get("points_file", "usv_ui/points.json"))

    def centre(self, ctx: TaskContext) -> Tuple[Optional[Tuple[float, float]], str]:
        """((x_n, y_e), '') or (None, why)."""
        a, o = self.args, ctx.b.origin
        if a.get("x_n") is not None and a.get("y_e") is not None:
            return (float(a["x_n"]), float(a["y_e"])), ""
        if o is None:
            return None, "no control origin yet (no GPS fix)"
        if a.get("lat") is not None and a.get("lon") is not None:
            return o.to_local(float(a["lat"]), float(a["lon"])), ""
        names = a.get("names") or []
        if names:
            path = self.points_file()
            try:
                data = json.load(open(path)) if os.path.exists(path) else {}
            except ValueError:
                data = {}
            p = data.get(names[0])
            if p is None:
                return None, f"point {names[0]} not in {path} (saved: {', '.join(sorted(data)) or 'none'})"
            return o.to_local(float(p["lat"]), float(p["lon"])), ""
        return None, "no centre: --centre NAME (a saved point), or --lat / --lon (the map)"

    def precheck(self, ctx: TaskContext) -> str:
        if self.p["direction"] not in ("cw", "ccw"):
            return f"direction {self.p['direction']!r}: cw | ccw"
        if not (0.5 <= float(self.p["r_m"]) <= 50.0):
            return f"radius {self.p['r_m']} m out of 0.5..50"
        c, why = self.centre(ctx)
        return "" if c is not None else why

    def check_entry(self, stage: str, ctx: TaskContext) -> str:
        c, why = self.centre(ctx)
        if c is None:
            return why
        if stage == "ORBIT":
            pose = ctx.b.pose
            if pose is None:
                return "no position yet"
            d = abs(math.hypot(pose.x_n - c[0], pose.y_e - c[1]) - float(self.p["r_m"]))
            if d > float(self.p["band_m"]):
                return f"the boat is {d:.1f} m off the circle (band {self.p['band_m']} m): start at APPROACH"
        return ""

    def behaviour_args(self, sub: str, ctx: TaskContext) -> Dict[str, Any]:
        c, why = self.centre(ctx)
        if c is None:
            raise ValueError(why)
        r = float(self.p["r_m"])
        if sub == "APPROACH":
            pose = ctx.b.pose
            if pose is None:
                ang = 0.0
            else:
                ang = math.atan2(pose.y_e - c[1], pose.x_n - c[0])
            return {"x_n": c[0] + r * math.cos(ang), "y_e": c[1] + r * math.sin(ang), "speed_mps": float(self.p["speed_mps"]),
                    "arrive_r_m": 1.0}
        return {"x_n": c[0], "y_e": c[1], "r_m": r, "direction": self.p["direction"], "turns": float(self.p["turns"]),
                "speed_mps": float(self.p["speed_mps"]), "la_m": float(self.p["la_m"]), "band_m": float(self.p["band_m"])}

    def on_behaviour_done(self, sub: str, ctx: TaskContext) -> None:
        self.m.handle("ON_CIRCLE" if sub == "APPROACH" else "CIRCLED")
