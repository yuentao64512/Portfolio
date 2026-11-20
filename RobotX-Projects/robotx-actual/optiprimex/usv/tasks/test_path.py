"""TEST_PATH (mission code m0t2): the pool-trial path leg through the final architecture (ROADMAP WP2).

PLAN -> FOLLOW -> DONE. PLAN takes, in this order: START_TASK args.points ([[lat, lon], ...] or [{lat, lon}, ...]);
args.names looked up in the points file (`test_path.points_file`, default usv_ui/points.json - usv_ui's saved points and
its `pathgen` output); else the mission manager's own map: every gate ahead (gatepath chain from the boat, pre / mid /
post each), entry and exit markers when present - the Proof of Readiness course (world_por) needs nothing else. FOLLOW is
the FollowPath behaviour (GUIDED carrots). No judge traffic; RobotXTask stays NONE.
"""
from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, List, Tuple

from ...companion.fsm import Transition
from ...companion.state import RobotXTask
from ..behaviours import FollowPath
from ..gatepath import planner as gp
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"points_file": "usv_ui/points.json", "speed_mps": 0.6, "la_m": 2.0, "corridor_m": 4.0, "gate_d_m": 2.0, "min_obs": 5,
            "end_r_m": 1.0, "plan_wait_s": 8.0}          # PLAN keeps trying this long (holding) before it gives up


class TestPath(Task):
    name = "TEST_PATH"
    robo_task = RobotXTask.NONE
    BEHAVIOURS = {"FOLLOW": FollowPath}

    def build_machine(self, on_transition, clock):
        return simple_machine("TEST_PATH", ["PLAN", "FOLLOW"], [
            Transition("PLANNED", ["PLAN"], "FOLLOW", reason="path ready"),
            Transition("ARRIVED", ["FOLLOW"], "DONE", reason="last point reached"),
            Transition("AGAIN", ["FOLLOW"], "PLAN", reason="leg done, the map may show more gates"),
            Transition("NOTHING_LEFT", ["PLAN"], "DONE", reason="no gate ahead any more"),
        ], initial="PLAN", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update(self.cfg.extra.get("test_path", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        self.p = p
        self.path: List[Tuple[float, float]] = []
        self.source = ""
        self.t_plan = ctx.b.now
        self.from_map = not (args or {}).get("points") and not (args or {}).get("names")
        self.done_mids: List[Tuple[float, float]] = []
        self.exit_done = False
        self.legs = 0

    def precheck(self, ctx: TaskContext) -> str:
        """v0.1.30 (WP8c, pool report F2 / F3): refuse at once what PLAN would give up on 8 s later - names not in the points
        file, or no names and no gate in the map. Without a position yet PLAN still waits (as before)."""
        names = self.args.get("names") or []
        if names:
            path = self.p["points_file"]
            if not os.path.exists(path):
                return f"points file {path} not found: save_point first"
            try:
                data = json.load(open(path))
            except ValueError:
                return f"points file {path} unreadable"
            missing = [n for n in names if n not in data]
            if missing:
                return f"point{'s' if len(missing) > 1 else ''} {', '.join(missing)} not in {path} (saved: {', '.join(sorted(data)) or 'none'})"
            return ""
        if self.args.get("points") or ctx.b.origin is None or ctx.b.pose is None or ctx.b.map is None:
            return ""
        why = self.plan(ctx)
        if why == "no points given and no gate in the map":
            return "no --names and no gate in the map: tick / give the saved points (--names A,B,...) or put a red + green gate in view"
        return ""

    def plan(self, ctx: TaskContext) -> str:
        o, pose = ctx.b.origin, ctx.b.pose
        if o is None or pose is None:
            return "no position / control origin yet"
        here = (pose.x_n, pose.y_e)
        ctrl: List[Tuple[float, float]] = []
        pts = self.args.get("points")
        if pts:
            for q in pts:
                lat, lon = (q["lat"], q["lon"]) if isinstance(q, dict) else (q[0], q[1])
                ctrl.append(o.to_local(float(lat), float(lon)))
            self.source = f"{len(ctrl)} points from START_TASK"
        elif self.args.get("names"):
            path = self.p["points_file"]
            if not os.path.exists(path):
                return f"points file {path} not found"
            data = json.load(open(path))
            for n in self.args["names"]:
                if n not in data:
                    return f"point {n} not in {path}"
                ctrl.append(o.to_local(float(data[n]["lat"]), float(data[n]["lon"])))
            self.source = f"{'-'.join(self.args['names'])} from {path}"
        else:
            m = ctx.b.map
            if m is None:
                return "no points given and no map"
            mk = m.markers()
            first = self.legs == 0
            cur, dir0 = (mk["entry"].x_n, mk["entry"].y_e) if (first and mk.get("entry")) else here, None
            if first and mk.get("entry"):
                ctrl.append(cur)
            items = [it for it in gp.gate_items(m, int(self.p["min_obs"]))
                     if not any(math.hypot(it.mid[0] - q[0], it.mid[1] - q[1]) < 1.5 for q in self.done_mids)]
            n_g = 0
            while items:
                ch = gp.choose_next(items, cur, dir0)
                items.remove(ch.item)
                if ch.wrong_side:
                    return f"gate {ch.item.key} would pass RED to port: refusing the plan"
                ctrl += gp.gate_points(ch.item, float(self.p["gate_d_m"]))
                self.done_mids.append(ch.item.mid)
                cur, dir0 = ctrl[-1], ch.item.dir
                n_g += 1
            if mk.get("exit") and not self.exit_done and not n_g:
                ex = (mk["exit"].x_n, mk["exit"].y_e)              # the exit once every gate is behind: stop 2.5 m short of the buoy
                u = gp._unit(ex[0] - cur[0], ex[1] - cur[1])
                obstacles = [(e.x_n, e.y_e) for e in m.entities("buoy") if e.n_obs >= int(self.p["min_obs"])]
                ctrl += gp.avoid([cur, (ex[0] - 2.5 * u[0], ex[1] - 2.5 * u[1])], obstacles)[1:]
                self.exit_done = True
            if not ctrl:
                return "nothing left" if not first else "no points given and no gate in the map"
            self.source = f"the map: {n_g} gate{'' if n_g == 1 else 's'}{', entry' if (first and mk.get('entry')) else ''}{', exit' if (self.exit_done and not n_g) else ''}"
        self.path = gp.resample([here] + ctrl, 0.5)
        if len(self.path) < 2:
            return "path too short"
        return ""

    def step(self, ctx: TaskContext):
        if self.m.state == "PLAN" and not self.holding:
            why = self.plan(ctx)
            if why == "nothing left":
                if ctx.b.now - self.t_plan < 2.0:
                    return super().step(ctx)                  # a short look for gates further on, then done
                self.m.handle("NOTHING_LEFT")
                return super().step(ctx)
            if why and ctx.b.now - self.t_plan < float(self.p["plan_wait_s"]):
                return super().step(ctx)                      # no behaviour in PLAN: the vehicle holds while the map fills
            if why:
                self.m.handle("ABORT", reason=f"TEST_PATH plan: {why}")
            else:
                ctx.log(f"TEST_PATH path from {self.source}: {len(self.path)} pts, {sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:])):.1f} m")
                self.m.handle("PLANNED")
        return super().step(ctx)

    def behaviour_args(self, sub: str, ctx: TaskContext) -> Dict[str, Any]:
        p = self.p
        return {"points": self.path, "speed_mps": p["speed_mps"], "la_m": p["la_m"], "corridor_m": p["corridor_m"], "end_r_m": p["end_r_m"]}

    def on_behaviour_done(self, sub: str, ctx: TaskContext) -> None:
        self.legs += 1
        if self.from_map:
            self.t_plan = ctx.b.now
            self.m.handle("AGAIN")                            # the map-made path is re-planned leg by leg
        else:
            self.m.handle("ARRIVED")
