"""m0t8 TEST_DETECT (v0.1.79, Dmitrii 2026-11-11: "test missions to check the buoy detection on the ground first"): the boat stands still
(StationKeep = HOLD: on land, props off) for `duration_s` while every buoy the camera puts into the map is logged - its range and bearing
now, sightings, the nearest and the farthest range it was seen at, and **the range at which it was last seen** (walk a buoy away from the
bow: the `lost` line says where it disappeared). One `detect` event per change (the OCS LOG tab), `<log_dir>/detect_<stamp>.jsonl`, a summary
at the end. No motion, no run needed (a test task)."""
from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Dict

from ...companion.fsm import Transition
from ...companion.state import RobotXTask, utc_iso
from ..behaviours import StationKeep
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"duration_s": 600.0, "lost_s": 3.0, "min_obs": 2, "log_dir": "logs", "every_m": 1.0, "chain_m": 2.5}


class TestDetect(Task):
    name = "TEST_DETECT"
    robo_task = RobotXTask.NONE
    BEHAVIOURS = {"WATCH": StationKeep}
    SKIP_REFUSE = {"WATCH": "a timer"}

    def build_machine(self, on_transition, clock):
        return simple_machine("TEST_DETECT", ["WATCH"], [Transition("TIME", ["WATCH"], "DONE", reason="duration over")],
                              initial="WATCH", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update((self.cfg.extra or {}).get("test_detect", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        self.p = p
        self.t0 = ctx.b.now
        self.seen: Dict[int, Dict[str, Any]] = {}
        self.alias: Dict[int, int] = {}                 # a map track -> the buoy record it continues
        self.lines = []
        self.log_path = os.path.join(str(p["log_dir"]), time.strftime("detect_%Y%m%d_%H%M%S.jsonl"))
        ctx.log(f"TEST_DETECT: standing still {p['duration_s']:.0f} s - walk a buoy toward / away from the bow; each buoy's last-seen range "
                f"is logged when it is gone {p['lost_s']:.0f} s")

    def _write(self, ctx: TaskContext, line: Dict[str, Any]) -> None:
        line = dict(line, t=utc_iso())
        self.lines.append(line)
        try:
            os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(line) + "\n")
        except OSError as ex:
            ctx.log(f"TEST_DETECT log write failed: {ex}")
        self.report(ctx, "detect", line)

    def watch(self, ctx: TaskContext) -> None:
        pose, m, now = ctx.b.pose, ctx.b.map, ctx.b.now
        if pose is None or m is None:
            return
        for e in m.entities("buoy", include_stale=True, include_young=True):
            if e.n_obs < int(self.p["min_obs"]):
                continue
            r = getattr(e, "last_range_m", None)        # the raw sighting (the map's position is an average that lags a moving buoy)
            b = getattr(e, "last_bearing_deg", None)
            if r is None:
                r = math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e)
                b = (math.degrees(math.atan2(e.y_e - pose.y_e, e.x_n - pose.x_n)) - pose.hdg_deg + 180.0) % 360.0 - 180.0
            xy = (pose.x_n + r * math.cos(math.radians(pose.hdg_deg + b)), pose.y_e + r * math.sin(math.radians(pose.hdg_deg + b)))
            s = self.seen.get(self.alias.get(e.id, e.id))
            fresh = e.t_last >= now - float(self.p["lost_s"])
            if s is None:                               # a new track where a recent one was last seen = the same buoy walked on (the map
                prev = min((v for v in self.seen.values() if not v["lost"] and e.t_last - v["t_last"] <= float(self.p["lost_s"])
                            and math.hypot(xy[0] - v["xy"][0], xy[1] - v["xy"][1]) <= float(self.p["chain_m"])),
                           key=lambda v: math.hypot(xy[0] - v["xy"][0], xy[1] - v["xy"][1]), default=None)   # splits a moving buoy)
                if prev is not None:
                    self.alias[e.id] = prev["id"]
                    s = prev
            if s is None:
                s = self.seen[e.id] = {"id": e.id, "first_m": round(r, 1), "min_m": r, "max_m": r, "last_m": r, "t_last": e.t_last,
                                       "n": e.n_obs, "lost": False, "logged_m": r, "xy": xy}
                self._write(ctx, {"ev": "seen", "id": e.id, "range_m": round(r, 1), "bearing_deg": round(b), "n_obs": e.n_obs,
                                  "colour": e.colour})
                ctx.log(f"detect: #{e.id} seen at {r:.1f} m, {b:+.0f} deg ({e.n_obs} obs)")
                continue
            if e.t_last > s["t_last"]:                  # a new sighting: where it is now
                s.update(t_last=e.t_last, last_m=r, n=max(s["n"], e.n_obs), min_m=min(s["min_m"], r), max_m=max(s["max_m"], r), xy=xy)
                if s["lost"]:
                    s["lost"] = False
                    self._write(ctx, {"ev": "back", "id": s["id"], "range_m": round(r, 1), "bearing_deg": round(b)})
                elif abs(r - s["logged_m"]) >= float(self.p["every_m"]):
                    s["logged_m"] = r
                    self._write(ctx, {"ev": "range", "id": s["id"], "range_m": round(r, 1), "bearing_deg": round(b), "n_obs": e.n_obs})
            elif not fresh and not s["lost"] and now - s["t_last"] > float(self.p["lost_s"]):
                s["lost"] = True
                self._write(ctx, {"ev": "lost", "id": s["id"], "last_seen_m": round(s["last_m"], 1), "max_m": round(s["max_m"], 1),
                                  "min_m": round(s["min_m"], 1), "n_obs": s["n"]})
                ctx.log(f"detect: #{s['id']} LOST - last seen at {s['last_m']:.1f} m (farthest {s['max_m']:.1f} m, {s['n']} obs)")

    def step(self, ctx: TaskContext):
        if not self.holding and self.m.state == "WATCH":
            self.watch(ctx)
            if ctx.b.now - self.t0 >= float(self.p["duration_s"]):
                self.finish(ctx)
                if self._beh is not None:
                    self._beh.cancel()
                    self._beh = None
                self.m.handle("TIME")
        out = super().step(ctx)
        if out.out is not None:
            out.out.info["detect"] = {k: {"last_m": round(v["last_m"], 1), "max_m": round(v["max_m"], 1), "lost": v["lost"]}
                                      for k, v in self.seen.items()}
        return out

    def finish(self, ctx: TaskContext) -> None:
        self._write(ctx, {"ev": "summary", "buoys": len(self.seen), "duration_s": round(ctx.b.now - self.t0, 1),
                          "per_buoy": [{"id": k, "first_m": v["first_m"], "min_m": round(v["min_m"], 1), "max_m": round(v["max_m"], 1),
                                        "last_seen_m": round(v["last_m"], 1), "lost": v["lost"]} for k, v in sorted(self.seen.items())]})

    def abort(self, reason: str) -> None:
        super().abort(reason)
