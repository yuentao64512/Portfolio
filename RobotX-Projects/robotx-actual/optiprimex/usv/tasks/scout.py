"""Mission `scout` (v0.1.79, WP35; docs/SCOUT_BRIEF_2026-11-11.md §2-§3, docs/PROMPT_SCOUT_BUILD_2026-11-11.md §1): buoy to buoy,
colour-blind, every buoy logged - declared as Task 1 (core alone, advanced with the drone). t1_safe_passage.py is not touched.

WAIT_START -> [WAIT_UAV_MAP when the drone is declared, as m1t2] -> ADVANCE (forward along the heading inside the inset polygon - toward its
centre when the heading leaves it within 3 m - until the FIRST LOCK; Dmitrii 2026-11-11: the boat never stands still at the start) ->
PICK (nearest unvisited buoy by path length round the clearance rings) -> APPROACH (the stand-off point) -> LOG (station-keep log_s, the
record) -> PICK ...; the map exhausted -> SCAN (one slow turn) -> PICK, or nothing new -> FRONTIER (the next unswept lawnmower point) ->
SCAN; ADVANCE's far point with nothing locked -> SCAN; no frontier left or budget_s -> RETURN (to the start pose) -> DONE (AUTO_HOLD).

The field: the RxCourse boundary (lat / lon -> local), or args `polygon` (local [x_n, y_e]: the sim's world bounds), else a square of
`field_m` ahead of the start. The log: `<log_dir>/scout_<run_id>.jsonl` (buoy / obstacle / summary lines) + a `scout` event per line (the
OCS LOG tab) + a SafePassageReport whenever the mapped set changes (entry / exit empty, colours only when voted red).
The ZED guard (WP36) hooks in through on_guard(); until then it only logs.
"""
from __future__ import annotations

import json
import math
import os
from typing import Any, Dict, List, Optional, Tuple

from ...companion.fsm import Transition
from ...companion.state import GuidedTarget, RobotXTask, VelocityYawTarget, utc_iso
from ..behaviours import FollowPath, ScanYaw, StationKeep
from ..gatepath import planner as gp
from .. import scout_geom as G
from .. import guidance as GD
from . import uav_gate as UG  # noqa: E402  (v0.1.89: the touchdown sequence)
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"standoff_m": 3.0, "arrive_m": 1.0, "transit_v": 0.6, "scan_rate_dps": 20.0, "log_s": 4.0, "budget_s": 600.0, "margin_m": 2.0,
            "lane_m": 14.0, "coverage_r_m": 14.0, "visit_r_m": 2.0, "clear_m": 1.6, "min_obs": 3, "report_period_s": 2.0,
            "sig_use_m": 3.0, "track_m": 0.45, "hull_len_m": 1.19, "hull_beam_m": 0.76, "la_m": 2.0, "corridor_m": 4.0,
            "hfov_deg": 50.9, "field_m": 30.0, "frontier_seen": 0.8, "frontier_r_m": 5.0, "advance_min_m": 3.0,
            "max_yaw_dps": 10.0, "turn_gate_deg": 20.0, "turn_exit_deg": 8.0,
            "log_dir": "logs", "uav_wait_s": 420.0, "uav_quiet_s": 5.0, "tier": None, "polygon": None}
STATES = ["WAIT_START", "WAIT_UAV_MAP", "ADVANCE", "PICK", "APPROACH", "LOG", "SCAN", "FRONTIER", "RETURN"]


class Scout(Task):
    name = "SCOUT"
    robo_task = RobotXTask.T1
    BEHAVIOURS = {"WAIT_START": StationKeep, "WAIT_UAV_MAP": StationKeep, "ADVANCE": FollowPath, "APPROACH": FollowPath, "LOG": StationKeep,
                  "SCAN": ScanYaw, "FRONTIER": FollowPath, "RETURN": FollowPath}
    SKIP_REFUSE = {"WAIT_START": "a timer", "WAIT_UAV_MAP": "the UAV", "SCAN": "the map", "LOG": "a timer"}
    NEEDS_FIELD = True
    UNDERWAY = True                                     # v0.1.85 (WP41): under guidance.mode underway - the law, no turn-first gate, move-first

    def build_machine(self, on_transition, clock):
        return simple_machine("SCOUT", STATES, [
            Transition("GO", ["WAIT_START"], "ADVANCE", reason="started: forward until the first lock"),
            Transition("GO_UAV", ["WAIT_START"], "WAIT_UAV_MAP", reason="the drone is declared: its survey first (handbook 3.3.2 Advanced)"),
            Transition("UAV_MAP", ["WAIT_UAV_MAP"], "PICK", reason="the UAV's map is in"),
            Transition("LOCK", ["ADVANCE"], "PICK", reason="the first lock: a usable buoy in the map"),
            Transition("FAR", ["ADVANCE"], "SCAN", reason="the far point reached, nothing locked"),
            Transition("TARGET", ["PICK"], "APPROACH", reason="the nearest unvisited buoy"),
            Transition("NONE", ["PICK"], "SCAN", reason="the map is exhausted"),
            Transition("ARRIVED", ["APPROACH"], "LOG", reason="at the stand-off point"),
            Transition("REPICK", ["APPROACH"], "PICK", reason="the target is gone or blocked"),
            Transition("LOGGED", ["LOG"], "PICK", reason="the buoy logged"),
            Transition("BUOYS", ["SCAN"], "PICK", reason="a new buoy in the map"),
            Transition("NOTHING", ["SCAN"], "FRONTIER", reason="nothing new: the next unswept lawnmower point"),
            Transition("AT_POINT", ["FRONTIER"], "SCAN", reason="at the frontier point"),
            Transition("SEEN", ["FRONTIER"], "PICK", reason="a new buoy on the way"),
            Transition("HOME", ["ADVANCE", "PICK", "APPROACH", "LOG", "SCAN", "FRONTIER"], "RETURN", reason="the field is done (or the budget)"),
            Transition("BACK", ["RETURN"], "DONE", reason="at the start pose"),
        ], initial="WAIT_START", on_transition=on_transition, clock=clock)

    # ---- set-up -------------------------------------------------------------------------------------------------
    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update((self.cfg.extra or {}).get("scout", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        self.p = p
        u = getattr(ctx.b.map, "uav_ocs", None) or {}
        self.uav_mode = str(p.get("tier") or "").lower() in ("advanced", "tier_advanced") or bool(u.get("declared"))
        self.clr = gp.Clearance(clear_m=float(p["clear_m"]), hull_len_m=float(p["hull_len_m"]), hull_beam_m=float(p["hull_beam_m"]),
                                track_m=float(p["track_m"]))
        pose = ctx.b.pose
        self.start_pose = (pose.x_n, pose.y_e, pose.hdg_deg) if pose is not None else (0.0, 0.0, 0.0)
        self.poly = self._field(ctx)
        self.poly_in = G.inset(self.poly, float(p["margin_m"]))
        self.cov = G.Coverage(self.poly)
        self.frontier = G.lanes(self.poly_in, float(p["lane_m"]))
        self.frontier_done: set = set()
        self.logged: List[Dict[str, Any]] = []          # the buoy records written (visited or blocked)
        self.blocked: set = set()                       # map ids not to visit again (no visit point, a guard block)
        self.target: Optional[Dict[str, Any]] = None    # {id, xy, vp}
        self.goal: Optional[Tuple[float, float]] = None
        self.t0 = ctx.b.now
        self.path_m, self.last_xy = 0.0, (self.start_pose[0], self.start_pose[1])
        self.scans, self.obstacles_n = 0, 0
        self.scan_known: set = set()
        self.report_sig, self.report_t, self.reports = None, -1e9, 0
        self.uav_wait_t0: Optional[float] = None
        self.uav_caution_t = -1e9
        self.summary: Optional[Dict[str, Any]] = None
        self.lines: List[Dict[str, Any]] = []           # every jsonl line (the tests; the file has the same)
        self.run_id = getattr(ctx.b.run, "run_id", None)
        self.log_path = os.path.join(str(p["log_dir"]), f"scout_{self.run_id if self.run_id is not None else 'norun'}.jsonl")
        if ctx.b.map is not None and self.uav_mode:
            ctx.b.map.uav_frozen = True                 # as m1t2: the UAV layer stays for the attempt
        ctx.log(f"SCOUT started: field {len(self.poly)} pts ({abs(G.area(self.poly)):.0f} m2), inset {p['margin_m']} m, {len(self.frontier)} "
                f"lane points, clear_m {p['clear_m']}, budget {p['budget_s']:.0f} s" + (" - UAV declared: WAIT_UAV_MAP first" if self.uav_mode else ""))

    def _field(self, ctx: TaskContext) -> List[Tuple[float, float]]:
        if self.p.get("polygon"):
            return G.open_ring([tuple(q) for q in self.p["polygon"]])
        o, c = ctx.b.origin, getattr(ctx.b.run, "course", None)
        b = list(getattr(c, "boundary", None) or [])
        if o is not None and len(b) >= 3:
            return G.open_ring([o.to_local(float(q[0]), float(q[1])) for q in b])
        x, y, h = self.start_pose                       # no course: a square of field_m ahead of the start
        f = float(self.p["field_m"])
        u, v = (math.cos(math.radians(h)), math.sin(math.radians(h))), (-math.sin(math.radians(h)), math.cos(math.radians(h)))
        x0, y0 = x - 2.0 * u[0], y - 2.0 * u[1]
        return [(x0 + a * f * u[0] + s * f / 2 * v[0], y0 + a * f * u[1] + s * f / 2 * v[1]) for a, s in ((0, -1), (1, -1), (1, 1), (0, 1))]

    def precheck(self, ctx: TaskContext) -> str:
        if self.uav_mode:
            u = getattr(ctx.b.map, "uav_ocs", None) or {}
            if not u.get("active"):
                return "scout with the drone declared needs the UAV (no fresh UAV status at the OCS)"
        return ""

    # ---- the map -----------------------------------------------------------------------------------------------------
    def obstacles(self, ctx: TaskContext) -> List[tuple]:
        return gp.map_obstacles(ctx.b.map, int(self.p["min_obs"]))      # buoys, docks, boxes, UAV-only, the guard's obstacles (v0.1.80)

    @staticmethod
    def source(e) -> str:
        if e.source == "uav":
            return "uav"
        return "both" if getattr(e, "uav", None) else "camera"

    def candidates(self, ctx: TaskContext) -> list:
        """Usable buoys in the course polygon not yet logged (min_obs, sigma < sig_use_m; UAV-only ones count as the map counts them)."""
        m = ctx.b.map
        if m is None:
            return []
        out = []
        ents = m.entities("buoy", include_stale=True) + (m.uav_entities() if hasattr(m, "uav_entities") else [])
        for e in ents:
            if e.id in self.blocked or float(e.sigma_m) >= float(self.p["sig_use_m"]):
                continue
            if e.source != "uav" and e.n_obs < int(self.p["min_obs"]):
                continue
            if not GD.in_course((e.x_n, e.y_e), self.poly, float(GD.params(self.cfg)["course_tol_m"])):   # v0.1.85 (WP46): the shared rule
                continue
            if any(math.hypot(e.x_n - r["_xy"][0], e.y_e - r["_xy"][1]) < float(self.p["visit_r_m"]) for r in self.logged):
                continue
            out.append(e)
        return out

    def _ok_point(self, q, obstacles) -> bool:
        if not G.inside(q, self.poly_in):
            return False
        return all(math.hypot(q[0] - o[0], q[1] - o[1]) >= self.clr.keep(o[2]) for o in obstacles)

    def pick(self, ctx: TaskContext) -> Optional[Dict[str, Any]]:
        pose = ctx.b.pose
        boat = (pose.x_n, pose.y_e)
        obstacles = self.obstacles(ctx)
        best = None
        c = G.centroid(self.poly)
        for e in self.candidates(ctx):
            others = [o for o in obstacles if math.hypot(o[0] - e.x_n, o[1] - e.y_e) > 0.05]    # its own ring is what standoff_m keeps
            vp = G.visit_point((e.x_n, e.y_e), boat, float(self.p["standoff_m"]), lambda q, ob=others: self._ok_point(q, ob))
            if vp is None:
                self.blocked.add(e.id)
                self._record(ctx, e, visited=False, blocked=True, why="no stand-off point outside the clearance rings inside the field")
                continue
            path = gp.avoid([boat, vp], obstacles, clr=self.clr)
            key = (round(G.path_len(path), 1), math.hypot(e.x_n - c[0], e.y_e - c[1]))
            if best is None or key < best[0]:
                best = (key, {"id": e.id, "xy": (e.x_n, e.y_e), "vp": vp, "path": path})
        return best[1] if best else None

    # ---- the record ---------------------------------------------------------------------------------------------------
    def _write(self, ctx: TaskContext, line: Dict[str, Any]) -> None:
        line = dict(line, t=utc_iso(), run_id=self.run_id)
        self.lines.append(line)
        try:
            os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
            with open(self.log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps({k: v for k, v in line.items() if not k.startswith("_")}) + "\n")
        except OSError as ex:
            ctx.log(f"SCOUT log write failed: {ex}")
        self.report(ctx, "scout", {k: v for k, v in line.items() if not k.startswith("_")})

    def _record(self, ctx: TaskContext, e, visited: bool, blocked: bool = False, why: str = "") -> None:
        o = ctx.b.origin
        lat, lon = o.from_local(e.x_n, e.y_e) if o is not None else (None, None)
        red = (e.colour_votes or {}).get("red", 0.0) if hasattr(e, "colour_votes") else 0.0
        tot = sum((e.colour_votes or {}).values()) if getattr(e, "colour_votes", None) else 0.0
        colour = "red" if e.colour == "red" and tot > 0 and red / tot >= 0.5 else "unknown"
        n = len(self.logged) + 1
        rec = {"ev": "buoy", "id": f"B{n}", "map_id": e.id, "lat": None if lat is None else round(lat, 7), "lon": None if lon is None else round(lon, 7),
               "n_obs": e.n_obs, "range_min_m": self.range_min.get(e.id), "colour": colour, "colour_conf": round(red / tot, 2) if tot else None,
               "lit": {"flash": "flash", "steady": "steady", "off": "off"}.get(e.light, "unknown"), "source": self.source(e),
               "visited": visited, "blocked": blocked, "standoff_m": float(self.p["standoff_m"]) if visited else None, "_xy": (e.x_n, e.y_e)}
        if why:
            rec["why"] = why
        self.logged.append(rec)
        self._write(ctx, rec)
        ctx.log(f"scout: {rec['id']} {'logged' if visited else 'blocked'} {rec['lat']} {rec['lon']} · {colour if colour != 'unknown' else 'colour ?'}"
                f" · {e.n_obs} obs · {rec['source']}" + (f" ({why})" if why else ""))

    def maybe_report(self, ctx: TaskContext, complete: bool = False) -> None:
        """SafePassageReport (event safe_passage) when the mapped set changes: every usable buoy in the field, beacon UNKNOWN unless the
        vote says red; entry / exit empty."""
        o, m = ctx.b.origin, ctx.b.map
        if o is None or m is None:
            return
        buoys = []
        ents = m.entities("buoy", include_stale=True) + (m.uav_entities() if hasattr(m, "uav_entities") else [])
        for e in sorted(ents, key=lambda e: e.id):
            if (e.source != "uav" and e.n_obs < int(self.p["min_obs"])) or not GD.in_course((e.x_n, e.y_e), self.poly, float(GD.params(self.cfg)["course_tol_m"])):
                continue
            lat, lon = o.from_local(e.x_n, e.y_e)
            red = e.colour == "red"
            buoys.append({"id": e.id, "colour": "red" if red else "unknown", "state": "unknown",
                          "beacon": "BEACON_STATE_FLASHING_RED" if red else "BEACON_STATE_UNKNOWN", "lat": round(lat, 7), "lon": round(lon, 7)})
        sig = (tuple(b["id"] for b in buoys), complete)
        if not complete and (sig == self.report_sig or ctx.b.now - self.report_t < float(self.p["report_period_s"])):
            return
        self.report_sig, self.report_t = sig, ctx.b.now
        self.reports += 1
        self.report(ctx, "safe_passage", {"entry": None, "exit": None, "buoys": buoys, "complete": complete, "gates_passed": 0, "t": utc_iso()})

    # ---- the guard (WP36, v0.1.80) ------------------------------------------------------------------------------------
    def guard_target(self):
        """The key the guard counts its triggers on: the buoy being approached, the frontier point, else the stage."""
        st = self.m.state
        if st == "APPROACH" and self.target is not None:
            return ("buoy", self.target["id"])
        if st == "FRONTIER" and self.goal is not None:
            return ("frontier", round(self.goal[0], 1), round(self.goal[1], 1))
        return ("stage", st)

    def guard_target_xy(self):
        """v0.1.85 (WP41 T0.2e): the buoy being approached and the radius within which a guard stop is blamed on it (2 x standoff + 1 m)."""
        if self.m.state == "APPROACH" and self.target is not None:
            return (self.target["xy"][0], self.target["xy"][1], 2.0 * float(self.p["standoff_m"]) + 1.0)
        return None

    def on_guard(self, ctx: TaskContext, ev: Dict[str, Any]) -> str:
        """A guard event (the supervisor's obstacle line): into the scout's log (jsonl + the LOG tab)."""
        self.obstacles_n += 1
        self._write(ctx, dict(ev))
        ctx.log(f"scout: obstacle at {ev.get('range_m')} m {ev.get('bearing_deg'):+} deg in {ev.get('stage')} -> {ev.get('action')}")
        return "logged"

    def on_guard_block(self, ctx: TaskContext, key) -> None:
        """The 2nd trigger on the same target: a buoy is logged blocked and skipped, a frontier point skipped; PICK goes on (after the
        back-off, the supervisor restarts the stage - PICK decides)."""
        if key and key[0] == "buoy":
            self.blocked.add(key[1])
            m = ctx.b.map
            ents = (m.entities("buoy", include_stale=True) + (m.uav_entities() if hasattr(m, "uav_entities") else [])) if m is not None else []
            e = next((q for q in ents if q.id == key[1]), None)
            if e is not None:
                self._record(ctx, e, visited=False, blocked=True, why="the guard stopped twice on the way")
        self.target = None
        if self.m.state in ("APPROACH", "FRONTIER", "ADVANCE"):
            self._cancel()
            self.m.force("PICK", reason=f"guard: {key} blocked")

    # ---- behaviour wiring ----------------------------------------------------------------------------------------------
    def _follow(self, ctx: TaskContext, goal, end_r: float, target=None) -> Dict[str, Any]:
        pose = ctx.b.pose
        obstacles = self.obstacles(ctx)
        if target is not None:                          # v0.1.85 (WP41 T0.2a): the target keeps its ring - shrunk to standoff - 0.3 m, never dropped
            ring = float(self.p["standoff_m"]) - 0.3
            obstacles = [o if math.hypot(o[0] - target[0], o[1] - target[1]) > 0.05 else
                         (o[0], o[1], min(float(o[2]), max(0.05, ring - (self.clr.keep(0.0))))) for o in obstacles]
        path = gp.resample(gp.avoid([(pose.x_n, pose.y_e), tuple(goal)], obstacles, clr=self.clr), 0.5)
        if len(path) < 2:
            path = [(pose.x_n, pose.y_e), tuple(goal)]
        return {"points": path, "speed_mps": float(self.p["transit_v"]), "la_m": float(self.p["la_m"]),
                "corridor_m": float(self.p["corridor_m"]), "end_r_m": end_r}

    def behaviour_args(self, sub: str, ctx: TaskContext) -> Dict[str, Any]:
        p = self.p
        if sub in ("WAIT_START", "WAIT_UAV_MAP"):
            return {}
        if sub == "LOG":
            return {"duration_s": float(p["log_s"])}
        if sub == "ADVANCE":
            pose = ctx.b.pose
            self.goal, how = G.advance_goal((pose.x_n, pose.y_e), pose.hdg_deg, self.poly_in, float(p["advance_min_m"]))
            ctx.log(f"SCOUT ADVANCE {how}: to N {self.goal[0]:+.1f} E {self.goal[1]:+.1f}")
            return self._follow(ctx, self.goal, 1.0)
        if sub == "APPROACH":
            return self._follow(ctx, self.target["vp"], float(p["arrive_m"]), target=self.target["xy"])
        if sub == "FRONTIER":
            return self._follow(ctx, self.goal, 1.0)
        if sub == "RETURN":
            return self._follow(ctx, self.home(ctx), 1.0)
        if sub == "SCAN":
            self.scans += 1
            self.scan_known = {e.id for e in self.candidates(ctx)}
            return {"until": lambda b: bool({e.id for e in self.candidates(ctx)} - self.scan_known), "rate_dps": min(float(p["scan_rate_dps"]), float(p["max_yaw_dps"])),
                    "turns": 1.0, "reason": "nothing new in the scan", "poly": self.poly}
        return {}

    def home(self, ctx: Optional[TaskContext] = None) -> Tuple[float, float]:
        """RETURN's point: the start pose; under way (v0.1.85, the addendum T0.2c) at least margin + 2 m inside the field - the entry sits on
        the boundary - and clear of every mapped object's ring: moved toward the field's centre (then round it) until it is."""
        h = (self.start_pose[0], self.start_pose[1])
        if not GD.underway(self.cfg):
            return h
        ins = G.inset(self.poly, float(self.p["margin_m"]) + 2.0)
        obstacles = self.obstacles(ctx) if ctx is not None else []

        def ok(q):
            return (len(ins) < 3 or G.inside(q, ins)) and all(math.hypot(q[0] - o[0], q[1] - o[1]) >= self.clr.keep(o[2]) for o in obstacles)
        if ok(h):
            return h
        c = G.centroid(self.poly)
        L = math.hypot(c[0] - h[0], c[1] - h[1])
        for k in range(1, int(L / 0.5) + 1):
            base = (h[0] + (c[0] - h[0]) * k * 0.5 / L, h[1] + (c[1] - h[1]) * k * 0.5 / L)
            for r in (0.0, 1.0, 2.0, 3.0):
                for a in range(0, 360, 30 if r else 360):
                    q = (base[0] + r * math.cos(math.radians(a)), base[1] + r * math.sin(math.radians(a)))
                    if ok(q):
                        return q
        return c

    def _next_frontier(self, ctx: TaskContext) -> Optional[Tuple[float, float]]:
        pose = ctx.b.pose
        obstacles = self.obstacles(ctx)
        left = []
        for i, q in enumerate(self.frontier):
            if i in self.frontier_done:
                continue
            if self.cov.seen_share(q, float(self.p["frontier_r_m"])) >= float(self.p["frontier_seen"]) or not self._ok_point(q, obstacles):
                self.frontier_done.add(i)
                continue
            left.append((math.hypot(q[0] - pose.x_n, q[1] - pose.y_e), i, q))
        if not left:
            return None
        _, i, q = min(left)
        self.frontier_done.add(i)
        return q

    def _cancel(self) -> None:
        if self._beh is not None:
            self._beh.cancel()
            self._beh = None
            self._beh_state = None

    def _decide(self, ctx: TaskContext) -> None:
        """PICK (a pure decision): the nearest target, else SCAN once the map is exhausted (a SCAN that found nothing -> FRONTIER)."""
        self.target = self.pick(ctx)
        if self.target is not None:
            ctx.log(f"SCOUT target #{self.target['id']} N {self.target['xy'][0]:+.1f} E {self.target['xy'][1]:+.1f}: stand-off point "
                    f"N {self.target['vp'][0]:+.1f} E {self.target['vp'][1]:+.1f}, {G.path_len(self.target['path']):.1f} m")
            self.m.handle("TARGET")
            return
        self.m.handle("NONE")

    def wait_uav_map(self, ctx: TaskContext) -> None:
        """As m1t2 (T1SafePassage.uav_map_ready): scan.complete, the drone back on the ground and quiet, or `uav scan done`; no entry
        needed (the scout visits whatever the drone mapped); uav_wait_s -> on without it (a CAUTION in the log)."""
        now, m = ctx.b.now, ctx.b.map
        if self.uav_wait_t0 is None:
            self.uav_wait_t0 = now
        u = getattr(m, "uav_ocs", None) or {}
        sc = getattr(m, "uav_scan", {}) or {}
        why = None
        if u.get("scan_done"):
            why = "the operator's `uav scan done`"
        elif sc.get("state") == "complete":
            why = "the UAV's scan.complete"
        else:
            self.uav_gate = UG.touchdown(u, float(getattr(m, "uav_change_t", -1e9)), now, float(self.p["uav_quiet_s"]),
                                         self.__dict__.setdefault("uav_latch", {}))  # v0.1.89: flying -> map quiet -> RTL -> disarmed
            if self.uav_gate["go"]:
                why = "TOUCHDOWN: the drone disarmed after flying, the map quiet"
        if why is None and now - self.uav_wait_t0 > float(self.p["uav_wait_s"]):
            why = f"no UAV map after {float(self.p['uav_wait_s']):.0f} s - scouting without it"
        if why is not None:
            ctx.log(f"SCOUT WAIT_UAV_MAP done after {now - self.uav_wait_t0:.0f} s: {why}")
            self._cancel()
            self.m.handle("UAV_MAP")

    def step(self, ctx: TaskContext):
        pose = ctx.b.pose
        if pose is not None and self.started:
            self.path_m += math.hypot(pose.x_n - self.last_xy[0], pose.y_e - self.last_xy[1])
            self.last_xy = (pose.x_n, pose.y_e)
            self.cov.mark((pose.x_n, pose.y_e), pose.hdg_deg, float(self.p["coverage_r_m"]), float(self.p["hfov_deg"]))
            if ctx.b.map is not None:
                for e in ctx.b.map.entities("buoy", include_stale=True):
                    d = math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e)
                    if e.state == "tracked" and (e.id not in self.range_min or d < self.range_min[e.id]):
                        self.range_min[e.id] = round(d, 1)
        if not self.holding:
            st = self.m.state
            moving = st in ("ADVANCE", "APPROACH", "SCAN", "FRONTIER", "PICK", "LOG")
            if moving and ctx.b.now - self.t0 > float(self.p["budget_s"]):
                ctx.log(f"SCOUT budget {float(self.p['budget_s']):.0f} s reached in {st}: RETURN")
                self._cancel()
                self.m.handle("HOME", reason="the budget")
            elif st == "WAIT_START":
                self.m.handle("GO_UAV" if self.uav_mode else "GO")
            elif st == "WAIT_UAV_MAP":
                self.wait_uav_map(ctx)
            elif st == "ADVANCE" and self.candidates(ctx):
                self._cancel()
                self.m.handle("LOCK")
            elif st == "PICK":
                self._decide(ctx)
            elif st == "APPROACH" and self.target is not None and self.target["id"] in self.blocked:
                self._cancel()
                self.m.handle("REPICK")
            elif st == "FRONTIER" and self.candidates(ctx):
                self._cancel()
                self.m.handle("SEEN")
            if self.m.state not in ("WAIT_START", "WAIT_UAV_MAP", "DONE", "ABORTED"):
                self.maybe_report(ctx)
        out = super().step(ctx)
        if out.out is not None:
            out.out.cmd = self._yaw_limit(ctx, out.out.cmd)
            if getattr(self, "uav_gate", None):                            # v0.1.89: the touchdown sequence -> the OCS
                out.out.info["uav_gate"] = dict(self.uav_gate, waiting=self.m.state == "WAIT_UAV_MAP")
            out.out.info["scout"] = {"logged": len(self.logged), "visited": sum(1 for r in self.logged if r["visited"]),
                                     "coverage_pct": round(self.cov.pct()), "target": self.target["id"] if self.target else None,
                                     "visited_ids": [r["map_id"] for r in self.logged if r["visited"]]}
        return out

    TURN_STATES = ("ADVANCE", "APPROACH", "FRONTIER", "RETURN")

    def _yaw_limit(self, ctx: TaskContext, cmd):
        """v0.1.82 (Dmitrii: "boat has very fast turns, lets limit yaw rate so it will be slowly taking enough time to detect"): a GUIDED
        carrot more than turn_gate_deg (20) off the bow -> turn on the spot at max_yaw_dps (10 deg/s: a buoy stays ~5 s in the 51 deg view)
        until within turn_exit_deg (8), then GUIDED again. GUIDED alone lets ArduRover (and the sim plant, 45 deg/s) pivot at its own rate.
        The SCAN spin is capped at the same rate."""
        pose, o = ctx.b.pose, ctx.b.origin
        if GD.underway(self.cfg):                       # v0.1.85 (WP41 T0.2): the 20 / 8 deg turn-first gate is gone under way (the law drives)
            self._turning = False
            return cmd
        if not isinstance(cmd, GuidedTarget) or pose is None or o is None or self.m.state not in self.TURN_STATES:
            self._turning = False
            return cmd
        x, y = o.to_local(cmd.lat, cmd.lon)
        if math.hypot(x - pose.x_n, y - pose.y_e) < 0.5:
            return cmd
        err = (math.degrees(math.atan2(y - pose.y_e, x - pose.x_n)) - pose.hdg_deg + 180.0) % 360.0 - 180.0
        gate = float(self.p["turn_exit_deg"] if getattr(self, "_turning", False) else self.p["turn_gate_deg"])
        self._turning = abs(err) > gate
        if not self._turning:
            return cmd
        return VelocityYawTarget(0.0, math.copysign(math.radians(float(self.p["max_yaw_dps"])), err))

    @property
    def range_min(self) -> Dict[int, float]:
        if not hasattr(self, "_range_min"):
            self._range_min = {}
        return self._range_min

    def on_behaviour_done(self, sub: str, ctx: TaskContext) -> None:
        if sub == "ADVANCE":
            self.m.handle("FAR")
        elif sub == "APPROACH":
            self.m.handle("ARRIVED")
        elif sub == "LOG":
            m = ctx.b.map
            ents = (m.entities("buoy", include_stale=True) + (m.uav_entities() if hasattr(m, "uav_entities") else [])) if m is not None else []
            e = next((q for q in ents if q.id == self.target["id"]), None)
            if e is None:                                # merged away (a UAV-only buoy the camera took over): the nearest map buoy
                tx, ty = self.target["xy"]
                e = min(ents, key=lambda q: math.hypot(q.x_n - tx, q.y_e - ty), default=None)
            if e is not None:
                self._record(ctx, e, visited=True)
            self.target = None
            self.m.handle("LOGGED")
        elif sub == "SCAN":
            self.m.handle("BUOYS")
        elif sub == "FRONTIER":
            self.m.handle("AT_POINT")
        elif sub == "RETURN":
            self._finish(ctx)
            self.m.handle("BACK")

    def on_behaviour_abort(self, sub: str, reason: str, ctx: TaskContext) -> None:
        if sub == "SCAN":
            self.goal = self._next_frontier(ctx)
            if self.goal is None:
                ctx.log("SCOUT: no unswept frontier point left: RETURN")
                self.m.handle("HOME")
            else:
                ctx.log(f"SCOUT FRONTIER: to N {self.goal[0]:+.1f} E {self.goal[1]:+.1f} (coverage {self.cov.pct():.0f} %)")
                self.m.handle("NOTHING")
            return
        if sub in ("APPROACH", "FRONTIER", "ADVANCE"):           # a leg that failed (off the path / timeout): decide again
            ctx.log(f"SCOUT {sub}: {reason} - picking again")
            if sub == "APPROACH" and self.target is not None:
                self.blocked.add(self.target["id"])
            self.m.force("PICK", reason=f"{sub}: {reason}")
            return
        if sub == "RETURN":
            self._finish(ctx)
        super().on_behaviour_abort(sub, reason, ctx)

    def _finish(self, ctx: TaskContext) -> None:
        if self.summary is not None:
            return
        self.maybe_report(ctx, complete=True)
        self.summary = {"ev": "summary", "buoys": len(self.logged), "visited": sum(1 for r in self.logged if r["visited"]),
                        "blocked": sum(1 for r in self.logged if r.get("blocked")), "path_m": round(self.path_m, 1),
                        "duration_s": round(ctx.b.now - self.t0, 1), "coverage_pct": round(self.cov.pct()), "scans": self.scans,
                        "obstacles": self.obstacles_n, "reports": self.reports}
        self._write(ctx, self.summary)
        ctx.log(f"scout: summary {self.summary}")
