"""Task 3 Coordinated Logistics, core tier (handbook 3.3.4; ROADMAP WP3; mission code m3t1):

WAIT_START -> FIND_DOCK (ScanYaw until the dock line and one GREEN indicator are in the map) -> TO_LINE (FollowPath to the
standoff line, `standoff_m` off the dock) -> SWAY_TO_ABEAM (SwayTransit along it, bow to the dock, until abeam of the bay)
-> TURN_IN (DockLos align) -> ENTER (DockLos enter, stop `stop_m` off the face) -> DOCKED (hold) -> REPORT_DOCK
(DockingReport) -> AIM_FIRE (cannon on at the bay until a window there is seen GREEN, at most `fire_s`) -> REPORT_FIRE
(FirefightingReport) -> UNDOCK (DockLos astern to the standoff) -> DONE.

Decided 2026-11-03: standoff 5 m, stop 1 m (1.5 m from v0.1.39: the windows in the 51 deg view), firing 30 s (read as: until GREEN is seen, 30 s at most), undock astern.
The cannon is asked through the task's output (info.cannon -> the mission manager publishes local/<v>/cannon/aim at 5 Hz);
in simulation fake_dock lights the window when it hears the DockingReport event and turns it GREEN under a fresh aim.
The dock is read only through usv/dockpath.dock_geometry (locked at the end of FIND_DOCK).

v0.1.48 (WP10j-b, docs/M3T1_SIM_PLAN.md section 1, decided 2026-11-06): FIND_DOCK ends on the dock line (`bays` faces, or two after a
full turn) - the bay is no longer chosen from far (CV_TRAIN: the indicator is found in only half the frames at 20-30 m). TO_LINE goes
to the near end of the search line (the standoff line from `search_margin_m` beyond the first bay to as far beyond the last) ->
SWAY_SEARCH sways the whole line, bow to the dock, counting each bay's indicator sightings (the map's ind_hist / ind_votes) on this
pass; at its end the one bay with >= `min_votes` GREEN and more GREEN than RED wins (none or a tie: a second pass back at
`standoff2_m`; still none: hold and report) -> SWAY_TO_ABEAM of it -> TURN_IN -> ENTER. On the way in (TURN_IN / ENTER) the bay's
indicator reading RED for `bay_confirm_s` (>= 3 sightings) is a wrong call -> BACK_OUT (DockLos astern to the standoff line) -> the
search again without that bay; leaving the bay centreline (DockLos' geometry abort) -> BACK_OUT -> SWAY_TO_ABEAM of the same bay; at most
`reengage_max` re-engagements, then hold. AIM_FIRE aims at the bay's window the map sees lit RED (window_id 1 upper / 2 lower, on the
aim topic: local/<v>/cannon/aim {on, bay, window, window_id}); none lit after `window_wait_s`: at the other window when one of the two
is seen dark (the camera misses the lit one), else at the bay centre (window_id null).
The reports strictly in order and once each: DockingReport {bay} only from REPORT_DOCK (after DOCKED), FirefightingReport {window,
window_id, complete, result, fired_s} only from REPORT_FIRE. `--bay N` skips the search (and is still re-checked on the way in).

v0.1.56 (D6, docs/OVERNIGHT_2026-11-07.md section 3): the standoff lines derive from the dock geometry (usv/dockpath.standoff_lines):
standoff_m = finger_m + hull_len_m / 2 + clear_m + standoff_margin_m (1.0), standoff2_m the same + standoff2_margin_m (0.3) - 5.70 /
5.00 m with the 3.5 m fingers, the 1.19 m hull and the 0.60 m rule; a non-null t3.standoff_m / standoff2_m is an explicit override (the
TUNE knob sets it). TO_LINE from inside the standoff line (bow toward the dock, nearer than the line) backs out to it first (DockLos
astern along the dock normal), then goes on as before.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional

from ...companion.fsm import Transition
from ...companion.state import Hold, RobotXTask, utc_iso
from ..behaviours.base import BehaviourOut
from ..behaviours import DockLos, FollowPath, ScanYaw, StationKeep, SwayTransit
from ..dockpath import FINGER_M, Bay, DockGeom, box_faces, dock_geometry, refit, standoff_lines
from ..gatepath import planner as gp
from .base import Task, TaskContext, TaskStatus, simple_machine

DEFAULTS = {"standoff_m": None, "stop_m": 1.5, "fire_s": 30.0, "bay": None, "line_v": 0.5, "sway_v": 0.25, "enter_v": 0.15,
            "undock": "astern", "undock_v": 0.2, "docked_hold_s": 3.0, "bay_width_m": 3.5, "scan_rate_dps": 20.0,
            "green_confirm_s": 1.0,
            # v0.1.48 (WP10j-b)
            "bays": 3, "standoff2_m": None, "search_margin_m": 1.5, "min_votes": 3, "bay_confirm_s": 1.0, "bay_recheck": True,
            "reengage_max": 2, "window_wait_s": 10.0,
            # v0.1.56 (D6): the standoff lines from the dock geometry (dockpath.standoff_lines; standoff_m / standoff2_m non-null = an override)
            "finger_m": FINGER_M, "standoff_margin_m": 1.0, "standoff2_margin_m": 0.3,
            # v0.1.58 (WP13, D14): where the dock comes from - map (the dock entities) | boxes (the faces' key points, BoxTrack.stable)
            "dock_source": "map", "box_window_s": 1.0, "box_min_samples": 3}
STATES = ["WAIT_START", "FIND_DOCK", "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "BACK_OUT", "DOCKED", "REPORT_DOCK",
          "AIM_FIRE", "REPORT_FIRE", "UNDOCK"]
WINDOW_NAME = {1: "upper", 2: "lower"}           # rx_reports.proto: window_id 1 = upper, 2 = lower (the rendered 1 upper-left, 2 lower-right)


class T3Logistics(Task):
    name = "T3"
    robo_task = RobotXTask.T3
    BEHAVIOURS = {"WAIT_START": StationKeep, "FIND_DOCK": ScanYaw, "TO_LINE": FollowPath, "SWAY_SEARCH": SwayTransit,
                  "SWAY_TO_ABEAM": SwayTransit, "TURN_IN": DockLos, "ENTER": DockLos, "BACK_OUT": DockLos, "DOCKED": StationKeep,
                  "REPORT_DOCK": None, "AIM_FIRE": StationKeep, "REPORT_FIRE": None, "UNDOCK": DockLos}
    SKIP_REFUSE = {"WAIT_START": "a timer", "FIND_DOCK": "the map", "DOCKED": "a timer", "AIM_FIRE": "the map"}   # v0.1.68 (F4)
    # v0.1.26 (WP7): `ocs_cli start_task m3t1 --from TURN_IN` - all need the dock line and the bay in the map now
    ENTRY_POINTS = {s: "the dock line and the bay (GREEN or args.bay) in the map" for s in
                    ("TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "AIM_FIRE", "UNDOCK")}
    ENTRY_POINTS["SWAY_SEARCH"] = "the dock line in the map"
    ENTRY_POINTS.update({s: "the dock line (every face: `bays`) and the bay (GREEN or args.bay) in the map" for s in ENTRY_POINTS if s != "SWAY_SEARCH"})

    def build_machine(self, on_transition, clock):
        return simple_machine("T3", STATES, [
            Transition("GO", ["WAIT_START"], "FIND_DOCK", reason="started"),
            Transition("FOUND", ["FIND_DOCK"], "TO_LINE", reason="the dock line is in the map"),
            Transition("ON_LINE", ["TO_LINE"], "SWAY_SEARCH", reason="on the standoff line: sway past every bay"),
            Transition("ON_LINE_BAY", ["TO_LINE"], "SWAY_TO_ABEAM", reason="on the standoff line (the bay is given)"),
            Transition("CHOSEN", ["SWAY_SEARCH"], "SWAY_TO_ABEAM", reason="one bay votes GREEN"),
            Transition("ABEAM", ["SWAY_TO_ABEAM"], "TURN_IN", reason="abeam of the bay"),
            Transition("ALIGNED", ["TURN_IN"], "ENTER", reason="bow to the bay"),
            Transition("WRONG_BAY", ["TURN_IN", "ENTER"], "BACK_OUT", reason="the bay's indicator reads RED"),
            Transition("MISALIGNED", ["TURN_IN", "ENTER"], "BACK_OUT", reason="off the bay centreline"),
            Transition("BACKED", ["BACK_OUT"], "SWAY_SEARCH", reason="back on the standoff line: search without that bay"),
            Transition("REALIGN", ["BACK_OUT"], "SWAY_TO_ABEAM", reason="back on the standoff line: line up again"),
            Transition("IN", ["ENTER"], "DOCKED", reason="stop distance reached"),
            Transition("SETTLED", ["DOCKED"], "REPORT_DOCK", reason="holding in the bay"),
            Transition("REPORTED", ["REPORT_DOCK"], "AIM_FIRE", reason="DockingReport sent"),
            Transition("FIRED", ["AIM_FIRE"], "REPORT_FIRE", reason="window GREEN (or the firing time is over)"),
            Transition("REPORTED", ["REPORT_FIRE"], "UNDOCK", reason="FirefightingReport sent"),
            Transition("OUT", ["UNDOCK"], "DONE", reason="back on the standoff line"),
        ], initial="WAIT_START", on_transition=on_transition, clock=clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update(self.cfg.extra.get("t3", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        if p["undock"] != "astern":
            raise ValueError("t3 undock: astern (the only way built)")
        if p["dock_source"] not in ("map", "boxes"):
            raise ValueError(f"t3 dock_source {p['dock_source']!r}: map | boxes")
        p["standoff_m"], p["standoff2_m"] = standoff_lines(p, self.cfg.extra.get("t1"))      # v0.1.56 (D6)
        # v0.1.56 (D7): nearer the dock than the finger tips + half the hull + the 0.60 m rule = inside the line: TO_LINE backs out first
        self.inner_m = standoff_lines(dict(p, standoff_m=None, standoff_margin_m=0.0), self.cfg.extra.get("t1"))[0]
        self.lining_out = False
        self.p = p
        self.geom: Optional[DockGeom] = None
        self.bay: Optional[Bay] = None
        self.forced = int(p["bay"]) if p["bay"] else None
        self.safe_hold: Optional[str] = None
        self.t_fire = None
        self.green_since = None
        self.saw_red = False
        self.fire_result: Dict[str, Any] = {}
        self.target: Optional[int] = None            # v0.1.48: the window aimed at (1 upper / 2 lower), None = not chosen
        self.at_centre = False                       # ... no window seen lit after window_wait_s: at the bay centre
        self.excluded: set = set()                   # dock entity ids ruled out by a wrong call
        self.reengaged = 0
        self.back_reason = ""
        self.pass_no = 1
        self.pass_sd = float(p["standoff_m"])
        self.pass_base: Dict[int, Dict[str, int]] = {}
        self.pass_t0 = None
        self.search_log: List[Dict[str, Any]] = []   # one record per pass (the campaign's grader reads it)
        self.t_approach = None
        self.reported: set = set()
        ctx.log(f"T3 started: standoff {p['standoff_m']:.2f} / {p['standoff2_m']:.2f} m, stop {p['stop_m']} m, fire up to {p['fire_s']} s, bay "
                f"{self.forced or 'by the sway search'}")

    def check_entry(self, stage: str, ctx: TaskContext) -> str:
        if stage == "SWAY_SEARCH":
            g = self._geometry(ctx)
            if g is None or len(g.bays) < 2:
                return f"{stage} needs {self.ENTRY_POINTS[stage]} - not there now (look at the dock first)"
            self.geom = g
            ctx.log(f"T3 from {stage}: {len(g.bays)} bays")
            return ""
        if not self._choose(ctx):
            return f"{stage} needs {self.ENTRY_POINTS[stage]} - not there now (look at the dock first)"
        ctx.log(f"T3 from {stage}: bay {self.bay.number} ({self.bay.colour}) face N {self.bay.face[0]:+.1f} E {self.bay.face[1]:+.1f}")
        return ""

    # ---- the dock ----------------------------------------------------------------------------------------------------
    def _geometry(self, ctx: TaskContext, pose=None) -> Optional[DockGeom]:
        """v0.1.58: usv/dockpath.dock_geometry from t3.dock_source (map | boxes)."""
        p = self.p
        kw = {"observation_s": float(p["box_window_s"]), "min_samples": int(p["box_min_samples"])}
        return dock_geometry(ctx.b.map, ctx.b.pose if pose is None else pose, ctx.b.now, source=str(p["dock_source"]), stable_kw=kw)

    def _boxes_why(self, ctx: TaskContext) -> str:
        """v0.1.58: with dock_source boxes, why there is no line (fewer than 2 stable faces with key points)."""
        if str(self.p["dock_source"]) != "boxes":
            return ""
        p = self.p
        n = len(box_faces(ctx.b.map, {"observation_s": float(p["box_window_s"]), "min_samples": int(p["box_min_samples"])}))
        return f" (dock_source boxes: {n} stable face(s) with key points, a line needs 2)" if n < 2 else ""

    def _choose(self, ctx: TaskContext) -> bool:
        """The entry points' bay: args.bay, else the one GREEN face of the map (its recent indicator sightings)."""
        g = self._geometry(ctx)
        if g is None or len(g.bays) < int(self.p["bays"]):     # v0.1.56: the whole line - a bay's number needs every face (seen 2 of 3,
            return False                                       # the middle bay was numbered 1)
        if self.forced:
            bay = next((b for b in g.bays if b.number == self.forced), None)
        else:
            bay = g.green()
        if bay is None:
            return False
        self.geom, self.bay = g, bay
        return True

    def _found(self, ctx: TaskContext, n_min: Optional[int] = None) -> bool:
        """FIND_DOCK's exit: the dock line with `bays` faces (n_min after a full turn); args.bay must be among them."""
        g = self._geometry(ctx)
        if g is None or len(g.bays) < (n_min or int(self.p["bays"])):
            return False
        if self.forced and not any(b.number == self.forced for b in g.bays):
            return False
        self.geom = g
        if self.forced:
            self.bay = next(b for b in g.bays if b.number == self.forced)
        return True

    def _relock(self, ctx: TaskContext) -> None:
        """The dock geometry again (a face found on the way in is numbered with the others); the chosen bay kept by its entity."""
        g = self._geometry(ctx)
        if g is not None and self.geom is not None and len(g.bays) >= len(self.geom.bays):
            self.geom = g
            if self.bay is not None:
                self.bay = next((b for b in g.bays if b.entity_id == self.bay.entity_id), self.bay)

    def _refresh_bay(self, ctx: TaskContext) -> None:
        """v0.1.56 (D8): before SWAY_TO_ABEAM, TURN_IN and ENTER the dock line refitted from the map's current face positions (dockpath.refit) - the
        line locked from far carries the far sightings' error (the geometric campaign: the face 0.4 m off, docked 1.1 m from it with both
        windows outside the 51 deg camera; the camera campaign: the normal 17 deg off)."""
        self._relock(ctx)
        if ctx.b.map is not None and self.geom is not None:
            if str(self.p["dock_source"]) == "map":            # v0.1.58 merge: the boxes source re-fits from its own key points each call
                self.geom = refit(self.geom, ctx.b.map)
            self.bay = next((b for b in self.geom.bays if b.entity_id == self.bay.entity_id), self.bay)

    def _ends(self, sd: float):
        """The search line's two ends at standoff sd: search_margin_m beyond the first and the last bay."""
        return list(self.geom.sway_path(sd, float(self.p["search_margin_m"])))      # v0.1.58: DockGeom.sway_path

    def _line_target(self, here) -> tuple:
        sd = float(self.p["standoff_m"])
        if self.bay is not None:
            return self.geom.on_standoff(here, sd)
        e = self._ends(sd)
        return min(e, key=lambda q: math.hypot(q[0] - here[0], q[1] - here[1]))

    def _votes(self, ctx: TaskContext, bay: Bay) -> Dict[str, int]:
        e = ctx.b.map.ents.get(bay.entity_id) if ctx.b.map is not None else None
        cur = dict(getattr(e, "ind_votes", None) or {}) if e is not None else {}
        if not cur and e is not None and self.geom is not None and self.geom.source == "boxes":
            cur = {c: int(v) for c, v in (getattr(e, "colour_votes", None) or {}).items()}   # v0.1.58: a box face's own colour
        base = self.pass_base.get(bay.entity_id, {})
        return {c: int(cur.get(c, 0) - base.get(c, 0)) for c in ("red", "green")}

    def _sway(self, a, b) -> Dict[str, Any]:
        theta = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360.0
        err = ((theta - 90.0) - self.geom.bow_in_hdg + 180.0) % 360.0 - 180.0
        return {"a": a, "b": b, "dir": 1 if abs(err) < 90.0 else -1}

    def windows(self, ctx):
        """Window entities belonging to the chosen bay (within half a bay width of its face, close to the dock line)."""
        if ctx.b.map is None or self.geom is None:
            return []
        out = []
        for w in ctx.b.map.entities("window"):
            dn, de = w.x_n - self.bay.face[0], w.y_e - self.bay.face[1]
            along = dn * self.geom.along[0] + de * self.geom.along[1]
            off = dn * self.geom.normal[0] + de * self.geom.normal[1]
            if abs(along) <= float(self.p["bay_width_m"]) / 2 and abs(off) < 1.0:
                out.append(w)
        return out

    def window_no(self, w) -> int:
        """1 upper / 2 lower: the camera's number (cv/sim_observer, by its place on the face), else the side: 1 is left of the bay
        centre as seen from the water, 2 right."""
        n = getattr(w, "window_no", None)
        if n in (1, 2):
            return int(n)
        along = (w.x_n - self.bay.face[0]) * self.geom.along[0] + (w.y_e - self.bay.face[1]) * self.geom.along[1]
        return 1 if along < 0 else 2

    # ---- behaviour wiring ----------------------------------------------------------------------------------------------
    def behaviour_args(self, sub: str, ctx: TaskContext) -> Dict[str, Any]:
        p, g = self.p, self.geom
        if sub == "WAIT_START":
            return {}
        if sub == "FIND_DOCK":
            return {"until": lambda b: self._found(ctx), "rate_dps": p["scan_rate_dps"], "turns": 1.0,
                    "reason": "no dock line found (no GREEN bay to look for)" if not self.forced else f"no dock line with bay {self.forced} found"}
        pose = ctx.b.pose
        here = (pose.x_n, pose.y_e)
        sd = float(p["standoff_m"])
        if sub == "TO_LINE":
            if "BEHAVIOURS" in self.__dict__:                         # v0.1.56 (D7): inside the standoff line - out along the normal first
                off = (here[0] - g.centre[0]) * g.normal[0] + (here[1] - g.centre[1]) * g.normal[1]
                foot = (here[0] - off * g.normal[0], here[1] - off * g.normal[1])
                ctx.log(f"T3 TO_LINE: {off:.1f} m off the dock line, inside the standoff line ({self.inner_m:.1f} m) - out along the normal "
                        f"to {sd:.1f} m first")
                return {"mode": "out", "face": foot, "normal": g.normal, "out_m": sd, "v": p["undock_v"], "corridor_m": 3.0, "timeout_s": 120.0}
            start = self._line_target(here)
            obstacles = [(e.x_n, e.y_e) for e in ctx.b.map.entities("buoy")] if ctx.b.map is not None else []
            path = gp.resample(gp.avoid([here, start], obstacles), 0.5)
            return {"points": path, "speed_mps": p["line_v"], "la_m": 1.5, "corridor_m": 3.0, "end_r_m": 0.7}
        if sub == "SWAY_SEARCH":
            if (ctx.b.map is not None and str(p["dock_source"]) == "map"
                    and any(b.entity_id not in ctx.b.map.ents for b in g.bays)):     # v0.1.56: after a lost line (RESUME); v0.1.58: the map source only
                g2 = self._geometry(ctx, pose)
                if g2 is not None and len(g2.bays) >= 2:
                    self.geom = g = g2
                    ctx.log(f"T3 SWAY_SEARCH: the dock line again ({len(g.bays)} bays)")
            self.pass_sd = sd if self.pass_no == 1 else float(p["standoff2_m"])
            a = g.on_standoff(here, self.pass_sd)
            e = self._ends(self.pass_sd)
            s_here = (a[0] - g.centre[0]) * g.along[0] + (a[1] - g.centre[1]) * g.along[1]
            b = max(e, key=lambda q: abs((q[0] - g.centre[0]) * g.along[0] + (q[1] - g.centre[1]) * g.along[1] - s_here))
            self.pass_base = {}
            self.pass_base = {bay.entity_id: self._votes(ctx, bay) for bay in g.bays}          # v0.1.58: the same counts _votes reads
            self.pass_t0 = ctx.b.now
            ctx.log(f"T3 search pass {self.pass_no} at {self.pass_sd:.1f} m: {len(g.bays)} bays, sway {math.hypot(b[0] - a[0], b[1] - a[1]):.1f} m"
                    + (f", without bay(s) {self._excluded_numbers()}" if self.excluded else ""))
            off = math.hypot(a[0] - here[0], a[1] - here[1])               # from --from SWAY_SEARCH the boat may start off the line
            return dict(self._sway(a, b), v=p["sway_v"], corridor_m=max(3.0, off + 1.0), end_r_m=0.4)
        if sub == "SWAY_TO_ABEAM":
            self._refresh_bay(ctx)                                         # v0.1.56: the line refitted before lining up on the bay
            g = self.geom
            a = g.on_standoff(here, sd)
            off = math.hypot(a[0] - here[0], a[1] - here[1])               # v0.1.56: from --from SWAY_TO_ABEAM the boat may start off the line
            return dict(self._sway(a, g.abeam(self.bay, sd)), v=p["sway_v"], corridor_m=max(2.5, off + 1.0), end_r_m=0.4)
        if sub in ("TURN_IN", "ENTER", "UNDOCK", "BACK_OUT"):
            if sub in ("TURN_IN", "ENTER"):
                self._refresh_bay(ctx)
                g = self.geom
            mode = {"TURN_IN": "align", "ENTER": "enter", "UNDOCK": "astern", "BACK_OUT": "astern"}[sub]
            return {"mode": mode, "face": self.bay.face, "normal": g.normal, "stop_m": p["stop_m"], "out_m": sd,
                    "v": p["undock_v"] if sub in ("UNDOCK", "BACK_OUT") else p["enter_v"], "corridor_m": 3.0 if sub == "BACK_OUT" else 1.2}
        if sub == "DOCKED":
            return {"duration_s": p["docked_hold_s"]}
        if sub == "AIM_FIRE":
            return {}
        return {}

    def _excluded_numbers(self):
        return [b.number for b in (self.geom.bays if self.geom else []) if b.entity_id in self.excluded]

    def step(self, ctx: TaskContext):
        st = self.m.state
        if st not in ("TURN_IN", "ENTER"):
            self.t_approach = None
        if not self.holding:
            if st == "WAIT_START":
                self.m.handle("GO")
            elif st == "TO_LINE" and self._beh is None:
                pose = ctx.b.pose
                g = self.geom
                off = (pose.x_n - g.centre[0]) * g.normal[0] + (pose.y_e - g.centre[1]) * g.normal[1]
                if off < self.inner_m and not self.lining_out:      # v0.1.56 (D7): the next behaviour is the way out (DockLos "out")
                    self.lining_out = True
                    self.BEHAVIOURS = dict(type(self).BEHAVIOURS, TO_LINE=DockLos)
                start = self._line_target((pose.x_n, pose.y_e))
                if math.hypot(start[0] - pose.x_n, start[1] - pose.y_e) < 1.5:
                    self._on_line(ctx, "already on the standoff line")
            elif st == "SWAY_TO_ABEAM" and self._beh is None:
                pose = ctx.b.pose
                sd = float(self.p["standoff_m"])
                a, b = self.geom.on_standoff((pose.x_n, pose.y_e), sd), self.geom.abeam(self.bay, sd)
                if math.hypot(b[0] - a[0], b[1] - a[1]) < 1.0 and math.hypot(a[0] - pose.x_n, a[1] - pose.y_e) < 0.8:
                    self.m.handle("ABEAM", reason="already abeam of the bay")
            elif st == "SWAY_SEARCH" and self._beh is not None and ctx.b.map is not None and self.geom is not None:
                gone = [b.number for b in self.geom.bays if b.entity_id not in ctx.b.map.ents] \
                    if str(self.p["dock_source"]) == "map" else []   # v0.1.58 merge: the map source only (box track ids are not map entities)
                if gone:                                         # v0.1.56 (D7): the map lost the line (CLEAR_MAP, aged out) - the stage aborts
                    self._beh.cancel()
                    self._beh = None
                    self.on_behaviour_abort(st, f"the dock line is lost (bay(s) {gone} no longer in the map)", ctx)
                    return TaskStatus(sub_state=st, out=BehaviourOut(cmd=Hold()))
            elif st in ("TURN_IN", "ENTER"):
                self.t_approach = ctx.b.now if self.t_approach is None else self.t_approach
                self._recheck(ctx)
            elif st == "REPORT_DOCK":
                if "docking" not in self.reported:                  # once, and only here: after DOCKED
                    self.reported.add("docking")
                    self.report(ctx, "docking", {"bay": self.bay.number, "t": utc_iso()})
                    ctx.log(f"T3 DockingReport: bay {self.bay.number}")
                self.m.handle("REPORTED")
            elif st == "AIM_FIRE":
                self._fire(ctx)
            elif st == "REPORT_FIRE":
                if "firefighting" not in self.reported:
                    self.reported.add("firefighting")
                    self.report(ctx, "firefighting", dict(self.fire_result, bay=self.bay.number, t=utc_iso()))
                    ctx.log(f"T3 FirefightingReport: {self.fire_result}")
                self.m.handle("REPORTED")
        out = super().step(ctx)
        if out.out is not None:
            if self.m.state == "AIM_FIRE":
                on = self.target is not None or self.at_centre
                out.out.info["cannon"] = {"on": on, "bay": self.bay.number, "window": WINDOW_NAME.get(self.target), "window_id": self.target}
            if self.bay is not None:
                out.out.info["bay"] = self.bay.number
            if self.m.state == "SWAY_SEARCH" and self.geom is not None:
                out.out.info["votes"] = {b.number: self._votes(ctx, b) for b in self.geom.bays}
        return out

    def _on_line(self, ctx: TaskContext, reason: str = "") -> None:
        """TO_LINE's end: to the given bay (args.bay, or the entry point's), else the sway search."""
        kw = {"reason": reason} if reason else {}
        if self.bay is not None:
            self.m.handle("ON_LINE_BAY", **kw)
        else:
            self.pass_no = 1
            self.m.handle("ON_LINE", **kw)

    def _recheck(self, ctx: TaskContext) -> None:
        """v0.1.48: the chosen bay's indicator RED for bay_confirm_s (>= 3 sightings, the newest under 1 s old) on the way in = a wrong call."""
        if not self.p["bay_recheck"] or ctx.b.map is None:
            return
        e = ctx.b.map.ents.get(self.bay.entity_id)
        run = []
        for t, c in reversed(getattr(e, "ind_hist", None) or []):
            if t < self.t_approach or c != "red":
                break
            run.append(t)
        now = ctx.b.now
        if len(run) < 3 or run[0] - run[-1] < float(self.p["bay_confirm_s"]) or now - run[0] > 1.0:
            return
        why = f"bay {self.bay.number} reads RED on the way in ({len(run)} sightings in {run[0] - run[-1]:.1f} s)"
        if self.reengaged >= int(self.p["reengage_max"]):
            self.safe_hold = f"T3 {self.m.state}: {why}; {self.reengaged} re-engagement(s) used - holding"
            ctx.log(self.safe_hold)
            self.t_approach = now + 1e6                          # do not repeat it every tick; RESUME goes on into this bay
            return
        self.excluded.add(self.bay.entity_id)
        self.reengaged += 1
        self.back_reason = "colour"
        self.forced = None
        ctx.log(f"T3 wrong call: {why} -> BACK_OUT, re-engagement {self.reengaged}/{self.p['reengage_max']}")
        self.m.handle("WRONG_BAY", reason=why)

    def _decide(self, ctx: TaskContext) -> None:
        """The end of a search pass: the one bay (not excluded) with min_votes GREEN and more GREEN than RED."""
        self._relock(ctx)
        mv = int(self.p["min_votes"])
        votes = {b.number: self._votes(ctx, b) for b in self.geom.bays}
        cands = [b for b in self.geom.bays if b.entity_id not in self.excluded
                 and votes[b.number]["green"] >= mv and votes[b.number]["green"] > votes[b.number]["red"]]
        txt = " · ".join(f"bay {n} R{v['red']} G{v['green']}" for n, v in votes.items())
        rec = {"pass": self.pass_no, "standoff_m": self.pass_sd, "votes": votes, "excluded": self._excluded_numbers(),
               "chosen": cands[0].number if len(cands) == 1 else None, "t": round(ctx.b.now, 1)}
        self.search_log.append(rec)
        if len(cands) == 1:
            self.bay = cands[0]
            ctx.log(f"T3 search pass {self.pass_no}: {txt} -> bay {self.bay.number} (green) face N {self.bay.face[0]:+.1f} E {self.bay.face[1]:+.1f}")
            self.m.handle("CHOSEN", reason=f"bay {self.bay.number} votes GREEN ({txt})")
        elif self.pass_no < 2:
            ctx.log(f"T3 search pass {self.pass_no}: {txt} -> {'a tie' if cands else 'no GREEN bay'}: a second pass at {self.p['standoff2_m']} m")
            self.pass_no += 1                                    # the behaviour restarts on the next tick with pass 2's line
        else:
            self.safe_hold = f"T3 SWAY_SEARCH: no single GREEN bay after two passes ({txt})"
            ctx.log(self.safe_hold)

    def _fire(self, ctx: TaskContext) -> None:
        now = ctx.b.now
        if self.t_fire is None:
            self.t_fire = now
        wins = self.windows(ctx)
        lit = sorted((w for w in wins if w.light == "red"), key=lambda w: -w.n_obs)
        if lit:
            self.saw_red = True
            n = self.window_no(lit[0])
            if self.target != n and (self.target is None or self.target not in [self.window_no(w) for w in lit]):
                ctx.log(f"T3 aim: window {n} ({WINDOW_NAME[n]}) of bay {self.bay.number} is lit RED")
                self.target = n
        greens = [self.window_no(w) for w in wins if w.light == "green"]
        hit = self.target if self.target in greens else (greens[0] if greens and self.target is None else None)
        if hit is not None:
            self.green_since = self.green_since if self.green_since is not None else now
            if now - self.green_since >= float(self.p["green_confirm_s"]):
                self.fire_result = {"window": WINDOW_NAME[hit], "window_id": hit, "result": "green", "seen_red": self.saw_red,
                                    "fired_s": round(now - self.t_fire, 1), "complete": True}
                self.m.handle("FIRED")
                return
        else:
            self.green_since = None
        if self.target is None and not self.at_centre and now - self.t_fire >= float(self.p["window_wait_s"]):
            dark = {self.window_no(w) for w in wins if w.light == "off" and now - w.t_last < 2.0}
            if len(dark) == 1:                                   # one of the two is seen dark: the lit one is the other (the camera misses it)
                self.target = 3 - dark.pop()
                ctx.log(f"T3 aim: no window of bay {self.bay.number} seen lit after {self.p['window_wait_s']:.0f} s, window {3 - self.target} "
                        f"seen dark - at window {self.target} ({WINDOW_NAME[self.target]})")
            else:
                self.at_centre = True
                ctx.log(f"T3 aim: no window of bay {self.bay.number} seen lit after {self.p['window_wait_s']:.0f} s - at the bay centre")
        if now - self.t_fire >= float(self.p["fire_s"]):
            self.fire_result = {"window": WINDOW_NAME.get(self.target), "window_id": self.target, "result": "not seen green",
                                "seen_red": self.saw_red, "fired_s": round(now - self.t_fire, 1), "complete": False}
            self.m.handle("FIRED", reason=f"firing time over ({self.p['fire_s']:.0f} s), no GREEN seen")

    def on_behaviour_done(self, sub: str, ctx: TaskContext) -> None:
        if sub == "FIND_DOCK":
            ctx.log(f"T3 dock: {len(self.geom.bays)} bays, bow-in {self.geom.bow_in_hdg:03.0f}, source {getattr(self.geom, 'source', 'map')}"
                    + (f" (fit rms {self.geom.rms_m:.2f} m)" if getattr(self.geom, "source", "map") == "boxes" else "")   # v0.1.58: which dock source
                    + (f", bay {self.bay.number} given" if self.forced and self.bay else ""))
            self.m.handle("FOUND")
        elif sub == "TO_LINE":
            if self.lining_out and "BEHAVIOURS" in self.__dict__:   # v0.1.56: out on the line - now the path along it as before
                del self.BEHAVIOURS
                ctx.log("T3 TO_LINE: out on the standoff line")
                return
            self._on_line(ctx)
        elif sub == "SWAY_SEARCH":
            self._decide(ctx)
        elif sub == "BACK_OUT":
            if self.back_reason == "colour":
                self.bay = None
                self.pass_no = 1
                self.m.handle("BACKED")
            else:
                self.m.handle("REALIGN")
        else:
            ev = {"SWAY_TO_ABEAM": "ABEAM", "TURN_IN": "ALIGNED", "ENTER": "IN", "DOCKED": "SETTLED", "UNDOCK": "OUT"}.get(sub)
            if ev:
                self.m.handle(ev)

    def on_behaviour_abort(self, sub: str, reason: str, ctx: TaskContext) -> None:
        if sub == "FIND_DOCK":
            if self._found(ctx, n_min=2):                       # a full turn: two faces are a line too
                ctx.log(f"T3 dock: only {len(self.geom.bays)} bays after a full turn - going on with them")
                self.on_behaviour_done(sub, ctx)
                return
            self.safe_hold = f"T3 FIND_DOCK: {reason}{self._boxes_why(ctx)}"
            ctx.log(self.safe_hold)
            return
        if sub in ("TURN_IN", "ENTER") and self.reengaged < int(self.p["reengage_max"]):
            self.reengaged += 1
            self.back_reason = "geometry"
            ctx.log(f"T3 {sub}: {reason} -> BACK_OUT, re-engagement {self.reengaged}/{self.p['reengage_max']}")
            self.m.handle("MISALIGNED", reason=reason)
            return
        if sub in ("SWAY_SEARCH", "BACK_OUT"):
            self.safe_hold = f"T3 {sub}: {reason}"
            ctx.log(self.safe_hold)
            return
        super().on_behaviour_abort(sub, reason, ctx)
