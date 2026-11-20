"""Task 1 Safe Passage, core tier (handbook 3.3.2; ROADMAP WP2; mission code m1t1):

WAIT_START -> SEARCH_ENTRY (ScanYaw until a FLASHING blue buoy is in the map) -> CIRCLE_ENTRY_CW (Orbit) -> TRANSIT
(FollowPath through one gate at a time: pre -> mid -> post, GREEN to port / RED to starboard) -> ... -> EXIT (FollowPath to
the SOLID blue buoy's circle) -> CIRCLE_EXIT_CCW (Orbit) -> DONE.

The next gate is chosen on the PerceptionMap each time (gatepath.choose_next); a gate that could only be crossed with RED
to port is refused. No usable gate ahead (missing, or refused) and no exit known -> SCAN (hold, turn on the spot); a
scan that finds nothing asks for AUTO_SAFE_HOLD (decided 2026-11-02: hold, then scan); RESUME scans again. The leg comes
from . import uav_gate as UG
from the planner chosen by `t1.planner` (simple | s6) or START_TASK args.planner. SafePassageReport bodies (event
`safe_passage`) go out whenever the mapped buoys change and once complete at DONE.

v0.1.39 (WP10h, the pathing spec's leads): a PENDING buoy (the map has seen a light on it, too rarely to call it lit or unlit) nearer
than the next gate -> LOOK (hold facing it, `look_s` at most) before a gate further on is taken: a flashing gate buoy sampled in
its dark phase is otherwise taken for an obstacle and its gate skipped. A SEARCH_ENTRY scan that finds no entry but has a far
sighting (a ray, 25-40 m) -> APPROACH_LEAD (`lead_step_m` toward it, then scan again; `lead_tries` times) before the safe hold.

v0.1.54 (Dmitrii: "S6 planner ... couldn't find the start from where it was. Then it found a dark buoy and started to orbit it"; the
fast SIL reproduced it with both planners - the planner was not the cause): (1) a dark buoy (or a flashing RED / GREEN one) far
away collects a few wrong-colour frames (the camera's far miscolour) and reads "blue, flashing" for a while - the task locked the
first such buoy and circled it. Now the locked entry is WATCHED on the way to its circle: seen `verify_n` times within
`verify_r_m` (where the camera's colour is clean) and never blue, it is rejected (never taken again in this run) and the search
starts again (NOT_ENTRY); the same for the exit (NOT_EXIT -> SCAN). (The map's own vote can waver on a real flashing light, so the
near sightings decide, not the map's colour.) The entry / exit is the best-evidenced blue of its light among the buoys
not rejected (was: PerceptionMap.markers(), every buoy). (2) A scan that finds no flashing blue now also takes a MAP LEAD before
the ray lead: the nearest buoy that is blue but its light still undecided, or still pending (far votes weigh little, so a far
flashing blue stays "?" - the start search failed from the far end of the field) -> APPROACH_LEAD to `lead_stop_m` short of it.
(3) The clearance rule (gatepath.planner.Clearance, t1.clear_m 0.60): every path keeps the hull 0.60 m off every mapped object
(buoys of any colour, box faces, dock faces), the circles pick their radius (circle_radius) and approach the ring along an avoided
path (Orbit `approach`), and each leg logs its planned clearance.

v0.1.66 (WP16, D22-D22c of docs/OVERNIGHT_2026-11-07_UAV.md) - m1t2 = T1 ADVANCED (START_TASK tier advanced, or colour_source uav): the
boat is colour-blind for buoys coloured only from above; the UAV's colours come into the PerceptionMap (ingest_geo, linked within 1 m
to the camera's buoys, which keep their position). The handbook's order (3.3.2 Advanced: the UAV overflies the start area and reports
before the USV transits): WAIT_START -> WAIT_UAV_MAP (station-keep outside the field) until the UAV's scan.complete, or the drone back on
the ground after having flown with no UAV item for 5 s, or the operator's `uav scan done` - and the UAV layer must hold an ENTRY
candidate (a flashing blue), else a CAUTION until t1.uav_wait_s (420 s), then AUTO_SAFE_HOLD (`uav scan done` forces the exit). The UAV
layer is frozen for the attempt. Then SEARCH_ENTRY as in m1t1, with `uav_lead` (the nearest UAV-only blue / undecided buoy, then the
nearest provisional gate) after the map lead; under colour_source uav the locked entry / exit is not re-checked by the camera's colour
(the colour-blind camera never sees blue - the UAV's colour decides). m1t1 is unchanged.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from ...companion.fsm import Transition
from ...companion.state import RobotXTask, utc_iso
from ..behaviours import FollowPath, Orbit, ScanYaw, StationKeep
from ..gatepath import planner as gp
from . import uav_gate as UG  # noqa: E402  (v0.1.89: the touchdown sequence)
from .base import Task, TaskContext, simple_machine

DEFAULTS = {"planner": "simple", "circle_r_m": 3.0, "circle_v": 0.5, "transit_v": 0.6, "gate_d_m": 2.0,
            "s6_d_m": 3.0, "rmin_m": 1.83,      # s6: the spec's d and the best tuning's rmin (pathing_sim spec 10, cfg 31)
            "min_obs": 5, "la_m": 2.0, "scan_rate_dps": 20.0, "scan_turns": 1.0, "report_period_s": 2.0, "corridor_m": 4.0,
            "look_s": 8.0, "look_r_m": 15.0, "lead_step_m": 12.0, "lead_tries": 4,                # v0.1.39 (WP10h)
            "lead_stop_m": 9.0,                                                                  # v0.1.54: a map lead stops this short
            "scout_entry_s": 420.0,             # v0.1.92: SEARCH_ENTRY's scouting budget (0 = off: v0.1.91's scan + leads + hold); Cowork: 300 = 3 visits
            "check_r_m": 7.0, "check_s": 6.0,   # v0.1.92 (Cowork consult): checked from within 7 m (12 Nov's one lantern read at 7 m) for 6 s
            "check_hold_m": 5.0,                # v0.1.92: a farther candidate is approached to this distance first (outside SLOW, ~100 px of buoy)
            "check_tries": 2, "scout_standoff_m": 3.0,   # v0.1.92: checks per buoy (the 2nd from 2 m closer); the colour-blind step's stand-off
            "scout_lead_tries": 2,
            "steady_n": 10,                     # v0.1.92: a locked entry seen this often up close, all blue, never dark = steady = the exit
            # v0.1.92 FINAL STRATEGY (Dmitrii + Cowork addendum 6 §6.4, 13 Nov 02:30): the start, the look-lock, the gate steps
            "start_mode": "straight",           # advance360 (the course profile): start_advance_m at start_v, one turn on the spot | straight
            "start_advance_m": 2.0, "start_v": 0.2, "start_scan_s": 200.0,
            "no_entry_finale": False,           # v0.1.93.2 (Dmitrii 13 Nov; the course profile: true): instead of the safe hold when no entry is
            "final_advance_m": 3.0, "final_scan_rate_dps": 20.0,   # found - final_advance_m ahead, one 360 (the yaw cap: 8 deg/s), DONE
            "start_scan_stop_blue": False,      # v0.1.93.2: end the start turn at a flashing blue lantern in the map ...
            "start_scan_stop_r_m": 10.0,        #   ... within this range, seen 2 x min_obs times (a far one misread: the sim's wrong entry)
            "look_lock": True, "look_lock_s": 4.0,   # stop, bow on the suspect, before approaching it
            "check_still_deg": 0.0,             # v0.1.94: LOOK_LOCK / CHECK_BLUE stop yawing once facing within this (0 = the old hunting)
            "look_lock_tol_deg": 10.0,          # v0.1.93.2: the bow within this of the suspect ends the lock (the course profile: 5)
            "approach_v": 0.2, "scout_v": 0.3,  # to a suspect / the colour-blind step
            "suspect_blue_share": 0.3,          # a suspect: undecided, or this share of its lit votes blue; unlit (off) / resolved red / green never
            "gate_steps": False,                # True (the course profile): TRANSIT = to the pre-point -> ALIGN_GATE -> THROUGH -> passed / missed
            "through_v": 0.12, "la_through_m": 1.5,
            # v0.1.94 (Dmitrii: "if the buoy / gate is mapped correctly, we can go through it with more confidence and comfortable speed"):
            # a CONFIDENT gate (both buoys >= conf_min_obs sightings, scatter <= conf_sigma_m, seen from <= conf_seen_m once, coloured by
            # their lamps, not lucky) runs in at transit_v_conf and through at through_v_conf; 0 = off (the plain speeds everywhere)
            "through_v_conf": 0.0, "transit_v_conf": 0.0, "conf_min_obs": 10, "conf_sigma_m": 0.5, "conf_seen_m": 8.0, "post_m": 1.5, "pre_end_r_m": 0.35,
            "lead_max_m": 3.0, "lead_min_m": 1.5,   # the pre-point: min(gate spacing - 1.5, 3.0), never behind the previous gate
            "align_deg": 6.0, "align_max_s": 90.0, "miss_tries": 2, "pass_margin_m": 0.25,
            "geofence_m": 1.0,                  # targets only inside the RxCourse polygon + this
            "turn_first_deg": 0.0,              # > 0: a carrot further off the bow -> hold and turn on the spot first (course profile: 25)
            "keep_clear_m": None, "keep_track_m": None,   # T1's planner keep (None = clear_m / track_m, shared with the dock standoff)
            # v0.1.93 (WP49 §3.5, Dmitrii 02:40: "bow-following with squeeze as a default"): an UNKNOWN object (a guard ring) in a gate's
            # run-in / through is passed on the wider gap beside it when that gap is >= beam + 2 margin, at squeeze_v; that object's guard STOP 0.3 m
            "squeeze": False, "squeeze_r_m": 0.5, "squeeze_margin_m": 0.25, "squeeze_v": 0.08, "squeeze_stop_m": 0.3,
            "confirm_entry": False,             # v0.1.92: True = a scan's entry not confirmed by the LED reader is CHECKED from check_hold_m first.
                                                #   OFF: the final 20-seed sweep - DONE A 13 B 7 C 15 D 5 /20 with it vs 17 15 17 16 without (NOT_ENTRY
                                                #   undoes the rare far false lock near; the detour costs the run)
            "circle_exit_tangent": False,       # v0.1.92: leave the entry circle when the bow points at gate 1's run-in (<= 1 more lap)
            "lead_in_m": 0.0,                   # v0.1.92: the run-in point before a gate the boat is not lined up with (course profile: 5)              # v0.1.92: the leads' cap while the scout step is on (4 x 12 m legs learn nothing)
            "lucky_pairs": "one_known",         # v0.1.92: one_known (a known red / green + an uncertain one) | any (v0.1.90: two unknowns too)
            "lucky": False,                     # v0.1.90: uncertain-colour pairs as gates; no flashing blue -> straight to the gates
            "lucky_blue": True,                 # v0.1.94: with lucky on, a blue whose light stays undecided after check_tries checks is orbited as the entry
                                                # v0.1.94.1 "any" (blue-agnostic): any blue not read as steady is orbited after its first check
            "lucky_exit_orbit": True,           # v0.1.94.2 (Dmitrii): lucky, the search spent, no entry but the exit known -> orbit the exit, then the gates
            "lucky_retry": True,                # v0.1.94.2: lucky without a decided entry, a flashing blue decided later -> circle it CW, the gates again (once)
            "circle_v_max": 0.0, "circle_r_for_max": False,   # v0.1.94.2: the orbit's drift boost (0 = off): up to this when pushed back
            "lucky_v_k": 1.25,                  # v0.1.94.2: lucky moves faster - transit / approach / start / scout speeds x this (circles, gates' lines stay)
            "verify_r_m": 12.0, "verify_n": 8, "replan_s": 1.0,
            "search_lat": None, "search_lon": None, "report_move_m": 1.0, "dedup_m": 1.2,       # v0.1.54: where the field is (the course map) - the last lead, from anywhere           # v0.1.54: the locked entry / exit seen this many times this near and never blue = not it
            "clear_m": 0.60, "track_m": 0.45, "hull_len_m": 1.19, "hull_beam_m": 0.76, "circle_r_max_m": 5.0,   # v0.1.54: the clearance rule
            "colour_source": "camera", "uav_wait_s": 420.0, "uav_quiet_s": 5.0, "tier": None}                     # v0.1.66: m1t2
STATES = ["WAIT_START", "WAIT_UAV_MAP", "START_ADVANCE", "START_SCAN", "SEARCH_ENTRY", "LOOK_LOCK", "SCOUT_ENTRY", "CHECK_BLUE",
          "ALIGN_GATE", "THROUGH", "APPROACH_LEAD", "CIRCLE_ENTRY_CW", "TRANSIT", "LOOK", "SCAN", "EXIT", "CIRCLE_EXIT_CCW",
          "FINAL_ADVANCE", "FINAL_SCAN"]     # v0.1.93.2 (Dmitrii 13 Nov): no blue buoy found - 3 m ahead, a quick 360, end the mission


def _seg_near(q, poly, d: float) -> bool:
    """v0.1.92: q within d of the polygon's edges (the geofence's position tolerance)."""
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        dx, dy = b[0] - a[0], b[1] - a[1]
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 <= 0 else max(0.0, min(1.0, ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / L2))
        if math.hypot(q[0] - (a[0] + t * dx), q[1] - (a[1] + t * dy)) <= d:
            return True
    return False


NEXT = {"WAIT_START": "START_ADVANCE", "WAIT_UAV_MAP": "SEARCH_ENTRY (the drone's map)", "START_ADVANCE": "START_SCAN (one turn on the spot)",
        "START_SCAN": "SEARCH_ENTRY", "SEARCH_ENTRY": "LOOK_LOCK on a suspect / scout", "LOOK_LOCK": "approach the suspect (SCOUT_ENTRY)",
        "SCOUT_ENTRY": "SEARCH_ENTRY", "CHECK_BLUE": "flashing: CIRCLE_ENTRY_CW · steady: the exit, search on", "APPROACH_LEAD": "SEARCH_ENTRY",
        "CIRCLE_ENTRY_CW": "TRANSIT to gate 1", "TRANSIT": "ALIGN_GATE", "ALIGN_GATE": "THROUGH", "THROUGH": "the next gate / EXIT",
        "LOOK": "TRANSIT", "SCAN": "TRANSIT / EXIT", "EXIT": "CIRCLE_EXIT_CCW", "CIRCLE_EXIT_CCW": "DONE", "DONE": "-",
        "FINAL_ADVANCE": "FINAL_SCAN (a quick 360)", "FINAL_SCAN": "the entry seen: SEARCH_ENTRY · else DONE (mission ended)"}


class T1SafePassage(Task):
    name = "T1"
    robo_task = RobotXTask.T1
    BEHAVIOURS = {"WAIT_START": StationKeep, "WAIT_UAV_MAP": StationKeep, "SEARCH_ENTRY": ScanYaw, "SCOUT_ENTRY": FollowPath, "CHECK_BLUE": ScanYaw,
                  "START_ADVANCE": FollowPath, "START_SCAN": ScanYaw, "LOOK_LOCK": ScanYaw, "ALIGN_GATE": ScanYaw, "THROUGH": FollowPath, "APPROACH_LEAD": FollowPath, "CIRCLE_ENTRY_CW": Orbit,
                  "TRANSIT": FollowPath, "LOOK": ScanYaw, "SCAN": ScanYaw, "EXIT": FollowPath, "CIRCLE_EXIT_CCW": Orbit,
                  "FINAL_ADVANCE": FollowPath, "FINAL_SCAN": ScanYaw}
    SKIP_REFUSE = {"WAIT_START": "a timer", "WAIT_UAV_MAP": "the UAV", "SEARCH_ENTRY": "the map", "CHECK_BLUE": "the map", "START_SCAN": "a timer", "LOOK_LOCK": "the map", "LOOK": "the map", "SCAN": "the map"}   # v0.1.68
    NEEDS_FIELD = True     # v0.1.85 (WP46)
    SPEED_CLASS = {"THROUGH": "gate"}   # v0.1.94.3: the live gate knob; the circles follow circle, the other paths cruise (tasks/base.speed_class)
    UNDERWAY = True        # v0.1.85 (WP41): under guidance.mode underway the base task steers its carrots by usv/guidance.py (no logic change here)
    # v0.1.26 (WP7): `ocs_cli start_task m1t1 --from TRANSIT`
    ENTRY_POINTS = {"SEARCH_ENTRY": "nothing", "TRANSIT": "a usable gate ahead in the map (no entry circle)",
                    "EXIT": "the exit buoy in the map (no gates)"}

    def build_machine(self, on_transition, clock):
        return simple_machine("T1", STATES, [
            Transition("GO", ["WAIT_START"], "SEARCH_ENTRY", reason="started"),
            Transition("FINALE", ["SEARCH_ENTRY"], "FINAL_ADVANCE", reason="no blue buoy found: 3 m ahead, one quick turn, then the end"),
            Transition("FINAL_ADVANCED", ["FINAL_ADVANCE"], "FINAL_SCAN", reason="advanced: one quick turn on the spot"),
            Transition("FINAL_ENTRY", ["FINAL_SCAN"], "SEARCH_ENTRY", reason="a flashing blue seen in the last turn: the entry sequence"),
            Transition("FINAL_END", ["FINAL_SCAN"], "DONE", reason="no blue buoy: the mission ended"),
            Transition("GO_START", ["WAIT_START"], "START_ADVANCE", reason="started: a short advance, then one turn on the spot (v0.1.92)"),
            Transition("ADVANCED", ["START_ADVANCE"], "START_SCAN", reason="advanced: one full turn on the spot"),
            Transition("START_SCANNED", ["START_SCAN"], "SEARCH_ENTRY", reason="the start turn done: the ladder"),
            Transition("LOCK", ["SEARCH_ENTRY", "CHECK_BLUE", "SCOUT_ENTRY", "LOOK_LOCK"], "LOOK_LOCK", reason="a suspect: stop, bow on it"),
            Transition("LOCKED", ["LOOK_LOCK"], "SCOUT_ENTRY", reason="locked on the suspect: approach it"),
            Transition("LOCK_CHECK", ["LOOK_LOCK"], "CHECK_BLUE", reason="locked on a near suspect: read its light"),
            Transition("ALIGN", ["TRANSIT"], "ALIGN_GATE", reason="at the pre-point: turn onto the gate line"),
            Transition("THRU", ["ALIGN_GATE", "TRANSIT"], "THROUGH", reason="lined up: through on the gate line"),
            Transition("THRU_DONE", ["THROUGH"], "TRANSIT", reason="through: the next gate"),
            Transition("GO_UAV", ["WAIT_START"], "WAIT_UAV_MAP", reason="m1t2: the UAV surveys first (handbook 3.3.2 Advanced)"),
            Transition("UAV_MAP", ["WAIT_UAV_MAP"], "SEARCH_ENTRY", reason="the UAV's map is in: transit"),
            Transition("ENTRY_SEEN", ["SEARCH_ENTRY"], "CIRCLE_ENTRY_CW", reason="flashing blue entry buoy in the map"),
            Transition("LEAD", ["SEARCH_ENTRY"], "APPROACH_LEAD", reason="no entry in view: toward a far sighting (a ray)"),
            Transition("SCOUT", ["SEARCH_ENTRY", "CHECK_BLUE", "SCOUT_ENTRY"], "SCOUT_ENTRY", reason="no entry yet: to the next buoy (v0.1.92)"),
            Transition("CHECK", ["SEARCH_ENTRY", "SCOUT_ENTRY", "CHECK_BLUE"], "CHECK_BLUE", reason="a blue / undecided buoy near: read its light"),
            Transition("SCOUTED", ["SCOUT_ENTRY"], "SEARCH_ENTRY", reason="at the buoy: scan again"),
            Transition("ENTRY_CHECKED", ["CHECK_BLUE"], "CIRCLE_ENTRY_CW", reason="the checked buoy flashes blue: the entry"),
            Transition("CHECKED", ["CHECK_BLUE"], "SEARCH_ENTRY", reason="checked, not the entry: search on"),
            Transition("LUCKY_GO", ["SEARCH_ENTRY", "SCOUT_ENTRY", "CHECK_BLUE", "LOOK_LOCK"], "TRANSIT", reason="lucky: no flashing blue entry - straight to the closest gate"),
            Transition("LUCKY_EXIT", ["SEARCH_ENTRY", "SCOUT_ENTRY", "CHECK_BLUE", "LOOK_LOCK"], "CIRCLE_ENTRY_CW", reason="lucky: no entry, the exit known - orbit it, then the gates"),
            Transition("LUCKY_RETRY", ["TRANSIT", "SCAN", "LOOK", "EXIT"], "CIRCLE_ENTRY_CW", reason="lucky: the real entry decided late - circle it, the gates again"),
            Transition("ARRIVED", ["APPROACH_LEAD"], "SEARCH_ENTRY", reason="along the ray: scan again"),
            Transition("LOOK", ["TRANSIT"], "LOOK", reason="a pending buoy nearer than the next gate: look at it"),
            Transition("LOOKED", ["LOOK"], "TRANSIT", reason="the pending buoy decided (or looked at long enough)"),
            Transition("CIRCLED", ["CIRCLE_ENTRY_CW"], "TRANSIT", reason="entry circled clockwise"),
            Transition("NOT_ENTRY", ["CIRCLE_ENTRY_CW"], "SEARCH_ENTRY", reason="the locked buoy is not the flashing blue: search again"),
            Transition("NOT_EXIT", ["EXIT", "CIRCLE_EXIT_CCW"], "SCAN", reason="the exit buoy is not blue: look again"),
            Transition("NO_GATE", ["TRANSIT"], "SCAN", reason="no usable gate ahead"),
            Transition("FOUND", ["SCAN"], "TRANSIT", reason="a gate ahead"),
            Transition("TO_EXIT", ["TRANSIT", "SCAN", "THROUGH"], "EXIT", reason="no gate left, exit buoy known"),
            Transition("AT_EXIT", ["EXIT"], "CIRCLE_EXIT_CCW", reason="at the exit circle"),
            Transition("CIRCLED", ["CIRCLE_EXIT_CCW"], "DONE", reason="exit circled counter-clockwise"),
        ], initial="WAIT_START", on_transition=on_transition, clock=clock)

    # ---- set-up -------------------------------------------------------------------------------------------------
    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update(self.cfg.extra.get("t1", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS})
        if p["planner"] not in ("simple", "s6"):
            raise ValueError(f"t1 planner {p['planner']!r}: simple | s6")
        if str(p.get("tier") or "").lower() == "advanced":           # v0.1.66: m1t2
            p["colour_source"] = "uav"
        if p.get("lucky") and float(p.get("lucky_v_k") or 1.0) != 1.0:      # v0.1.94.2 (Dmitrii: "in lucky mode move with higher speed")
            for k in ("transit_v", "approach_v", "start_v", "scout_v", "transit_v_conf"):
                if float(p.get(k) or 0) > 0:
                    p[k] = round(float(p[k]) * float(p["lucky_v_k"]), 3)
        self.p = p
        self.lucky_unsure = False                       # v0.1.94.2: lucky took the gates without a decided (flashing) entry
        self.lucky_exit_id: Optional[int] = None        #   the exit being orbited CW as the stand-in entry
        self.lucky_from_exit = False                    #   that orbit done: the gates in their own direction, then the exit CCW
        self.lucky_retried = False
        self.turn_first_deg = float(p.get("turn_first_deg") or 0.0) or None   # v0.1.92: read by base._underway -> guidance.Steer
        self.uav_mode = p.get("colour_source") == "uav"
        self.uav_wait_t0: Optional[float] = None
        self.uav_wait_why: Optional[str] = None
        self.uav_caution_t = -1e9
        if ctx.b.map is not None:                                     # the map follows the task (D22b / D22c)
            ctx.b.map.colour_source = "uav" if self.uav_mode else "camera"
            ctx.b.map.uav_frozen = self.uav_mode
        self.passed: List[Dict[str, Any]] = []         # {mid, dir, key, correct}
        self.current: Optional[gp.GateItem] = None
        self.last_dir: Optional[Tuple[float, float]] = None
        self.refused: Optional[str] = None
        self.safe_hold: Optional[str] = None            # read (and cleared) by the mission manager
        self.entry_xy: Optional[Tuple[float, float]] = None
        self.exit_xy: Optional[Tuple[float, float]] = None
        self.report_sig = None
        self.report_t = -1e9
        self.report_xy: Dict[int, Tuple[float, float]] = {}
        self.field: Dict[int, dict] = {}                # v0.1.54: every buoy reported so far, its latest state (the map may drop it)
        self.rid: Dict[int, int] = {}                   # v0.1.94 (Dmitrii: "ids from 0 to 9"): map id -> the report's id, in first-reported order
        self._next_rid = 0
        self.leg_how = ""
        self.skipped: set = set()                       # v0.1.39: pending buoys already looked at
        self.look_id: Optional[int] = None
        self.look_xy: Optional[Tuple[float, float]] = None
        self.lead_tries = 0
        self.scout_t0: Optional[float] = None           # v0.1.92: the scouting for the entry (SCOUT_ENTRY / CHECK_BLUE)
        self.checks: Dict[int, int] = {}                # id -> checks done
        self.checked: set = set()                       # decided (or given up): not checked again
        self.exit_seen: set = set()                     # steady blue found while searching the entry: the exit, never orbited now
        self.visited: List[Tuple[float, float]] = []    # the colour-blind steps' buoys
        self.doubtful: set = set()                      # undecided after check_tries: re-checked once when the budget is spent
        self.gate_misses: Dict[tuple, int] = {}         # v0.1.92: gate key -> missed passes
        self.thru = None                                # v0.1.92: the THROUGH leg {key, mid, dir, width, pre, post, crossed, lat}
        self.why = "starting"                           # v0.1.92: the micro-stage's reason (the OCS stage line)
        self.squeeze_xy = None                          # v0.1.93: the squeezed object's face (the guard's 0.3 m stop for it)
        self.started_scan = False
        self.rechecked, self.extra_s = False, 0.0
        self.check_id: Optional[int] = None
        self.check_xy: Optional[Tuple[float, float]] = None
        self.scout_goal: Optional[Tuple[float, float]] = None
        self.scout_then_check = False
        self.lead_goal: Optional[Tuple[float, float]] = None
        self.entry_id: Optional[int] = None             # v0.1.54: the locked buoys, watched while circled
        self.exit_id: Optional[int] = None
        self.rejected: set = set()                      # v0.1.54: buoys locked and then decided not blue
        self.blue_fix_id = None                         # v0.1.93.2: the blue lantern that ended the start turn
        self.finale_done = False                        # v0.1.93.2: the no-entry finale runs once
        self.near: Dict[int, list] = {}                 # v0.1.54: id -> [(t_last, colour)] sightings of a locked buoy from near
        self.lead_ids: set = set()                      # v0.1.54: map leads already approached
        self.circle_r: Dict[str, float] = {}
        self.clr = gp.Clearance(clear_m=float(p["keep_clear_m"] if p.get("keep_clear_m") is not None else p["clear_m"]),   # v0.1.92: T1's own
                                hull_len_m=float(p["hull_len_m"]), hull_beam_m=float(p["hull_beam_m"]),
                                track_m=float(p["keep_track_m"] if p.get("keep_track_m") is not None else p["track_m"]))
        self.min_planned: Optional[float] = None        # the smallest planned hull-edge clearance of any leg (the log, the tests)
        self.replan_t, self.plan_sig, self.replans = -1e9, None, 0
        ctx.log(f"T1 started: planner {p['planner']}, circles r {p['circle_r_m']} m at {p['circle_v']} m/s"
                + (" - m1t2: colour from the UAV, WAIT_UAV_MAP first" if self.uav_mode else ""))

    def precheck(self, ctx: TaskContext) -> str:
        """v0.1.66 (D22b): m1t2 needs the UAV - a fresh UAV status in ocs/state (the mission manager copies it into the map)."""
        if not self.uav_mode:
            return ""
        u = getattr(ctx.b.map, "uav_ocs", None) or {}
        if not u.get("active"):
            return "m1t2 needs the UAV (no fresh UAV status at the OCS: the adapter / the Orin's listener)"
        return ""

    def check_entry(self, stage: str, ctx: TaskContext) -> str:
        if stage == "TRANSIT":
            ch = self.choose(ctx)
            if ch.item is None:
                return "TRANSIT needs a usable gate ahead in the map - none now"
            if ch.wrong_side:
                return f"TRANSIT: the nearest gate {ch.item.key} would pass RED to port from here"
        if stage == "EXIT" and self.marker(ctx, "exit") is None:
            return "EXIT needs the exit buoy (steady blue) in the map - not there now"
        return ""

    def pcfg(self, ctx: TaskContext) -> gp.PlannerCfg:
        d = float(self.p["s6_d_m"] if self.p["planner"] == "s6" else self.p["gate_d_m"])
        return gp.PlannerCfg(kind=self.p["planner"], d=d, rmin=float(self.p["rmin_m"]),
                             min_obs=int(self.p["min_obs"]), obstacles=self.obstacles(ctx), clr=self.clr,
                             lead_in_m=float(self.p.get("lead_in_m") or 0.0))

    def obstacles(self, ctx: TaskContext, skip=()) -> List[tuple]:
        """v0.1.54: every mapped course object as (x, y, r) (gatepath.map_obstacles: buoys of any colour, box and dock faces)."""
        return gp.map_obstacles(ctx.b.map, int(self.p["min_obs"]), skip)

    def _planned(self, ctx: TaskContext, what: str, path, obstacles, skip_end_m: float = 0.0) -> None:
        c = gp.path_clearance(path, obstacles, self.clr, skip_end_m)
        self.min_planned = c if self.min_planned is None else min(self.min_planned, c)
        if c < self.clr.clear_m:
            ctx.log(f"T1 {what}: planned clearance {c:.2f} m < {self.clr.clear_m:.2f} m (the geometry is narrower)")

    # ---- the map ---------------------------------------------------------------------------------------------------
    def buoys(self, ctx: TaskContext):
        m = ctx.b.map
        return [e for e in m.entities("buoy") if e.n_obs >= int(self.p["min_obs"])] if m is not None else []

    def marker(self, ctx: TaskContext, which: str):
        """entry = a FLASHING blue buoy until the task has locked one (CIRCLE_ENTRY_CW); exit = then any other well-seen blue
        buoy, the SOLID one first - a far steady blue can look flashing (wrong-colour frames read as the off phase)."""
        m = ctx.b.map
        if m is None:
            return None
        if self.entry_xy is None:                       # v0.1.54: as PerceptionMap.markers(), the rejected buoys left out
            light = "flash" if which == "entry" else "steady"
            c = [e for e in m.entities("buoy") if e.colour == "blue" and e.id not in self.rejected
                 and (e.light == light or (which == "exit" and e.id in getattr(self, "exit_seen", ())))   # v0.1.94.2: a checked exit stays it
                 and not (which == "entry" and e.id in getattr(self, "exit_seen", ())) and self._in_course(ctx, (e.x_n, e.y_e))]
            return max(c, key=lambda e: e.n_obs) if c else None
        if which == "entry":
            return None
        blues = [e for e in m.entities("buoy") if e.colour == "blue" and e.n_obs >= int(self.p["min_obs"]) and e.id not in self.rejected
                 and math.hypot(e.x_n - self.entry_xy[0], e.y_e - self.entry_xy[1]) > 3.0 and self._in_course(ctx, (e.x_n, e.y_e))]
        if not blues:
            return None
        return max(blues, key=lambda e: (e.light == "steady", e.n_obs))

    def items(self, ctx: TaskContext) -> List[gp.GateItem]:
        if ctx.b.map is None:
            return []
        out = []
        mem = self.__dict__.setdefault("gate_mem", [])      # v0.1.92 (Cowork addendum 2): the gate memory - every gate seen stays until passed,
        live = gp.gate_items(ctx.b.map, int(self.p["min_obs"]), lucky=bool(self.p.get("lucky")),          # whatever the map forgets
                             lucky_pairs=str(self.p.get("lucky_pairs") or "one_known"))
        for it in live:
            j = next((i for i, q in enumerate(mem) if math.hypot(it.mid[0] - q.mid[0], it.mid[1] - q.mid[1]) < 1.5), None)
            if j is None:
                mem.append(it)
            else:
                mem[j] = it                                  # the freshest position of the same gate
        if not self.p.get("gate_steps"):
            mem[:] = list(live)                              # v0.1.91's behaviour: the live map only
        for it in mem:
            if any(math.hypot(it.mid[0] - q["mid"][0], it.mid[1] - q["mid"][1]) < 1.5 for q in self.passed):
                continue
            out.append(it)
        return out

    def choose(self, ctx: TaskContext) -> gp.Choice:
        pose = ctx.b.pose
        cur = (pose.x_n, pose.y_e) if pose else (0.0, 0.0)
        items = self.items(ctx)
        if self.p.get("gate_steps") and self.entry_xy is not None and items:   # v0.1.92: the gate memory, ordered from the entry outwards
            ax = self._axis(ctx, items)
            order = sorted(items, key=lambda it: (it.mid[0] - self.entry_xy[0]) * ax[0] + (it.mid[1] - self.entry_xy[1]) * ax[1])
            it = order[0]
            if it.lucky and it.dir[0] * ax[0] + it.dir[1] * ax[1] < 0:      # a lucky pair: crossed the way we go
                it = gp.GateItem(it.key, it.green, it.red, it.mid, (-it.dir[0], -it.dir[1]), it.width, lucky=True)
            return gp.Choice(item=it, wrong_side=False, candidates=len(items))
        if self.p.get("gate_steps") and getattr(self, "lucky_from_exit", False) and items:   # v0.1.94.2: no entry - upstream gate first
            dx = sum(it.dir[0] for it in items if not it.lucky) or sum(it.dir[0] for it in items)
            dy = sum(it.dir[1] for it in items if not it.lucky) or sum(it.dir[1] for it in items)
            ax = gp._unit(dx, dy)
            it = min(items, key=lambda it: it.mid[0] * ax[0] + it.mid[1] * ax[1])
            return gp.Choice(item=it, wrong_side=False, candidates=len(items))
        return gp.choose_next(items, cur, self.last_dir)

    def _lucky_go(self, ctx: TaskContext, why: str) -> bool:
        """v0.1.94.2 (Dmitrii): lucky with no entry - the exit known (lucky_exit_orbit): orbit it CW as the stand-in entry; else
        straight to the closest gate (v0.1.90). Either way lucky_unsure: a flashing blue decided later is circled and the gates rerun."""
        p = self.p
        ext = None
        if p.get("lucky_exit_orbit", True) and ctx.b.map is not None:
            ents = {e.id: e for e in ctx.b.map.entities("buoy")}
            ext = next((ents[i] for i in sorted(self.exit_seen) if i in ents), None) or self.marker(ctx, "exit")
        if ext is not None:
            self.lucky_unsure, self.lucky_exit_id = True, ext.id
            self.entry_xy, self.entry_id = (ext.x_n, ext.y_e), ext.id
            ctx.log(f"T1 LUCKY: {why}, the exit #{ext.id} known - orbiting it as the stand-in entry, then the gates")
            self.m.handle("LUCKY_EXIT")
            return True
        it = self.choose(ctx).item
        if it is None:
            return False
        self.lucky_unsure = True
        ctx.log(f"T1 LUCKY: {why} - straight to the closest gate {it.key}{' (uncertain colours)' if it.lucky else ''}")
        self.current, self.refused = it, None            # v0.1.94.1: the gate before TRANSIT starts (crashed the sim)
        self.m.handle("LUCKY_GO")
        return True

    def _lucky_retry(self, ctx: TaskContext) -> bool:
        """v0.1.94.2 (Dmitrii: "if during the course we determine the entry, re-orbit entry and retry the run"): once, between gates."""
        p = self.p
        if (not p.get("lucky") or not p.get("lucky_retry", True) or not self.lucky_unsure or self.lucky_retried
                or self.m.state not in ("TRANSIT", "SCAN", "LOOK", "EXIT") or ctx.b.map is None):   # never mid-gate (ALIGN / THROUGH)
            return False
        c = [e for e in ctx.b.map.entities("buoy") if e.colour == "blue" and e.light == "flash" and getattr(e, "led", None) is not None
             and e.id not in self.rejected and e.id not in self.exit_seen and e.id != self.entry_id and e.id != self.lucky_exit_id
             and e.n_obs >= int(p["min_obs"]) and self._in_course(ctx, (e.x_n, e.y_e))]
        if not c:
            return False
        e = max(c, key=lambda e: e.n_obs)
        self.lucky_retried, self.lucky_unsure, self.lucky_from_exit = True, False, False
        if self._beh is not None:
            self._beh.cancel()
            self._beh = None
        self.entry_xy, self.entry_id = (e.x_n, e.y_e), e.id
        n = len(self.passed)
        self.passed, self.gate_misses, self.current, self.thru, self.last_dir = [], {}, None, None, None
        self.circle_r.pop("entry", None)
        ctx.log(f"T1 LUCKY RETRY: #{e.id} decided flashing blue (led {e.led}) - circling it CW, then the gates again ({n} passed so far)")
        self.m.handle("LUCKY_RETRY")
        return True

    def _axis(self, ctx: TaskContext, items) -> Tuple[float, float]:
        """v0.1.92: the course's axis from the entry - toward the exit when it is known, else toward the gates' centre."""
        ext = None
        for i in sorted(getattr(self, "exit_seen", ())):
            e = next((e for e in ctx.b.map.entities("buoy", include_stale=True) if e.id == i), None) if ctx.b.map is not None else None
            if e is not None:
                ext = (e.x_n, e.y_e)
        if ext is None:
            m = self.marker(ctx, "exit") if self.entry_xy is not None else None
            ext = (m.x_n, m.y_e) if m is not None else None
        if ext is None:
            mids = [it.mid for it in items] + [q["mid"] for q in self.passed]
            ext = (sum(m[0] for m in mids) / len(mids), sum(m[1] for m in mids) / len(mids))
        return gp._unit(ext[0] - self.entry_xy[0], ext[1] - self.entry_xy[1])

    def _lead(self, it) -> float:
        """v0.1.92: the pre-point's distance before the gate - min(spacing - 1.5, 3.0), never behind the previous gate."""
        lead = float(self.p["lead_max_m"])
        if self.passed:
            pm = self.passed[-1]["mid"]
            sp = (it.mid[0] - pm[0]) * it.dir[0] + (it.mid[1] - pm[1]) * it.dir[1]
            if sp > 0:
                lead = min(lead, sp - 1.5)
        return max(float(self.p["lead_min_m"]), lead)

    # ---- leads (v0.1.39, WP10h) ---------------------------------------------------------------------------------------
    def pending_lead(self, ctx: TaskContext, gate: Optional[gp.GateItem]):
        """The nearest pending buoy (colour still undecided, seen 3+ times, not looked at yet, not behind) nearer than `gate`."""
        m, pose = ctx.b.map, ctx.b.pose
        if m is None or pose is None or getattr(self, "uav_mode", False):   # v0.1.66: the colour-blind camera cannot decide by looking
            return None
        lim = float(self.p["look_r_m"])
        if gate is not None:
            lim = min(lim, math.hypot(gate.mid[0] - pose.x_n, gate.mid[1] - pose.y_e) - 2.0)
        best = None
        for e in m.entities("buoy"):
            if e.colour != "unknown" or e.n_obs < 3 or e.id in self.skipped:
                continue
            dx, dy = e.x_n - pose.x_n, e.y_e - pose.y_e
            d = math.hypot(dx, dy)
            if d > lim or (self.last_dir is not None and dx * self.last_dir[0] + dy * self.last_dir[1] < 0):
                continue
            if best is None or d < best[0]:
                best = (d, e)
        return best[1] if best else None

    def map_lead(self, ctx: TaskContext) -> Optional[Tuple[float, float]]:
        """v0.1.54: a point toward the buoy that may be the entry - blue with its light undecided, else still pending (colour
        unknown), else another colour with blue votes (far, a flashing blue's frames spread over the colours), the bluest first, else
        any buoy whose light is still undecided ("?": too few weighted votes) -
        stopping `lead_stop_m` short of it; None when there is none (or the boat is already that close)."""
        m, pose = ctx.b.map, ctx.b.pose
        if m is None or pose is None:
            return None
        def blue_share(e):
            v = e.colour_votes
            return v.get("blue", 0.0) / max(1e-6, sum(x for k, x in v.items() if k != "black"))
        c = [e for e in m.entities("buoy") if e.id not in self.rejected and e.id not in self.lead_ids and self._in_course(ctx, (e.x_n, e.y_e))
             and ((e.colour == "blue" and e.light not in ("flash", "steady")) or e.colour == "unknown" or e.light == "?"
                  or (e.colour != "blue" and e.colour_votes.get("blue", 0.0) >= 1.0))]      # far: a flashing blue can read green
        if not c:
            return None
        e = min(c, key=lambda e: (e.colour != "blue", e.colour != "unknown", e.colour_votes.get("blue", 0.0) < 1.0, e.light != "?",
                                  -blue_share(e), math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e)))
        dx, dy = e.x_n - pose.x_n, e.y_e - pose.y_e
        go = math.hypot(dx, dy) - float(self.p["lead_stop_m"])
        if go < 3.0:
            return None
        self.lead_ids.add(e.id)
        u = gp._unit(dx, dy)
        return (pose.x_n + go * u[0], pose.y_e + go * u[1])

    def hint_lead(self, ctx: TaskContext) -> Optional[Tuple[float, float]]:
        """v0.1.54: nothing in view and no sighting (Course Extra's start is ~50 m from Task 1, beyond the camera's 40 m bearings):
        to the operator's search point (START_TASK args / the profile's t1.search_lat, search_lon: a point before the entry, from the
        course map; before the rays - the camera also sees other tasks' buoys); None without one or within `lead_stop_m` of it.
        Counted in lead_tries with the other leads."""
        o, pose = ctx.b.origin, ctx.b.pose
        if o is None or pose is None or self.p.get("search_lat") is None or self.p.get("search_lon") is None:
            return None
        x, y = o.to_local(float(self.p["search_lat"]), float(self.p["search_lon"]))
        dx, dy = x - pose.x_n, y - pose.y_e
        d = math.hypot(dx, dy)
        if d < float(self.p["lead_stop_m"]):
            return None                                  # there already: the rays / the safe hold
        return (x, y)

    def watch_locked(self, ctx: TaskContext) -> None:
        """v0.1.54: the locked entry / exit must stay blue in the map (the near frames are clean): decided otherwise -> rejected.
        v0.1.66: not under colour_source uav - the colour-blind camera never sees blue; the UAV's colour decides."""
        m = ctx.b.map
        if m is None or getattr(self, "uav_mode", False):
            return
        st = self.m.state
        which = "entry" if st == "CIRCLE_ENTRY_CW" else ("exit" if st in ("EXIT", "CIRCLE_EXIT_CCW") else None)
        eid = self.entry_id if which == "entry" else self.exit_id
        if which is None or eid is None or (which == "entry" and eid == getattr(self, "lucky_exit_id", None)):
            return
        e = next((e for e in m.entities("buoy") if e.id == eid), None)
        pose = ctx.b.pose
        if e is None or pose is None:
            return
        near = self.near.setdefault(eid, [])
        if e.t_last != (near[-1][0] if near else None) and math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e) <= float(self.p["verify_r_m"]):
            near.append((e.t_last, e.colour_now))                # a sighting from near: the camera's colour is clean there
        if which == "entry" and len(near) >= int(self.p["steady_n"]) and getattr(e, "led", None) is None:
            blue = sum(1 for _, c in near if c == "blue")             # v0.1.92 (the sim at 0.3 m/s: the far votes read the steady EXIT
            dark = sum(1 for _, c in near if c in ("black", None))    # "flashing" - it was locked and circled as the entry): near, a steady
            if blue >= int(self.p["steady_n"]) - 1 and dark == 0:     # light never reads dark -> it is the exit: remembered, search on
                self.exit_seen.add(eid)                               # (not `rejected`: it stays the exit for EXIT; marker(entry) skips it)
                ctx.log(f"T1 entry #{eid} is STEADY blue up close ({blue} of {len(near)} sightings within {float(self.p['verify_r_m']):.0f} m blue, "
                        f"none dark) - the EXIT, not the entry: remembered, searching again")
                if self._beh is not None:
                    self._beh.cancel()
                    self._beh = None
                self.circle_r.pop(which, None)
                self.entry_xy, self.entry_id = None, None
                self.m.handle("NOT_ENTRY")
                return
        if len(near) < int(self.p["verify_n"]) or any(c == "blue" for _, c in near):
            return
        self.rejected.add(eid)
        seen = {c: sum(1 for _, x in near if x == c) for c in sorted({x for _, x in near})}
        ctx.log(f"T1 {which} #{eid} is not blue after all: {len(near)} sightings within {float(self.p['verify_r_m']):.0f} m, none blue ({seen}; "
                f"the map: {e.colour} {e.light}) - rejected, {'searching again' if which == 'entry' else 'scanning for the exit'}")
        if self._beh is not None:
            self._beh.cancel()
            self._beh = None
        self.circle_r.pop(which, None)
        if which == "entry":
            self.entry_xy, self.entry_id = None, None
            self.m.handle("NOT_ENTRY")
        else:
            self.exit_xy, self.exit_id = None, None
            self.m.handle("NOT_EXIT")

    def replan_check(self, ctx: TaskContext) -> None:
        """v0.1.54: a path planned on a partial map (a far start: the field comes into view on the way) is planned again - at most
        every `replan_s` - when the rest of it now passes a mapped object closer than the clearance rule and the map has changed."""
        st, b = self.m.state, self._beh
        if b is None or st not in ("APPROACH_LEAD", "TRANSIT", "EXIT", "CIRCLE_ENTRY_CW", "CIRCLE_EXIT_CCW"):
            return
        if ctx.b.now - self.replan_t < float(self.p["replan_s"]):
            return
        self.replan_t = ctx.b.now
        if hasattr(b, "approach"):                       # Orbit: only while still on its way to the ring
            rest = b.approach[b.ai:] if b.approach else []
        else:
            rest = b.pts[b.idx:]
        if len(rest) < 3:
            return
        obs = self.obstacles(ctx)
        skip = 1.0
        if st == "TRANSIT" and self.current is not None:      # only the head: never once the crossing (pre -> mid -> post) is near
            d = float(self.p["s6_d_m"] if self.p["planner"] == "s6" else self.p["gate_d_m"])
            pose = ctx.b.pose
            if pose is None or math.hypot(pose.x_n - self.current.mid[0], pose.y_e - self.current.mid[1]) < d + 3.0:
                return
            obs = [o for o in obs if (o[0], o[1]) not in (self.current.red, self.current.green)]
            skip = 2.0 * d + 1.0
        sig = tuple(sorted((round(o[0]), round(o[1])) for o in obs))
        if sig == self.plan_sig:
            return
        self.plan_sig = sig
        c = gp.path_clearance(rest, obs, self.clr, skip_end_m=skip)
        if c >= self.clr.clear_m:
            return
        ctx.log(f"T1 {st}: the rest of the path passes {c:.2f} m from a mapped object (the map grew) - planning it again")
        b.cancel()
        self._beh = None
        self.replans += 1

    def uav_lead(self, ctx: TaskContext) -> Optional[Tuple[float, float]]:
        """v0.1.66 (D22): toward the UAV's map - the nearest UAV-only buoy that is blue (or undecided), else the nearest provisional
        gate (two UAV-only buoys) - `lead_stop_m` short of it, so the camera takes it over; None without one or when already near."""
        m, pose = ctx.b.map, ctx.b.pose
        if m is None or pose is None or not hasattr(m, "uav_entities"):
            return None
        cand = [((e.x_n, e.y_e), e.id) for e in m.uav_entities() if e.colour in ("blue", "unknown") and e.id not in self.lead_ids]
        if not cand:
            cand = [((g[2], g[3]), ("pg", g[0].id, g[1].id)) for g in m.provisional_gates() if ("pg", g[0].id, g[1].id) not in self.lead_ids]
        if not cand:
            return None
        (x, y), key = min(cand, key=lambda c: math.hypot(c[0][0] - pose.x_n, c[0][1] - pose.y_e))
        dx, dy = x - pose.x_n, y - pose.y_e
        go = math.hypot(dx, dy) - float(self.p["lead_stop_m"])
        if go < 3.0:
            return None
        self.lead_ids.add(key)
        u = gp._unit(dx, dy)
        return (pose.x_n + go * u[0], pose.y_e + go * u[1])

    def uav_map_ready(self, ctx: TaskContext) -> Tuple[bool, str]:
        """D22c: may the boat leave WAIT_UAV_MAP? (go, why). The UAV's scan.complete, or the drone landed after flying with no UAV
        item for uav_quiet_s, or the operator's `uav scan done` (forces it) - and an entry candidate in the UAV layer."""
        m = ctx.b.map
        u = getattr(m, "uav_ocs", None) or {}
        if u.get("scan_done"):
            return True, "the operator's `uav scan done`"
        sc = getattr(m, "uav_scan", {}) or {}
        why = None
        self.uav_gate = UG.touchdown(u, float(getattr(m, "uav_change_t", -1e9)), ctx.b.now, float(self.p["uav_quiet_s"]),
                                     self.__dict__.setdefault("uav_latch", {}))      # v0.1.89: flying -> map quiet -> RTL -> disarmed
        if sc.get("state") == "complete":
            why = "the UAV's scan.complete"
        elif self.uav_gate["go"]:
            why = "TOUCHDOWN: the drone disarmed after flying, the map quiet %.0f s" % float(self.p["uav_quiet_s"])
        if why is None:
            return False, ""
        if m is not None and hasattr(m, "uav_entry_candidate") and m.uav_entry_candidate() is None:
            return False, why + " - but the UAV map has no entry (flashing blue)"
        return True, why

    def wait_uav_map(self, ctx: TaskContext) -> None:
        now = ctx.b.now
        if self.uav_wait_t0 is None:
            self.uav_wait_t0 = now
        go, why = self.uav_map_ready(ctx)
        if go:
            ctx.log(f"T1 WAIT_UAV_MAP done after {now - self.uav_wait_t0:.0f} s: {why}")
            if self._beh is not None:
                self._beh.cancel()
                self._beh = None
            self.m.handle("UAV_MAP")
            return
        if why and now - self.uav_caution_t > 10.0:                   # "UAV map has no entry": a CAUTION, said every 10 s
            self.uav_caution_t = now
            ctx.log(f"T1 WAIT_UAV_MAP: {why} (waiting; `uav scan done` forces the transit)")
        if now - self.uav_wait_t0 > float(self.p["uav_wait_s"]) and not self.safe_hold:
            self.safe_hold = (f"T1 WAIT_UAV_MAP: no usable UAV map after {float(self.p['uav_wait_s']):.0f} s"
                              + (f" ({why})" if why else " (no scan.complete, the drone not back)") + " - uav scan done, or abort")
            ctx.log(self.safe_hold)

    def ray_lead(self, ctx: TaskContext) -> Optional[Tuple[float, float]]:
        """A point `lead_step_m` toward the freshest far sighting (a blue one first), or None (no ray, or already close to it)."""
        m, pose = ctx.b.map, ctx.b.pose
        if m is None or pose is None:
            return None
        ttl = float(getattr(self.cfg.map, "ray_ttl_s", 30.0))
        rays = [r for r in getattr(m, "rays", []) if ctx.b.now - float(r.get("t", -1e9)) <= ttl]
        if not rays:
            return None
        r = max(rays, key=lambda r: (r.get("colour") == "blue", r.get("n", 1), r.get("t", 0.0)))
        az = math.radians(float(r["az_deg"]))
        far = (float(r["x0"]) + 32.0 * math.cos(az), float(r["y0"]) + 32.0 * math.sin(az))      # mid of the 25-40 m band
        dx, dy = far[0] - pose.x_n, far[1] - pose.y_e
        go = min(float(self.p["lead_step_m"]), math.hypot(dx, dy) - 15.0)
        if go < 3.0:
            return None
        u = gp._unit(dx, dy)
        return (pose.x_n + go * u[0], pose.y_e + go * u[1])

    def _look_done(self, ctx: TaskContext) -> bool:
        e = next((e for e in ctx.b.map.entities("buoy") if e.id == self.look_id), None) if ctx.b.map is not None else None
        return e is None or e.colour != "unknown"

    # ---- the entry search: scout + check (v0.1.92, Dmitrii 2026-11-13) ---------------------------------------------------
    def _final_scan_end(self, ctx: TaskContext) -> None:
        """v0.1.93.2: the finale's last turn ended - a flashing blue in the map -> back to the entry sequence, else the mission ends (DONE)."""
        if self.marker(ctx, "entry") is not None or self.blue_fix_id is not None:
            ctx.log("T1 a flashing blue seen in the last turn: the entry sequence")
            self.m.handle("FINAL_ENTRY")
            return
        ctx.log("T1 no blue buoy after the last turn: the mission ends (DONE)")
        self.m.handle("FINAL_END")

    def _blue_fix(self, b) -> bool:
        """v0.1.93.2 (Dmitrii 13 Nov): a FLASHING blue lantern in the map (the entry; a steady one is the exit - the sim's decoy world
        stopped the turn on it and never saw the entry behind: 1 / 6 done),
        seen min_obs times, inside the course, not rejected / the exit already: the start turn stops there, the ladder takes it (look-lock,
        approach, check). b = the live BehaviourContext."""
        m = getattr(b, "map", None)
        if m is None:
            return False
        for e in m.entities("buoy"):
            if e.colour != "blue" or e.light != "flash" or e.n_obs < 2 * int(self.p["min_obs"]):   # a steady blue = the exit: turn on
                continue
            pose = getattr(b, "pose", None)
            if pose is None or math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e) > float(self.p.get("start_scan_stop_r_m", 10.0)):
                continue
            if e.id in self.rejected or e.id in self.exit_seen:
                continue
            if not self._in_course_b(b, (e.x_n, e.y_e)):
                continue
            self.blue_fix_id = e.id                     # logged when the turn ends (on_behaviour_done)
            return True
        return False

    def _in_course_b(self, b, q) -> bool:
        class _C:                                           # _in_course reads ctx.b.origin / ctx.b.run.course
            pass
        c = _C(); c.b = b
        return self._in_course(c, q)

    def check_candidates(self, ctx: TaskContext) -> list:
        """Buoys that may be the entry: blue whose light is not yet read flashing, undecided colour, or another colour with blue votes;
        not checked yet, not rejected, not the exit seen, inside the course (when one is known)."""
        m = ctx.b.map
        if m is None:
            return []
        out = []
        for e in m.entities("buoy"):
            if e.id in self.checked or e.id in self.rejected or e.id in self.exit_seen or e.n_obs < 3:
                continue
            if not self._in_course(ctx, (e.x_n, e.y_e)):
                continue
            if e.light == "off" or e.colour in ("black", "red", "green", "yellow") and e.light in ("flash", "steady"):
                if e.colour != "blue" and not (e.colour_votes.get("blue", 0.0) >= 1.0 and e.light == "?"):
                    continue                                   # v0.1.92: unlit, or resolved another colour - never a suspect
            lit = sum(v for k, v in e.colour_votes.items() if k != "black")
            share = e.colour_votes.get("blue", 0.0) / lit if lit > 0 else 0.0
            if e.colour == "blue" or e.colour == "unknown" or e.light == "?" or share >= float(self.p["suspect_blue_share"]):
                out.append(e)
        return out

    def _in_course(self, ctx: TaskContext, q) -> bool:
        o, c = ctx.b.origin, getattr(ctx.b.run, "course", None)
        b = list(getattr(c, "boundary", None) or [])
        if o is None or len(b) < 3:
            return True
        from .. import scout_geom as G
        if not hasattr(self, "_poly"):
            self._poly = G.open_ring([o.to_local(float(x[0]), float(x[1])) for x in b])
        return G.inside(q, self._poly) or _seg_near(q, self._poly, float(self.p.get("geofence_m", 1.0)))

    def _scout_next(self, ctx: TaskContext) -> bool:
        """The ladder's steps 3-5: a candidate within check_r_m -> CHECK_BLUE; a farther one -> SCOUT_ENTRY to check_hold_m of it, then
        CHECK_BLUE; else the nearest unvisited buoy (colour-blind) -> SCOUT_ENTRY to scout_standoff_m, then the scan. The budget
        scout_entry_s gone -> the lucky gates (when on) or False (the leads / the hold). True = a transition was made."""
        pose, p, now = ctx.b.pose, self.p, ctx.b.now
        if pose is None:
            return False
        if self.scout_t0 is None:
            self.scout_t0 = now
        if now - self.scout_t0 > float(p["scout_entry_s"]) + self.extra_s and self.doubtful and not self.rechecked:
            self.rechecked, self.extra_s = True, self.extra_s + 60.0     # Cowork: a twice-unknown buoy beats a random pair of gates
            self.checked -= self.doubtful
            for i in self.doubtful:
                self.checks[i] = int(p["check_tries"]) - 1
            ctx.log(f"T1 entry search: the budget is spent - one more look at the doubtful {sorted(self.doubtful)} (60 s)")
            self.doubtful = set()
        if now - self.scout_t0 > float(p["scout_entry_s"]) + self.extra_s:
            ctx.log(f"T1 entry search: {float(p['scout_entry_s']) + self.extra_s:.0f} s of scouting, no flashing blue")
            if p.get("lucky") and self._lucky_go(ctx, "the scouting budget is spent"):
                return True
            if self.m.state != "SEARCH_ENTRY":
                self.m.handle("CHECKED" if self.m.state == "CHECK_BLUE" else "SCOUTED", reason="the scouting budget is spent")
            return False
        boat = (pose.x_n, pose.y_e)
        obstacles = self.obstacles(ctx)
        cands = sorted(self.check_candidates(ctx), key=lambda e: (e.colour != "blue", math.hypot(e.x_n - boat[0], e.y_e - boat[1])))
        for e in cands:
            d = math.hypot(e.x_n - boat[0], e.y_e - boat[1])
            self.check_id, self.check_xy = e.id, (e.x_n, e.y_e)
            if d <= float(p["check_r_m"]):
                ctx.log(f"T1 check #{e.id} ({e.colour} {e.light}, {d:.1f} m): reading its light for {float(p['check_s']):.0f} s")
                self.m.handle("CHECK")
                return True
            u = gp._unit(boat[0] - e.x_n, boat[1] - e.y_e)
            goal = (e.x_n + float(p["check_hold_m"]) * u[0], e.y_e + float(p["check_hold_m"]) * u[1])
            self.scout_goal = gp.free_point(goal, obstacles, self.clr)
            if not self._reachable(ctx, self.scout_goal, obstacles):
                ctx.log(f"T1 #{e.id}: no clear way to read it from {float(p['check_hold_m']):.0f} m - skipped")
                self.checked.add(e.id)
                continue
            self.scout_then_check = True
            if p.get("look_lock") and self.m.state != "LOOK_LOCK":
                self.why = f"suspect #{e.id} ({e.colour} {e.light}) at {d:.1f} m"
                ctx.log(f"T1 LOOK_LOCK on #{e.id} ({e.colour} {e.light}, {d:.1f} m): stop, bow on it, {float(p['look_lock_s']):.0f} s")
                self.m.handle("LOCK")
                return True
            self.why = f"to {float(p['check_hold_m']):.0f} m of suspect #{e.id} ({d:.1f} m)"
            ctx.log(f"T1 to #{e.id} ({e.colour} {e.light}, {d:.1f} m) to read its light from {float(p['check_hold_m']):.0f} m")
            self.m.handle("SCOUT")
            return True
        self.check_id, self.check_xy, self.scout_then_check = None, None, False
        best = None
        for e in self.buoys(ctx):                        # colour-blind: the nearest buoy not visited yet
            if e.id in self.checked:
                continue
            if any(math.hypot(e.x_n - v[0], e.y_e - v[1]) < 2.0 for v in self.visited) or not self._in_course(ctx, (e.x_n, e.y_e)):
                continue
            d = math.hypot(e.x_n - boat[0], e.y_e - boat[1])
            if d < float(p["scout_standoff_m"]) + 1.0:
                self.visited.append((e.x_n, e.y_e))
                continue
            if best is None or d < best[0]:
                best = (d, e)
        if best is None:
            return False
        e = best[1]
        self.visited.append((e.x_n, e.y_e))
        u = gp._unit(boat[0] - e.x_n, boat[1] - e.y_e)
        goal = (e.x_n + float(p["scout_standoff_m"]) * u[0], e.y_e + float(p["scout_standoff_m"]) * u[1])
        self.scout_goal = gp.free_point(goal, obstacles, self.clr)
        if not self._reachable(ctx, self.scout_goal, obstacles):
            ctx.log(f"T1 scouting: no clear way to #{e.id} - skipped")
            return self._scout_next(ctx)
        ctx.log(f"T1 no entry yet: scouting to #{e.id} ({e.colour}, {best[0]:.1f} m)")
        self.m.handle("SCOUT")
        return True

    def _check_decided(self, ctx: TaskContext) -> bool:
        e = next((e for e in ctx.b.map.entities("buoy") if e.id == self.check_id), None) if ctx.b.map is not None else None
        return e is None or getattr(e, "led", None) is not None and e.light in ("flash", "steady")

    def _check_result(self, ctx: TaskContext, decided: bool) -> None:
        """After CHECK_BLUE: flashing blue -> the entry (locked, ENTRY_CHECKED); steady blue -> the exit (remembered); other -> checked;
        undecided -> again (check_tries), then given up. Then the next step of the ladder."""
        eid = self.check_id
        e = next((e for e in ctx.b.map.entities("buoy") if e.id == eid), None) if ctx.b.map is not None else None
        self.checks[eid] = self.checks.get(eid, 0) + 1
        if e is not None and e.colour == "blue" and e.light == "flash":
            self.checked.add(eid)
            self.entry_xy, self.entry_id = (e.x_n, e.y_e), e.id
            ctx.log(f"T1 entry found by checking #{eid}: flashing blue (led {getattr(e, 'led', None)}, votes {e.colour_votes})")
            self.m.handle("ENTRY_CHECKED")
            return
        lb = self.p.get("lucky_blue", True) if self.p.get("lucky") else False
        agn = str(lb).lower() == "any"               # v0.1.94.1 (Dmitrii): blue-agnostic - not read steady = the entry, one check
        if (e is not None and e.colour == "blue" and lb and eid not in self.exit_seen
                and (e.light != "steady" if agn else
                     (e.light not in ("flash", "steady", "off") and self.checks[eid] >= int(self.p["check_tries"])))):
            self.checked.add(eid)                      # v0.1.94 (Dmitrii: "if we can't tell solid from flashing, lucky mode orbits regardless")
            self.entry_xy, self.entry_id = (e.x_n, e.y_e), e.id
            self.lucky_unsure = True                   # v0.1.94.2: a decided flashing blue later -> the retry
            ctx.log(f"T1 LUCKY{' (blue-agnostic)' if agn else ''}: #{eid} is blue, light {e.light} after {self.checks[eid]} "
                    f"check(s) - orbiting it as the entry")
            self.m.handle("ENTRY_CHECKED")
            return
        if e is not None and e.colour == "blue" and e.light == "steady":
            self.checked.add(eid)
            self.exit_seen.add(eid)
            ctx.log(f"T1 #{eid} is steady blue: the EXIT (remembered), the entry search goes on")
        elif e is None or e.light in ("flash", "steady", "off") or self.checks[eid] >= int(self.p["check_tries"]):
            self.checked.add(eid)
            if e is not None and e.light not in ("flash", "steady", "off"):
                self.doubtful.add(eid)
            ctx.log(f"T1 #{eid} checked: " + ("gone" if e is None else f"{e.colour} {e.light}") + " - not the entry"
                    + (" (doubtful)" if eid in self.doubtful else ""))
        else:
            pose = ctx.b.pose
            closer = max(3.0, float(self.p["check_hold_m"]) - 2.0)
            d = math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e) if pose is not None else 0.0
            if pose is not None and d > closer + 0.5:                    # Cowork: the 2nd look from 2 m closer
                u = gp._unit(pose.x_n - e.x_n, pose.y_e - e.y_e)
                goal = gp.free_point((e.x_n + closer * u[0], e.y_e + closer * u[1]), self.obstacles(ctx), self.clr)
                if self._reachable(ctx, goal, self.obstacles(ctx)):
                    self.scout_goal, self.scout_then_check = goal, True
                    ctx.log(f"T1 #{eid} still undecided ({e.colour} {e.light}): again from {closer:.0f} m")
                    self.m.handle("SCOUT")
                    return
            ctx.log(f"T1 #{eid} still undecided ({e.colour} {e.light}): once more")
            self.m.handle("CHECK")
            return
        if not self._scout_next(ctx) and self.m.state == "CHECK_BLUE":
            self.m.handle("CHECKED")

    # ---- reports ---------------------------------------------------------------------------------------------------
    def beacon(self, e) -> str:
        """v0.1.54 (item 12): the RoboCommand BeaconState of a mapped buoy (rx_common.proto; handbook 3.3.2: FLASHING RED / GREEN
        gate buoys, FLASHING BLUE entry, STEADY BLUE exit, BLACK = OFF). The gate colours only flash in Task 1, so a red / green
        buoy whose light pattern is still undecided ("?", or read steady from a few lit frames) is FLASHING; the task's own entry
        / exit (locked or identified) is FLASHING / STEADY BLUE; a blue buoy of undecided light and a pending one (colour unknown)
        are UNKNOWN; black (an unlit obstacle) is OFF whatever its pattern."""
        if e.id in self.rejected:                       # v0.1.54: what the near sightings showed (the map may still say blue)
            near = [c for _, c in self.near.get(e.id, [])]
            top = max(set(near), key=near.count) if near else "black"
            return "BEACON_STATE_OFF" if top == "black" else (f"BEACON_STATE_FLASHING_{top.upper()}" if top in ("red", "green")
                                                               else "BEACON_STATE_UNKNOWN")
        if e.colour == "black":
            return "BEACON_STATE_OFF"
        if e.colour in ("red", "green"):
            return f"BEACON_STATE_FLASHING_{e.colour.upper()}"
        if e.colour == "blue":
            if e.id == self.entry_id or (self.entry_id is None and e.light == "flash"):
                return "BEACON_STATE_FLASHING_BLUE"
            if e.id == self.exit_id or e.light == "steady":
                return "BEACON_STATE_STEADY_BLUE"
            if e.light == "flash" and self.entry_id is not None:
                return "BEACON_STATE_FLASHING_BLUE"
        return "BEACON_STATE_UNKNOWN"

    def maybe_report(self, ctx: TaskContext, complete: bool = False) -> None:
        """SafePassageReport bodies (event `safe_passage`). v0.1.54 (item 12): every mapped buoy with its position and its BeaconState
        (`beacon`, beside the colour / state words) - the stale ones too and those the map has dropped since (remembered), a ghost
        of the same state within `dedup_m` of a better-seen one left out; entry / exit = the task's entry / exit buoy where it is mapped now (the position
        where it was locked once out of the map); a new report when a buoy is added or dropped, a state changes, a buoy moves more
        than `report_move_m`, or the entry / exit changes - at most every `report_period_s` (the OCS coalesces to 1 Hz; RoboCommand:
        at most 5 messages a second per system, handbook 3.4) - and once complete at DONE."""
        o = ctx.b.origin
        if o is None or ctx.b.map is None:
            return
        self._follow_merges(ctx.b.map)
        ents = {e.id: e for e in ctx.b.map.entities("buoy", include_stale=True)}
        ent = ents.get(self.entry_id) if self.entry_id is not None else self.marker(ctx, "entry")
        ext = ents.get(self.exit_id) if self.exit_id is not None else (self.marker(ctx, "exit") if self.entry_xy is None else None)
        if ext is None and self.entry_xy is not None and self.exit_id is None:
            ext = self.marker(ctx, "exit")
        for e in ctx.b.map.entities("buoy", include_stale=True):     # the field as mapped: out of view (stale) still counts
            if e.n_obs < int(self.p["min_obs"]):
                continue
            lat, lon = o.from_local(e.x_n, e.y_e)
            state = {"flash": "flashing", "steady": "solid", "off": "off"}.get(e.light, "unknown")
            self.field[e.id] = {"id": e.id, "colour": e.colour, "state": state, "beacon": self.beacon(e), "lat": round(lat, 7),
                                "lon": round(lon, 7), "_xy": (e.x_n, e.y_e), "_n": e.n_obs}
        keep = []                                       # dropped from the map (aged out): remembered; a duplicate of the same state
        for b in sorted(self.field.values(), key=lambda b: -b["_n"]):     # within dedup_m (a ghost) is left out
            if any(k["beacon"] == b["beacon"] and math.hypot(k["_xy"][0] - b["_xy"][0], k["_xy"][1] - b["_xy"][1]) < float(self.p["dedup_m"])
                   for k in keep):
                continue
            keep.append(b)
        for b in sorted(keep, key=lambda b: b["id"]):
            if b["id"] not in self.rid:
                self.rid[b["id"]] = self._next_rid
                self._next_rid += 1
        keep.sort(key=lambda b: self.rid[b["id"]])
        buoys = [dict({k: v for k, v in b.items() if not k.startswith("_")}, id=self.rid[b["id"]], map_id=b["id"]) for b in keep]
        where = {b["id"]: b["_xy"] for b in keep}
        sig = (tuple((b["map_id"], b["beacon"]) for b in buoys), ent.id if ent else None, ext.id if ext else None, complete)
        moved = any(math.hypot(xy[0] - self.report_xy[i][0], xy[1] - self.report_xy[i][1]) > float(self.p["report_move_m"])
                    for i, xy in where.items() if i in self.report_xy)
        if getattr(self, "_merged_new", False):          # v0.1.94: a merge changed the field - resend at once (not after report_period_s)
            self._merged_new, self.report_t = False, -1e9
        if not complete and ((sig == self.report_sig and not moved) or ctx.b.now - self.report_t < float(self.p["report_period_s"])):
            return
        self.report_sig, self.report_t, self.report_xy = sig, ctx.b.now, where

        def pos(e, remembered):
            xy = (e.x_n, e.y_e) if e is not None else remembered      # out of view by now: where the task found it
            if xy is None:
                return None
            lat, lon = o.from_local(*xy)
            return {"lat": round(lat, 7), "lon": round(lon, 7)}
        self.report(ctx, "safe_passage", {"entry": pos(ent, self.entry_xy), "exit": pos(ext, self.exit_xy), "buoys": buoys, "complete": complete,
                                          "gates_passed": len(self.passed), "t": utc_iso()})

    def _follow_merges(self, m) -> None:
        """v0.1.94: the map's merge pass took ids away (PerceptionMap.merged): the task's entry / exit / check / rejected / checked follow
        to the id they went into; a merged-away buoy leaves the reported field; the survivor keeps the lower report id."""
        merged = getattr(m, "merged", None) or {}
        if not merged:
            return
        for k in ("entry_id", "exit_id", "check_id"):
            v = getattr(self, k, None)
            if v is not None and v in merged:
                setattr(self, k, m.resolve(v))
        for name in ("rejected", "checked"):
            st = getattr(self, name, None)
            if isinstance(st, set) and st & merged.keys():
                setattr(self, name, {m.resolve(i) for i in st})
        for gone in [i for i in self.field if i in merged]:
            keep = m.resolve(gone)
            del self.field[gone]
            if gone in self.rid:
                r = self.rid.pop(gone)
                self.rid[keep] = min(r, self.rid.get(keep, r))
            self._merged_new = True

    # ---- behaviour wiring --------------------------------------------------------------------------------------------
    def behaviour_args(self, sub: str, ctx: TaskContext) -> Dict[str, Any]:
        p = self.p
        if sub == "START_ADVANCE":                         # v0.1.92: Dmitrii - "a small advance forward (2 m), a full rotation, then the ladder"
            pose = ctx.b.pose
            h = math.radians(pose.hdg_deg)
            goal = (pose.x_n + float(p["start_advance_m"]) * math.cos(h), pose.y_e + float(p["start_advance_m"]) * math.sin(h))
            return {"points": gp.resample([(pose.x_n, pose.y_e), goal], 0.5), "speed_mps": p["start_v"], "la_m": 1.0,
                    "corridor_m": p["corridor_m"], "end_r_m": 0.5}
        if sub == "START_SCAN":
            self.started_scan = True
            stop_blue = bool(p.get("start_scan_stop_blue"))
            return {"until": (lambda b: self._blue_fix(b)) if stop_blue else (lambda b: False), "rate_dps": p["scan_rate_dps"], "turns": 1.0,
                    "inplace": True, "timeout_s": p["start_scan_s"], "reason": "the start turn done (or a blue lantern found)"}
        if sub == "FINAL_ADVANCE":                        # v0.1.93.2: the no-entry finale
            pose = ctx.b.pose
            h = math.radians(pose.hdg_deg)
            goal = gp.free_point((pose.x_n + float(p["final_advance_m"]) * math.cos(h), pose.y_e + float(p["final_advance_m"]) * math.sin(h)),
                                 self.obstacles(ctx), self.clr)
            return {"points": gp.resample([(pose.x_n, pose.y_e), goal], 0.5), "speed_mps": p["start_v"], "la_m": 1.0,
                    "corridor_m": p["corridor_m"], "end_r_m": 0.5}
        if sub == "FINAL_SCAN":
            return {"until": lambda b: self._blue_fix(b) or self.marker(ctx, "entry") is not None, "rate_dps": p["final_scan_rate_dps"],
                    "turns": 1.0, "inplace": True, "timeout_s": p["start_scan_s"], "reason": "the last turn done"}
        if sub == "LOOK_LOCK":
            t0 = ctx.b.now
            xy = self.check_xy

            def locked(b):                                  # b = the live BehaviourContext (ctx.b is the snapshot of the start)
                pose = b.pose
                if pose is None or xy is None:
                    return True
                want = math.degrees(math.atan2(xy[1] - pose.y_e, xy[0] - pose.x_n))
                return b.now - t0 >= float(p["look_lock_s"]) and abs((want - pose.hdg_deg + 180.0) % 360.0 - 180.0) < float(p.get("look_lock_tol_deg", 10.0))
            return {"until": locked, "face": xy, "max_s": float(p["look_lock_s"]) + 60.0, "rate_dps": p["scan_rate_dps"],
                    "still_deg": min(float(p.get("check_still_deg", 0.0)), float(p.get("look_lock_tol_deg", 10.0)) / 2.5),   # 2x inside the tolerance
                    "reason": "the bow did not come round"}
        if sub == "ALIGN_GATE":
            it = self.current
            far = (it.mid[0] + 20.0 * it.dir[0], it.mid[1] + 20.0 * it.dir[1])

            def lined(b):                                   # the live pose (b), not the start's snapshot
                pose = b.pose
                return pose is not None and abs((it.hdg_deg - pose.hdg_deg + 180.0) % 360.0 - 180.0) <= float(p["align_deg"])
            return {"until": lined, "face": far, "max_s": p["align_max_s"], "rate_dps": p["scan_rate_dps"],
                    "reason": f"not within {float(p['align_deg']):.0f} deg after {float(p['align_max_s']):.0f} s - through anyway"}
        if sub == "THROUGH":
            t = self.thru
            if p.get("squeeze") and not t.get("squeeze") and self.current is not None:   # v0.1.93: a restart (after a guard stop) re-plans:
                it = self.current                                              # the object the stop found is in the map by now
                lead = (it.mid[0] - t["pre"][0]) * it.dir[0] + (it.mid[1] - t["pre"][1]) * it.dir[1]
                pm = (t["post"][0] - it.mid[0]) * it.dir[0] + (t["post"][1] - it.mid[1]) * it.dir[1]
                sq = self._squeeze_plan(ctx, it, max(lead, 1.5), max(pm, 0.75))
                if sq is not None:
                    t.update(sq)
            pts = t.get("path") or [t["pre"], t["post"]]
            return {"points": gp.resample(pts, 0.5), "speed_mps": p["squeeze_v"] if t.get("squeeze") else self._gate_v(ctx, "through"),
                    "la_m": p["la_through_m"], "corridor_m": 3.0, "end_r_m": 0.5}
        if sub == "SEARCH_ENTRY":
            if self.started_scan:                           # v0.1.92: the start turn already looked round - a short look, then the ladder
                return {"until": lambda b: self.marker(ctx, "entry") is not None, "rate_dps": p["scan_rate_dps"], "turns": 0.05,
                        "inplace": True, "reason": "no flashing blue entry buoy in the map"}
            return {"until": lambda b: self.marker(ctx, "entry") is not None, "rate_dps": p["scan_rate_dps"],
                    "turns": p["scan_turns"], "reason": "no flashing blue entry buoy found"}
        if sub == "SCOUT_ENTRY":
            pose = ctx.b.pose
            obstacles = [o for o in self.obstacles(ctx) if self.check_xy is None or not self.scout_then_check
                         or math.hypot(o[0] - self.check_xy[0], o[1] - self.check_xy[1]) > 0.5]
            path = gp.resample(gp.avoid([(pose.x_n, pose.y_e), self.scout_goal], obstacles, clr=self.clr), 0.5)
            self._planned(ctx, "scout", path, obstacles)
            v = p["approach_v"] if self.scout_then_check else p["scout_v"]
            return {"points": path, "speed_mps": v, "la_m": p["la_m"], "corridor_m": p["corridor_m"], "end_r_m": 1.0}
        if sub == "CHECK_BLUE":
            return {"until": lambda b: self._check_decided(ctx), "face": self.check_xy, "max_s": p["check_s"], "rate_dps": p["scan_rate_dps"],
                    "still_deg": p.get("check_still_deg", 0.0), "reason": "the light stayed undecided"}
        if sub == "APPROACH_LEAD":
            pose = ctx.b.pose
            obstacles = self.obstacles(ctx)
            path = gp.resample(gp.avoid([(pose.x_n, pose.y_e), self.lead_goal], obstacles, clr=self.clr), 0.5)
            self._planned(ctx, "lead", path, obstacles)
            return {"points": path, "speed_mps": p["transit_v"], "la_m": p["la_m"], "corridor_m": p["corridor_m"], "end_r_m": 1.0}
        if sub == "LOOK":
            return {"until": lambda b: self._look_done(ctx), "face": self.look_xy, "max_s": p["look_s"], "rate_dps": p["scan_rate_dps"],
                    "reason": "the pending buoy stayed undecided"}
        if sub == "CIRCLE_ENTRY_CW":
            if self.entry_id is not None:                # v0.1.54: a re-plan of the approach (a new object on its way)
                e = next((e for e in self.buoys(ctx) if e.id == self.entry_id), None)
                return self._circle_args(ctx, e, "cw", xy=self.entry_xy)
            e = self.marker(ctx, "entry")
            if e is None and self.check_xy is not None:     # v0.1.93.2: never a None entry (crashed the sim twice): the last checked buoy
                e = next((x for x in self.buoys(ctx) if x.id == self.check_id), None)
                if e is None:
                    return self._circle_args(ctx, None, "cw", xy=self.check_xy)
            if e is None:                                   # unreachable since ENTRY_SEEN / ENTRY_CHECKED pin the entry; never crash the run
                ctx.log("T1 CIRCLE_ENTRY_CW with no entry pinned - circling where the boat is (report this)")
                return self._circle_args(ctx, None, "cw", xy=(ctx.b.pose.x_n, ctx.b.pose.y_e))
            self.entry_xy, self.entry_id = (e.x_n, e.y_e), e.id
            ctx.log(f"T1 entry locked: #{e.id} N {e.x_n:+.1f} E {e.y_e:+.1f} ({e.n_obs} sightings, votes {e.colour_votes})")
            return self._circle_args(ctx, e, "cw")
        if sub == "TRANSIT" and self.current is None and ctx.b.pose is not None:    # v0.1.94.1: never a None gate (lucky crashed the sim)
            ch = self.choose(ctx)
            if ch.item is not None and not ch.wrong_side:
                self.current = ch.item
            else:
                pose = ctx.b.pose
                ctx.log("T1 TRANSIT with no gate picked - holding here for the next pick")
                return {"points": [(pose.x_n, pose.y_e), (pose.x_n, pose.y_e)], "speed_mps": 0.0, "la_m": p["la_m"],
                        "corridor_m": p["corridor_m"], "end_r_m": 5.0}
        if sub == "TRANSIT" and p.get("gate_steps") and p["planner"] == "simple":   # v0.1.92: to the gate's pre-point only
            pose, it = ctx.b.pose, self.current
            lead = self._lead(it)
            pre = (it.mid[0] - lead * it.dir[0], it.mid[1] - lead * it.dir[1])
            last = len(self.items(ctx)) <= 1                  # the last gate known: run on until the stern is clear (the exit leg turns)
            pm = float(p["post_m"]) + (1.0 if last else 0.0)
            post = (it.mid[0] + pm * it.dir[0], it.mid[1] + pm * it.dir[1])
            self.thru = {"key": it.key, "mid": it.mid, "dir": it.dir, "width": it.width, "pre": pre, "post": post, "crossed": None,
                         "prev_along": None, "n": len(self.passed) + 1}
            self.squeeze_xy = None
            if p.get("squeeze"):
                sq = self._squeeze_plan(ctx, it, lead, pm)
                if sq is not None:
                    self.thru.update(sq)
                    pre = sq.get("pre", pre)
            obstacles = [o for o in self.obstacles(ctx) if (o[0], o[1]) not in (it.red, it.green)]   # the pair being passed is the corridor
            ring = (self.thru.get("squeeze") or {}).get("ring")
            if ring is not None:                              # v0.1.93: the squeezed object keeps r + beam/2 + margin, not its 2 m ring
                obstacles = [o for o in obstacles if math.hypot(o[0] - ring[0], o[1] - ring[1]) > 0.3] + [
                    (ring[0], ring[1], float(p["squeeze_r_m"]))]
            path = gp.resample(gp.avoid([(pose.x_n, pose.y_e), pre], obstacles, clr=self.clr), 0.5)
            if len(path) < 2:
                path = [(pose.x_n, pose.y_e), pre]
            self._planned(ctx, f"gate {len(self.passed) + 1} run-in", path, obstacles)
            self.why = f"to the pre-point of gate {len(self.passed) + 1} ({lead:.1f} m before it, {it.width:.1f} m wide)"
            ctx.log(f"T1 gate {len(self.passed) + 1} {it.key}: to its pre-point {lead:.1f} m before the line, through {it.hdg_deg:03.0f}, "
                    f"{it.width:.1f} m wide" + (" (lucky)" if it.lucky else ""))
            return {"points": path, "speed_mps": self._gate_v(ctx, "transit"), "la_m": p["la_m"], "corridor_m": p["corridor_m"], "end_r_m": p["pre_end_r_m"]}
        if sub == "TRANSIT":
            pose = ctx.b.pose
            cfg = self.pcfg(ctx)
            path, self.leg_how = gp.leg(cfg, (pose.x_n, pose.y_e, pose.hdg_deg), self.current)
            own = (self.current.red, self.current.green)
            others = [o for o in cfg.obstacles if (o[0], o[1]) not in own]
            c_own = gp.path_clearance(path, [o for o in cfg.obstacles if (o[0], o[1]) in own], self.clr)
            self._planned(ctx, f"gate {len(self.passed) + 1} leg", path, others)
            ctx.log(f"T1 gate {len(self.passed) + 1} {self.current.key}: {self.leg_how}, {len(path)} pts, through {self.current.hdg_deg:03.0f}, "
                    f"{self.current.width:.1f} m wide: {c_own:.2f} m off its buoys" + (" (narrower than the rule: on the centre line)"
                                                                                       if c_own < self.clr.clear_m else ""))
            return {"points": path, "speed_mps": p["transit_v"], "la_m": p["la_m"], "corridor_m": p["corridor_m"], "end_r_m": 0.8}
        if sub == "SCAN":
            return {"until": lambda b: self._scan_found(ctx), "rate_dps": p["scan_rate_dps"], "turns": p["scan_turns"],
                    "reason": self.refused or "no gate ahead and no exit buoy found"}
        if sub == "EXIT":
            ext = next((e for e in self.buoys(ctx) if e.id == self.exit_id), None) or self.marker(ctx, "exit")
            self.exit_xy, self.exit_id = (ext.x_n, ext.y_e), ext.id
            pose = ctx.b.pose
            r = self._radius(ctx, ext, "exit")
            obstacles = self.obstacles(ctx)
            goal = self._ring_goal((ext.x_n, ext.y_e), r, (pose.x_n, pose.y_e), obstacles)
            path = gp.resample(gp.avoid([(pose.x_n, pose.y_e), goal], obstacles, clr=self.clr), 0.5)
            self._planned(ctx, "exit leg", path, obstacles, skip_end_m=1.0)
            return {"points": path, "speed_mps": p["transit_v"], "la_m": p["la_m"], "corridor_m": p["corridor_m"], "end_r_m": 1.0}
        if sub == "CIRCLE_EXIT_CCW":
            ext = next((e for e in self.buoys(ctx) if e.id == self.exit_id), None)
            return self._circle_args(ctx, ext, "ccw", xy=self.exit_xy)
        return {}

    def _radius(self, ctx: TaskContext, e, which: str) -> float:
        """v0.1.54: the circle's radius by the clearance rule (gatepath.circle_radius), chosen once per marker."""
        if which not in self.circle_r:
            others = [o for o in self.obstacles(ctx) if math.hypot(o[0] - e.x_n, o[1] - e.y_e) > 0.5]
            r, edge = gp.circle_radius((e.x_n, e.y_e), others, float(self.p["circle_r_m"]), self.clr, float(self.p["circle_r_max_m"]))
            self.circle_r[which] = r
            self.min_planned = edge if self.min_planned is None else min(self.min_planned, edge)
            if abs(r - float(self.p["circle_r_m"])) > 0.01 or edge < self.clr.clear_m:
                ctx.log(f"T1 {which} circle r {r:.2f} m (circle_r_m {float(self.p['circle_r_m']):.1f}): {edge:.2f} m off the nearest object"
                        + (" - narrower than the rule" if edge < self.clr.clear_m else ""))
        return self.circle_r[which]

    def _circle_args(self, ctx: TaskContext, e, direction: str, xy=None) -> Dict[str, Any]:
        p = self.p
        x, y = xy if xy is not None else (e.x_n, e.y_e)
        which = "entry" if direction == "cw" else "exit"
        if e is not None:
            r = self._radius(ctx, e, which)
        else:
            r = self.circle_r.get(which, float(p["circle_r_m"]))
        out = {"x_n": x, "y_e": y, "r_m": r, "direction": direction, "speed_mps": p["circle_v"], "la_m": p["la_m"],
               "approach_speed": max(float(p["circle_v"]), 0.15),    # v0.1.92: ALIGN_ORBIT at 0.15
               "speed_max": float(p.get("circle_v_max") or 0.0), "r_for_max": bool(p.get("circle_r_for_max"))}   # v0.1.94.2: drift boost
        if direction == "cw" and p.get("circle_exit_tangent") and float(p.get("lead_in_m") or 0.0) > 0:   # v0.1.92 (sim world D): leave the entry circle on the tangent to the
            ch = self.choose(ctx)                                           # first gate's run-in point, not wherever the lap ends
            if ch.item is not None and not ch.wrong_side:
                L = float(p["lead_in_m"])
                out["exit_toward"] = (ch.item.mid[0] - L * ch.item.dir[0], ch.item.mid[1] - L * ch.item.dir[1])
        pose = ctx.b.pose
        if pose is not None and math.hypot(pose.x_n - x, pose.y_e - y) > r + 1.5:      # v0.1.54: to the ring along an avoided path
            obstacles = [o for o in self.obstacles(ctx) if math.hypot(o[0] - x, o[1] - y) > 0.5]
            goal = self._ring_goal((x, y), r, (pose.x_n, pose.y_e), obstacles)
            out["approach"] = gp.resample(gp.avoid([(pose.x_n, pose.y_e), goal], obstacles, clr=self.clr), 0.5)
            self._planned(ctx, f"{which} circle approach", out["approach"], obstacles, skip_end_m=1.0)
        return out

    def _ring_goal(self, c, r: float, boat, obstacles) -> Tuple[float, float]:
        """v0.1.92 (the overnight sim, world E): where the boat joins a circle - the ring point nearest the boat that keeps every
        other object's keep disc (was: the point straight toward the boat, which can sit on an obstacle next to the marker: the
        avoided path then ends inside it). Every 10 deg; none free -> the straight one."""
        others = [o for o in obstacles if math.hypot(o[0] - c[0], o[1] - c[1]) > 0.5]
        u = gp._unit(boat[0] - c[0], boat[1] - c[1])
        straight = (c[0] + r * u[0], c[1] + r * u[1])
        best = None
        for k in range(36):
            a = math.radians(10.0 * k)
            q = (c[0] + r * math.cos(a), c[1] + r * math.sin(a))
            if all(math.hypot(q[0] - o[0], q[1] - o[1]) >= self.clr.keep(float(o[2])) for o in others):
                d = math.hypot(q[0] - boat[0], q[1] - boat[1])
                if best is None or d < best[0]:
                    best = (d, q)
        return best[1] if best else straight

    def _reachable(self, ctx: TaskContext, goal, obstacles) -> bool:
        """v0.1.92 (world F): a scouting / checking leg only when the avoided path keeps the hull off every object (>= track_m + 0.05 m edge: the boat wanders ~0.45 m off its path);
        in a compact course the planner can find no way round and the straight line runs over a gate buoy."""
        pose = ctx.b.pose
        path = gp.avoid([(pose.x_n, pose.y_e), goal], obstacles, clr=self.clr)
        return gp.path_clearance(path, obstacles, self.clr) >= self.clr.track_m + 0.05      # the tracker's own allowance + 5 cm

    def _scan_found(self, ctx: TaskContext) -> bool:
        if self.pending_lead(ctx, None) is not None:
            return True                                  # back to TRANSIT, which LOOKs at it
        ch = self.choose(ctx)
        if ch.item is not None and not ch.wrong_side:
            return True
        return ch.item is None and self.marker(ctx, "exit") is not None

    def _next_from_transit(self, ctx: TaskContext) -> None:
        """At TRANSIT with no leg running: pick the next gate, or go to EXIT / SCAN."""
        ch = self.choose(ctx)
        lead = self.pending_lead(ctx, ch.item if ch.item is not None and not ch.wrong_side else None)
        if lead is not None:                             # v0.1.39: decide the nearer buoy before passing it by
            self.look_id, self.look_xy = lead.id, (lead.x_n, lead.y_e)
            ctx.log(f"T1 look at #{lead.id} N {lead.x_n:+.1f} E {lead.y_e:+.1f}: pending ({lead.n_obs} sightings, votes "
                    f"{ {k: round(v, 1) for k, v in lead.colour_votes.items()} })")
            self.m.handle("LOOK")
            return
        if ch.item is not None and not ch.wrong_side:
            self.current, self.refused = ch.item, None
            return
        if ch.item is None and self.marker(ctx, "exit") is not None:
            self.m.handle("TO_EXIT")
            return
        if ch.item is not None and ch.wrong_side:
            self.refused = f"gate {ch.item.key} would pass RED to port from here: refusing it"
            ctx.log("T1 " + self.refused)
        self.m.handle("NO_GATE", reason=self.refused or "no usable gate ahead")

    def step(self, ctx: TaskContext):
        if not self.holding:
            st = self.m.state
            if st == "WAIT_START":
                if getattr(self, "uav_mode", False):
                    self.m.handle("GO_UAV")
                elif str(self.p.get("start_mode")) == "advance360":
                    self.why = f"{float(self.p['start_advance_m']):.0f} m ahead at {float(self.p['start_v']):.1f} m/s, then one turn on the spot"
                    self.m.handle("GO_START")
                else:
                    self.m.handle("GO")
            elif st == "THROUGH":
                self._track_through(ctx)
            elif st == "SCOUT_ENTRY" and self.scout_then_check and self.check_id is not None:
                self._nearer_suspect(ctx)
            elif st == "WAIT_UAV_MAP":
                self.wait_uav_map(ctx)
            elif st == "TRANSIT" and self._beh is None:
                self._next_from_transit(ctx)
            if ctx.b.map is not None:
                self._follow_merges(ctx.b.map)          # v0.1.94: before anything reads entry / exit / check ids
            self._lucky_retry(ctx)                      # v0.1.94.2
            self.watch_locked(ctx)                      # v0.1.54
            self.replan_check(ctx)
            if self.m.state not in ("WAIT_START", "WAIT_UAV_MAP", "DONE", "ABORTED"):
                self.maybe_report(ctx)
        out = super().step(ctx)
        if out.out is not None and self.m.state in ("TRANSIT", "ALIGN_GATE", "THROUGH") and self.current is not None:
            out.out.info["gate"] = list(self.current.key)
        if out.out is not None:
            out.out.info["micro"] = self.micro(ctx)       # v0.1.92: the stage line / the run banner (mission manager -> extra["micro"])
        if out.out is not None and getattr(self, "uav_gate", None):        # v0.1.89: the touchdown sequence -> the map digest -> the OCS
            out.out.info["uav_gate"] = dict(self.uav_gate, waiting=self.m.state == "WAIT_UAV_MAP")
        return out

    def on_behaviour_done(self, sub: str, ctx: TaskContext) -> None:
        if sub == "SEARCH_ENTRY":
            e = self.marker(ctx, "entry")
            if (e is not None and self.p.get("confirm_entry") and not getattr(self, "uav_mode", False)
                    and getattr(e, "led", None) is None and e.id not in self.checked):
                self.check_id, self.check_xy = e.id, (e.x_n, e.y_e)
                pose = ctx.b.pose
                d = math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e)
                if self.scout_t0 is None:
                    self.scout_t0 = ctx.b.now
                if d <= float(self.p["check_r_m"]):
                    ctx.log(f"T1 entry candidate #{e.id} ({d:.1f} m, votes {e.colour_votes}): confirming its light first")
                    self.m.handle("CHECK")
                    return
                obstacles = self.obstacles(ctx)
                u = gp._unit(pose.x_n - e.x_n, pose.y_e - e.y_e)
                goal = gp.free_point((e.x_n + float(self.p["check_hold_m"]) * u[0], e.y_e + float(self.p["check_hold_m"]) * u[1]), obstacles, self.clr)
                if self._reachable(ctx, goal, obstacles):
                    self.scout_goal, self.scout_then_check = goal, True
                    ctx.log(f"T1 entry candidate #{e.id} ({d:.1f} m, votes {e.colour_votes}): to {float(self.p['check_hold_m']):.0f} m of it to confirm")
                    self.m.handle("SCOUT")
                    return
            if e is None:                                # v0.1.93.2: the light flipped since the search's check - search on
                return
            self.entry_xy, self.entry_id = (e.x_n, e.y_e), e.id     # v0.1.93.2: pinned now (the map may flip it before the circle starts)
            ctx.log(f"T1 entry locked: #{e.id} N {e.x_n:+.1f} E {e.y_e:+.1f} ({e.n_obs} sightings, votes {e.colour_votes})")
            self.m.handle("ENTRY_SEEN")
        elif sub == "CIRCLE_ENTRY_CW":
            if self.lucky_exit_id is not None and self.entry_id == self.lucky_exit_id:   # v0.1.94.2: the exit stand-in circled
                ctx.log(f"T1 LUCKY: exit #{self.lucky_exit_id} orbited as the stand-in entry - the gates in their own direction, then it CCW")
                self.entry_xy, self.entry_id, self.lucky_from_exit = None, None, True
                self.circle_r.pop("entry", None)
            self.m.handle("CIRCLED")
        elif sub == "TRANSIT" and self.p.get("gate_steps") and self.p["planner"] == "simple" and self.thru is not None:
            self.why = f"at the pre-point of gate {self.thru['n']}: turning onto its line (within {float(self.p['align_deg']):.0f} deg)"
            self.m.handle("ALIGN")
        elif sub in ("START_ADVANCE",):
            self.m.handle("ADVANCED")
        elif sub == "FINAL_ADVANCE":
            self.m.handle("FINAL_ADVANCED")
        elif sub == "FINAL_SCAN":
            self._final_scan_end(ctx)
        elif sub == "START_SCAN":
            if self.blue_fix_id is not None:            # v0.1.93.2: the turn ended early on a blue lantern
                ctx.log(f"T1 blue fix #{self.blue_fix_id} during the start turn: stop yawing - look-lock + approach")
            self.m.handle("START_SCANNED")
        elif sub == "LOOK_LOCK":
            self._after_lock(ctx)
        elif sub == "ALIGN_GATE":
            self.why = f"lined up on gate {self.thru['n']}: through at {self._gate_v(ctx, 'through'):.2f} m/s on its line" + (
                " (confident)" if self.gate_confident(ctx) else "")
            self.m.handle("THRU")
        elif sub == "THROUGH":
            self._through_done(ctx, finished=True)
        elif sub == "TRANSIT" and self.current is None:   # v0.1.94.1: the no-gate hold ended: pick again on the next step
            pass
        elif sub == "TRANSIT":
            it = self.current
            pose = ctx.b.pose
            along = (pose.x_n - it.mid[0]) * it.dir[0] + (pose.y_e - it.mid[1]) * it.dir[1]
            self.passed.append({"mid": it.mid, "dir": it.dir, "key": it.key, "correct": along > 0})
            self.last_dir = it.dir
            self.current = None
            ctx.log(f"T1 gate {len(self.passed)} {it.key} passed ({'correct' if along > 0 else 'NOT crossed'}, {along:+.1f} m past)")
        elif sub == "SCAN":
            self.m.handle("FOUND" if self.choose(ctx).item is not None or self.pending_lead(ctx, None) is not None else "TO_EXIT")
        elif sub == "LOOK":
            self._looked(ctx, "decided")
        elif sub == "APPROACH_LEAD":
            self.m.handle("ARRIVED")
        elif sub == "SCOUT_ENTRY":
            if self.scout_then_check and self.check_id is not None:
                self.m.handle("CHECK")
            else:
                self.m.handle("SCOUTED")
        elif sub == "CHECK_BLUE":
            self._check_result(ctx, decided=True)
        elif sub == "EXIT":
            self.m.handle("AT_EXIT")
        elif sub == "CIRCLE_EXIT_CCW":
            self.maybe_report(ctx, complete=True)
            self.m.handle("CIRCLED")

    def on_guard_block(self, ctx: TaskContext, key) -> None:
        """v0.1.94 (fork B's sim: SAFE_HOLD "obstacle persists" after the boat drove at a lead / a scouted buoy until the guard stopped it,
        backed off, re-planned and drove at it again): the 2nd guard stop while approaching a lead or scouting a buoy = arrived - the
        thing is right there; the stage ends as if reached (SCOUT_ENTRY -> its check from here; APPROACH_LEAD -> the search goes on)."""
        st = self.m.state
        if st not in ("APPROACH_LEAD", "SCOUT_ENTRY", "FINAL_ADVANCE"):
            return
        ctx.log(f"T1 guard stopped {st} twice on the same thing: taken as arrived (no 3rd try into it)")
        if self._beh is not None:
            self._beh.cancel()
            self._beh = None
        self.on_behaviour_done(st, ctx)

    def gate_pass_xy(self):
        """v0.1.94: the two buoys of the gate being lined up on / passed (ALIGN_GATE, THROUGH) for the guard - None otherwise."""
        if self.m.state not in ("ALIGN_GATE", "THROUGH") or self.thru is None or self.current is None:
            return None
        it = self.current
        return [tuple(it.red), tuple(it.green)]

    # ---- v0.1.94: confident gates ----------------------------------------------------------------------------------------
    def gate_confident(self, ctx: TaskContext, it=None) -> bool:
        """Both buoys of the current gate mapped well: >= conf_min_obs sightings, scatter <= conf_sigma_m, seen from <= conf_seen_m at least
        once (a near, well-ranged sighting), the colour from their own lamp (a decided LED or the lamp's colour - or, with no lamp reader at
        all, decided flashing by the votes), and not a lucky pair."""
        it = it or self.current
        m = ctx.b.map
        if it is None or m is None or getattr(it, "lucky", False):
            return False
        p = self.p
        ents = {e.id: e for e in m.entities("buoy")}
        for eid, want in zip(it.key, ("red", "green")):
            e = ents.get(eid)
            if e is None or e.colour != want or e.n_obs < int(p["conf_min_obs"]) or e.sigma_m > float(p["conf_sigma_m"]):
                return False
            if float(getattr(e, "min_range_m", 1e9) or 1e9) > float(p["conf_seen_m"]):
                return False
            led, lamp = getattr(e, "led", None), getattr(e, "lamp", None)
            if not ((led and led.get("colour") == want) or (lamp and lamp.get("colour") == want) or (led is None and lamp is None and e.light == "flash")):
                return False
        return True

    def _gate_v(self, ctx: TaskContext, which: str) -> float:
        p = self.p
        base, fast = float(p[f"{which}_v"]), float(p.get(f"{which}_v_conf") or 0.0)
        if fast > 0 and self.gate_confident(ctx):
            return max(base, fast)
        return base

    # ---- v0.1.92: the look-lock and the gate steps ----------------------------------------------------------------------------
    def _after_lock(self, ctx: TaskContext) -> None:
        pose = ctx.b.pose
        xy = self.check_xy
        d = math.hypot(xy[0] - pose.x_n, xy[1] - pose.y_e) if (pose is not None and xy is not None) else 0.0
        if d <= float(self.p["check_r_m"]):
            self.why = f"suspect #{self.check_id} at {d:.1f} m: reading its light"
            self.m.handle("LOCK_CHECK")
        else:
            self.why = f"to {float(self.p['check_hold_m']):.0f} m of suspect #{self.check_id} ({d:.1f} m) at {float(self.p['approach_v']):.1f} m/s"
            self.m.handle("LOCKED")

    def _nearer_suspect(self, ctx: TaskContext) -> None:
        """Cowork addendum 1: a nearer suspect interrupts the approach to a far one (2 m nearer at least)."""
        pose = ctx.b.pose
        if pose is None or self.check_xy is None:
            return
        cur = math.hypot(self.check_xy[0] - pose.x_n, self.check_xy[1] - pose.y_e)
        for e in self.check_candidates(ctx):
            if e.id == self.check_id:
                continue
            d = math.hypot(e.x_n - pose.x_n, e.y_e - pose.y_e)
            if d < cur - 2.0:
                ctx.log(f"T1 a nearer suspect #{e.id} ({d:.1f} m) than #{self.check_id} ({cur:.1f} m): switching")
                if self._beh is not None:
                    self._beh.cancel()
                    self._beh = None
                self._scout_next(ctx)
                return

    def _track_through(self, ctx: TaskContext) -> None:
        """The gate line's crossing: where (lateral) the boat's centre crossed it."""
        t, pose = self.thru, ctx.b.pose
        if t is None or pose is None:
            return
        rx, ry = pose.x_n - t["mid"][0], pose.y_e - t["mid"][1]
        along = rx * t["dir"][0] + ry * t["dir"][1]
        lat = -rx * t["dir"][1] + ry * t["dir"][0]
        if t["prev_along"] is not None and t["prev_along"] < 0.0 <= along and t["crossed"] is None:
            t["crossed"] = lat
        t["prev_along"] = along

    def _through_done(self, ctx: TaskContext, finished: bool) -> None:
        """Passed only when the hull's centre crossed the gate line inside width / 2 - pass_margin_m; else MISSED -> back to the pre-point
        (miss_tries times), then on to the next gate, logged."""
        self._track_through(ctx)
        t, it = self.thru, self.current
        ok = t is not None and t["crossed"] is not None and abs(t["crossed"]) <= t["width"] / 2.0 - float(self.p["pass_margin_m"])
        n = t["n"] if t else len(self.passed) + 1
        if ok:
            self.passed.append({"mid": it.mid, "dir": it.dir, "key": it.key, "correct": True, "off_m": round(t["crossed"], 2)})
            self.last_dir, self.current, self.thru = it.dir, None, None
            self.why = f"gate {n} passed ({abs(t['crossed']):.1f} m off centre)"
            ctx.log(f"T1 gate {n} {it.key} passed ({t['crossed']:+.2f} m off centre)")
        else:
            k = it.key if it is not None else None
            self.gate_misses[k] = self.gate_misses.get(k, 0) + 1
            where = "not crossed" if (t is None or t["crossed"] is None) else f"crossed {t['crossed']:+.2f} m off centre"
            if self.gate_misses[k] <= int(self.p["miss_tries"]):
                self.why = f"gate {n} MISSED ({where}): back to its pre-point, try {self.gate_misses[k] + 1}"
                ctx.log(f"T1 gate {n} {k} MISSED ({where}) - back to its pre-point (try {self.gate_misses[k] + 1})")
            else:
                self.passed.append({"mid": it.mid, "dir": it.dir, "key": it.key, "correct": False})
                self.last_dir, self.current, self.thru = it.dir, None, None
                self.why = f"gate {n} given up after {self.gate_misses[k]} misses"
                ctx.log(f"T1 gate {n} {k} MISSED {self.gate_misses[k]} times - on to the next gate")
        self.m.handle("THRU_DONE")

    def _squeeze_plan(self, ctx: TaskContext, it, lead: float, post_m: float) -> Optional[Dict[str, Any]]:
        """v0.1.93 (WP49 §3.5): an unknown object (a guard ring) on this gate's run-in / through (along -lead-1.5 .. +1.0, |lateral| < 1.5 m) ->
        a dog-leg on the gap beside it; one beyond the gate (along 1.0 .. 5.0) -> a short run-out (0.75 m) so the next gate's dog-leg takes it.
        None = no squeeze (nothing there, or no gap >= beam + 2 margin: v0.1.92's behaviour)."""
        p, m = self.p, ctx.b.map
        if m is None:
            return None
        d = it.dir
        n = (-d[1], d[0])                                 # lateral + = this side
        R, beam, marg = float(p["squeeze_r_m"]), float(p["hull_beam_m"]), float(p["squeeze_margin_m"])
        need = beam + 2.0 * marg
        try:
            obs = m.entities("obstacle", include_stale=True)
        except TypeError:
            obs = m.entities("obstacle")
        best = None
        for o in obs:
            behind = float(getattr(o, "ring_behind_m", 1.0) or 1.0)
            fx, fy = o.x_n - behind * d[0], o.y_e - behind * d[1]           # the seen face (the ring is centred behind it)
            a = (fx - it.mid[0]) * d[0] + (fy - it.mid[1]) * d[1]
            lat = (fx - it.mid[0]) * n[0] + (fy - it.mid[1]) * n[1]
            if abs(lat) >= 1.5:
                continue
            if 1.0 < a <= 5.0:
                self.log_once(ctx, f"sq_beyond_{it.key}", f"T1 SQUEEZE (risk accepted): an unknown object {a:.1f} m past gate {len(self.passed) + 1}: "
                              "short run-out (0.75 m), its guard stop 0.3 m; the next gate's dog-leg passes it")
                self.squeeze_xy = (fx, fy)
                return {"post": (it.mid[0] + 0.75 * d[0], it.mid[1] + 0.75 * d[1]), "squeeze_beyond": True}
            if not (-lead - 1.5 <= a <= 1.0):
                continue
            if best is None or a < best[0]:
                best = (a, lat, o)
        if best is None:
            return None
        a_o, lat_o, o = best
        edge = it.width / 2.0 - 0.25
        wl, wr = (edge, -edge) if abs(a_o) < 1.0 else (min(2.5, edge + 1.0), -min(2.5, edge + 1.0))
        gl, gr = wl - (lat_o + R), (lat_o - R) - wr
        pose = ctx.b.pose
        own = 1 if ((pose.x_n - it.mid[0]) * n[0] + (pose.y_e - it.mid[1]) * n[1]) >= 0 else -1
        side = None
        if (own > 0 and gl >= need + 0.3) or (own < 0 and gr >= need + 0.3):
            side = own
        elif max(gl, gr) >= need:
            side = 1 if gl >= gr else -1
        if side is None:
            ctx.log(f"T1 no squeeze past the object before gate {len(self.passed) + 1}: gaps {gl:.2f} / {gr:.2f} m < {need:.2f} m (the old way)")
            return None
        off = ((lat_o + R) + wl) / 2.0 if side > 0 else (wr + (lat_o - R)) / 2.0
        clamp = it.width / 2.0 - 0.25 - beam / 2.0 - 0.12
        hull = float(p["hull_len_m"])

        def lat_at(a):
            lo, hi = a_o - R - hull / 2.0, a_o + R + hull / 2.0
            w = 1.0 if lo <= a <= hi else max(0.0, 1.0 - (lo - a) / 2.0) if a < lo else max(0.0, 1.0 - (a - hi) / 2.0)
            v = off * w
            return max(-clamp, min(clamp, v)) if abs(a) <= 1.2 else v
        pts = []
        a = -lead
        while a <= post_m + 1e-6:
            L = lat_at(a)
            pts.append((it.mid[0] + a * d[0] + L * n[0], it.mid[1] + a * d[1] + L * n[1]))
            a += 0.25
        self.squeeze_xy = (o.x_n - float(getattr(o, "ring_behind_m", 1.0) or 1.0) * d[0], o.y_e - float(getattr(o, "ring_behind_m", 1.0) or 1.0) * d[1])
        gap = gl if side > 0 else gr
        msg = (f"SQUEEZE (risk accepted): unknown object {'left' if lat_o > 0 else 'right'} of the line {-a_o:.1f} m before gate "
               f"{len(self.passed) + 1} - pass it on its {'left' if side > 0 else 'right'} with {gap:.2f} m of water "
               f"({(gap - beam) / 2.0:.2f} m a side) at {float(p['squeeze_v']):.2f} m/s")
        ctx.log("T1 " + msg)
        self.why = f"squeeze {len(self.passed) + 1}: {gap:.2f} m of water on the {'left' if side > 0 else 'right'}"
        return {"pre": pts[0], "path": pts, "squeeze": {"side": side, "offset_m": round(off, 2), "gap_m": round(gap, 2), "ring": (o.x_n, o.y_e)}}

    def log_once(self, ctx: TaskContext, key: str, text: str) -> None:
        seen = self.__dict__.setdefault("_logged", set())
        if key not in seen:
            seen.add(key)
            ctx.log(text)

    def micro(self, ctx: TaskContext) -> Dict[str, Any]:
        """v0.1.92 (Dmitrii: the micro-stages in the GUI, "which might be next"): {stage, sub, why, next, mods, gates}."""
        st = self.m.state
        sub = ""
        if st in ("TRANSIT", "ALIGN_GATE", "THROUGH") and self.thru is not None:
            sub = {"TRANSIT": "to pre-point", "ALIGN_GATE": "aligning", "THROUGH": "squeeze" if self.thru.get("squeeze") else "through"}[st] + f" {self.thru['n']}"
        elif st in ("LOOK_LOCK", "SCOUT_ENTRY", "CHECK_BLUE") and self.check_id is not None:
            sub = f"suspect #{self.check_id}"
        nxt = NEXT.get(st, "")
        if st == "THROUGH" and self.thru is not None:
            nxt = f"gate {self.thru['n']} passed -> gate {self.thru['n'] + 1} / EXIT"
        gates = " ".join((f"G{i + 1} ✓" if q.get("correct") else f"G{i + 1} ✗") for i, q in enumerate(self.passed))
        if self.current is not None and st in ("TRANSIT", "ALIGN_GATE", "THROUGH"):
            k = self.gate_misses.get(self.current.key, 0)
            gates = (gates + f" G{len(self.passed) + 1}" + (f" ✗{k}" if k else "")).strip()
        return {"stage": st, "sub": sub, "why": self.why, "next": nxt, "mods": self.mods(), "gates": gates}

    def mods(self) -> str:
        p = self.p
        return (f"start={'advance+360' if str(p.get('start_mode')) == 'advance360' else 'straight'} · orbit=bow "
                f"{float(p['circle_v']):.2f} m/s · follow=bow · squeeze {'on' if p.get('squeeze') else 'off'} · look-lock {'on' if p.get('look_lock') else 'off'} · "
                f"lucky {('ANY' if p.get('lucky_pairs') == 'any' else 'on') if p.get('lucky') else 'off'} · {float(p.get('scout_entry_s') or 0):.0f} s")   # v0.1.93.1: ANY

    def _looked(self, ctx: TaskContext, how: str) -> None:
        self.skipped.add(self.look_id)
        e = next((e for e in ctx.b.map.entities("buoy") if e.id == self.look_id), None) if ctx.b.map is not None else None
        ctx.log(f"T1 looked at #{self.look_id}: {how}" + (f" -> {e.colour} {e.light}" if e is not None else " (gone from the map)"))
        self.m.handle("LOOKED")

    def on_behaviour_abort(self, sub: str, reason: str, ctx: TaskContext) -> None:
        if sub == "LOOK":
            self._looked(ctx, "still undecided after %.0f s" % float(self.p["look_s"]))
            return
        if sub == "CHECK_BLUE":
            self._check_result(ctx, decided=False)
            return
        if sub == "START_ADVANCE":                       # v0.1.92: blocked / slow: on with the turn
            self.m.handle("ADVANCED")
            return
        if sub == "START_SCAN":
            self.m.handle("START_SCANNED")
            return
        if sub == "LOOK_LOCK":
            self._after_lock(ctx)
            return
        if sub == "ALIGN_GATE":
            ctx.log(f"T1 gate {self.thru['n'] if self.thru else '?'}: {reason}")
            self.m.handle("THRU")
            return
        if sub == "THROUGH":
            self._through_done(ctx, finished=False)
            return
        if sub == "SCOUT_ENTRY":                         # blocked / timed out on the way: the next candidate
            ctx.log(f"T1 scout leg aborted ({reason}): the next one")
            if self.check_id is not None:
                self.checked.add(self.check_id)
            self._scout_next(ctx)
            return
        if sub == "SEARCH_ENTRY" and float(self.p.get("scout_entry_s") or 0) > 0 and not getattr(self, "uav_mode", False):
            if self._scout_next(ctx):                    # v0.1.92: scout + check before the leads, the lucky and the hold
                return
        if sub == "SEARCH_ENTRY" and self.p.get("lucky") and self._lucky_go(ctx, "no flashing blue entry after the scan"):   # v0.1.90
            return
        lead_cap = int(self.p["scout_lead_tries"] if float(self.p.get("scout_entry_s") or 0) > 0 else self.p["lead_tries"])
        if sub == "SEARCH_ENTRY" and self.lead_tries < lead_cap:
            goal, how = self.map_lead(ctx), "toward a buoy that may be the entry"            # v0.1.54: the map lead first,
            if goal is None:                                                                   # v0.1.66: the UAV's map
                goal, how = self.uav_lead(ctx), "toward the UAV's buoys (UAV-positioned: the camera takes them over)"
            if goal is None:                                                                   # then the operator's search point
                goal, how = self.hint_lead(ctx), "to the search point (t1.search_lat / search_lon)"
            if goal is None:
                goal, how = self.ray_lead(ctx), f"{self.p['lead_step_m']:.0f} m toward a far sighting"
            if goal is not None:
                self.lead_tries += 1
                self.lead_goal = gp.free_point(goal, self.obstacles(ctx), self.clr)     # v0.1.54: never stop on top of an object
                ctx.log(f"T1 no entry in view: {how}, to N {goal[0]:+.1f} E {goal[1]:+.1f} "
                        f"(try {self.lead_tries}/{int(self.p['lead_tries'])})")
                self.m.handle("LEAD")
                return
        if sub == "SEARCH_ENTRY" and self.p.get("no_entry_finale") and not self.finale_done:   # v0.1.93.2 (Dmitrii): not the hold -
            self.finale_done = True                                                            # 3 m ahead, a quick 360, then DONE
            ctx.log(f"T1 no blue buoy found ({reason}): {float(self.p['final_advance_m']):.0f} m ahead, one quick turn, then the mission ends")
            self.m.handle("FINALE")
            return
        if sub == "FINAL_ADVANCE":                       # blocked / slow: on with the turn
            self.m.handle("FINAL_ADVANCED")
            return
        if sub == "FINAL_SCAN":
            self._final_scan_end(ctx)
            return
        if sub in ("SEARCH_ENTRY", "SCAN"):
            self.safe_hold = f"T1 {sub}: {reason}"          # the mission manager parks the vehicle in V5; RESUME scans again
            ctx.log(self.safe_hold)
            return
        super().on_behaviour_abort(sub, reason, ctx)
