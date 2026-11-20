"""Box inspection mission core.

This module is deliberately pure: it converts timestamped box observations and
vehicle state into high-level actions.  MAVLink, MQTT and camera I/O stay in the
existing adapters.  Keeping the mission core pure makes the scan/select/approach
logic replayable from SVO and MAVLink logs before it is allowed near a real FCU.

Coordinates are local N/E metres and headings are NED degrees clockwise.

v0.1.27 (ROADMAP WP8, the review docs/merge/BOX_MISSION_REVIEW.md §4, Dmitrii's answers 2026-11-04): the target is a Task 3
bay, so the blink requirement is a parameter (off: the bay indicator is steady); the vehicle picks the colour (GREEN unless
the operator gave one); the stop distance defaults to 1 m; the search is a dwell / strafe-leg loop in mode 20 with a length
and a side (no HOLD / frame alternation); ALIGN yaws in place on arrival and times out into HOLD; HOLD is a real hold the
operator resumes (the task wrapper raises it as a safe hold); `enter()` gives the task its entry stages.
"""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from ..companion.frames import wrap180, wrap360


COLOURS = ("red", "green", "blue")


class BoxState(str, Enum):
    PREFLIGHT = "PREFLIGHT"
    GO_SCAN = "GO_SCAN"
    OBSERVE = "OBSERVE"
    STRAFE = "STRAFE"
    ASK_OCS = "ASK_OCS"
    ALIGN = "ALIGN"
    APPROACH = "APPROACH"
    VALIDATE = "VALIDATE"
    STOP = "STOP"
    DONE = "DONE"
    HOLD = "HOLD"
    ABORTED = "ABORTED"


@dataclass(frozen=True)
class PointNE:
    n: float
    e: float


@dataclass(frozen=True)
class FaceGeometry:
    left: PointNE
    midpoint: PointNE
    right: PointNE

    @property
    def width_m(self) -> float:
        return math.hypot(self.right.n - self.left.n, self.right.e - self.left.e)


@dataclass(frozen=True)
class BoxSample:
    """One frame-level result after AGX range/bearing projection."""

    t: float
    box_id: int
    face: FaceGeometry
    colour: str
    lit: bool
    confidence: float = 1.0


@dataclass(frozen=True)
class StableBox:
    box_id: int
    face: FaceGeometry
    colour: str
    colour_share: float
    duration_s: float
    transition_rate_hz: float
    n_samples: int


@dataclass
class BoxTrack:
    box_id: int
    samples: List[BoxSample] = field(default_factory=list)
    max_samples: int = 200

    def add(self, sample: BoxSample) -> None:
        if sample.box_id != self.box_id:
            raise ValueError("sample id does not match track")
        self.samples.append(sample)
        self.samples.sort(key=lambda s: s.t)
        if len(self.samples) > self.max_samples:
            self.samples = self.samples[-self.max_samples :]

    def stable(
        self,
        *,
        observation_s: float = 4.0,
        min_colour_share: float = 0.8,
        min_confidence: float = 0.5,
        min_samples: int = 12,
        min_transition_rate_hz: float = 0.4,
        max_transition_rate_hz: float = 1.6,
        require_blink: bool = True,
        colours: Sequence[str] = COLOURS,
        min_span_frac: float = 0.95,
    ) -> Optional[StableBox]:
        good = [s for s in self.samples if s.confidence >= min_confidence]
        if len(good) < min_samples:
            return None
        end = good[-1].t
        window = [s for s in good if s.t >= end - observation_s]
        if len(window) < min_samples or window[-1].t - window[0].t < observation_s * min_span_frac:   # v0.1.58: min_span_frac (WP13)
            return None

        lit = [s for s in window if s.lit and s.colour.lower() in colours]      # v0.1.58: colours (white = a dock face, WP13)
        if not lit:
            return None
        counts: Dict[str, int] = {}
        for s in lit:
            c = s.colour.lower()
            counts[c] = counts.get(c, 0) + 1
        colour, votes = max(counts.items(), key=lambda kv: kv[1])
        share = votes / len(lit)
        if share < min_colour_share:
            return None

        transitions = sum(a.lit != b.lit for a, b in zip(window, window[1:]))
        duration = window[-1].t - window[0].t
        transition_rate = transitions / duration if duration > 0 else 0.0
        # A nominal 1 s on / 1 s off beacon changes state about once per second. A Task 3 bay indicator is steady (handbook
        # 3.3.4): require_blink=False accepts it (and a blinking one too).
        if require_blink and not (min_transition_rate_hz <= transition_rate <= max_transition_rate_hz):
            return None

        def med(getter):
            return statistics.median(getter(s.face) for s in window)

        face = FaceGeometry(
            left=PointNE(med(lambda f: f.left.n), med(lambda f: f.left.e)),
            midpoint=PointNE(med(lambda f: f.midpoint.n), med(lambda f: f.midpoint.e)),
            right=PointNE(med(lambda f: f.right.n), med(lambda f: f.right.e)),
        )
        return StableBox(
            box_id=self.box_id,
            face=face,
            colour=colour,
            colour_share=share,
            duration_s=duration,
            transition_rate_hz=transition_rate,
            n_samples=len(window),
        )


class ActionKind(str, Enum):
    HOLD = "HOLD"
    STILL = "STILL"            # v0.1.27: stay in mode 20 with a zero frame (observing), not a flight-mode HOLD
    YAW = "YAW"                # v0.1.27: turn in place to heading_deg (mode 20)
    GOTO = "GOTO"
    STRAFE = "STRAFE"
    REPORT = "REPORT"
    APPROACH = "APPROACH"
    BACK_AWAY = "BACK_AWAY"
    ALERT = "ALERT"
    DONE = "DONE"


@dataclass(frozen=True)
class MissionAction:
    kind: ActionKind
    target: Optional[PointNE] = None
    speed_mps: Optional[float] = None
    heading_deg: Optional[float] = None
    body: Dict[str, object] = field(default_factory=dict)


@dataclass(frozen=True)
class MissionSnapshot:
    now: float
    position: PointNE
    heading_deg: float
    preflight: Dict[str, bool]
    tracks: Sequence[BoxTrack]
    target_colour: Optional[str] = None
    target_visible: bool = True
    ocs_link: bool = True
    rtk_ok: bool = True


class BoxInspectionMission:
    """The plan's state machine, independent of the transport/adapters."""

    REQUIRED_PREFLIGHT = ("rtk", "heading", "camera", "thrusters", "ocs")
    ENTRY_STAGES = ("OBSERVE", "ALIGN", "APPROACH")      # v0.1.27: what enter() accepts (START_TASK args.from)

    def __init__(
        self,
        scan_point: PointNE,
        *,
        expected_boxes: int,
        align_range_m: float = 15.0,
        stop_range_m: float = 1.0,
        align_pos_tol_m: float = 1.0,
        align_heading_tol_deg: float = 5.0,
        require_blink: bool = False,
        default_colour: str = "green",
        observe_s: float = 4.5,
        strafe_step_m: float = 2.0,
        strafe_len_m: float = 6.0,
        strafe_side: int = 1,
        align_timeout_s: float = 90.0,
        stop_tol_m: float = 0.1,
        scan_hdg_deg: Optional[float] = None,
    ):
        if expected_boxes < 1:
            raise ValueError("expected_boxes must be positive")
        if not (stop_range_m > 0 and align_range_m > stop_range_m):
            raise ValueError("align_range_m must be greater than stop_range_m")
        self.scan_point = scan_point
        self.expected_boxes = expected_boxes
        self.align_range_m = align_range_m
        self.stop_range_m = stop_range_m
        self.align_pos_tol_m = align_pos_tol_m
        self.align_heading_tol_deg = align_heading_tol_deg
        self.require_blink = require_blink
        self.default_colour = (default_colour or "").lower()
        self.observe_s = observe_s
        self.strafe_step_m = strafe_step_m
        self.strafe_len_m = strafe_len_m
        self.strafe_side = 1 if strafe_side >= 0 else -1     # + = starboard (body frame), - = port
        self.align_timeout_s = align_timeout_s
        self.stop_tol_m = stop_tol_m
        self.scan_hdg_deg = scan_hdg_deg       # the search looks this way (None = the heading on arrival)
        self.state = BoxState.PREFLIGHT
        self.reason = ""
        self._report_sent = False
        self._rescan_used = False
        self.target: Optional[StableBox] = None
        self.align_point: Optional[PointNE] = None
        self.stop_point: Optional[PointNE] = None
        self.approach_heading_deg: Optional[float] = None
        self._target_lost_since: Optional[float] = None
        # v0.1.27 search / align / hold bookkeeping
        self.held_from: Optional[BoxState] = None
        self.hold_new = False                # True for the one step() that entered HOLD (the wrapper raises the safe hold)
        self._t_state: Optional[float] = None
        self._search_origin: Optional[PointNE] = None
        self._search_hdg: Optional[float] = None
        self._strafe_budget_m = strafe_len_m
        self._leg_start: Optional[PointNE] = None
        self._yawing = False

    @staticmethod
    def _distance(a: PointNE, b: PointNE) -> float:
        return math.hypot(a.n - b.n, a.e - b.e)

    def _stable_boxes(self, tracks: Iterable[BoxTrack]) -> List[StableBox]:
        return [b for b in (t.stable(require_blink=self.require_blink) for t in tracks) if b is not None]

    @staticmethod
    def _approach_geometry(face: FaceGeometry, boat: PointNE, distance_m: float) -> Tuple[PointNE, float]:
        tn, te = face.right.n - face.left.n, face.right.e - face.left.e
        norm = math.hypot(tn, te)
        if norm < 0.1:
            raise ValueError("front face is too narrow to define an approach line")
        tn, te = tn / norm, te / norm
        candidates = [(-te, tn), (te, -tn)]
        bn, be = boat.n - face.midpoint.n, boat.e - face.midpoint.e
        nn, ne = max(candidates, key=lambda q: q[0] * bn + q[1] * be)
        point = PointNE(face.midpoint.n + nn * distance_m, face.midpoint.e + ne * distance_m)
        heading = wrap360(math.degrees(math.atan2(face.midpoint.e - point.e, face.midpoint.n - point.n)))
        return point, heading

    @staticmethod
    def _speed_for_range(range_m: float) -> float:
        if range_m > 20.0:
            return 1.5
        if range_m > 10.0:
            return 0.8
        if range_m > 7.0:
            return 0.3
        return 0.15

    def _face_distance(self, pos: PointNE) -> float:
        """Distance from the target face's plane along the approach line (what the stop distance measures)."""
        assert self.target is not None and self.approach_heading_deg is not None
        h = math.radians(self.approach_heading_deg)
        m = self.target.face.midpoint
        return (m.n - pos.n) * math.cos(h) + (m.e - pos.e) * math.sin(h)

    def _set(self, state: BoxState, now: float) -> None:
        self.state = state
        self._t_state = now

    def _hold(self, reason: str) -> MissionAction:
        self.reason = reason
        return MissionAction(ActionKind.HOLD, body={"reason": reason})

    def _enter_hold(self, reason: str) -> MissionAction:
        """A hold the operator resumes (v0.1.27): remembers where it came from; resume() goes back there."""
        if self.state != BoxState.HOLD:
            self.held_from = self.state
            self.hold_new = True
        self.state = BoxState.HOLD
        return self._hold(reason)

    def resume(self, now: float) -> None:
        """The operator's RESUME after a hold: back to the held stage with fresh timers; a search that ran out of strafe
        length sweeps back the other way (twice the length, so it covers the far side of where it started)."""
        self.hold_new = False
        if self.state == BoxState.HOLD and self.held_from is not None:
            back = self.held_from
            if back in (BoxState.OBSERVE, BoxState.STRAFE) and self.reason.startswith("strafe limit"):
                self.strafe_side = -self.strafe_side
                self._search_origin, self._strafe_budget_m = None, 2.0 * self.strafe_len_m
                back = BoxState.OBSERVE
            self.held_from = None
            self._yawing = False
            self._set(back, now)
        elif self.state not in (BoxState.DONE, BoxState.ABORTED):
            self._t_state = now                 # a step park or a pause: the stage's timer starts again

    def _select(self, s: MissionSnapshot, boxes: List[StableBox]) -> str:
        """The target face (GREEN unless the operator chose) and its align / stop points; '' or why not."""
        colour = (s.target_colour or self.default_colour or "").lower()
        matches = [b for b in boxes if b.colour == colour]
        if not colour:
            return "no target colour"
        if len(matches) != 1:
            return f"target colour {colour}: expected one face, got {len(matches)}"
        self.target = matches[0]
        self.align_point, self.approach_heading_deg = self._approach_geometry(self.target.face, s.position, self.align_range_m)
        self.stop_point, _ = self._approach_geometry(self.target.face, s.position, self.stop_range_m)
        return ""

    def enter(self, stage: str, s: MissionSnapshot) -> str:
        """Start at an entry stage (START_TASK args.from): '' on success, else why not (nothing changed)."""
        if stage not in self.ENTRY_STAGES:
            return f"{stage} is not an entry stage"
        if stage == "OBSERVE":
            self._set(BoxState.OBSERVE, s.now)
            return ""
        boxes = self._stable_boxes(s.tracks)
        keep = (self.target, self.align_point, self.stop_point, self.approach_heading_deg)
        try:
            why = self._select(s, boxes)
        except ValueError as exc:
            why = str(exc)
        if why:
            self.target, self.align_point, self.stop_point, self.approach_heading_deg = keep
            return f"{stage} needs one stable face of the target colour in the map ({why})"
        if stage == "APPROACH":
            off = self._distance(s.position, self.align_point)
            herr = abs(wrap180(self.approach_heading_deg - s.heading_deg))
            if off > 3.0 or herr > 15.0:
                self.target, self.align_point, self.stop_point, self.approach_heading_deg = keep
                return f"APPROACH needs the boat on the approach line ({off:.1f} m from the align point, heading off {herr:.0f} deg)"
        self._report_sent = True
        self._set(BoxState[stage], s.now)
        return ""

    def _abort(self, reason: str, *, back_away: bool = False) -> MissionAction:
        self.state = BoxState.ABORTED
        self.reason = reason
        return MissionAction(ActionKind.BACK_AWAY if back_away else ActionKind.ALERT, body={"reason": reason})

    def step(self, s: MissionSnapshot) -> MissionAction:
        self.hold_new = False
        if self.state in (BoxState.DONE, BoxState.ABORTED):
            return MissionAction(ActionKind.DONE if self.state == BoxState.DONE else ActionKind.ALERT,
                                 body={"reason": self.reason})
        if not s.ocs_link or not s.rtk_ok:
            return self._enter_hold("OCS link lost" if not s.ocs_link else "RTK lost")
        if self.state == BoxState.HOLD:
            return self._hold(self.reason or "operator resume required")
        if self._t_state is None:
            self._t_state = s.now

        boxes = self._stable_boxes(s.tracks)
        if self.state == BoxState.PREFLIGHT:
            failed = [k for k in self.REQUIRED_PREFLIGHT if not s.preflight.get(k, False)]
            if failed:
                return self._hold("preflight failed: " + ", ".join(failed))
            self._set(BoxState.GO_SCAN, s.now)

        if self.state == BoxState.GO_SCAN:
            if boxes or self._distance(s.position, self.scan_point) <= self.align_pos_tol_m:
                self._set(BoxState.OBSERVE, s.now)
            else:
                return MissionAction(ActionKind.GOTO, target=self.scan_point, speed_mps=0.8)

        # The search: dwell (zero frames) for observe_s, then a strafe leg of strafe_step_m, until expected_boxes faces are
        # stable or the strafe length is used up (-> HOLD). Mode 20 throughout: no flight-mode change per tick.
        if self.state in (BoxState.OBSERVE, BoxState.STRAFE):
            if self._search_origin is None:
                self._search_origin = s.position
                self._search_hdg = s.heading_deg if self.scan_hdg_deg is None else wrap360(self.scan_hdg_deg)
            target = (s.target_colour or self.default_colour or "").lower()
            if len(boxes) >= self.expected_boxes:
                self._set(BoxState.ASK_OCS, s.now)
            elif self.state == BoxState.OBSERVE:
                if abs(wrap180(self._search_hdg - s.heading_deg)) > 10.0:     # turn to the search heading first
                    self._t_state = s.now
                    return MissionAction(ActionKind.YAW, heading_deg=self._search_hdg, body={"search": True})
                if s.now - self._t_state < self.observe_s:
                    return MissionAction(ActionKind.STILL, heading_deg=self._search_hdg,
                                         body={"stable_boxes": len(boxes), "expected_boxes": self.expected_boxes})
                if self._distance(s.position, self._search_origin) >= self._strafe_budget_m - 0.25:
                    if any(b.colour == target for b in boxes):    # the survey is short, but the target is known: go on
                        self._set(BoxState.ASK_OCS, s.now)
                    else:
                        return self._enter_hold(f"strafe limit {self._strafe_budget_m:.0f} m: {len(boxes)} of "
                                                f"{self.expected_boxes} faces stable, no {target or 'target'} face")
                else:
                    self._set(BoxState.STRAFE, s.now)
                    self._leg_start = s.position
            if self.state == BoxState.STRAFE:
                leg = self._distance(s.position, self._leg_start or s.position)
                total = self._distance(s.position, self._search_origin)
                if leg >= self.strafe_step_m or total >= self._strafe_budget_m or s.now - self._t_state > 30.0:
                    self._set(BoxState.OBSERVE, s.now)
                    return MissionAction(ActionKind.STILL, heading_deg=self._search_hdg,
                                         body={"stable_boxes": len(boxes), "expected_boxes": self.expected_boxes})
                return MissionAction(ActionKind.STRAFE, speed_mps=0.5, heading_deg=self._search_hdg,
                                     body={"side": self.strafe_side, "stable_boxes": len(boxes),
                                           "expected_boxes": self.expected_boxes})

        if self.state == BoxState.ASK_OCS:
            if not self._report_sent:
                self._report_sent = True
                return MissionAction(ActionKind.REPORT, body={
                    "boxes": [{"id": b.box_id, "colour": b.colour,
                               "left": (b.face.left.n, b.face.left.e),
                               "midpoint": (b.face.midpoint.n, b.face.midpoint.e),
                               "right": (b.face.right.n, b.face.right.e),
                               "colour_share": b.colour_share} for b in boxes]
                })
            try:
                why = self._select(s, boxes)
            except ValueError as exc:
                return self._abort(str(exc))
            if why:
                return self._enter_hold(why)
            self._set(BoxState.ALIGN, s.now)

        if self.state == BoxState.ALIGN:
            assert self.align_point is not None and self.approach_heading_deg is not None
            if s.now - self._t_state > self.align_timeout_s:
                return self._enter_hold(f"ALIGN timeout {self.align_timeout_s:.0f} s")
            dist = self._distance(s.position, self.align_point)
            herr = wrap180(self.approach_heading_deg - s.heading_deg)
            pos_ok = dist <= self.align_pos_tol_m
            # On arrival: yaw in place (a GUIDED target does not control the final heading); drifting off -> GUIDED again.
            self._yawing = (pos_ok or (self._yawing and dist <= 3.0 * self.align_pos_tol_m))
            if self._yawing and abs(herr) <= self.align_heading_tol_deg:
                self._yawing = False
                self._set(BoxState.APPROACH, s.now)
            elif self._yawing:
                return MissionAction(ActionKind.YAW, heading_deg=self.approach_heading_deg, body={"heading_err_deg": herr})
            else:
                return MissionAction(ActionKind.GOTO, target=self.align_point, speed_mps=0.5,
                                     heading_deg=self.approach_heading_deg)

        if self.state == BoxState.APPROACH:
            assert self.target is not None and self.stop_point is not None and self.approach_heading_deg is not None
            range_m = self._distance(s.position, self.target.face.midpoint)
            if not s.target_visible:
                if self._target_lost_since is None:
                    self._target_lost_since = s.now
                if s.now - self._target_lost_since >= 3.0:
                    return self._enter_hold("target lost for 3 seconds during approach")
            else:
                self._target_lost_since = None
            if range_m <= self.align_range_m:
                self._set(BoxState.VALIDATE, s.now)
            else:
                return MissionAction(ActionKind.APPROACH, target=self.stop_point,
                                     speed_mps=self._speed_for_range(range_m), heading_deg=self.approach_heading_deg)

        if self.state == BoxState.VALIDATE:
            colour = (s.target_colour or self.default_colour or "").lower()
            current = next((b for b in boxes if self.target and b.box_id == self.target.box_id), None)
            if current is None:
                if s.now - self._t_state < self.observe_s:      # look again for one observation window, still in mode 20
                    self._rescan_used = True
                    return MissionAction(ActionKind.STILL, heading_deg=self.approach_heading_deg,
                                         body={"reason": "target validation: looking again"})
                return self._abort("target validation unavailable after rescan")
            if current.colour != colour:
                return self._abort(f"target colour changed: expected {colour}, saw {current.colour}", back_away=True)
            self.target = current
            self._set(BoxState.STOP, s.now)

        if self.state == BoxState.STOP:
            assert self.stop_point is not None and self.approach_heading_deg is not None
            stop_error_m = self._distance(s.position, self.stop_point)
            if self._face_distance(s.position) <= self.stop_range_m + self.stop_tol_m:
                self.state = BoxState.DONE
                target = None if self.target is None else {
                    "box_id": self.target.box_id,
                    "colour": self.target.colour,
                    "midpoint": {
                        "n": self.target.face.midpoint.n,
                        "e": self.target.face.midpoint.e,
                    },
                }
                return MissionAction(ActionKind.DONE, body={
                    "target": target,
                    "stop_error_m": stop_error_m,
                    "stop_point": {"n": self.stop_point.n, "e": self.stop_point.e},
                    "face_distance_m": round(self._face_distance(s.position), 2),
                })
            range_m = self._distance(s.position, self.target.face.midpoint) if self.target else self.stop_range_m
            return MissionAction(ActionKind.APPROACH, target=self.stop_point,
                                 speed_mps=self._speed_for_range(range_m), heading_deg=self.approach_heading_deg)

        return self._hold("no action")
