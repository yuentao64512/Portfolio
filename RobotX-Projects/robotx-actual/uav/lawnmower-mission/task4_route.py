"""
Route planning for task4_mission.py (RobotX Task 4), using lawnmower_planner's
fence geometry and router. Pure geometry - no MAVLink.

A RouteMap is the fence plus the keep-out zones active right now, each one a
circle the drone must stay --margin metres clear of. It answers: is this path
still safe, how do I get to an assistance position, and how do I finish the
survey - flying only the parts of the unflown legs that are safe now, and
keeping the blocked parts for after an All Clear.
"""

from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence, Tuple

from lawnmower_planner import (
    FenceZones,
    PlanError,
    _clean_polygon,
    _lerp,
    _LocalFrame,
    _norm,
    _Region,
    _Router,
    _sub,
)

LatLon = Tuple[float, float]

CLIP_STEP_M = 0.2  # resolution when cutting a leg at a keep-out zone
MIN_LEG_M = 1.0  # a safe part of a leg shorter than this isn't worth flying
ASSIST_MIN_CLEARANCE_M = 1.0  # an assistance position closer than this to the fence or a keep-out is refused
ESCAPE_STEP_M = 1.0
ESCAPE_MAX_M = 200.0


@dataclass(frozen=True)
class Piece:
    """A stretch of a survey leg still to fly, a -> b."""

    a: LatLon
    b: LatLon


@dataclass(frozen=True)
class RoutePoint:
    position: LatLon
    kind: str  # "transit", "leg_start", "leg_end" or "target"
    piece: Optional[Piece] = None  # for leg_end: the stretch that is flown once this is passed


@dataclass
class SurveyPath:
    points: List[RoutePoint]
    pieces: List[Piece]  # everything still to fly, including what is blocked now
    goes_home: bool  # False: home can't be reached now - hold at the last point
    warnings: List[str] = field(default_factory=list)


class RouteMap:
    def __init__(self, zones: FenceZones, margin_m: float, keep_outs: Sequence[Tuple[LatLon, float]] = ()):
        self.margin = margin_m
        points = [p for poly in zones.inclusion_polygons + zones.exclusion_polygons for p in poly]
        points += [c for c, _ in zones.inclusion_circles + zones.exclusion_circles]
        self.frame = _LocalFrame(sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))

        def polys(raw, label):
            return [_clean_polygon([self.frame.to_xy(*p) for p in poly], f"{label} {i + 1}") for i, poly in enumerate(raw)]

        def circles(raw):
            return [(self.frame.to_xy(*c), float(r)) for c, r in raw]

        inc_polys = polys(zones.inclusion_polygons, "inclusion polygon")
        exc_polys = polys(zones.exclusion_polygons, "keep-out polygon")
        inc_circles, exc_circles = circles(zones.inclusion_circles), circles(zones.exclusion_circles)
        self.fence = _Region(inc_polys, exc_polys, inc_circles, exc_circles)
        self.region = _Region(inc_polys, exc_polys, inc_circles, exc_circles + circles(keep_outs))

        def extent(axis):
            lo, hi = -math.inf, math.inf
            for poly in inc_polys:
                vals = [p[axis] for p in poly]
                lo, hi = max(lo, min(vals)), min(hi, max(vals))
            for centre, r in inc_circles:
                lo, hi = max(lo, centre[axis] - r), min(hi, centre[axis] + r)
            return lo, hi

        (x_lo, x_hi), (y_lo, y_hi) = extent(0), extent(1)
        self.router = _Router(self.region, (x_lo, y_lo, x_hi, y_hi), margin_m)

    def clearance(self, p: LatLon) -> float:
        """Metres inside the allowed area - negative = outside the fence or in a keep-out."""
        return self.region.clearance(self.frame.to_xy(*p))

    def _threshold(self, p: LatLon) -> float:
        return min(self.margin, self.clearance(p)) - 0.01

    def route(self, a: LatLon, b: LatLon) -> Optional[List[LatLon]]:
        """Points to fly through from a to b (neither included); None = no safe way."""
        via = self.router.route(self.frame.to_xy(*a), self.frame.to_xy(*b), self._threshold(a), self._threshold(b))
        return None if via is None else [self.frame.to_latlon(*p) for p in via]

    def leave_keep_out(self, start: LatLon) -> Optional[List[LatLon]]:
        """[] if start is outside every keep-out, else the nearest safe point to
        fly straight to (it stays inside the fence). None = none found."""
        if self.clearance(start) >= 0:
            return []
        xy = self.frame.to_xy(*start)
        floor = min(self.margin, self.fence.clearance(xy)) - 0.01
        distance = ESCAPE_STEP_M
        while distance <= ESCAPE_MAX_M:
            for i in range(24):
                bearing = math.radians(i * 15.0)
                p = (xy[0] + distance * math.sin(bearing), xy[1] + distance * math.cos(bearing))
                if self.region.clearance(p) >= self.margin and self.fence.segment_clear(xy, p, floor):
                    return [self.frame.to_latlon(*p)]
            distance += ESCAPE_STEP_M
        return None

    def path_ok(self, start: LatLon, points: Sequence[LatLon]) -> bool:
        """Is flying start -> points still clear of the fence and keep-outs?
        The first and last points may be as close as they already are."""
        if self.clearance(start) < 0 or (points and self.clearance(points[-1]) < 0):
            return False
        full = [start, *points]
        for k in range(len(full) - 1):
            threshold = self.margin - 0.01
            if k == 0:
                threshold = min(threshold, self._threshold(start))
            if k == len(full) - 2:
                threshold = min(threshold, self._threshold(full[-1]))
            if not self.region.segment_clear(self.frame.to_xy(*full[k]), self.frame.to_xy(*full[k + 1]), threshold):
                return False
        return True

    def assist_path(self, start: LatLon, target: LatLon) -> Optional[List[RoutePoint]]:
        """How to reach an assistance position. None = refuse: it is too close
        to the fence or a keep-out, or there is no safe way."""
        if self.clearance(target) < ASSIST_MIN_CLEARANCE_M:
            return None
        escape = self.leave_keep_out(start)
        if escape is None:
            return None
        via = self.route(escape[-1] if escape else start, target)
        if via is None:
            return None
        points = [RoutePoint(p, "transit") for p in escape + via] + [RoutePoint(target, "target")]
        self._check(start, escape, points)
        return points

    def _split(self, piece: Piece) -> List[Tuple[Piece, bool]]:
        """A leg stretch cut into (part, safe to fly now), in flying order."""
        a, b = self.frame.to_xy(*piece.a), self.frame.to_xy(*piece.b)
        steps = max(1, math.ceil(_norm(_sub(b, a)) / CLIP_STEP_M))
        flags = [self.region.clearance(_lerp(a, b, i / steps)) >= self.margin for i in range(steps + 1)]
        if all(flags):
            return [(piece, True)]

        def part(i, j, safe):
            p, q = _lerp(a, b, i / steps), _lerp(a, b, j / steps)
            return Piece(self.frame.to_latlon(*p), self.frame.to_latlon(*q)), safe and _norm(_sub(q, p)) >= MIN_LEG_M

        parts, start, state = [], 0, flags[0]
        for i in range(1, steps + 1):
            if flags[i] != state:
                end = i - 1 if state else i  # a safe part ends on its last safe sample
                if end > start:
                    parts.append(part(start, end, state))
                start, state = end, flags[i]
        if start < steps:
            parts.append(part(start, steps, state))
        return parts

    def survey_path(self, start: LatLon, pieces: Sequence[Piece], home: LatLon) -> SurveyPath:
        """Finish the survey from start, then go home (if home can be reached)."""
        escape = self.leave_keep_out(start)
        if escape is None:
            raise PlanError("the drone is inside a keep-out zone with no safe way out of it")
        points = [RoutePoint(p, "transit") for p in escape]
        current = escape[-1] if escape else start
        remaining, warnings, unreachable, blocked = [], [], 0, 0
        for piece in pieces:
            for stretch, safe in self._split(piece):
                remaining.append(stretch)
                via = self.route(current, stretch.a) if safe else None
                if via is None:
                    blocked += not safe
                    unreachable += safe
                    continue
                points += [RoutePoint(p, "transit") for p in via]
                points += [RoutePoint(stretch.a, "leg_start"), RoutePoint(stretch.b, "leg_end", stretch)]
                current = stretch.b
        if blocked:
            warnings.append(f"{blocked} stretch(es) of the survey skipped for now: inside a keep-out zone")
        if unreachable:
            warnings.append(f"{unreachable} stretch(es) of the survey skipped: no safe way to reach them")
        via_home = self.route(current, home) if self.clearance(home) >= 0 else None
        goes_home = via_home is not None
        if goes_home:
            points += [RoutePoint(p, "transit") for p in via_home]
        self._check(start, escape, points + ([RoutePoint(home, "target")] if goes_home else []))
        return SurveyPath(points, remaining, goes_home, warnings)

    def _check(self, start: LatLon, escape: List[LatLon], points: Sequence[RoutePoint]) -> None:
        """Independent final check of the whole path, like the planner's."""
        positions = [p.position for p in points]
        begin = positions[len(escape) - 1] if escape else start
        rest = positions[len(escape):]
        if rest and not self.path_ok(begin, rest):
            raise PlanError("internal check failed: the new route comes closer than the margin to the fence or a keep-out")
