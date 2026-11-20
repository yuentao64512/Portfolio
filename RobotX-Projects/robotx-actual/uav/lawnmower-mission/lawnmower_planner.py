"""
Lawnmower (boustrophedon) path planning inside a geofence of any shape.

Pure geometry - no MAVLink, no pymavlink import - so it can be tested and
reused on its own. lawnmower_mission.py turns the result into an ArduPilot
mission.

Works with whatever fence is on the FCU, wherever it is:
  - inclusion polygons of any number of corners, convex or not (L, U, ...)
  - exclusion polygons (keep-out zones inside the area)
  - inclusion / exclusion circles
The drone must stay inside EVERY inclusion zone and outside every exclusion
zone, --margin metres clear of all of them. (If ArduPilot only needs one of
several inclusion zones, this is just stricter - never less safe.)

How the path is built:
  1. Everything is projected to flat metres around the fence (the flat-earth
     error over a few hundred metres is millimetres).
  2. "Safe" = at least --margin from every fence edge, on the right side.
  3. Parallel sweep lines are laid across the area, --spacing apart. The safe
     stretches of each line become the legs - a line that crosses a keep-out
     zone or a notch gives more than one leg.
  4. Legs are flown back and forth, nearest next leg first, starting from the
     outermost leg nearest home. Each hop between legs, and from/to home, is
     a straight line if that is safe, otherwise it is routed around the
     corner on a grid and straightened.
  5. Every segment of the final route is checked again, every 10 cm, against
     the real fence. Any failure is an error - never a quietly unsafe path.
"""

import heapq
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple, Union

EARTH_RADIUS_M = 6378137.0
CHECK_STEP_M = 0.1  # resolution of the segment safety check
SCAN_STEP_M = 0.2  # resolution of the leg scan (edges then refined to 1 cm)
MIN_MARGIN_M = 0.5  # below this the sampled checks could miss a thin sliver
MAX_GRID_CELLS = 300  # per side, for routing hops around corners

LatLon = Tuple[float, float]
XY = Tuple[float, float]


class PlanError(ValueError):
    """The fence/home can't produce a safe lawnmower path."""


@dataclass
class FenceZones:
    """The fence as the FCU has it. Lat/lon in degrees, radii in metres."""

    inclusion_polygons: List[List[LatLon]] = field(default_factory=list)
    exclusion_polygons: List[List[LatLon]] = field(default_factory=list)
    inclusion_circles: List[Tuple[LatLon, float]] = field(default_factory=list)
    exclusion_circles: List[Tuple[LatLon, float]] = field(default_factory=list)

    def describe(self) -> str:
        parts = [f"{len(p)}-corner inclusion polygon" for p in self.inclusion_polygons]
        parts += [f"{r:g} m inclusion circle" for _, r in self.inclusion_circles]
        parts += [f"{len(p)}-corner keep-out polygon" for p in self.exclusion_polygons]
        parts += [f"{r:g} m keep-out circle" for _, r in self.exclusion_circles]
        return ", ".join(parts) if parts else "no zones"


@dataclass(frozen=True)
class Waypoint:
    lat: float
    lon: float
    kind: str  # "leg_start", "leg_end" or "transit" (a hop around a corner)
    leg: Optional[int] = None  # 1-based leg number for leg_start / leg_end


@dataclass
class LawnmowerPlan:
    waypoints: List[Waypoint]  # in flying order, not including home
    leg_count: int
    spacing_m: float  # actual spacing between sweep lines (<= requested)
    sweep_bearing_deg: float  # compass bearing the legs run along
    path_length_m: float  # home -> every waypoint -> home
    warnings: List[str] = field(default_factory=list)


# --- small vector helpers ---------------------------------------------------


def _sub(a: XY, b: XY) -> XY:
    return (a[0] - b[0], a[1] - b[1])


def _dot(a: XY, b: XY) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _cross(a: XY, b: XY) -> float:
    return a[0] * b[1] - a[1] * b[0]


def _norm(a: XY) -> float:
    return math.hypot(a[0], a[1])


def _lerp(a: XY, b: XY, t: float) -> XY:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def _signed_area(poly: Sequence[XY]) -> float:
    return 0.5 * sum(_cross(poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))


def _bearing_of(vec: XY) -> float:
    return math.degrees(math.atan2(vec[0], vec[1])) % 360.0


class _LocalFrame:
    """Flat x (east) / y (north) metres around an origin."""

    def __init__(self, lat0: float, lon0: float):
        self.lat0 = lat0
        self.lon0 = lon0
        self._cos = math.cos(math.radians(lat0))

    def to_xy(self, lat: float, lon: float) -> XY:
        return (
            math.radians(lon - self.lon0) * EARTH_RADIUS_M * self._cos,
            math.radians(lat - self.lat0) * EARTH_RADIUS_M,
        )

    def to_latlon(self, x: float, y: float) -> LatLon:
        return (
            self.lat0 + math.degrees(y / EARTH_RADIUS_M),
            self.lon0 + math.degrees(x / (EARTH_RADIUS_M * self._cos)),
        )


# --- fence geometry -----------------------------------------------------------


def _clean_polygon(points: Sequence[XY], label: str, min_edge_m: float = 0.05) -> List[XY]:
    """Drop the closing duplicate, repeated points and straight-through
    corners; refuse polygons with no area or crossing edges."""
    pts: List[XY] = []
    for p in points:
        if not pts or _norm(_sub(p, pts[-1])) > min_edge_m:
            pts.append(p)
    if len(pts) >= 2 and _norm(_sub(pts[0], pts[-1])) <= min_edge_m:
        pts.pop()

    changed = True
    while changed and len(pts) >= 3:
        changed = False
        for i in range(len(pts)):
            a, b, c = pts[i - 1], pts[i], pts[(i + 1) % len(pts)]
            ab, bc = _sub(b, a), _sub(c, b)
            if abs(_cross(ab, bc)) <= 1e-6 * _norm(ab) * _norm(bc) and _dot(ab, bc) > 0:
                pts.pop(i)
                changed = True
                break

    if len(pts) < 3:
        raise PlanError(f"{label} needs at least 3 distinct corners, got {len(pts)}")

    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue  # neighbouring edges share a corner
            if _segments_cross(pts[i], pts[(i + 1) % n], pts[j], pts[(j + 1) % n]):
                raise PlanError(f"{label} has edges that cross each other - check the order of its corners")
    if abs(_signed_area(pts)) < 1.0:
        raise PlanError(f"{label} has (almost) no area")
    return pts


def _segments_cross(a: XY, b: XY, c: XY, d: XY) -> bool:
    d1 = _cross(_sub(b, a), _sub(c, a))
    d2 = _cross(_sub(b, a), _sub(d, a))
    d3 = _cross(_sub(d, c), _sub(a, c))
    d4 = _cross(_sub(d, c), _sub(b, c))
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def _distance_to_segment(p: XY, a: XY, b: XY) -> float:
    ab = _sub(b, a)
    t = max(0.0, min(1.0, _dot(_sub(p, a), ab) / _dot(ab, ab)))
    return _norm(_sub(p, _lerp(a, b, t)))


def _inside_polygon(p: XY, poly: Sequence[XY]) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > p[1]) != (y2 > p[1]):
            if p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
                inside = not inside
    return inside


class _Region:
    """Signed clearance from every fence zone: positive = that many metres
    inside the allowed area, negative = outside it."""

    def __init__(self, inc_polys, exc_polys, inc_circles, exc_circles):
        self.inc_polys = inc_polys
        self.exc_polys = exc_polys
        self.inc_circles = inc_circles
        self.exc_circles = exc_circles

    @staticmethod
    def _edge_distance(p: XY, poly: Sequence[XY]) -> float:
        return min(_distance_to_segment(p, poly[i], poly[(i + 1) % len(poly)]) for i in range(len(poly)))

    def clearance(self, p: XY) -> float:
        c = math.inf
        for poly in self.inc_polys:
            d = self._edge_distance(p, poly)
            c = min(c, d if _inside_polygon(p, poly) else -d)
        for centre, r in self.inc_circles:
            c = min(c, r - _norm(_sub(p, centre)))
        for poly in self.exc_polys:
            d = self._edge_distance(p, poly)
            c = min(c, -d if _inside_polygon(p, poly) else d)
        for centre, r in self.exc_circles:
            c = min(c, _norm(_sub(p, centre)) - r)
        return c

    def segment_clear(self, a: XY, b: XY, threshold: float) -> bool:
        steps = max(1, math.ceil(_norm(_sub(b, a)) / CHECK_STEP_M))
        return all(self.clearance(_lerp(a, b, i / steps)) >= threshold - 1e-6 for i in range(steps + 1))


class _Router:
    """Finds a safe way between two points when the straight line isn't:
    A* over a grid of safe cells, then straightened back into as few
    segments as possible (each re-checked exactly)."""

    def __init__(self, region: _Region, bounds: Tuple[float, float, float, float], margin: float):
        self.region = region
        self.margin = margin
        self.x0, self.y0, x1, y1 = bounds
        self.res = max(0.5, max(x1 - self.x0, y1 - self.y0) / MAX_GRID_CELLS)
        self.nx = int((x1 - self.x0) / self.res) + 1
        self.ny = int((y1 - self.y0) / self.res) + 1
        self._safe: Dict[Tuple[int, int], bool] = {}

    def _centre(self, cell: Tuple[int, int]) -> XY:
        return (self.x0 + cell[0] * self.res, self.y0 + cell[1] * self.res)

    def _is_safe(self, cell: Tuple[int, int]) -> bool:
        if cell not in self._safe:
            ok = 0 <= cell[0] < self.nx and 0 <= cell[1] < self.ny
            # The extra res keeps the straight hop between neighbour cells safe too.
            self._safe[cell] = ok and self.region.clearance(self._centre(cell)) >= self.margin + self.res
        return self._safe[cell]

    def _entry_cell(self, p: XY, threshold: float) -> Optional[Tuple[int, int]]:
        ci, cj = round((p[0] - self.x0) / self.res), round((p[1] - self.y0) / self.res)
        max_ring = int((3 * self.margin + 5.0) / self.res) + 2
        for ring in range(max_ring + 1):
            cells = [
                (ci + di, cj + dj)
                for di in range(-ring, ring + 1)
                for dj in range(-ring, ring + 1)
                if max(abs(di), abs(dj)) == ring
            ]
            cells.sort(key=lambda c: _norm(_sub(self._centre(c), p)))
            for cell in cells:
                if self._is_safe(cell) and self.region.segment_clear(p, self._centre(cell), threshold):
                    return cell
        return None

    def _astar(self, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        def h(c):
            return math.hypot(c[0] - goal[0], c[1] - goal[1])

        open_heap = [(h(start), 0.0, start)]
        came_from = {start: None}
        best_g = {start: 0.0}
        while open_heap:
            _f, g, cell = heapq.heappop(open_heap)
            if cell == goal:
                path = []
                while cell is not None:
                    path.append(cell)
                    cell = came_from[cell]
                return path[::-1]
            if g > best_g.get(cell, math.inf):
                continue
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == dj == 0:
                        continue
                    nxt = (cell[0] + di, cell[1] + dj)
                    if not self._is_safe(nxt):
                        continue
                    ng = g + math.hypot(di, dj)
                    if ng < best_g.get(nxt, math.inf):
                        best_g[nxt] = ng
                        came_from[nxt] = cell
                        heapq.heappush(open_heap, (ng + h(nxt), ng, nxt))
        return None

    def route(self, a: XY, b: XY, threshold_a: float, threshold_b: float) -> Optional[List[XY]]:
        """Points to fly through between a and b (not including either), [] if
        the straight line is safe, None if there's no safe way at all."""
        if self.region.segment_clear(a, b, min(threshold_a, threshold_b)):
            return []
        start, goal = self._entry_cell(a, threshold_a), self._entry_cell(b, threshold_b)
        if start is None or goal is None:
            return None
        cells = self._astar(start, goal)
        if cells is None:
            return None
        pts = [a] + [self._centre(c) for c in cells] + [b]
        last = len(pts) - 1

        def threshold(i: int, j: int) -> float:
            t = self.margin
            if i == 0:
                t = min(t, threshold_a)
            if j == last:
                t = min(t, threshold_b)
            return t

        out = [a]
        i = 0
        while i < last:
            j = i + 1
            while j < last and self.region.segment_clear(pts[i], pts[j + 1], threshold(i, j + 1)):
                j += 1
            out.append(pts[j])
            i = j
        return out[1:-1]


# --- planning -------------------------------------------------------------


def _route_length(points: Sequence[XY]) -> float:
    return sum(_norm(_sub(points[i + 1], points[i])) for i in range(len(points) - 1))


def _scan_line(region: _Region, d: XY, n: XY, v: float, u_lo: float, u_hi: float, margin: float, min_leg: float):
    """Safe stretches (u_start, u_end) of the sweep line at offset v."""

    def safe(u: float) -> bool:
        return region.clearance((d[0] * u + n[0] * v, d[1] * u + n[1] * v)) >= margin

    def edge(u_out: float, u_in: float) -> float:
        for _ in range(12):  # bisection: SCAN_STEP_M / 2^12 < 1 mm
            mid = (u_out + u_in) / 2
            if safe(mid):
                u_in = mid
            else:
                u_out = mid
        return u_in

    steps = max(1, math.ceil((u_hi - u_lo) / SCAN_STEP_M))
    samples = [u_lo + (u_hi - u_lo) * i / steps for i in range(steps + 1)]
    flags = [safe(u) for u in samples]
    spans = []
    start = None
    for i, ok in enumerate(flags):
        if ok and start is None:
            start = samples[i] if i == 0 else edge(samples[i - 1], samples[i])
        if start is not None and (not ok or i == len(flags) - 1):
            end = samples[i] if ok else edge(samples[i], samples[i - 1])
            if end - start >= min_leg:
                spans.append((start, end))
            start = None
    return spans


def plan_lawnmower(
    fence: Union[FenceZones, Sequence[LatLon]],
    home: LatLon,
    spacing_m: float = 4.0,
    margin_m: float = 3.0,
    sweep_bearing_deg: Optional[float] = None,
    min_leg_m: float = 1.0,
) -> LawnmowerPlan:
    """Plan a lawnmower path inside the fence, starting and ending at home.

    fence: a FenceZones, or just a list of (lat, lon) corners for a single
    inclusion polygon. sweep_bearing_deg: compass bearing the legs run along
    (0 = north-south). Default: along the longest edge of the largest
    inclusion polygon (fewest turns).
    """
    if spacing_m <= 0:
        raise PlanError("spacing must be positive")
    if margin_m < MIN_MARGIN_M:
        raise PlanError(f"margin must be at least {MIN_MARGIN_M:g} m")
    zones = fence if isinstance(fence, FenceZones) else FenceZones(inclusion_polygons=[list(fence)])
    if not zones.inclusion_polygons and not zones.inclusion_circles:
        raise PlanError("the fence has no inclusion zone, so nothing bounds the area to cover")

    all_points = [p for poly in zones.inclusion_polygons + zones.exclusion_polygons for p in poly]
    all_points += [c for c, _ in zones.inclusion_circles + zones.exclusion_circles]
    frame = _LocalFrame(sum(p[0] for p in all_points) / len(all_points), sum(p[1] for p in all_points) / len(all_points))

    def polys(raw, label):
        return [_clean_polygon([frame.to_xy(*p) for p in poly], f"{label} {i + 1}") for i, poly in enumerate(raw)]

    def circles(raw, label):
        out = []
        for i, (centre, r) in enumerate(raw):
            if r <= 0:
                raise PlanError(f"{label} {i + 1} has radius {r:g} m")
            out.append((frame.to_xy(*centre), float(r)))
        return out

    inc_polys = polys(zones.inclusion_polygons, "inclusion polygon")
    exc_polys = polys(zones.exclusion_polygons, "keep-out polygon")
    inc_circles = circles(zones.inclusion_circles, "inclusion circle")
    exc_circles = circles(zones.exclusion_circles, "keep-out circle")
    region = _Region(inc_polys, exc_polys, inc_circles, exc_circles)

    home_xy = frame.to_xy(*home)
    home_clearance = region.clearance(home_xy)
    if home_clearance < 0:
        raise PlanError(
            f"home (the drone's current position) is {-home_clearance:.1f} m outside the allowed area "
            "(outside the fence or inside a keep-out zone) - move the drone before starting"
        )
    warnings = []
    if home_clearance < margin_m:
        warnings.append(
            f"home is only {home_clearance:.1f} m from the fence (margin is {margin_m:g} m) - "
            "take-off and landing happen there"
        )
    home_threshold = min(margin_m, home_clearance) - 0.01

    # Leg direction d, across-legs direction n.
    if sweep_bearing_deg is None and inc_polys:
        largest = max(inc_polys, key=lambda p: abs(_signed_area(p)))
        edges = [_sub(largest[(i + 1) % len(largest)], largest[i]) for i in range(len(largest))]
        longest = max(edges, key=_norm)
        d = (longest[0] / _norm(longest), longest[1] / _norm(longest))
    else:
        b = math.radians(sweep_bearing_deg or 0.0)
        d = (math.sin(b), math.cos(b))
    n = (-d[1], d[0])

    # Extent of the area every inclusion zone allows, along d/n and in x/y.
    def extents(project):
        lo, hi = -math.inf, math.inf
        for poly in inc_polys:
            vals = [project(p) for p in poly]
            lo, hi = max(lo, min(vals)), min(hi, max(vals))
        for centre, r in inc_circles:
            lo, hi = max(lo, project(centre) - r), min(hi, project(centre) + r)
        return lo, hi

    u_lo, u_hi = extents(lambda p: _dot(p, d))
    v_lo, v_hi = extents(lambda p: _dot(p, n))
    if (v_hi - v_lo) < 2 * margin_m or u_hi - u_lo < 2 * margin_m:
        raise PlanError(f"the fence is too small for a {margin_m:g} m safety margin - lower the margin")

    def has_leg(v: float) -> bool:
        return bool(_scan_line(region, d, n, v, u_lo, u_hi, margin_m, min_leg_m))

    def first_line(v_from: float, v_to: float) -> Optional[float]:
        """The first sweep offset from v_from towards v_to that has a leg -
        so the outermost legs hug the safe area, even past a pointed corner."""
        step = (v_to - v_from) / max(1, math.ceil(abs(v_to - v_from) / (spacing_m / 4)))
        prev, v = v_from, v_from
        while (v - v_to) * (v_from - v_to) >= 0:
            if has_leg(v):
                if v == v_from:
                    return v
                bad, good = prev, v
                for _ in range(8):  # bisection, to a few cm
                    mid = (bad + good) / 2
                    bad, good = (bad, mid) if has_leg(mid) else (mid, good)
                return good
            prev, v = v, v + step
        return None

    v_first = first_line(v_lo + margin_m, v_hi - margin_m)
    if v_first is None:
        raise PlanError("no leg fits inside the fence with this margin/spacing")
    v_last = first_line(v_hi - margin_m, v_first)
    width = v_last - v_first
    gaps = max(1, math.ceil(width / spacing_m - 1e-9)) if width > 1e-6 else 0
    used_spacing = width / gaps if gaps else 0.0
    positions = [v_first + k * used_spacing for k in range(gaps + 1)]

    legs = []  # (line index, start xy, end xy)
    for line, v in enumerate(positions):
        for s, e in _scan_line(region, d, n, v, u_lo, u_hi, margin_m, min_leg_m):
            legs.append((line, (d[0] * s + n[0] * v, d[1] * s + n[1] * v), (d[0] * e + n[0] * v, d[1] * e + n[1] * v)))
    if not legs:
        raise PlanError("no leg fits inside the fence with this margin/spacing")

    x_lo, x_hi = extents(lambda p: p[0])
    y_lo, y_hi = extents(lambda p: p[1])
    router = _Router(region, (x_lo, y_lo, x_hi, y_hi), margin_m)

    # First leg: an end of an outermost line, whichever is nearest home - so
    # the sweep runs from one side to the other instead of starting mid-way.
    edge_lines = {legs[0][0], legs[-1][0]}
    first = min(
        ((i, end) for i, leg in enumerate(legs) if leg[0] in edge_lines for end in (1, 2)),
        key=lambda c: _norm(_sub(legs[c[0]][c[1]], home_xy)),
    )

    route: List[Tuple[XY, str, Optional[int]]] = []
    remaining = set(range(len(legs)))
    current, current_threshold = home_xy, home_threshold
    choice: Optional[Tuple[int, int]] = first
    leg_number = 0
    def pick(candidates):
        best = None  # (cost, leg, end, hop)
        for i, end in candidates:
            target = legs[i][end]
            if best is not None and _norm(_sub(target, current)) >= best[0]:
                break  # sorted by straight distance - nothing further can be cheaper
            hop = router.route(current, target, current_threshold, margin_m)
            if hop is None:
                continue
            cost = _route_length([current, *hop, target])
            if best is None or cost < best[0]:
                best = (cost, i, end, hop)
        return best

    while remaining:
        best = pick([choice]) if choice is not None else None
        choice = None
        if best is None:
            nearest = sorted(
                ((i, end) for i in remaining for end in (1, 2)), key=lambda c: _norm(_sub(legs[c[0]][c[1]], current))
            )
            best = pick(nearest[:8]) or pick(nearest[8:])
        if best is None:
            if leg_number == 0:
                raise PlanError("no leg can be reached from home without breaking the safety margin")
            warnings.append(
                f"{len(remaining)} leg(s) skipped: they're past a part of the fence narrower than "
                f"2 x {margin_m:g} m, so they can't be reached safely"
            )
            break
        _cost, i, end, hop = best
        leg_number += 1
        route += [(p, "transit", None) for p in hop]
        route.append((legs[i][end], "leg_start", leg_number))
        route.append((legs[i][3 - end], "leg_end", leg_number))
        current, current_threshold = legs[i][3 - end], margin_m
        remaining.discard(i)

    back = router.route(current, home_xy, margin_m, home_threshold)
    if back is None:
        raise PlanError("no safe way back to home from the last leg")
    route += [(p, "transit", None) for p in back]

    # Independent final check of every segment against the real fence.
    full = [home_xy] + [p for p, _, _ in route] + [home_xy]
    for k in range(len(full) - 1):
        threshold = home_threshold if k == 0 or k == len(full) - 2 else margin_m
        if not region.segment_clear(full[k], full[k + 1], threshold):
            raise PlanError(f"internal check failed: route segment {k + 1} comes closer than the margin to the fence")

    return LawnmowerPlan(
        waypoints=[Waypoint(*frame.to_latlon(*p), kind=kind, leg=leg) for p, kind, leg in route],
        leg_count=leg_number,
        spacing_m=used_spacing,
        sweep_bearing_deg=_bearing_of(d),
        path_length_m=_route_length(full),
        warnings=warnings,
    )
