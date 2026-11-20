"""geofence: RoboCommand's acceptance rule for the UAV geofence and the course fence (v0.1.64, D8 of docs/OVERNIGHT_2026-11-07_UAV.md).

PORTED VERBATIM from the UAV team's field-tested v2 archive (`uav/geofence-tools/`, MERGE_GUIDE §4 invariant 4 - "the goal is
integration, not improvement"):
- `_xy`, `_inside`, `_outside_by`, `_closed`, `geofence_problems` and the constants from `run_declaration.py`;
- `course_fence()` and its helpers from `course_fence.py` (its `_xy` renamed `_cf_xy` here, the only edit: the two files had a
  function of that name each, same body).
tests/test_v0164.py pins the function bodies against the snapshot's source text and his test cases. Pure, standard library."""
from __future__ import annotations

import math
from typing import List, Sequence, Tuple

GEOFENCE_TOLERANCE_M = 1.0  # RoboCommand's: how far outside the course/transit area a point may be
EDGE_STEP_M = 0.5
MAX_CEILING_HAE_M = 50.0  # robocommand_bridge.py: the highest geofence ceiling RoboCommand accepts


def _xy(p, origin):
    """Flat metres east/north of origin."""
    return ((p[1] - origin[1]) * 111320.0 * math.cos(math.radians(origin[0])), (p[0] - origin[0]) * 111320.0)


def _inside(p, ring) -> bool:
    inside = False
    for (x1, y1), (x2, y2) in zip(ring, ring[1:]):
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _outside_by(p, ring) -> float:
    """0 inside the ring, else metres to its nearest edge."""
    if _inside(p, ring):
        return 0.0
    best = math.inf
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        dx, dy = bx - ax, by - ay
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def _closed(points):
    return points if points and points[0] == points[-1] else points + points[:1]


def geofence_problems(fence, corners, transit, ceiling, max_ceiling) -> list:
    """Why RoboCommand would refuse this UAV geofence (empty: it's fine)."""
    problems = []
    if len(fence) < 4:
        problems.append(f"the geofence has {len(fence)} point(s), needs at least 4 (first point repeated at the end)")
    elif fence[0] != fence[-1]:
        problems.append("the geofence isn't closed: its first and last points must be the same")
    elif corners:
        origin = corners[0]
        areas = [[_xy(p, origin) for p in _closed(ring)] for ring in (corners, transit) if len(ring) >= 3]
        pts = [_xy(p, origin) for p in fence]
        worst, where = 0.0, None
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            steps = max(1, math.ceil(math.dist(a, b) / EDGE_STEP_M))
            for k in range(steps + 1):
                q = (a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
                out = min(_outside_by(q, area) for area in areas)
                if out > worst:
                    worst = out
                    where = f"point {i + 1}" if k == 0 else f"point {i + 2}" if k == steps else f"edge {i + 1}-{i + 2}"
        if worst > GEOFENCE_TOLERANCE_M:
            problems.append(f"the geofence leaves the course and UAV transit area by {worst:.1f} m at {where}")
    if not 0 < ceiling <= max_ceiling:
        problems.append(f"the ceiling is {ceiling:.1f} m HAE: it must be above 0 and at most {max_ceiling:g} m HAE")
    return problems


# ---- course_fence.py ----------------------------------------------------------------------------------------------------

LatLon = Tuple[float, float]
SAME_POINT_DEG = 1e-7  # ~1 cm: corners RoboCommand repeats between the two areas


class CourseFenceError(ValueError):
    pass


def _open_ring(points: Sequence[LatLon]) -> List[LatLon]:
    pts = [(float(a), float(b)) for a, b in points]
    if len(pts) >= 2 and _same(pts[0], pts[-1]):
        pts.pop()
    return pts


def _same(a: LatLon, b: LatLon) -> bool:
    return abs(a[0] - b[0]) < SAME_POINT_DEG and abs(a[1] - b[1]) < SAME_POINT_DEG


def _cf_xy(p: LatLon, origin: LatLon) -> Tuple[float, float]:
    return ((p[1] - origin[1]) * 111320.0 * math.cos(math.radians(origin[0])), (p[0] - origin[0]) * 111320.0)


def _area(ring: Sequence[LatLon], origin: LatLon) -> float:
    pts = [_cf_xy(p, origin) for p in ring]
    return 0.5 * sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))


def _segments_cross(a, b, c, d) -> bool:
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    return cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0


def _is_simple(ring: Sequence[LatLon], origin: LatLon) -> bool:
    pts = [_cf_xy(p, origin) for p in ring]
    n = len(pts)
    for i in range(n):
        for j in range(i + 1, n):
            if j == i + 1 or (i == 0 and j == n - 1):
                continue
            if _segments_cross(pts[i], pts[(i + 1) % n], pts[j], pts[(j + 1) % n]):
                return False
    return True


def _with_touching_points(ring: List[LatLon], other: List[LatLon], origin: LatLon) -> List[LatLon]:
    """ring with every corner of `other` that lies on one of its edges added
    as a vertex there - so areas sharing only PART of an edge (one corner
    sitting mid-edge on the other) still share an exact edge."""
    out = []
    for idx, a in enumerate(ring):
        b = ring[(idx + 1) % len(ring)]
        out.append(a)
        ax, ay = _cf_xy(a, origin)
        bx, by = _cf_xy(b, origin)
        length2 = (bx - ax) ** 2 + (by - ay) ** 2
        on_edge = []
        for p in other:
            if _same(p, a) or _same(p, b) or length2 == 0:
                continue
            px, py = _cf_xy(p, origin)
            t = ((px - ax) * (bx - ax) + (py - ay) * (by - ay)) / length2
            if 0 < t < 1 and math.hypot(px - (ax + t * (bx - ax)), py - (ay + t * (by - ay))) < 0.01:
                on_edge.append((t, p))
        out += [p for _, p in sorted(on_edge)]
    return out


def course_fence(corners: Sequence[LatLon], transit: Sequence[LatLon]) -> List[LatLon]:
    """The fence polygon, closed (first point repeated at the end)."""
    course = _open_ring(corners)
    if len(course) < 3:
        raise CourseFenceError(f"the course has {len(course)} corners - need at least 3")
    area = _open_ring(transit)
    origin = course[0]
    if _area(course, origin) < 0:
        course.reverse()
    if len(area) < 3:
        if not _is_simple(course, origin):
            raise CourseFenceError("the course boundary's edges cross each other")
        return course + course[:1]  # no transit area: the course alone
    if _area(area, origin) < 0:
        area.reverse()
    original_course, original_area = course, area
    course = _with_touching_points(original_course, original_area, origin)
    area = _with_touching_points(original_area, original_course, origin)

    # Both counter-clockwise now, so a shared edge runs p->q in the course and
    # q->p in the transit area.
    shared = [(i, k) for i in range(len(course)) for k in range(len(area))
              if _same(course[i], area[(k + 1) % len(area)]) and _same(course[(i + 1) % len(course)], area[k])]
    if len(shared) != 1:
        raise CourseFenceError(
            f"the course and the UAV transit area share {len(shared)} edges (need exactly 1) - "
            "can't make them one fence automatically; publish a fence by hand")
    i, k = shared[0]
    m = len(area)
    merged = course[:i + 1] + [area[(k + 2 + t) % m] for t in range(m - 2)] + course[i + 1:]

    expected = _area(course, origin) + _area(area, origin)
    if not _is_simple(merged, origin) or abs(_area(merged, origin) - expected) > 0.01 * expected + 0.5:
        raise CourseFenceError("the course and the UAV transit area overlap - can't merge them into one fence; "
                               "publish a fence by hand")
    return merged + merged[:1]
