"""
The UAV geofence from RoboCommand's course: one polygon covering the course
boundary plus the UAV transit area (RxCourse.corners + uav_transit_area) -
everywhere RoboCommand lets the UAV be.

It has to be ONE polygon: ArduPilot requires the vehicle to be inside every
inclusion polygon at once, so the course and the transit area uploaded as
two polygons would only allow the edge they share. The two are merged along
that shared edge. If they don't share exactly one edge (or the result isn't
a clean polygon) this refuses rather than guessing - publish a fence by hand
then.

Pure geometry, standard library only: used by the listener (Orin) and by
run_declaration.py (laptop).
"""

import math
from typing import List, Sequence, Tuple

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


def _xy(p: LatLon, origin: LatLon) -> Tuple[float, float]:
    return ((p[1] - origin[1]) * 111320.0 * math.cos(math.radians(origin[0])), (p[0] - origin[0]) * 111320.0)


def _area(ring: Sequence[LatLon], origin: LatLon) -> float:
    pts = [_xy(p, origin) for p in ring]
    return 0.5 * sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts)))


def _segments_cross(a, b, c, d) -> bool:
    def cross(o, p, q):
        return (p[0] - o[0]) * (q[1] - o[1]) - (p[1] - o[1]) * (q[0] - o[0])
    return cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0


def _is_simple(ring: Sequence[LatLon], origin: LatLon) -> bool:
    pts = [_xy(p, origin) for p in ring]
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
        ax, ay = _xy(a, origin)
        bx, by = _xy(b, origin)
        length2 = (bx - ax) ** 2 + (by - ay) ** 2
        on_edge = []
        for p in other:
            if _same(p, a) or _same(p, b) or length2 == 0:
                continue
            px, py = _xy(p, origin)
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
