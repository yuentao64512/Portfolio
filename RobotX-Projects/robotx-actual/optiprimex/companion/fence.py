"""fence: the USV's FCU geofence - the polygon rules and the bookkeeping (v0.1.53, Dmitrii 2026-11-06). Pure, no I/O.

"USV only, with a button to apply to FCU, with acknowledgment from Mavlink" - the operator applies the received course (RxCourse,
handbook 3.4.9: ordered corners, first == last) or the OCS profile's geofence as ONE inclusion polygon on the FCU through the MAVLink
mission protocol (companion/link: fence_upload / fence_download / fence_enable), reads it back and compares within 1e-6 deg (the UAV
team's field lesson, ROADMAP §8), and toggles it with MAV_CMD_DO_FENCE_ENABLE. The fence is OFF by default: nothing uploads or
enables by itself. The FENCE_* parameters (FENCE_TYPE / FENCE_ACTION / ...) are apply_params.py's (presets), never set here.

The inside-the-course rule is the RoboCommand one ported from robotx-uav-robocommand/geofence-tools/run_declaration.py
(`_inside` / `_outside_by`, every point and edge sampled at 0.5 m, 1 m tolerance).
"""
from __future__ import annotations

import math
import threading
from typing import List, Optional, Sequence, Tuple

LatLon = Tuple[float, float]

TOL_DEG = 1e-6              # read back identical: |sent - got| < 1e-6 deg per coordinate (~10 cm; geofence_udp_listener._same_fence)
CLOSE_DEG = 1e-7            # a last point equal to the first = the closing marker, dropped (ArduPilot closes the polygon itself)
INSIDE_TOL_M = 1.0          # RoboCommand's tolerance: how far outside the course a point / an edge may be
EDGE_STEP_M = 0.5
MAX_POINTS = 70             # well under ArduPilot's fence storage on a Pixhawk (FENCE_TOTAL / the fence's own storage)
SOURCES = ("course", "geofence")
MISSION_RESULT = {0: "MAV_MISSION_ACCEPTED", 1: "MAV_MISSION_ERROR", 2: "MAV_MISSION_UNSUPPORTED_FRAME", 3: "MAV_MISSION_UNSUPPORTED",
                  4: "MAV_MISSION_NO_SPACE", 5: "MAV_MISSION_INVALID", 6: "MAV_MISSION_INVALID_PARAM1", 7: "MAV_MISSION_INVALID_PARAM2",
                  8: "MAV_MISSION_INVALID_PARAM3", 9: "MAV_MISSION_INVALID_PARAM4", 10: "MAV_MISSION_INVALID_PARAM5_X",
                  11: "MAV_MISSION_INVALID_PARAM6_Y", 12: "MAV_MISSION_INVALID_PARAM7", 13: "MAV_MISSION_INVALID_SEQUENCE",
                  14: "MAV_MISSION_DENIED", 15: "MAV_MISSION_OPERATION_CANCELLED"}


def ring(points) -> List[LatLon]:
    """[[lat, lon], ...] -> an open ring of floats (a closing point equal to the first is dropped). Raises ValueError on junk."""
    try:
        pts = [(float(p[0]), float(p[1])) for p in (points or [])]
    except (TypeError, ValueError, IndexError):
        raise ValueError("points: a list of [lat, lon]")
    if len(pts) >= 2 and abs(pts[0][0] - pts[-1][0]) < CLOSE_DEG and abs(pts[0][1] - pts[-1][1]) < CLOSE_DEG:
        pts = pts[:-1]
    return pts


def problem(points) -> Optional[str]:
    """Why this polygon cannot be a fence (None = fine): >= 3 distinct points, lat / lon in range, not too many."""
    try:
        pts = ring(points)
    except ValueError as exc:
        return str(exc)
    if any(not (math.isfinite(a) and math.isfinite(b) and -90 <= a <= 90 and -180 <= b <= 180) for a, b in pts):
        return "a point is out of range (lat -90..90, lon -180..180)"
    distinct = {(round(a, 7), round(b, 7)) for a, b in pts}
    if len(distinct) < 3:
        return f"a fence needs at least 3 distinct points (got {len(distinct)})"
    if len(pts) > MAX_POINTS:
        return f"{len(pts)} points: at most {MAX_POINTS}"
    return None


def same(a, b, tol: float = TOL_DEG) -> Tuple[bool, str]:
    """(identical, why not) - the read-back compare: the same count, each coordinate within tol."""
    a, b = ring(a), ring(b)
    if len(a) != len(b):
        return False, f"sent {len(a)} points, the FCU holds {len(b)}"
    for i, (p, q) in enumerate(zip(a, b)):
        if abs(p[0] - q[0]) >= tol or abs(p[1] - q[1]) >= tol:
            return False, f"point {i + 1} sent {p[0]:.7f},{p[1]:.7f} got {q[0]:.7f},{q[1]:.7f}"
    return True, ""


# ---- inside the course (run_declaration.py's rule) -------------------------------------------------------------------------
def _xy(p: LatLon, origin: LatLon) -> Tuple[float, float]:
    return ((p[1] - origin[1]) * 111320.0 * math.cos(math.radians(origin[0])), (p[0] - origin[0]) * 111320.0)


def _closed(pts: List) -> List:
    return pts if pts and pts[0] == pts[-1] else pts + pts[:1]


def _inside(p, closed_ring) -> bool:
    inside = False
    for (x1, y1), (x2, y2) in zip(closed_ring, closed_ring[1:]):
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _outside_by(p, closed_ring) -> float:
    if _inside(p, closed_ring):
        return 0.0
    best = math.inf
    for (ax, ay), (bx, by) in zip(closed_ring, closed_ring[1:]):
        dx, dy = bx - ax, by - ay
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def contains(polygon, lat: float, lon: float) -> bool:
    """The point inside the polygon (lat / lon, flat-earth; a fence is a few hundred metres)."""
    pts = ring(polygon)
    if len(pts) < 3:
        return False
    o = pts[0]
    return _inside(_xy((lat, lon), o), _closed([_xy(p, o) for p in pts]))


def outside_course(fence_pts, course_pts, tol_m: float = INSIDE_TOL_M) -> Optional[str]:
    """None when the fence lies inside the course (points and edges sampled every 0.5 m, within tol_m), else where it leaves it."""
    f, c = ring(fence_pts), ring(course_pts)
    if len(c) < 3:
        return None
    o = c[0]
    area = _closed([_xy(p, o) for p in c])
    pts = _closed([_xy(p, o) for p in f])
    worst, where = 0.0, ""
    n = len(f)
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        steps = max(1, math.ceil(math.dist(a, b) / EDGE_STEP_M))
        for k in range(steps + 1):
            q = (a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
            out = _outside_by(q, area)
            if out > worst:
                worst = out
                where = f"point {i + 1}" if k == 0 else (f"point {(i + 1) % n + 1}" if k == steps else f"edge {i + 1}-{(i + 1) % n + 1}")
    if worst > tol_m:
        return f"the geofence leaves the course boundary by {worst:.1f} m at {where}"
    return None


# ---- the vehicle's bookkeeping (status/usv `fence`) -----------------------------------------------------------------------
SYS_FRESH_S = 3.5           # SYS_STATUS / FENCE_STATUS are 1 Hz: they count while younger than this


class FenceBook:
    """What this mission manager applied and what the FCU answered; view() is status/usv `fence` - never fabricated (null = unknown).
    One fence operation at a time (busy)."""

    def __init__(self):
        self.lock = threading.Lock()
        self.points_n: Optional[int] = None       # what the FCU holds, from our upload's read-back (or the read before an enable)
        self.source: Optional[str] = None         # course | geofence - what was applied
        self.matches: Optional[bool] = None       # the read-back identical to what was sent
        self.applied_t: Optional[str] = None      # UTC of the accepted upload
        self.sent: Optional[List[LatLon]] = None  # the polygon we uploaded (the compare base for a later read-back)
        self.enable_ack: Optional[bool] = None    # the last DO_FENCE_ENABLE the FCU ACCEPTED: on (True) / off (False)
        self.last: Optional[str] = None           # the last operation's result text
        self.busy: Optional[str] = None           # the running operation ("apply" / "enable on" / ...), None = idle
        self.result: Optional[Tuple[bool, str]] = None   # the running / last operation's (ok, text), None while it runs

    def view(self, fcu) -> dict:
        """status/usv `fence`: enabled from the FCU (SYS_STATUS MAV_SYS_STATUS_GEOFENCE while fresh and present, else the last
        DO_FENCE_ENABLE ack, else null), breach from a fresh FENCE_STATUS (else null)."""
        ages = getattr(fcu, "ages", {}) or {}
        enabled, src = None, None
        if getattr(fcu, "fence_sys_present", None) and ages.get("SYS_STATUS", math.inf) <= SYS_FRESH_S:
            enabled, src = bool(fcu.fence_sys_enabled), "sys_status"
        elif self.enable_ack is not None:
            enabled, src = self.enable_ack, "ack"
        breach = None
        if getattr(fcu, "fence_breach_status", None) is not None and ages.get("FENCE_STATUS", math.inf) <= SYS_FRESH_S:
            breach = {"status": bool(fcu.fence_breach_status), "count": fcu.fence_breach_count, "type": fcu.fence_breach_type}
        return {"enabled": enabled, "enabled_from": src, "points_n": self.points_n, "source": self.source, "matches": self.matches,
                "breach": breach, "applied_t": self.applied_t, "busy": self.busy, "last": self.last}
