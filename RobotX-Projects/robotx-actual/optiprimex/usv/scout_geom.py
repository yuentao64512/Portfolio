"""scout_geom: the scout mission's pure geometry (v0.1.79, WP35; docs/SCOUT_BRIEF_2026-11-11.md §2-§3).

The course polygon in the local frame (x_n, y_e), its inset by margin_m, point-in-polygon, the boustrophedon lanes along the long axis
(the FRONTIER points), the coverage grid (1 m cells; seen inside the camera's cone within coverage_r_m, or within 2 m of the hull), the
ADVANCE ray clipped to the inset polygon, and the visit point on the stand-off circle. No state of the task, no I/O.
"""
from __future__ import annotations

import math
from typing import Callable, List, Optional, Sequence, Tuple

P = Tuple[float, float]


def area(poly: Sequence[P]) -> float:
    return 0.5 * sum(poly[i][0] * poly[(i + 1) % len(poly)][1] - poly[(i + 1) % len(poly)][0] * poly[i][1] for i in range(len(poly)))


def centroid(poly: Sequence[P]) -> P:
    a = area(poly)
    if abs(a) < 1e-9:
        return (sum(p[0] for p in poly) / len(poly), sum(p[1] for p in poly) / len(poly))
    cx = cy = 0.0
    for i in range(len(poly)):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % len(poly)]
        c = x0 * y1 - x1 * y0
        cx += (x0 + x1) * c
        cy += (y0 + y1) * c
    return (cx / (6 * a), cy / (6 * a))


def open_ring(poly: Sequence[P]) -> List[P]:
    pts = [(float(p[0]), float(p[1])) for p in poly]
    if len(pts) > 1 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6:
        pts = pts[:-1]
    return pts


def inside(p: P, poly: Sequence[P]) -> bool:
    x, y = p
    n, c = len(poly), False
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        if (y0 > y) != (y1 > y) and x < (x1 - x0) * (y - y0) / (y1 - y0) + x0:
            c = not c
    return c


def inset(poly: Sequence[P], d: float) -> List[P]:
    """A convex polygon moved inward by d (each edge shifted, neighbours intersected). The course is a quadrilateral (handbook 3.4:
    RxCourse's boundary); a polygon too small for d shrinks to its centroid."""
    pts = open_ring(poly)
    if len(pts) < 3 or d <= 0:
        return pts
    s = 1.0 if area(pts) > 0 else -1.0                      # inward normal side
    lines = []
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        L = math.hypot(ex, ey) or 1.0
        nx, ny = -ey / L * s, ex / L * s
        lines.append(((a[0] + nx * d, a[1] + ny * d), (ex, ey)))
    out = []
    for i in range(len(lines)):
        (p1, d1), (p2, d2) = lines[i - 1], lines[i]
        den = d1[0] * d2[1] - d1[1] * d2[0]
        if abs(den) < 1e-12:
            out.append(p2)
            continue
        t = ((p2[0] - p1[0]) * d2[1] - (p2[1] - p1[1]) * d2[0]) / den
        out.append((p1[0] + t * d1[0], p1[1] + t * d1[1]))
    if area(out) * area(pts) <= 0 or not all(inside(q, pts) for q in out):
        c = centroid(pts)
        return [c, c, c]
    return out


def ray_exit(p: P, hdg_deg: float, poly: Sequence[P], max_m: float = 200.0) -> float:
    """How far a ray from p along hdg stays inside poly (0 when p is outside)."""
    if not inside(p, poly):
        return 0.0
    u = (math.cos(math.radians(hdg_deg)), math.sin(math.radians(hdg_deg)))
    best = max_m
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        den = u[0] * ey - u[1] * ex
        if abs(den) < 1e-12:
            continue
        t = ((a[0] - p[0]) * ey - (a[1] - p[1]) * ex) / den
        s = ((a[0] - p[0]) * u[1] - (a[1] - p[1]) * u[0]) / den
        if t > 1e-6 and -1e-9 <= s <= 1 + 1e-9:
            best = min(best, t)
    return best


def advance_goal(pose: P, hdg_deg: float, poly_in: Sequence[P], min_m: float = 3.0) -> Tuple[P, str]:
    """ADVANCE's far point: along the heading while it stays inside the inset polygon; when the heading leaves it within min_m (or the
    boat is outside it) - toward the polygon's centre, as far as the polygon allows."""
    d = ray_exit(pose, hdg_deg, poly_in)
    if d >= min_m:
        u = (math.cos(math.radians(hdg_deg)), math.sin(math.radians(hdg_deg)))
        return (pose[0] + d * u[0], pose[1] + d * u[1]), "along the heading"
    c = centroid(poly_in)
    h = math.degrees(math.atan2(c[1] - pose[1], c[0] - pose[0]))
    if inside(pose, poly_in):
        d = ray_exit(pose, h, poly_in)
    else:                                                    # outside the inset: the centre, then on along that line to the far side
        d = math.hypot(c[0] - pose[0], c[1] - pose[1]) + ray_exit(c, h, poly_in)
    u = (math.cos(math.radians(h)), math.sin(math.radians(h)))
    return (pose[0] + d * u[0], pose[1] + d * u[1]), "toward the field's centre"


def long_axis_deg(poly: Sequence[P]) -> float:
    pts = open_ring(poly)
    best, h = -1.0, 0.0
    for i in range(len(pts)):
        a, b = pts[i], pts[(i + 1) % len(pts)]
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L > best:
            best, h = L, math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
    return h


def lanes(poly_in: Sequence[P], lane_m: float) -> List[P]:
    """Boustrophedon lane points over the inset polygon, along its long axis: the width cut into equal strips no wider than lane_m,
    a lane down the middle of each; each lane's two ends in snake order."""
    pts = open_ring(poly_in)
    if len(pts) < 3 or abs(area(pts)) < 1.0:
        return []
    h = math.radians(long_axis_deg(pts))
    u, v = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))
    vs = [p[0] * v[0] + p[1] * v[1] for p in pts]
    lo, hi = min(vs), max(vs)
    out: List[P] = []
    nl = max(1, int(math.ceil((hi - lo) / lane_m - 1e-9)))       # lanes centred in equal strips no wider than lane_m
    step = (hi - lo) / nl
    for k in range(nl):
        w = lo + step * (k + 0.5)
        ts = []
        for i in range(len(pts)):
            a, b = pts[i], pts[(i + 1) % len(pts)]
            va, vb = a[0] * v[0] + a[1] * v[1], b[0] * v[0] + b[1] * v[1]
            if (va - w) * (vb - w) <= 0 and abs(vb - va) > 1e-9:
                f = (w - va) / (vb - va)
                q = (a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]))
                ts.append((q[0] * u[0] + q[1] * u[1], q))
        if len(ts) >= 2:
            ts.sort()
            ends = [ts[0][1], ts[-1][1]]
            out += ends if k % 2 == 0 else ends[::-1]
    return out


class Coverage:
    """1 m cells inside the course polygon; seen when inside the camera's cone (hfov) within r_m of the camera, or within near_m
    of the hull."""

    def __init__(self, poly: Sequence[P], cell_m: float = 1.0):
        pts = open_ring(poly)
        self.cell = cell_m
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        self.cells: List[P] = []
        x = math.floor(min(xs)) + cell_m / 2.0
        while x < max(xs):
            y = math.floor(min(ys)) + cell_m / 2.0
            while y < max(ys):
                if inside((x, y), pts):
                    self.cells.append((x, y))
                y += cell_m
            x += cell_m
        self.seen = [False] * len(self.cells)

    def mark(self, pose: P, hdg_deg: float, r_m: float, hfov_deg: float, near_m: float = 2.0) -> None:
        h = math.radians(hdg_deg)
        half = math.radians(hfov_deg) / 2.0
        for i, c in enumerate(self.cells):
            if self.seen[i]:
                continue
            dx, dy = c[0] - pose[0], c[1] - pose[1]
            d = math.hypot(dx, dy)
            if d <= near_m:
                self.seen[i] = True
            elif d <= r_m and abs(math.atan2(math.sin(math.atan2(dy, dx) - h), math.cos(math.atan2(dy, dx) - h))) <= half:
                self.seen[i] = True

    def pct(self) -> float:
        return 100.0 * sum(self.seen) / len(self.cells) if self.cells else 100.0

    def seen_share(self, p: P, r_m: float) -> float:
        idx = [i for i, c in enumerate(self.cells) if math.hypot(c[0] - p[0], c[1] - p[1]) <= r_m]
        return sum(self.seen[i] for i in idx) / len(idx) if idx else 1.0


def visit_point(buoy: P, boat: P, standoff_m: float, ok: Callable[[P], bool], n: int = 12) -> Optional[P]:
    """The point of the stand-off circle round the buoy nearest the boat that `ok` accepts (outside every clearance ring, inside the
    inset polygon), of n candidates; None when none is."""
    cands = []
    for k in range(n):
        t = 2 * math.pi * k / n
        q = (buoy[0] + standoff_m * math.cos(t), buoy[1] + standoff_m * math.sin(t))
        if ok(q):
            cands.append((math.hypot(q[0] - boat[0], q[1] - boat[1]), q))
    return min(cands)[1] if cands else None


def path_len(path: Sequence[P]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))
