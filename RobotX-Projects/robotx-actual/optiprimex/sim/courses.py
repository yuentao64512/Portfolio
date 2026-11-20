"""The course catalogue (v0.1.53): the courses the simulator, the fake judge, rc-test (through tools/rctest_manual.py) and the
judge page know, and the course frame the worlds are laid out in.

  ALPHA  RoboNation's Course Alpha as rc_test/test_server.py publishes it (sim/marina.py: 30.1 x 25.0 m, long side 032.08 deg,
         pinger 25 kHz = test_server.py's PINGER_FREQ_HZ default and the handbook's example "A 25kHz", 2308). Too small for the
         tasks at handbook-like distances: it carries the compact v0.1.33 layout (world_marina: the T1 field, a compact dock and
         the box faces).
  EXTRA  OURS, synthetic (Dmitrii 2026-11-06: "a bigger course Extra (to be default) ... we fit our course elements to it"):
         120 x 80 m on Alpha's axis, sharing Alpha's south corner (u_min, v_max) and growing away from the shore (north-north-east
         and west-north-west), so it contains Alpha; its own UAV transit area (40 x 20 m) beyond its shore-side (122 deg) edge at the
         start end, where the UAV launch / landing platform sits (handbook 2352: each course has one). Pinger 30 kHz (the fake
         judge's value since v0.1.23; inside 25-40 kHz, handbook 2293). It carries every task element (world_extra).

What the handbook says about the courses (handbook/RobotX-2026_Team-Handbook-20261023.md): four competition courses Alpha, Bravo,
Charlie, Delta, each with its own RoboCommand computer and network (1718, 1727-1733), one team per course at a time (1720), a
pinger frequency per course (2299-2303; the example timing 2308: A 25, B 32, C 35, D 29 kHz) and a UAV launch / landing platform per
course (2352); the mission is one continuous run over all tasks (258), the practice courses "contain an instance of each task" (3633).
It gives no course boundary, size or per-course task split (the "Preliminary Course Layout" 1343 is a picture without coordinates),
and RoboNation says to read the boundary from RxCourse (test_server.py, lines 29-32). So: no Bravo / Charlie / Delta here (their
boundaries are unknown - a synthetic stand-in would be named SIM-BRAVO ...), every course is assumed to carry every task, and a
course received that is not in this catalogue is still usable: from_body() fits its rectangle (centre, axis, length, width).

The course frame: u metres along the course's axis (its long side, `axis_deg`), v metres to starboard of it (axis + 90), from the
course's centre. A world anchored to a course (world.py `anchor: {course: ID}`) has its origin at the centre, so x_n / y_e and u / v
are a rotation of each other; world.move_to_course() re-anchors a world to another course.

  python -m optiprimex.sim.courses                       the catalogue (id, size, axis, pinger, tasks)
  python -m optiprimex.sim.courses --start-args FILE     "--lat .. --lon .. --hdg .." of a course-anchored world's start (run_bench.sh)
"""
from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from . import marina

R_EARTH = 6378137.0                           # as sim/world.py (the fake FCU's plant)
DEFAULT = "EXTRA"                             # the launchers' (run_bench / run_sim / run_rctest) and the judge page's default
MARGIN_M = 5.0                                # task elements stay this far inside a full-size course (handbook 2749: out = the run ends)


def _k(lat0: float) -> Tuple[float, float]:
    return math.radians(1.0) * R_EARTH, math.radians(1.0) * R_EARTH * math.cos(math.radians(lat0))


@dataclass
class Course:
    id: str
    name: str
    centre: Tuple[float, float]                       # lat, lon
    axis_deg: float                                   # the long side's bearing (u)
    length_m: float                                   # along u
    width_m: float                                    # along v
    pinger_hz: int
    tasks: Tuple[str, ...]
    world: Optional[str]                              # the world laid out for it (profiles/bench/world_<name>.yaml)
    source: str                                       # rc-test | ours | received
    corners: List[Tuple[float, float]] = field(default_factory=list)      # open ring, lat / lon (RxCourse closes it on the wire)
    transit: List[Tuple[float, float]] = field(default_factory=list)      # the UAV transit area, open ring
    aliases: Tuple[str, ...] = ()
    notes: str = ""

    # ---- the course frame ----------------------------------------------------------------------------------------------------
    def uv_to_ne(self, u: float, v: float) -> Tuple[float, float]:
        """Course frame -> north / east metres from the centre."""
        a = math.radians(self.axis_deg)
        return (u * math.cos(a) - v * math.sin(a), u * math.sin(a) + v * math.cos(a))

    def ne_to_uv(self, x_n: float, y_e: float) -> Tuple[float, float]:
        a = math.radians(self.axis_deg)
        return (x_n * math.cos(a) + y_e * math.sin(a), -x_n * math.sin(a) + y_e * math.cos(a))

    def latlon(self, u: float, v: float) -> Tuple[float, float]:
        x, y = self.uv_to_ne(u, v)
        kl, ko = _k(self.centre[0])
        return (self.centre[0] + x / kl, self.centre[1] + y / ko)

    def uv_of(self, lat: float, lon: float) -> Tuple[float, float]:
        kl, ko = _k(self.centre[0])
        return self.ne_to_uv((lat - self.centre[0]) * kl, (lon - self.centre[1]) * ko)

    def contains_uv(self, u: float, v: float, margin_m: float = 0.0) -> bool:
        return abs(u) <= self.length_m / 2.0 - margin_m + 1e-6 and abs(v) <= self.width_m / 2.0 - margin_m + 1e-6

    def margin_uv(self, u: float, v: float) -> float:
        """How far inside the boundary (negative = outside), m."""
        return min(self.length_m / 2.0 - abs(u), self.width_m / 2.0 - abs(v))

    # ---- what goes out ------------------------------------------------------------------------------------------------------
    def ring(self) -> List[List[float]]:
        """The boundary as a closed ring (first == last), as RxCourse carries it (handbook 3.4.9)."""
        pts = [list(p) for p in self.corners]
        return pts + [list(self.corners[0])] if pts else []

    def transit_ring(self) -> List[List[float]]:
        pts = [list(p) for p in self.transit]
        return pts + [list(self.transit[0])] if pts else []

    def corners_ne(self) -> List[List[float]]:
        """The corners in the course-anchored world frame (x_n, y_e from the centre) - a world's `bounds`."""
        return [[round(c, 2) for c in self.uv_to_ne(*self.uv_of(*p))] for p in self.corners]

    def body(self) -> dict:
        """The JSON RxCourse body of the bench wire (rc/course: open ring, as the fake judge has always sent it)."""
        return {"course_id": self.id, "boundary": [list(p) for p in self.corners], "uav_transit_area": [list(p) for p in self.transit],
                "pinger_hz": int(self.pinger_hz)}

    def summary(self) -> dict:
        return {"id": self.id, "name": self.name, "source": self.source, "length_m": round(self.length_m, 1), "width_m": round(self.width_m, 1),
                "axis_deg": round(self.axis_deg, 2), "centre": [round(self.centre[0], 7), round(self.centre[1], 7)], "pinger_hz": self.pinger_hz,
                "tasks": list(self.tasks), "world": self.world, "notes": self.notes, "corners": [list(p) for p in self.corners],
                "transit": [list(p) for p in self.transit]}


def from_rect(cid: str, name: str, centre, axis_deg: float, length_m: float, width_m: float, **kw) -> Course:
    """A rectangle `length_m` along `axis_deg` x `width_m` around `centre`; corners in Alpha's order (u-, v-), (u-, v+), (u+, v+), (u+, v-)."""
    c = Course(cid, name, (float(centre[0]), float(centre[1])), float(axis_deg), float(length_m), float(width_m), **kw)
    L, Wd = length_m / 2.0, width_m / 2.0
    c.corners = [tuple(round(x, 7) for x in c.latlon(u, v)) for u, v in ((-L, -Wd), (-L, Wd), (L, Wd), (L, -Wd))]
    return c


def from_body(body: dict, cid: Optional[str] = None) -> Optional[Course]:
    """A Course from a received RxCourse body (open or closed ring): the catalogue's when its corners match one, else the
    rectangle fitted to it (centre of the extents, the longest edge's bearing as the axis) - so "move there" works for a course we
    have never seen (the real Course Alpha on the day)."""
    pts = [tuple(map(float, p[:2])) for p in (body or {}).get("boundary") or [] if p is not None and len(p) >= 2]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if len(pts) < 3:
        return None
    known = identify(pts)
    if known:
        return COURSES[known]
    lat0 = sum(p[0] for p in pts) / len(pts)
    lon0 = sum(p[1] for p in pts) / len(pts)
    kl, ko = _k(lat0)
    ne = [((p[0] - lat0) * kl, (p[1] - lon0) * ko) for p in pts]
    best = max(range(len(ne)), key=lambda i: math.dist(ne[i], ne[i - 1]))
    a, b = ne[best - 1], ne[best]
    axis = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 180.0
    probe = Course("?", "?", (lat0, lon0), axis, 0, 0, 0, (), None, "received")
    uv = [probe.ne_to_uv(*p) for p in ne]
    us, vs = [p[0] for p in uv], [p[1] for p in uv]
    cu, cv = (max(us) + min(us)) / 2.0, (max(vs) + min(vs)) / 2.0
    clat, clon = probe.latlon(cu, cv)
    c = Course(cid or str(body.get("course_id") or "RECEIVED"), "received " + str(body.get("course_id") or "course"), (clat, clon), axis,
               max(us) - min(us), max(vs) - min(vs), int(body.get("pinger_hz") or 0), ("T1", "T2", "T3", "T4"), None, "received",
               notes="fitted to a received RxCourse (not in the catalogue)")
    c.corners = [tuple(p) for p in pts]
    c.transit = [tuple(map(float, p[:2])) for p in body.get("uav_transit_area") or []]
    if len(c.transit) > 1 and c.transit[0] == c.transit[-1]:
        c.transit = c.transit[:-1]
    return c


def _open(boundary: Sequence) -> List[Tuple[float, float]]:
    pts = [tuple(map(float, p[:2])) for p in boundary or [] if p is not None and len(p) >= 2]
    return pts[:-1] if len(pts) > 1 and pts[0] == pts[-1] else pts


def same_ring(a: Sequence, b: Sequence, tol_m: float = 2.0) -> bool:
    """Two boundaries (lat / lon, open or closed rings, any start corner / direction) with the same corners within tol_m."""
    pa, pb = _open(a), _open(b)
    if len(pa) < 3 or len(pa) != len(pb):
        return False
    kl, ko = _k(pa[0][0])
    d = lambda p, q: math.hypot((p[0] - q[0]) * kl, (p[1] - q[1]) * ko)      # noqa: E731
    return all(min(d(p, q) for q in pb) <= tol_m for p in pa) and all(min(d(q, p) for p in pa) <= tol_m for q in pb)


def identify(boundary: Sequence, tol_m: float = 2.0) -> Optional[str]:
    """The catalogue id whose corners are the boundary's (any start corner / direction, open or closed ring) within tol_m; else None."""
    return next((cid for cid, c in COURSES.items() if same_ring(c.corners, boundary, tol_m)), None)


def get(cid) -> Optional[Course]:
    """A catalogue course by id or alias, any case (`alpha`, `COURSE-ALPHA`, `course_alpha`, `extra`)."""
    if cid is None:
        return None
    k = str(cid).strip().upper().replace("_", "-")
    if k in COURSES:
        return COURSES[k]
    return next((c for c in COURSES.values() if k in c.aliases), None)


def catalogue() -> List[dict]:
    return [c.summary() for c in COURSES.values()]


# ---- the catalogue ---------------------------------------------------------------------------------------------------------
def _alpha() -> Course:
    kl, ko = _k(marina.CENTRE[0])
    ne = [((p[0] - marina.CENTRE[0]) * kl, (p[1] - marina.CENTRE[1]) * ko) for p in marina.COURSE_ALPHA]
    long_edge = (ne[2][0] - ne[1][0], ne[2][1] - ne[1][1])                   # corner 1 -> 2: the long side, bearing 032
    axis = math.degrees(math.atan2(long_edge[1], long_edge[0])) % 360.0
    c = Course("ALPHA", "Course Alpha (rc-test's demo course)", marina.CENTRE, axis, math.hypot(*long_edge),
               math.dist(ne[0], ne[1]), 25000, ("T1", "T3", "BOXES"), "marina", "rc-test", aliases=("COURSE-ALPHA", "COURSE_ALPHA", "A"),
               notes="RoboNation rc_test/test_server.py COURSE_ALPHA + TRANSIT_A (sim/marina.py). 30 x 25 m holds the T1 field and a "
                     "compact dock / box faces (world_marina, v0.1.33) - not the handbook distances; no room for T2 or the UAV pad")
    c.corners = list(marina.COURSE_ALPHA)
    c.transit = list(marina.TRANSIT_A)
    return c


ALPHA = _alpha()
# EXTRA: Alpha's axis, 120 x 80 m, its (u_min, v_max) corner on Alpha's (the south corner 1.2808457 103.8550331): centre at Alpha's
# (u, v) = (-L_A/2 + 60, W_A/2 - 40)
_EXTRA_L, _EXTRA_W = 120.0, 80.0
_EXTRA_C = ALPHA.latlon(-ALPHA.length_m / 2.0 + _EXTRA_L / 2.0, ALPHA.width_m / 2.0 - _EXTRA_W / 2.0)
EXTRA = from_rect("EXTRA", "Course Extra (ours, synthetic, full size)", (round(_EXTRA_C[0], 7), round(_EXTRA_C[1], 7)), ALPHA.axis_deg,
                  _EXTRA_L, _EXTRA_W, pinger_hz=30000, tasks=("T1", "T2", "T3", "T4", "UAV", "BOXES"), world="extra", source="ours",
                  aliases=("COURSE-EXTRA", "X", "E"),
                  notes="sized from the handbook's elements: the T1 field (10 buoys, 3 gates 3 m wide 9 m apart, ~40 m with the entry / "
                        "exit circles), the T2 pipeline (20 m) with its two buoys and the two 2 m delivery platforms, the T3 dock (3 bays "
                        "x 3.5 m + 10 m to manoeuvre), our box faces, open water for T4, >= 5 m inside the boundary; the UAV pad in its "
                        "transit area")
EXTRA.transit = [tuple(round(x, 7) for x in EXTRA.latlon(u, v)) for u, v in ((-60.0, 40.0), (-60.0, 60.0), (-20.0, 60.0), (-20.0, 40.0))]

COURSES: Dict[str, Course] = {"EXTRA": EXTRA, "ALPHA": ALPHA}


def start_args(world_path: str) -> str:
    """`--lat .. --lon .. --hdg ..` of a world's start for the fake FCU - only for a course-anchored world (`anchor:`); '' otherwise
    (the other worlds keep the fake FCU's default start, Course Alpha's centre, as before v0.1.53)."""
    from . import world as W
    try:
        wd = W.load(world_path)
    except Exception:
        return ""
    if not wd.get("anchor"):
        return ""
    st = wd.get("start") or {}
    lat, lon = W.WorldFrame(wd.get("origin")).to_latlon(float(st.get("x_n", 0.0)), float(st.get("y_e", 0.0)))
    return f"--lat {lat:.8f} --lon {lon:.8f} --hdg {float(st.get('hdg_deg', 0.0)):.1f}"


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["--start-args"] and len(argv) > 1:
        print(start_args(argv[1]))
        return 0
    for c in COURSES.values():
        print(f"{c.id:6s} {c.length_m:6.1f} x {c.width_m:5.1f} m  axis {c.axis_deg:6.2f}  pinger {c.pinger_hz} Hz  tasks {','.join(c.tasks):24s} "
              f"world {c.world}  ({c.source})  centre {c.centre[0]:.7f} {c.centre[1]:.7f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
