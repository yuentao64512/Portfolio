"""Task 1 gate planning on the PerceptionMap (ROADMAP WP2; pathing_sim spec section 6).

Two planners behind one switch (`t1.planner` in the usv profile, `planner` in START_TASK args):
  simple  the gate-midpoint chain (what usv_ui `pathgen` builds by hand): boat -> pre -> mid -> post, straight legs
  s6      the pathing_sim S6/P4 planner's leg: a Dubins curve (turn radius rmin) from the boat's pose into the gate's
          pre point along its crossing direction, then mid and post (spec 6.4-6.6); straight when the Dubins curve loops
Both use the same chain ordering (spec 6.3: nearest, penalising items behind and gates that would be crossed backwards)
and the same buoy avoidance (spec 6.5 avoid). The tracker is the PerceptionMap (not ported: its entities carry the light
classification since v0.1.20), so "usable" = seen at least min_obs times.

v0.1.54 (Dmitrii 2026-11-06: "need to avoid smashing into any course object - make 0.60 m distance rule for path planner"): the
CLEARANCE RULE, hull-edge clearance. The BlueBoat is a 1.19 x 0.76 m rectangle; the planned path keeps its centre line
`keep(r) = r + beam/2 + clear_m + track_m` from the centre of every mapped object of radius r (buoys of any colour 0.25 m, the box
faces and dock faces as discs, OBJ_R) - beam/2 because a path passes an object side-on (the bow only meets what lies ahead, and a
path never ends at an obstacle), track_m for what the GUIDED carrot cuts and the plant overshoots (measured in the fast SIL, the
campaign's `min clr` column). `avoid()` now keeps a path whose every segment clears every keep disc and otherwise plans the shortest
way round the discs (free_path: A* on a 0.25 m grid, pulled straight) instead of the spec's single detour points. The documented exceptions: a gate's own two buoys while crossing
it (the crossing runs on the gate's centre line: a gate narrower than 2 keep(0.25) is passed there with its actual clearance logged),
the start and the end of a path (where the boat is / the gate's mid and post). `circle_radius()` picks the entry / exit circle's
radius nearest circle_r_m that keeps every other mapped object out of the ring's keep band.
"""
from __future__ import annotations

import math
from dataclasses import replace, dataclass, field
from typing import Iterable, List, Optional, Sequence, Set, Tuple

from . import dubins

P = Tuple[float, float]
BUOY_R = 0.25
BEAM = 0.91                            # the spec's beam (its RA); v0.1.54 the clearance rule uses the BlueBoat's 0.76 m (HULL_BEAM)
HULL_LEN, HULL_BEAM = 1.19, 0.76       # v0.1.54: the BlueBoat's footprint
OBJ_R = {"buoy": BUOY_R, "box": 1.15, "dock": 1.2}     # v0.1.54: a mapped object as a disc (a 2 x 1 m box: half its diagonal 1.12; a bay's 1.3 m cube on its deck)
HUG_M = 0.4                            # v0.1.54: an avoided path is planned this much wider (its corners touch the discs)


@dataclass
class Clearance:
    """v0.1.54: the clearance rule (t1.clear_m, hull_len_m, hull_beam_m, track_m in the usv profile)."""
    clear_m: float = 0.60              # hull edge to the object's surface
    hull_len_m: float = HULL_LEN
    hull_beam_m: float = HULL_BEAM
    track_m: float = 0.45              # the tracker's allowance: the carrot's corner cut + the plant's overshoot (fast SIL)

    def keep(self, r_obj: float = BUOY_R) -> float:
        """The centre-line distance the planned path keeps from the centre of an object of radius r_obj."""
        return r_obj + self.hull_beam_m / 2.0 + self.clear_m + self.track_m

    def edge(self, centre_dist: float, r_obj: float = BUOY_R) -> float:
        """The hull-edge clearance of a side-on pass at centre_dist."""
        return centre_dist - r_obj - self.hull_beam_m / 2.0


def _ob(o) -> Tuple[float, float, float]:
    return (float(o[0]), float(o[1]), float(o[2]) if len(o) > 2 else BUOY_R)


def _unit(x: float, y: float) -> P:
    L = math.hypot(x, y)
    return (x / L, y / L) if L > 1e-9 else (0.0, 0.0)


def _dot(a: P, b: P) -> float:
    return a[0] * b[0] + a[1] * b[1]


@dataclass
class GateItem:
    key: Tuple[int, int]               # (red id, green id) in the map
    red: P
    green: P
    mid: P
    dir: P                             # crossing direction: RED to starboard, GREEN to port
    width: float
    lucky: bool = False                # v0.1.90: a pair of uncertain colour (either crossing direction is fine)

    @property
    def hdg_deg(self) -> float:
        return math.degrees(math.atan2(self.dir[1], self.dir[0])) % 360.0


@dataclass
class Choice:
    item: Optional[GateItem]
    wrong_side: bool = False           # reaching it from here means crossing it with RED to port
    candidates: int = 0


@dataclass
class PlannerCfg:
    kind: str = "simple"               # simple | s6
    d: float = 2.0                     # pre / post distance from the gate line (spec d = 3; 2 m in the pool)
    rmin: float = 3.0                  # Dubins turn radius (spec rmin)
    min_obs: int = 5                   # a buoy seen fewer times is not a gate (ghosts)
    margin: float = 0.1
    step: float = 0.5                  # path sample step (m)
    obstacles: List[tuple] = field(default_factory=list)      # (x, y) buoys or (x, y, r) any object (v0.1.54)
    clr: Clearance = field(default_factory=Clearance)          # v0.1.54: the clearance rule
    lead_in_m: float = 0.0             # v0.1.92: a run-in point this far before the gate on its centre line when the boat is not lined up
                                       # (the real hull turns on ~5 m at transit speed: a 2 m pre point is reached across the gate's buoy)


def gate_items(pmap, min_obs: int, passed: Iterable[Tuple[int, int]] = (), lucky: bool = False, lucky_pairs: str = "one_known") -> List[GateItem]:
    done: Set[Tuple[int, int]] = set(passed)
    out = []
    gates = list(pmap.gates(None, min_obs=min_obs))
    if lucky and hasattr(pmap, "lucky_gates"):                     # v0.1.90: + the uncertain-colour pairs, after the real ones
        try:
            gates += pmap.lucky_gates(min_obs, used_ids={i for g in gates for i in (g.red.id, g.green.id)}, mode=lucky_pairs)
        except TypeError:                                          # a map stand-in without v0.1.92's pairs
            gates += pmap.lucky_gates(min_obs, used_ids={i for g in gates for i in (g.red.id, g.green.id)})
    for g in gates:
        key = (g.red.id, g.green.id)
        if key in done:
            continue
        th = math.radians(g.through_hdg_deg)
        out.append(GateItem(key, (g.red.x_n, g.red.y_e), (g.green.x_n, g.green.y_e), (g.mid_n, g.mid_e), (math.cos(th), math.sin(th)), g.width_m,
                            lucky=bool(getattr(g, "lucky", False))))
    return out


LUCKY_COST_M = 10.0                    # v0.1.90: a real gate this much farther still wins over a lucky one


def choose_next(items: Sequence[GateItem], cur: P, dir0: Optional[P]) -> Choice:
    """spec 6.3 cost: distance + 50 if behind the travel direction + 50 if the gate would be crossed backwards."""
    best, best_c = None, 1e18
    for it in items:
        v = (it.mid[0] - cur[0], it.mid[1] - cur[1])
        u = _unit(*v)
        if it.lucky and _dot(it.dir, u) < 0:                     # v0.1.90: no colour to obey - cross it the way we come (roles swapped)
            it = replace(it, red=it.green, green=it.red, dir=(-it.dir[0], -it.dir[1]))
        c = (math.hypot(*v) + (50.0 if dir0 is not None and _dot(u, dir0) < 0 else 0.0) + (50.0 if _dot(it.dir, u) < 0 else 0.0)
             + (LUCKY_COST_M if it.lucky else 0.0))
        if c < best_c:
            best, best_c = it, c
    if best is None:
        return Choice(None, False, 0)
    u = _unit(best.mid[0] - cur[0], best.mid[1] - cur[1])
    return Choice(best, _dot(best.dir, u) < 0, len(items))


def gate_points(it: GateItem, d: float) -> List[P]:
    (mx, my), (dx, dy) = it.mid, it.dir
    return [(mx - d * dx, my - d * dy), (mx, my), (mx + d * dx, my + d * dy)]


def resample(pts: Sequence[P], step: float) -> List[P]:
    out = [tuple(pts[0])]
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(math.ceil(L / step)))
        for i in range(1, n + 1):
            out.append((a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n))
    return out


def _seg_dist(p: P, a: P, b: P) -> Tuple[float, P]:
    ab = (b[0] - a[0], b[1] - a[1])
    L2 = _dot(ab, ab)
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, _dot((p[0] - a[0], p[1] - a[1]), ab) / L2))
    q = (a[0] + t * ab[0], a[1] + t * ab[1])
    return math.hypot(p[0] - q[0], p[1] - q[1]), q


def avoid(path: List[P], obstacles: Sequence[tuple], margin: float = 0.1, clr: Optional[Clearance] = None) -> List[P]:
    """v0.1.54: the path as it is when every segment keeps every object's keep disc (Clearance.keep + margin); else the shortest
    path from its start to its end round the discs (free_path: A* on a 0.25 m grid, then pulled straight). A disc that holds the
    start (where the boat is) or the end (a gate's pre point, the circle's ring) is soft: crossed only at a cost. (Before v0.1.54:
    the spec's 6.5 avoid, up to three detour points at RA + 0.2.)"""
    clr = clr or Clearance()
    obs = [_ob(o) for o in obstacles]
    if len(path) < 2 or not obs:
        return list(path)
    a, b = tuple(path[0]), tuple(path[-1])
    hard, soft = _split(obs, a, b, clr, margin)
    if all(_seg_free(p, q, hard) for p, q in zip(path, path[1:])) and all(_seg_free(p, q, soft, 0.0) for p, q in zip(path[1:-1], path[2:-1])):
        return list(path)
    hug = [(d[0], d[1], d[2] + HUG_M) for d in hard]              # a pulled-straight path touches its discs at its corners: room for the cut
    out = free_path(a, b, hug, soft)
    if out is None:
        out = free_path(a, b, hard, soft)
    return out if out is not None else list(path)


def _split(obs, a, b, clr: Clearance, margin: float):
    """(hard, soft) discs (x, y, R): soft = holding the start or the end."""
    hard, soft = [], []
    for o in obs:
        R = clr.keep(o[2]) + margin
        (soft if math.hypot(a[0] - o[0], a[1] - o[1]) < R or math.hypot(b[0] - o[0], b[1] - o[1]) < R else hard).append((o[0], o[1], R))
    return hard, soft


def _seg_free(p, q, discs, slack: float = 0.0) -> bool:
    return all(_seg_dist((d[0], d[1]), p, q)[0] >= d[2] - slack for d in discs)


def free_path(a: P, b: P, hard, soft=(), res: float = 0.25, pad: float = 8.0) -> Optional[List[P]]:
    """v0.1.54: the shortest 8-connected grid path a -> b outside the hard discs (x, y, R), a soft disc's cells at 4x the cost, in the
    box round a, b and the discs near them (+ pad), pulled straight (the farthest point in a free line of sight); None = no way."""
    import heapq
    near = [d for d in list(hard) + list(soft) if _seg_dist((d[0], d[1]), a, b)[0] < d[2] + pad]
    xs = [a[0], b[0]] + [d[0] - d[2] for d in near] + [d[0] + d[2] for d in near]
    ys = [a[1], b[1]] + [d[1] - d[2] for d in near] + [d[1] + d[2] for d in near]
    x0, y0 = min(xs) - 2.0, min(ys) - 2.0
    nx, ny = int((max(xs) + 2.0 - x0) / res) + 1, int((max(ys) + 2.0 - y0) / res) + 1
    if nx * ny > 400000:
        return None
    hset = [d for d in hard if d in near]
    sset = [d for d in soft if d in near]

    def cell_cost(i, j):
        x, y = x0 + i * res, y0 + j * res
        for d in hset:
            if (x - d[0]) ** 2 + (y - d[1]) ** 2 < d[2] * d[2]:
                return None
        return 4.0 if any((x - d[0]) ** 2 + (y - d[1]) ** 2 < d[2] * d[2] for d in sset) else 1.0
    cache: dict = {}

    def cost(c):
        if c not in cache:
            cache[c] = cell_cost(*c)
        return cache[c]
    s = (int(round((a[0] - x0) / res)), int(round((a[1] - y0) / res)))
    g = (int(round((b[0] - x0) / res)), int(round((b[1] - y0) / res)))
    cache[s] = cache[g] = 1.0
    dist = {s: 0.0}
    prev = {}
    pq = [(0.0, s)]
    steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0), (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    while pq:
        f, c = heapq.heappop(pq)
        if c == g:
            break
        dc = dist[c]
        if f - math.hypot(g[0] - c[0], g[1] - c[1]) > dc + 1e-9:
            continue
        for di, dj, w in steps:
            n = (c[0] + di, c[1] + dj)
            if not (0 <= n[0] < nx and 0 <= n[1] < ny):
                continue
            k = cost(n)
            if k is None:
                continue
            nd = dc + w * k
            if nd < dist.get(n, 1e18):
                dist[n], prev[n] = nd, c
                heapq.heappush(pq, (nd + math.hypot(g[0] - n[0], g[1] - n[1]), n))
    if g not in dist:
        return None
    cells = [g]
    while cells[-1] != s:
        cells.append(prev[cells[-1]])
    pts = [a] + [(x0 + i * res, y0 + j * res) for i, j in reversed(cells[1:-1])] + [b]
    out = [pts[0]]                                   # pulled straight: never through a hard disc, never deeper into a soft one
    k = 0
    while k < len(pts) - 1:
        m = len(pts) - 1
        while m > k + 1 and not (_seg_free(pts[k], pts[m], hset) and _soft_ok(pts[k], pts[m], sset)):
            m -= 1
        out.append(pts[m])
        k = m
    return out


def _soft_ok(p, q, soft) -> bool:
    """A straight cut through a soft disc may not get deeper than its two ends already are."""
    for d in soft:
        lim = min(math.hypot(p[0] - d[0], p[1] - d[1]), math.hypot(q[0] - d[0], q[1] - d[1]), d[2])
        if _seg_dist((d[0], d[1]), p, q)[0] < lim - 1e-6:
            return False
    return True


def map_obstacles(pmap, min_obs: int = 5, skip=()) -> List[Tuple[float, float, float]]:
    """v0.1.54: every mapped course object as (x, y, r): the buoys (any colour, the dark ones too, seen min_obs times), the dock
    faces (min_obs), the box faces (3 x min_obs: a far box face's ghost - 1.5 m off, a few sightings - is not a wall); the stale ones
    too (out of view for max_age_s: still there)."""
    if pmap is None:
        return []
    out = []

    def ents(kind, **kw):                          # v0.1.85 (WP46): outside-course entities are still obstacles
        try:
            return pmap.entities(kind, include_outside=True, **kw)
        except TypeError:
            return pmap.entities(kind, **kw)
    for kind, n in (("buoy", min_obs), ("dock", min_obs), ("box", 3 * min_obs)):
        out += [(e.x_n, e.y_e, OBJ_R[kind]) for e in ents(kind, include_stale=True) if e.id not in skip and e.n_obs >= n]
    out += [(e.x_n, e.y_e, float(getattr(e, "ring_m", 1.5) or 1.5)) for e in ents("obstacle", include_stale=True, include_young=True)
            if e.id not in skip and (not hasattr(pmap, "obstacle_counts") or pmap.obstacle_counts(e))]
    # v0.1.80 (WP36): the guard's obstacles, their ring as the radius; v0.1.81.1: the depth ones once seen perception.min_obs times
    if hasattr(pmap, "uav_entities"):              # v0.1.66 (D22): a UAV-only buoy is an obstacle, its sigma_m added to the keep radius
        try:
            uo = pmap.uav_entities(include_outside=True)
        except TypeError:
            uo = pmap.uav_entities()
        out += [(e.x_n, e.y_e, OBJ_R["buoy"] + float(e.sigma_m)) for e in uo if e.id not in skip]
    return out


def via_points(a: P, b: P, obstacles: Sequence[tuple], clr: Optional[Clearance] = None) -> List[P]:
    """v0.1.54: the corners of the avoided way a -> b (none when the straight line keeps the rule) - Goto's `via`."""
    path = avoid([a, b], obstacles, clr=clr)
    return [tuple(p) for p in path[1:-1]]


def free_point(q: P, obstacles: Sequence[tuple], clr: Optional[Clearance] = None, margin: float = 0.2) -> P:
    """v0.1.54: q, or the nearest point outside every keep disc (+ margin) - rings of 0.25 m round q, 32 directions, up to 8 m (else
    q) - a lead's goal or T4's waiting point never on top of an object."""
    clr = clr or Clearance()
    discs = [(o[0], o[1], clr.keep(o[2]) + margin) for o in (_ob(o) for o in obstacles)]

    def free(x, y):
        return all(math.hypot(x - d[0], y - d[1]) >= d[2] for d in discs)
    if free(*q):
        return tuple(q)
    for k in range(1, 33):
        rr = 0.25 * k
        for a in range(32):
            t = 2 * math.pi * a / 32
            x, y = q[0] + rr * math.cos(t), q[1] + rr * math.sin(t)
            if free(x, y):
                return (x, y)
    return tuple(q)


def path_clearance(path: Sequence[P], obstacles: Sequence[tuple], clr: Optional[Clearance] = None, skip_end_m: float = 0.0) -> float:
    """v0.1.54: the planned hull-edge clearance (side-on) of a path from the objects - for the log and the tests."""
    clr = clr or Clearance()
    obs = [_ob(o) for o in obstacles]
    pts = list(path)
    if skip_end_m > 0 and len(pts) > 1:
        s, k = 0.0, len(pts) - 1
        while k > 0 and s < skip_end_m:
            s += math.hypot(pts[k][0] - pts[k - 1][0], pts[k][1] - pts[k - 1][1])
            k -= 1
        pts = pts[:k + 1]
    return min((clr.edge(math.hypot(p[0] - o[0], p[1] - o[1]), o[2]) for p in pts for o in obs), default=99.0)


def circle_radius(centre: P, others: Sequence[tuple], r0: float, clr: Optional[Clearance] = None, r_max: float = 5.0,
                  r_floor: float = 0.0) -> Tuple[float, float]:
    """v0.1.54: the entry / exit circle's radius: the one nearest r0 within [keep(buoy), r_max] whose ring keeps every
    other object out of its keep band (|D - r| >= keep(r_obj)); none (the geometry is narrower): within the same range the radius
    with the largest planned clearance - never closer to the circled buoy itself than its keep. -> (r, the ring's planned hull-edge
    clearance from the nearest object, the centre buoy included)."""
    clr = clr or Clearance()
    obs = [_ob(o) for o in others]
    r_min = max(clr.keep(BUOY_R), float(r_floor))     # v0.1.92: r_floor = the radius the hull flies at the circle speed (no tighter)

    def margin(r: float) -> float:
        return min([abs(math.hypot(o[0] - centre[0], o[1] - centre[1]) - r) - clr.keep(o[2]) for o in obs] + [r - r_min])

    def edge(r: float) -> float:
        return min([clr.edge(abs(math.hypot(o[0] - centre[0], o[1] - centre[1]) - r), o[2]) for o in obs] + [clr.edge(r)])
    cands = sorted({round(r_min + 0.05 * k, 2) for k in range(int((r_max - r_min) / 0.05) + 1)} | {round(r0, 2)}, key=lambda r: abs(r - r0))
    ok = [r for r in cands if r_min <= r <= r_max and margin(r) >= 0.0]
    r = ok[0] if ok else max(cands, key=lambda r: (round(edge(r), 2), -abs(r - r0)))
    return r, edge(r)


def leg(cfg: PlannerCfg, pose: Tuple[float, float, float], it: GateItem) -> Tuple[List[P], str]:
    """The path from the boat (x_n, y_e, compass heading deg) through one gate: (points, how). v0.1.54: the head (boat -> pre)
    keeps the clearance rule from every other mapped object (the gate's own two: the crossing pre -> mid -> post on its centre line
    is the exception)."""
    pre, mid, post = gate_points(it, cfg.d)
    pos = (pose[0], pose[1])
    how = "straight"
    head: List[P] = [pos, pre]
    lead = float(getattr(cfg, "lead_in_m", 0.0) or 0.0)
    if cfg.kind == "simple" and lead > cfg.d:
        rel = (pos[0] - it.mid[0], pos[1] - it.mid[1])
        along = rel[0] * it.dir[0] + rel[1] * it.dir[1]                  # < 0: before the gate
        lat = abs(-rel[0] * it.dir[1] + rel[1] * it.dir[0])
        hd = abs((math.degrees(math.atan2(it.dir[1], it.dir[0])) - pose[2] + 180.0) % 360.0 - 180.0)
        if not (along <= -cfg.d + 0.5 and lat <= 0.75 and hd <= 25.0) and along < -0.5 * lead:
            far = (it.mid[0] - lead * it.dir[0], it.mid[1] - lead * it.dir[1])
            head = [pos, far, pre]
            how = f"run-in {lead:.0f} m"
        elif not (lat <= 0.75 and hd <= 25.0):
            far = (it.mid[0] - lead * it.dir[0], it.mid[1] - lead * it.dir[1])
            head = [pos, far, pre]
            how = f"run-in {lead:.0f} m (from beside / past the gate)"
    if cfg.kind == "s6" and math.hypot(pre[0] - pos[0], pre[1] - pos[1]) > 1.0:
        cand = dubins.shortest((pos[0], pos[1], math.radians(pose[2])), (pre[0], pre[1], math.atan2(it.dir[1], it.dir[0])), cfg.rmin)
        if cand is not None and not (cand.ccc or cand.turn > 1.1 * math.pi):
            head = cand.sample(cfg.step)
            how = f"dubins {cand.kind}"
        else:
            how = "straight (dubins loops)"
    obstacles = [o for o in cfg.obstacles if (o[0], o[1]) not in (it.red, it.green)]     # its own two: the crossing's centre line
    head = avoid(head, obstacles, cfg.margin, cfg.clr)
    d = float(cfg.d)                                   # v0.1.92 (the overnight sim, world E): an object just past the gate on its centre
    while d > 0.8 and obstacles and path_clearance([mid, post], obstacles, cfg.clr) < cfg.clr.track_m + 0.05:   # line - the post point
        d -= 0.2                                       # comes back toward the gate (>= 0.8 m past it) until the run-out keeps it
        post = (mid[0] + d * it.dir[0], mid[1] + d * it.dir[1])
    return resample(head + [mid, post], cfg.step), how
