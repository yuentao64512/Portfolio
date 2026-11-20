"""guidance: turn under way, never in place (v0.1.85, WP41 T0.2 + the P0 addendum T0.2a-e + move-first; docs/PROMPT_COMPUTE_NIGHT_2026-11-11.md P0,
docs/PROMPT_PRESETS_NIGHT_2026-11-12.md §6 items 9-10, docs/SCOUT_BRIEF_2026-11-11.md §8b).

The water day (11 Nov): both planners asked for a turn in place (forward 0, yaw 10-20 deg/s); the hull turns ~1.8 deg/s in place and ~3.9 deg/s
under way, so auto never moved the boat. With the profile's `guidance.mode: underway` the tasks that move the boat (UNDERWAY = True: scout,
T1 m1t1 / m1t2) no longer hand the FCU a GUIDED position carrot (ArduRover pivots in place beyond PIVOT_TURN_ANGLE) - `shape()` turns that
carrot into a GUIDED velocity-yaw target by one law:

  yaw  = clamp(k_yaw * e, +-yaw_rate_max_dps 5)                      e = the heading error to the carrot
  v    = clamp(v_target * cos e, v_min_mps 0.4, v_target)            while |e| <= drive_cone_deg 90 (slower near the end of a leg)
       = arc_v_mps 0.3                                                beyond the cone: a slow arc, not a pirouette
  bow-clear (T0.2b): a mapped object < bow_clear_m 3.5 in the +-bow_cone_deg 30 bow cone and |e| > 30 deg -> v 0 (turn in place)
  boundary (T0.2c): the ray along the heading to the course polygon < bnd_slow_m 6 -> v <= 0.15; < bnd_stop_m 2.5 with |e| > 15 deg -> v 0
  heading time-out: v held at 0 for heading_timeout_s 15 without |e| shrinking -> drive anyway at v_min (logged once per leg)

`mode: fcu` (the default) = exactly as before: the ideal-hull campaigns and every existing test keep their numbers.
`MoveFirst` (§6 item 10, Dmitrii 00:40): every UNDERWAY task begins with >= start_fwd_m 1.0 straight ahead at >= v_min, within the first tick,
before any heading change - unless the bow-clear cone (any heading) or the boundary sees something closer than bow_clear_m ahead.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from ..companion.state import GuidedTarget, VelocityYawTarget

P = Tuple[float, float]

DEFAULTS: Dict[str, Any] = {
    "mode": "fcu", "v_min_mps": 0.4, "v_target_mps": 0.6, "drive_cone_deg": 90.0, "arc_v_mps": 0.3, "yaw_rate_max_dps": 5.0, "k_yaw": 1.0,
    "heading_timeout_s": 15.0, "bow_clear_m": 3.5, "bow_cone_deg": 30.0, "bow_err_deg": 30.0,
    "bnd_slow_m": 6.0, "bnd_slow_v_mps": 0.15, "bnd_stop_m": 2.5, "bnd_err_deg": 15.0, "los_m": 3.0,
    "start_fwd_m": 1.0, "start_fwd_max_s": 10.0, "end_slow_m": 2.0, "scan_mode": None, "scan_r_m": 5.0, "scan_v_mps": 0.3,
    "turn_first_deg": 0.0,                         # v0.1.92 (Cowork 5.2 lesson 2): > 0 = a carrot further off the bow -> hold and turn (hysteresis 15)
    "require_rxcourse": "judge", "scan_max_s": 130.0, "course_tol_m": 1.5, "hull_r0_dps": 2.0, "hull_r05_dps": 4.0, "arrive_extra_m": 1.5,
}


def hull_rate_dps(g: Dict[str, Any], v: float) -> float:
    """The yaw rate this hull does at v (the water day's numbers; the plant's omni_usv): the orbit / scan radius is v / this."""
    return float(g["hull_r0_dps"]) + (float(g["hull_r05_dps"]) - float(g["hull_r0_dps"])) * min(1.0, abs(v) / 0.5)


def turn_radius(g: Dict[str, Any], v: float) -> float:
    w = math.radians(min(hull_rate_dps(g, v), float(g["yaw_rate_max_dps"])))
    return abs(v) / w if w > 1e-6 else 0.0


class Arrival:
    """T0.2e closest-approach arrival: reached inside arrive_m, or when the range grows again after a closest approach < arrive + arrive_extra_m."""

    def __init__(self, arrive_m: float, extra_m: float = 1.5):
        self.arrive, self.extra, self.dmin = float(arrive_m), float(extra_m), float("inf")

    def __call__(self, d: Optional[float]) -> bool:
        if d is None:
            return False
        if d <= self.arrive:
            return True
        self.dmin = min(self.dmin, d)
        return self.dmin < self.arrive + self.extra and d > self.dmin + 0.3


def params(cfg) -> Dict[str, Any]:
    p = dict(DEFAULTS)
    p.update(((getattr(cfg, "extra", None) or {}).get("guidance") or {}))
    return p


def underway(cfg) -> bool:
    return str(params(cfg).get("mode") or "fcu").lower() == "underway"


def scan_mode(cfg) -> str:
    """yaw (turn on the spot, as before) | orbit (T0.3 ScanOrbit); default orbit under `mode: underway`, else yaw."""
    p = params(cfg)
    m = p.get("scan_mode")
    if m is None:
        return "orbit" if underway(cfg) else "yaw"
    return "orbit" if str(m).lower() == "orbit" else "yaw"


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def ray_to_poly(p: P, hdg_deg: float, poly: Sequence[P]) -> Optional[float]:
    """Distance from p along the heading to the first crossing of the polygon's edges (None: no crossing / no polygon)."""
    if not poly or len(poly) < 3:
        return None
    dx, dy = math.cos(math.radians(hdg_deg)), math.sin(math.radians(hdg_deg))
    best = None
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        den = dx * ey - dy * ex
        if abs(den) < 1e-9:
            continue
        wx, wy = a[0] - p[0], a[1] - p[1]
        t = (wx * ey - wy * ex) / den
        s = (wx * dy - wy * dx) / den
        if t >= 0.0 and 0.0 <= s <= 1.0 and (best is None or t < best):
            best = t
    return best


def bow_blocked(p: P, hdg_deg: float, obstacles: Sequence[tuple], g: Dict[str, Any], skip: Optional[P] = None) -> Optional[float]:
    """The range to the nearest mapped object (x, y, r) inside bow_clear_m in the +-bow_cone_deg bow cone (edge range: centre - r), or None."""
    best = None
    for o in obstacles or ():
        if skip is not None and math.hypot(o[0] - skip[0], o[1] - skip[1]) < 0.05:
            continue
        d = math.hypot(o[0] - p[0], o[1] - p[1]) - (float(o[2]) if len(o) > 2 else 0.0)
        if d > float(g["bow_clear_m"]):
            continue
        brg = math.degrees(math.atan2(o[1] - p[1], o[0] - p[0]))
        if abs(wrap180(brg - hdg_deg)) <= float(g["bow_cone_deg"]) and (best is None or d < best):
            best = d
    return best


class Steer:
    """One per task attempt: GuidedTarget carrot -> VelocityYawTarget by the law above, with the leg memory the time-out needs."""

    def __init__(self, cfg, log=None):
        self.g = params(cfg)
        self.log = log or (lambda s: None)
        self.leg = None
        self.stop_since: Optional[float] = None
        self.stop_err: Optional[float] = None
        self.forced = False
        self.last: Dict[str, Any] = {}

    def new_leg(self, key) -> None:
        if key != self.leg:
            self.leg, self.stop_since, self.stop_err, self.forced = key, None, None, False

    def law(self, now: float, pos: P, hdg_deg: float, goal: P, v_target: Optional[float], obstacles=(), poly=(), leg=None,
            end_dist: Optional[float] = None, end_xy: Optional[P] = None) -> VelocityYawTarget:
        g = self.g
        self.new_leg(leg)
        bnd_turn = False
        if poly:                                   # the carrot past the boundary (a path corner outside the field): near the edge aim at the leg's end
            ray0 = ray_to_poly(pos, hdg_deg, poly)
            if ray0 is not None and ray0 < float(g["bnd_slow_m"]) and not inside(goal, poly):
                goal = end_xy if end_xy is not None and inside(end_xy, poly) else centroid(poly)
                bnd_turn = ray0 < float(g["bnd_stop_m"])
        e = wrap180(math.degrees(math.atan2(goal[1] - pos[1], goal[0] - pos[0])) - hdg_deg)
        ymax = math.radians(float(g["yaw_rate_max_dps"]))
        yaw = max(-ymax, min(ymax, float(g["k_yaw"]) * math.radians(e)))
        vt = float(v_target or g["v_target_mps"])
        vmin = min(float(g["v_min_mps"]), vt)
        why = "law"
        if abs(e) <= float(g["drive_cone_deg"]):
            v = max(vmin, min(vt, vt * math.cos(math.radians(e))))
        else:
            v, why = min(vt, float(g["arc_v_mps"])), "arc"
        dgoal = math.hypot(goal[0] - pos[0], goal[1] - pos[1])
        if abs(e) > 20.0:                          # the goal inside the turning circle at v (reachable only if d >= 2 R(v) sin|e|): slower, else in place
            need = dgoal / (2.0 * math.sin(math.radians(min(90.0, abs(e)))))
            if turn_radius(g, v) > need:
                for vv in (0.3, 0.2, 0.15):
                    if vv < v and turn_radius(g, vv) <= need:
                        v, why = vv, "tighter turn"
                        break
                else:
                    v, why = 0.0, "inside the turning circle"
        if end_dist is not None and end_dist < float(g["end_slow_m"]):     # the last metres of a leg: ease off (never 0 while ahead)
            v = min(v, max(0.2, vmin * end_dist / float(g["end_slow_m"])))
        tf = float(getattr(self, "turn_first", None) or g.get("turn_first_deg") or 0.0)   # the task's own (T1) or the profile's
        if tf > 0:                                 # v0.1.92: a short leg is a straight leg - put the bow on it before moving
            if getattr(self, "turning", False) and abs(e) < tf - 15.0:
                self.turning = False
            elif abs(e) > tf:
                self.turning = True
            if getattr(self, "turning", False):
                v, why = 0.0, f"turn first ({abs(e):.0f} deg off)"
        bow = bow_blocked(pos, hdg_deg, obstacles, g, skip=None) if abs(e) > float(g["bow_err_deg"]) else None
        if bow is not None:
            v, why = 0.0, f"bow-clear: an object {bow:.1f} m ahead"
        ray = ray_to_poly(pos, hdg_deg, poly) if poly else None
        if poly and ray is None and not inside(pos, poly) and abs(e) > float(g["bnd_err_deg"]):
            v, why = 0.0, "outside the course, heading away"
        if ray is not None:
            if ray < float(g["bnd_stop_m"]) and (abs(e) > float(g["bnd_err_deg"]) or bnd_turn):
                v, why = 0.0, f"boundary {ray:.1f} m ahead"
            elif ray < float(g["bnd_slow_m"]):
                v = min(v, float(g["bnd_slow_v_mps"]))
                why = why if why != "law" else f"boundary slow ({ray:.1f} m)"
        if v == 0.0:
            if self.stop_since is None:
                self.stop_since, self.stop_err = now, abs(e)
            elif abs(e) < self.stop_err - 5.0:
                self.stop_since, self.stop_err = now, abs(e)               # the heading is coming round: the clock restarts
            elif now - self.stop_since >= float(g["heading_timeout_s"]) and (ray is None or ray > 1.0):
                if not self.forced:
                    self.log(f"guidance: heading time-out ({now - self.stop_since:.0f} s, |e| {abs(e):.0f} deg, {why}) - driving anyway at {vmin:.1f} m/s")
                self.forced = True
                v, why = vmin, "heading time-out"
        else:
            self.stop_since = None
        self.last = {"e_deg": round(e, 1), "v": round(v, 2), "yaw_dps": round(math.degrees(yaw), 1), "why": why,
                     "ray_m": None if ray is None else round(ray, 1)}
        return VelocityYawTarget(v, yaw)

    def shape(self, cmd, now: float, pose, origin, obstacles=(), poly=(), leg=None, end_xy: Optional[P] = None):
        """A GuidedTarget -> the law's VelocityYawTarget; a VelocityYawTarget -> its yaw rate capped; anything else unchanged."""
        if pose is None or origin is None:
            return cmd
        if isinstance(cmd, GuidedTarget):
            goal = origin.to_local(cmd.lat, cmd.lon)
            pos = (pose.x_n, pose.y_e)
            end_d = math.hypot(end_xy[0] - pos[0], end_xy[1] - pos[1]) if end_xy is not None else None
            if math.hypot(goal[0] - pos[0], goal[1] - pos[1]) < 0.3 and end_d is None:
                return cmd
            return self.law(now, pos, pose.hdg_deg, goal, cmd.speed_mps, obstacles, poly, leg, end_d, end_xy)
        if isinstance(cmd, VelocityYawTarget):
            ymax = math.radians(float(self.g["yaw_rate_max_dps"]))
            if abs(cmd.yaw_rate_rad_s) > ymax:
                return VelocityYawTarget(cmd.vx_mps, math.copysign(ymax, cmd.yaw_rate_rad_s))
        return cmd


class MoveFirst:
    """§6 item 10: >= start_fwd_m straight ahead at >= v_min from the first tick of the task, before any heading change."""

    def __init__(self, cfg, log=None):
        self.g = params(cfg)
        self.log = log or (lambda s: None)
        self.p0: Optional[P] = None
        self.t0: Optional[float] = None
        self.done = float(self.g["start_fwd_m"]) <= 0.0
        self.hdg: Optional[float] = None

    def cmd(self, now: float, pose, obstacles=(), poly=()) -> Optional[VelocityYawTarget]:
        """The forward command while the leg lasts, else None (the task's own command goes)."""
        if self.done or pose is None:
            return None
        pos = (pose.x_n, pose.y_e)
        if self.p0 is None:
            self.p0, self.t0, self.hdg = pos, now, pose.hdg_deg
            g = dict(self.g, bow_cone_deg=self.g["bow_cone_deg"])
            bow = bow_blocked(pos, pose.hdg_deg, obstacles, g)
            ray = ray_to_poly(pos, pose.hdg_deg, poly) if poly else None
            if bow is not None or (ray is not None and ray < float(self.g["bnd_stop_m"]) + float(self.g["start_fwd_m"])):
                self.done = True
                self.log("move-first: skipped - " + (f"an object {bow:.1f} m ahead (bow-clear)" if bow is not None else f"the course boundary {ray:.1f} m ahead")
                         + "; the turn rule applies")
                return None
            self.log(f"move-first: {float(self.g['start_fwd_m']):.1f} m straight ahead at {float(self.g['v_min_mps']):.1f} m/s before any turn")
        moved = math.hypot(pos[0] - self.p0[0], pos[1] - self.p0[1])
        if moved >= float(self.g["start_fwd_m"]) or now - self.t0 > float(self.g["start_fwd_max_s"]):
            self.done = True
            self.log(f"move-first: done ({moved:.1f} m in {now - self.t0:.1f} s)")
            return None
        return VelocityYawTarget(max(float(self.g["v_min_mps"]), 0.4), 0.0)


# ---- WP46: our course only ---------------------------------------------------------------------------------------------------------
def _seg_dist(p: P, a: P, b: P) -> float:
    ex, ey = b[0] - a[0], b[1] - a[1]
    L = ex * ex + ey * ey
    t = 0.0 if L < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / L))
    return math.hypot(p[0] - a[0] - t * ex, p[1] - a[1] - t * ey)


def centroid(poly: Sequence[P]) -> P:
    return (sum(q[0] for q in poly) / len(poly), sum(q[1] for q in poly) / len(poly))


def inside(p: P, poly: Sequence[P]) -> bool:
    x, y = p
    c = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def in_course(p: P, poly: Sequence[P], tol_m: float = 1.5) -> bool:
    """True when p is inside the polygon or within tol_m of its edge (no polygon: True)."""
    if not poly or len(poly) < 3:
        return True
    if inside(p, poly):
        return True
    n = len(poly)
    return min(_seg_dist(p, poly[i], poly[(i + 1) % n]) for i in range(n)) <= tol_m


def course_local(run, origin) -> List[P]:
    """The RxCourse boundary on the boat (run.course.boundary, lat / lon) in the local frame; [] without one."""
    c = getattr(run, "course", None)
    b = list(getattr(c, "boundary", None) or [])
    if origin is None or len(b) < 3:
        return []
    pts = [origin.to_local(float(q[0]), float(q[1])) for q in b]
    if len(pts) > 3 and math.hypot(pts[0][0] - pts[-1][0], pts[0][1] - pts[-1][1]) < 1e-6:
        pts = pts[:-1]
    return pts


def course_refusal(cfg, ctx) -> str:
    """WP46: '' when a field task may start; else why not. require_rxcourse: judge (default - refuse on a real RoboCommand, ocs/state judge
    `protobuf`, when the boat has no RxCourse boundary) | always | never. Bench / sim (the fake judge) keep scout's square-ahead fallback."""
    req = str(params(cfg).get("require_rxcourse") or "judge").lower()
    if req == "never":
        return ""
    b = ctx.b
    if len(course_local(b.run, b.origin)) >= 3:
        return ""
    judge = getattr(b.map, "ocs_judge", None) if b.map is not None else None
    if req == "always" or str(judge or "") == "protobuf":
        return "no RxCourse on the boat - is the course acknowledged? (a field task on a real judge never runs without our course boundary)"
    return ""
