"""campaign: the mission profile swept in the fast SIL (v0.1.39, ROADMAP WP10h) - m1t1 / m3t1 / m4t0 / m0t4 over seeds, start poses,
both planners and both perceptions (the geometric fake | the camera path: rendered frames -> the students' measuring code -> the map),
each run graded as the SIL tests grade it, the table written as markdown.

  python -m optiprimex.sim.campaign                          # everything, 8 seeds, all cores
  python -m optiprimex.sim.campaign --missions m1t1 --seeds 4 --worlds pool --perception camera
  python -m optiprimex.sim.campaign --doc docs/CAMPAIGN.md   # the table into the doc (between its CAMPAIGN markers)

A seed changes the noise (the fake camera's, the ZED-like depth's, the light phases) and, from seed 2 on, the start pose (±2 m,
±25 deg about the world's start; m3t1 about its dock approach; m4t0 the request's position; m0t4 the scan point and the target).
m1t1 also starts `far` in open water (the marina; the pool's walls would hide it): 33 m short of the entry, turned 60 deg away -
the entry is a ray until the boat drives toward it.

v0.1.48 (WP10j-b, docs/M3T1_SIM_PLAN.md 2.5): m3t1 through the camera - the dock too (dock_face / window / indicator through
cv/sim_observer), per detector (`--detectors synthetic,model`: the renderer's masks, a CV_TRAIN model `--model`, default
../CV_TRAIN/models/cvt-v2-l), each seed a random start 8-20 m off the dock line, up to 40 deg off its normal, any heading, and a random
GREEN bay; the strict grader (grade_m3t1 with the task's search log): every bay seen (>= 3 indicator votes) before the choice, the
chosen bay = GREEN, the DockingReport's bay = the bay docked in, the FirefightingReport's window_id = the lit window, one of each, in
that order, each from its own stage (no report before DOCKED); re-engagements counted. `--doc-section m3t1` writes the table between
the doc's CAMPAIGN M3T1 markers (the main table stays).

v0.1.54 (Dmitrii's items 13 / 14): m1t1 also on Course Extra (world_extra, the default worlds now pool,marina,extra) and from a `ring`
start (8-30 m from the entry at any bearing, any heading - where the S6 report came from: a far dark buoy read flashing blue and
circled); m1t1 runs get the operator's search point (t1_hint: the world's "T1: before the entry" teleport) - Extra's start area is
~50 m from Task 1. The grader fails a run whose entry circle went round another buoy ("circled buoy N ... as the entry"). Every run
is measured for the 0.60 m clearance rule (hull_clearance: the 1.19 x 0.76 m hull rectangle along the track against every course
object - buoys of any colour, the dock's structure, the box faces, the platforms; the pool walls and the dock's pontoon fingers - not
mapped objects - reported only): the table's `min clr (exempt)` column, a run below 0.60 m fails, the documented exemptions apart (a
narrow gate's own buoys while T1 crosses it - down to its centre-line clearance less NARROW_TOL_M; m3t1's dock inside the bay; m0t4's
box face in the final approach).

v0.1.56 (D8, docs/OVERNIGHT_2026-11-07.md section 3): m3t1 on every world with a dock (`--worlds`, default pool,marina,extra), the random
start of v0.1.48 for the geometric perception too, `--model-seeds N` limits the model detector (cvt-v2-l on the CPU) to the first N
seeds; t3.finger_m from the world's pontoons (the standoff lines derived, D6); the row reports the lines and the hull's smallest
distance to a finger (the fingers stay exempt from the 0.60 m rule: no camera class); the strict grader's report checks are
report_order() (each report once, from its own stage, never before DOCKED, in order).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from typing import Any, Dict, List, Optional

KIT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BENCH = os.path.join(KIT, "profiles", "bench")
T1_FAST = {"circle_v": 1.0, "transit_v": 1.2}
STRIKE_M = 0.25 + 0.91 / 2                       # buoy radius + half the beam: closer = the hull touches the buoy
MISSIONS = ("m1t1", "m3t1", "m4t0", "m0t4")
NEAR_DOCK = {"pool": {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}}
DEFAULT_MODEL = os.path.join(os.path.dirname(KIT), "CV_TRAIN", "models", "cvt-v2-l")
DETECTORS = ("synthetic", "model")
BOX_FACE = {"red": (50.0, -6.0), "green": (50.0, 0.0), "blue": (50.0, 6.0)}


def far_start(wd: dict, seed: int) -> Dict[str, float]:
    """m1t1 from far (v0.1.39): 33 m short of the entry along the line start -> entry, facing 60 deg off it (the entry beyond the
    depth limit: a ray at first) - the search must drive toward the ray."""
    e = next(o for o in wd["objects"] if o.get("role") == "entry")
    st = wd.get("start") or {"x_n": 0.0, "y_e": 0.0}
    dx, dy = e["x_n"] - st["x_n"], e["y_e"] - st["y_e"]
    n = math.hypot(dx, dy) or 1.0
    r = random.Random(seed * 31337)
    a = math.atan2(dy, dx) + math.radians(r.uniform(-15.0, 15.0))
    hdg = math.degrees(a) + r.choice([-60.0, 60.0])
    return {"x_n": e["x_n"] - 33.0 * math.cos(a), "y_e": e["y_e"] - 33.0 * math.sin(a), "hdg_deg": hdg % 360.0}


def ring_start(wd: dict, seed: int) -> Dict[str, float]:
    """v0.1.54 (item 13: Dmitrii's "couldn't find the start from where it was"): m1t1 from anywhere round the field - 8-30 m from the
    entry at any bearing, any heading; >= 3 m from every course object, inside the walls / the course (1.5 m)."""
    e = next(o for o in wd["objects"] if o.get("role") == "entry")
    obs = course_objects(wd)
    b = wd.get("bounds") if isinstance(wd.get("bounds"), list) else []
    xs, ys = [p[0] for p in b], [p[1] for p in b]
    r = random.Random(seed * 6151 + 7)
    for _ in range(2000):
        d, a = r.uniform(8.0, 30.0), r.uniform(0.0, 2 * math.pi)
        x, y = e["x_n"] + d * math.cos(a), e["y_e"] + d * math.sin(a)
        if b and wd.get("walls", True) and not (min(xs) + 1.5 <= x <= max(xs) - 1.5 and min(ys) + 1.5 <= y <= max(ys) - 1.5):
            continue
        near = min((math.hypot(x - ob["c"][0], y - ob["c"][1]) - ob["r"] if ob["shape"] == "circle"
                    else min(_pt_seg((x, y), p, q) for p, q in zip(ob["pts"], ob["pts"][1:] + ob["pts"][:1])) for ob in obs), default=99.0)
        if near < 3.0:
            continue
        return {"x_n": round(x, 2), "y_e": round(y, 2), "hdg_deg": round(r.uniform(0.0, 360.0), 1)}
    raise RuntimeError("no ring start found")


def t1_hint(wd: dict) -> Dict[str, float]:
    """v0.1.54: the operator's search point for m1t1 (t1.search_lat / search_lon) - the world's "T1: before the entry" teleport
    (Course Extra: the start area is ~50 m from Task 1, beyond the camera); {} for a world without one."""
    from . import world as W
    tp = next((t for t in wd.get("teleports") or [] if str(t.get("name", "")).startswith("T1: before")), None)
    if tp is None:
        return {}
    lat, lon = W.WorldFrame(wd.get("origin") or {"lat": 1.2810202, "lon": 103.8550097}).to_latlon(float(tp["x_n"]), float(tp["y_e"]))
    return {"search_lat": lat, "search_lon": lon}


def open_water(name: str) -> bool:
    from . import world as W
    wd = W.load(world_file(name))
    return not wd.get("bounds") or wd.get("walls", True) is False


def world_file(name: str) -> str:
    return os.path.join(BENCH, f"world_{name}.yaml")


def jitter(start: Dict[str, float], seed: int, d_m: float = 2.0, d_deg: float = 25.0) -> Dict[str, float]:
    if seed <= 1:
        return dict(start)
    r = random.Random(seed * 7919)
    return {"x_n": start["x_n"] + r.uniform(-d_m, d_m), "y_e": start["y_e"] + r.uniform(-d_m, d_m),
            "hdg_deg": (start.get("hdg_deg", 0.0) + r.uniform(-d_deg, d_deg)) % 360.0}


def m3t1_start(wd: dict, seed: int) -> Dict[str, float]:
    """v0.1.48: a random start 8-20 m off the dock line, up to 40 deg off its normal (seen from the dock's centre), any heading -
    inside the walls (1.5 m margin) and >= 2.5 m from every buoy; plus the GREEN bay (1 / 2 / 3)."""
    from . import world as W
    objs = wd.get("objects") or []
    faces = W.bay_faces(objs)
    st = wd.get("start") or {"x_n": 0.0, "y_e": 0.0}
    nn, ne = W.dock_normal(objs, (float(st.get("x_n", 0.0)), float(st.get("y_e", 0.0))))
    cx = sum(o["x_n"] for o in faces.values()) / len(faces)
    cy = sum(o["y_e"] for o in faces.values()) / len(faces)
    b = wd.get("bounds") or []
    xs, ys = [p[0] for p in b], [p[1] for p in b]
    buoys = [(o["x_n"], o["y_e"]) for o in objs if o.get("kind") == "buoy"]
    r = random.Random(seed * 7727 + 13)
    for _ in range(500):
        d, a = r.uniform(8.0, 20.0), math.radians(r.uniform(-40.0, 40.0))
        lat = d * math.tan(a)
        x, y = cx + d * nn - lat * ne, cy + d * ne + lat * nn
        if b and not (min(xs) + 1.5 <= x <= max(xs) - 1.5 and min(ys) + 1.5 <= y <= max(ys) - 1.5):
            continue
        if any(math.hypot(x - bx, y - by) < 2.5 for bx, by in buoys):
            continue
        return {"x_n": round(x, 2), "y_e": round(y, 2), "hdg_deg": round(r.uniform(0.0, 360.0), 1), "green_bay": r.choice(sorted(faces))}
    raise RuntimeError("no start found")


# ---- grading (the same measures as tests/test_t1_sim.py, test_t3_sim.py, test_box_wp8.py) -----------------------------------------
def crossings(objs: List[dict], track) -> Dict[int, Optional[bool]]:
    """Per truth gate: None (never crossed), True (RED to starboard), False (RED to port)."""
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    gates: Dict[int, Dict[str, tuple]] = {}
    for b in objs:
        if b.get("gate") is not None:
            gates.setdefault(int(b["gate"]), {})[b["colour"]] = (b["x_n"], b["y_e"])
    tr = [(x, y) for _, x, y, _, _ in track]
    out: Dict[int, Optional[bool]] = {}
    for g, d in sorted(gates.items()):
        out[g] = None
        a, b = d["red"], d["green"]
        for p1, p2 in zip(tr, tr[1:]):
            s1, s2 = o(a, b, p1), o(a, b, p2)                 # half-open: a sample exactly on the line counts once (v0.1.39)
            if o(p1, p2, a) * o(p1, p2, b) < 0 and ((s1 < 0 <= s2) or (s2 < 0 <= s1)):
                mv = (p2[0] - p1[0], p2[1] - p1[1])
                out[g] = ((a[0] - p1[0]) * -mv[1] + (a[1] - p1[1]) * mv[0]) > 0
                break
    return out


def clearance(objs: List[dict], track) -> float:
    buoys = [o for o in objs if o.get("kind") == "buoy"]
    return min((math.hypot(x - b["x_n"], y - b["y_e"]) for _, x, y, _, _ in track for b in buoys), default=99.0)


# ---- v0.1.54 (item 14): the hull-edge clearance --------------------------------------------------------------------------------
HULL_LEN_M, HULL_BEAM_M = 1.19, 0.76             # the BlueBoat's footprint (the rectangle round the track's pose, heading = its long axis)
CLEAR_M = 0.60                                   # Dmitrii 2026-11-06: keep the hull this far from every course object
NARROW_TOL_M = 0.20                              # a gate narrower than 2 (r + beam/2 + CLEAR_M): its own buoys may come to its centre-line
                                                 # clearance less this (the tracker's tolerance) - reported, not failed
DOCK_EXEMPT = ("TURN_IN", "ENTER", "DOCKED", "REPORT_DOCK", "AIM_FIRE", "REPORT_FIRE", "BACK_OUT", "UNDOCK")   # m3t1: inside the bay
BOX_EXEMPT = ("APPROACH", "STOP", "STOPPED", "DONE", "REPORT")    # m0t4: the final approach to the chosen face (its own stop distance)


def _rect(x: float, y: float, hdg_deg: float, half_l: float, half_w: float) -> List[tuple]:
    h = math.radians(hdg_deg)
    f, r = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))
    return [(x + sl * half_l * f[0] + sw * half_w * r[0], y + sl * half_l * f[1] + sw * half_w * r[1])
            for sl, sw in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def _pt_seg(p, a, b) -> float:
    ab = (b[0] - a[0], b[1] - a[1])
    L2 = ab[0] * ab[0] + ab[1] * ab[1]
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / L2))
    return math.hypot(p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1])


def _inside(p, poly) -> bool:
    s = None
    for a, b in zip(poly, poly[1:] + poly[:1]):
        c = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(c) < 1e-12:
            continue
        if s is None:
            s = c > 0
        elif (c > 0) != s:
            return False
    return True


def _poly_dist(A: List[tuple], B: List[tuple]) -> float:
    """Distance between two convex polygons (a segment = a 2-point polygon); 0 when they overlap."""
    if (len(B) > 2 and any(_inside(p, B) for p in A)) or (len(A) > 2 and any(_inside(p, A) for p in B)):
        return 0.0
    ea, eb = list(zip(A, A[1:] + A[:1])), list(zip(B, B[1:] + B[:1]))
    return min(min(_pt_seg(p, a, b) for p in A for a, b in eb), min(_pt_seg(p, a, b) for p in B for a, b in ea))


def course_objects(wd: dict) -> List[dict]:
    """Every physical course object as a plan shape: {kind, id, shape: circle (c, r) | poly (pts), gate, width}. Buoys (any colour,
    the dark ones too), the dock (each bay's deck / cube / face from the renderer's structure), the pontoon fingers, the box faces, the
    delivery platforms, the pool walls (walls: true). Not the UAV pad (outside the course)."""
    objs = wd.get("objects") or []
    out: List[dict] = []
    gates: Dict[int, List[dict]] = {}
    for o in objs:
        if o.get("kind") == "buoy" and o.get("gate") is not None:
            gates.setdefault(int(o["gate"]), []).append(o)
    gw = {g: math.hypot(v[0]["x_n"] - v[1]["x_n"], v[0]["y_e"] - v[1]["y_e"]) for g, v in gates.items() if len(v) == 2}
    for o in objs:
        k = o.get("kind")
        if k == "buoy":
            out.append({"kind": "buoy", "id": o.get("id"), "shape": "circle", "c": (o["x_n"], o["y_e"]), "r": float(o.get("size_m", 0.5)) / 2.0,
                        "gate": o.get("gate"), "width": gw.get(o.get("gate"))})
        elif k == "pontoon":
            out.append({"kind": "pontoon", "id": o.get("id"), "shape": "poly", "pts": [(o["x_n"], o["y_e"]), (o["x2_n"], o["y2_e"])], "pad": 0.0})   # the world: a line
        elif k == "platform":
            hs = float(o.get("size_m", 2.0)) / 2.0
            out.append({"kind": "platform", "id": o.get("id"), "shape": "poly", "pts": _rect(o["x_n"], o["y_e"], 0.0, hs, hs), "pad": 0.0})
    try:
        from . import render as R
        sc = R.scene_from_world({k: v for k, v in wd.items() if k != "bounds"})
        for bx in sc.boxes:
            kind = bx.tag.split(":", 1)[0]
            if kind in ("window", "indicator"):
                continue                                        # on the face: the face's panel covers them
            yaw = bx.yaw_deg
            out.append({"kind": "box" if kind == "box" else "dock", "id": bx.tag, "shape": "poly",
                        "pts": _rect(bx.c[0], bx.c[1], yaw, bx.half[0], bx.half[1]), "pad": 0.0})
    except ImportError:                                          # no numpy: the buoys / pontoons only
        pass
    b = wd.get("bounds") or []
    if isinstance(b, list) and wd.get("walls", True):
        for i in range(len(b)):
            out.append({"kind": "wall", "id": f"wall{i}", "shape": "poly", "pts": [tuple(b[i]), tuple(b[(i + 1) % len(b)])], "pad": 0.0})
    return out


def _exempt(mission: str, ob: dict, state: str) -> bool:
    if mission == "m3t1" and ob["kind"] == "dock" and state in DOCK_EXEMPT:
        return True
    if mission == "m0t4" and ob["kind"] == "box" and state in BOX_EXEMPT:
        return True
    if ob["kind"] == "wall":                     # the pool walls / the course edge: not a mapped object (the FCU fence guards it)
        return True
    if ob["kind"] == "pontoon":                  # the dock's fingers: not mapped (no camera class) - reported; m3t1's sway line keeps
        return True                              # 1.6 m from their tips nominal (v0.1.56: standoff 5.7 m - finger 3.5 m - half the hull 0.6 m)
    return False


def finger_clearance(wd: dict, track) -> Optional[float]:
    """v0.1.56: the hull rectangle's smallest distance to a pontoon finger over the whole track (None without fingers) - reported."""
    fs = [o for o in course_objects(wd) if o["kind"] == "pontoon"]
    if not fs:
        return None
    d = 99.0
    for _t, x, y, h, _st in track:
        hull = _rect(x, y, h, HULL_LEN_M / 2.0, HULL_BEAM_M / 2.0)
        d = min(d, min(_poly_dist(hull, f["pts"]) for f in fs))
    return round(d, 2)


def hull_clearance(wd: dict, track, mission: str = "m1t1", clear_m: float = CLEAR_M, step_m: float = 0.1) -> Dict[str, Any]:
    """v0.1.54 (item 14): the minimum distance from the hull's rectangle (1.19 x 0.76 m at the track's pose, the samples interpolated
    every `step_m`) to every course object. `min_m` / `at` = over the objects that count; `exempt_min_m` = over the documented
    exemptions (m3t1 the dock inside the bay stages, m0t4 the box faces in the final approach, a narrow gate's own buoys down to its
    centre-line clearance less NARROW_TOL_M); `ok` = min_m >= clear_m and no narrow gate below its own floor."""
    obs = course_objects(wd)
    hl, hw = HULL_LEN_M / 2.0, HULL_BEAM_M / 2.0
    best, best_ex = (99.0, ""), (99.0, "")
    floor_bad = ""
    pts = []
    for (t0, x0, y0, h0, s0), (t1, x1, y1, h1, _s1) in zip(track, track[1:]):
        n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step_m))
        dh = (h1 - h0 + 180.0) % 360.0 - 180.0
        for i in range(n):
            f = i / n
            pts.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0), h0 + f * dh, s0, t0 + f * (t1 - t0)))
    if track:
        pts.append(tuple(track[-1][1:4]) + (track[-1][4], track[-1][0]))
    for x, y, h, st, t in pts:
        hull = None
        for ob in obs:
            if ob["shape"] == "circle":
                cx, cy = ob["c"]
                if math.hypot(cx - x, cy - y) > 6.0:
                    continue
                hr = math.radians(h)                            # the circle's centre in the hull frame, clamped to the rectangle
                dx, dy = cx - x, cy - y
                a, b = dx * math.cos(hr) + dy * math.sin(hr), -dx * math.sin(hr) + dy * math.cos(hr)
                d = max(0.0, math.hypot(a - max(-hl, min(hl, a)), b - max(-hw, min(hw, b))) - ob["r"])
            else:
                if min(_pt_seg((x, y), a, b) for a, b in zip(ob["pts"], ob["pts"][1:] + ob["pts"][:1])) > 6.0:
                    continue
                hull = hull or _rect(x, y, h, hl, hw)
                d = max(0.0, _poly_dist(hull, ob["pts"]) - ob.get("pad", 0.0))
            what = f"{ob['kind']} {ob['id']} in {st} at t {t:.0f} s"
            w = ob.get("width")
            narrow = (mission == "m1t1" and st in ("TRANSIT", "THROUGH") and ob["kind"] == "buoy" and w is not None   # v0.1.92: THROUGH = the crossing
                      and w < 2 * (ob["r"] + hw + clear_m))             # crossing a gate: T1's TRANSIT only
            if narrow and d < clear_m:
                fl = w / 2 - ob["r"] - hw - NARROW_TOL_M
                if d < fl and not floor_bad:
                    floor_bad = f"{what}: {d:.2f} m < the {w:.1f} m gate's floor {fl:.2f} m"
                if d < best_ex[0]:
                    best_ex = (d, what + f" (gate {w:.1f} m)")
            elif _exempt(mission, ob, st):
                if d < best_ex[0]:
                    best_ex = (d, what + " (exempt)")
            elif d < best[0]:
                best = (d, what)
    return {"min_m": round(best[0], 2), "at": best[1], "exempt_min_m": round(best_ex[0], 2), "exempt_at": best_ex[1],
            "ok": best[0] >= clear_m and not floor_bad, "floor_bad": floor_bad}


def winding(track, c, state: str, within: float = 6.0) -> float:
    """The winding (deg) about c while in `state`; v0.1.54: per stretch of the state (a rejected entry is circled, then the real
    one), the largest in magnitude, counting only the samples within `within` m of c (the approach to the ring - Orbit's `approach`
    round other objects - is not the circle)."""
    runs, cur = [], []
    for _, x, y, _, s in track:
        if s == state and math.hypot(x - c[0], y - c[1]) <= within:
            cur.append((x, y))
        elif cur:
            runs.append(cur)
            cur = []
    if cur:
        runs.append(cur)
    best = 0.0
    for pts in runs:
        tot = 0.0
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            a0, a1 = math.atan2(y0 - c[1], x0 - c[0]), math.atan2(y1 - c[1], x1 - c[0])
            tot += math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        if abs(tot) > abs(best):
            best = tot
    return math.degrees(best)


def grade_m1t1(objs: List[dict], r: dict) -> Dict[str, Any]:
    entry = next(o for o in objs if o.get("role") == "entry")
    ext = next(o for o in objs if o.get("role") == "exit")
    cx = crossings(objs, r["track"])
    clr = clearance(objs, r["track"])
    w_in = winding(r["track"], (entry["x_n"], entry["y_e"]), "CIRCLE_ENTRY_CW")
    w_out = winding(r["track"], (ext["x_n"], ext["y_e"]), "CIRCLE_EXIT_CCW")
    why = []
    if r["state"] != "DONE":
        why.append(f"{r['state']} in {r['sub_state']}" + (f": {r['reason'] or (r['safe_holds'] or [''])[0]}" if (r["reason"] or r["safe_holds"]) else ""))
    if any(v is not True for v in cx.values()):
        why.append("gates " + " ".join(f"{g}:{'ok' if v else ('WRONG' if v is False else 'missed')}" for g, v in cx.items()))
    if clr <= STRIKE_M:
        why.append(f"touched a buoy ({clr:.2f} m)")
    if r["state"] == "DONE" and w_in < 330:
        why.append(f"entry circle {w_in:.0f} deg")
    if r["state"] == "DONE" and w_out > -330:
        why.append(f"exit circle {w_out:.0f} deg")
    for b in objs:                              # v0.1.54 (item 13): the entry circle round another buoy (a dark one taken for the entry)
        if b.get("kind") == "buoy" and b is not entry and winding(r["track"], (b["x_n"], b["y_e"]), "CIRCLE_ENTRY_CW") >= 300:
            why.append(f"circled buoy {b['id']} ({b.get('colour')} {b.get('light')}) as the entry")
    return {"ok": not why, "why": "; ".join(why), "clearance_m": round(clr, 2), "gates": sum(1 for v in cx.values() if v), "n_gates": len(cx)}


def grade_m3t1(objs: List[dict], r: dict, green_bay: Optional[int] = None, lit_window: Optional[int] = None,
               strict: bool = False) -> Dict[str, Any]:
    """Docked in the GREEN bay + the fire complete; v0.1.48 strict (the reporting grader, M3T1_SIM_PLAN 2.5): + every bay seen
    (>= 3 indicator votes over the passes up to the choice), the chosen bay = GREEN, the DockingReport's bay = the bay docked in, the
    FirefightingReport's window_id = the lit window, exactly one report of each, docking first, each from its own stage."""
    ev = {e["event"]: e for e in r["events"]}
    faces = {int(o["bay"]): o for o in objs if o.get("kind") == "dock" and o.get("bay") is not None}
    if green_bay is None:
        g = next((o for o in objs if o.get("kind") == "dock" and o.get("colour") == "green"), None)
        green_bay = int(g["bay"]) if g is not None else None
    why = []
    if r["state"] != "DONE":
        why.append(f"{r['state']} in {r['sub_state']}" + (f": {(r['safe_holds'] or [r['reason']])[0]}" if (r["reason"] or r["safe_holds"]) else ""))
    if green_bay is not None and (ev.get("docking") or {}).get("bay") != green_bay:
        why.append(f"docked bay {(ev.get('docking') or {}).get('bay')} (green = {green_bay})")
    if r["state"] == "DONE" and not (ev.get("firefighting") or {}).get("complete"):
        why.append("fire not complete")
    out: Dict[str, Any] = {"reengaged": r.get("reengaged", 0)}
    if strict:
        search = r.get("search") or []
        chosen = next((p for p in search if p.get("chosen") is not None), None)
        upto = search[:search.index(chosen) + 1] if chosen else search
        seen = {}
        for p in upto:
            for n, v in (p.get("votes") or {}).items():
                seen[int(n)] = seen.get(int(n), 0) + int(v.get("red", 0)) + int(v.get("green", 0))
        out["votes_min"] = min(seen.values()) if seen else 0
        if not seen or len(seen) < len(faces) or min(seen.values()) < 3:
            why.append(f"not every bay seen before the choice ({seen})")
        final = [p for p in search if p.get("chosen") is not None]
        if not final or final[-1]["chosen"] != green_bay:
            why.append(f"chose bay {final[-1]['chosen'] if final else None} (green = {green_bay})")
        docked = [(x, y) for _, x, y, _, s in r["track"] if s == "DOCKED"]
        if docked and faces:
            yx = (sum(p[0] for p in docked) / len(docked), sum(p[1] for p in docked) / len(docked))
            in_bay = min(faces, key=lambda n: math.hypot(faces[n]["x_n"] - yx[0], faces[n]["y_e"] - yx[1]))
            if (ev.get("docking") or {}).get("bay") != in_bay:
                why.append(f"DockingReport bay {(ev.get('docking') or {}).get('bay')} but docked in bay {in_bay}")
        elif r["state"] == "DONE":
            why.append("never DOCKED")
        ff = ev.get("firefighting") or {}
        if lit_window is not None and ff.get("window_id") != lit_window:
            why.append(f"FirefightingReport window_id {ff.get('window_id')} (lit = {lit_window})")
        why += report_order(r)
    return dict(out, ok=not why, why="; ".join(why))


def report_order(r: dict, entry: Optional[str] = None) -> List[str]:
    """v0.1.56 (D7): m3t1's reports - each at most once, each from its own stage (docking from REPORT_DOCK, firefighting from
    REPORT_FIRE), never before DOCKED, the FirefightingReport after the DockingReport; a run DONE from the start (entry None or before
    DOCKED) has exactly the pair in that order; a run that ends elsewhere (a hold) has only the reports its stages reached. [] = right."""
    why = []
    log = [(k, st) for k, st, _ in (r.get("event_log") or []) if k in ("docking", "firefighting")]
    states = r.get("states") or []
    for k, stage in (("docking", "REPORT_DOCK"), ("firefighting", "REPORT_FIRE")):
        mine = [st for kk, st in log if kk == k]
        if len(mine) > 1:
            why.append(f"{len(mine)} {k} reports")
        if any(st != stage for st in mine):
            why.append(f"a {k} report from {[st for st in mine if st != stage]}")
    if "REPORT_DOCK" in states and "DOCKED" not in states[:states.index("REPORT_DOCK")]:
        why.append("DockingReport before DOCKED")
    kinds = [k for k, _ in log]
    if "docking" in kinds and "firefighting" in kinds and kinds.index("firefighting") < kinds.index("docking"):
        why.append("FirefightingReport before the DockingReport")
    if "docking" in kinds and "DOCKED" not in states:
        why.append("a DockingReport without DOCKED")
    from_start = entry in (None, "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER")
    if r["state"] == "DONE" and from_start and log != [("docking", "REPORT_DOCK"), ("firefighting", "REPORT_FIRE")]:
        why.append(f"reports {log}")
    return why


def grade_m4t0(r: dict, target, at) -> Dict[str, Any]:
    d = math.hypot(at[0] - target[0], at[1] - target[1]) if at else 99.0
    why = [] if r["state"] == "DONE" and d < 3.0 else [f"{r['state']} {d:.1f} m from the request"]
    return {"ok": not why, "why": "; ".join(why), "miss_m": round(d, 2)}


def grade_m0t4(r: dict, colour: str) -> Dict[str, Any]:
    why = []
    if r["state"] != "DONE":
        why.append(f"{r['state']} in {r['sub_state']}")
    else:
        done = next((e for e in r["events"] if e.get("event") == "box_mission_complete"), {})
        got = (done.get("target") or {}).get("colour")
        n, e = BOX_FACE[colour]
        x, y = r["track"][-1][1], r["track"][-1][2]
        if got != colour:
            why.append(f"target {got} (asked {colour})")
        if not (0.8 <= n - x <= 1.35 and abs(y - e) < 0.4):
            why.append(f"stopped {n - x:.2f} m off, {y - e:+.2f} m across")
    return {"ok": not why, "why": "; ".join(why)}


# ---- one run ---------------------------------------------------------------------------------------------------------------------
def run_case(c: Dict[str, Any]) -> Dict[str, Any]:
    from .fast_sil import FastSil
    from . import world as W
    t0 = time.time()
    seed, wname = int(c["seed"]), c["world"]
    wf = world_file(wname)
    wd = W.load(wf)
    objs = wd.get("objects") or []
    kw = dict(seed=seed, perception=c["perception"])
    from ..companion.config import Config as _Cfg                       # v0.1.92: the campaign's fast speeds (T1_FAST 1.2 m/s) were designed
    _cfg = _Cfg()                                                       # uncapped - limits.max_speed_mps (now enforced) lifted for them
    _cfg.limits.max_speed_mps = float(c.get("max_speed_mps") or 10.0)
    kw["cfg"] = _cfg
    if c.get("detector") and c["detector"] != "-":
        kw.update(detector=c["detector"], model=c.get("model") if c["detector"] == "model" else None, cam_dock=True)
        if c["detector"] == "model":
            kw.update(device=c.get("device", "cpu"))                       # v0.1.56: the CPU (the campaign never takes the GPU / MPS)
    try:
        if c["mission"] == "m1t1":
            if c.get("start") == "far":
                st = far_start(wd, seed)
            elif c.get("start") == "ring":                                  # v0.1.54
                st = ring_start(wd, seed)
            else:
                st = jitter(wd.get("start") or {"x_n": 0.0, "y_e": 0.0, "hdg_deg": 0.0}, seed)
            r = FastSil(wf, "T1", dict(T1_FAST, planner=c["planner"], **t1_hint(wd)), start=st, **kw).run(600)
            g = grade_m1t1(objs, r)
        elif c["mission"] == "m3t1" and c.get("start") == "random":       # v0.1.48: the random start, the GREEN bay drawn, the strict grader
            st = m3t1_start(wd, seed)
            ww = with_windows(wd)
            if ww is not None:                                               # v0.1.56: a world without window objects (marina)
                import tempfile
                import yaml
                fd, wf = tempfile.mkstemp(prefix=f"campaign_{wname}_", suffix=".yaml")
                with os.fdopen(fd, "w") as fh:
                    yaml.safe_dump({k: v for k, v in ww.items() if k not in ("anchor", "course", "course_id")}, fh, sort_keys=False)
                wd, objs = ww, ww["objects"]
            fm = world_finger(wd)                                            # v0.1.56 (D6): the standoff lines from this world's fingers
            s = FastSil(wf, "T3", {"finger_m": fm} if fm else {}, start={k: st[k] for k in ("x_n", "y_e", "hdg_deg")}, **kw)
            s.dock.command({"cmd": "green_bay", "bay": st["green_bay"]}, s.t)
            r = s.run(600)
            g = grade_m3t1(objs, r, green_bay=st["green_bay"], lit_window=s.dock.window, strict=True)
            g.update(green_bay=st["green_bay"], start_pose=st, infer_s=round(getattr(s.cam, "wall_infer", 0.0), 1) if s.cam else 0.0,
                     standoff_m=s.task.p["standoff_m"], standoff2_m=s.task.p["standoff2_m"], finger_clr_m=finger_clearance(wd, r["track"]))
        elif c["mission"] == "m3t1":
            st = jitter(NEAR_DOCK[wname], seed, 1.5, 15.0)
            r = FastSil(wf, "T3", {}, start=st, **kw).run(400)
            g = grade_m3t1(objs, r)
        elif c["mission"] == "m4t0":
            rr = random.Random(seed * 104729)
            target = (rr.uniform(8.0, 28.0), rr.uniform(-10.0, 10.0)) if seed > 1 else (20.0, 5.0)
            s = FastSil(wf, "T4", {}, **kw)
            lat, lon = s.origin.from_local(*target)
            s.run(1.0)
            s.task.on_incident({"kind": "assistance_request", "command_seq": 7, "position": {"lat": lat, "lon": lon}}, s.ctx())
            s.run(200, until=lambda x: x.task.m.state == "WAIT_CONFIRM")
            p = s.pose()
            at = (p.x_n, p.y_e) if s.task.m.state == "WAIT_CONFIRM" else None
            s.task.on_incident({"kind": "readiness_confirm", "command_seq": 7}, s.ctx())
            r = s.run(5)
            g = grade_m4t0(r, target, at)
        elif c["mission"] == "m0t4":
            rr = random.Random(seed * 15485863)
            colour = ["green", "red", "blue"][seed % 3] if seed > 1 else "green"
            scan = (rr.uniform(28.0, 36.0), rr.uniform(-8.0, 8.0)) if seed > 1 else (30.0, 0.0)
            a = {"sim": True, "expected_boxes": 3, "scan_hdg_deg": 0.0, "scan_x_n": scan[0], "scan_y_e": scan[1]}
            if seed > 1:
                a["target_colour"] = colour
            s = FastSil(wf, "BOX_INSPECT", a, **kw)
            r = s.run(400)
            while r["state"] == "SAFE_HOLD" and len(r["safe_holds"]) < 3:      # the operator's RESUME, as the box tests do
                s.task.resume()
                r = s.run(300)
            g = grade_m0t4(r, colour)
        else:
            raise ValueError(c["mission"])
        hc = hull_clearance(wd, r["track"], c["mission"], float(c.get("clear_m", CLEAR_M)))     # v0.1.54 (item 14)
        g.update(clr_m=hc["min_m"], clr_at=hc["at"], clr_exempt_m=hc["exempt_min_m"], clr_exempt_at=hc["exempt_at"])
        if not hc["ok"] and c.get("clearance", True):
            g["ok"] = False
            g["why"] = "; ".join(x for x in (g.get("why"), hc["floor_bad"] or f"hull {hc['min_m']:.2f} m from {hc['at']}") if x)
        out = dict(c, **g, state=r["state"], t_s=r["t"], states=r["states"], wall_s=round(time.time() - t0, 1), stage_s=stage_times(r["track"]))
        out["leads"] = sum(1 for l in r["logs"] if "toward a far sighting" in l or "toward a buoy that may be" in l)
        out["rejected"] = sum(1 for l in r["logs"] if "is not blue after all" in l)          # v0.1.54
        out["looks"] = sum(1 for l in r["logs"] if l.startswith("T1 look at"))
        if not g["ok"]:
            out["logs"] = r["logs"][-6:]
        return out
    except Exception as exc:                                    # a crash is a finding, not the end of the sweep
        return dict(c, ok=False, why=f"crash {exc.__class__.__name__}: {exc}", state="CRASH", t_s=0, states=[], wall_s=round(time.time() - t0, 1))


def has_dock(name: str) -> bool:
    from . import world as W
    return os.path.exists(world_file(name)) and any(o.get("kind") == "dock" for o in W.load(world_file(name)).get("objects") or [])


def world_finger(wd: dict) -> Optional[float]:
    """v0.1.56 (D6): the world's pontoon finger length (the median of its pontoons), None without pontoons - m3t1's t3.finger_m."""
    ln = sorted(math.hypot(o["x2_n"] - o["x_n"], o["y2_e"] - o["y_e"]) for o in wd.get("objects") or [] if o.get("kind") == "pontoon")
    return round(ln[len(ln) // 2], 2) if ln else None


def with_windows(wd: dict) -> Optional[dict]:
    """v0.1.56 (D8): a world whose dock lists no window objects (world_marina: the renderer numbers its own, render.SYN_ID0) gets the
    two windows per bay as world_pool has them (0.6 m left / right of the face point as seen from the water) under the renderer's
    synthetic ids - the geometric perception reports only listed objects, so it had no window to aim at. None = nothing to add."""
    from . import world as W
    from . import render as R
    objs = wd.get("objects") or []
    if any(o.get("kind") == "window" for o in objs) or not any(o.get("kind") == "dock" for o in objs):
        return None
    faces = W.bay_faces(objs)
    st = wd.get("start") or {"x_n": 0.0, "y_e": 0.0}
    nn, ne = W.dock_normal(objs, (float(st.get("x_n", 0.0)), float(st.get("y_e", 0.0))))
    an, ae = ne, -nn                                            # left -> right as seen from the water (dockpath's `along`)
    add = [{"id": R.SYN_ID0 + int(o["id"]) * 10 + k, "kind": "window", "colour": "black", "lit": False, "bay": int(b), "window": k,
            "x_n": round(o["x_n"] + s_ * an, 3), "y_e": round(o["y_e"] + s_ * ae, 3)}
           for b, o in sorted(faces.items()) for k, s_ in ((1, -0.6), (2, 0.6))]
    return dict(wd, objects=objs + add)


def cases(missions, seeds: int, worlds, planners, perceptions, starts=("near",), detectors=("synthetic",),
          model: Optional[str] = None, model_seeds: Optional[int] = None) -> List[Dict[str, Any]]:
    out = []
    for m in missions:
        for seed in range(1, seeds + 1):
            if m == "m1t1":
                for w in worlds:
                    for pl in planners:
                        for pe in perceptions:
                            for sp in starts:
                                if sp == "far" and not open_water(w):
                                    continue                    # a far start is outside the pool's walls (the camera sees the wall)
                                out.append({"mission": m, "world": w, "planner": pl, "perception": pe, "seed": seed, "start": sp})
            elif m == "m3t1":                                   # v0.1.56 (D8): every world with a dock, the random start for both perceptions
                for w in [w for w in worlds if has_dock(w)] or ["pool"]:
                    if "camera" in perceptions:                 # v0.1.48: the dock through the camera, per detector, random starts
                        for det in detectors:
                            if det == "model" and not (model and os.path.isdir(model)):
                                continue
                            if det == "model" and model_seeds is not None and seed > model_seeds:
                                continue                        # v0.1.56: cvt-v2-l on the CPU - the first `--model-seeds` only
                            out.append({"mission": m, "world": w, "planner": "-", "perception": "camera", "detector": det, "seed": seed,
                                        "start": "random", **({"model": model} if det == "model" else {})})
                    if "geometric" in perceptions:
                        out.append({"mission": m, "world": w, "planner": "-", "perception": "geometric", "seed": seed, "start": "random"})
            elif m == "m4t0":
                out.append({"mission": m, "world": "pool", "planner": "-", "perception": "geometric", "seed": seed})
            elif m == "m0t4":
                out.append({"mission": m, "world": "boxes", "planner": "-", "perception": "geometric", "seed": seed})
    return out


def stage_times(track) -> Dict[str, float]:
    """v0.1.43: seconds spent in each stage (the track's sub-state per 0.5 s sample) - the page's 'typical' per stage."""
    out: Dict[str, float] = {}
    for (t0, *_r0, s0), (t1, *_r1, _s1) in zip(track, track[1:]):
        out[s0] = out.get(s0, 0.0) + (t1 - t0)
    return {k: round(v, 1) for k, v in out.items()}


def stage_typical(results: List[Dict[str, Any]]) -> Dict[str, Dict[str, float]]:
    """The median seconds per stage per mission code over the passed runs (seconds in a stage the run never entered are not counted)."""
    acc: Dict[str, Dict[str, List[float]]] = {}
    for r in results:
        if r.get("ok") and r.get("stage_s"):
            for st, v in r["stage_s"].items():
                acc.setdefault(r["mission"], {}).setdefault(st, []).append(v)
    return {m: {st: sorted(v)[len(v) // 2] for st, v in d.items()} for m, d in acc.items()}


def table(results: List[Dict[str, Any]]) -> str:
    """The summary per configuration + every failure, as markdown."""
    groups: Dict[tuple, List[dict]] = {}
    for r in results:
        pe = r["perception"] + (f"/{r['detector']}" if r.get("detector") else "")
        groups.setdefault((r["mission"], r["world"], r["planner"], pe, r.get("start", "near")), []).append(r)
    lines = ["| mission | world | planner | perception | start | passed | sim time (median) | LOOK / ray leads | min clr (exempt) | failures |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for k, rs in sorted(groups.items()):
        ok = [r for r in rs if r["ok"]]
        ts = sorted(r["t_s"] for r in ok)
        med = f"{ts[len(ts) // 2]:.0f} s" if ts else "-"
        bad = ", ".join(f"s{r['seed']}" for r in rs if not r["ok"]) or "-"
        lk, ld = sum(r.get("looks", 0) for r in rs), sum(r.get("leads", 0) for r in rs)
        cl = [r["clr_m"] for r in rs if r.get("clr_m") is not None]           # v0.1.54: the hull-edge clearance, the documented exemptions apart
        ce = [r["clr_exempt_m"] for r in rs if r.get("clr_exempt_m") is not None and r["clr_exempt_m"] < 99]
        clr = (f"{min(cl):.2f} m" if cl else "-") + (f" ({min(ce):.2f})" if ce else "")
        if k[0] == "m3t1" and k[4] == "random":                 # v0.1.48: the re-engagements in place of LOOK / leads, + the wall time
            walls = sorted(r.get("wall_s", 0) for r in rs)
            fc = [r["finger_clr_m"] for r in rs if r.get("finger_clr_m") is not None]          # v0.1.56: the pontoon fingers (reported)
            sd = sorted({(r.get("standoff_m"), r.get("standoff2_m")) for r in rs if r.get("standoff_m") is not None})
            lines.append(f"| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {k[4]} | {len(ok)}/{len(rs)} | {med} | re-engaged {sum(r.get('reengaged', 0) for r in rs)}"
                         f" · wall {walls[len(walls) // 2]:.0f} s/run"
                         + (f" · lines {' '.join(f'{a:.2f}/{b:.2f}' for a, b in sd)} m" if sd else "")
                         + (f" · fingers ≥ {min(fc):.2f} m" if fc else "") + f" | {clr} | {bad} |")
            continue
        lines.append(f"| {k[0]} | {k[1]} | {k[2]} | {k[3]} | {k[4]} | {len(ok)}/{len(rs)} | {med} | {lk} / {ld} | {clr} | {bad} |")
    fails = [r for r in results if not r["ok"]]
    if fails:
        lines += ["", "Failures:", ""]
        for r in sorted(fails, key=lambda r: (r["mission"], r["world"], r["planner"], r["perception"], r.get("start", ""), r["seed"])):
            lines.append(f"- {r['mission']} {r['world']} {r['planner']} {r['perception']}{'/' + r['detector'] if r.get('detector') else ''} "
                         f"{r.get('start', 'near')} seed {r['seed']}: {r['why']}")
    return "\n".join(lines)


def write_doc(path: str, text: str, header: str, section: str = "") -> None:
    tag = f"CAMPAIGN {section.upper()}" if section else "CAMPAIGN"          # v0.1.48: a section of its own (m3t1)
    begin, end = f"<!-- {tag} BEGIN -->", f"<!-- {tag} END -->"
    doc = open(path).read() if os.path.exists(path) else f"# The mission profile campaign\n\n{begin}\n{end}\n"
    if begin not in doc:
        doc += f"\n{begin}\n{end}\n"
    a, b = doc.index(begin) + len(begin), doc.index(end)
    with open(path, "w") as f:
        f.write(doc[:a] + "\n" + header + "\n\n" + text + "\n" + doc[b:])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--missions", default=",".join(MISSIONS))
    ap.add_argument("--seeds", type=int, default=8)
    ap.add_argument("--worlds", default="pool,marina,extra", help="m1t1's worlds (v0.1.54: + extra); m3t1 runs on those with a dock (v0.1.56)")
    ap.add_argument("--planners", default="simple,s6")
    ap.add_argument("--perception", default="geometric,camera")
    ap.add_argument("--starts", default="near,far,ring", help="m1t1's starts: near (the world's start, jittered) | far (33 m short of the "
                    "entry) | ring (v0.1.54: 8-30 m from the entry at any bearing, any heading)")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", default=os.path.join(KIT, "logs", "campaign"))
    ap.add_argument("--doc", default=None, help="write the table into this markdown file (between its CAMPAIGN markers)")
    ap.add_argument("--detectors", default=",".join(DETECTORS), help="v0.1.48: m3t1's camera detectors (synthetic = the renderer's masks, "
                    "model = --model)")
    ap.add_argument("--model", default=DEFAULT_MODEL, help="the CV_TRAIN model for --detectors model (skipped when the folder is missing)")
    ap.add_argument("--doc-section", default="", help="v0.1.48: write the table between the doc's CAMPAIGN <SECTION> markers (e.g. m3t1)")
    ap.add_argument("--model-seeds", type=int, default=None, help="v0.1.56: m3t1's model detector on the first N seeds only (CPU time)")
    ap.add_argument("--stages-out", default=None, help="v0.1.43: write the median seconds per stage per mission (the page's 'typical') as JSON")
    a = ap.parse_args(argv)
    cs = cases(a.missions.split(","), a.seeds, a.worlds.split(","), a.planners.split(","), a.perception.split(","), a.starts.split(","),
               a.detectors.split(","), a.model, a.model_seeds)
    t0 = time.time()
    if "m3t1" in a.missions and "model" in a.detectors and "camera" in a.perception and not os.path.isdir(a.model or ""):
        print(f"[campaign] m3t1 --detectors model skipped: no model folder {a.model} (--model DIR)", flush=True)
    print(f"[campaign] {len(cs)} runs on {a.jobs} processes", flush=True)
    results = []
    with ProcessPoolExecutor(max_workers=a.jobs) as ex:
        for r in ex.map(run_case, cs):
            results.append(r)
            print(f"  {'ok ' if r['ok'] else 'BAD'} {r['mission']} {r['world']} {r['planner']} {r['perception']}{'/' + r['detector'] if r.get('detector') else ''} {r.get('start', '')} s{r['seed']} "
                  f"{r['state']} {r['t_s']} s ({r['wall_s']} s){'  ' + r['why'] if not r['ok'] else ''}", flush=True)
    os.makedirs(a.out, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    with open(os.path.join(a.out, f"campaign_{stamp}.json"), "w") as f:
        json.dump(results, f, indent=1, default=str)
    txt = table(results)
    n_ok = sum(r["ok"] for r in results)
    header = f"{n_ok}/{len(results)} passed · {time.strftime('%Y-%m-%d %H:%M')} · {a.seeds} seeds · {time.time() - t0:.0f} s wall · `python -m optiprimex.sim.campaign {' '.join(argv or sys.argv[1:])}`"
    print("\n" + header + "\n\n" + txt)
    if a.doc:
        write_doc(a.doc, txt, header, a.doc_section)
    if a.stages_out:
        with open(a.stages_out, "w") as f:
            json.dump({"source": f"python -m optiprimex.sim.campaign {' '.join(argv or sys.argv[1:])}", "when": time.strftime("%Y-%m-%d"),
                       "median_s": stage_typical(results)}, f, indent=1)
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
