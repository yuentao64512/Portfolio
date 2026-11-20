"""dock_keypoints_eval: the dock / box face key points from the camera, measured offline on rendered frames (v0.1.58, WP13).

Dmitrii 2026-11-07: "for m3t1 - test if we could detect docking box key points (center + boundaries) for at least 2 boxes, to establish
the line and offset it, to use as sway-search path for the next stage of the mission". This renders the simulator's camera
(sim/render, the ZED 2i's intrinsics, ZED-like depth noise) at boat poses 4-15 m from each dock / row of boxes, runs a detector
(`synthetic` = the renderer's own masks; `model` = a CV_TRAIN model, e.g. ../CV_TRAIN/models/cvt-v2-l, on the CPU), the observer's
key points (cv/sim_observer.face_keypoints: left / midpoint / right of every face not cut by the frame), and then the line the task
would fit (usv/dockpath.fit_line through every face point) and its sway-search path (the line offset by the standoff,
DockGeom.sway_path). Every number is against the world's truth (the renderer's own geometry).

  python -m optiprimex.tools.dock_keypoints_eval [--detector synthetic --detector model] [--model ../CV_TRAIN/models/cvt-v2-l]
         [--world pool --world marina --world extra --world boxrow] [--out docs/DOCK_KEYPOINTS.md] [--size 1104x621] [--device cpu]

Per pose: faces in view (truth), faces with key points, the centre / boundary errors (m), the fitted line's angle error (deg) and
offset error (m, along the truth normal at the dock centre), and whether the sway path (standoff 5 m) is within 0.3 m / 5 deg of
the truth's. Nothing here talks to a broker or a port.
"""
from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from typing import Dict, List, Optional, Tuple

from ..companion.state import LocalPose
from ..sim import render as R
from ..sim import world as W
from ..sim.dataset import world_path
from ..sim.fast_sil import CameraPath
from ..usv.dockpath import Bay, DockGeom, fit_line

STANDOFF_M, MARGIN_M, PATH_TOL_M, PATH_TOL_DEG = 5.0, 1.5, 0.3, 5.0
DISTANCES = (4.0, 6.0, 8.0, 10.0, 12.0, 15.0)
OFF_NORMAL = (-30.0, 0.0, 30.0)
P = Tuple[float, float]

# an extra world: a row of three box faces in the renderer's convention (face_hdg_deg = the direction the face looks) - world_boxes
# itself uses fake_perception's (face_hdg_deg = along the face), so its rendered boxes stand in a file behind each other (see the doc)
BOXROW = {"name": "boxrow", "origin": {"lat": 1.2810202, "lon": 103.8550097}, "start": {"x_n": 0.0, "y_e": 0.0, "hdg_deg": 0.0},
          "camera": {"name": "zed2i_left", "max_range_m": 25.0},
          "objects": [{"id": 101 + i, "kind": "box", "colour": c, "light": "steady", "x_n": 30.0, "y_e": y, "width_m": 2.0,
                       "face_hdg_deg": 180.0} for i, (c, y) in enumerate((("red", -6.0), ("green", 0.0), ("blue", 6.0)))]}


def load_world(name: str) -> dict:
    if name == "boxrow":
        return {k: (list(v) if isinstance(v, list) else v) for k, v in BOXROW.items()}
    if os.path.exists(name):
        return W.load(name)
    from ..sim.dataset import KIT
    named = os.path.join(KIT, "profiles", "bench", f"world_{name}.yaml")
    return W.load(named if os.path.exists(named) else world_path(name))


def truth_faces(wd: dict) -> Dict[str, List[dict]]:
    """{dock: [...], box: [...]}: each face's id, left / midpoint / right (N, E) as seen from the water, its look direction."""
    sc = R.scene_from_world(wd)
    out: Dict[str, List[dict]] = {"dock": [], "box": []}
    for key, part in sc.parts.items():
        if part.get("kind") != "dock_face":
            continue
        h = math.radians(part["face_hdg_deg"])
        r = (math.cos(h - math.pi / 2), math.sin(h - math.pi / 2))
        c, half = part["centre"], part["side_m"] / 2.0
        out["dock"].append({"id": part["id"], "mid": (c[0], c[1]), "left": (c[0] - half * r[0], c[1] - half * r[1]),
                            "right": (c[0] + half * r[0], c[1] + half * r[1]), "hdg": part["face_hdg_deg"], "z": c[2]})
    for o in wd.get("objects") or []:
        if o.get("kind") != "box":
            continue
        h = math.radians(float(o.get("face_hdg_deg", 90.0)))
        n, r = (math.cos(h), math.sin(h)), (math.cos(h - math.pi / 2), math.sin(h - math.pi / 2))
        dep, half = float(o.get("depth_m", 1.0)), float(o.get("width_m", 2.0)) / 2.0
        c = (float(o["x_n"]) + dep / 2 * n[0], float(o["y_e"]) + dep / 2 * n[1])
        out["box"].append({"id": int(o["id"]), "mid": c, "left": (c[0] - half * r[0], c[1] - half * r[1]),
                           "right": (c[0] + half * r[0], c[1] + half * r[1]), "hdg": float(o.get("face_hdg_deg", 90.0)),
                           "z": float(o.get("height_m", 1.2)) / 2.0})
    return {k: v for k, v in out.items() if v}


def group_geom(faces: List[dict]) -> Tuple[P, P, P]:
    """The truth line of a group: centre, normal (the faces' look direction), along (left -> right)."""
    cx = sum(f["mid"][0] for f in faces) / len(faces)
    cy = sum(f["mid"][1] for f in faces) / len(faces)
    h = math.radians(statistics.median(f["hdg"] for f in faces))
    return (cx, cy), (math.cos(h), math.sin(h)), (math.cos(h - math.pi / 2), math.sin(h - math.pi / 2))


def poses(centre: P, normal: P) -> List[Tuple[float, float, LocalPose]]:
    out = []
    for d in DISTANCES:
        for a in OFF_NORMAL:
            th = math.atan2(normal[1], normal[0]) + math.radians(a)
            x, y = centre[0] + d * math.cos(th), centre[1] + d * math.sin(th)
            hdg = math.degrees(math.atan2(centre[1] - y, centre[0] - x)) % 360.0
            out.append((d, a, LocalPose(x, y, hdg, 0.0, 0.0, 0.0)))
    return out


def in_view(cam: R.Camera, pz: LocalPose, f: dict, max_range: float = 25.0) -> bool:
    for k in ("left", "right"):
        p = R.project(cam, pz.x_n, pz.y_e, pz.hdg_deg, (f[k][0], f[k][1], f["z"]))
        if p is None or not (2 <= p[0] <= cam.w - 3) or p[2] > max_range:
            return False
    return True


def to_local(pz: LocalPose, p: dict) -> P:
    h = math.radians(pz.hdg_deg)
    f, r = float(p["forward_m"]), float(p["right_m"])
    return pz.x_n + f * math.cos(h) - r * math.sin(h), pz.y_e + f * math.sin(h) + r * math.cos(h)


def dist(a: P, b: P) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def line_metrics(dets: List[dict], tc: P, tn: P, ta: P, pz: LocalPose) -> Optional[dict]:
    """The task's fit through every face point of the frame vs the truth: angle, offset at the dock centre, the sway path."""
    if len(dets) < 2:
        return None
    pts = [p for d in dets for p in (d["left"], d["mid"], d["right"])]
    fit = fit_line(pts, (pz.x_n, pz.y_e))
    if fit is None:
        return None
    c, n, al, rms = fit
    ang = abs(math.degrees(math.atan2(al[0] * ta[1] - al[1] * ta[0], al[0] * ta[0] + al[1] * ta[1])))
    ang = min(ang, 180.0 - ang)
    # offset: where the fitted line crosses the truth normal through the truth centre (signed, + toward the water)
    den = n[0] * tn[0] + n[1] * tn[1]
    off = ((c[0] - tc[0]) * n[0] + (c[1] - tc[1]) * n[1]) / den if abs(den) > 1e-6 else float("inf")
    bays = [Bay(i, d["mid"], "white", i) for i, d in enumerate(dets, 1)]
    g = DockGeom(c, n, al, bays, source="boxes", rms_m=rms)
    path = g.sway_path(STANDOFF_M, MARGIN_M)
    t0 = (tc[0] + STANDOFF_M * tn[0], tc[1] + STANDOFF_M * tn[1])
    perp = max(abs((q[0] - t0[0]) * tn[0] + (q[1] - t0[1]) * tn[1]) for q in path)
    return {"angle_deg": ang, "offset_m": off, "path_m": perp, "rms_m": rms,
            "ok": perp <= PATH_TOL_M and ang <= PATH_TOL_DEG}


def evaluate(world: str, detector: str, model: Optional[str], size: Tuple[int, int], device: Optional[str], depth_noise: str = "zed",
             seed: int = 7) -> List[dict]:
    wd = load_world(world)
    truth = truth_faces(wd)
    cp = CameraPath(wd, seed, size, depth_noise, detector, model, True, device)
    rows = []
    for kind, faces in truth.items():
        tc, tn, ta = group_geom(faces)
        for d, a, pz in poses(tc, tn):
            t0 = time.time()
            fr = cp.frame(pz, 0.0, 0.0, 0.0)
            dock = cp.det.detect_all(fr)[1] if cp.det is not None else cp.SO.synthetic_dock_detections(fr)
            pay = cp.SO.observe(fr, [], cp.args, dock)
            recs = [r for r in (pay.get("dock") or []) if r.get("cls") == "dock_face"] if kind == "dock" else (pay.get("boxes") or [])
            dets = []
            for r in recs:
                kp = r.get("face_points")
                if not kp:
                    continue
                dets.append({"left": to_local(pz, kp["left"]), "mid": to_local(pz, kp["midpoint"]), "right": to_local(pz, kp["right"])})
            view = [f for f in faces if in_view(cp.cam, pz, f)]
            ce, be, matched = [], [], 0
            for dt in dets:
                f = min(faces, key=lambda f: dist(f["mid"], dt["mid"]))
                if dist(f["mid"], dt["mid"]) > 1.5:
                    continue                                    # a false face (or one matched to nothing)
                matched += 1
                ce.append(dist(f["mid"], dt["mid"]))
                be += [dist(f["left"], dt["left"]), dist(f["right"], dt["right"])]
            lm = line_metrics(dets, tc, tn, ta, pz)
            rows.append({"world": wd.get("name", world), "kind": kind, "detector": detector, "d_m": d, "off_deg": a,
                         "in_view": len(view), "found": len(dets), "matched": matched, "false": len(dets) - matched,
                         "centre_m": statistics.mean(ce) if ce else None, "bound_m": statistics.mean(be) if be else None,
                         "bound_max_m": max(be) if be else None, "line": lm, "wall_s": time.time() - t0})
    return rows


def _f(v, nd=2):
    return "–" if v is None else (f"{v:.{nd}f}" if isinstance(v, float) else str(v))


def _med(xs):
    xs = [x for x in xs if x is not None]
    return statistics.median(xs) if xs else None


def _p90(xs):
    xs = sorted(x for x in xs if x is not None)
    return xs[min(len(xs) - 1, int(math.ceil(0.9 * len(xs))) - 1)] if xs else None


def summary(rows: List[dict]) -> List[dict]:
    out = []
    keys = []
    for r in rows:
        k = (r["world"], r["kind"], r["detector"])
        if k not in keys:
            keys.append(k)
    for k in keys:
        rs = [r for r in rows if (r["world"], r["kind"], r["detector"]) == k]
        two = [r for r in rs if r["in_view"] >= 2]
        lines = [r["line"] for r in rs if r["line"]]
        out.append({"world": k[0], "kind": k[1], "detector": k[2], "poses": len(rs), "view2": len(two),
                    "found2": sum(1 for r in two if r["found"] >= 2), "faces_in_view": sum(r["in_view"] for r in rs),
                    "faces_found": sum(r["matched"] for r in rs), "false": sum(r["false"] for r in rs),
                    "centre_med": _med([r["centre_m"] for r in rs]), "centre_p90": _p90([r["centre_m"] for r in rs]),
                    "bound_med": _med([r["bound_m"] for r in rs]), "bound_p90": _p90([r["bound_m"] for r in rs]),
                    "ang_med": _med([l["angle_deg"] for l in lines]), "ang_p90": _p90([l["angle_deg"] for l in lines]),
                    "off_med": _med([abs(l["offset_m"]) for l in lines]), "off_p90": _p90([abs(l["offset_m"]) for l in lines]),
                    "path_ok": sum(1 for l in lines if l["ok"]), "lines": len(lines),
                    "wall_s": statistics.mean(r["wall_s"] for r in rs)})
    return out


def markdown(rows: List[dict], meta: dict) -> str:
    s = summary(rows)
    L = ["# Dock / box face key points from the camera — offline evaluation (v0.1.58, WP13)", "",
         f"Generated by `python -m optiprimex.tools.dock_keypoints_eval` ({meta['when']}; {meta['size']} frames, depth noise "
         f"`{meta['depth']}`, model `{meta['model'] or '-'}` on `{meta['device'] or 'default'}`). Poses: {len(DISTANCES)} distances "
         f"{', '.join(f'{d:g}' for d in DISTANCES)} m × {len(OFF_NORMAL)} bearings off the face normal "
         f"({', '.join(f'{a:+g}' for a in OFF_NORMAL)} deg), bow to the group's centre. A face counts when neither boundary is cut by "
         "the frame (the observer gives no key points for a cut face). The line = the task's TLS fit through every face point of "
         "the frame (`usv/dockpath.fit_line`); sway path = that line offset by the standoff "
         f"{STANDOFF_M:g} m, {MARGIN_M:g} m beyond the outer faces (`DockGeom.sway_path`); **ok** = both path ends within "
         f"{PATH_TOL_M} m of the truth's standoff line and the angle within {PATH_TOL_DEG:g} deg.", "",
         "## Summary (per world × group × detector)", "",
         "| world | group | detector | poses | poses ≥2 in view | of them ≥2 found | faces found / in view | false | centre err med / p90 (m) "
         "| boundary err med / p90 (m) | line angle med / p90 (deg) | line offset med / p90 (m) | sway path ok | s/frame |",
         "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in s:
        L.append(f"| {r['world']} | {r['kind']} | {r['detector']} | {r['poses']} | {r['view2']} | {r['found2']} | {r['faces_found']} / "
                 f"{r['faces_in_view']} | {r['false']} | {_f(r['centre_med'])} / {_f(r['centre_p90'])} | {_f(r['bound_med'])} / "
                 f"{_f(r['bound_p90'])} | {_f(r['ang_med'], 1)} / {_f(r['ang_p90'], 1)} | {_f(r['off_med'])} / {_f(r['off_p90'])} | "
                 f"{r['path_ok']} / {r['lines']} | {r['wall_s']:.2f} |")
    L += ["", "## Verdict (computed)", ""]
    for det in sorted({r["detector"] for r in rows}):
        for kind in ("dock", "box"):
            ls = [r["line"] for r in rows if r["detector"] == det and r["kind"] == kind and r["line"]]
            bs = [r["bound_m"] for r in rows if r["detector"] == det and r["kind"] == kind]
            if ls:
                L.append(f"- **{det} · {kind} faces**: sway path within {PATH_TOL_M} m / {PATH_TOL_DEG:g} deg on **{sum(1 for x in ls if x['ok'])} of "
                         f"{len(ls)}** frames with ≥ 2 faces; boundary error median {_f(_med(bs))} m, p90 {_f(_p90(bs))} m.")
    L += ["", "## How the key points are computed (and how to read the tables)", "",
          "- `cv/sim_observer.face_keypoints` (ours, v0.1.58): the bbox's outer columns give the boundaries' bearings through the "
          "intrinsics; their range comes from the depth - the face is a vertical plane, so 1/depth is linear in the normalized column: "
          "a robust line (3 passes, MAD gate) through the mask's per-column median depths, evaluated at the outer edges of the columns "
          "on that plane (a box's visible side face and a loose mask's water / sky columns are rejected). Midpoint = the 3-D middle of "
          "the two boundaries. The ranging is the students' idea (measure_mask: the median of the mask's ZED XYZ, "
          "cv/buoys/actual_buoy_observer.py) carried per column; their dock observer (cv/docking/dock_observer.py) has only the "
          "centre, no boundaries. A face within 1 % of the frame's width of an edge gets no key points.",
          "- The *dock* group = the Task 3 bays' white `dock_face` (front panel + frame); the *box* group = the m0t4 coloured boxes "
          "(solid, seen obliquely the side face shows: the corner is the boundary). `boxrow` is an extra world built here: three boxes "
          "in a row in the renderer's convention. `world_boxes` is not rendered: its `face_hdg_deg` follows fake_perception's convention "
          "(along the face) while the renderer reads it as the look direction, so its rendered boxes stand one behind the other.",
          "- *faces found* can exceed *in view*: the truth count needs both boundary points (at the face centre's height) inside the "
          "frame; the model's bbox sometimes stays clear of the edge on a face the truth check calls cut - those still matched truth "
          "(see the centre error).",
          "- 4 m: one bay (3.5 m apart) or one box (6 m apart) fills the 51 deg view - no line (the task waits / holds with the reason)."]
    L += ["", "## By distance (all worlds and groups)", "",
          "| detector | d (m) | poses | lines (≥2 faces) | sway ok | boundary err med (m) | angle med (deg) |", "|---|---|---|---|---|---|---|"]
    for det in sorted({r["detector"] for r in rows}):
        for d in DISTANCES:
            rs = [r for r in rows if r["detector"] == det and r["d_m"] == d]
            ls = [r["line"] for r in rs if r["line"]]
            L.append(f"| {det} | {d:g} | {len(rs)} | {len(ls)} | {sum(1 for x in ls if x['ok'])} | {_f(_med([r['bound_m'] for r in rs]))} | "
                     f"{_f(_med([x['angle_deg'] for x in ls]), 1)} |")
    L += ["", "## Every pose", "", "| world | group | detector | d (m) | off (deg) | in view | found | false | centre (m) | boundary mean / max (m) "
          "| angle (deg) | offset (m) | path (m) | ok |", "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lm = r["line"] or {}
        L.append(f"| {r['world']} | {r['kind']} | {r['detector']} | {r['d_m']:g} | {r['off_deg']:+g} | {r['in_view']} | {r['found']} | {r['false']} | "
                 f"{_f(r['centre_m'])} | {_f(r['bound_m'])} / {_f(r['bound_max_m'])} | {_f(lm.get('angle_deg'), 1)} | {_f(lm.get('offset_m'))} | "
                 f"{_f(lm.get('path_m'))} | {'yes' if lm.get('ok') else ('no' if lm else '–')} |")
    return "\n".join(L) + "\n"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--detector", action="append", choices=("synthetic", "model"))
    ap.add_argument("--model", default=os.path.join("..", "CV_TRAIN", "models", "cvt-v2-l"))
    ap.add_argument("--world", action="append", help="pool | marina | extra | boxrow | a world file (default: the four)")
    ap.add_argument("--size", default="1104x621")
    ap.add_argument("--depth-noise", choices=("zed", "none"), default="zed")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--out", default=os.path.join("docs", "DOCK_KEYPOINTS.md"))
    a = ap.parse_args(argv)
    dets = a.detector or ["synthetic", "model"]
    worlds = a.world or ["pool", "marina", "extra", "boxrow"]
    size = tuple(int(x) for x in a.size.lower().split("x"))
    rows: List[dict] = []
    for det in dets:
        if det == "model" and not os.path.isdir(a.model):
            print(f"[dock_keypoints_eval] no model at {a.model}: the model rows are skipped", flush=True)
            continue
        for w in worlds:
            t0 = time.time()
            rs = evaluate(w, det, a.model if det == "model" else None, size, a.device, a.depth_noise)
            rows += rs
            ok = sum(1 for r in rs if r["line"] and r["line"]["ok"])
            print(f"[dock_keypoints_eval] {w} {det}: {len(rs)} poses, {sum(1 for r in rs if r['line'])} lines, {ok} sway paths ok "
                  f"({time.time() - t0:.0f} s)", flush=True)
    meta = {"when": time.strftime("%Y-%m-%d %H:%M"), "size": a.size, "depth": a.depth_noise,
            "model": os.path.basename(os.path.normpath(a.model)) if "model" in dets else None, "device": a.device}
    with open(a.out, "w") as f:
        f.write(markdown(rows, meta))
    print(f"[dock_keypoints_eval] -> {a.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
