#!/usr/bin/env python3
"""sim_observer - OUR buoy observer for the simulator (v0.1.36, ROADMAP WP10e): the boat's chain on rendered frames.

The same steps as cv/buoys_multi/multi_buoy_observer.py on the AGX, with the simulator in place of the ZED:
  simview GET /frame.npz (the picture, the exact depth, the object under each pixel, the boat's pose)
  -> depth: exact, or ZED-like (`--depth-noise zed`: optiprimex/sim/zed.py stereo_noise)            [the camera]
  -> masks: `--detector synthetic` = the renderer's own buoy masks; `--detector model DIR` = a CV_TRAIN model (models/<name>/:
     multi_core.model_spec) run by Ultralytics on the picture, its `buoy` class                       [the detector]
  -> the students' measure_mask on the point cloud (x right, y down, z forward, as the ZED SDK gives it; their code, verbatim, loaded
     from the read-only snapshot cv/buoys/actual_buoy_observer.py - that file imports pyzed at the top, so its functions' source is
     taken, not the module), our near_limit / centre_correction / bearing_record / colour vote (cv/buoys_multi/multi_core.py),
     their buoy_geolocation with the boat's pose                                                     [the measuring]
  -> <output>/<stamp>/latest_status.json (+ buoy_observations.csv; default logs/sim_observer/), the layout cv_publish follows ->
     the boat broker -> the map.

  python3 cv/sim_observer/sim_observer.py --sim http://127.0.0.1:8097 [--detector synthetic | model DIR] [--depth-noise zed | none]
          [--size 1104x621] [--hz 4] [--output cv/sim_observer/runs] [--max-depth 25]
scripts/run_sim.sh starts it (window `obs`) with cv_publish (`cvpub`) following its runs.

v0.1.48 (WP10j-b, docs/M3T1_SIM_PLAN.md 2.1): the Task 3 dock from the camera too (`--dock on`; default: on with --detector model,
off with synthetic - run_sim.sh then keeps the geometric fake for the dock). `dock_face` -> the face point (the median of its mask's
depth), `window` -> its own depth, the light from the colour vote on its mask (off / red / green) and the window number by its place
on the face it lies on (1 = upper-left, 2 = lower-right as seen from the water), `indicator` -> the bay colour (red / green) at the
range of the face above it (CV_TRAIN: range the dock off dock_face, the indicator's mask is poor). In latest_status.json as `dock`.
"""
from __future__ import annotations

import argparse
import ast
import csv
import io
import json
import math
import os
import sys
import time
import types
import urllib.request
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np

HERE = Path(__file__).resolve().parent
CV = HERE.parent
sys.path.insert(1, str(CV / "buoys_multi"))
import multi_core as M  # noqa: E402
import guard_core as GC  # noqa: E402  (v0.1.80, WP36: the AGX observer's own guard filter)

STUDENTS = CV / "buoys" / "actual_buoy_observer.py"
WANTED = ("EARTH_M", "BuoyMeasurement", "offset_to_gps", "measure_mask", "buoy_geolocation")


def _erode(img, kernel, iterations=1):
    """cv2.erode for a binary 0/1 image and a square kernel (the only OpenCV call in measure_mask) - when OpenCV is absent (the Mac)."""
    k = kernel.shape[0] // 2
    out = img.astype(bool)
    for _ in range(iterations):
        p = np.pad(out, k, constant_values=False)
        acc = np.ones_like(out)
        for dy in range(2 * k + 1):
            for dx in range(2 * k + 1):
                acc &= p[dy:dy + out.shape[0], dx:dx + out.shape[1]]
        out = acc
    return out.astype(img.dtype)


def students(path: Path = STUDENTS) -> types.SimpleNamespace:
    """The students' measuring functions, their source verbatim, without importing the module (pyzed, ultralytics at its top)."""
    src = path.read_text()
    tree = ast.parse(src)
    keep = []
    for node in tree.body:
        name = getattr(node, "name", None) or (node.targets[0].id if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name) else None)
        if name in WANTED:
            keep.append(node)
    missing = set(WANTED) - {getattr(n, "name", None) or n.targets[0].id for n in keep}
    if missing:
        raise RuntimeError(f"{path}: not found {sorted(missing)}")
    try:
        import cv2  # noqa: F401
    except ImportError:
        cv2 = types.SimpleNamespace(erode=_erode)
    mod = types.ModuleType("students_snapshot")              # a real module: their @dataclass looks itself up in sys.modules
    mod.__dict__.update(np=np, math=math, cv2=cv2)
    sys.modules[mod.__name__] = mod
    exec("from dataclasses import dataclass\n", mod.__dict__)
    exec(compile(ast.Module(body=keep, type_ignores=[]), str(path), "exec"), mod.__dict__)
    return types.SimpleNamespace(**{k: mod.__dict__[k] for k in WANTED})


S = students()


def fetch(sim: str, w: int, h: int) -> dict:
    with urllib.request.urlopen(f"{sim.rstrip('/')}/frame.npz?w={w}&h={h}", timeout=10) as r:
        z = np.load(io.BytesIO(r.read()), allow_pickle=False)
        return {k: z[k] for k in z.files}


def point_cloud(depth: np.ndarray, cam: dict) -> np.ndarray:
    """depth along the optical axis -> XYZ (x right, y down, z forward), NaN where there is no return - the ZED's MEASURE.XYZ."""
    h, w = depth.shape
    u = (np.arange(w, dtype=np.float32) + 0.5 - cam["cx"]) / cam["fx"]
    v = (np.arange(h, dtype=np.float32) + 0.5 - cam["cy"]) / cam["fy"]
    z = np.where(np.isfinite(depth), depth, np.nan).astype(np.float32)
    return np.stack([u[None, :] * z, v[:, None] * z, z], axis=2)


def synthetic_detections(fr: dict, min_px: int = 12):
    """The renderer's own masks: one per buoy in view (the beacon is part of it), confidence 0.9."""
    ids, codes, objs, kinds = fr["ids"], fr["prim_code"], fr["prim_obj"], fr["prim_kind"]
    out = {}
    for c, o, k in zip(codes.tolist(), objs.tolist(), kinds.tolist()):
        if k == "buoy":
            out.setdefault(o, []).append(c)
    dets = []
    for oid, prims in out.items():
        m = np.isin(ids, prims)
        if int(m.sum()) >= min_px:
            dets.append({"conf": 0.9, "mask": m.astype(np.float32), "obj": oid})
    return dets


DOCK_TAGS = {"dock": "dock_face", "window": "window", "indicator": "indicator", "box": "box"}  # v0.1.48: the renderer's tags -> the model's classes
DOCK_CLASSES = ("dock_face", "window", "indicator", "box")      # v0.1.58: + box (observe_boxes; observe_dock ignores it)


def synthetic_dock_detections(fr: dict, min_px: int = 12):
    """v0.1.48: the renderer's own dock masks - the face (front panel + its frame), each window (border + pane), each indicator."""
    ids, codes, objs, kinds = fr["ids"], fr["prim_code"], fr["prim_obj"], fr["prim_kind"]
    groups = {}
    for c, o, k in zip(codes.tolist(), objs.tolist(), kinds.tolist()):
        if k in DOCK_TAGS:
            groups.setdefault((DOCK_TAGS[k], o), []).append(c)
    dets = []
    for (cls, oid), prims in groups.items():
        m = np.isin(ids, prims)
        if int(m.sum()) >= (4 if cls == "indicator" else min_px):
            dets.append({"cls": cls, "conf": 0.9, "mask": m.astype(np.float32), "obj": oid})
    return dets


class ModelDetector:
    """A CV_TRAIN model (multi_core.model_spec) on the rendered picture; Ultralytics' result shape, as on the AGX."""

    def __init__(self, model_dir: str, device: str = None):
        from ultralytics import YOLO
        self.spec = M.model_spec(model_dir, prefer_engine=False)
        self.model = YOLO(self.spec["weights"], task="segment")
        self.buoy_ids = set(self.spec["buoy_ids"])
        self.device = device
        self.last_dock = []                                        # v0.1.48: the same frame's dock classes (detect_all)

    def detect_all(self, fr: dict):
        """Every detection of the frame with its class name: (buoys, dock parts) - one inference."""
        rgb = fr["rgb"]
        bgr = rgb[:, :, ::-1].copy()
        kw = {"device": self.device} if self.device else {}
        res = self.model.predict(bgr, conf=self.spec["conf"], imgsz=self.spec["imgsz"], iou=self.spec["iou"], verbose=False, **kw)[0]
        buoys, dock = [], []
        if res.boxes is None or res.masks is None:
            return buoys, dock
        masks = res.masks.data.cpu().numpy()
        h, w = rgb.shape[:2]
        names = self.spec.get("names") or {}
        for i, box in enumerate(res.boxes):
            c = int(box.cls.item())
            name = "buoy" if c in self.buoy_ids else names.get(c)
            if name != "buoy" and name not in DOCK_CLASSES:
                continue
            m = masks[i]
            if m.shape != (h, w):                                   # nearest resize to the frame (no OpenCV needed)
                yy = (np.arange(h) * m.shape[0] / h).astype(int)
                xx = (np.arange(w) * m.shape[1] / w).astype(int)
                m = m[yy][:, xx]
            d = {"conf": float(box.conf.item()), "mask": m.astype(np.float32), "obj": None}
            if name == "buoy":
                buoys.append(d)
            else:
                dock.append(dict(d, cls=name))
        return buoys, dock

    def __call__(self, fr: dict):
        buoys, self.last_dock = self.detect_all(fr)
        return buoys


def _dock_point(xyz: np.ndarray, mask: np.ndarray, max_depth: float):
    """The median 3-D point of a dock part's mask (a MAD gate on the depth) -> (forward, right, down, n) or None."""
    valid = (mask > 0.5) & np.isfinite(xyz).all(axis=2) & (xyz[:, :, 2] > 0.3) & (xyz[:, :, 2] < float(max_depth))
    pts = xyz[valid]
    if len(pts) < 4:
        return None
    z = pts[:, 2]
    mz = float(np.median(z))
    gate = max(0.10, 3.0 * 1.4826 * float(np.median(np.abs(z - mz))))
    pts = pts[np.abs(z - mz) <= gate]
    if len(pts) < 4:
        return None
    right, down, forward = (float(v) for v in np.median(pts, axis=0))
    return forward, right, down, len(pts)


def _bbox(mask: np.ndarray):
    ys, xs = np.nonzero(mask > 0.5)
    if len(xs) == 0:
        return None
    return float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max()), float(np.median(xs)), float(np.median(ys))


def _px(mask: np.ndarray):
    """v0.1.54 (item 11): a mask's pixel box [x0, y0, x1, y1] (inclusive), None when empty - the overlay's rectangle."""
    bb = _bbox(mask)
    return None if bb is None else [int(bb[0]), int(bb[1]), int(bb[2]), int(bb[3])]


def kp_pixels(fp: dict, px, cam: Optional[dict]) -> Optional[dict]:
    """v0.1.62 (Dmitrii 2026-11-07: the surface points on the camera): a face's key points (left / midpoint / right, body-polar) -> pixel
    [u, v] for the overlays: u from each point's bearing through the intrinsics (u = cx + fx * right / forward), v = the box's middle row.
    None without the intrinsics or a box."""
    if not fp or not px or not cam or not cam.get("fx"):
        return None
    v = round((px[1] + px[3]) / 2.0, 1)
    out = {}
    for k, p in fp.items():
        f, r = p.get("forward_m"), p.get("right_m")
        if f is None or r is None or f <= 0.05:
            continue
        out[k] = [round(cam["cx"] + cam["fx"] * r / f, 1), v]
    return out or None


def frame_dets(fr: dict, meta: dict, buoys: list, dock: Optional[list], cam: Optional[dict] = None) -> dict:
    """v0.1.54 (item 11): the frame's pixel detections for the camera overlays - {t, frame_w, frame_h, frame_id, dets: [{cls, colour,
    light, x0, y0, x1, y1, range_m, conf}]}. buoys / dock = the records with their `px` (a record without a box is left out: nothing
    invented). cv_publish -> local/<v>/perception/frame_dets."""
    h, w = fr["rgb"].shape[:2]
    out = []
    for rec in list(buoys) + list(dock or []):
        px = rec.get("px")
        if not px:
            continue
        rng = rec.get("range_m")
        d = {"cls": rec.get("cls") or "buoy", "colour": rec.get("colour") or "unknown", "light": rec.get("light"),
             "x0": px[0], "y0": px[1], "x1": px[2], "y1": px[3],
             "range_m": None if rng is None else round(float(rng), 2), "conf": rec.get("confidence")}
        kp = kp_pixels(rec.get("face_points"), px, cam)              # v0.1.62: the face's key points in pixels (dock / box faces)
        if kp:
            d["kp"] = kp
        out.append(d)
    return {"t": meta.get("t"), "frame_w": int(w), "frame_h": int(h), "frame_id": meta.get("frame"), "dets": out}


def observe_dock(fr: dict, dets, a, xyz: np.ndarray, bgr: np.ndarray, cam: dict) -> list:
    """v0.1.48 (WP10j-b): the dock parts of one frame -> records (cls, range_m (horizontal), bearing_deg, forward / right / down, colour,
    light, window, face). Faces first; a window / indicator is associated with the face whose box holds its centre."""
    faces, out = [], []
    w_img = fr["rgb"].shape[1]
    for d in sorted((d for d in dets if d["cls"] == "dock_face"), key=lambda d: -d["conf"]):
        bb = _bbox(d["mask"])
        p = _dock_point(xyz, d["mask"], a.max_depth)
        if bb is None or p is None:
            continue
        faces.append((bb, p))
        cut = bb[0] <= 1 or bb[2] >= w_img - 2                     # cut by the frame edge: its median is not the face centre
        if not cut:
            rec = _dock_rec("dock_face", d, p, "white", len(faces) - 1)
            kp = face_keypoints(xyz, d["mask"], cam, a.max_depth)
            if kp is not None:                                      # v0.1.58 (WP13): the face's left / midpoint / right
                rec["face_points"] = kp
            out.append(rec)

    def face_of(bb):
        u, v = bb[4], bb[5]
        for i, (fb, _) in enumerate(faces):
            if fb[0] - 2 <= u <= fb[2] + 2 and fb[1] - 2 <= v <= fb[3] + 0.25 * (fb[3] - fb[1]) + 2:
                return i
        return None
    wins = []
    for d in (d for d in dets if d["cls"] == "window"):
        bb = _bbox(d["mask"])
        p = _dock_point(xyz, d["mask"], a.max_depth)
        if bb is None or p is None:
            continue
        colour, _ = colour_of(bgr, d["mask"])
        light = colour if colour in ("red", "green") else "off"
        fi = face_of(bb)
        num = None
        if fi is not None:
            fb = faces[fi][0]
            fw, fh = max(1.0, fb[2] - fb[0]), max(1.0, fb[3] - fb[1])
            score = (bb[4] - (fb[0] + fb[2]) / 2) / fw + (bb[5] - (fb[1] + fb[3]) / 2) / fh     # window 1: -0.42 S, window 2: +0.20 S
            num = 1 if score < -0.11 else 2
        wins.append([d, p, light, fi, num, bb])
    if len(wins) == 2 and all(x[4] is None for x in wins):          # close in, no face in the box: the upper-left of the two is 1
        a0, a1 = sorted(wins, key=lambda x: x[5][4] + x[5][5])
        a0[4], a1[4] = 1, 2
    for d, p, light, fi, num, _ in wins:
        r = _dock_rec("window", d, p, light, fi)
        r.update(light=light, window=num)
        out.append(r)
    for d in (d for d in dets if d["cls"] == "indicator"):
        bb = _bbox(d["mask"])
        if bb is None:
            continue
        fi = face_of(bb)
        if fi is None:
            continue                                                # no face above it: no range worth having
        ys, xs = np.nonzero(d["mask"] > 0.5)
        idx = M.sample_indices(len(ys))
        colour, _ = M.classify_bgr(bgr[ys[idx], xs[idx]].tolist(), min_votes=4)
        if colour not in ("red", "green"):
            continue
        out.append(_dock_rec("indicator", d, faces[fi][1], colour, fi))
    return out


def face_keypoints(xyz: np.ndarray, mask: np.ndarray, cam: dict, max_depth: float, min_cols: int = 4,
                   edge_px: Optional[int] = None) -> Optional[dict]:
    """v0.1.58 (WP13, Dmitrii 2026-11-07): a face's key points - its left and right boundaries and its midpoint - as body-polar
    {left, midpoint, right: {range_m, bearing_deg, forward_m, right_m}} (camera frame; the contract of the students' box observer,
    `quality.face_points`, as sim/fake_perception emits it). The bbox's outer columns give the boundaries' bearings through the
    intrinsics (u -> (u - cx) / fx); their range comes from the depth: the face is a vertical plane, so 1 / depth is linear in the
    normalized column - a robust line through the mask's per-column median depths, evaluated at the outer edges of the columns on
    that plane (the MAD gate rejects a loose mask's water / sky columns and a box's visible side face). The midpoint is the 3-D middle of the two (not the image middle). The ranging
    is the students' idea (measure_mask: the median of the mask's ZED XYZ, cv/buoys/actual_buoy_observer.py) carried per column."""
    bb = _bbox(mask)
    if bb is None:
        return None
    x0, x1 = int(bb[0]), int(bb[2])
    w_img = mask.shape[1]
    edge = max(2, int(0.01 * w_img)) if edge_px is None else int(edge_px)
    if x0 < edge or x1 > w_img - 1 - edge:                            # near the frame edge: a boundary may be outside the picture
        return None
    z = xyz[:, :, 2]
    valid = (mask > 0.5) & np.isfinite(z) & (z > 0.3) & (z < float(max_depth))
    sub = valid[:, x0:x1 + 1]
    cols = np.nonzero(sub.sum(axis=0) >= 3)[0]
    if len(cols) < min_cols:
        return None
    zc = np.array([np.median(z[sub[:, c], x0 + c]) for c in cols], dtype=np.float64)
    un = (cols + x0 + 0.5 - cam["cx"]) / cam["fx"]
    iz = 1.0 / zc
    keep = np.ones(len(cols), bool)
    coef = None
    for _ in range(3):
        if int(keep.sum()) < min_cols:
            return None
        coef = np.polyfit(un[keep], iz[keep], 1)
        r = np.abs(iz - np.polyval(coef, un))
        s = 1.4826 * float(np.median(r[keep])) + 1e-4
        keep = r <= 3.0 * s

    def pt(u_px: float):
        u = (u_px - cam["cx"]) / cam["fx"]
        inv = float(np.polyval(coef, u))
        if inv <= 1e-3:
            return None
        f = 1.0 / inv
        return f, u * f
    ins = cols[keep] + x0                                           # the columns on the face's plane: a box's visible side is not
    lft, rgt = pt(float(ins.min())), pt(float(ins.max() + 1))         # the outer pixel edges of the face
    if lft is None or rgt is None:
        return None
    mid = ((lft[0] + rgt[0]) / 2.0, (lft[1] + rgt[1]) / 2.0)

    def polar(p):
        return {"range_m": round(math.hypot(p[0], p[1]), 3), "bearing_deg": round(math.degrees(math.atan2(p[1], p[0])), 3),
                "forward_m": round(p[0], 3), "right_m": round(p[1], 3)}
    return {"left": polar(lft), "midpoint": polar(mid), "right": polar(rgt)}


def observe_boxes(fr: dict, dets, a, xyz: np.ndarray, bgr: np.ndarray, cam: dict) -> list:
    """v0.1.58 (WP13): the `box` detections of a frame -> records (cls box, range / bearing of the face midpoint, colour from the
    mask, face_points as face_keypoints). A box cut by the frame edge has no boundary: left out."""
    out = []
    w_img = fr["rgb"].shape[1]
    for d in (d for d in dets or [] if d.get("cls") == "box"):
        bb = _bbox(d["mask"])
        if bb is None or bb[0] <= 1 or bb[2] >= w_img - 2:
            continue
        kp = face_keypoints(xyz, d["mask"], cam, a.max_depth)
        if kp is None:
            continue
        colour, _ = colour_of(bgr, d["mask"])
        m = kp["midpoint"]
        out.append({"cls": "box", "confidence": round(d["conf"], 3), "range_m": m["range_m"], "bearing_deg": m["bearing_deg"],
                    "colour": colour, "face_points": kp, "truth_obj": d.get("obj"), "px": _px(d["mask"])})
    return out


def _dock_rec(cls: str, d: dict, p, colour: str, face) -> dict:
    fwd, right, down, n = p
    return {"cls": cls, "confidence": round(d["conf"], 3), "range_m": round(math.hypot(fwd, right), 3),
            "bearing_deg": round(math.degrees(math.atan2(right, fwd)), 2), "forward_m": round(fwd, 3), "right_m": round(right, 3),
            "down_m": round(down, 3), "colour": colour, "face": face, "valid_pixels": n, "truth_obj": d.get("obj"),
            "px": _px(d["mask"])}                                   # v0.1.54: the pixel box (frame_dets)


def observe(fr: dict, dets, a, dock_dets=None) -> dict:
    """One frame -> latest_status.json's payload (multi_core.frame_payload), exactly the buoys_multi rules; v0.1.48: + `dock` (the
    dock parts, observe_dock) when dock_dets is given."""
    meta = json.loads(str(fr["meta"]))
    cam, pose = meta["camera"], meta["pose"]
    depth = fr["depth"]
    if a.depth_noise == "zed":
        from optiprimex.sim import zed
        depth = zed.stereo_noise(depth, fr["ids"], cam["fx"], np.random.default_rng(meta["frame"]))
    xyz = point_cloud(depth, cam)
    bgr = fr["rgb"][:, :, ::-1]
    tel = {"lat": pose["lat"], "lon": pose["lon"], "yaw_rad": math.radians(pose["hdg_deg"]), "gps_fix": pose.get("gps_fix", 3),
           "position_age_s": 0.0, "attitude_age_s": 0.0, "source": "simview"}
    fresh = (tel["gps_fix"] or 0) >= 3
    ranged, bearings = [], []
    for d in dets:
        mask = d["mask"]
        meas = S.measure_mask(xyz, mask, max_depth_m=a.max_depth)
        if meas and M.near_limit(meas.range_m, a.max_depth, cam["fx"], a.limit_sigma):
            if d["conf"] >= a.bearing_conf:
                bearings.append((d, "near_limit"))
            continue
        if meas:
            c = M.centre_correction({f: getattr(meas, f) for f in ("range_m", "forward_m", "right_m", "down_m", "bearing_deg")}, a.buoy_radius)
            meas = meas.__class__(**{**{f: getattr(meas, f) for f in meas.__dataclass_fields__}, **{k: c[k] for k in ("range_m", "forward_m", "right_m", "down_m")}})
            ranged.append((meas.range_m, d, meas))
        elif d["conf"] >= a.bearing_conf:
            zs = depth[mask > 0.5]
            bearings.append((d, "depth_invalid" if int(np.isfinite(zs).sum()) >= 30 else "beyond_depth"))
    ranged.sort(key=lambda r: r[0])
    buoys = []
    for k, (_, d, meas) in enumerate(ranged[:max(1, a.max_buoys)]):
        colour, shares = colour_of(bgr, d["mask"])
        geo = S.buoy_geolocation(meas, tel) if fresh else None
        rec = {f: getattr(meas, f) for f in meas.__dataclass_fields__}
        rec.update(surface_range_m=round(meas.range_m - a.buoy_radius, 3), radius_m=a.buoy_radius, truth_obj=d.get("obj"))
        br = M.buoy_record(rec, d["conf"], colour, shares, geo, k)
        br["px"] = _px(d["mask"])                                   # v0.1.54: the pixel box (frame_dets)
        buoys.append(br)
    for d, why in sorted(bearings, key=lambda b: -b[0]["conf"])[:max(0, a.max_buoys - len(buoys))]:
        ys, xs = np.nonzero(d["mask"] > 0.5)
        if len(xs) == 0:
            continue
        u, v = M.mask_centre(xs.tolist(), ys.tolist())
        colour, shares = colour_of(bgr, d["mask"])
        rec = M.bearing_record(u, v, cam["fx"], cam["fy"], cam["cx"], cam["cy"], d["conf"], colour, shares, why, len(buoys))
        rec["truth_obj"] = d.get("obj")
        rec["px"] = [int(min(xs)), int(min(ys)), int(max(xs)), int(max(ys))]      # v0.1.54
        buoys.append(rec)
    utc = datetime.fromtimestamp(meta["t"], timezone.utc).isoformat()
    payload = M.frame_payload(utc, meta["frame"], buoys, tel)
    payload["observer"] = "sim_observer"
    payload["detector"] = a.detector
    if dock_dets is not None:
        payload["dock"] = observe_dock(fr, dock_dets, a, xyz, bgr, cam)
        payload["boxes"] = observe_boxes(fr, dock_dets, a, xyz, bgr, cam)     # v0.1.58: box faces with face_points (not published yet)
    payload["frame_dets"] = frame_dets(fr, meta, buoys, list(payload.get("dock") or []) + list(payload.get("boxes") or []), cam)   # v0.1.62: + boxes, kp     # v0.1.54 (item 11): the camera overlays' boxes
    if a.detector == "model" and a.model:                       # v0.1.49: cv_publish forwards which model in perception/health
        payload["model"] = os.path.basename(os.path.normpath(a.model))
        try:
            import yaml
            names = (yaml.safe_load(open(os.path.join(a.model, "classes.yaml"))) or {}).get("names") or {}
            payload["classes"] = [str(names[k]) for k in sorted(names, key=lambda x: int(x))]
        except Exception:
            pass
    return payload


def cannon_inputs(fr: dict, a):
    """v0.1.61: the frame as the ZED gives it to the AGX observers: BGR and the point cloud (the same depth noise as observe())."""
    meta = json.loads(str(fr["meta"]))
    depth = fr["depth"]
    if a.depth_noise == "zed":
        from optiprimex.sim import zed
        depth = zed.stereo_noise(depth, fr["ids"], meta["camera"]["fx"], np.random.default_rng(meta["frame"]))
    return np.ascontiguousarray(fr["rgb"][:, :, ::-1]), point_cloud(depth, meta["camera"])


def colour_of(bgr: np.ndarray, mask: np.ndarray, sat_min: float = 0.35, min_share: float = 0.45):
    ys, xs = np.nonzero(mask > 0.5)
    if len(ys) == 0:
        return "unknown", {"n": 0}
    idx = M.sample_indices(len(ys))
    return M.classify_bgr(bgr[ys[idx], xs[idx]].tolist(), sat_min=sat_min, min_share=min_share)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", default=os.environ.get("SIM_URL", "http://127.0.0.1:8097"))
    p.add_argument("--detector", default="synthetic", help="synthetic | model (with --model DIR)")
    p.add_argument("--model", default=None, help="a CV_TRAIN model folder (models/<name>/) for --detector model")
    p.add_argument("--depth-noise", choices=("zed", "none"), default="zed")
    p.add_argument("--size", default="1104x621", help="the observer's frame, WxH (default: half the AGX observer's hd2k)")
    p.add_argument("--hz", type=float, default=4.0)
    p.add_argument("--output", default=str(CV.parent / "logs" / "sim_observer"), help="runs/<stamp>/ go here (logs/: never archived)")
    p.add_argument("--max-depth", type=float, default=25.0)
    p.add_argument("--limit-sigma", type=float, default=2.0)
    p.add_argument("--bearing-conf", type=float, default=0.5)
    p.add_argument("--buoy-radius", type=float, default=M.buoy_radius())
    p.add_argument("--max-buoys", type=int, default=12)
    p.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run)")
    p.add_argument("--dock", choices=("auto", "on", "off"), default="auto", help="v0.1.48: the dock parts (dock_face, window, indicator) "
                   "too - auto = on with --detector model, off with synthetic (run_sim.sh keeps the geometric dock then)")
    p.add_argument("--cannon", choices=("red", "green"), default=None, help="v0.1.61: also measure the cannon's target window of this "
                   "colour on the rendered frame (../cannon/aim_cv.py) into --cannon-out every frame")
    p.add_argument("--cannon-out", default=None, help="v0.1.61: the target file (default: cv/cannon/cannon_config.json target_file)")
    p.add_argument("--device", default=None, help="v0.1.48: the model's device (cpu | mps | 0); default: Ultralytics' choice")
    p.add_argument("--guard", choices=("on", "off"), default="on", help="v0.1.80 (WP36): the ZED guard (cv/buoys_multi/guard_core.py) on the "
                   "rendered depth -> latest_status.json `guard` (cv_publish -> perception/guard); the obstacles are rendered, never buoys")
    a = p.parse_args(argv)
    if a.detector not in ("synthetic", "model") or (a.detector == "model" and not a.model):
        p.error("--detector synthetic | model --model DIR")
    a.w, a.h = (int(x) for x in a.size.lower().split("x"))
    a.dock_on = a.dock == "on" or (a.dock == "auto" and a.detector == "model")
    return a


def guard_cloud(fr: dict):
    """v0.1.80: the frame's cloud for the guard (point_cloud, the ZED's MEASURE.XYZ) with no return where nothing was hit (water / sky);
    the sim camera is level (no roll / pitch)."""
    xyz = point_cloud(np.asarray(fr["depth"], np.float32), json.loads(str(fr["meta"]))["camera"]).copy()
    xyz[np.asarray(fr["ids"]) < 0] = np.nan
    return xyz


def main(argv=None) -> int:
    a = parse_args(argv)
    gfilt = GC.GuardFilter() if a.guard == "on" else None
    det = synthetic_detections if a.detector == "synthetic" else ModelDetector(a.model, a.device)
    run_dir = Path(a.output) / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    fields = ["utc", "frame", "index", "colour", "confidence", "range_m", "surface_range_m", "reason", "bearing_deg", "truth_obj", "status"]
    csv_f = (run_dir / "buoy_observations.csv").open("w", newline="")
    wr = csv.DictWriter(csv_f, fieldnames=fields, extrasaction="ignore")
    wr.writeheader()
    print(f"[sim_observer] {a.sim} -> {run_dir} · detector {a.detector}{' ' + a.model if a.model else ''} · depth {a.depth_noise} · {a.w}x{a.h} at {a.hz} Hz"
          f" · dock {'on' if a.dock_on else 'off'}", flush=True)
    n, period, last_err = 0, 1.0 / max(0.2, a.hz), 0.0
    cannon = None
    if a.cannon:                                                     # v0.1.61: the cannon's target from the rendered camera
        sys.path.insert(1, str(CV / "cannon"))
        import aim_cv
        cannon = aim_cv.TargetWriter(a.cannon, a.cannon_out, source="sim_observer", max_depth_m=a.max_depth)
        print(f"[sim_observer] cannon target: {a.cannon} -> {cannon.path}", flush=True)
    try:
        while not a.frames or n < a.frames:
            t0 = time.monotonic()
            try:
                fr = fetch(a.sim, a.w, a.h)
            except (OSError, ValueError) as exc:
                if time.monotonic() - last_err > 10:
                    print(f"[sim_observer] no frame from {a.sim}: {exc}", flush=True)
                    last_err = time.monotonic()
                time.sleep(1.0)
                continue
            dets = det(fr) if callable(det) else det
            dock = None
            if a.dock_on:
                dock = det.last_dock if isinstance(det, ModelDetector) else synthetic_dock_detections(fr)
            payload = observe(fr, dets, a, dock)
            if gfilt is not None:                                       # v0.1.80 (WP36): the real guard filter on the rendered depth
                gfilt.c["h_cam_m"] = float(json.loads(str(fr["meta"]))["camera"].get("height_m", 0.8))
                payload["guard"] = dict(gfilt.frame(guard_cloud(fr), None, time.time()), source="sim_observer")
            if cannon is not None:
                cannon.frame(*cannon_inputs(fr, a))
            for b in payload["buoys"]:
                wr.writerow(dict(b, utc=payload["updated_utc"], frame=payload["frame"], status=payload["status"]))
            csv_f.flush()
            tmp = run_dir / "latest_status.json.tmp"
            tmp.write_text(json.dumps(payload, indent=1, default=float))
            os.replace(tmp, run_dir / "latest_status.json")
            if n % 20 == 0:
                print(f"frame={payload['frame']} status={payload['status']} buoys={payload['n_buoys']} " + " ".join(
                    f"{b['colour']}@{b['range_m']:.1f}m/{b['bearing_deg']:+.0f}" if b.get("range_m") is not None
                    else f"{b['colour']}@ray/{b['bearing_deg']:+.0f}" for b in payload["buoys"][:8]), flush=True)
            n += 1
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        if cannon is not None:
            cannon.stopped()
        csv_f.close()
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(CV.parent))                       # optiprimex (the depth model) from the kit's root
    raise SystemExit(main())
