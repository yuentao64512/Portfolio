"""dataset: synthetic training frames from our renderer (v0.1.33, ROADMAP WP10b; the first input of CV_TRAIN, ROADMAP 7b and its
request docs/REQUEST_dataset_export.md in ../CV_TRAIN).

Random poses inside a world's course (its `bounds`; +-30 m without one), most facing an object, ~10 % facing none (negatives); a few
frames per pose (the boat creeping on, the lights re-drawn) that share a stem prefix so a split can keep them together. Rendered by
render.py with the per-pixel object ids and a domain jitter (sun direction, water tint, haze, gain, noise, JPEG quality). Written in
the layout Ultralytics opens as it is (no converter):

  DIR/data.yaml                 path, train / val, names {0: buoy, 1: dock_face, 2: window, 3: box} and the thresholds (comments)
  DIR/images/STEM.jpg           the camera picture (1280 x 720 by default, the ZED's own intrinsics scaled: 50.9 deg, sim/zed.py)
  DIR/labels/STEM.txt           one line per instance: `class x1 y1 x2 y2 ...` - the outer contour of its visible mask, normalised
                                (YOLO segment format; a detector trains from it too)
  DIR/masks/STEM.png            16-bit instance mask: 0 = nothing, k = truth `objects[k-1]` (exact, per pixel)
  DIR/depth/STEM.npz            `depth`: float16 metres along the optical axis (inf = no return), as the ZED reports it; exact,
                                or with --depth-noise zed the stereo model of sim/zed.py (range noise ~ z^2, holes, flying pixels)
  DIR/truth/STEM.json           the pose (x_n, y_e, hdg, pitch, height), the intrinsics (w, h, hfov, fx, fy, cx, cy), per instance: world id, kind,
                                class, colour, lit, k (its id in the mask), visible pixels and the visible fraction, bbox, the true
                                forward / right / range / bearing from the camera - the field metric's reference
  DIR/manifest.json             the command, seed, worlds (sha256), versions, counts, sha256 of the file list
STEM = <world>_s<seed>_p<pose>_f<frame>. `--layout split` puts them in images/{train,val}/ ... instead, split by pose (never
a pose in both). An instance smaller than 12 px or less than 20 % visible (occluded; by a solo render) is not labelled.

Classes (`--classes kind`, the default = CV_TRAIN's option A, decided 2026-11-05): buoy, dock_face, window, box - the colour and the light
come from the truth (and the kit's colour vote); the beacon is part of its buoy. `--classes colour` = option B's colour classes.
v0.1.46 (WP10j, docs/DOCK_MOCKUP_SPEC.md): the Task 3 structure is drawn (the white cube, windows 1 upper-left / 2 lower-right, the
RED / GREEN indicator); `--scene dock` (the approach / docked / passing bands) and `--scene mix` (half dock, half random poses) label
five classes - buoy, dock_face, window, box, indicator - with the light state per window in the truth; `--scene buoys` (the default)
stays the four classes of v0.1.33.

  python -m optiprimex.sim.render --dataset DIR --n 2000 --world marina [--world pool] [--seed 1] [--frames-per-pose 3] [--size 1280x720]
         [--scene buoys|dock|mix]
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import random
import sys
import time
from collections import deque
from typing import Dict, List, Optional, Tuple

import numpy as np

from . import render as R
from . import world as W

KIT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CLASSES_KIND = ["buoy", "dock_face", "window", "box"]
CLASSES_COLOUR = ["buoy_red", "buoy_green", "buoy_blue", "buoy_black", "buoy_yellow", "buoy_white", "dock_face_red",
                  "dock_face_green", "dock_face_white", "window", "box"]
KIND_OF_TAG = {"buoy": "buoy", "light": "buoy", "dock": "dock_face", "box": "box", "window": "window", "indicator": "indicator"}
CLASSES_KIND5 = CLASSES_KIND + ["indicator"]               # v0.1.46 (WP10j): --scene dock | mix (decided 2026-11-06, DOCK_MOCKUP_SPEC 3)
CLASSES_COLOUR5 = CLASSES_COLOUR + ["indicator_red", "indicator_green"]
SCENES = ("buoys", "dock", "mix")
DOCK_BANDS = (("approach", 0.5), ("docked", 0.3), ("passing", 0.2))     # the spec's 50 / 30 / 20 %
MIN_PX = 12                      # an instance smaller than this is not labelled (CV_TRAIN's request)
MIN_VIS = 0.2                    # nor one less than 20 % visible (occluded: its pixels / its pixels rendered alone)
MAX_POLY = 120                   # outline points per object, at most


def world_path(name: str) -> str:
    if name in ("empty", "bench", "pool", "por", "boxes", "marina"):
        return os.path.join(KIT, "profiles", "bench", f"world_{name}.yaml")
    return name


def class_names(scheme: str, scene: str = "buoys") -> List[str]:
    """The names of an export: four (five with `indicator`) by kind, or the colour classes; the dock scenes add the indicator."""
    if scheme == "kind":
        return CLASSES_KIND if scene == "buoys" else CLASSES_KIND5
    return CLASSES_COLOUR if scene == "buoys" else CLASSES_COLOUR5


def class_id(kind: str, colour: str, scheme: str, names: Optional[List[str]] = None) -> Optional[int]:
    names = names or (CLASSES_KIND if scheme == "kind" else CLASSES_COLOUR)
    if scheme == "kind":
        return names.index(kind) if kind in names else None
    if kind in ("window", "box"):
        return names.index(kind)
    name = f"{kind}_{colour}"
    return names.index(name) if name in names else None


# ---- poses -----------------------------------------------------------------------------------------------------------
def inside(poly: List[Tuple[float, float]], x: float, y: float) -> bool:
    c = False
    for i in range(len(poly)):
        (x1, y1), (x2, y2) = poly[i], poly[(i - 1) % len(poly)]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def edge_dist(poly: List[Tuple[float, float]], x: float, y: float) -> float:
    best = float("inf")
    for i in range(len(poly)):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % len(poly)]
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey or 1e-9
        s = max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / L2))
        best = min(best, math.hypot(x - ax - s * ex, y - ay - s * ey))
    return best


CLEAR_M = 2.0                    # the camera stays this far from every object (a buoy filling the frame teaches nothing)


def near_object(objs: List[dict], x: float, y: float) -> bool:
    return any(math.hypot(x - float(o["x_n"]), y - float(o["y_e"])) < CLEAR_M for o in objs)


def sample_pose(world: dict, rng: random.Random, hfov_deg: float, aim_p: float = 0.75) -> Tuple[float, float, float]:
    """A point inside the bounds (0.5 m off the walls, CLEAR_M off every object) and a heading: toward a random object +- 0.4 hfov with
    probability aim_p, else any."""
    b = [(float(p[0]), float(p[1])) for p in (world.get("bounds") or [])]
    if len(b) < 3:
        b = [(-30.0, -30.0), (30.0, -30.0), (30.0, 30.0), (-30.0, 30.0)]
    objs = [o for o in world.get("objects") or [] if o.get("kind") in ("buoy", "dock", "box")]
    xs, ys = [p[0] for p in b], [p[1] for p in b]
    for _ in range(500):
        x, y = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
        if not inside(b, x, y) or edge_dist(b, x, y) < 0.5:
            continue
        if near_object(objs, x, y):
            continue
        break
    if objs and rng.random() < aim_p:
        o = rng.choice(objs)
        hdg = math.degrees(math.atan2(float(o["y_e"]) - y, float(o["x_n"]) - x)) + rng.uniform(-0.4, 0.4) * hfov_deg
    else:
        hdg = rng.uniform(0.0, 360.0)
    return x, y, hdg % 360.0


def negative_heading(world: dict, x: float, y: float, hfov_deg: float, rng: random.Random) -> Optional[float]:
    """A heading with no object within the field of view (+ 8 deg) - a negative frame; None when every direction sees one."""
    objs = [o for o in world.get("objects") or [] if o.get("kind") in ("buoy", "dock", "box")]
    brg = [math.degrees(math.atan2(float(o["y_e"]) - y, float(o["x_n"]) - x)) for o in objs]
    free = [h for h in range(0, 360, 5) if all(abs((h - b + 180) % 360 - 180) > hfov_deg / 2 + 8 for b in brg)]
    return float(rng.choice(free)) + rng.uniform(-2.0, 2.0) if free else None


def random_style(rng: random.Random) -> dict:
    """The domain jitter of one pose (CV_TRAIN's request: the gap is what will be measured): the sun's azimuth / elevation (the
    shading), the water's tint, the haze toward the horizon (sky and far water), the walls' grey."""
    az, el = math.radians(rng.uniform(0, 360)), math.radians(rng.uniform(15, 75))
    tint = np.array([rng.uniform(-12, 12), rng.uniform(-18, 18), rng.uniform(-25, 20)], np.float32)
    haze = rng.uniform(-14, 10)
    return {"light": [math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)],
            "water": (R._D_WATER + tint).clip(0, 255).tolist(), "water_far": (R._D_WATER_FAR + tint * 0.5 + haze).clip(0, 255).tolist(),
            "sky_top": (R._D_SKY_TOP + haze).clip(0, 255).tolist(), "sky_horizon": (R._D_SKY_HORIZON + haze * 0.6).clip(0, 255).tolist(),
            "wall": (R._D_WALL + rng.uniform(-20, 15)).clip(0, 255).tolist()}


def random_lit(world: dict, rng: random.Random) -> Dict[int, bool]:
    """Flashing lights on or off at random (half the time each), steady on, off off."""
    return {o.get("id"): (rng.random() < 0.5 if o.get("light") == "flash" else o.get("light") == "steady")
            for o in world.get("objects") or [] if o.get("kind") == "buoy"}


# ---- v0.1.46 (WP10j): the dock scenes (docs/DOCK_MOCKUP_SPEC.md 3) -------------------------------------------------------
def sample_dock_pose(world: dict, rng: random.Random, hfov_deg: float) -> Tuple[float, float, float, str, Optional[int]]:
    """A pose looking at the dock in one of the spec's bands: approach (3-15 m from a bay's face, within +-25 deg of its normal,
    heading at the face +-20 deg - m3t1's standoff 5 m and abeam ~9 m), docked (1.2-2.5 m straight on +-15 deg, +-0.4 m across - the
    stop at 1.5 m), passing (20-30 m from the dock's middle, +-70 deg off its normal, heading at it +-25 deg). Inside the bounds, 1 m
    off every buoy; a world without a dock -> sample_pose ("random"). Returns x, y, heading, the band, the bay it looks at."""
    objs = world.get("objects") or []
    faces = W.bay_faces(objs)
    if not faces:
        x, y, h = sample_pose(world, rng, hfov_deg)
        return x, y, h, "random", None
    u = rng.random()
    band = "approach" if u < DOCK_BANDS[0][1] else ("docked" if u < DOCK_BANDS[0][1] + DOCK_BANDS[1][1] else "passing")
    bay = rng.choice(sorted(faces))
    face = faces[bay]
    fx, fy = float(face["x_n"]), float(face["y_e"])
    mx = sum(float(f["x_n"]) for f in faces.values()) / len(faces)
    my = sum(float(f["y_e"]) for f in faces.values()) / len(faces)
    h = R.face_heading(world, face)
    b = [(float(p[0]), float(p[1])) for p in (world.get("bounds") or [])]
    buoys = [o for o in objs if o.get("kind") == "buoy"]
    x = y = hdg = 0.0
    for _ in range(200):
        if band == "approach":
            rho, phi = rng.uniform(3.0, 15.0), math.radians(h + rng.uniform(-25.0, 25.0))
            x, y = fx + rho * math.cos(phi), fy + rho * math.sin(phi)
            hdg = math.degrees(math.atan2(fy - y, fx - x)) + rng.uniform(-20.0, 20.0)
        elif band == "docked":
            rho, lat, hr = rng.uniform(1.2, 2.5), rng.uniform(-0.4, 0.4), math.radians(h)
            x, y = fx + rho * math.cos(hr) + lat * math.sin(hr), fy + rho * math.sin(hr) - lat * math.cos(hr)
            hdg = h + 180.0 + rng.uniform(-15.0, 15.0)
        else:
            rho, phi = rng.uniform(20.0, 30.0), math.radians(h + rng.uniform(-70.0, 70.0))
            x, y = mx + rho * math.cos(phi), my + rho * math.sin(phi)
            hdg = math.degrees(math.atan2(my - y, mx - x)) + rng.uniform(-25.0, 25.0)
        if len(b) >= 3 and (not inside(b, x, y) or edge_dist(b, x, y) < 0.5):
            continue
        if any(math.hypot(x - float(o["x_n"]), y - float(o["y_e"])) < 1.0 for o in buoys):
            continue
        break
    return x, y, hdg % 360.0, band, bay


def random_dock_style(rng: random.Random, world: dict) -> dict:
    """The dock's jitter for one pose (the spec): side S 0.8-1.2 m (the bays 1.0 / 0.9 / 1.1 S), yaw +-10 deg per bay, border 3-4 cm,
    the dark pane's tint, the panels' whiteness, the tubes, the planks, the light intensity and its bloom."""
    bays = sorted(W.bay_faces(world.get("objects") or []))
    wt = rng.uniform(222, 250)
    tint = [rng.uniform(-6, 6) for _ in range(3)]
    return {"side_m": round(rng.uniform(0.8, 1.2), 3), "yaw_deg": {b: round(rng.uniform(-10.0, 10.0), 2) for b in bays},
            "border_m": round(rng.uniform(0.03, 0.04), 4),
            "pane": [int(np.clip(v + rng.uniform(-18, 14), 0, 255)) for v in R.PANE_OFF],
            "white": [int(np.clip(wt + t, 0, 255)) for t in tint], "tube": [int(np.clip(wt - rng.uniform(5, 25) + t, 0, 255)) for t in tint],
            "plank": [int(np.clip(v + d, 0, 255)) for v, d in zip(R.PLANK, [rng.uniform(-25, 25)] * 2 + [rng.uniform(-30, 20)])],
            "light_gain": round(rng.uniform(0.8, 1.2), 3), "bloom": round(rng.uniform(0.3, 1.0), 3)}


def random_dock_state(rng: random.Random, green_bay: Optional[int]) -> dict:
    """A sim/dock/state-like body for one frame: GREEN on exactly one bay, the target window off 40 %, RED 30 %, GREEN 20 %, the
    Advanced flash 10 % (red / green / blue, on or off by phase) - always in the GREEN bay, window 1 or 2 at random."""
    u = rng.random()
    wn = rng.choice([1, 2])
    if u < 0.4:
        win = {"bay": None, "window": None, "colour": "black"}
        phase = "idle"
    elif u < 0.7:
        win, phase = {"bay": green_bay, "window": wn, "colour": "red"}, "lit"
    elif u < 0.9:
        win, phase = {"bay": green_bay, "window": wn, "colour": "green"}, "green"
    else:
        win = {"bay": green_bay, "window": wn, "colour": rng.choice(["red", "green", "blue"]), "flash": True, "on": rng.random() < 0.5}
        phase = "flash"
    return {"green_bay": green_bay, "phase": phase, "window": win}


# ---- outlines ----------------------------------------------------------------------------------------------------------
_DIRS = [(0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1)]      # W NW N NE E SE S SW (rows down)


def largest_component(m: np.ndarray) -> np.ndarray:
    """The largest 8-connected component of a boolean mask (a buoy in front can split a box face in two). v0.1.46: by runs (a row's
    horizontal runs joined to the touching runs of the row above, union-find) - the same answer as the pixel walk below, which
    took ~1 s on a dock face filling the frame."""
    H, Wd = m.shape
    if H == 0 or Wd == 0 or not m.any():
        return np.zeros_like(m, bool)
    pad = np.zeros((H, Wd + 2), np.int8)
    pad[:, 1:-1] = m
    dif = np.diff(pad, axis=1)
    sr, sc = np.nonzero(dif == 1)
    er, ec = np.nonzero(dif == -1)                          # runs [sc, ec) per row, in raster order (both lists sorted alike)
    parent = list(range(len(sr)))

    def find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    rows = np.searchsorted(sr, np.arange(H + 1))
    for r in range(1, H):
        a0, a1, b0, b1 = rows[r - 1], rows[r], rows[r], rows[r + 1]
        i = a0
        for j in range(b0, b1):                              # runs of row r against those of row r-1 (8-connected: +-1 column)
            while i < a1 and ec[i] < sc[j]:
                i += 1
            k = i
            while k < a1 and sc[k] <= ec[j]:
                ra, rb = find(int(k)), find(int(j))
                if ra != rb:
                    parent[max(ra, rb)] = min(ra, rb)
                k += 1
    roots = np.array([find(i) for i in range(len(sr))])
    size = np.bincount(roots, weights=(ec - sc), minlength=len(sr))
    best = int(np.argmax(size))                             # ties: the lowest root = the first in raster order, as the walk did
    out = np.zeros_like(m, bool)
    for i in np.flatnonzero(roots == best):
        out[sr[i], sc[i]:ec[i]] = True
    return out


def _largest_component_walk(m: np.ndarray) -> np.ndarray:
    """v0.1.33's pixel walk (kept as the reference of the tests)."""
    seen = np.zeros_like(m, bool)
    best: List[Tuple[int, int]] = []
    H, Wd = m.shape
    for r0, c0 in zip(*np.nonzero(m)):
        if seen[r0, c0]:
            continue
        comp, q = [], deque([(int(r0), int(c0))])
        seen[r0, c0] = True
        while q:
            r, c = q.popleft()
            comp.append((r, c))
            for dr, dc in _DIRS:
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < Wd and m[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    q.append((rr, cc))
        if len(comp) > len(best):
            best = comp
            if len(best) * 2 > int(m.sum()):
                break
    out = np.zeros_like(m, bool)
    if best:
        rr, cc = zip(*best)
        out[list(rr), list(cc)] = True
    return out


def outline(m: np.ndarray) -> List[Tuple[int, int]]:
    """Moore-neighbour boundary trace of a single-component mask -> (row, col) pixels clockwise, from the top-left pixel."""
    pad = np.pad(m, 1)
    rs, cs = np.nonzero(pad)
    if len(rs) == 0:
        return []
    k = int(np.lexsort((cs, rs))[0])
    start = (int(rs[k]), int(cs[k]))
    pts = [start]
    cur, back = start, 0                                    # we came from the west (raster order: the west pixel is empty)
    first_next = None
    for _ in range(4 * pad.size):
        found = None
        for i in range(1, 9):
            d = (back + i) % 8
            n = (cur[0] + _DIRS[d][0], cur[1] + _DIRS[d][1])
            if pad[n]:
                p = (cur[0] + _DIRS[(back + i - 1) % 8][0], cur[1] + _DIRS[(back + i - 1) % 8][1])
                found = (n, _DIRS.index((p[0] - n[0], p[1] - n[1])))
                break
        if found is None:                                   # a single pixel
            break
        n, back = found
        if cur == start and first_next is not None and n == first_next:
            break
        if first_next is None:
            first_next = n
        cur = n
        if cur == start:
            continue
        pts.append(cur)
    return [(r - 1, c - 1) for r, c in pts]


def polygon(m: np.ndarray, x0: int, y0: int, w: int, h: int) -> List[float]:
    """The normalised outline of a mask crop (offset x0, y0 in a w x h frame), at most MAX_POLY points, pixel corners included
    so a one-pixel-wide object still has an area."""
    pts = outline(largest_component(m))
    if len(pts) < 3:
        r, c = (pts[0] if pts else (0, 0))
        pts = [(r, c), (r, c + 1), (r + 1, c + 1), (r + 1, c)]
        off = 0.0
    else:
        off = 0.5
    step = max(1, math.ceil(len(pts) / MAX_POLY))
    out: List[float] = []
    for r, c in pts[::step]:
        out += [min(1.0, max(0.0, (x0 + c + off) / w)), min(1.0, max(0.0, (y0 + r + off) / h))]
    return out


# ---- one frame ---------------------------------------------------------------------------------------------------------
def visible_fraction(scene: R.Scene, prims: List[int], visible_px: int, pose: tuple, cam: R.Camera, k: int = 4) -> float:
    """Visible pixels / the object's pixels rendered alone (no other object, no wall), at 1/k resolution - the occlusion. An object
    too small for the coarse render counts as fully visible."""
    solo = R.Scene(spheres=[sp for i, sp in enumerate(scene.spheres) if i in prims],
                   boxes=[bx for j, bx in enumerate(scene.boxes) if R.BOX_ID0 + j in prims])
    sw, sh = max(16, cam.w // k), max(9, cam.h // k)
    small = R.Camera(w=sw, h=sh, height_m=cam.height_m, max_range_m=cam.max_range_m, fx_px=cam.fx * sw / cam.w, fy_px=cam.fy * sh / cam.h,
                     cx_px=cam.cx * sw / cam.w, cy_px=cam.cy * sh / cam.h)
    x, y, hdg, pitch, z = pose
    _, _, pid = R.render_ids(solo, x, y, hdg, small, pitch_deg=pitch, z_m=z)
    alone = int((pid >= 0).sum()) * (cam.w / small.w) * (cam.h / small.h)
    return 1.0 if alone < 4 * (cam.w / small.w) * (cam.h / small.h) else min(1.0, visible_px / alone)


def frame(world: dict, rng: random.Random, cam: R.Camera, scheme: str, pose: tuple, lit: Dict[int, bool], style: Optional[dict],
          jitter: bool, hfov_in: Optional[float] = None, depth_noise: Optional[str] = None, dock_state: Optional[dict] = None,
          dock_style: Optional[dict] = None, names: Optional[List[str]] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray, dict, List[str]]:
    x, y, hdg, pitch, z = pose
    scene = R.scene_from_world(world, lit=lit, dock_state=dock_state, dock_style=dock_style)
    rgb, depth, pid = R.render_ids(scene, x, y, hdg, cam, pitch_deg=pitch, z_m=z, style=style)
    if depth_noise == "zed":                                   # v0.1.34: a ZED-like depth (sim/zed.py stereo_noise), not the exact one
        from . import zed
        depth = zed.stereo_noise(depth, pid, cam.fx, np.random.default_rng(rng.randrange(1 << 30)))
    gain = rng.uniform(0.8, 1.15) if jitter else 1.0
    noise = rng.uniform(1.0, 5.0) if jitter else 0.0
    if jitter:
        img = rgb.astype(np.float32) * gain + np.random.default_rng(rng.randrange(1 << 30)).normal(0.0, noise, rgb.shape)
        rgb = img.clip(0, 255).astype(np.uint8)
    by_obj: Dict[Tuple[str, str], List[int]] = {}                  # (kind, id) -> its primitives (v0.1.46: a dock's parts apart)
    raw: Dict[Tuple[str, str], str] = {}
    for i, sp in enumerate(scene.spheres):
        tk, oid = sp.tag.split(":", 1)
        by_obj.setdefault((KIND_OF_TAG.get(tk, tk), oid), []).append(i)
    for j, bx in enumerate(scene.boxes):
        tk, oid = bx.tag.split(":", 1)
        by_obj.setdefault((KIND_OF_TAG.get(tk, tk), oid), []).append(R.BOX_ID0 + j)
        raw[(KIND_OF_TAG.get(tk, tk), oid)] = tk
    children: Dict[str, List[int]] = {}                            # a dock face's windows and indicator (its visible fraction)
    for key, prims in by_obj.items():
        part = scene.parts.get(f"{raw.get(key, '')}:{key[1]}")
        if part and key[0] in ("window", "indicator"):
            children.setdefault(str(part["dock_id"]), []).extend(prims)
    kinds = {str(o.get("id")): o for o in world.get("objects") or []}
    inst = np.zeros(pid.shape, np.uint16)
    objects, lines, skipped = [], [], []
    fwd, right, _ = cam.basis(hdg, pitch)
    names = names or (CLASSES_KIND if scheme == "kind" else CLASSES_COLOUR)
    for (kind, oid), prims in by_obj.items():
        m = np.isin(pid, prims)
        area = int(m.sum())
        part = scene.parts.get(f"{raw.get((kind, oid), '')}:{oid}")
        o = part if part is not None else kinds.get(oid)
        if o is None or area == 0 or kind not in ("buoy", "dock_face", "window", "box", "indicator"):
            continue
        if part is not None:
            colour = str(part.get("indicator_colour") if kind == "dock_face" else part.get("colour") or "black")
        else:
            colour = str(o.get("colour") or "white")               # the light's colour (v0.1.36: a dark `body` is reported apart)
        cid = class_id(kind, colour, scheme, names)
        if cid is None:
            if kind == "indicator":                                # the four-class (buoys) export: seen, not a class there
                skipped.append({"id": o.get("id"), "kind": kind, "area_px": area, "why": "no indicator class in the buoys scene"})
            continue
        if kind == "dock_face" and children.get(oid):
            allp = prims + children[oid]
            vis = visible_fraction(scene, allp, int(np.isin(pid, allp).sum()), pose, cam)
        else:
            vis = visible_fraction(scene, prims, area, pose, cam)
        if area < MIN_PX or vis < MIN_VIS:
            skipped.append({"id": o.get("id"), "kind": kind, "area_px": area, "visible_frac": round(vis, 3)})
            continue
        rr, cc = np.nonzero(m)
        y0, y1, x0, x1 = int(rr.min()), int(rr.max()), int(cc.min()), int(cc.max())
        poly = polygon(m[y0:y1 + 1, x0:x1 + 1], x0, y0, cam.w, cam.h)
        k = len(objects) + 1
        inst[m] = k
        cx_, cy_ = (part["centre"][0], part["centre"][1]) if part is not None else (float(o["x_n"]), float(o["y_e"]))
        rel = np.array([cx_ - x, cy_ - y, 0.0], np.float32)
        f_m, r_m = float(rel @ fwd) / max(1e-6, math.cos(math.radians(pitch))), float(rel @ right)
        d = depth[m]
        d = d[np.isfinite(d)]
        objects.append({"k": k, "id": o.get("id"), "kind": kind, "class_id": cid, "class": names[cid],
                        "colour": colour, "body": o.get("body"), "light": o.get("light"), "lit": bool(lit.get(o.get("id"), False)) if kind == "buoy" else None,
                        "bay": o.get("bay"), "bbox": [x0, y0, x1 + 1, y1 + 1], "area_px": area, "visible_frac": round(vis, 3),
                        "truth": {"forward_m": round(f_m, 3), "right_m": round(r_m, 3), "range_m": round(math.hypot(f_m, r_m), 3),
                                  "bearing_deg": round(math.degrees(math.atan2(r_m, f_m)), 2),
                                  "depth_median_m": round(float(np.median(d)), 3) if d.size else None}})
        if part is not None:                                       # v0.1.46: the dock's truth (the spec's list)
            hr = math.radians(part["face_hdg_deg"])
            vx, vy = x - part["centre"][0], y - part["centre"][1]
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (vx * math.cos(hr) + vy * math.sin(hr)) / max(1e-6, math.hypot(vx, vy))))))
            objects[-1].update({"colour": colour, "light": part.get("light"), "lit": part.get("lit"), "window": part.get("window"),
                                "dock_id": part["dock_id"], "indicator_colour": part.get("indicator_colour"), "side_m": part["side_m"],
                                "face_hdg_deg": part["face_hdg_deg"], "face_angle_deg": round(ang, 2), "height_m": round(part["centre"][2], 3)})
        lines.append(f"{cid} " + " ".join(f"{v:.5f}" for v in poly))
    meta = {"pose": {"x_n": round(x, 3), "y_e": round(y, 3), "hdg_deg": round(hdg, 2), "z_m": round(z, 3), "pitch_deg": round(pitch, 2)},
            "camera": {"w": cam.w, "h": cam.h, "hfov_deg": round(cam.hfov_deg, 3), "fx": round(cam.fx, 3), "fy": round(cam.fy, 3), "cx": round(cam.cx, 3), "cy": round(cam.cy, 3),
                       "model": "zed2i SN33545355 left hd2k, scaled" if hfov_in is None else "pinhole",
                       "max_range_m": cam.max_range_m, "depth": "along the optical axis, metres"},
            "jitter": {"gain": round(gain, 3), "noise_sigma": round(noise, 2), "style": style}, "world": world.get("name"),
            "objects": objects, "not_labelled": skipped}
    if dock_state is not None or dock_style is not None:
        meta["dock"] = {"state": dock_state, "style": dock_style}
    return rgb, inst, depth, meta, lines


def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def export(out: str, n: int, worlds: List[str], seed: int = 1, val: float = 0.2, scheme: str = "kind", w: int = 1280, h: int = 720,
           hfov: Optional[float] = None, jitter: bool = True, quality: int = 90, layout: str = "flat", frames_per_pose: int = 3,
           negatives: float = 0.1, depth_fmt: str = "npz", depth_noise: Optional[str] = None, scene: str = "buoys", log=print) -> dict:
    """n frames in all, frames_per_pose at a time (the boat creeping on 0.3-0.6 m between them, the lights re-drawn). layout flat =
    images/STEM.jpg (the consumer splits by the stem's pose); split = images/{train,val}/ by pose with `val` of the poses in val.
    scene (v0.1.46, WP10j): buoys = the random poses of v0.1.33, four classes (unchanged, the default); dock = the dock bands
    (sample_dock_pose), the dock's jitter and random light states, five classes (+ indicator); mix = half dock poses, half random
    poses (with the dock's jitter and lights too), five classes - CV_TRAIN's combined set."""
    from PIL import Image
    if os.path.isdir(out) and os.listdir(out):
        raise SystemExit(f"{out} is not empty - pick a new directory (a dataset is never overwritten)")
    if layout not in ("flat", "split") or depth_fmt not in ("npz", "png", "none") or scene not in SCENES:
        raise SystemExit("layout: flat | split; depth: npz | png | none; scene: buoys | dock | mix")
    rng = random.Random(seed)
    cam = R.Camera(w=w, h=h, hfov_deg=hfov)
    loaded = [(p, W.load(world_path(p))) for p in worlds]
    names = class_names(scheme, scene)
    bands: Dict[str, int] = {}
    splits = ("train", "val") if layout == "split" else ("all",)
    counts = {s: {c: 0 for c in names} for s in splits}
    frames = {s: 0 for s in splits}
    n_neg = 0
    files: List[str] = []
    subs = ("images", "labels", "masks", "truth") + (("depth",) if depth_fmt != "none" else ())
    for sub in subs:
        for s in splits:
            os.makedirs(os.path.join(out, sub, s) if layout == "split" else os.path.join(out, sub), exist_ok=True)
    t0 = time.time()
    i, pose_no = 0, 0
    while i < n:
        wname, world = loaded[pose_no % len(loaded)]
        split = ("val" if rng.random() < val else "train") if layout == "split" else "all"
        bay = None
        if scene == "buoys":
            x, y, hdg = sample_pose(world, rng, cam.hfov_deg)
            band = "random"
        elif scene == "dock" or rng.random() < 0.5:
            x, y, hdg, band, bay = sample_dock_pose(world, rng, cam.hfov_deg)
        else:
            x, y, hdg = sample_pose(world, rng, cam.hfov_deg)
            band = "random"
        want_neg = jitter is not None and rng.random() < negatives
        if want_neg:
            if scene != "buoys":                                     # a dock pose faces the dock: a negative from a random place
                x, y, hdg = sample_pose(world, rng, cam.hfov_deg)
                band = "negative"
            nh = negative_heading(world, x, y, cam.hfov_deg, rng)
            hdg = nh if nh is not None else hdg
        d_style, green = None, None
        if scene != "buoys":
            faces = sorted(W.bay_faces(world.get("objects") or []))
            d_style = random_dock_style(rng, world) if jitter else None
            green = rng.choice(faces) if faces else None
            if bay is not None and rng.random() < 0.7:               # mostly the bay in view is the GREEN one (m3t1 docks there)
                green = bay
        z = rng.uniform(0.6, 1.0) if jitter else cam.height_m
        pitch = rng.uniform(-4.0, 4.0) if jitter else 0.0
        style = random_style(rng) if jitter else None
        q = rng.randint(70, 95) if jitter else quality
        for f in range(frames_per_pose):
            if i >= n:
                break
            if f:
                step = rng.uniform(0.3, 0.6)
                nx, ny = x + step * math.cos(math.radians(hdg)), y + step * math.sin(math.radians(hdg))
                if not near_object([o for o in world.get("objects") or [] if o.get("kind") in ("buoy", "dock", "box")], nx, ny):
                    x, y = nx, ny                                    # else it holds its place (the heading still drifts)
                hdg, pitch = hdg + rng.uniform(-2.0, 2.0), pitch + (rng.uniform(-1.0, 1.0) if jitter else 0.0)
            lit = random_lit(world, rng)
            d_state = random_dock_state(rng, green) if scene != "buoys" else None
            rgb, inst, depth, meta, lines = frame(world, rng, cam, scheme, (x, y, hdg, pitch, z), lit, style, jitter, hfov, depth_noise,
                                                  dock_state=d_state, dock_style=d_style, names=names)
            stem = f"{world.get('name', 'world')}_s{seed}_p{pose_no:05d}_f{f:02d}"
            meta.update({"stem": stem, "split": split, "index": i, "pose_group": pose_no, "frame_in_group": f, "seed": seed,
                         "jpeg_quality": q, "negative": not lines, "scene": scene, "band": band})
            d = f"{split}/" if layout == "split" else ""
            rel = {"images": f"images/{d}{stem}.jpg", "labels": f"labels/{d}{stem}.txt", "masks": f"masks/{d}{stem}.png",
                   "truth": f"truth/{d}{stem}.json"}
            Image.fromarray(rgb).save(os.path.join(out, rel["images"]), format="JPEG", quality=q)
            with open(os.path.join(out, rel["labels"]), "w") as fh:
                fh.write("".join(ln + "\n" for ln in lines))
            Image.fromarray(inst).save(os.path.join(out, rel["masks"]))
            if depth_fmt == "npz":
                rel["depth"] = f"depth/{d}{stem}.npz"
                np.savez_compressed(os.path.join(out, rel["depth"]), depth=depth.astype(np.float16))
            elif depth_fmt == "png":
                rel["depth"] = f"depth/{d}{stem}.png"
                mm = np.where(np.isfinite(depth), np.clip(depth * 1000.0, 0, 65535), 0).astype(np.uint16)
                Image.fromarray(mm).save(os.path.join(out, rel["depth"]))
            with open(os.path.join(out, rel["truth"]), "w") as fh:
                json.dump(meta, fh, indent=1)
            files += rel.values()
            frames[split] += 1
            n_neg += 0 if lines else 1
            bands[band] = bands.get(band, 0) + 1
            for o in meta["objects"]:
                counts[split][o["class"]] += 1
            i += 1
            if i % 50 == 0 or i == n:
                log(f"dataset: {i}/{n} frames ({time.time() - t0:.0f} s)")
        pose_no += 1
    tv = ("images/train", "images/val") if layout == "split" else ("images", "images")
    with open(os.path.join(out, "data.yaml"), "w") as fh:
        fh.write(f"# synthetic frames from optiprimex.sim.render (MASTER_SCRIPTS); see manifest.json and README.md\n"
                 f"# labelled: instances >= {MIN_PX} px and >= {int(MIN_VIS * 100)} % visible (occlusion by a solo render); YOLO segment polygons\n"
                 + ("# flat layout: train and val both point at images/ - split by the stem's pose (<world>_s<seed>_p<pose>_f<frame>)\n" if layout == "flat" else "")
                 + f"path: {os.path.abspath(out)}\ntrain: {tv[0]}\nval: {tv[1]}\nnames:\n" + "".join(f"  {k}: {v}\n" for k, v in enumerate(names)))
    from .. import __version__
    import PIL
    man = {"what": "synthetic frames, optiprimex.sim.render --dataset (ROADMAP WP10b; CV_TRAIN docs/REQUEST_dataset_export.md)",
           "kit_version": __version__, "command": " ".join([os.path.basename(sys.executable), "-m", "optiprimex.sim.render"] + sys.argv[1:]),
           "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "seed": seed, "n": n, "layout": layout, "val_fraction": val,
           "frames_per_pose": frames_per_pose, "poses": pose_no, "scene": scene, "bands": bands, "negatives_wanted": negatives, "negative_frames": n_neg,
           "classes": scheme, "names": names, "camera": {"w": w, "h": h, "hfov_deg": round(cam.hfov_deg, 3), "fx": round(cam.fx, 3), "cx": round(cam.cx, 3), "cy": round(cam.cy, 3),
                      "model": "zed2i SN33545355 left hd2k, scaled (sim/zed.py)" if hfov is None else "pinhole"}, "jitter": jitter, "depth": depth_fmt, "depth_noise": depth_noise or "exact",
           "thresholds": {"min_px": MIN_PX, "min_visible_frac": MIN_VIS},
           "worlds": [{"arg": p, "file": os.path.relpath(world_path(p), KIT) if world_path(p).startswith(KIT) else world_path(p),
                       "sha256": sha256(world_path(p))} for p, _ in loaded],
           "versions": {"python": sys.version.split()[0], "numpy": np.__version__, "pillow": PIL.__version__},
           "frames": frames, "instances": counts,
           "files_sha256": hashlib.sha256("\n".join(sorted(files)).encode()).hexdigest(),
           "notes": ["labels: YOLO segment polygons (the largest visible part); masks: exact, 0 = none, k = truth objects[k-1]",
                     "depth: npz `depth` float16 metres along the optical axis, inf = no return (png: uint16 mm, 0 = none)",
                     "the Task 3 structure (v0.1.46): dock_face = the front panel + its frame tubes; window = border + pane, one each "
                     "(truth: window 1 upper-left / 2 lower-right, light off|red|green|flash, lit); indicator = the base cylinder "
                     "(truth: indicator_colour); --scene buoys keeps the four v0.1.33 classes (an indicator seen there is in not_labelled)",
                     "flashing lights are on or off at random per frame; in the dock scenes the dock's light state is drawn per frame",
                     "truth not_labelled: instances seen but under the thresholds (they are in the picture, not in the labels)"]}
    with open(os.path.join(out, "manifest.json"), "w") as fh:
        json.dump(man, fh, indent=1)
    with open(os.path.join(out, "README.md"), "w") as fh:
        fh.write(__doc__.split("\n\n", 1)[1].split("\n  python -m")[0] + "\n")
    log(f"dataset: {out} - {sum(frames.values())} frames ({n_neg} negative) from {pose_no} poses; instances "
        + ", ".join(f"{c} {sum(counts[s][c] for s in splits)}" for c in names))
    return man
