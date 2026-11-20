"""guard_core: the ZED proximity guard's filter (v0.1.80, WP36; docs/SCOUT_BRIEF_2026-11-11.md §4) - pure, numpy only.

Our observer (cv/buoys_multi, the process that holds the ZED - Dmitrii 2026-11-11: hook into OUR observer, never the students' snapshot)
hands each frame's point cloud (the ZED's XYZ, camera frame: x right, y down, z forward - the observer opens the camera in IMAGE
coordinates) and the ZED IMU's orientation at the image time. The cloud is levelled with the IMU's roll / pitch (a fixed pixel band
dips into the glitter or lifts into the sky at 10 deg of roll) - the measured down axis rotated onto the frame's (Rodrigues); then a point is an obstacle candidate when
  height above the water in 0.15 .. 1.5 m (the camera h_cam_m above the water: h = h_cam - down_level),
  horizontal range < slow_m (3.0), bearing within +- bearing_deg (35) of the bow;
the candidates count as pixels of the full frame (a subsampled grid, each point = step x step pixels) - a cluster of >= min_px (400)
- and STOP needs `frames` (3) consecutive frames with the nearest (5th percentile) range < stop_m (1.5). The message carries the
cluster's bearing extent (a pontoon is a wall, not a point). Nothing here needs the detector.

v0.1.81.1 (Dmitrii 2026-11-11: "if we see something, we put a 2-3 m circle behind that area"): `far` - the solid things the same
levelled cloud shows between slow_m and map_m (8 m) ahead, within +- map_bearing_deg - where a low object is still in the frame (the
frame's lower edge meets the water 3.7 m ahead at 1.0 m): the candidates binned by bearing (map_bin_deg), neighbouring bins with ranges
within map_gap_m joined into one cluster, each >= map_min_px pixels -> {range_m, bearing_deg, bearing_min_deg, bearing_max_deg, px}, the
nearest first, at most map_max. The boat maps them (usv/guard.map_far: a 2 m ring centred 1 m behind the seen face, skipped where a
mapped buoy / dock / box explains it); the guard's STOP logic does not read `far`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

DEFAULTS = {"stop_m": 1.5, "slow_m": 3.0, "min_px": 400, "frames": 3, "h_cam_m": 1.0, "h_min_m": 0.15, "h_max_m": 1.5,
            "bearing_deg": 35.0, "step": 4,
            "map_m": 8.0, "map_bearing_deg": 50.0, "map_min_px": 150, "map_bin_deg": 2.0, "map_gap_m": 0.8, "map_max": 8}


def far_clusters(rng: np.ndarray, brg: np.ndarray, k: int, c: dict) -> list:
    """v0.1.81.1: the candidates between slow_m and map_m (their ranges / bearings) -> clusters by bearing, nearest first."""
    if not len(rng) or float(c.get("map_m") or 0) <= float(c["slow_m"]):
        return []
    w = float(c["map_bin_deg"])
    bins = np.floor(brg / w).astype(int)
    order = np.argsort(bins, kind="stable")
    bins, rng, brg = bins[order], rng[order], brg[order]
    keys, starts = np.unique(bins, return_index=True)
    cells = []                                                   # (bin, near range, n, b min, b max)
    for j, b0 in enumerate(keys):
        sl = slice(starts[j], starts[j + 1] if j + 1 < len(starts) else len(bins))
        r = rng[sl]
        cells.append([int(b0), float(np.percentile(r, 5)), int(len(r)), float(brg[sl].min()), float(brg[sl].max())])
    out, cur = [], None
    for cell in cells:
        if cur is not None and cell[0] == cur["last"] + 1 and abs(cell[1] - cur["r_last"]) <= float(c["map_gap_m"]):
            cur.update(last=cell[0], r_last=cell[1], n=cur["n"] + cell[2], r=min(cur["r"], cell[1]), b1=cell[4],
                       nb=cur["nb"] + [(cell[1], (cell[3] + cell[4]) / 2.0)])
            continue
        if cur is not None:
            out.append(cur)
        cur = {"last": cell[0], "r_last": cell[1], "n": cell[2], "r": cell[1], "b0": cell[3], "b1": cell[4], "nb": [(cell[1], (cell[3] + cell[4]) / 2.0)]}
    if cur is not None:
        out.append(cur)
    res = []
    for q in out:
        px = q["n"] * k * k
        if px < int(c["map_min_px"]):
            continue
        b_near = min(q["nb"])[1]
        res.append({"range_m": round(q["r"], 2), "bearing_deg": round(b_near, 1), "bearing_min_deg": round(q["b0"], 1),
                    "bearing_max_deg": round(q["b1"], 1), "px": int(px)})
    return sorted(res, key=lambda d: d["range_m"])[: int(c["map_max"])]


def gravity_down(q) -> np.ndarray:
    """The ZED IMU's orientation quaternion (ox, oy, oz, ow) - the camera's rotation into the gravity-aligned world, IMAGE frame (x right,
    y down, z forward) - -> the world's down axis seen in the camera frame (R^T e_y), a unit vector."""
    x, y, z, w = (float(v) for v in q)
    return np.array([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], np.float64)


def roll_pitch(g: np.ndarray) -> Tuple[float, float]:
    """(roll, pitch) in radians for the display: roll = the down axis leaning sideways, pitch = leaning along the optical axis."""
    return math.atan2(-g[0], g[1]), math.atan2(g[2], math.hypot(g[0], g[1]))


def quat_to_roll_pitch(q) -> Tuple[float, float]:
    return roll_pitch(gravity_down(q))


def level_by(xyz: np.ndarray, g: np.ndarray) -> np.ndarray:
    """(N, 3) camera points -> the gravity-aligned frame: the shortest rotation taking the measured down axis g onto (0, 1, 0)
    (Rodrigues) - no Euler order, the heading about the vertical untouched."""
    g = g / (np.linalg.norm(g) or 1.0)
    e = np.array([0.0, 1.0, 0.0])
    v = np.cross(g, e)
    s, c = float(np.linalg.norm(v)), float(np.dot(g, e))
    if s < 1e-9:
        return xyz.copy() if c > 0 else xyz * np.array([1.0, -1.0, -1.0])
    k = v / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    R = np.eye(3) + s * K + (1 - c) * (K @ K)
    return (R @ xyz.T).T


def level(xyz: np.ndarray, roll: float, pitch: float) -> np.ndarray:
    """Kept for callers with angles: the down axis from (roll, pitch), then level_by."""
    g = np.array([-math.sin(roll) * math.cos(pitch), math.cos(roll) * math.cos(pitch), math.sin(pitch)])
    return level_by(xyz, g)


@dataclass
class GuardState:
    n_stop: int = 0                                           # consecutive frames with something inside stop_m
    last: Optional[dict] = None


class GuardFilter:
    def __init__(self, cfg: Optional[dict] = None):
        self.c = dict(DEFAULTS)
        self.c.update({k: v for k, v in (cfg or {}).items() if k in DEFAULTS and v is not None})
        self.st = GuardState()

    def frame(self, xyz: np.ndarray, quat=None, t: float = 0.0) -> dict:
        """One frame -> the guard message ({clear: true} or the nearest cluster with its extent)."""
        c = self.c
        k = max(1, int(c["step"]))
        pts = np.asarray(xyz)[::k, ::k, :3].reshape(-1, 3).astype(np.float32)
        pts = pts[np.isfinite(pts).all(axis=1)]
        g = gravity_down(quat) if quat is not None else np.array([0.0, 1.0, 0.0])
        roll, pitch = roll_pitch(g)
        msg = {"t": t, "clear": True, "frames": 0, "h_m": c["h_cam_m"], "roll_deg": round(math.degrees(roll), 1),
               "pitch_deg": round(math.degrees(pitch), 1)}
        if len(pts):
            L = level_by(pts, g)
            h = float(c["h_cam_m"]) - L[:, 1]                    # above the water (y is down)
            rng = np.hypot(L[:, 0], L[:, 2])
            brg = np.degrees(np.arctan2(L[:, 0], L[:, 2]))
            sel = (h >= c["h_min_m"]) & (h <= c["h_max_m"]) & (L[:, 2] > 0) & (rng < c["slow_m"]) & (np.abs(brg) <= c["bearing_deg"])
            n_px = int(sel.sum()) * k * k
            if n_px >= c["min_px"]:
                r, b = rng[sel], brg[sel]
                near = float(np.percentile(r, 5))
                i = int(np.argmin(np.abs(r - near)))
                msg.update(clear=False, range_m=round(near, 2), bearing_deg=round(float(b[i]), 1), bearing_min_deg=round(float(b.min()), 1),
                           bearing_max_deg=round(float(b.max()), 1), px=n_px, zone="stop" if near < c["stop_m"] else "slow")
            fsel = (h >= c["h_min_m"]) & (h <= c["h_max_m"]) & (L[:, 2] > 0) & (rng >= c["slow_m"]) & (rng < float(c["map_m"])) & \
                (np.abs(brg) <= float(c["map_bearing_deg"]))
            far = far_clusters(rng[fsel], brg[fsel], k, c)               # v0.1.81.1: what to map 3-8 m ahead (the boat rings it)
            if far:
                msg["far"] = far
        if not msg["clear"] and msg["zone"] == "stop":
            self.st.n_stop += 1
        else:
            self.st.n_stop = 0
        msg["frames"] = self.st.n_stop
        if not msg["clear"] and msg["zone"] == "stop" and self.st.n_stop < int(c["frames"]):
            msg["zone"] = "slow"                                 # not yet `frames` in a row: SLOW until it persists
        self.st.last = msg
        return msg
