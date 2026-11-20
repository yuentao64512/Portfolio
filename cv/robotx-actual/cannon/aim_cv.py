"""aim_cv - the cannon target from one frame: colour segmentation + depth, PORTED from the students' color_dock_sender.py.

Their sender (snapshot beside this file, sha256 5e23b699..., not run by us) opens the ZED itself; our observers already hold the
camera, so its per-frame steps live here as functions the observers call on the frame and point cloud they already have
(decided with Dmitrii 2026-11-07: one camera owner). What is theirs, kept to the line: green_mask (now `colour_mask`, the colour a
parameter), green_components (`colour_components`), measure_mask (`measure_component`, their variant: MAD depth gate, spread
check), the nearest-candidate choice and the confidence rule. Ours: the colour presets (aim_core.HSV), find_target's wrapper, the
`status` words for the target file.

xyz: the ZED's MEASURE.XYZ(RGBA) in the IMAGE coordinate system (x right, y down, z forward, metres) - the sim's point_cloud gives
the same. Needs numpy and cv2 (the AGX's venv, the kit's .venv on the Mac).
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import cv2
import numpy as np

try:
    from . import aim_core as A          # imported as a package (tests)
except ImportError:                      # run from cv/cannon or with cv/cannon on sys.path (the observers)
    import aim_core as A

MIN_COLOUR_PIXELS = 200                  # their max(200, detection.minimum_color_pixels)


def colour_mask(frame_bgr: np.ndarray, hsv_cfg: dict) -> np.ndarray:
    """Their green_mask with the band from hsv_cfg (two hue bands, saturation / value minimum, open 5 then close 11)."""
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    low1 = np.array([hsv_cfg["hue_low_1"], hsv_cfg["saturation_min"], hsv_cfg["value_min"]], np.uint8)
    high1 = np.array([hsv_cfg["hue_high_1"], 255, 255], np.uint8)
    low2 = np.array([hsv_cfg["hue_low_2"], hsv_cfg["saturation_min"], hsv_cfg["value_min"]], np.uint8)
    high2 = np.array([hsv_cfg["hue_high_2"], 255, 255], np.uint8)
    mask = cv2.bitwise_or(cv2.inRange(hsv, low1, high1), cv2.inRange(hsv, low2, high2))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((11, 11), np.uint8))


def colour_components(mask: np.ndarray, minimum_pixels: int):
    """Their green_components, verbatim."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    components = []
    for label in range(1, count):
        x, y, width, height, area = stats[label]
        if int(area) < int(minimum_pixels) or width < 8 or height < 8:
            continue
        component = np.zeros_like(mask)
        component[labels == label] = 255
        bbox_area = max(int(width * height), 1)
        components.append({"mask": component, "bbox": [int(x), int(y), int(x + width), int(y + height)],
                           "pixels": int(area), "fraction": float(area) / bbox_area})
    return components


def measure_component(xyz: np.ndarray, mask: np.ndarray, min_valid_fraction: float = 0.10, max_depth_m: float = 25.0) -> Optional[dict]:
    """Their measure_mask (color_dock_sender.py), verbatim but for the return: a dict instead of GreenMeasurement."""
    binary = (mask > 0.5).astype(np.uint8)
    if int(binary.sum()) < 80:
        return None
    inner = cv2.erode(binary, np.ones((5, 5), np.uint8), iterations=1)
    if int(inner.sum()) < 40:
        inner = binary
    valid = (inner.astype(bool) & np.isfinite(xyz[:, :, :3]).all(axis=2) & (xyz[:, :, 2] > 0.30) & (xyz[:, :, 2] < float(max_depth_m)))
    fraction = float(valid.sum() / max(int(inner.sum()), 1))
    points = xyz[:, :, :3][valid]
    if len(points) < 30 or fraction < float(min_valid_fraction):
        return None
    z = points[:, 2]
    median_z = float(np.median(z))
    mad = float(np.median(np.abs(z - median_z)))
    gate = max(0.08, 3.0 * 1.4826 * mad)
    points = points[np.abs(z - median_z) <= gate]
    if len(points) < 30:
        return None
    spread = float(np.percentile(points[:, 2], 90) - np.percentile(points[:, 2], 10))
    if spread > max(0.35, 0.18 * median_z):
        return None
    right, down, forward = np.median(points, axis=0)
    return {"range_m": float(np.linalg.norm([right, down, forward])), "forward_m": float(forward), "right_m": float(right),
            "down_m": float(down), "bearing_deg": math.degrees(math.atan2(float(right), float(forward))),
            "valid_pixels": int(len(points)), "valid_fraction": fraction, "depth_spread_m": spread}


def find_target(frame_bgr: np.ndarray, xyz: np.ndarray, colour: str = "red", cfg: Optional[dict] = None,
                max_depth_m: float = 25.0, min_valid_fraction: float = 0.10) -> Tuple[str, Optional[dict], int]:
    """One frame -> (status, target or None, components seen). Their loop body: every component of the colour measured with its
    (dilated 7x7) mask, the nearest kept. status: OK | NO_<COLOUR>_TARGET | <COLOUR>_NO_RELIABLE_DEPTH (their words, the colour
    named)."""
    hsv_cfg = A.hsv_for(colour, cfg)
    mount = A.mount_of(cfg)
    minimum = max(MIN_COLOUR_PIXELS, int((cfg or {}).get("minimum_color_pixels", MIN_COLOUR_PIXELS)))
    comps = colour_components(colour_mask(np.ascontiguousarray(frame_bgr), hsv_cfg), minimum)
    candidates = []
    for comp in comps:
        expanded = cv2.dilate(comp["mask"], np.ones((7, 7), np.uint8), iterations=1)
        meas = measure_component(xyz, expanded.astype(np.float32) / 255.0, min_valid_fraction, max_depth_m)
        if meas is None:
            continue
        coords = A.cannon_coordinates(meas["forward_m"], meas["right_m"], meas["down_m"], mount)
        candidates.append(A.target_record(colour, comp["fraction"], comp["pixels"], comp["bbox"], coords,
                                          valid_depth_pixels=meas["valid_pixels"], valid_depth_fraction=meas["valid_fraction"],
                                          depth_spread_m=meas["depth_spread_m"]))
    word = colour.upper()
    if candidates:
        return "OK", min(candidates, key=lambda t: t["range_m"]), len(comps)
    return (f"{word}_NO_RELIABLE_DEPTH" if comps else f"NO_{word}_TARGET"), None, len(comps)


# ---- the observers' hook: --cannon COLOUR [--cannon-out FILE] --------------------------------------------------------------
HERE = __import__("pathlib").Path(__file__).resolve().parent


def load_cfg(path=None) -> dict:
    """cannon_config.json (the HSV overrides, the mount, target_file). The key_file entry is a path; it is never opened here."""
    import json
    return json.loads((__import__("pathlib").Path(path) if path else HERE / "cannon_config.json").read_text())


class TargetWriter:
    """One per observer: frame(bgr, xyz) -> find_target -> the target file the sender reads (every frame, OK or not, so a
    target that leaves the view is gone at the next frame). An exception in the cannon part never stops the observer."""

    def __init__(self, colour: str, out=None, cfg: Optional[dict] = None, source: str = "observer", max_depth_m: float = 25.0):
        self.cfg = cfg if cfg is not None else load_cfg()
        self.colour, self.source, self.max_depth_m = colour, source, max_depth_m
        A.hsv_for(colour, self.cfg)                                 # refuses an unknown colour at start
        self.path = __import__("pathlib").Path(out) if out else (HERE / self.cfg.get("target_file", "runs/latest/cannon_target.json"))
        self.last: Optional[dict] = None
        self.errors = 0

    def frame(self, frame_bgr, xyz) -> dict:
        try:
            status, target, n = find_target(frame_bgr, xyz, self.colour, self.cfg, self.max_depth_m)
        except Exception as exc:                                    # noqa: BLE001 - the observer's own job goes on
            self.errors += 1
            status, target, n = f"CANNON_ERROR {type(exc).__name__}", None, 0
        self.last = A.write_target(self.path, status, target, self.colour, self.source)
        self.last["components"] = n
        return self.last

    def stopped(self) -> None:
        try:
            A.write_target(self.path, "OBSERVER_STOPPED", None, self.colour, self.source)
        except OSError:
            pass
