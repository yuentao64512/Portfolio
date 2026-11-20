"""multi_core: the pure part of OUR multi-buoy observer (v0.1.30, ROADMAP WP8c) - no OpenCV, no ZED, no numpy needed.

The students' actual_buoy_observer.py measures every buoy its model segments, then keeps the nearest (`min(candidates)`), and
its model has one class ("buoy"): no colour. Our observer (multi_buoy_observer.py, same measuring functions imported read-only)
reports every measured buoy of a frame; this module gives each one a colour from its own pixels and builds the frame's
latest_status.json, which tools cv/publish/cv_publish.py read (`buoys`: one item each; `measurement` = the nearest, as before).

Colour: the mask's pixels (a sample) in HSV; only saturated, lit ones vote (S >= 0.35, V >= 0.25), by hue: red < 15 deg or
> 340 deg, yellow 40-70, green 80-170, blue 190-260. A colour needs `min_share` of the voters and `min_votes` of them; few
saturated pixels = black (V low) or white (V high), else unknown; the winner must also hold 1.5x the runner-up's votes. Tunable at the pool: --min-share, --sat-min.

v0.1.34 (ROADMAP WP10g, CV_TRAIN's REQUEST_localisation.md):
- Range = surface + radius (`centre_correction`): the students' measure_mask returns the median VISIBLE SURFACE point, so every buoy
  read 0.22 m short in the simulator (CV_TRAIN measured it) - and the bias points at the boat, so one buoy seen from two sides lands
  up to ~0.45 m apart on the map. The point is pushed out along its own line of sight by SURFACE_FRACTION (0.85) x the buoy's radius
  (BUOY_RADIUS_M, per type; the RobotX 2026 handbook gives no RoboBuoy size - 0.25 m, the pathing spec's BUOY_R, until measured):
  +0.21 m; `surface_range_m` kept.
- Bearing-only (`bearing_record`): a confident detection that measure_mask refused (beyond its 20 m depth limit, or no valid depth)
  is still reported - its bearing from the mask's centre column through the camera's intrinsics, `range_m: null`, a reason. The
  perception map keeps it as a ray (never fused) and triangulates rays from several boat positions.
"""
from __future__ import annotations

import colorsys
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

HUES = (("red", 0.0, 15.0), ("yellow", 40.0, 70.0), ("green", 80.0, 170.0), ("blue", 190.0, 260.0), ("red", 340.0, 360.0))


def classify_bgr(pixels: Iterable[Sequence[float]], sat_min: float = 0.35, val_min: float = 0.25, min_share: float = 0.45,
                 min_votes: int = 12, min_sat_frac: float = 0.0) -> Tuple[str, Dict[str, float]]:
    """(colour, shares) for a buoy's pixels, each (b, g, r) in 0..255."""
    votes: Dict[str, int] = {}
    n = n_sat = n_dark = n_bright = 0
    for b, g, r in pixels:
        h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
        n += 1
        if v < val_min:
            n_dark += 1
            continue
        if s < sat_min:
            if v > 0.75:
                n_bright += 1
            continue
        n_sat += 1
        deg = h * 360.0
        for name, lo, hi in HUES:
            if lo <= deg < hi:
                votes[name] = votes.get(name, 0) + 1
                break
    shares = {k: round(c / max(n_sat, 1), 2) for k, c in votes.items()}
    shares.update(n=n, saturated=round(n_sat / max(n, 1), 2))
    # v0.1.93.2 (13 Nov replay, 805 masks): a colour only when >= min_sat_frac of ALL the pixels are saturated - the white markers in shade /
    # with the foam float showed 3-21 % bluish pixels and won "blue" on 12 of 400 (0 = the old rule)
    if n_sat >= min_votes and votes and n_sat >= min_sat_frac * max(n, 1):
        ranked = sorted(votes.items(), key=lambda kv: -kv[1])
        name, c = ranked[0]
        second = ranked[1][1] if len(ranked) > 1 else 0
        if c / n_sat >= min_share and c >= 1.5 * second:          # and clearly ahead of the runner-up (a mask spilling onto a neighbour)
            return name, shares
    if n and n_dark / n > 0.6:
        return "black", shares
    if n and n_bright / n > 0.6:
        return "white", shares
    return "unknown", shares


def sample_indices(n: int, k: int = 400) -> List[int]:
    """Evenly spread indices into n pixels (at most k)."""
    if n <= k:
        return list(range(n))
    step = n / float(k)
    return [int(i * step) for i in range(k)]


def frame_payload(utc: str, frame: int, buoys: List[dict], telemetry: dict) -> dict:
    """latest_status.json of one frame: every buoy (the measured ones nearest first, then the bearing-only ones), the nearest
    measured one as `measurement` (the single observer's key, so an older cv_publish still reads it), the status as the students'
    observer names it (BEARING_ONLY when nothing in the frame had a range)."""
    buoys = sorted(buoys, key=lambda b: (b.get("range_m") is None, b.get("range_m") or 0.0))
    ranged = [b for b in buoys if b.get("range_m") is not None]
    if not buoys:
        status = "NO_RELIABLE_BUOY"
    elif not ranged:
        status = "BEARING_ONLY"
    elif any(b.get("buoy_lat") is not None for b in ranged):
        status = "OK"
    else:
        status = "BUOY_DEPTH_OK_GPS_UNAVAILABLE"
    out = {"updated_utc": utc, "frame": frame, "status": status, "n_buoys": len(buoys), "n_bearing_only": len(buoys) - len(ranged), "buoys": buoys,
           "measurement": dict(ranged[0], utc=utc, frame=frame, status=status) if ranged else {"utc": utc, "frame": frame, "status": status},
           "telemetry": telemetry, "observer": "multi_buoy_observer"}
    return out


def buoy_record(measurement: dict, confidence: float, colour: str, shares: Dict[str, float], geo: Optional[dict], index: int) -> dict:
    rec = dict(measurement)
    rec.update(index=index, confidence=round(float(confidence), 3), colour=colour, colour_shares=shares)
    if geo:
        rec.update(buoy_lat=geo["latitude_deg"], buoy_lon=geo["longitude_deg"], north_offset_m=geo["north_offset_m"], east_offset_m=geo["east_offset_m"])
    return rec


# ---- v0.1.34: range = surface + radius; bearing-only ---------------------------------------------------------------------------
BUOY_RADIUS_M = {"default": 0.25}                   # per buoy type (the detector's class); the handbook gives no RoboBuoy size
SURFACE_FRACTION = 0.85     # the median of the visible (eroded) surface sits ~0.8-0.9 r in front of a sphere's centre, not r: CV_TRAIN
                            # measured -0.22 m with r 0.25, our render -0.20 (tests/test_wp10g.py) -> adding 0.85 r leaves both within 0.05 m


def buoy_radius(kind: Optional[str] = None, table: Optional[Dict[str, float]] = None) -> float:
    """What is added to the measured range: SURFACE_FRACTION x the buoy type's radius."""
    t = table or BUOY_RADIUS_M
    return round(SURFACE_FRACTION * float(t.get(str(kind or "default"), t.get("default", 0.25))), 4)


def centre_correction(measurement: dict, radius_m: float) -> dict:
    """The measured surface point pushed out by radius_m along its own line of sight (bearing and elevation unchanged): range,
    forward, right, down -> the buoy's centre; `surface_range_m` keeps what the camera measured."""
    out = dict(measurement)
    r = float(measurement.get("range_m") or 0.0)
    if r <= 0.0 or radius_m <= 0.0:
        return out
    k = (r + radius_m) / r
    for key in ("forward_m", "right_m", "down_m"):
        if measurement.get(key) is not None:
            out[key] = float(measurement[key]) * k
    out["range_m"] = r + radius_m
    out["surface_range_m"] = r
    out["radius_m"] = radius_m
    return out


def mask_centre(xs: Sequence[float], ys: Sequence[float]) -> Tuple[float, float]:
    """The mask's centre (median column, median row) - robust to a ragged edge."""
    sx, sy = sorted(xs), sorted(ys)
    return float(sx[len(sx) // 2]), float(sy[len(sy) // 2])


def bearing_record(u: float, v: float, fx: float, fy: float, cx: float, cy: float, confidence: float, colour: str,
                   shares: Dict[str, float], reason: str, index: int) -> dict:
    """A detection without a range: bearing (deg, + starboard) and elevation (deg, + up) of the mask's centre through the
    intrinsics; range_m None; reason beyond_depth | depth_invalid."""
    return {"index": index, "confidence": round(float(confidence), 3), "colour": colour, "colour_shares": shares,
            "range_m": None, "bearing_only": True, "reason": reason,
            "bearing_deg": round(math.degrees(math.atan2(u - cx, fx)), 3), "elev_deg": round(-math.degrees(math.atan2(v - cy, fy)), 3),
            "pixel": [round(u, 1), round(v, 1)]}


# ---- v0.1.34.2: CV_TRAIN's follow-up and its first model -------------------------------------------------------------------------
ZED_BASELINE_M = 0.11991847621653                   # optiprimex/sim/zed.py BASELINE_M (SN33545355) - copied: this module stays standalone


def sigma_z(range_m: float, fx_px: float, sigma_d_px: float = 0.25, baseline_m: float = ZED_BASELINE_M) -> float:
    """Stereo depth error at a range: z^2 sigma_d / (fx B) (the model of optiprimex/sim/zed.py)."""
    return range_m * range_m * sigma_d_px / max(fx_px * baseline_m, 1e-9)


def near_limit(range_m: float, max_depth_m: float, fx_px: float, k_sigma: float = 2.0, sigma_d_px: float = 0.25) -> bool:
    """A measured range within k sigma_z of the depth limit is not trusted as a range (CV_TRAIN 2026-11-05: buoys whose centre is
    beyond the limit get ranged only when the noise pulls their surface under it - those read ~0.6 m short); report a bearing."""
    if k_sigma <= 0 or max_depth_m <= 0:
        return False
    return range_m >= max_depth_m - k_sigma * sigma_z(range_m, fx_px, sigma_d_px)


def model_spec(model_dir: str, prefer_engine: bool = True) -> dict:
    """A CV_TRAIN model folder (models/<name>/: weights.engine / weights.pt, input.yaml, classes.yaml) -> what the observer needs:
    the weights file (the TensorRT engine where it exists and prefer_engine - it is built for that AGX - else the .pt), imgsz, conf,
    iou, the class ids that are buoys, the names."""
    import os
    import yaml
    with open(os.path.join(model_dir, "input.yaml"), encoding="utf-8") as f:
        inp = yaml.safe_load(f) or {}
    with open(os.path.join(model_dir, "classes.yaml"), encoding="utf-8") as f:
        cls = yaml.safe_load(f) or {}
    names = {int(k): str(v) for k, v in (cls.get("names") or {}).items()}
    eng, pt = os.path.join(model_dir, "weights.engine"), os.path.join(model_dir, "weights.pt")
    weights = eng if prefer_engine and os.path.isfile(eng) else pt
    if not os.path.isfile(weights):
        raise FileNotFoundError(f"{model_dir}: no weights.engine / weights.pt")
    return {"weights": weights, "imgsz": int(inp.get("imgsz", 768)), "conf": float(inp.get("conf_default", 0.5)),
            "iou": float(inp.get("iou_default", 0.7)), "buoy_ids": sorted(k for k, v in names.items() if v == "buoy"), "names": names,
            "name": os.path.basename(os.path.normpath(model_dir))}
