"""zed: the boat's camera as it really is - the ZED 2i's factory calibration, the one source of the intrinsics (v0.1.34, WP10g).

Copied 2026-11-05 from the AGX (read only): /usr/local/zed/settings/SN33545355.conf, the LEFT camera (the image the observers
segment), and [STEREO] Baseline. The observers run `--resolution hd2k` (2208 x 1242): fx 2317.2 -> a rectified horizontal field of
view of 50.9 deg (HD720: 57.8 deg). Everything that pretends to be this camera derives from here: the renderer (render.Camera()
default, simview, the dataset export: hd2k scaled to the render size), fake perception's field of view (a world's
`camera: {intrinsics: zed2i_hd2k}`), and the stereo depth model below. Before v0.1.34 the renderer used 100 deg and the world files
72 deg (CV_TRAIN's REQUEST_localisation.md, item 0). Distortion is not modelled (the ZED SDK rectifies before we see the image).

Stereo depth (item 3, `stereo_noise`): disparity d = fx * B / z, so a disparity error of sigma_d pixels gives sigma_z = z^2 * sigma_d
/ (fx * B) - the error grows with the square of the range. Plus holes where there is no texture to match (flat water, uniform paint;
a share of those pixels returns nothing) and "flying pixels" on the edge between a near and a far surface (depth smeared between
the two). The parameters are a guess to fit at the pool (docs/POOL_CALIBRATION.md); off unless asked for.
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

SERIAL = "SN33545355"
BASELINE_M = 0.11991847621653                      # [STEREO] Baseline 119.918 mm
# LEFT camera, per resolution: (width, height, fx, fy, cx, cy) - verbatim from the factory file
LEFT: Dict[str, Tuple[int, int, float, float, float, float]] = {
    "hd2k": (2208, 1242, 2317.220195126216, 2318.0692477562825, 1134.0962636377467, 597.9421012970183),
    "hd1080": (1920, 1080, 2317.220195126216, 2318.0692477562825, 990.09626363775, 516.94210129702),
    "hd720": (1280, 720, 1158.6100975631, 1159.0346238781, 653.54813181887, 346.97105064851),
    "vga": (672, 376, 579.30504878155, 579.51731193907, 342.27406590944, 180.98552532425),
}
OBSERVER_RESOLUTION = "hd2k"                       # cv/buoys*/ --resolution default
DEPTH_MAX_M = 25.0                                 # our observer's --max-depth (v0.1.34, decided 2026-11-05: the pathing spec's R for the
                                                   # 4 mm lens; the students' own default is 20 - their file unchanged, we pass the value)
INTRINSICS_NAMES = {"zed2i_hd2k": "hd2k", "zed2i_hd1080": "hd1080", "zed2i_hd720": "hd720", "zed2i_vga": "vga"}


def scaled(w: int, h: int, res: str = OBSERVER_RESOLUTION) -> Tuple[float, float, float, float]:
    """(fx, fy, cx, cy) of the `res` calibration scaled to a w x h picture (the same field of view, the same principal point
    relative to the frame)."""
    W, H, fx, fy, cx, cy = LEFT[res]
    sx, sy = w / float(W), h / float(H)
    return fx * sx, fy * sy, cx * sx, cy * sy


def hfov_deg(res: str = OBSERVER_RESOLUTION) -> float:
    W, _, fx, _, _, _ = LEFT[res]
    return math.degrees(2.0 * math.atan(W / 2.0 / fx))


def vfov_deg(res: str = OBSERVER_RESOLUTION) -> float:
    _, H, _, fy, _, _ = LEFT[res]
    return math.degrees(2.0 * math.atan(H / 2.0 / fy))


def resolution_of(name: Optional[str]) -> Optional[str]:
    """A world file's `camera.intrinsics` (zed2i_hd2k ...) -> the resolution key, None when not ours."""
    return INTRINSICS_NAMES.get(str(name or "").lower())


def bearing_deg(u: float, w: int, h: int, res: str = OBSERVER_RESOLUTION) -> float:
    """The bearing (deg, + starboard) of picture column u (pixels, a w x h picture of this camera)."""
    fx, _, cx, _ = scaled(w, h, res)
    return math.degrees(math.atan2(u - cx, fx))


# ---- item 3: ZED-like depth -------------------------------------------------------------------------------------------------
STEREO_DEFAULTS = {"sigma_d_px": 0.25, "hole_flat": 0.15, "hole_far_m": 35.0, "flying_px": 1, "max_m": 40.0}


def stereo_noise(depth, ids, fx: float, rng=None, params: Optional[dict] = None):
    """A ZED-like depth from an exact one (float32 metres, inf = none): range noise sigma_z = z^2 sigma_d / (fx B); holes on
    low-texture surfaces (ids < 0: water, sky, walls - `hole_flat` of them, more with range, all beyond `hole_far_m`); flying pixels
    on the edges between objects (`flying_px` wide: the depth mixed between the two sides); nothing beyond `max_m`.
    ids = render.render_ids()'s per-pixel primitive (>= 0 = an object). Returns a new float32 array."""
    import numpy as np
    p = dict(STEREO_DEFAULTS, **(params or {}))
    rng = rng if rng is not None else np.random.default_rng(0)
    z = depth.astype(np.float32).copy()
    fin = np.isfinite(z)
    sigma = np.where(fin, z * z * float(p["sigma_d_px"]) / (fx * BASELINE_M), 0.0).astype(np.float32)
    z[fin] += (rng.standard_normal(int(fin.sum())) * sigma[fin]).astype(np.float32)
    flat = fin & (ids < 0)
    frac = np.clip(float(p["hole_flat"]) * (1.0 + np.where(flat, z, 0.0) / max(1.0, float(p["hole_far_m"]))), 0.0, 1.0)
    holes = flat & ((rng.random(z.shape) < frac) | (z > float(p["hole_far_m"])))
    k = int(p["flying_px"])
    if k > 0:
        edge = np.zeros(z.shape, bool)
        edge[:, k:] |= ids[:, k:] != ids[:, :-k]
        edge[k:, :] |= ids[k:, :] != ids[:-k, :]
        edge &= fin
        left = np.roll(z, k, axis=1)
        mix = rng.random(z.shape).astype(np.float32)
        both = edge & np.isfinite(left)
        z[both] = mix[both] * z[both] + (1.0 - mix[both]) * left[both]
    z[holes] = np.inf
    z[z > float(p["max_m"])] = np.inf
    z[z <= 0.3] = np.inf
    return z
