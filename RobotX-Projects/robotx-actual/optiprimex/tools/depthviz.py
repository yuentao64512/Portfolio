"""depthviz: a depth map (metres along the optical axis, inf / nan = nothing) -> a colour picture for the CAMERA tab (v0.1.60, WP10m).

One scale everywhere (the simulator here, cam_stream on the AGX with the same stops): near = red, far = blue, nothing = black,
linear from NEAR_M to the far limit (default FAR_M, the ZED's useful range on the water). numpy only, so the simulator, the tests
and the OCS can share it; cam_stream carries its own copy of the stops (it runs in the students' venv, without optiprimex).
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np

NEAR_M = 0.5
FAR_M = 25.0
# (fraction, (r, g, b)) - a turbo-like ramp: near red -> yellow -> green -> cyan -> far blue
STOPS: Sequence[Tuple[float, Tuple[int, int, int]]] = ((0.0, (220, 40, 30)), (0.25, (250, 200, 40)), (0.5, (70, 200, 90)),
                                                      (0.75, (40, 180, 220)), (1.0, (40, 60, 200)))


def colourise(depth: np.ndarray, far_m: float = FAR_M, near_m: float = NEAR_M) -> np.ndarray:
    """(h, w) float -> (h, w, 3) uint8. Beyond far_m is drawn as far (blue); no return (inf / nan / <= 0) is black."""
    d = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(d) & (d > 0)
    f = np.clip((np.where(valid, d, far_m) - near_m) / max(1e-6, far_m - near_m), 0.0, 1.0)
    xs = np.array([s[0] for s in STOPS], np.float32)
    out = np.zeros(d.shape + (3,), np.uint8)
    for c in range(3):
        out[..., c] = np.interp(f, xs, np.array([s[1][c] for s in STOPS], np.float32)).astype(np.uint8)
    out[~valid] = 0
    return out


def depth_at(depth: Optional[np.ndarray], x: float, y: float) -> Optional[float]:
    """The range (m) at the normalised picture position (x, y in 0..1, from the top left); None outside or where nothing returns.
    The median of a 5 x 5 patch, so one noisy pixel does not decide."""
    if depth is None or not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
        return None
    h, w = depth.shape[:2]
    u, v = min(w - 1, int(x * w)), min(h - 1, int(y * h))
    patch = np.asarray(depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3], np.float32)
    ok = patch[np.isfinite(patch) & (patch > 0)]
    if ok.size == 0:
        return None
    r = float(np.median(ok))
    return None if not math.isfinite(r) else round(r, 2)


def scale_bar(far_m: float = FAR_M, near_m: float = NEAR_M) -> dict:
    """What the page draws beside a depth picture: the ends and the colour stops (CSS gradient)."""
    return {"near_m": near_m, "far_m": far_m, "stops": [[f, "#%02x%02x%02x" % c] for f, c in STOPS]}
