"""fake_cannon_target - the cannon's target in the GEOMETRIC simulator (v0.1.61, WP12-rest): the stand-in for an observer's
`--cannon COLOUR` when no camera runs (run_bench.sh, run_sim.sh --perception geometric, the fast SIL).

On the boat the target is measured (cv/cannon/aim_cv.py on the ZED frame); here it is computed from the world's truth: the lit
window of the dock (sim/dock/state: bay, window, colour) where the RENDERER draws it (render.dock_structure's `window:<id>` part:
the mock-up's place on the face and its height), seen from the boat's pose (local/<v>/mm/state), in the cannon frame (camera =
cannon pivot, at the camera's height above the water). Only inside the camera's field of view (HFOV_DEG, the ZED's 51 deg) and the
window range (8 m), like the geometric perception. The record is aim_cv's (aim_core.target_record), so the sender and the receiver
cannot tell the two apart - `source: truth` in the file says it. Measured on rendered frames (v0.1.61): the camera path agrees
with this to a few cm.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import List, Optional, Tuple

KIT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(KIT / "cv" / "cannon"))
import aim_core as A  # noqa: E402

HFOV_DEG = 51.0
MAX_RANGE_M = 8.0
CAMERA_HEIGHT_M = 0.8                   # render.Camera.height_m: the ZED (= the cannon pivot here) above the water
_SCENES: dict = {}


def window_centre(world: dict, bay: int, win: int, dock_state: dict) -> Optional[Tuple[float, float, float]]:
    """(x_n, y_e, z) of the window as the renderer draws it (cached per world and lit window)."""
    from . import render as R
    key = (id(world), bay, win, (dock_state.get("window") or {}).get("colour"))
    parts = _SCENES.get(key)
    if parts is None:
        parts = R.scene_from_world(world, dock_state=dock_state).parts
        _SCENES.clear()
        _SCENES[key] = parts
    for part in parts.values():
        if part.get("kind") == "window" and int(part.get("bay", -1)) == int(bay) and int(part.get("window", -1)) == int(win):
            return tuple(float(v) for v in part["centre"])
    return None


def lit_window(dock_state: Optional[dict]) -> Tuple[Optional[int], Optional[int], Optional[str]]:
    w = (dock_state or {}).get("window") or {}
    colour = w.get("colour")
    return w.get("bay"), w.get("window"), colour if colour in A.COLOURS else None


def truth_target(world: dict, pose: Optional[dict], dock_state: Optional[dict], colour: str = "red",
                 max_range_m: float = MAX_RANGE_M, hfov_deg: float = HFOV_DEG) -> Tuple[str, Optional[dict]]:
    """-> (status, target) with aim_cv's words: OK | NO_<COLOUR>_TARGET."""
    none = (f"NO_{colour.upper()}_TARGET", None)
    bay, win, lit = lit_window(dock_state)
    if pose is None or bay is None or win is None or lit != colour:
        return none
    c = window_centre(world, int(bay), int(win), dock_state)
    if c is None:
        return none
    dn, de = c[0] - float(pose["x_n"]), c[1] - float(pose["y_e"])
    h = math.radians(float(pose["hdg_deg"]))
    forward = dn * math.cos(h) + de * math.sin(h)
    right = -dn * math.sin(h) + de * math.cos(h)
    coords = A.cannon_coordinates(forward, right, -(c[2] - CAMERA_HEIGHT_M), A.MOUNT0)
    if forward <= 0.3 or coords["range_m"] > max_range_m or abs(coords["bearing_deg"]) > hfov_deg / 2.0:
        return none
    return "OK", A.target_record(colour, 0.80, 1500, [0, 0, 0, 0], coords, source="truth", bay_truth=int(bay), window_truth=int(win))
