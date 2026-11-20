"""two_stage_core (OURS, v0.1.78.18): the pure part of the Model A adapter in multi_buoy_observer (`--detector model-a`).

The colleague's Model A (cv/buoys/two_stage/PROVENANCE.md; weights read-only in ~/buoy_robotx/runtime/models/) is a DETECT model:
boxes, no masks, classes buoy_led_red / buoy_led_green / buoy_no_led / dock / home / obstacle. Our loop measures a buoy through the
students' measure_mask (a mask over the ZED point cloud). The adapter gives it one: the ellipse inscribed in the box, shrunk to
`shrink` of its size so the water and the sky at the box's corners and edges stay out of the depth median (`box_mask`).

Colour (`pick_colour`, --colour):
  auto   Model A's LED class when it is red / green and the box's confidence >= led_conf; otherwise (buoy_no_led, a low score, and
         every colour Model A has no class for: blue, yellow, black, white) our HSV vote on the same pixels (multi_core.classify_bgr)
  model  Model A's LED class only (red / green / "black" for buoy_no_led - an unlit buoy, cv_publish: lit = colour != black)
  hsv    our HSV vote for every buoy, whatever Model A says - THE BACKUP (Dmitrii 2026-11-10: "a backup option for colour")
Each record says where its colour came from (`colour_source`: model | hsv) and Model A's own class (`model_class`).

v0.1.87 (Dmitrii 2026-11-12: "buoy detection with a colour and obstacle detection with boundaries"): `--detector boundary-v14` = the
colleague's boundary-v14 YOLO11m (~/buoy_robotx/runtime/boundary_v14_20261111/, read-only; the AGX FP16 engine, 1280 x 1280), the same
adapter. Its classes: buoy_red / buoy_green / buoy_blue (a colour each: --colour auto takes them at >= led_conf like Model A's LEDs),
buoy_boundary_no_light (a buoy, unlit: "black" in model mode, else our HSV vote), dock (skipped by default: the colleague's README saw
numbered buoys as dock), obstacle. The obstacle boxes (`--obstacle-classes`, default obstacle) are measured like a buoy (the box's inner
ellipse over the ZED depth) and handed to the boat as the guard's `far` clusters (`obstacle_far`: range, bearing, the box's bearing
extent) - the boat already rings those on its map (usv/guard.map_far: 3 sightings, none where a mapped buoy explains it). The
engine's class list is checked at start (`check_names`): another file under the same name stops the observer.
"""
from __future__ import annotations

from typing import Optional, Sequence, Tuple

BUOY_CLASSES = ("buoy_led_red", "buoy_led_green", "buoy_no_led",
                "buoy_red", "buoy_green", "buoy_blue", "buoy_boundary_no_light",           # v0.1.87: + boundary-v14's
                "buoy")                                                                    # v0.1.87: six-class's generic buoy (colour: our HSV vote)
LED_COLOUR = {"buoy_led_red": "red", "buoy_led_green": "green",
              "buoy_red": "red", "buoy_green": "green", "buoy_blue": "blue"}
UNLIT = ("buoy_no_led", "buoy_boundary_no_light")
COLOUR_MODES = ("auto", "model", "hsv")
DETECTORS = ("mask", "model-a", "boundary-v14", "six-class")
BOX_DETECTORS = ("model-a", "boundary-v14", "six-class")
SIX_NAMES = ("buoy_red", "buoy_green", "buoy_blue", "buoy", "dock", "platform")   # v0.1.87: robotx_six_class_yolo11m_1280 (11 Nov)
V14_NAMES = ("buoy_red", "buoy_green", "buoy_blue", "buoy_boundary_no_light", "dock", "obstacle")   # the colleague's EXPECTED_NAMES
V14_DEFAULTS = {"conf": 0.25, "imgsz": 1280, "iou": 0.55}                     # their test_boundary_v14_engine.py defaults


def buoy_class_ids(names) -> list:
    """The ids of Model A's buoy classes (names: the model's {id: name} or list)."""
    items = names.items() if isinstance(names, dict) else enumerate(names)
    return sorted(int(k) for k, v in items if str(v) in BUOY_CLASSES)


def pick_colour(mode: str, model_class: str, conf: float, hsv_colour: Optional[str], led_conf: float = 0.5) -> Tuple[str, str]:
    """-> (colour, source). hsv_colour may be None when the HSV vote was not run (mode model)."""
    if mode not in COLOUR_MODES:
        raise ValueError(f"colour mode {mode!r}: one of {COLOUR_MODES}")
    led = LED_COLOUR.get(model_class)
    if mode == "model":
        return (led or ("black" if model_class in UNLIT else "unknown")), "model"
    if mode == "auto" and led and conf >= led_conf:
        return led, "model"
    return (hsv_colour or "unknown"), "hsv"


def inner_ellipse(box: Sequence[float], shrink: float = 0.8) -> Tuple[float, float, float, float]:
    """box [x1, y1, x2, y2] -> (cx, cy, semi-axis x, semi-axis y) of the inscribed ellipse scaled by shrink."""
    if not 0.0 < shrink <= 1.0:
        raise ValueError("shrink must be in (0, 1]")
    x1, y1, x2, y2 = (float(v) for v in box)
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0, max(0.5, (x2 - x1) / 2.0 * shrink), max(0.5, (y2 - y1) / 2.0 * shrink)


def box_mask(height: int, width: int, box: Sequence[float], shrink: float = 0.8):
    """A float32 mask (1 inside the shrunk inscribed ellipse, 0 elsewhere) of the frame's size - what measure_mask takes."""
    import numpy as np
    cx, cy, ax, ay = inner_ellipse(box, shrink)
    ys, xs = np.ogrid[:height, :width]
    return (((xs - cx) / ax) ** 2 + ((ys - cy) / ay) ** 2 <= 1.0).astype(np.float32)


def names_list(names) -> list:
    """The model's {id: name} or list -> the names in id order."""
    return [str(names[k]) for k in sorted(names)] if isinstance(names, dict) else [str(n) for n in names]


def check_names(detector: str, names) -> None:
    """v0.1.87: boundary-v14's engine must carry exactly its six classes (the colleague's check, test_boundary_v14_engine.py)."""
    want = {"boundary-v14": V14_NAMES, "six-class": SIX_NAMES}.get(detector)
    if want and names_list(names) != list(want):
        raise SystemExit(f"--detector {detector}: unexpected classes {names_list(names)} (want {list(want)})")


def class_ids(names, wanted) -> list:
    """The ids of the named classes (e.g. the obstacle classes)."""
    items = names.items() if isinstance(names, dict) else enumerate(names)
    return sorted(int(k) for k, v in items if str(v) in set(wanted))


def parse_classes(text: str, detector: str) -> tuple:
    """--obstacle-classes 'obstacle,dock' -> ('obstacle', 'dock'); '' / 'none' -> (); only boundary-v14's non-buoy classes."""
    out = tuple(c.strip() for c in (text or "").split(",") if c.strip() and c.strip() != "none")
    allowed = ("obstacle", "dock") if detector == "boundary-v14" else ("obstacle", "dock", "home")
    bad = [c for c in out if c not in allowed]
    if bad:
        raise ValueError(f"--obstacle-classes: {bad} not in {allowed}")
    return out


def obstacle_far(forward_m: float, right_m: float, box: Sequence[float], fx: float, cx: float, conf: float, cls: str) -> dict:
    """v0.1.87: one measured obstacle box -> a `far` cluster as guard_core gives it (horizontal range, bearing + starboard, the box's
    bearing extent through the intrinsics) + where it came from (src model, its class and score)."""
    import math
    x1, _, x2, _ = (float(v) for v in box)
    return {"range_m": round(math.hypot(forward_m, right_m), 2), "bearing_deg": round(math.degrees(math.atan2(right_m, forward_m)), 1),
            "bearing_min_deg": round(math.degrees(math.atan2(x1 - cx, fx)), 1), "bearing_max_deg": round(math.degrees(math.atan2(x2 - cx, fx)), 1),
            "px": int(max(0.0, x2 - x1)), "src": "model", "cls": cls, "conf": round(float(conf), 3)}
