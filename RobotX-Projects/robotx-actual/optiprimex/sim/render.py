"""render: the simulator's camera - a primitive pseudo-3D world drawn from a pose (v0.1.32, ROADMAP WP10). numpy + Pillow only.

Decided 2026-11-04 with Dmitrii: our own simulator, the renderer in Python, primitives first (blue water, white / coloured buoys, a
light-grey background), no disturbances. One ray per pixel, vectorised: the water plane (z = 0), buoys as shaded spheres floating
half out of the water with a lit beacon on top, box faces / dock bays as boxes, the pool walls (`bounds`) as light-grey vertical
walls, a pale sky with a horizon haze. Frames: x_n north, y_e east, z up (metres, the world file's frame); the camera's heading is
the boat's (deg from north, clockwise), `pitch_deg` down positive. Returns RGB (h, w, 3) uint8 and depth (h, w) float32 = the
distance along the optical axis, as the ZED reports it (inf where the ray meets nothing within `max_range_m`).

A pinhole camera with the ZED 2i's left calibration by default (sim/zed.py, hd2k: 50.9 deg; v0.1.33 and before: 100 deg); `project()` is the inverse used by the tests and
the sim detector: a world point -> (u, v, depth) or None.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

SKY_TOP = np.array([196, 205, 214], np.float32)      # light grey-blue
SKY_HORIZON = np.array([224, 228, 232], np.float32)  # light grey haze
WATER = np.array([28, 92, 158], np.float32)          # blue surface
WATER_FAR = np.array([120, 160, 196], np.float32)    # the water fades toward the horizon
WALL = np.array([200, 200, 196], np.float32)         # light grey background
COLOURS = {"white": (238, 238, 232), "red": (205, 40, 40), "green": (30, 165, 70), "blue": (40, 90, 210), "yellow": (232, 200, 40),
           "black": (35, 35, 38), "orange": (240, 130, 30), "grey": (150, 150, 150)}
LIGHT = np.array([0.35, -0.45, 0.82], np.float32)
LIGHT /= np.linalg.norm(LIGHT)
_D_SKY_TOP, _D_SKY_HORIZON, _D_WATER, _D_WATER_FAR, _D_WALL, _D_LIGHT = SKY_TOP, SKY_HORIZON, WATER, WATER_FAR, WALL, LIGHT
BOX_ID0 = 100000                                     # render_ids: boxes after the spheres


@dataclass
class Camera:
    """A pinhole camera. Default (v0.1.34, WP10g): the boat's ZED 2i left camera at the observers' hd2k, scaled to w x h
    (sim/zed.py: fx, cx, cy from the factory calibration - 50.9 deg). An explicit hfov_deg = a symmetric pinhole of that field of
    view (the chase view; --hfov)."""
    w: int = 640
    h: int = 360
    hfov_deg: Optional[float] = None
    height_m: float = 0.8                          # above the water (the ZED on the boat)
    pitch_deg: float = 0.0
    max_range_m: float = 80.0
    fx_px: Optional[float] = None
    fy_px: Optional[float] = None
    cx_px: Optional[float] = None
    cy_px: Optional[float] = None

    def __post_init__(self):
        if self.fx_px is None:
            if self.hfov_deg is None:
                from . import zed
                self.fx_px, self.fy_px, self.cx_px, self.cy_px = zed.scaled(self.w, self.h)
            else:
                self.fx_px = (self.w / 2.0) / math.tan(math.radians(self.hfov_deg) / 2.0)
        self.fy_px = self.fx_px if self.fy_px is None else self.fy_px
        self.cx_px = self.w / 2.0 if self.cx_px is None else self.cx_px
        self.cy_px = self.h / 2.0 if self.cy_px is None else self.cy_px
        self.hfov_deg = math.degrees(2.0 * math.atan(self.w / 2.0 / self.fx_px)) if self.hfov_deg is None else self.hfov_deg

    def basis(self, hdg_deg: float, pitch_deg: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        psi, th = math.radians(hdg_deg), math.radians(self.pitch_deg if pitch_deg is None else pitch_deg)
        fwd = np.array([math.cos(psi) * math.cos(th), math.sin(psi) * math.cos(th), -math.sin(th)], np.float32)
        right = np.array([-math.sin(psi), math.cos(psi), 0.0], np.float32)
        up = np.cross(fwd, right).astype(np.float32)
        return fwd, right, up

    @property
    def fx(self) -> float:
        return float(self.fx_px)

    @property
    def fy(self) -> float:
        return float(self.fy_px)

    @property
    def cx(self) -> float:
        return float(self.cx_px)

    @property
    def cy(self) -> float:
        return float(self.cy_px)


@dataclass
class Sphere:
    c: Tuple[float, float, float]
    r: float
    colour: Tuple[int, int, int]
    emissive: bool = False                         # a lit beacon: drawn bright, unshaded
    tag: str = ""


@dataclass
class Box:
    c: Tuple[float, float, float]                  # centre
    half: Tuple[float, float, float]               # half extents along its own axes (along the face, depth, height)
    yaw_deg: float                                 # the box's first axis, deg from north (clockwise)
    colour: Tuple[int, int, int]
    tag: str = ""
    emissive: bool = False                         # v0.1.46: a lit pane / indicator - drawn bright, unshaded
    shape: str = "box"                             # "cyl" = a vertical cylinder: radius half[0], half-height half[2]
    bloom: float = 0.0                             # a soft glow around it in the picture (not in the ids), 0 = none


@dataclass
class Wall:
    a: Tuple[float, float]
    b: Tuple[float, float]
    height_m: float = 1.5


@dataclass
class Scene:
    spheres: List[Sphere] = field(default_factory=list)
    boxes: List[Box] = field(default_factory=list)
    walls: List[Wall] = field(default_factory=list)
    parts: Dict[str, dict] = field(default_factory=dict)     # v0.1.46: tag -> the dock part's truth (kind, bay, window, light, centre)


def colour_of(name: Optional[str]) -> Tuple[int, int, int]:
    return COLOURS.get(str(name or "white").lower(), COLOURS["white"])


# ---- v0.1.46 (WP10j): the Task 3 docking structure (docs/DOCK_MOCKUP_SPEC.md) --------------------------------------------
DOCK_SIDE_M = 1.0                                    # the cube's side S (no dimension in the handbook: measure at the event)
DOCK_BAY_SCALE = {1: 1.0, 2: 0.9, 3: 1.1}            # the three bays differ a little (the spec: 1.0 / 0.9 / 1.1 S)
WINDOW_M = 0.25                                      # the target hole, square
WINDOWS = {1: (-0.22, 0.20), 2: (0.15, -0.05)}       # window -> centre (x right as seen from the water, z up) in units of S
TUBE_M = 0.03                                        # the PVC frame tubes
DECK_Z_M = 0.20                                      # the floating dock's planks above the water
INDICATOR_R_M, INDICATOR_H_M = 0.05, 0.08            # the RED / GREEN indicator: a cylinder at the base centre of the face
SYN_ID0 = 900000                                     # a window the world file does not list: SYN_ID0 + dock id * 10 + window
PANE_OFF = (214, 198, 200)                           # the dark pane: pale grey-pink, translucent
PANEL_WHITE, TUBE_GREY, PLANK = (236, 236, 230), (214, 216, 214), (168, 160, 146)
LIT_COLOURS = {"red": (255, 46, 34), "green": (40, 240, 90), "blue": (60, 110, 255)}


def face_heading(world: dict, o: dict) -> float:
    """The direction the dock face looks (toward the water), deg: `face_hdg_deg`, else the normal of the bay line toward the start."""
    if o.get("face_hdg_deg") is not None:
        return float(o["face_hdg_deg"])
    from . import world as W
    st = world.get("start") or {}
    nn, ne = W.dock_normal(world.get("objects") or [], (float(st.get("x_n", 0.0)), float(st.get("y_e", 0.0))) if st else None)
    return math.degrees(math.atan2(ne, nn)) % 360.0


def window_lights(objects: List[dict], state: Optional[dict]) -> Dict[Tuple[int, int], dict]:
    """(bay, window) -> {light: off|red|green|flash, colour, on} from a sim/dock/state body (the lit window RED, then GREEN, else
    dark; `flash: true` + `on` = the Advanced tier's flashing colour) or, without one, the world's window objects (lit / colour)."""
    out: Dict[Tuple[int, int], dict] = {}
    for o in objects:
        if o.get("kind") == "window" and o.get("bay") is not None and o.get("window") is not None and o.get("lit"):
            c = str(o.get("colour") or "red")
            out[(int(o["bay"]), int(o["window"]))] = {"light": c if c in ("red", "green") else "flash", "colour": c, "on": True}
    win = (state or {}).get("window") or {}
    if state is not None:
        out = {}
        c = win.get("colour")
        if win.get("bay") is not None and win.get("window") is not None and c in LIT_COLOURS:
            flash = bool(win.get("flash"))
            out[(int(win["bay"]), int(win["window"]))] = {"light": "flash" if flash else c, "colour": c, "on": bool(win.get("on", True))}
    return out


def dock_structure(sc: Scene, world: dict, o: dict, lights: Dict[Tuple[int, int], dict], style: Optional[dict] = None) -> None:
    """One bay's structure: the white cube on its PVC frame standing on the planks, the two black-bordered windows on the face toward
    the water (1 upper-left, 2 lower-right as seen from the water), the RED / GREEN indicator at the base. Tags (render_ids: one id
    per primitive, the dataset groups by tag): dock:<id> the front panel + its four frame tubes (= the dock_face mask), window:<wid>
    border + pane, indicator:<id>, cube:<id> the body and the other eight tubes, deck:<id> the planks. style (the dataset's jitter):
    side_m, yaw_deg {bay: deg}, border_m, pane, white, tube, plank, light_gain, bloom."""
    st = style or {}
    oid, bay = int(o.get("id")), int(o.get("bay", 0) or 0)
    S = float(o.get("size_m") or (st.get("side_m", DOCK_SIDE_M) * DOCK_BAY_SCALE.get(bay, 1.0)))
    S = min(1.3, max(0.85, S))                                     # two 25 cm windows + borders fit side by side (the spec's jitter)
    B = max(S, 0.95)                                               # the window positions' scale (never overlapping)
    h = face_heading(world, o) + float((st.get("yaw_deg") or {}).get(bay, 0.0))
    hr = math.radians(h)
    n = (math.cos(hr), math.sin(hr))                               # out of the face, toward the water
    r = (math.cos(hr - math.pi / 2), math.sin(hr - math.pi / 2))   # to the right as seen from the water
    px, py = float(o.get("x_n", 0.0)), float(o.get("y_e", 0.0))
    zc = DECK_Z_M + S / 2.0
    yaw = h - 90.0                                                 # a box's first axis = r, the second = n
    white = tuple(st.get("white", PANEL_WHITE))
    tube = tuple(st.get("tube", TUBE_GREY))
    gain = float(st.get("light_gain", 1.0))
    bloom = float(st.get("bloom", 0.6))
    bw = float(st.get("border_m", 0.035))

    def at(fx: float, fn: float, fz: float) -> Tuple[float, float, float]:
        return (px + fx * r[0] + fn * n[0], py + fx * r[1] + fn * n[1], zc + fz)

    def add(kind: str, tid: int, c, half, colour, **kw) -> None:
        sc.boxes.append(Box(c, half, yaw, tuple(int(v) for v in colour), tag=f"{kind}:{tid}", **kw))

    face = {"dock_id": oid, "bay": bay, "face_hdg_deg": round(h % 360.0, 2), "side_m": round(S, 3)}
    colour = str(o.get("colour") or "red")
    sc.parts[f"dock:{oid}"] = dict(face, kind="dock_face", id=oid, centre=at(0.0, 0.0, 0.0), indicator_colour=colour)
    sc.parts[f"cube:{oid}"] = dict(face, kind="cube", id=oid, centre=at(0.0, -S / 2, 0.0))
    sc.parts[f"deck:{oid}"] = dict(face, kind="deck", id=oid, centre=at(0.0, (0.4 - S - 0.3) / 2, -S / 2))
    add("deck", oid, at(0.0, (0.4 - S - 0.3) / 2, -S / 2 - DECK_Z_M / 2), ((S + 1.0) / 2, (S + 0.7) / 2, DECK_Z_M / 2), st.get("plank", PLANK))
    add("cube", oid, at(0.0, -S / 2 - 0.005, 0.0), (S / 2, S / 2 - 0.005, S / 2), white)
    add("dock", oid, at(0.0, 0.005, 0.0), (S / 2, 0.005, S / 2), white)
    t2 = TUBE_M / 2
    for fz in (S / 2, -S / 2):                                     # the front frame: top, bottom, left, right
        add("dock", oid, at(0.0, 0.0, fz), (S / 2 + t2, t2, t2), tube)
    for fx in (-S / 2, S / 2):
        add("dock", oid, at(fx, 0.0, 0.0), (t2, t2, S / 2 + t2), tube)
    add("cube", oid, at(0.0, -S, S / 2), (S / 2 + t2, t2, t2), tube)                  # the back frame and the four side edges
    add("cube", oid, at(0.0, -S, -S / 2), (S / 2 + t2, t2, t2), tube)
    for fx in (-S / 2, S / 2):
        add("cube", oid, at(fx, -S, 0.0), (t2, t2, S / 2 + t2), tube)
        for fz in (S / 2, -S / 2):
            add("cube", oid, at(fx, -S / 2, fz), (t2, S / 2, t2), tube)
    wins = {(int(w["bay"]), int(w["window"])): int(w["id"]) for w in world.get("objects") or []
            if w.get("kind") == "window" and w.get("bay") is not None and w.get("window") is not None and w.get("id") is not None}
    for wn, (ux, uz) in WINDOWS.items():
        wid = wins.get((bay, wn), SYN_ID0 + oid * 10 + wn)
        lim = S / 2 - WINDOW_M / 2 - bw - TUBE_M
        fx, fz = max(-lim, min(lim, ux * B)), max(-lim, min(lim, uz * B))
        li = lights.get((bay, wn)) or {"light": "off", "colour": "black", "on": False}
        on = li["light"] != "off" and li.get("on", True)
        add("window", wid, at(fx, 0.012, fz), (WINDOW_M / 2 + bw, 0.002, WINDOW_M / 2 + bw), COLOURS["black"])
        if on:
            pane = tuple(min(255.0, v * gain) for v in LIT_COLOURS.get(li["colour"], LIT_COLOURS["red"]))
            add("window", wid, at(fx, 0.016, fz), (WINDOW_M / 2, 0.002, WINDOW_M / 2), pane, emissive=True, bloom=bloom)
        else:
            add("window", wid, at(fx, 0.016, fz), (WINDOW_M / 2, 0.002, WINDOW_M / 2), st.get("pane", PANE_OFF))
        sc.parts[f"window:{wid}"] = dict(face, kind="window", id=wid, window=wn, centre=at(fx, 0.016, fz), light=li["light"],
                                         colour=li["colour"] if li["light"] != "off" else "black", lit=bool(on))
    ind = tuple(min(255.0, v * gain) for v in LIT_COLOURS.get(colour, COLOURS.get(colour, COLOURS["red"])))
    add("indicator", oid, at(0.0, 0.016 + INDICATOR_R_M, -S / 2 + INDICATOR_H_M / 2 + 0.002),
        (INDICATOR_R_M, INDICATOR_R_M, INDICATOR_H_M / 2), ind, emissive=True, shape="cyl", bloom=bloom * 0.5)
    sc.parts[f"indicator:{oid}"] = dict(face, kind="indicator", id=oid, colour=colour, indicator_colour=colour,
                                        centre=at(0.0, 0.012 + INDICATOR_R_M, -S / 2 + INDICATOR_H_M / 2))


def scene_from_world(world: dict, lit: Optional[Dict[int, bool]] = None, dock_state: Optional[dict] = None,
                     dock_style: Optional[dict] = None) -> Scene:
    """World file objects -> primitives. Buoys: a sphere (`size_m` diameter, default 0.5; `body` colour, default the light's colour
    for a lit / coloured buoy) + a small beacon on top in the light colour, bright while lit (lit[id], default on). Boxes: a box
    (`width_m` along the face). Dock bays (v0.1.46): the Task 3 structure (dock_structure), its indicator and window lights from
    dock_state (a sim/dock/state body, as world.apply_dock_state applies it; the flashing buoy lights come in `lit` the same way).
    `bounds`: the pool walls."""
    sc = Scene()
    lit = lit or {}
    objects = world.get("objects") or []
    if dock_state:
        from . import world as W
        objects = W.apply_dock_state(objects, dock_state)
    lights = window_lights(objects, dock_state)
    for o in objects:
        k = o.get("kind")
        x, y = float(o.get("x_n", 0.0)), float(o.get("y_e", 0.0))
        col = colour_of(o.get("colour"))
        if k == "buoy":
            d = float(o.get("size_m", 0.5))
            body = colour_of(o.get("body")) if o.get("body") else col
            if o.get("colour_from_above"):        # v0.1.66 (D23): coloured only on its top cap - one neutral hull from the water
                body = COLOURS["grey"]
            sc.spheres.append(Sphere((x, y, d * 0.2), d / 2.0, body, tag=f"buoy:{o.get('id')}"))
            if o.get("light") in ("flash", "steady") and lit.get(o.get("id"), True) and not o.get("colour_from_above"):
                sc.spheres.append(Sphere((x, y, d * 0.2 + d / 2.0 + 0.08), 0.08, col, emissive=True, tag=f"light:{o.get('id')}"))
        elif k == "dock":
            dock_structure(sc, world, o, lights, dock_style)
        elif k == "obstacle":                   # v0.1.80 (WP36): what the detector cannot see (a pole, a boom) - in the depth, never a buoy mask
            hgt = float(o.get("height_m", 0.6))
            if o.get("x2_n") is not None:
                x2, y2 = float(o["x2_n"]), float(o["y2_e"])
                L = math.hypot(x2 - x, y2 - y)
                sc.boxes.append(Box(((x + x2) / 2, (y + y2) / 2, hgt / 2.0), (L / 2.0, 0.1, hgt / 2.0), math.degrees(math.atan2(y2 - y, x2 - x)),
                                    COLOURS["grey"], tag=f"obstacle:{o.get('id')}"))
            else:
                r = float(o.get("r_m", 0.15))
                sc.boxes.append(Box((x, y, hgt / 2.0), (r, r, hgt / 2.0), 0.0, COLOURS["grey"], tag=f"obstacle:{o.get('id')}", shape="cyl"))
        elif k == "box":
            wdt = float(o.get("width_m", 2.0 if k == "box" else 1.0))
            hgt = float(o.get("height_m", 1.2 if k == "box" else 0.6))
            dep = float(o.get("depth_m", 1.0 if k == "box" else 0.5))
            yaw = float(o.get("face_hdg_deg", 90.0)) + 90.0          # the face looks along face_hdg: its width runs across it
            sc.boxes.append(Box((x, y, hgt / 2.0), (wdt / 2.0, dep / 2.0, hgt / 2.0), yaw, col, tag=f"{k}:{o.get('id')}"))
    b = (world.get("bounds") or []) if world.get("walls", True) else []          # walls: false = open water (world_marina)
    for i in range(len(b)):
        p, q = b[i], b[(i + 1) % len(b)]
        sc.walls.append(Wall((float(p[0]), float(p[1])), (float(q[0]), float(q[1]))))
    return sc


def rays(cam: Camera, hdg_deg: float, pitch_deg: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
    """(h*w, 3) ray directions (not unit: the forward component is 1) and the forward axis."""
    fwd, right, up = cam.basis(hdg_deg, pitch_deg)
    u = (np.arange(cam.w, dtype=np.float32) + 0.5 - cam.cx) / cam.fx
    v = (np.arange(cam.h, dtype=np.float32) + 0.5 - cam.cy) / cam.fy
    uu, vv = np.meshgrid(u, v)
    d = fwd[None, None, :] + uu[..., None] * right[None, None, :] - vv[..., None] * up[None, None, :]
    return d.reshape(-1, 3), fwd


def render(scene: Scene, x_n: float, y_e: float, hdg_deg: float, cam: Optional[Camera] = None, pitch_deg: Optional[float] = None,
           z_m: Optional[float] = None) -> Tuple[np.ndarray, np.ndarray]:
    rgb, depth, _ = render_ids(scene, x_n, y_e, hdg_deg, cam, pitch_deg, z_m)
    return rgb, depth


def render_ids(scene: Scene, x_n: float, y_e: float, hdg_deg: float, cam: Optional[Camera] = None, pitch_deg: Optional[float] = None,
               z_m: Optional[float] = None, style: Optional[dict] = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """render() plus the primitive under each pixel (h, w) int32: -1 sky / water / wall, i = scene.spheres[i],
    BOX_ID0 + j = scene.boxes[j] - the masks of the dataset export (v0.1.33) and the sim observer (WP10e).
    style (the dataset's domain jitter): light (sun direction, 3), water / water_far / sky_top / sky_horizon / wall (RGB)."""
    cam = cam or Camera()
    st = style or {}
    SKY_TOP, SKY_HORIZON, WATER, WATER_FAR, WALL = (np.asarray(st.get(k, d), np.float32) for k, d in (
        ("sky_top", _D_SKY_TOP), ("sky_horizon", _D_SKY_HORIZON), ("water", _D_WATER), ("water_far", _D_WATER_FAR), ("wall", _D_WALL)))
    LIGHT = np.asarray(st.get("light", _D_LIGHT), np.float32)
    LIGHT = LIGHT / np.linalg.norm(LIGHT)
    o = np.array([x_n, y_e, cam.height_m if z_m is None else z_m], np.float32)
    d, fwd = rays(cam, hdg_deg, pitch_deg)
    n = d.shape[0]
    t = np.full(n, np.inf, np.float32)
    rgb = np.zeros((n, 3), np.float32)
    pid = np.full(n, -1, np.int32)
    # sky (the default) and the water plane
    dn = d / np.linalg.norm(d, axis=1, keepdims=True)
    sky_k = np.clip(dn[:, 2] * 4.0, 0.0, 1.0)[:, None]
    rgb[:] = SKY_HORIZON * (1 - sky_k) + SKY_TOP * sky_k
    down = d[:, 2] < -1e-6
    tw = np.where(down, -o[2] / np.where(down, d[:, 2], -1.0), np.inf)
    hitw = down & (tw < cam.max_range_m * 1.5)
    if hitw.any():
        p = o + d[hitw] * tw[hitw, None]
        ripple = 0.06 * np.sin(1.3 * p[:, 0] + 0.7 * p[:, 1]) * np.sin(0.9 * p[:, 1] - 0.4 * p[:, 0])
        far = np.clip(np.hypot(p[:, 0] - o[0], p[:, 1] - o[1]) / cam.max_range_m, 0.0, 1.0)[:, None]
        rgb[hitw] = (WATER * (1.0 + ripple[:, None])) * (1 - far) + WATER_FAR * far
        t[hitw] = tw[hitw]
    # walls: vertical rectangles on the bounds' edges
    for wl in scene.walls:
        ax, ay = wl.a
        ex, ey = wl.b[0] - ax, wl.b[1] - ay
        L = math.hypot(ex, ey)
        if L < 1e-6:
            continue
        nx, ny = -ey / L, ex / L                                      # the wall's horizontal normal
        den = d[:, 0] * nx + d[:, 1] * ny
        ok = np.abs(den) > 1e-6
        tt = np.where(ok, ((ax - o[0]) * nx + (ay - o[1]) * ny) / np.where(ok, den, 1.0), np.inf)
        with np.errstate(invalid="ignore"):
            p = o + d * tt[:, None]
            s = ((p[:, 0] - ax) * ex + (p[:, 1] - ay) * ey) / (L * L)
        hit = ok & (tt > 0) & (tt < t) & (s >= 0) & (s <= 1) & (p[:, 2] >= 0) & (p[:, 2] <= wl.height_m)
        if hit.any():
            shade = 0.85 + 0.15 * abs(nx * LIGHT[0] + ny * LIGHT[1])
            stripe = np.where((p[hit, 2] > wl.height_m - 0.12), 0.9, 1.0)[:, None]
            rgb[hit] = WALL * shade * stripe
            t[hit] = tt[hit]
    # boxes (slabs in the box's own frame; v0.1.46: vertical cylinders, emissive faces) - only the rays of the box's screen rectangle
    glow: List[Tuple[np.ndarray, float, int]] = []
    for j, bx in enumerate(scene.boxes):
        yaw = math.radians(bx.yaw_deg)
        ax1 = np.array([math.cos(yaw), math.sin(yaw), 0.0], np.float32)
        ax2 = np.array([-math.sin(yaw), math.cos(yaw), 0.0], np.float32)
        ax3 = np.array([0.0, 0.0, 1.0], np.float32)
        idx = _screen_rows(cam, o, fwd, bx.c, (ax1, ax2, ax3), bx.half)
        if idx is not None and idx.size == 0:
            continue
        ds = d if idx is None else d[idx]
        m = ds.shape[0]
        oc = o - np.array(bx.c, np.float32)
        tmin = np.full(m, -np.inf, np.float32)
        tmax = np.full(m, np.inf, np.float32)
        nrm = np.zeros((m, 3), np.float32)
        if bx.shape == "cyl":                                        # x^2 + y^2 = r^2 between the caps
            rad = bx.half[0]
            ox, oy = float(oc[0]), float(oc[1])
            a2 = ds[:, 0] ** 2 + ds[:, 1] ** 2
            b2 = ox * ds[:, 0] + oy * ds[:, 1]
            disc = b2 * b2 - a2 * (ox * ox + oy * oy - rad * rad)
            okc = (disc > 0) & (a2 > 1e-12)
            sq = np.sqrt(np.where(okc, disc, 0.0))
            sa = np.where(okc, a2, 1.0)
            t1, t2 = np.where(okc, (-b2 - sq) / sa, np.inf), np.where(okc, (-b2 + sq) / sa, -np.inf)
            safe = np.where(np.abs(ds[:, 2]) < 1e-9, 1e-9, ds[:, 2])
            z1, z2 = (-bx.half[2] - float(oc[2])) / safe, (bx.half[2] - float(oc[2])) / safe
            zlo, zhi = np.minimum(z1, z2), np.maximum(z1, z2)
            tmin, tmax = np.maximum(t1, zlo), np.minimum(t2, zhi)
            side = t1 >= zlo
            pc = oc + ds * np.where(np.isfinite(tmin), tmin, 0.0)[:, None]
            nrm[side, 0], nrm[side, 1] = pc[side, 0] / rad, pc[side, 1] / rad
            nrm[~side, 2] = -np.sign(ds[~side, 2])
        else:
            for axis, half in ((ax1, bx.half[0]), (ax2, bx.half[1]), (ax3, bx.half[2])):
                od, dd = float(oc @ axis), ds @ axis
                safe = np.where(np.abs(dd) < 1e-9, 1e-9, dd)
                t1, t2 = (-half - od) / safe, (half - od) / safe
                lo, hi = np.minimum(t1, t2), np.maximum(t1, t2)
                upd = lo > tmin
                nrm[upd] = -np.sign(dd[upd])[:, None] * axis[None, :]
                tmin, tmax = np.maximum(tmin, lo), np.minimum(tmax, hi)
        ts = t if idx is None else t[idx]
        hit = (tmax >= tmin) & (tmin > 0) & (tmin < ts)
        if hit.any():
            hi_ = np.flatnonzero(hit) if idx is None else idx[hit]
            if bx.emissive:
                rgb[hi_] = np.clip(np.array(bx.colour, np.float32) * 1.05 + 25.0, 0, 255)
                if bx.bloom > 0:
                    glow.append((hi_, bx.bloom, BOX_ID0 + j))
            else:
                lam = np.clip(nrm[hit] @ LIGHT, 0.0, 1.0)
                rgb[hi_] = np.array(bx.colour, np.float32) * (0.45 + 0.55 * lam)[:, None]
            t[hi_] = tmin[hit]
            pid[hi_] = BOX_ID0 + j
    # spheres
    dd = np.einsum("ij,ij->i", d, d)
    for i, sp in enumerate(scene.spheres):
        oc = o - np.array(sp.c, np.float32)
        b = d @ oc
        cc = float(oc @ oc) - sp.r * sp.r
        disc = b * b - dd * cc
        ok = disc > 0
        ts = np.where(ok, (-b - np.sqrt(np.where(ok, disc, 0.0))) / dd, np.inf)
        hit = ok & (ts > 0) & (ts < t)
        if not hit.any():
            continue
        p = o + d[hit] * ts[hit, None]
        if not sp.emissive:
            above = p[:, 2] >= -0.02                                   # the part under the water is hidden by the plane anyway
            hit_idx = np.flatnonzero(hit)[above]
            p = p[above]
            nn = (p - np.array(sp.c, np.float32)) / sp.r
            lam = np.clip(nn @ LIGHT, 0.0, 1.0)
            spec = np.clip(nn @ LIGHT, 0.0, 1.0) ** 24 * 60.0
            rgb[hit_idx] = np.clip(np.array(sp.colour, np.float32) * (0.5 + 0.5 * lam)[:, None] + spec[:, None], 0, 255)
            t[hit_idx] = ts[hit_idx]
            pid[hit_idx] = i
        else:
            rgb[hit] = np.clip(np.array(sp.colour, np.float32) * 1.15 + 40.0, 0, 255)
            t[hit] = ts[hit]
            pid[hit] = i
    depth = (t * (d @ fwd)).astype(np.float32)                          # along the optical axis (d·fwd = 1)
    depth[~np.isfinite(t) | (depth > cam.max_range_m)] = np.inf
    img = rgb.reshape(cam.h, cam.w, 3)
    if glow:
        img = _bloom(img, pid.reshape(cam.h, cam.w), glow, cam)
    return img.clip(0, 255).astype(np.uint8), depth.reshape(cam.h, cam.w), pid.reshape(cam.h, cam.w)


def _screen_rows(cam: Camera, o: np.ndarray, fwd: np.ndarray, c, axes, half) -> Optional[np.ndarray]:
    """The flat ray indices of the screen rectangle a box (or a cylinder's box) projects into (+2 px), an empty array when it is
    off screen or behind the camera, None when a corner is behind the camera plane (test every ray). Exact: only a speed-up."""
    right = np.array([-fwd[1], fwd[0], 0.0], np.float32)
    nr = float(np.linalg.norm(right))
    if nr < 1e-6:
        return None
    right /= nr
    up = np.cross(fwd, right)
    cc = np.array(c, np.float32)
    corners = np.array([cc + sx * half[0] * axes[0] + sy * half[1] * axes[1] + sz * half[2] * axes[2]
                        for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], np.float32) - o
    z = corners @ fwd
    if (z <= 0).all():
        return np.zeros(0, np.int64)
    if (z <= 0.05).any():
        return None
    u = cam.cx + cam.fx * (corners @ right) / z
    v = cam.cy - cam.fy * (corners @ up) / z
    c0, c1 = max(0, int(math.floor(u.min())) - 2), min(cam.w, int(math.ceil(u.max())) + 2)
    r0, r1 = max(0, int(math.floor(v.min())) - 2), min(cam.h, int(math.ceil(v.max())) + 2)
    if c0 >= c1 or r0 >= r1:
        return np.zeros(0, np.int64)
    if (c1 - c0) * (r1 - r0) > 0.6 * cam.w * cam.h:
        return None
    return (np.arange(r0, r1)[:, None] * cam.w + np.arange(c0, c1)[None, :]).ravel()


def _bloom(img: np.ndarray, pid: np.ndarray, glow: List[Tuple[np.ndarray, float, int]], cam: Camera) -> np.ndarray:
    """A soft glow around the visible lit panes (a box blur of their colour added to the picture; the ids stay exact)."""
    src = np.zeros_like(img)
    flat = img.reshape(-1, 3)
    for idx, k, code in glow:
        rr = idx[pid.reshape(-1)[idx] == code]                     # still in front (nothing drawn over it later)
        src.reshape(-1, 3)[rr] = flat[rr] * k
    if not src.any():
        return img
    rad = max(2, int(round(cam.w / 160)))
    out = src
    for ax in (0, 1):                                               # two box blurs per axis ~ a soft falloff
        for _ in range(2):
            c = np.cumsum(np.pad(out, [(rad + 1, rad) if a == ax else (0, 0) for a in range(3)], mode="constant"), axis=ax)
            out = (np.take(c, range(2 * rad + 1, c.shape[ax]), axis=ax) - np.take(c, range(0, c.shape[ax] - 2 * rad - 1), axis=ax)) / (2 * rad + 1)
    return img + out * 0.8


def project(cam: Camera, x_n: float, y_e: float, hdg_deg: float, p: Tuple[float, float, float], z_m: Optional[float] = None,
            pitch_deg: Optional[float] = None) -> Optional[Tuple[float, float, float]]:
    """A world point -> (u, v, depth along the axis) in pixels, or None behind the camera."""
    fwd, right, up = cam.basis(hdg_deg, pitch_deg)
    rel = np.array(p, np.float32) - np.array([x_n, y_e, cam.height_m if z_m is None else z_m], np.float32)
    z = float(rel @ fwd)
    if z <= 1e-6:
        return None
    return cam.cx + cam.fx * float(rel @ right) / z, cam.cy - cam.fy * float(rel @ up) / z, z


def jpeg(rgb: np.ndarray, quality: int = 80) -> bytes:
    import io

    from PIL import Image
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


def main(argv: Optional[list] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(prog="python -m optiprimex.sim.render", description="The simulator's renderer: one frame, or a dataset "
                                 "(v0.1.33, WP10b: YOLO labels + masks + depth for CV_TRAIN; layout in optiprimex/sim/dataset.py).")
    ap.add_argument("--dataset", metavar="DIR", help="write a dataset into DIR (a new or empty directory)")
    ap.add_argument("--n", type=int, default=100, help="frames (default 100)")
    ap.add_argument("--world", action="append", help="marina|pool|por|boxes|bench|empty or a world file; repeat to mix (default pool)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--val", type=float, default=0.2, help="--layout split: the share of poses in val (default 0.2)")
    ap.add_argument("--classes", choices=("kind", "colour"), default="kind", help="kind = buoy/dock_face/window/box (CV_TRAIN option A, "
                    "default; + indicator with --scene dock | mix); colour = buoy_red ... (option B)")
    ap.add_argument("--scene", choices=("buoys", "dock", "mix"), default="buoys", help="v0.1.46: buoys = random poses, four classes "
                    "(default, as v0.1.33); dock = poses at the Task 3 dock (approach / docked / passing), five classes; mix = half each")
    ap.add_argument("--size", default=None, help="WxH (dataset default 1280x720, a frame 640x360)")
    ap.add_argument("--layout", choices=("flat", "split"), default="flat", help="flat = images/STEM.jpg (default); split = images/{train,val}/ by pose")
    ap.add_argument("--frames-per-pose", type=int, default=3, help="frames per pose, the boat creeping on (default 3)")
    ap.add_argument("--negatives", type=float, default=0.1, help="the share of poses facing no object (default 0.1)")
    ap.add_argument("--depth", choices=("npz", "png", "none"), default="npz", help="npz = float16 metres (default); png = uint16 mm")
    ap.add_argument("--depth-noise", choices=("none", "zed"), default="none", help="zed = the stereo model of sim/zed.py (v0.1.34); default exact")
    ap.add_argument("--hfov", type=float, default=None, help="a symmetric pinhole of this field of view instead of the ZED's intrinsics (sim/zed.py)")
    ap.add_argument("--no-jitter", action="store_true", help="no camera height / pitch / gain / noise variation")
    ap.add_argument("--frame", metavar="JPG", help="one frame instead: --frame out.jpg --pose x_n,y_e,hdg")
    ap.add_argument("--pose", default="0,0,0")
    a = ap.parse_args(argv)
    w, h = (int(v) for v in (a.size or ("1280x720" if a.dataset else "640x360")).lower().split("x"))
    worlds = a.world or ["pool"]
    if a.dataset:
        from . import dataset
        dataset.export(a.dataset, a.n, worlds, seed=a.seed, val=a.val, scheme=a.classes, w=w, h=h, hfov=a.hfov, jitter=not a.no_jitter,
                       layout=a.layout, frames_per_pose=max(1, a.frames_per_pose), negatives=a.negatives, depth_fmt=a.depth,
                       depth_noise=None if a.depth_noise == "none" else a.depth_noise, scene=a.scene)
        return 0
    if a.frame:
        from . import dataset, world as W
        x, y, hdg = (float(v) for v in a.pose.split(","))
        rgb, _ = render(scene_from_world(W.load(dataset.world_path(worlds[0]))), x, y, hdg, Camera(w=w, h=h, hfov_deg=a.hfov))
        with open(a.frame, "wb") as f:
            f.write(jpeg(rgb, 90))
        print(f"wrote {a.frame}")
        return 0
    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
