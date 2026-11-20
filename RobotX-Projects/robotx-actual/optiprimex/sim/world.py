"""World files for the simulators (profiles/bench/world_*.yaml): loading, the world frame, buoy lights, the dock state.

Shared by sim/fake_perception.py (what the camera sees) and sim/fake_dock.py (the Task 3 indicator and windows). Pure
functions, no I/O beyond reading the YAML. The world frame is x_n north / y_e east metres from the file's `origin`; a
pose from mm/state is placed in it by its lat/lon (WorldFrame.to_world), so the world does not move when the mission
manager or usv_ui pick another control origin.

v0.1.53: a world can be anchored to a course of the catalogue (sim/courses.py): `anchor: {course: EXTRA}` puts its origin at the
course's centre, and points may be given in the course frame (`u` along the course's axis, `v` to starboard of it; `face_rel_deg` /
`hdg_rel` relative to the axis; `bounds: course` = the course's corners) - load() resolves them to x_n / y_e. move_to_course()
re-anchors any world to another course (origin + rotation, the relative geometry kept) so the simulator can "move there".
"""
from __future__ import annotations

import copy
import math
import random
from typing import Dict, List, Optional, Tuple

import yaml

R_EARTH = 6378137.0


def load(path: str) -> dict:
    with open(path) as f:
        w = yaml.safe_load(f) or {}
    w.setdefault("objects", [])
    w.setdefault("camera", {})
    w.setdefault("name", str(path).rsplit("/", 1)[-1].rsplit(".", 1)[0])
    cam = w["camera"]
    if cam.get("intrinsics"):                       # v0.1.34: the boat's camera, one source (sim/zed.py)
        from . import zed
        res = zed.resolution_of(cam["intrinsics"])
        if res is None:
            raise ValueError(f"{path}: camera.intrinsics {cam['intrinsics']!r}: one of {sorted(zed.INTRINSICS_NAMES)}")
        cam["fov_deg"] = round(zed.hfov_deg(res), 3)
    resolve_anchor(w)
    return w


# ---- v0.1.53: worlds anchored to a course ---------------------------------------------------------------------------------------
def anchor_course(w: dict):
    """The catalogue course a world is anchored to: `anchor: {course: ID}`, else a `course:` key naming a catalogue course (the
    v0.1.46.1 `course: course_alpha` of world_marina); None for a free world (the pool, the boxes ...)."""
    from . import courses as C
    a = w.get("anchor")
    if isinstance(a, dict) and a.get("course"):
        if C.get(a["course"]) is None and a.get("corners"):          # a received course outside the catalogue (fitted again)
            return C.from_body({"course_id": a["course"], "boundary": a["corners"], "uav_transit_area": a.get("transit") or [],
                                "pinger_hz": a.get("pinger_hz")}, a["course"])
        return C.get(a["course"])
    if isinstance(a, str):
        return C.get(a)
    c = w.get("course")
    return C.get(c) if isinstance(c, str) and c != "bounds" else None


def _points(w: dict):
    """Every placed thing of a world: (dict, x key, y key, heading key) - objects (and a pontoon's second end), start, teleports."""
    for o in w.get("objects") or []:
        yield o, "x_n", "y_e", "face_hdg_deg"
        if "x2_n" in o or "u2" in o:
            yield o, "x2_n", "y2_e", None
    if isinstance(w.get("start"), dict):
        yield w["start"], "x_n", "y_e", "hdg_deg"
    for t in w.get("teleports") or []:
        yield t, "x_n", "y_e", "hdg"


def resolve_anchor(w: dict) -> dict:
    """An anchored world in place: the origin = the course's centre; `u` / `v` (`u2` / `v2`) -> x_n / y_e; `face_rel_deg` /
    `hdg_rel` -> absolute headings; `bounds: course` -> the course's corners. A free world is left as it is."""
    c = anchor_course(w)
    if c is None or not w.get("anchor"):
        return w
    w["origin"] = {"lat": round(c.centre[0], 7), "lon": round(c.centre[1], 7)}
    for d, kx, ky, kh in _points(w):
        ku, kv = ("u2", "v2") if kx == "x2_n" else ("u", "v")
        if ku in d and kv in d:
            x, y = c.uv_to_ne(float(d[ku]), float(d[kv]))
            d[kx], d[ky] = round(x, 3), round(y, 3)
        if kh and kx == "x_n":
            rel = "face_rel_deg" if kh == "face_hdg_deg" else "hdg_rel"
            if rel in d:
                d[kh] = round((c.axis_deg + float(d[rel])) % 360.0, 2)
    if w.get("bounds") == "course":
        w["bounds"], w["bounds_course"] = c.corners_ne(), True
    return w


def layout_frame(w: dict) -> Tuple[float, float, float]:
    """(x_n, y_e, axis_deg) of the world's layout: an anchored world = (0, 0, its course's axis); a free world = the centre of its
    bounds (else the origin) with `axis_deg` (default 0: its x_n runs along the course's axis after a move)."""
    c = anchor_course(w)
    if c is not None:
        return 0.0, 0.0, c.axis_deg
    b = w.get("bounds") or []
    if isinstance(b, list) and len(b) >= 3:
        xs, ys = [float(p[0]) for p in b], [float(p[1]) for p in b]
        return (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0, float(w.get("axis_deg", 0.0))
    return 0.0, 0.0, float(w.get("axis_deg", 0.0))


def move_to_course(w: dict, course, margin_m: float = 0.0) -> Tuple[dict, dict]:
    """A copy of the world re-anchored to `course` (a courses.Course): its layout's centre onto the course's centre, turned by the
    difference of the axes - every distance and angle between its elements is kept. Bounds that were the old course's corners become
    the new course's; a free world's bounds (the pool's walls) move with it. Returns (world, report): report = {fits, outside: [ids
    / names of the elements outside the course by more than -margin_m], from, to, rotation_deg}."""
    cx, cy, axis = layout_frame(w)
    old = anchor_course(w)
    d = (course.axis_deg - axis) % 360.0
    ca, sa = math.cos(math.radians(d)), math.sin(math.radians(d))
    out = copy.deepcopy(w)

    def tr(x, y):
        dx, dy = float(x) - cx, float(y) - cy
        return round(ca * dx - sa * dy, 3), round(sa * dx + ca * dy, 3)

    for o, kx, ky, kh in _points(out):
        if kx in o and ky in o:
            o[kx], o[ky] = tr(o[kx], o[ky])
            ku, kv = ("u2", "v2") if kx == "x2_n" else ("u", "v")
            o[ku], o[kv] = (round(q, 3) for q in course.ne_to_uv(o[kx], o[ky]))
        if kh and kx == "x_n" and kh in o:
            o[kh] = round((float(o[kh]) + d) % 360.0, 2)
            o.pop("face_rel_deg", None)
            o.pop("hdg_rel", None)
    b = out.get("bounds") or []
    course_bounds = bool(w.get("bounds_course")) or (old is not None and w.get("walls") is False)
    if course_bounds:
        out["bounds"] = course.corners_ne()                # the course's corners (drawn, not walls)
    elif b:
        out["bounds"] = [list(tr(p[0], p[1])) for p in b]  # a free world's walls (the pool) move with it
    out["bounds_course"] = course_bounds
    out["origin"] = {"lat": round(course.centre[0], 7), "lon": round(course.centre[1], 7)}
    out["anchor"] = {"course": course.id}
    if course.source == "received":                   # not in the catalogue: the corners travel with the world
        out["anchor"].update(corners=[list(p) for p in course.corners], transit=[list(p) for p in course.transit], pinger_hz=course.pinger_hz)
    out["course"], out["course_id"] = course.id, course.id      # the fake judge's course for this world (--course world)
    out["moved_from"] = old.id if old is not None else (w.get("name") or "free world")
    outside = []
    for o in out.get("objects") or []:
        if o.get("sensed", True) is False and o.get("kind") in ("uav_pad",):
            continue                                       # the UAV pad lives in the transit area, outside the course
        uu, vv = course.ne_to_uv(float(o.get("x_n", 0.0)), float(o.get("y_e", 0.0)))
        if not course.contains_uv(uu, vv, margin_m):
            outside.append(o.get("id"))
    return out, {"fits": not outside, "outside": outside, "from": out["moved_from"], "to": course.id, "rotation_deg": round(d, 2)}


def dump(w: dict, path: str) -> str:
    """Write a (moved) world as YAML for the other simulator processes (fake_perception, fake_dock: --world FILE)."""
    import os
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    keep = {k: v for k, v in w.items() if k != "camera" or v}
    cam = dict(keep.get("camera") or {})
    if cam.get("intrinsics"):
        cam.pop("fov_deg", None)                          # load() derives it again from the intrinsics
        keep["camera"] = cam
    with open(path, "w") as f:
        f.write(f"# v0.1.53: {w.get('name')} re-anchored to course {w.get('course_id')} by the simulator (sim/world.py move_to_course) - "
                "a run file, the profiles stay as they are\n")
        yaml.safe_dump(keep, f, sort_keys=False, default_flow_style=None, width=200)
    return path


class WorldFrame:
    """Equirectangular lat/lon <-> world metres around the world's origin (the same model as the fake FCU's plant)."""

    def __init__(self, origin: Optional[dict]):
        self.lat0 = float((origin or {}).get("lat", 0.0))
        self.lon0 = float((origin or {}).get("lon", 0.0))
        self.ok = origin is not None
        self.k_lat = math.radians(1.0) * R_EARTH
        self.k_lon = math.radians(1.0) * R_EARTH * math.cos(math.radians(self.lat0))

    def to_world(self, lat: float, lon: float) -> Tuple[float, float]:
        return ((lat - self.lat0) * self.k_lat, (lon - self.lon0) * self.k_lon)

    def to_latlon(self, x_n: float, y_e: float) -> Tuple[float, float]:
        return (self.lat0 + x_n / self.k_lat, self.lon0 + y_e / self.k_lon)


def pose_from_state(body: dict, frame: WorldFrame) -> Optional[dict]:
    """The boat pose in the world frame from a local/<v>/mm/state body (lat/lon preferred, else its local x/y)."""
    lat, lon = body.get("lat"), body.get("lon")
    loc = body.get("local") or {}
    if frame.ok and isinstance(lat, (int, float)) and isinstance(lon, (int, float)):
        x, y = frame.to_world(float(lat), float(lon))
    elif "x_n" in loc:
        x, y = float(loc["x_n"]), float(loc["y_e"])
    else:
        return None
    fcu = body.get("fcu") or {}
    return {"x_n": x, "y_e": y, "hdg_deg": float(body.get("hdg_deg") or loc.get("hdg_deg") or 0.0), "lat": lat, "lon": lon,
            "roll_deg": fcu.get("roll_deg", 0.0), "pitch_deg": fcu.get("pitch_deg", 0.0)}


# ---- lights (handbook 3.3.2: flashing = 1 s on / 1 s off) -------------------------------------------------------------
def light_phases(objects: List[dict], seed: int) -> Dict[int, float]:
    """A random phase per object (seconds), reproducible from the seed; a `phase_s` in the file wins."""
    rng = random.Random(seed ^ 0x9E3779B9)
    return {int(o["id"]): float(o.get("phase_s", rng.random() * 2.0)) for o in objects}


def observed_colour(o: dict, t: float, phase: float, lights: Optional[dict] = None) -> str:
    """The colour the camera sees at time t: `light: off` -> black, `flash` -> its colour or black by phase, else its colour."""
    light = o.get("light")
    if light == "off":
        return "black"
    if light == "flash":
        lights = lights or {}
        period, on = float(lights.get("period_s", 2.0)), float(lights.get("on_s", 1.0))
        return o.get("colour", "unknown") if ((t + phase) % period) < on else "black"
    return o.get("colour", "unknown")


# ---- the dock (sim/fake_dock.py publishes the state, fake_perception applies it) ---------------------------------------
def apply_dock_state(objects: List[dict], state: Optional[dict]) -> List[dict]:
    """Copies of the objects with the dock indicators and windows set from a sim/dock/state body."""
    if not state:
        return objects
    out = copy.deepcopy(objects)
    green = state.get("green_bay")
    win = state.get("window") or {}
    for o in out:
        if o.get("kind") == "dock" and green is not None and o.get("bay") is not None:
            o["colour"] = "green" if int(o["bay"]) == int(green) else "red"
        elif o.get("kind") == "window":
            lit = (win.get("bay") is not None and int(o.get("bay", -1)) == int(win["bay"]) and int(o.get("window", -1)) == int(win.get("window", -2))
                   and win.get("colour") in ("red", "green"))
            o["lit"] = bool(lit)
            o["colour"] = win["colour"] if lit else "black"
    return out


def bay_faces(objects: List[dict]) -> Dict[int, dict]:
    return {int(o["bay"]): o for o in objects if o.get("kind") == "dock" and o.get("bay") is not None}


def in_bay(pose: dict, face: dict, depth_m: float, width_m: float, dock_normal: Tuple[float, float]) -> bool:
    """Is the boat inside the bay box in front of the face (depth along the normal away from the face, width across)?"""
    nn, ne = dock_normal
    dn, de = pose["x_n"] - face["x_n"], pose["y_e"] - face["y_e"]
    out = dn * nn + de * ne                  # distance out of the face along the normal (positive = in the water)
    across = abs(-dn * ne + de * nn)
    return 0.0 <= out <= depth_m and across <= width_m / 2.0


def dock_normal(objects: List[dict], toward: Optional[Tuple[float, float]] = None) -> Tuple[float, float]:
    """Unit normal of the line through the bay faces, pointing to the water (toward `toward`, else south-ish)."""
    faces = sorted(bay_faces(objects).values(), key=lambda o: int(o["bay"]))
    if len(faces) < 2:
        return (-1.0, 0.0)
    a, b = faces[0], faces[-1]
    ln, le = b["x_n"] - a["x_n"], b["y_e"] - a["y_e"]
    L = math.hypot(ln, le) or 1.0
    nn, ne = -le / L, ln / L
    cx = sum(o["x_n"] for o in faces) / len(faces)
    cy = sum(o["y_e"] for o in faces) / len(faces)
    tx, ty = toward if toward is not None else (cx - 10.0, cy)
    if nn * (tx - cx) + ne * (ty - cy) < 0:
        nn, ne = -nn, -ne
    return (nn, ne)
