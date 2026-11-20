"""ghost: the free-run position (v0.1.68, WP17 F2; docs/PROMPT_FREE_RUN_2026-11-08.md).

`--free-run` (the lab profile only) runs the mission manager's stages against this in-process plant (sim/plant.Plant, the fake
FCU's model and laws) instead of the FCU's position: the RoverAdapter hands every behaviour output here (usv/adapters/rover.py,
muted) and the real FCU receives the mode requests only. The pose comes from the ghost; the FCU's lat / lon / hdg are not used
for it (an indoor fix is shown, not used).

The start: `--free-start LAT,LON,HDG`, else the `--world FILE`'s `start` (its origin + x_n / y_e), else the profile's
`free_run.start` {lat, lon, hdg_deg}. dt = the tick clock, clamped to <= 0.5 s. Hold / nothing -> zero targets.
"""
from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

from ..sim.plant import Plant, frame_uvr, guided_ur, velocity_ur

FRAME_TIMEOUT_S = 0.25          # as the firmware's mode-20 frame timeout (the fake FCU's 250 ms)
VEL_TIMEOUT_S = 3.0             # as the fake FCU's velocity-target timeout
DT_MAX_S = 0.5


def parse_start(text: str) -> Tuple[float, float, float]:
    """'LAT,LON[,HDG]' -> (lat, lon, hdg_deg)."""
    parts = [p.strip() for p in str(text).split(",") if p.strip()]
    if len(parts) not in (2, 3):
        raise ValueError(f"--free-start {text!r}: LAT,LON[,HDG]")
    lat, lon = float(parts[0]), float(parts[1])
    hdg = float(parts[2]) if len(parts) == 3 else 0.0
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        raise ValueError(f"--free-start {text!r}: lat / lon out of range")
    return lat, lon, hdg % 360.0


def world_start(path: str) -> Tuple[float, float, float]:
    """A sim world file's `start` as (lat, lon, hdg_deg) - its origin + start x_n / y_e (sim/world.WorldFrame)."""
    from ..sim import world as W
    w = W.load(path)
    if not isinstance(w.get("origin"), dict):
        raise ValueError(f"{path}: no origin")
    st = w.get("start") or {}
    lat, lon = W.WorldFrame(w["origin"]).to_latlon(float(st.get("x_n", 0.0)), float(st.get("y_e", 0.0)))
    return lat, lon, float(st.get("hdg_deg", 0.0)) % 360.0


def resolve_start(free_start: Optional[str], world: Optional[str], cfg_block: Optional[dict]) -> Tuple[Tuple[float, float, float], str]:
    """(start, where it came from) - the order of the brief: --free-start, --world, free_run.start."""
    if free_start:
        return parse_start(free_start), "--free-start"
    if world:
        return world_start(world), f"--world {world}"
    st = (cfg_block or {}).get("start") or {}
    if "lat" not in st or "lon" not in st:
        raise ValueError("free_run.start {lat, lon, hdg_deg} missing in the profile (or give --free-start / --world)")
    return (float(st["lat"]), float(st["lon"]), float(st.get("hdg_deg", 0.0)) % 360.0), "free_run.start"


class GhostNav:
    """The ghost plant + the last behaviour output handed to it (target | vel | frame | hold)."""

    def __init__(self, start: Tuple[float, float, float], wp_speed: float = 2.0, hull: str = "ideal", str_ramp_s: float = 0.0):
        lat, lon, hdg = start
        self.start = (lat, lon, hdg)
        self.plant = Plant(lat, lon, hdg, hull=hull, str_ramp_s=str_ramp_s)     # v0.1.85: the profile's plant.hull
        self.wp_speed = float(wp_speed)          # the fake FCU's WP_SPEED default; do_change_speed sets it
        self.kind: Optional[str] = None          # target | vel | frame | hold | None
        self.target: Optional[Tuple[float, float]] = None
        self.vel: Optional[Tuple[float, float]] = None
        self.frame: Sequence[float] = (0.0, 0.0, 0.0, 0.0)
        self.t_cmd = float("-inf")
        self.t_last: Optional[float] = None
        self.counts = {"target": 0, "vel": 0, "frame": 0, "hold": 0}

    # ---- the adapter's outputs ------------------------------------------------------------------------------
    def set_target(self, lat: float, lon: float, now: float) -> None:
        self.kind, self.target, self.vel, self.t_cmd = "target", (float(lat), float(lon)), None, now
        self.counts["target"] += 1

    def set_speed(self, mps: float) -> None:
        self.wp_speed = float(mps)

    def set_vel(self, vx: float, r: float, now: float) -> None:
        self.kind, self.vel, self.target, self.t_cmd = "vel", (float(vx), float(r)), None, now
        self.counts["vel"] += 1

    def set_frame(self, m: Sequence[float], now: float) -> None:
        self.kind, self.frame, self.target, self.vel, self.t_cmd = "frame", tuple(float(x) for x in m), None, None, now
        self.counts["frame"] += 1

    def set_hold(self) -> None:
        self.kind, self.target, self.vel, self.frame = "hold", None, None, (0.0, 0.0, 0.0, 0.0)
        self.counts["hold"] += 1

    # ---- the plant --------------------------------------------------------------------------------------------
    def targets(self, now: float) -> Tuple[float, float, float]:
        """(u, v, r) targets now: as the fake FCU's physics for the same output; hold / stale / nothing = zero."""
        if self.kind == "frame" and now - self.t_cmd <= FRAME_TIMEOUT_S:
            return frame_uvr(self.frame)
        if self.kind == "target" and self.target is not None:
            u, r = guided_ur(self.plant, self.target, self.wp_speed)
            return u, 0.0, r
        if self.kind == "vel" and self.vel is not None and now - self.t_cmd <= VEL_TIMEOUT_S:
            u, r = velocity_ur(self.vel)
            return u, 0.0, r
        return 0.0, 0.0, 0.0

    def step(self, now: float) -> float:
        """Advance to `now` (the tick clock); returns the dt used (clamped to <= 0.5 s; 0 on the first call)."""
        if self.t_last is None:
            self.t_last = now
            return 0.0
        dt = max(0.0, min(DT_MAX_S, now - self.t_last))
        self.t_last = now
        if dt > 0.0:
            u, v, r = self.targets(now)
            self.plant.step(dt, u, v, r)
        return dt

    @property
    def lat(self) -> float:
        return self.plant.lat

    @property
    def lon(self) -> float:
        return self.plant.lon

    @property
    def hdg_deg(self) -> float:
        return self.plant.hdg_deg

    @property
    def vn_ve(self) -> Tuple[float, float]:
        return self.plant.vn_ve

    @property
    def sog_mps(self) -> float:
        return math.hypot(*self.plant.vn_ve)

    def view(self) -> dict:
        vn, ve = self.vn_ve
        return {"lat": round(self.lat, 7), "lon": round(self.lon, 7), "hdg_deg": round(self.hdg_deg, 1),
                "sog_mps": round(self.sog_mps, 2), "vn": round(vn, 2), "ve": round(ve, 2), "out": self.kind,
                "counts": dict(self.counts)}
