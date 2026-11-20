"""plant: the planar boat model shared by the fake FCU and the free-run ghost (v0.1.68, WP17 F2).

Moved here from sim/fake_fcu.py unchanged (names and numbers): `Plant` (3-DOF first-order surge / sway / yaw rate), the GUIDED
pursuit the fake FCU flies to a position target, the velocity-yaw target clip, and the mode-20 thrust frame -> (u, v, r)
mapping (v0.1.85: + the hull's yaw authority, `hull` ideal | omni_usv, and the optional steering lag `str_ramp_s`). sim/fake_fcu.py imports them back, so the fake FCU behaves exactly as before; usv/ghost.py (the mission manager's
--free-run) steps the same plant in process.
"""
from __future__ import annotations

import math
from typing import Sequence, Tuple

R_EARTH = 6_378_137.0
HULLS = ("ideal", "omni_usv")          # v0.1.85 (WP41 T0.4): profile plant.hull
OMNI_R0_DPS, OMNI_R05_DPS = 2.0, 4.0   # omni_usv: yaw authority in place / at 0.5 m/s (docs/WATER_DAY_2026-11-11_NOTE.md)


class Plant:
    """Planar 3-DOF first-order plant: surge u, sway v, yaw rate r respond to commanded targets."""
    U_MAX, V_MAX, R_MAX = 1.5, 0.7, math.radians(45.0)
    TAU_U, TAU_V, TAU_R = 1.2, 1.0, 0.6

    def __init__(self, lat: float, lon: float, hdg_deg: float, hull: str = "ideal", str_ramp_s: float = 0.0):
        if hull not in HULLS:
            raise ValueError(f"hull {hull!r}: {' | '.join(HULLS)}")
        self.hull, self.str_ramp_s = hull, float(str_ramp_s or 0.0)
        self.r0_dps, self.r05_dps = OMNI_R0_DPS, OMNI_R05_DPS     # v0.1.94: settable (13 Nov water: 6.7 deg/s in place at 6.5 asked, peaks 8.6)
        self.r_sp = 0.0                    # v0.1.85: the steering output after the optional first-order lag (str_ramp_s)
        self.lat0, self.lon0 = lat, lon
        self.x = self.y = 0.0
        self.psi = math.radians(hdg_deg)
        self.u = self.v = self.r = 0.0
        self.k_lat = math.radians(1.0) * R_EARTH
        self.k_lon = math.radians(1.0) * R_EARTH * math.cos(math.radians(lat))

    @property
    def lat(self) -> float:
        return self.lat0 + self.x / self.k_lat

    @property
    def lon(self) -> float:
        return self.lon0 + self.y / self.k_lon

    @property
    def hdg_deg(self) -> float:
        return math.degrees(self.psi) % 360.0

    def to_local(self, lat: float, lon: float):
        return ((lat - self.lat0) * self.k_lat, (lon - self.lon0) * self.k_lon)

    def r_max(self, u: float) -> float:
        """v0.1.85 (WP41 T0.4): the yaw authority at surge u. ideal = R_MAX (45 deg/s, unchanged); omni_usv = the water day's hull (11 Nov):
        <= 2 deg/s in place at full differential, rising linearly to ~4 deg/s at 0.5 m/s (flat above)."""
        if self.hull == "omni_usv":
            return math.radians(self.r0_dps + (self.r05_dps - self.r0_dps) * min(1.0, abs(u) / 0.5))
        return self.R_MAX

    def step(self, dt: float, u_t: float, v_t: float, r_t: float) -> None:
        if self.str_ramp_s > 0.0:          # v0.1.85: the FCU's slow differential ramp (11 Nov: ~25 s to +-200 us) as a first-order lag
            self.r_sp += (r_t - self.r_sp) * min(1.0, dt / self.str_ramp_s)
            r_t = self.r_sp
        if self.hull != "ideal":
            rm = self.r_max(self.u)
            r_t = max(-rm, min(rm, r_t))
        self.u += (u_t - self.u) * min(1.0, dt / self.TAU_U)
        self.v += (v_t - self.v) * min(1.0, dt / self.TAU_V)
        self.r += (r_t - self.r) * min(1.0, dt / self.TAU_R)
        self.psi = (self.psi + self.r * dt) % (2 * math.pi)
        self.x += (self.u * math.cos(self.psi) - self.v * math.sin(self.psi)) * dt
        self.y += (self.u * math.sin(self.psi) + self.v * math.cos(self.psi)) * dt

    @property
    def vn_ve(self):
        return (self.u * math.cos(self.psi) - self.v * math.sin(self.psi),
                self.u * math.sin(self.psi) + self.v * math.cos(self.psi))


def frame_uvr(frame: Sequence[float]) -> Tuple[float, float, float]:
    """A mode-20 frame [m1, m2, m3, m4] -> the plant's (u, v, r) targets (the fake FCU's mapping)."""
    m1, m2, m3, m4 = frame
    u_c = (m1 + m3) / 2.0
    r_c = (m1 - m3) / 2.0
    v_c = (m2 / 0.957 + m4) / 2.0
    return u_c * Plant.U_MAX, v_c * Plant.V_MAX, r_c * Plant.R_MAX


def guided_ur(p: Plant, target: Tuple[float, float], wp_speed: float) -> Tuple[float, float]:
    """GUIDED to a (lat, lon) target: (u, r) targets - turn toward it, slow when far off the bearing, stop within 1 m."""
    u_t = r_t = 0.0
    tx, ty = p.to_local(*target)
    dx, dy = tx - p.x, ty - p.y
    dist = math.hypot(dx, dy)
    if dist > 1.0:
        bearing = math.atan2(dy, dx)
        err = (bearing - p.psi + math.pi) % (2 * math.pi) - math.pi
        r_t = max(-Plant.R_MAX, min(Plant.R_MAX, 1.5 * err))
        u_t = min(wp_speed, 0.5 * dist) if abs(err) < math.radians(60) else 0.15 * wp_speed
    return u_t, r_t


def velocity_ur(vel: Tuple[float, float]) -> Tuple[float, float]:
    """A GUIDED velocity-yaw target (vx, yaw_rate) -> clipped (u, r) targets."""
    u_t = max(-Plant.U_MAX, min(Plant.U_MAX, vel[0]))
    r_t = max(-Plant.R_MAX, min(Plant.R_MAX, vel[1]))
    return u_t, r_t
