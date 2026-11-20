"""Mode-20 (CUSTOM_MANUAL) helpers shared by the direct-thruster behaviours: the companion allocation with the calibrated
stern factors (usv_ui `sway_lat`, pool 2026-10-21) and the heading-hold yaw term. Pure functions."""
from __future__ import annotations

import math

from ...companion.state import ThrusterFrame


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def sway_lat(gains, v: float):
    """(M2 stern, M4 bow) for a sway command v (+ = starboard): M2 = F v, M4 = v; F > 1 lowers M4 so neither exceeds |v|."""
    f = gains.sway_f_pos if v >= 0 else gains.sway_f_neg
    return (f * v, v) if f <= 1.0 else (v, v / f)


def yaw_cmd(gains, err_deg: float) -> float:
    """+ = turn clockwise (M1 > M3)."""
    return max(-gains.sway_rmax, min(gains.sway_rmax, gains.sway_kpsi * err_deg / 45.0))


def frame(gains, max_amp: float, u: float, v: float, r: float) -> ThrusterFrame:
    """surge u, sway v, yaw r -> M1 = u + r, M2 / M4 from sway_lat, M3 = u - r, jointly scaled to max_amp.
    v0.1.85 (WP41 T0.5): + the side pair's yaw couple M2 += k r, M4 -= k r with k = gains.yaw_side_gain (profile; default 0 = unchanged)."""
    m2, m4 = sway_lat(gains, v)
    k = float(getattr(gains, "yaw_side_gain", 0.0) or 0.0)
    if k:
        m2, m4 = m2 + k * r, m4 - k * r
    cmd = [u + r, m2, u - r, m4]
    mx = max(abs(x) for x in cmd)
    if mx > max_amp > 0:
        cmd = [x * max_amp / mx for x in cmd]
    return ThrusterFrame(*cmd)


def unit_hdg(hdg_deg: float):
    h = math.radians(hdg_deg)
    return math.cos(h), math.sin(h)
