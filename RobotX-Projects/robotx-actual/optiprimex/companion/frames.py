"""The only place where lat/lon <-> local happens (ADR-4). From pool_test2.Origin.

Local frame: control-origin tangent plane, x_n north, y_e east, metres. Headings NED 0-360 CW.
"""
from __future__ import annotations

import bisect
import math
from collections import deque
from typing import Deque, Optional, Tuple

from .state import FcuState, LocalPose

R_EARTH = 6_378_137.0


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def wrap360(a: float) -> float:
    return a % 360.0


def dist_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def bearing_deg(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    """Bearing from local point a to b, NED degrees 0-360 (x_n, y_e)."""
    return wrap360(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))


class ControlOrigin:
    def __init__(self, lat: float, lon: float):
        self.lat = float(lat)
        self.lon = float(lon)
        self._k_lat = math.radians(1.0) * R_EARTH
        self._k_lon = math.radians(1.0) * R_EARTH * math.cos(math.radians(self.lat))

    def to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        return ((lat - self.lat) * self._k_lat, (lon - self.lon) * self._k_lon)

    def from_local(self, x_n: float, y_e: float) -> Tuple[float, float]:
        return (self.lat + x_n / self._k_lat, self.lon + y_e / self._k_lon)

    def pose_from_fcu(self, fcu: FcuState) -> Optional[LocalPose]:
        if fcu.lat is None or fcu.lon is None or fcu.hdg_deg is None:
            return None
        x, y = self.to_local(fcu.lat, fcu.lon)
        return LocalPose(x_n=x, y_e=y, hdg_deg=fcu.hdg_deg, co_lat=self.lat, co_lon=self.lon,
                         t=fcu.t_mono, t_utc=fcu.t_utc, sog_mps=fcu.sog_mps or 0.0)


def body_to_local(range_m: float, bearing_body_deg: float, pose: LocalPose,
                  cam_forward_m: float = 0.0, cam_right_m: float = 0.0, cam_yaw_deg: float = 0.0) -> Tuple[float, float]:
    """Camera-frame polar observation -> local (x_n, y_e). bearing_body positive to starboard."""
    cam_yaw = math.radians(cam_yaw_deg)
    fwd = range_m * math.cos(math.radians(bearing_body_deg))
    right = range_m * math.sin(math.radians(bearing_body_deg))
    body_f = cam_forward_m + fwd * math.cos(cam_yaw) - right * math.sin(cam_yaw)
    body_r = cam_right_m + fwd * math.sin(cam_yaw) + right * math.cos(cam_yaw)
    psi = math.radians(pose.hdg_deg)
    north = body_f * math.cos(psi) - body_r * math.sin(psi)
    east = body_f * math.sin(psi) + body_r * math.cos(psi)
    return (pose.x_n + north, pose.y_e + east)


class PoseBuffer:
    """Time-indexed poses; at(t) interpolates position and heading (shortest arc)."""

    def __init__(self, maxlen: int = 200):
        self._t: Deque[float] = deque(maxlen=maxlen)
        self._p: Deque[LocalPose] = deque(maxlen=maxlen)

    def push(self, pose: LocalPose) -> None:
        if self._t and pose.t <= self._t[-1]:
            return
        self._t.append(pose.t)
        self._p.append(pose)

    def latest(self) -> Optional[LocalPose]:
        return self._p[-1] if self._p else None

    def at(self, t: float) -> Optional[LocalPose]:
        if not self._t:
            return None
        ts = list(self._t)
        i = bisect.bisect_left(ts, t)
        if i == 0:
            return self._p[0]
        if i >= len(ts):
            return self._p[-1]
        a, b = self._p[i - 1], self._p[i]
        f = (t - a.t) / max(b.t - a.t, 1e-6)
        hdg = wrap360(a.hdg_deg + f * wrap180(b.hdg_deg - a.hdg_deg))
        return LocalPose(x_n=a.x_n + f * (b.x_n - a.x_n), y_e=a.y_e + f * (b.y_e - a.y_e), hdg_deg=hdg,
                         co_lat=a.co_lat, co_lon=a.co_lon, t=t, t_utc=a.t_utc, sog_mps=a.sog_mps)
