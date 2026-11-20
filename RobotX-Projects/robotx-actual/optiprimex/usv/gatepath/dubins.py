"""Dubins paths (pathing_sim spec 6.6): the shortest of LSL RSR LSR RSL RLR LRL between two poses with turn radius rho.

Frame: the local N/E frame, headings are compass radians (0 = north, + = clockwise), so a point is (x_n, y_e) and the unit
heading is (cos h, sin h). The spec's maths is written for a counter-clockwise (x right, y up) plane: we map N/E to that
plane as (x, y) = (y_e, x_n) and compass h to math angle pi/2 - h, solve there, and map back.
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

TWO_PI = 2.0 * math.pi


def mod2pi(a: float) -> float:
    return a - TWO_PI * math.floor(a / TWO_PI)


def _words(alpha: float, beta: float, d: float):
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    out = []
    p2 = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p2 >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out.append(("LSL", mod2pi(-alpha + tmp), math.sqrt(p2), mod2pi(beta - tmp)))
    p2 = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p2 >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out.append(("RSR", mod2pi(alpha - tmp), math.sqrt(p2), mod2pi(-beta + tmp)))
    p2 = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out.append(("LSR", mod2pi(-alpha + tmp), p, mod2pi(-mod2pi(beta) + tmp)))
    p2 = d * d - 2 + 2 * c_ab - 2 * d * (sa + sb)
    if p2 >= 0:
        p = math.sqrt(p2)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out.append(("RSL", mod2pi(alpha - tmp), p, mod2pi(beta - tmp)))
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1:
        p = mod2pi(TWO_PI - math.acos(tmp))
        t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
        out.append(("RLR", t, p, mod2pi(alpha - beta - t + mod2pi(p))))
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp) <= 1:
        p = mod2pi(TWO_PI - math.acos(tmp))
        t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out.append(("LRL", t, p, mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))))
    return out


class DubinsPath:
    def __init__(self, kind: str, t: float, p: float, q: float, rho: float, start: Tuple[float, float, float]):
        self.kind, self.t, self.p, self.q, self.rho, self.start = kind, t, p, q, rho, start   # start in the math plane

    @property
    def length(self) -> float:
        return (self.t + self.p + self.q) * self.rho

    @property
    def turn(self) -> float:
        """Total turning angle (rad): t + q for CSC, t + p + q for CCC (spec 6.6)."""
        return self.t + self.p + self.q if self.kind in ("RLR", "LRL") else self.t + self.q

    @property
    def ccc(self) -> bool:
        return self.kind in ("RLR", "LRL")

    def sample(self, step: float = 0.25) -> List[Tuple[float, float]]:
        """Points every `step` metres along the path, in the N/E frame (start and end included)."""
        x, y, h = self.start
        pts = [(x, y)]
        rho = self.rho
        for seg, ang in zip(self.kind, (self.t, self.p, self.q)):
            L = ang * rho
            n = max(1, int(math.ceil(L / step)))
            if seg == "S":
                for i in range(1, n + 1):
                    l = L * i / n
                    pts.append((x + l * math.cos(h), y + l * math.sin(h)))
                x, y = x + L * math.cos(h), y + L * math.sin(h)
            else:
                s = 1.0 if seg == "L" else -1.0
                cx, cy = x - s * rho * math.sin(h), y + s * rho * math.cos(h)
                for i in range(1, n + 1):
                    a = h + s * (L * i / n) / rho
                    pts.append((cx + s * rho * math.sin(a), cy - s * rho * math.cos(a)))
                h2 = h + s * ang
                x, y, h = cx + s * rho * math.sin(h2), cy - s * rho * math.cos(h2), h2
        return [(py, px) for px, py in pts]          # math plane (x = east, y = north) -> (x_n, y_e)


def _to_plane(x_n: float, y_e: float, hdg: float) -> Tuple[float, float, float]:
    return (y_e, x_n, math.pi / 2.0 - hdg)


def shortest(start: Tuple[float, float, float], goal: Tuple[float, float, float], rho: float) -> Optional[DubinsPath]:
    """start / goal = (x_n, y_e, compass heading rad). None when start == goal."""
    x0, y0, t0 = _to_plane(*start)
    x1, y1, t1 = _to_plane(*goal)
    dx, dy = x1 - x0, y1 - y0
    D = math.hypot(dx, dy)
    if D < 1e-9 and abs(mod2pi(t1 - t0)) < 1e-9:
        return None
    d = D / rho
    th = math.atan2(dy, dx) if D > 1e-12 else 0.0
    alpha, beta = mod2pi(t0 - th), mod2pi(t1 - th)
    best = None
    for kind, t, p, q in _words(alpha, beta, d):
        if best is None or t + p + q < best[1] + best[2] + best[3]:
            best = (kind, t, p, q)
    if best is None:
        return None
    return DubinsPath(best[0], best[1], best[2], best[3], rho, (x0, y0, t0))
