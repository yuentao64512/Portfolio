"""fake_guard: what the AGX's ZED guard (cv/buoys_multi/guard_core.py) would publish, from a world file's geometry (v0.1.80, WP36).

The obstacles the detector cannot see: world objects `kind: obstacle` - a vertical cylinder (x_n, y_e, r_m, height_m) or a wall segment
(x_n, y_e, x2_n, y2_e, height_m) - plus every buoy taller than the guard's lower band (a buoy is an obstacle to the guard too: the
scout keeps clear_m 1.6 so it never comes inside stop_m). The camera at the boat's pose: points on each object's outline every 0.1 m,
those inside +- bearing_deg and < slow_m kept; the pixel count is the cluster's angular width x its height in pixels at that range (the
ZED 2i hd2k scaled to the observer's half frame); `frames` consecutive STOP frames as the real filter. Used by the fast SIL (and by
cv/sim_observer for run_sim.sh, publishing local/usv/perception/guard like cv_publish).
v0.1.81.1: `far` as guard_core's - one cluster per world object between slow_m and map_m within +- map_bearing_deg (no occlusion), the
nearest first; buoys included (the boat skips what its map explains)."""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

from . import zed

DEFAULTS = {"stop_m": 1.5, "slow_m": 3.0, "min_px": 400, "frames": 3, "h_cam_m": 1.0, "h_min_m": 0.15, "bearing_deg": 35.0,
            "width_px": 1104, "buoys": True, "map_m": 8.0, "map_bearing_deg": 50.0, "map_min_px": 150, "map_max": 8,
            # v0.1.94 (real_13nov): false obstacles - a near episode (range near_m, frames near_len) starts with near_p per frame;
            # a far phantom cluster (3..map_m) with far_p per frame (13 Nov: SLOW 11 %, BRAKE/BACKOFF/STOP 4 %, 90 phantom depth obstacles)
            "phantom_near_p": 0.0, "phantom_near_m": (1.0, 3.0), "phantom_near_len": (2, 6), "phantom_far_p": 0.0, "phantom_seed": 7}


class FakeGuard:
    def __init__(self, world: dict, cfg: Optional[dict] = None):
        self.c = dict(DEFAULTS)
        self.c.update({k: v for k, v in (cfg or {}).items() if k in DEFAULTS and v is not None})
        cam = world.get("camera") or {}
        if cam.get("height_m") is not None:
            self.c["h_cam_m"] = float(cam["height_m"])
        self.pts: List[Tuple[float, float, float, int]] = []   # (x, y, height above the water, the object's index)
        for oi, o in enumerate(world.get("objects") or []):
            k = o.get("kind")
            if k == "obstacle" and o.get("x2_n") is not None:
                x0, y0, x1, y1 = float(o["x_n"]), float(o["y_e"]), float(o["x2_n"]), float(o["y2_e"])
                n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.1))
                self.pts += [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n, float(o.get("height_m", 0.6)), oi) for i in range(n + 1)]
            elif k == "obstacle" or (k == "buoy" and self.c["buoys"]):
                r = float(o.get("r_m", float(o.get("size_m", 0.5)) / 2.0))
                h = float(o.get("height_m", 0.7 * float(o.get("size_m", 0.5)) if k == "buoy" else 0.6))
                n = max(8, int(2 * math.pi * r / 0.1))
                self.pts += [(float(o["x_n"]) + r * math.cos(2 * math.pi * i / n), float(o["y_e"]) + r * math.sin(2 * math.pi * i / n), h, oi)
                             for i in range(n)]
        self.tan_half_v = math.tan(math.radians(zed.vfov_deg("hd2k") / 2.0))   # v0.1.80: the frame's lower edge (+-15 deg at hd2k)
        _, _, fx, fy, _, _ = zed.LEFT["hd2k"]
        s = float(self.c["width_px"]) / zed.LEFT["hd2k"][0]
        self.fx, self.fy = fx * s, fy * s
        self.n_stop = 0
        import random as _r
        self._prng = _r.Random(int(self.c["phantom_seed"]))
        self._ph = None                                   # the current near phantom: [range, bearing, frames left]
        self.n_phantom_near = self.n_phantom_far = 0

    def frame(self, x_n: float, y_e: float, hdg_deg: float, t: float = 0.0) -> Dict:
        c = self.c
        sel = []
        far: Dict[int, list] = {}
        for (px, py, h, oi) in self.pts:
            if h < c["h_min_m"]:
                continue
            dx, dy = px - x_n, py - y_e
            r = math.hypot(dx, dy)
            if h <= float(c["h_cam_m"]) - r * self.tan_half_v:              # below the frame's lower edge: the camera cannot see it
                continue
            b = (math.degrees(math.atan2(dy, dx)) - hdg_deg + 180.0) % 360.0 - 180.0
            if r >= c["slow_m"]:
                if r < float(c["map_m"]) and abs(b) <= float(c["map_bearing_deg"]):
                    far.setdefault(oi, []).append((r, b, h))
                continue
            if abs(b) <= c["bearing_deg"]:
                sel.append((r, b, h))
        msg = {"t": t, "clear": True, "frames": 0, "h_m": c["h_cam_m"], "source": "fake_guard"}
        pr = self._prng
        if self._ph is None and float(c["phantom_near_p"]) > 0 and pr.random() < float(c["phantom_near_p"]):
            lo, hi = c["phantom_near_m"]
            self._ph = [pr.uniform(lo, hi), pr.uniform(-25, 25), pr.randint(*c["phantom_near_len"])]
            self.n_phantom_near += 1
        if self._ph is not None:                          # the phantom joins the frame's candidates (a 0.4 m tall patch, 6 deg wide)
            r0, b0, _ = self._ph
            sel += [(r0, b0 + d, 0.4) for d in (-3.0, 0.0, 3.0)]
            self._ph[2] -= 1
            if self._ph[2] <= 0:
                self._ph = None
        if sel:
            rs = sorted(s[0] for s in sel)
            near = rs[max(0, int(0.05 * len(rs)) - 0)]
            b0, b1 = min(s[1] for s in sel), max(s[1] for s in sel)
            width = max(1.0, math.radians(b1 - b0) * self.fx)
            height = self.fy * min(max(s[2] for s in sel), 1.5) / max(0.3, near)
            n_px = int(width * height)
            if n_px >= c["min_px"]:
                bn = min(sel, key=lambda s: s[0])[1]
                msg.update(clear=False, range_m=round(near, 2), bearing_deg=round(bn, 1), bearing_min_deg=round(b0, 1),
                           bearing_max_deg=round(b1, 1), px=n_px, zone="stop" if near < c["stop_m"] else "slow")
        if float(c["phantom_far_p"]) > 0 and pr.random() < float(c["phantom_far_p"]):
            r0, b0 = pr.uniform(float(c["slow_m"]), float(c["map_m"])), pr.uniform(-float(c["map_bearing_deg"]), float(c["map_bearing_deg"]))
            far[-1 - self.n_phantom_far] = [(r0, b0 - 4.0, 0.5), (r0, b0, 0.5), (r0, b0 + 4.0, 0.5)]
            self.n_phantom_far += 1
        fl = []
        for pts in far.values():                                          # v0.1.81.1: one cluster per object, 3-8 m ahead
            near = min(p[0] for p in pts)
            b0, b1 = min(p[1] for p in pts), max(p[1] for p in pts)
            n_px = int(max(1.0, math.radians(b1 - b0) * self.fx) * self.fy * min(max(p[2] for p in pts), 1.5) / max(0.3, near))
            if n_px >= int(c["map_min_px"]):
                fl.append({"range_m": round(near, 2), "bearing_deg": round(min(pts)[1], 1), "bearing_min_deg": round(b0, 1),
                           "bearing_max_deg": round(b1, 1), "px": n_px})
        if fl:
            msg["far"] = sorted(fl, key=lambda d: d["range_m"])[: int(c["map_max"])]
        if not msg["clear"] and msg["zone"] == "stop":
            self.n_stop += 1
        else:
            self.n_stop = 0
        msg["frames"] = self.n_stop
        if not msg["clear"] and msg["zone"] == "stop" and self.n_stop < int(c["frames"]):
            msg["zone"] = "slow"
        return msg
