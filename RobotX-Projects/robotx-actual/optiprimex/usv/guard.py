"""guard: the companion's side of the ZED proximity guard (v0.1.80, WP36; docs/SCOUT_BRIEF_2026-11-11.md §4.3, Dmitrii's decision: brake ->
back off 2 m -> re-plan on its own).

The AGX observer publishes `guard` messages (cv/buoys_multi/guard_core.py via cv_publish -> local/usv/perception/guard) at the frame rate:
{clear: true} or {range_m, bearing_deg, bearing_min_deg, bearing_max_deg, px, frames, zone slow | stop}. The mission manager feeds them
here and passes every task command through `filter()`, in every moving stage of every task (goto too); muted in WAIT_* and LOG and while
the command is a hold:
  - silence > lost_s (once required, or after the first message) -> the speed cap slow_v + CAUTION "guard lost";
  - zone slow (<= slow_m) -> the leg's speed capped at slow_v;
  - zone stop (<= stop_m for `frames` frames) -> BRAKE (a mode-20 frame, surge -brake_surge for brake_s, or until the plant reports
    <= 0.02 m/s) -> BACKOFF (straight astern on mode 20 at backoff_surge until backoff_m travelled or backoff_max_s; blind: no stern
    sensor) -> the obstacle into the map (`kind obstacle`, ring_m at the nearest point and at both bearing ends when the extent exceeds
    8 deg) -> the task's behaviour is restarted, so the leg is re-planned round the ring;
  - the 2nd trigger on the same target -> task.on_guard_block() (the scout logs the buoy blocked and picks on);
  - the max_repeats-th trigger within repeat_r_m of earlier ones -> AUTO_SAFE_HOLD + CAUTION "obstacle persists at ...".
The guard never arms, never changes the FCU mode by itself beyond what a task already uses (GUIDED carrots / mode-20 frames).

v0.1.81.1 (Dmitrii 2026-11-11: obstacles on the USV map, "a 2-3 m circle behind that area" -> 2 m, centred 1 m behind): every ring is
ring_m 2.0 centred ring_behind_m 1.0 past the seen face along the line of sight (the camera sees the front face only); map_far() maps
the observer's `far` clusters (3-8 m ahead, where a low object is still in the frame) at map_hz (2) as `source depth` obstacles -
any vehicle state, the boat need not be in a task (a manual RC run maps too); zed_guard.map false = off.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Tuple

from ..companion.state import GuidedTarget, Hold, ThrusterFrame, VelocityYawTarget

DEFAULTS = {"memory_s": 20.0, "enabled": True, "require": False, "stop_m": 1.5, "slow_m": 3.0, "slow_v": 0.3, "brake_s": 1.5, "brake_surge": 0.4,
            "backoff_m": 2.0, "backoff_surge": 0.3, "backoff_max_s": 10.0, "lost_s": 1.5, "max_repeats": 3, "repeat_r_m": 3.0,
            "ring_m": 2.0, "ring_behind_m": 1.0, "map": True, "map_hz": 2.0, "extent_deg": 8.0, "min_px": 400, "frames": 3, "h_cam_m": 1.0, "bearing_deg": 35.0,
            "stop_rule": "corridor", "stop_lat_m": 0.7, "slow_lat_m": 1.2,
            # v0.1.92 (Cowork addendum 4/6, Dmitrii "the buoy seems very close to the side but we pass just nicely"): a cluster the MAP knows as
            # a buoy (within mapped_m of a mapped buoy) stops the boat only on a collision course; unknown objects keep the corridor above
            "mapped_m": 0.8, "stop_mapped_m": 1.0, "stop_mapped_lat": 0.45, "slow_mapped_m": 2.0, "slow_mapped_lat": 1.0,
            # v0.1.94 (fork B's sim, 80 runs a cell: SAFE_HOLD "obstacle persists" on the gate's own buoys = most failures - a 2.5 m gate puts
            # them ~1.25 m abeam, inside STOP's reach once the map position is off): while the task passes a gate (task.gate_pass_xy()), a cluster
            # within gate_r_m of one of its two buoys stops only on a collision course (gate_stop_m ahead, gate_stop_lat abeam), never slows,
            # and the memory near them does not count
            "gate_r_m": 1.0, "gate_stop_m": 0.8, "gate_stop_lat": 0.3}   # v0.1.92: the hull's corridor (Cowork consult 2026-11-13)
MUTED = ("LOG",)


class GuardSupervisor:
    def __init__(self, cfg: Optional[dict] = None):
        self.p = dict(DEFAULTS)
        self.p.update({k: v for k, v in (cfg or {}).items() if k in DEFAULTS and v is not None})
        self.msg: Optional[dict] = None
        self.msg_t: Optional[float] = None
        self.mode = "CLEAR"                       # CLEAR | BRAKE | BACKOFF
        self.t_mode = 0.0
        self.start_xy: Optional[Tuple[float, float]] = None
        self.triggers: List[Dict[str, Any]] = []  # {t, x, y, key}
        self.per_key: Dict[Any, int] = {}
        self.safe_hold: Optional[str] = None      # read (and cleared) by the caller
        self.caution: Optional[str] = None
        self.last_view: Dict[str, Any] = {}
        self.events: List[Dict[str, Any]] = []    # obstacle lines (the scout log, the tests)
        self.far_t = -1e9                         # v0.1.81.1: the last map_far pass
        self.n_far = 0                            # depth rings handed to the map (status / tests)
        self.seen: List[Dict[str, Any]] = []      # v0.1.80: SLOW sightings as local points {x, y, t} - the frame's lower edge hides a
                                                  # low object inside ~2 m (ZED +-15 deg, 1.0 m up): remembered, STOP when it comes in

    # ---- input -------------------------------------------------------------------------------------------------------------
    def on_message(self, msg: dict, now: float, pmap=None) -> None:
        if isinstance(msg, dict):
            self.msg, self.msg_t = msg, now
            if isinstance(msg.get("pipe_ms"), (int, float)):        # v0.1.86.8 (WP43 §0): the latency ring (the last 60 messages)
                import time as _t
                from collections import deque
                if not hasattr(self, "lat"):
                    self.lat = deque(maxlen=60)
                tp = msg.get("t_pub")
                self.lat.append((float(msg["pipe_ms"]), (_t.time() - float(tp)) * 1000.0 if isinstance(tp, (int, float)) else None))
            if pmap is not None:                         # the map carries the last message for the tasks (m0t9 reads it)
                pmap.guard_last = (msg, now)
                self.pmap = pmap                         # v0.1.92: the mapped-buoy rule

    def latency(self):
        """v0.1.86.8: {pipe_ms_p50, pipe_ms_p95, net_ms_p50, n} - pipe = t_pub - t_cam on the AGX clock; net = this NX's receive - t_pub (the
        two clocks' offset included: read it with state.clocks, never corrected here). None before the first guard message with pipe_ms."""
        d = list(getattr(self, "lat", []) or [])
        if not d:
            return None
        p = sorted(x for x, _ in d)
        n = sorted(y for _, y in d if y is not None)
        q = lambda s, f: round(s[min(len(s) - 1, int(f * (len(s) - 1) + 0.5))], 1) if s else None
        return {"pipe_ms_p50": q(p, 0.5), "pipe_ms_p95": q(p, 0.95), "net_ms_p50": q(n, 0.5), "n": len(d)}

    def map_far(self, msg: dict, pose, pmap, now: float) -> int:
        """v0.1.81.1: the observer's `far` clusters -> depth obstacles in the map (a ring_m ring ring_behind_m past each seen face; a wide
        cluster - a pontoon, a boom - gets rings along its arc, <= ring_m apart, at most 6). Returns the rings handed over."""
        if not (self.p["enabled"] and self.p["map"]) or pose is None or pmap is None or not hasattr(pmap, "add_obstacle"):
            return 0
        far = (msg or {}).get("far") or []
        if not far or now - self.far_t < 1.0 / max(0.1, float(self.p["map_hz"])):
            return 0
        self.far_t = now
        fused = pmap.fuse_depth(msg, pose, now) if getattr(pmap, "depth_fuse_tol_deg", 0) else set()   # v0.1.94: a depthless buoy's range
        ring, back, n = float(self.p["ring_m"]), float(self.p["ring_behind_m"]), 0
        for i, c in enumerate(far):
            if i in fused:
                continue
            try:
                r, b = float(c["range_m"]), float(c["bearing_deg"])
            except (KeyError, TypeError, ValueError):
                continue
            bs = [b]
            b0, b1 = c.get("bearing_min_deg"), c.get("bearing_max_deg")
            if b0 is not None and b1 is not None and float(b1) - float(b0) > float(self.p["extent_deg"]):
                span = math.radians(float(b1) - float(b0)) * r                       # the arc's length at its range
                k = min(5, max(1, int(math.ceil(span / ring))))
                bs = sorted({b} | {float(b0) + (float(b1) - float(b0)) * i / k for i in range(k + 1)})
            for bb in bs:
                face = self._point(pose, r, bb)
                cx, cy = self._point(pose, r + back, bb)
                if pmap.add_obstacle(cx, cy, ring, now, source="depth", face=face) is not None:
                    n += 1
        self.n_far += n
        return n

    def _ring_xy(self, pose, x: float, y: float) -> Tuple[float, float]:
        """v0.1.81.1: a seen face point -> its ring's centre, ring_behind_m further along the line of sight."""
        dx, dy = x - pose.x_n, y - pose.y_e
        d = math.hypot(dx, dy) or 1.0
        back = float(self.p["ring_behind_m"])
        return x + back * dx / d, y + back * dy / d

    @property
    def busy(self) -> bool:
        return self.mode in ("BRAKE", "BACKOFF")

    def lost(self, now: float) -> bool:
        if not self.p["enabled"]:
            return False
        if self.msg_t is None:
            return bool(self.p["require"])
        return now - self.msg_t > float(self.p["lost_s"])

    # ---- the filter ------------------------------------------------------------------------------------------------------------
    @staticmethod
    def moving(stage: str, cmd) -> bool:
        if stage.startswith("WAIT") or stage in MUTED:
            return False
        if isinstance(cmd, GuidedTarget):
            return True
        if isinstance(cmd, VelocityYawTarget):
            return abs(cmd.vx_mps) > 0.01
        if isinstance(cmd, ThrusterFrame):
            return any(abs(v) > 0.02 for v in cmd.as_list())
        return False

    def _cap(self, cmd):
        v = float(self.p["slow_v"])
        if isinstance(cmd, GuidedTarget):
            return GuidedTarget(cmd.lat, cmd.lon, min(v, cmd.speed_mps) if cmd.speed_mps else v)
        if isinstance(cmd, VelocityYawTarget):
            return VelocityYawTarget(max(-v, min(v, cmd.vx_mps)), cmd.yaw_rate_rad_s)
        return cmd

    def view(self, now: float) -> Dict[str, Any]:
        m = self.msg or {}
        age = None if self.msg_t is None else round(now - self.msg_t, 1)
        if not self.p["enabled"]:
            chip = "off"
        elif self.lost(now):
            chip = "lost"
        elif self.busy:
            chip = self.mode
        elif m.get("clear", True):
            chip = "clear"
        else:
            chip = "STOP" if m.get("zone") == "stop" else "SLOW"
        return {"chip": chip, "mode": self.mode, "range_m": m.get("range_m") if not m.get("clear", True) else None, "age_s": age,
                "triggers": len(self.triggers), "caution": self.caution}

    def filter(self, task, cmd, ctx) -> Tuple[Any, Dict[str, Any]]:
        """The task's command -> the command to send, and the guard's view. May set self.safe_hold (the caller puts the vehicle in V5)."""
        now, pose = ctx.b.now, ctx.b.pose
        stage = str(getattr(getattr(task, "m", None), "state", "") or "")
        self.caution = None
        if not self.p["enabled"] or pose is None or getattr(task, "guard_exempt", False):   # m0t9 runs its own pulse
            return cmd, self.view(now)
        if self.busy:
            return self._manoeuvre(task, ctx), self.view(now)
        if not self.moving(stage, cmd):
            return cmd, self.view(now)
        if self.lost(now):
            self.caution = "guard lost: speed capped at %.1f m/s" % float(self.p["slow_v"])
            return self._cap(cmd), self.view(now)
        m = self.msg or {}
        if not m.get("clear", True):                    # remember where it is (the camera may lose it under the frame closer in)
            for b in {m.get("bearing_deg"), m.get("bearing_min_deg"), m.get("bearing_max_deg")} - {None}:
                x, y = self._point(pose, float(m.get("range_m") or self.p["slow_m"]), float(b))
                self.seen.append({"x": x, "y": y, "t": now})
        self.seen = [q for q in self.seen if now - q["t"] <= float(self.p["memory_s"])][-60:]
        mapped = self.is_mapped(pose, m) if not m.get("clear", True) else False
        gate = task.gate_pass_xy() if hasattr(task, "gate_pass_xy") else None
        if gate:
            gr = float(self.p["gate_r_m"])
            self.seen = [q for q in self.seen if all(math.hypot(q["x"] - g[0], q["y"] - g[1]) > gr for g in gate)]
            if not m.get("clear", True) and any(self._near_point(pose, m, g, gr) for g in gate):
                fwd, lat = self._fwd_lat(m)
                if m.get("zone") == "stop" and int(m.get("frames", 0) or 0) >= int(self.p["frames"]) and fwd <= float(self.p["gate_stop_m"]) \
                        and lat <= float(self.p["gate_stop_lat"]):
                    return self._trigger(task, ctx, m), self.view(now)
                return cmd, self.view(now)
        sq = getattr(task, "squeeze_xy", None)
        if sq is not None and not m.get("clear", True) and self._near_point(pose, m, sq, 1.0):     # v0.1.93: the squeezed object only
            if m.get("zone") == "stop" and int(m.get("frames", 0) or 0) >= int(self.p["frames"]) and self._fwd_lat(m)[0] <= float(
                    getattr(task, "p", {}).get("squeeze_stop_m", 0.3)) and self._fwd_lat(m)[1] <= float(self.p["stop_mapped_lat"]):
                return self._trigger(task, ctx, m), self.view(now)
            return self._cap(cmd), self.view(now)
        if m.get("zone") == "stop" and int(m.get("frames", 0) or 0) >= int(self.p["frames"]) and self.in_corridor(m, "stop", mapped):
            return self._trigger(task, ctx, m), self.view(now)
        near = self._remembered(pose)
        if near is not None:                            # a remembered point now inside stop_m ahead: STOP as if seen
            return self._trigger(task, ctx, near), self.view(now)
        seen_slow = not m.get("clear", True) and self.in_corridor(m, "slow", mapped)
        if not seen_slow and not self._remembered(pose, float(self.p["slow_m"])):
            return cmd, self.view(now)
        return self._cap(cmd), self.view(now)

    def _fwd_lat(self, m: dict):
        r = float(m.get("range_m") or 0.0)
        bs = [float(b) for b in (m.get("bearing_deg"), m.get("bearing_min_deg"), m.get("bearing_max_deg")) if b is not None] or [0.0]
        lo, hi = min(bs), max(bs)
        b = 0.0 if lo <= 0.0 <= hi else (lo if abs(lo) < abs(hi) else hi)
        return r * math.cos(math.radians(b)), abs(r * math.sin(math.radians(b)))

    def _near_point(self, pose, m: dict, xy, within: float) -> bool:
        if pose is None or m.get("range_m") is None:
            return False
        bs = [float(b) for b in (m.get("bearing_deg"), m.get("bearing_min_deg"), m.get("bearing_max_deg")) if b is not None] or [0.0]
        lo, hi = min(bs), max(bs)
        b = 0.0 if lo <= 0.0 <= hi else (lo if abs(lo) < abs(hi) else hi)
        x, y = self._point(pose, float(m["range_m"]), b)
        return math.hypot(x - xy[0], y - xy[1]) <= within

    def is_mapped(self, pose, m: dict) -> bool:
        """v0.1.92: the cluster's nearest point lies within mapped_m of a buoy the map knows (any colour, stale too)."""
        pm = getattr(self, "pmap", None)
        if pm is None or pose is None or m.get("range_m") is None:
            return False
        bs = [float(b) for b in (m.get("bearing_deg"), m.get("bearing_min_deg"), m.get("bearing_max_deg")) if b is not None]
        if not bs:
            return False
        lo, hi = min(bs), max(bs)
        b = 0.0 if lo <= 0.0 <= hi else (lo if abs(lo) < abs(hi) else hi)
        x, y = self._point(pose, float(m["range_m"]), b)
        try:
            ents = pm.entities("buoy", include_stale=True)
        except TypeError:
            ents = pm.entities("buoy")
        return any(math.hypot(e.x_n - x, e.y_e - y) <= float(self.p["mapped_m"]) + 0.25 for e in ents)

    def in_corridor(self, m: dict, zone: str, mapped: bool = False) -> bool:
        """v0.1.92 (Cowork consult 2026-11-13; the overnight sim: the +-35 deg cone stops the boat in every 2-3 m gate - a gate buoy 1.0-1.25 m
        abeam enters the cone at 1.5-1.7 m with 0.3 m of tracking error): the cluster's nearest point (the bearing end closest to the bow) as
        forward / lateral; STOP when forward <= stop_m and |lateral| <= stop_lat_m (half the beam + 0.32), SLOW when forward <= slow_m and
        |lateral| <= slow_lat_m. stop_rule cone = v0.1.91 (the cone alone, the AGX's own window)."""
        if str(self.p.get("stop_rule")) == "cone":
            return True
        r = float(m.get("range_m") or 0.0)
        bs = [float(b) for b in (m.get("bearing_deg"), m.get("bearing_min_deg"), m.get("bearing_max_deg")) if b is not None]
        if not bs:
            return True
        lo, hi = min(bs), max(bs)
        b = 0.0 if lo <= 0.0 <= hi else (lo if abs(lo) < abs(hi) else hi)
        fwd, lat = r * math.cos(math.radians(b)), abs(r * math.sin(math.radians(b)))
        if mapped:                                                       # v0.1.92: a mapped buoy - the planner's business
            if zone == "stop":
                return fwd <= float(self.p["stop_mapped_m"]) and lat <= float(self.p["stop_mapped_lat"])
            return fwd <= float(self.p["slow_mapped_m"]) and lat <= float(self.p["slow_mapped_lat"])
        if zone == "stop":
            return fwd <= float(self.p["stop_m"]) and lat <= float(self.p["stop_lat_m"])
        return fwd <= float(self.p["slow_m"]) and lat <= float(self.p["slow_lat_m"])

    def _remembered(self, pose, within: Optional[float] = None) -> Optional[dict]:
        """The nearest remembered sighting ahead (+- bearing_deg) within `within` (stop_m) - as a guard message, else None."""
        lim = float(self.p["stop_m"] if within is None else within)
        best = None
        for q in self.seen:
            r = math.hypot(q["x"] - pose.x_n, q["y"] - pose.y_e)
            b = (math.degrees(math.atan2(q["y"] - pose.y_e, q["x"] - pose.x_n)) - pose.hdg_deg + 180.0) % 360.0 - 180.0
            fwd, lat = r * math.cos(math.radians(b)), abs(r * math.sin(math.radians(b)))
            lat_lim = float(self.p["stop_lat_m"] if within is None else self.p["slow_lat_m"])
            ok = (abs(b) <= float(self.p["bearing_deg"]) if str(self.p.get("stop_rule")) == "cone" else (fwd > 0 and lat <= lat_lim))
            if r < lim and ok and (best is None or r < best["range_m"]):
                best = {"clear": False, "zone": "stop", "range_m": round(r, 2), "bearing_deg": round(b, 1), "frames": int(self.p["frames"]),
                        "remembered": True}
        return best

    # ---- a trigger ---------------------------------------------------------------------------------------------------------------
    def _point(self, pose, r: float, b: float) -> Tuple[float, float]:
        h = math.radians(pose.hdg_deg + b)
        return pose.x_n + r * math.cos(h), pose.y_e + r * math.sin(h)

    def _trigger(self, task, ctx, m: dict):
        now, pose = ctx.b.now, ctx.b.pose
        r = float(m.get("range_m") or self.p["stop_m"])
        b = float(m.get("bearing_deg") or 0.0)
        x, y = self._point(pose, r, b)
        key = task.guard_target() if hasattr(task, "guard_target") else (getattr(task, "name", "?"), getattr(task.m, "state", "?"))
        txy = task.guard_target_xy() if hasattr(task, "guard_target_xy") else None
        if txy is not None and math.hypot(x - txy[0], y - txy[1]) > float(txy[2]):
            key = ("intermediate",) + tuple(key)      # v0.1.85 (WP41 T0.2e): a stop far from the target re-plans, it does not block the target
        self.per_key[key] = self.per_key.get(key, 0) + 1
        near = [t for t in self.triggers if math.hypot(t["x"] - x, t["y"] - y) <= float(self.p["repeat_r_m"])]
        self.triggers.append({"t": now, "x": x, "y": y, "key": key})
        rings = [(x, y)]
        b0, b1 = m.get("bearing_min_deg"), m.get("bearing_max_deg")
        if b0 is not None and b1 is not None and float(b1) - float(b0) > float(self.p["extent_deg"]):
            rings += [self._point(pose, r, float(b0)), self._point(pose, r, float(b1))]
        for q in self.seen:                             # the remembered sightings near the boat: the rest of the thing (a boom, a pontoon)
            if math.hypot(q["x"] - pose.x_n, q["y"] - pose.y_e) <= float(self.p["slow_m"]) and \
                    all(math.hypot(q["x"] - p0[0], q["y"] - p0[1]) > 0.7 for p0 in rings) and len(rings) < 6:
                rings.append((q["x"], q["y"]))
        if ctx.b.map is not None and hasattr(ctx.b.map, "add_obstacle"):
            for q in rings:                             # v0.1.81.1: the ring ring_behind_m past the face
                ctx.b.map.add_obstacle(*self._ring_xy(pose, q[0], q[1]), float(self.p["ring_m"]), now)
        o = ctx.b.origin
        lat, lon = o.from_local(x, y) if o is not None else (None, None)
        ev = {"ev": "obstacle", "lat": None if lat is None else round(lat, 7), "lon": None if lon is None else round(lon, 7),
              "range_m": round(r, 2), "bearing_deg": round(b, 1), "px": m.get("px"), "rings": len(rings), "stage": getattr(task.m, "state", None)}
        self.seen = []                                  # the obstacle is in the map now (its ring): the planner keeps away from it
        if m.get("remembered"):
            ev["remembered"] = True
        if len(near) + 1 >= int(self.p["max_repeats"]):
            self.safe_hold = f"guard: obstacle persists at N {x:+.1f} E {y:+.1f} ({len(near) + 1} triggers within {self.p['repeat_r_m']:.0f} m)"
            ev["action"] = "safe_hold"
            self._event(task, ctx, ev)
            return Hold()
        ev["action"] = "brake"
        if self.per_key[key] >= 2:
            ev["blocked"] = True
        self._event(task, ctx, ev)
        if self.per_key[key] >= 2 and hasattr(task, "on_guard_block"):
            task.on_guard_block(ctx, key)
        self.mode, self.t_mode, self.start_xy = "BRAKE", now, (pose.x_n, pose.y_e)
        ctx.log(f"guard STOP at {r:.2f} m {b:+.0f} deg in {getattr(task.m, 'state', '?')}: BRAKE -> BACKOFF {self.p['backoff_m']:.0f} m -> re-plan "
                f"(trigger {self.per_key[key]} on this target)")
        return self._brake_frame()

    def _event(self, task, ctx, ev: dict) -> None:
        self.events.append(ev)
        if hasattr(task, "on_guard"):
            task.on_guard(ctx, ev)

    def _brake_frame(self) -> ThrusterFrame:
        s = -float(self.p["brake_surge"])
        return ThrusterFrame(s, 0.0, s, 0.0)

    def astern_free(self, ctx) -> float:
        """v0.1.92 (the overnight sim: the blind 2 m BACKOFF reversed into the gate just passed, 2 contacts in 40 runs): how far the boat
        may go straight astern before its stern meets a MAPPED object (buoys of any colour, stale too, docks, boxes) - the stern corridor
        half the beam + the object + 0.3 m wide; >= 0.3 m (a nudge), <= backoff_m."""
        pose, m = ctx.b.pose, getattr(ctx.b, "map", None)
        full = float(self.p["backoff_m"])
        if pose is None or m is None:
            return full
        from .gatepath import planner as gp
        h = math.radians(pose.hdg_deg)
        free = full
        for (x, y, r) in gp.map_obstacles(m, 3):
            dx, dy = x - pose.x_n, y - pose.y_e
            along = dx * math.cos(h) + dy * math.sin(h)
            lat = abs(-dx * math.sin(h) + dy * math.cos(h))
            if along < 0 and lat <= 0.38 + r + 0.3:
                free = min(free, -along - (0.6 + r + 0.3))
        return max(0.3, min(full, free))

    def _manoeuvre(self, task, ctx):
        now, pose = ctx.b.now, ctx.b.pose
        if self.mode == "BRAKE":
            sog = float(getattr(pose, "sog_mps", None) or 0.0)
            if now - self.t_mode >= float(self.p["brake_s"]) or (now - self.t_mode > 0.3 and sog <= 0.02):
                self.mode, self.t_mode, self.start_xy = "BACKOFF", now, (pose.x_n, pose.y_e)
                self.backoff_lim = self.astern_free(ctx)
                if self.backoff_lim < float(self.p["backoff_m"]) - 0.05:
                    ctx.log(f"guard BACKOFF {self.backoff_lim:.1f} m only: a mapped object astern")
            return self._brake_frame()
        gone = math.hypot(pose.x_n - self.start_xy[0], pose.y_e - self.start_xy[1])
        lim = min(float(self.p["backoff_m"]), getattr(self, "backoff_lim", float(self.p["backoff_m"])))
        if gone >= lim or now - self.t_mode > float(self.p["backoff_max_s"]):
            self.mode = "CLEAR"
            if getattr(task, "_beh", None) is not None:          # the leg is re-planned round the new ring on the next tick
                task._beh.cancel()
                task._beh, task._beh_state = None, None
            ctx.log(f"guard BACKOFF done ({gone:.1f} m astern): the stage re-plans round the obstacle")
            return Hold()
        s = -float(self.p["backoff_surge"])
        return ThrusterFrame(s, 0.0, s, 0.0)
