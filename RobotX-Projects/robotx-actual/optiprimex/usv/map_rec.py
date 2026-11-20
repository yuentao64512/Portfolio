"""map_rec: the recorded USV map (v0.1.81.1, Dmitrii 2026-11-11: "after the run, can we save the map we recorded? later we might want to
replay that" - both places, both uses).

One recorder per process: the boat's mission manager (`--map-rec DIR`, the launchers pass $RUN_DIR/maps: no gaps when the radio drops)
and the OCS page server (logs/ocs/maps: what reached the Mac). Each is fed the telemetry/<v>/map body (the PerceptionMap digest + the
boat's pose, the course, the path) and writes:

  <stamp>_<source>_map.jsonl     the timeline - a header line, then a `frame` line per second {k, t (unix), map (the body as sent)}
                                 -> "watch the run again" (a later replay page / tool steps through it)
  <stamp>_<source>_latest.json   the map now (rewritten every latest_s, atomically: a killed process still leaves its map)
  <stamp>_<source>_<task>_<HHMMSS>.json   the map when a task ends (the body's `task` changes away from it), and `_end_` on close
                                 -> "load as a starting map" (seed the next run / a simulator world)

A snapshot (format usv_map/1) is self-contained: every entity, ray and gate end also in lat / lon (the body's local frame is the
ControlOrigin `co` of the moment, which a later run will not share), the boat's track in lat / lon (a point per >= 0.5 m), the
course and the counters. Nothing here reads the map back - the loaders come after the runs.
"""
from __future__ import annotations

import json
import math
import os
import time
from typing import Any, Callable, List, Optional

from ..companion.frames import ControlOrigin

FORMAT_SNAP = "usv_map/1"
FORMAT_REC = "usv_map_rec/1"
IDLE = ("idle", "none", "")                    # no snapshot when these "end" (the vehicle between tasks)


def _ll(co: Optional[dict], x: Any, y: Any) -> Optional[List[float]]:
    """local (x_n, y_e) in the body's ControlOrigin -> [lat, lon] (the same flat-earth as companion.frames)."""
    try:
        lat, lon = ControlOrigin(float(co["lat"]), float(co["lon"])).from_local(float(x), float(y))
    except (TypeError, KeyError, ValueError):
        return None
    return [round(lat, 7), round(lon, 7)]


def _safe(name: Any) -> str:
    return "".join(ch if ch.isalnum() or ch in "-_" else "-" for ch in str(name or "none"))[:32] or "none"


def snapshot(body: dict, source: str, vehicle: str, task: Optional[str], track: List[list], version: str = "") -> dict:
    """A telemetry map body -> the usv_map/1 snapshot (lat / lon on every item)."""
    co = body.get("co") or {}
    ents = []
    for e in body.get("entities") or []:
        q = dict(e)
        ll = _ll(co, e.get("x_n"), e.get("y_e"))
        if ll:
            q["lat"], q["lon"] = ll
        ents.append(q)
    rays = []
    for r in body.get("rays") or []:
        q = dict(r)
        ll = _ll(co, r.get("x_n"), r.get("y_e"))
        if ll:
            q["lat"], q["lon"] = ll
        rays.append(q)
    b = body.get("boat") or {}
    boat = _ll(co, b.get("x_n"), b.get("y_e"))
    return {"format": FORMAT_SNAP, "saved_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "source": source, "vehicle": vehicle,
            "version": version, "task": task, "co": co, "boat": None if boat is None else {"lat": boat[0], "lon": boat[1], "hdg_deg": b.get("hdg_deg")},
            "entities": ents, "rays": rays, "gates": body.get("gates") or [], "markers": body.get("markers") or {},
            "course_boundary_local": body.get("course_boundary_local") or [],
            "counts": {k: sum(1 for e in ents if e.get("kind") == k) for k in sorted({str(e.get("kind")) for e in ents})},
            "track": track}


class MapRecorder:
    def __init__(self, out_dir: str, source: str, vehicle: str = "usv", hz: float = 1.0, latest_s: float = 10.0,
                 clock: Callable[[], float] = time.time, version: str = ""):
        self.dir, self.source, self.vehicle, self.version = out_dir, source, vehicle, version
        self.period, self.latest_s, self.clock = 1.0 / max(0.05, float(hz)), float(latest_s), clock
        self.stamp: Optional[str] = None
        self.fh = None
        self.last_t = -1e18
        self.latest_t = -1e18
        self.last_body: Optional[dict] = None
        self.task: Optional[str] = None
        self.track: List[list] = []
        self.n_frames = 0
        self.written: List[str] = []           # the snapshot files, in order (status / tests)

    def path(self, tail: str) -> str:
        return os.path.join(self.dir, f"{self.stamp}_{self.source}_{tail}")

    def _open(self, body: dict, t: float) -> None:
        os.makedirs(self.dir, exist_ok=True)
        self.stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(t))
        self.fh = open(self.path("map.jsonl"), "a", encoding="utf-8")
        self._line({"k": "header", "format": FORMAT_REC, "source": self.source, "vehicle": self.vehicle, "version": self.version, "t": round(t, 2),
                    "co": body.get("co")})

    def _line(self, d: dict) -> None:
        self.fh.write(json.dumps(d, separators=(",", ":")) + "\n")
        self.fh.flush()

    def _write_json(self, path: str, d: dict) -> None:
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(d, f, separators=(",", ":"))
        os.replace(tmp, path)

    def feed(self, body: Optional[dict], t: Optional[float] = None) -> None:
        """One telemetry map body (any rate: the recorder keeps one a period). Never raises into the caller's loop."""
        if not isinstance(body, dict):
            return
        t = self.clock() if t is None else t
        try:
            task = body.get("task")
            if self.last_body is not None and self.task and str(self.task).lower() not in IDLE and task != self.task:
                self.save(self.task, self.last_body)              # the task ended (or another began): its map as it stood
            self.task = task
            self.last_body = body
            if t - self.last_t < self.period:
                return
            if self.fh is None:
                self._open(body, t)
            self.last_t = t
            b = body.get("boat") or {}
            ll = _ll(body.get("co"), b.get("x_n"), b.get("y_e"))
            if ll and (not self.track or math.hypot((ll[0] - self.track[-1][0]) * 111320.0,
                                                    (ll[1] - self.track[-1][1]) * 111320.0 * math.cos(math.radians(ll[0]))) >= 0.5):
                self.track.append([ll[0], ll[1], round(t, 1)])
            self._line({"k": "frame", "t": round(t, 2), "map": body})
            self.n_frames += 1
            if t - self.latest_t >= self.latest_s:
                self.latest_t = t
                self._write_json(self.path("latest.json"), snapshot(body, self.source, self.vehicle, task, self.track, self.version))
        except OSError:
            pass

    def save(self, task: Optional[str], body: Optional[dict] = None) -> Optional[str]:
        body = body if body is not None else self.last_body
        if body is None:
            return None
        try:
            if self.fh is None:
                self._open(body, self.clock())
            p = self.path(f"{_safe(task)}_{time.strftime('%H%M%S')}.json")
            self._write_json(p, snapshot(body, self.source, self.vehicle, task, self.track, self.version))
            self._line({"k": "saved", "t": round(self.clock(), 2), "task": task, "file": os.path.basename(p)})
            self.written.append(p)
            return p
        except OSError:
            return None

    def close(self) -> None:
        if self.last_body is not None:
            self.save("end", self.last_body)
            try:
                self._write_json(self.path("latest.json"), snapshot(self.last_body, self.source, self.vehicle, self.task, self.track, self.version))
            except OSError:
                pass
        if self.fh is not None:
            self.fh.close()
            self.fh = None

    def view(self) -> dict:
        return {"dir": self.dir, "file": None if self.stamp is None else os.path.basename(self.path("map.jsonl")), "frames": self.n_frames,
                "snapshots": [os.path.basename(p) for p in self.written][-5:]}
