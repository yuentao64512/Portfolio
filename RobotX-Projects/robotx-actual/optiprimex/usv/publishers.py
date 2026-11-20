"""Rate-limited publishers: status 2 Hz, local state 5 Hz, map digest 2 Hz, events immediately."""
from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple

from ..companion.config import Config
from ..companion.mqtt import Bus
from ..companion.state import Event, VehicleState


class _Rated:
    def __init__(self, hz: float):
        self.period = 1.0 / hz if hz > 0 else float("inf")
        self.last = -1e9

    def due(self, now: float) -> bool:
        if now - self.last >= self.period:
            self.last = now
            return True
        return False


class StatusPublisher:
    def __init__(self, bus: Bus, cfg: Config):
        self.bus, self.cfg = bus, cfg
        self.topic = cfg.team("status", cfg.vehicle)
        self.rate = _Rated(cfg.rates.status_hz)
        self.count = 0

    def maybe(self, vs: VehicleState, now: float) -> None:
        if self.rate.due(now):
            body = vs.status_body()
            r = self.cfg.rates                                  # v0.1.86.1 (session A's SETTINGS card): the boat's own rates, read-only
            body["rates"] = {"tick_hz": r.tick_hz, "status_hz": r.status_hz, "local_hz": r.local_hz, "map_hz": r.map_hz}
            self.bus.publish(self.topic, "status", body, qos=0, timeout_s=0.0)
            self.count += 1


class LocalStatePublisher:
    def __init__(self, bus: Bus, cfg: Config):
        self.bus, self.cfg = bus, cfg
        self.topic = cfg.local("mm", "state")
        self.rate = _Rated(cfg.rates.local_hz)

    def maybe(self, vs: VehicleState, now: float) -> None:
        if self.rate.due(now):
            self.bus.publish(self.topic, "local", vs.local_body(), qos=0, timeout_s=0.0)


class MapDigestPublisher:
    """telemetry/<v>/map: boat pose and trail, plus the PerceptionMap digest (entities, gates, dock) when given."""

    def __init__(self, bus: Bus, cfg: Config):
        self.bus, self.cfg = bus, cfg
        self.topic = cfg.team("telemetry", cfg.vehicle, "map")
        self.rate = _Rated(cfg.rates.map_hz)
        self.trail: Deque[Tuple[float, float]] = deque(maxlen=50)
        self.recorder = None               # v0.1.81.1: usv/map_rec.MapRecorder (mission_manager --map-rec DIR) - fed every body sent

    def maybe(self, vs: VehicleState, now: float, path=None, carrot=None, extra: Optional[dict] = None) -> None:
        if not self.rate.due(now) or vs.local is None:
            return
        self.trail.append((round(vs.local.x_n, 2), round(vs.local.y_e, 2)))
        body = {
            "t": vs.fcu.t_utc, "co": {"lat": vs.local.co_lat, "lon": vs.local.co_lon},
            "boat": {"x_n": round(vs.local.x_n, 2), "y_e": round(vs.local.y_e, 2), "hdg_deg": vs.local.hdg_deg,
                     "trail": list(self.trail), **({"nav": vs.nav} if vs.nav else {})},   # v0.1.68: nav ghost -> the dashed glyph
            "course_boundary_local": [], "entities": [], "gates": [], "dock": None,
            "path": path or [], "carrot": carrot, "task_state": vs.task_state, "task": vs.task_name,
        }
        if extra:
            body.update(extra)             # entities, gates, dock, ingest counters from the PerceptionMap
        self.bus.publish(self.topic, "telemetry", body, qos=0, timeout_s=0.0)
        if self.recorder is not None:
            self.recorder.feed(body)


class EventPublisher:
    def __init__(self, bus: Bus, cfg: Config):
        self.bus, self.cfg = bus, cfg
        self.count = 0

    def emit(self, ev: Event) -> str:
        topic = self.cfg.team("event", self.cfg.vehicle, ev.topic_suffix())
        self.count += 1
        return self.bus.publish(topic, "event", ev.body, qos=1, timeout_s=0.0)
