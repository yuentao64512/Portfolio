"""report_mapper: vehicle events -> RoboCommand report bodies, coalesced to <= 1 Hz per (vehicle, kind)."""
from __future__ import annotations

import time
from typing import Callable, Dict, Optional, Tuple

REPORT_KINDS = (
    "safe_passage", "docking", "firefighting", "resource_request", "incident",
)
# v0.1.27 (WP8, Dmitrii 2026-11-04): m0t4's events are ours, not RoboNation reports - the OCS keeps them (ocs/state
# `internal`), the judge never sees them
INTERNAL_EVENTS = ("box_survey", "box_mission_complete")


def event_to_report(vehicle_id: str, event_body: dict) -> Optional[Tuple[str, dict]]:
    """Returns (coalesce_key, report_body) or None for events that are not reports (task_state, alert)."""
    kind = event_body.get("event")
    if kind == "uav_report":                       # v0.1.64 (D13): the UAV's own report, RoboNation's RxReport field as given
        return (f"uav_report:{event_body.get('id')}", {"kind": "pb", "field": event_body.get("kind"), "pb": event_body.get("body") or {},
                                                        "report_kind": event_body.get("kind")})
    if kind == "incident":
        sub = event_body.get("kind")
        if sub == "ack":
            return (f"incident_ack:{event_body.get('command_seq')}",
                    {"kind": "incident_ack", "command_seq": event_body.get("command_seq"), "what": event_body.get("what")})
        if sub == "readiness":
            return (f"readiness:{event_body.get('command_seq')}",
                    {"kind": "readiness", "command_seq": event_body.get("command_seq"), "position": event_body.get("position")})
        return None
    if kind == "safe_passage":
        return ("safe_passage", {"kind": "safe_passage", "entry_position": event_body.get("entry"),
                                 "exit_position": event_body.get("exit"), "buoys": event_body.get("buoys", []),
                                 "complete": event_body.get("complete", False)})
    if kind == "docking":
        return ("docking", {"kind": "docking", "bay": event_body.get("bay")})
    if kind == "firefighting":
        return ("firefighting", {"kind": "firefighting", "window": event_body.get("window"),
                                 "window_id": event_body.get("window_id")})   # 1 upper / 2 lower (rx_reports.proto); None -> 0 = unset
    if kind == "resource_request":
        return ("resource_request", {"kind": "resource_request", "task": event_body.get("task", "TASK_COORDINATED_LOGISTICS"),
                                     "resource_color": event_body.get("resource_color"), "delivery_color": event_body.get("delivery_color")})
    return None


def internal_event(event_body: dict) -> Optional[dict]:
    """The OCS-internal record of an m0t4 event (v0.1.27), or None for any other event."""
    kind = event_body.get("event")
    if kind == "box_survey":
        return {"kind": "box_survey", "boxes": event_body.get("boxes", []), "complete": bool(event_body.get("complete", True))}
    if kind == "box_mission_complete":
        return {"kind": "box_mission_complete", "target": event_body.get("target"), "stop_error_m": event_body.get("stop_error_m"),
                "face_distance_m": event_body.get("face_distance_m"), "complete": True}
    return None


class ReportMapper:
    """Coalesces progressive reports: the latest body per key is sent at most once per min_period_s;
    acks and readiness (unique keys) go out immediately."""

    def __init__(self, publish: Callable[[str, dict], Optional[int]], min_period_s: float = 1.0, clock=time.monotonic):
        self.publish = publish
        self.min_period_s = min_period_s
        self.clock = clock
        self._pending: Dict[Tuple[str, str], dict] = {}
        self._last_sent: Dict[Tuple[str, str], float] = {}
        self.sent = 0
        self.history = []

    def on_event(self, vehicle_id: str, event_body: dict) -> bool:
        m = event_to_report(vehicle_id, event_body)
        if m is None:
            return False
        key, body = m
        k = (vehicle_id, key)
        now = self.clock()
        if now - self._last_sent.get(k, -1e9) >= self.min_period_s:
            self._send(k, body, now)
        else:
            self._pending[k] = body
        return True

    def tick(self) -> None:
        now = self.clock()
        for k in list(self._pending):
            if now - self._last_sent.get(k, -1e9) >= self.min_period_s:
                self._send(k, self._pending.pop(k), now)

    def _send(self, k, body, now) -> None:
        seq = self.publish(k[0], body)
        if seq is not None:
            self._last_sent[k] = now
            self.sent += 1
            self.history.append((k[0], body.get("kind"), seq))
        else:
            self._pending[k] = body   # rate capped: retry on the next tick
