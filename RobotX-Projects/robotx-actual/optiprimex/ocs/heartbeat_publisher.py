"""HeartbeatPublisher: 2 Hz per declared vehicle from the last status (decision 4 stale rule).

v0.1.59 (A2 (ii), decision 4 as amended 2026-11-08): a vehicle in FAULT (V6) or BOOT (V0) keeps its last known robot_state
(KILLED / MANUAL / AUTO from an earlier status) for stale_stop_s from the moment it entered that state, then its heartbeat stops
(no fabrication beyond that; `stopped_fault`). Without a known state the fallback is STATE_MANUAL, as before. KILLED and MANUAL
come in the status itself (V1 / V2) and are reported at once."""
from __future__ import annotations

import time
from typing import Dict, List, Optional

from .judge_client import JudgeClient


class HeartbeatPublisher:
    def __init__(self, judge: JudgeClient, vehicles: List[str], hz: float = 2.0, stale_stop_s: float = 10.0,
                 clock=time.monotonic, stale_by: Optional[Dict[str, float]] = None):
        self.judge = judge
        self.vehicles = list(vehicles)
        self.period = 1.0 / hz
        self.stale_stop_s = stale_stop_s
        self.stale_by: Dict[str, float] = {k: float(v) for k, v in (stale_by or {}).items() if k}   # v0.1.64 (D2): the UAV 5 s
        self.clock = clock
        self.last_status: Dict[str, dict] = {}
        self.last_status_t: Dict[str, float] = {}
        self._next: Dict[str, float] = {}
        self.enabled = False
        self.sent: Dict[str, int] = {}
        self.stopped_stale: Dict[str, bool] = {}
        self.last_known: Dict[str, str] = {}         # v0.1.59: the last robot_state other than STATE_UNKNOWN
        self.fault_since: Dict[str, float] = {}      # v0.1.59: when the vehicle entered FAULT / BOOT
        self.stopped_fault: Dict[str, bool] = {}
        self.last_sent: Dict[str, dict] = {}         # v0.1.59 (B5): {t, state, seq} of the last heartbeat that went out

    def update(self, vehicle_id: str, status_body: dict) -> None:
        self.last_status[vehicle_id] = status_body
        self.last_status_t[vehicle_id] = self.clock()
        self.stopped_stale[vehicle_id] = False
        rs = status_body.get("robot_state") or "STATE_UNKNOWN"
        if status_body.get("state") in self.UNKNOWN_STATES:
            self.fault_since.setdefault(vehicle_id, self.clock())
        else:
            self.fault_since.pop(vehicle_id, None)
            self.stopped_fault[vehicle_id] = False
            if rs != "STATE_UNKNOWN":
                self.last_known[vehicle_id] = rs

    def age(self, vehicle_id: str) -> float:
        t = self.last_status_t.get(vehicle_id)
        return float("inf") if t is None else self.clock() - t

    UNKNOWN_STATES = ("FAULT", "BOOT")

    @staticmethod
    def to_heartbeat(status: dict) -> dict:
        """status/<v> body -> RoboCommand heartbeat fields (RobotState, RobotXTask, position, speed, heading)."""
        rs = status.get("robot_state") or "STATE_UNKNOWN"
        if rs == "STATE_UNKNOWN":
            rs = {"BOOT": "STATE_MANUAL", "FAULT": "STATE_MANUAL"}.get(status.get("state"), "STATE_MANUAL")
        return {"kind": "heartbeat", "state": rs, "current_task": status.get("current_task") or "TASK_NONE",
                "lat": status.get("lat"), "lon": status.get("lon"), "spd_mps": status.get("sog_mps"),
                "heading_deg": status.get("hdg_deg"), "roll_deg": status.get("roll_deg"), "pitch_deg": status.get("pitch_deg"),
                "altitude_hae_m": status.get("altitude_hae_m"),
                "flight_phase": status.get("flight_phase"), "vehicle_type": status.get("vehicle_type", "USV"),
                "uav_registration": status.get("uav_registration")}

    def tick(self) -> None:
        if not self.enabled:
            return
        now = self.clock()
        for v in self.vehicles:
            if now < self._next.get(v, 0.0):
                continue
            st = self.last_status.get(v)
            if st is None:
                continue
            self._next[v] = now + self.period
            cut = self.stale_by.get(v, self.stale_stop_s)
            ta = (st.get("health") or {}).get("telemetry_age_s") if v in self.stale_by else None
            if self.age(v) > cut or (ta is not None and float(ta) > cut) or (v in self.stale_by and ta is None):
                self.stopped_stale[v] = True
                continue  # no fabricated heartbeat after 10 s without status (v0.1.64: the UAV - telemetry older than 5 s, his invariant 3)
            if st.get("state") == "OFFLINE":
                continue
            hb = self.to_heartbeat(st)
            if v in self.fault_since:                   # v0.1.59 (A2): the last known state for stale_stop_s, then nothing
                if now - self.fault_since[v] > self.stale_stop_s:
                    self.stopped_fault[v] = True
                    continue
                hb["state"] = self.last_known.get(v, hb["state"])
            seq = self.judge.publish_report(v, hb)
            if seq is not None:
                self.sent[v] = self.sent.get(v, 0) + 1
                self.last_sent[v] = {"t": now, "state": hb["state"], "seq": seq}

    def view(self) -> Dict[str, dict]:
        """v0.1.59 (B5): per vehicle - heartbeats sent, the last one's state / seq / age, stopped (stale or the FAULT grace over)."""
        now = self.clock()
        out = {}
        for v in self.vehicles:
            ls = self.last_sent.get(v) or {}
            out[v] = {"n": self.sent.get(v, 0), "state": ls.get("state"), "seq": ls.get("seq"),
                      "age_s": None if not ls else round(now - ls["t"], 1),
                      "stopped": bool(self.stopped_stale.get(v) or self.stopped_fault.get(v))}
        return out
