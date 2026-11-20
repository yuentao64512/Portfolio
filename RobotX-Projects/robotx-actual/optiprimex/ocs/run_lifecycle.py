"""Layer 0: the run lifecycle R0-R6 owned by the OCS (ARCHITECTURE 3.1, ADR-8 stall path)."""
from __future__ import annotations

import time
from typing import Callable, Dict, List, Optional

from ..companion.fsm import Machine, State, Transition
from ..companion.state import TaskStateEvent

R = {"R0": "DISCONNECTED", "R1": "CONNECTED", "R2": "COURSE_VALID", "R3": "DECLARED", "R4": "READY",
     "R5": "RUNNING", "R6": "ENDED"}


class RunLifecycle:
    def __init__(self, stall_s: float = 60.0, on_transition: Optional[Callable[[TaskStateEvent], None]] = None,
                 on_alert: Optional[Callable[[str, dict], None]] = None, clock=time.monotonic, prepare_ack_s: float = 3.0):
        self.stall_s = stall_s
        self.prepare_ack_s = prepare_ack_s
        self.pending_prepare: Dict[str, Optional[str]] = {}   # vehicle -> ack result while a declaration waits for PREPARE acks
        self._prepare_t: Optional[float] = None
        self.on_alert = on_alert
        self.clock = clock
        self.course: Optional[dict] = None
        self.declaration: Optional[dict] = None
        self.declaration_seq: int = 0
        self.run_id: Optional[int] = None
        self.stalled = False
        self.ready_vehicles: Dict[str, bool] = {}
        self.ignored_run_starts: List[dict] = []
        self._ready_since: Optional[float] = None
        self.declared_t: Optional[float] = None          # v0.1.67 (B3): when R3 began (the AUTO caution's clock)
        self.m = Machine(
            name="run", layer="run", initial=R["R0"], on_transition=on_transition, clock=clock,
            states=[State(v) for v in R.values()],
            transitions=[
                Transition("CONNECTED", [R["R0"]], R["R1"], reason="RoboCommand link up"),
                Transition("DISCONNECTED", "*", R["R0"], reason="RoboCommand link lost"),
                Transition("COURSE", [R["R1"]], R["R2"], guard=lambda c: self._course_valid(c), reason="course valid"),
                Transition("COURSE", [R["R2"], R["R3"], R["R4"], R["R5"]], R["R2"],
                           guard=lambda c: self._course_changed(c) and self._course_valid(c), reason="course republished with changes"),
                Transition("DECLARE", [R["R2"], R["R4"], R["R6"]], R["R3"], reason="RunDeclaration published"),
                Transition("VEHICLES_READY", [R["R3"]], R["R4"], reason="all declared vehicles STATE_AUTO"),
                Transition("RUN_START", [R["R4"], R["R3"]], R["R5"], guard=lambda c: self._seq_matches(c), reason="RunStart"),
                Transition("END", [R["R3"], R["R4"], R["R5"]], R["R6"], reason="run ended by the operator"),
                Transition("RESET_COURSE", [R["R2"], R["R3"], R["R4"], R["R5"], R["R6"]], R["R2"], reason="reset by the operator: run context dropped"),
                Transition("RESET_NOCOURSE", [R["R2"], R["R3"], R["R4"], R["R5"], R["R6"]], R["R1"], reason="reset by the operator: no valid course"),
            ],
        )

    # ---- guards ---------------------------------------------------------------------------
    @staticmethod
    def _course_valid(c: dict) -> bool:
        b = (c or {}).get("boundary") or []
        return bool((c or {}).get("course_id")) and len(b) >= 3

    COURSE_KEYS = ("course_id", "pinger_hz", "boundary", "uav_transit_area")     # v0.1.67 (B1): the UAV team's course_fingerprint fields

    def _course_changed(self, c: dict) -> bool:
        """A republished course counts as changed when its content differs - id, pinger frequency, corners or the UAV transit area
        (v0.1.67, borrowed from the UAV team's course_fingerprint, their invariant 7; was: id + boundary only - a new pinger frequency
        passed unnoticed). Never the publish time (a retained republish is not a change)."""
        if self.course is None:
            return True
        norm = lambda v: [list(map(float, p)) if isinstance(p, (list, tuple)) else p for p in v] if isinstance(v, list) else v
        return any(norm(c.get(k) or ([] if k in ("boundary", "uav_transit_area") else None))
                   != norm(self.course.get(k) or ([] if k in ("boundary", "uav_transit_area") else None)) for k in self.COURSE_KEYS)

    def _seq_matches(self, cmd: dict) -> bool:
        return cmd is not None and int(cmd.get("declaration_seq", -1)) == self.declaration_seq and self.declaration_seq > 0

    # ---- inputs -------------------------------------------------------------------------------
    @property
    def state(self) -> str:
        return self.m.state

    @property
    def r_index(self) -> int:
        return list(R.values()).index(self.m.state)

    def on_link(self, up: bool) -> None:
        if up:
            if self.m.state == R["R0"]:
                self.m.handle("CONNECTED")
                # reconnect keeps context (3.1 rules): re-validate the retained course when it arrives
        else:
            self.m.handle("DISCONNECTED")

    def on_course(self, course: dict) -> bool:
        changed = self.m.handle("COURSE", course)
        if changed or (self.course is None and self._course_valid(course)):
            self.course = course
        return changed

    def declare(self, vehicle_ids: List[str], tiers: Dict[str, str], uav_geofence: List, extra: Optional[dict] = None,
                require_prepare_ack: bool = False) -> dict:
        """Build the declaration (new declaration_seq). With require_prepare_ack the R3 transition waits for on_prepare_ack()
        from every vehicle (the OCS app sends PREPARE first and publishes to the judge only then); without it, R3 at once."""
        self.declaration_seq += 1
        self.stalled = False
        self._ready_since = None                 # a fresh declaration: the stall clock starts again at VEHICLES_READY
        self.ready_vehicles = {v: False for v in vehicle_ids}
        self.declaration = {"kind": "run_declaration", "declaration_seq": self.declaration_seq, "vehicle_ids": list(vehicle_ids),
                            "tiers": dict(tiers), "uav_geofence": list(uav_geofence), **(extra or {})}
        self.run_id = None
        if require_prepare_ack:
            self.pending_prepare = {v: None for v in vehicle_ids}
            self._prepare_t = self.clock()
        else:
            self.pending_prepare = {}
            if self.m.handle("DECLARE", self.declaration):
                self.declared_t = self.clock()
        return self.declaration

    def on_prepare_ack(self, vehicle_id: str, result: str, reason: str = "") -> bool:
        """A PREPARE ack for the pending declaration. Returns True when the last DONE moved the run to R3."""
        if vehicle_id not in self.pending_prepare:
            return False
        if result in ("REJECTED", "FAILED"):
            if self.on_alert:
                self.on_alert("prepare_rejected", {"vehicle": vehicle_id, "reason": reason, "declaration_seq": self.declaration_seq,
                                                   "action": "fix the vehicle, declare again"})
            self.pending_prepare = {}
            return False
        if result == "DONE":
            self.pending_prepare[vehicle_id] = "DONE"
            if all(r == "DONE" for r in self.pending_prepare.values()):
                self.pending_prepare = {}
                ok = self.m.handle("DECLARE", self.declaration)
                if ok:
                    self.declared_t = self.clock()
                return ok
        return False

    def reset(self) -> bool:
        """Operator reset: drop the declaration / run context and go back to R2 (valid course) or R1. R0/R1: nothing to do."""
        if self.m.state in (R["R0"], R["R1"]):
            return False
        self.declaration = None
        self.run_id = None
        self.stalled = False
        self.ready_vehicles = {}
        self._ready_since = None
        self.pending_prepare = {}
        return self.m.handle("RESET_COURSE" if self._course_valid(self.course or {}) else "RESET_NOCOURSE")

    def vehicle_status(self, vehicle_id: str, robot_state: str) -> None:
        if vehicle_id in self.ready_vehicles:
            self.ready_vehicles[vehicle_id] = (robot_state == "STATE_AUTO")
            if self.m.state == R["R3"] and all(self.ready_vehicles.values()):
                self.m.handle("VEHICLES_READY")
                self._ready_since = self.clock()

    def on_run_start(self, cmd: dict) -> bool:
        if self.m.handle("RUN_START", cmd):
            self.run_id = int(cmd.get("run_id") or int(self.clock() * 1000) % 100000)
            self.stalled = False
            return True
        self.ignored_run_starts.append(cmd)
        if self.on_alert:
            self.on_alert("run_start_ignored", {"got_seq": cmd.get("declaration_seq"), "ours": self.declaration_seq, "state": self.m.state})
        return False

    def end(self) -> bool:
        ok = self.m.handle("END")
        if ok:
            self.run_id = None           # R6: no run to stamp on later envelopes (3.4.13)
        return ok

    def tick(self) -> None:
        """Stall path: in R4 with no RunStart within stall_s -> operator alert, redeclare is the only re-arm.
        PREPARE acks not all back within prepare_ack_s -> alert, the declaration is dropped (still R2)."""
        if self.pending_prepare and self._prepare_t is not None and self.clock() - self._prepare_t > self.prepare_ack_s:
            missing = [v for v, r in self.pending_prepare.items() if r != "DONE"]
            self.pending_prepare = {}
            if self.on_alert:
                self.on_alert("prepare_timeout", {"vehicles": missing, "waited_s": self.prepare_ack_s, "declaration_seq": self.declaration_seq,
                                                  "action": "is the mission manager up and bridged? declare again"})
        if self.m.state == R["R4"] and self._ready_since is not None and not self.stalled:
            if self.clock() - self._ready_since > self.stall_s:
                self.stalled = True
                if self.on_alert:
                    self.on_alert("run_start_stall", {"waited_s": self.stall_s, "declaration_seq": self.declaration_seq,
                                                      "action": "declared, waiting for the course manager - a refusal is shown on RoboCommand's "
                                                                "display, ask the staff; redeclare (a new declaration_seq) only after that"})

    def body(self) -> dict:
        return {"r_state": self.m.state, "r": f"R{self.r_index}", "declaration_seq": self.declaration_seq, "run_id": self.run_id,
                "stalled": self.stalled, "course_id": (self.course or {}).get("course_id"),
                "ready": dict(self.ready_vehicles), "prepare_pending": [v for v, r in self.pending_prepare.items() if r != "DONE"],
                "stall_s": self.stall_s,                     # v0.1.43: the RunStart slot - waited / to the stall alert
                "declared_s": (round(self.clock() - self.declared_t, 1) if (self.m.state == R["R3"] and self.declared_t is not None) else None),
                "waiting_s": (round(self.clock() - self._ready_since, 1) if (self.m.state == R["R4"] and self._ready_since is not None) else None)}
