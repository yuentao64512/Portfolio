"""Layer 1: the vehicle state machine V0-V6 (ARCHITECTURE 3.2, ADR-8, ADR-9).

Precedence when inputs conflict (highest first): V1 kill > V2 RC override > V6 fault > V5 safe-hold
> V4 task > V3 hold. The companion never arms, never clears a kill, never leaves MANUAL on its own:
leaving V1/V6 needs the input cleared AND an operator CONFIRM_SAFE from the OCS; leaving V2 needs
ENABLE_AUTO from the OCS with the guards below.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple

from ..companion.config import Config
from ..companion.fsm import Machine, State, Transition
from ..companion.state import Command, FcuState, Health, TaskStateEvent, VState

V = VState
AUTO_STATES = (V.V3.value, V.V4.value, V.V5.value)


@dataclass
class Inputs:
    """Derived each tick from FcuState + Health + TUI (vehicle_fsm.feed)."""
    fcu_ready: bool = False
    kill: bool = False
    rc_override: bool = False
    rc6_high: bool = False          # v0.1.29.3 (F6): which part of rc_override, for the refusal text
    hard_faults: List[str] = field(default_factory=list)
    soft_faults: List[str] = field(default_factory=list)
    armed: Optional[bool] = None
    ekf_pos_ok: bool = False
    mode_name: Optional[str] = None
    tui_active: bool = False


RUN_FREE_TASKS = ("TEST_MANUAL", "TEST_AUTO", "TEST_PATH", "TEST_SWAY", "TEST_ORBIT", "GOTO", "RETURN_HOME", "BOX_INSPECT", "TEST_DETECT", "TEST_GUARD", "TEST_YAW")   # v0.1.37


class VehicleFsm:
    def __init__(self, cfg: Config, on_transition: Optional[Callable[[TaskStateEvent], None]] = None,
                 clock=None):
        self.cfg = cfg
        self.inputs = Inputs()
        self.hold_reason: Optional[str] = None
        self.fault_reason: Optional[str] = None
        self.task_paused: bool = False       # set by the task runner on HOLD / soft fault
        self.task_running: bool = False
        self.confirm_safe_pending: bool = False
        self.sticky_holds: set = set()        # task-layer reasons that must be cleared explicitly (keep-out, link policy)
        self.declaration_seq: Optional[int] = None
        self.run_id: Optional[int] = None
        self.rc6_policy: str = getattr(self.cfg.guards, "rc6_policy", "latched")   # v0.1.50: latched | hold
        self._rc6_prev: bool = False
        self._rc6_latch: bool = False
        self._rc6_mode: Optional[str] = None      # the FCU mode when the latch was set (a change after it = someone commanded)
        g = self.cfg.guards
        self.m = Machine(
            name="vehicle", layer="vehicle", initial=V.V0.value, on_transition=on_transition,
            clock=clock or __import__("time").monotonic,
            states=[State(s.value) for s in V],
            transitions=[
                # physical inputs (applied by feed() in precedence order)
                Transition("FCU_READY", [V.V0.value], V.V2.value, guard=lambda p: not self.inputs.kill, reason="FCU link up"),
                Transition("FCU_READY", [V.V0.value], V.V1.value, guard=lambda p: self.inputs.kill, reason="FCU link up, kill asserted"),
                Transition("KILL_ON", [V.V2.value, *AUTO_STATES, V.V6.value], V.V1.value, reason="kill or E-stop asserted"),
                Transition("RC_OVERRIDE_ON", list(AUTO_STATES), V.V2.value, reason="RC6 / FCU MANUAL / TUI"),
                Transition("FAULT", [V.V2.value, *AUTO_STATES], V.V6.value, reason="hard fault"),
                Transition("SOFT_FAULT", [V.V4.value], V.V5.value, reason="degraded health"),
                Transition("SAFE_CONFIRMED", [V.V1.value], V.V2.value,
                           guard=lambda p: not self.inputs.kill and (self.confirm_safe_pending or not g.v1_exit_requires_confirm),
                           reason="kill cleared and operator confirmed"),
                Transition("SAFE_CONFIRMED", [V.V6.value], V.V2.value,
                           guard=lambda p: not self.inputs.hard_faults and not self.inputs.kill and (self.confirm_safe_pending or not g.v6_exit_requires_confirm),
                           reason="fault cleared and operator confirmed"),
                # OCS commands
                Transition("ENABLE_AUTO", [V.V2.value], V.V3.value, guard=lambda p: self.can_enable_auto()[0], reason="ENABLE_AUTO"),
                Transition("DISABLE_AUTO", [V.V3.value], V.V2.value, reason="DISABLE_AUTO"),
                Transition("START_TASK", [V.V3.value], V.V4.value, guard=lambda p: self.can_start_task(p)[0], reason="START_TASK"),
                Transition("HOLD", [V.V3.value, V.V4.value], V.V5.value, reason="HOLD"),
                Transition("RESUME", [V.V5.value], V.V4.value, guard=lambda p: self.task_paused and self.hold_cleared(), reason="RESUME to task"),
                Transition("RESUME", [V.V5.value], V.V3.value, guard=lambda p: not self.task_paused and self.hold_cleared(), reason="RESUME to hold"),
                Transition("ABORT_TASK", [V.V4.value, V.V5.value], V.V3.value, reason="ABORT_TASK"),
                # task layer outcomes
                Transition("TASK_DONE", [V.V4.value], V.V3.value, reason="task done"),
                Transition("TASK_ABORTED", [V.V4.value], V.V3.value, reason="task aborted"),
            ],
        )

    # ---- queries ------------------------------------------------------------------
    @property
    def state(self) -> VState:
        return V(self.m.state)

    def can_enable_auto(self) -> Tuple[bool, str]:
        g, i = self.cfg.guards, self.inputs
        if i.kill:
            return False, "kill asserted"
        if i.rc_override:
            if i.tui_active:
                return False, "usv_ui active (type handover there)"
            if i.rc6_high:                                # v0.1.29.3 (pool report F6): say what to do
                return False, "RC6 override switch is high on the transmitter: switch it low, then enable_auto"
            return False, (f"the FCU is in {i.mode_name or 'MANUAL'} (RC6 override or the transmitter's mode switch): "
                           "put it in HOLD, then enable_auto")
        if i.hard_faults:
            return False, "fault: " + ",".join(i.hard_faults)
        if "RC_LOST" in i.soft_faults:               # v0.1.29.3: guards.rc_lost_action hold - no autonomy without the transmitter
            return False, "the transmitter is lost (RC_LOST, guards.rc_lost_action hold): switch it on, then enable_auto"
        if g.require_armed_for_auto and not i.armed:
            return False, "not armed"
        if g.require_ekf_for_auto and not i.ekf_pos_ok:
            return False, "EKF position not ok"
        if g.require_prepare_for_auto and self.declaration_seq is None and not g.test_mode:
            return False, "no PREPARE (declaration_seq unknown)"
        return True, ""

    def can_start_task(self, cmd: Optional[Command]) -> Tuple[bool, str]:
        if cmd is None or not cmd.task:
            return False, "no task named"
        if cmd.task == "GOTO" and not self.cfg.guards.allow_goto_task:
            return False, "GOTO task not allowed by vehicle config"
        if cmd.task not in ("T1", "T3", "T4", "GOTO", "TEST_MANUAL", "TEST_AUTO", "TEST_PATH", "TEST_SWAY", "BOX_INSPECT",
                            "RETURN_HOME", "TEST_ORBIT", "SCOUT", "TEST_DETECT", "TEST_GUARD", "TEST_YAW"):
            return False, f"unknown task {cmd.task}"
        if cmd.task == "RETURN_HOME":                # v0.1.30: the go-home override needs no run (a safety function)
            return True, ""
        if self.cfg.guards.test_mode and cmd.task in RUN_FREE_TASKS:     # v0.1.37: TEST MODE (pool / lab / bench, never the course)
            return True, ""
        if self.cfg.guards.require_run_for_task and self.run_id is None:
            return False, "no START_RUN (run_id unknown)"
        return True, ""

    # ---- v0.1.50: RC6 latched (last wins) or held -----------------------------------------
    def _rc6_effective(self, raw: bool, mode: Optional[str]) -> bool:
        if self.rc6_policy != "latched":
            self._rc6_prev, self._rc6_latch = raw, False
            return raw
        if raw and not self._rc6_prev:                       # the flip up: MANUAL at once
            self._rc6_latch, self._rc6_mode = True, mode
        elif not raw:
            self._rc6_latch = False
        elif self._rc6_latch and mode not in (None, "MANUAL") and mode != self._rc6_mode:
            self._rc6_latch = False                          # the FCU was commanded to another mode after the flip (a page, a GCS)
        self._rc6_prev = raw
        return self._rc6_latch

    def clear_rc6_latch(self) -> bool:
        """A page command after the flip wins (latched policy): True when a latch was cleared."""
        if self.rc6_policy != "latched" or not self._rc6_latch:
            return False
        self._rc6_latch = False
        i = self.inputs
        i.rc6_high = False
        i.rc_override = bool(i.mode_name == "MANUAL" or i.tui_active)
        return True

    def set_rc6_policy(self, policy: str) -> None:
        self.rc6_policy = policy
        if policy != "latched":
            self._rc6_latch = False

    def hold_cleared(self) -> bool:
        """A hold is cleared when no soft fault is present and no sticky reason (set by the task layer) remains."""
        if self.inputs.soft_faults:
            return False
        return not self.sticky_holds

    # ---- per tick -------------------------------------------------------------------
    def feed(self, fcu: FcuState, health: Health, tui_active: bool, hard_faults: List[str],
             soft_faults: List[str]) -> List[str]:
        """Apply physical inputs in precedence order. Returns the events that fired."""
        i = self.inputs
        i.fcu_ready = fcu.seen_heartbeat and fcu.mode_name is not None
        i.kill = bool(fcu.kill or fcu.estop or fcu.rc5_estop)
        rc6 = self._rc6_effective(bool(fcu.rc6_high), fcu.mode_name)
        i.rc_override = bool(rc6 or fcu.mode_name == "MANUAL" or tui_active)
        i.rc6_high = rc6
        i.hard_faults = list(hard_faults)
        i.soft_faults = list(soft_faults)
        i.armed, i.ekf_pos_ok, i.mode_name, i.tui_active = fcu.armed, fcu.ekf_pos_ok, fcu.mode_name, tui_active
        fired: List[str] = []

        def fire(ev: str, reason: str = "") -> bool:
            if self.m.handle(ev, None, reason=reason):
                fired.append(ev)
                return True
            return False

        if self.m.state == V.V0.value:
            if i.fcu_ready:
                fire("FCU_READY")
            return fired
        if i.kill:
            fire("KILL_ON", "kill" if (fcu.kill or fcu.estop) else "RC5 E-stop")
        elif i.rc_override:
            reason = "usv_ui active (type handover there)" if tui_active else ("FCU in MANUAL" if fcu.mode_name == "MANUAL" else "RC6 override")
            if fire("RC_OVERRIDE_ON", reason):
                self.task_paused = False
        elif i.hard_faults:
            self.fault_reason = ",".join(i.hard_faults)
            if fire("FAULT", self.fault_reason):
                self.task_paused = False
        elif i.soft_faults and self.m.state == V.V4.value:
            self.hold_reason = ",".join(i.soft_faults)
            if fire("SOFT_FAULT", self.hold_reason):
                self.task_paused = True
        if self.m.state in (V.V1.value, V.V6.value):
            if fire("SAFE_CONFIRMED"):
                self.confirm_safe_pending = False
                self.fault_reason = None
        if self.m.state not in (V.V6.value,):
            self.fault_reason = None if self.m.state != V.V6.value else self.fault_reason
        return fired

    # ---- commands --------------------------------------------------------------------
    def handle_command(self, cmd: Command) -> Tuple[str, str]:
        """Returns (result, reason) where result is ACCEPTED or REJECTED. Transitions apply immediately."""
        a = cmd.action
        s = self.m.state
        if a == "CONFIRM_SAFE":
            if s not in (V.V1.value, V.V6.value):
                return "REJECTED", f"CONFIRM_SAFE only in KILLED or FAULT (now {s})"
            self.confirm_safe_pending = True
            return "ACCEPTED", "will leave on the next tick if the input is cleared"
        if a == "ENABLE_AUTO":
            if s != V.V2.value:
                return "REJECTED", f"ENABLE_AUTO only from MANUAL (now {s})"
            ok, why = self.can_enable_auto()
            if not ok:
                return "REJECTED", why
            self.m.handle("ENABLE_AUTO")
            return "ACCEPTED", ""
        if a == "DISABLE_AUTO":
            if not self.m.handle("DISABLE_AUTO"):
                return "REJECTED", f"DISABLE_AUTO only from AUTO_HOLD (now {s})"
            return "ACCEPTED", ""
        if a == "START_TASK":
            if s != V.V3.value:
                return "REJECTED", f"START_TASK only from AUTO_HOLD (now {s})"
            ok, why = self.can_start_task(cmd)
            if not ok:
                return "REJECTED", why
            self.m.handle("START_TASK", cmd)
            self.task_running = True
            self.task_paused = False
            return "ACCEPTED", ""
        if a == "HOLD":
            if s not in (V.V3.value, V.V4.value):
                return "REJECTED", f"HOLD only from AUTO_HOLD or AUTO_TASK (now {s})"
            self.hold_reason = "HOLD"
            self.task_paused = (s == V.V4.value)
            self.m.handle("HOLD")
            return "ACCEPTED", ""
        if a == "RESUME":
            if s != V.V5.value:
                return "REJECTED", f"RESUME only from AUTO_SAFE_HOLD (now {s})"
            if not self.hold_cleared():
                return "REJECTED", f"hold trigger still active: {self.hold_reason or self.inputs.soft_faults}"
            self.hold_reason = None
            if not self.m.handle("RESUME"):
                return "REJECTED", "no RESUME transition applicable"
            self.task_paused = False
            return "ACCEPTED", ""
        if a == "ABORT_TASK":
            if s not in (V.V4.value, V.V5.value):
                return "REJECTED", f"ABORT_TASK only from AUTO_TASK or AUTO_SAFE_HOLD (now {s})"
            self.m.handle("ABORT_TASK")
            self.task_running = False
            self.task_paused = False
            self.hold_reason = None
            return "ACCEPTED", ""
        return "REJECTED", f"unknown vehicle command {a}"

    # ---- task layer feedback ------------------------------------------------------------
    def task_finished(self, aborted: bool, reason: str = "") -> None:
        self.task_running = False
        self.task_paused = False
        self.m.handle("TASK_ABORTED" if aborted else "TASK_DONE", reason=reason)

    def request_safe_hold(self, reason: str) -> bool:
        """Task layer asks for V5 (incident keep-out, moving object, link-loss policy)."""
        if self.m.state not in (V.V3.value, V.V4.value):
            return False
        self.hold_reason = reason
        self.task_paused = (self.m.state == V.V4.value)
        return self.m.handle("HOLD", reason=reason)
