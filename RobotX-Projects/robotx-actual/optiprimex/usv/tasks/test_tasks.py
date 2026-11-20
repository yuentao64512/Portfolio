"""TEST_MANUAL and TEST_AUTO: two test missions with no motion, so every console shows the switching (GUIDE stage A/B).

    TEST_MANUAL   S1 -> S2 -> S3 -> DONE, each step on the operator's STEP command (ocs_cli step, or `step` in usv_ui)
    TEST_AUTO     the same stages on timers (step_s each, default 5 s), then DONE by itself

Both run inside V4 AUTO_TASK like a real task (start_task, hold / resume, abort, STOP_RUN, RESET all apply), report
their sub-state transitions on event/<v>/task_state, and send nothing to the FCU (no behaviour in any stage: the FCU
stays in the HOLD the mission manager set on entering AUTO). RobotXTask stays NONE for the judge's heartbeat.
"""
from __future__ import annotations

from typing import Any, Dict

from ...companion.fsm import Transition
from ...companion.state import RobotXTask
from .base import Task, TaskContext, simple_machine

STAGES = ["S1", "S2", "S3"]
STAGE_MEANING = {"S1": "stage 1 of 3 (arm the sequence)", "S2": "stage 2 of 3 (the middle)", "S3": "stage 3 of 3 (last before DONE)"}


def _transitions():
    tr = [Transition("NEXT", [a], b, reason="next stage") for a, b in zip(STAGES, STAGES[1:])]
    tr.append(Transition("NEXT", [STAGES[-1]], "DONE", reason="last stage done"))
    return tr


class TestManual(Task):
    name = "TEST_MANUAL"
    robo_task = RobotXTask.NONE
    BEHAVIOURS: Dict[str, Any] = {}          # no motion anywhere

    def build_machine(self, on_transition, clock):
        return simple_machine("TEST_MANUAL", STAGES, _transitions(), initial="S1", on_transition=on_transition, clock=clock)

    def on_step(self, ctx: TaskContext) -> str:
        """The operator's STEP: advance one stage. Returns what happened (for the ack reason)."""
        before = self.m.state
        if not self.m.handle("NEXT"):
            return f"no step from {before}"
        return f"{before} -> {self.m.state}"


class TestAuto(Task):
    name = "TEST_AUTO"
    robo_task = RobotXTask.NONE
    BEHAVIOURS: Dict[str, Any] = {}

    def build_machine(self, on_transition, clock):
        step_s = float(self.cfg.extra.get("test_auto", {}).get("step_s", 5.0))
        return simple_machine("TEST_AUTO", STAGES, _transitions(), initial="S1", on_transition=on_transition, clock=clock,
                              timeouts={s: (step_s, "NEXT") for s in STAGES})

    def on_step(self, ctx: TaskContext) -> str:
        """STEP also works here: it skips the rest of the current timer."""
        before = self.m.state
        if not self.m.handle("NEXT"):
            return f"no step from {before}"
        return f"{before} -> {self.m.state} (timer skipped)"
