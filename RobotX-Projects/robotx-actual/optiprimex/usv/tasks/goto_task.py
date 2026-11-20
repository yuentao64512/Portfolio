"""GOTO: a one-shot bench task (decision 23: bench only, never on the course). Goto a point, then done."""
from __future__ import annotations

from typing import Any, Dict

from ...companion.fsm import Transition
from ...companion.state import RobotXTask
from ..behaviours import Goto
from .base import Task, TaskContext, simple_machine


class GotoTask(Task):
    name = "GOTO"
    robo_task = RobotXTask.NONE
    BEHAVIOURS = {"GO": Goto}

    def build_machine(self, on_transition, clock):
        return simple_machine("GOTO", ["GO"], [Transition("ARRIVED", ["GO"], "DONE", reason="arrived")],
                              initial="GO", on_transition=on_transition, clock=clock)

    def behaviour_args(self, sub_state: str, ctx: TaskContext) -> Dict[str, Any]:
        return dict(self.args)

    def on_behaviour_done(self, sub_state: str, ctx: TaskContext) -> None:
        self.m.handle("ARRIVED")
