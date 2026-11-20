"""TaskRunner: exactly one task at a time, with the Task 4 breakpoint detour (ARCHITECTURE 10.9).

interrupt() saves (task, sub-state) and runs T4 from ACK; when T4 finishes the saved task resumes.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from ..companion.config import Config
from ..companion.state import Event, RobotXTask, TaskStateEvent
from .tasks import REGISTRY, Task, TaskContext, TaskStatus

log = logging.getLogger("optiprimex.tasks")


@dataclass
class Breakpoint:
    task: Task
    sub_state: str


class TaskRunner:
    def __init__(self, cfg: Config, on_transition: Optional[Callable[[TaskStateEvent], None]] = None, clock=None,
                 registry: Optional[Dict[str, type]] = None):
        self.cfg = cfg
        self.on_transition = on_transition
        self.clock = clock
        self.registry = dict(registry or REGISTRY)
        self.task: Optional[Task] = None
        self.saved: Optional[Breakpoint] = None
        self.last_status: Optional[TaskStatus] = None

    # ---- queries ----------------------------------------------------------------------
    @property
    def active(self) -> bool:
        return self.task is not None

    @property
    def current_task(self) -> str:
        return self.task.robo_task.value if self.task else RobotXTask.NONE.value

    @property
    def task_name(self) -> Optional[str]:
        return self.task.name if self.task else None

    @property
    def sub_state(self) -> Optional[str]:
        return self.task.m.state if self.task else None

    # ---- lifecycle --------------------------------------------------------------------
    def start(self, task_name: str, args: Dict[str, Any], ctx: TaskContext) -> Task:
        if task_name not in self.registry:
            raise KeyError(f"unknown task {task_name}")
        if self.task is not None:
            self.task.abort("replaced")
        self.task = self.registry[task_name](self.cfg, on_transition=self.on_transition, clock=self.clock)
        self.task.start(args, ctx)
        if (args or {}).get("from"):                 # v0.1.26: validated by the command handler before the FSM moved
            why = self.task.enter_from(str(args["from"]), ctx)
            if why:
                log.warning("from %s refused after start: %s (the task starts at its first stage)", args["from"], why)
        log.info("task %s started in %s", task_name, self.task.m.state)
        return self.task

    def interrupt(self, task_name: str, args: Dict[str, Any], ctx: TaskContext) -> Task:
        """Save the running task at its breakpoint and run another one (T4 detour)."""
        if self.task is not None and not self.task.is_terminal:
            self.task.pause()
            self.saved = Breakpoint(task=self.task, sub_state=self.task.m.state)
            log.info("breakpoint saved: %s at %s", self.task.name, self.saved.sub_state)
        self.task = None
        return self.start(task_name, args, ctx)

    def pause(self) -> None:
        if self.task:
            self.task.pause()

    def resume(self) -> None:
        if self.task:
            self.task.resume()

    def abort(self, reason: str) -> None:
        if self.task:
            self.task.abort(reason)
            self.task = None
        self.saved = None

    def on_incident(self, inc: Dict[str, Any], ctx: TaskContext) -> str:
        """Route an incident payload: to the running T4, as a detour, or refuse."""
        if self.task is not None and self.task.name == "T4":
            return self.task.on_incident(inc, ctx)  # type: ignore[attr-defined]
        if inc.get("kind") == "assistance_request":
            if self.task is not None and not self.task.is_terminal:
                self.interrupt("T4", {"incident": inc}, ctx)
                return "detour started at breakpoint"
            self.start("T4", {"incident": inc}, ctx)
            return "T4 started"
        if inc.get("kind") in ("keep_out_zone", "all_clear", "moving_object"):
            # no T4 running: start one so the ack goes out and the constraint is stored, then keep listening
            if self.task is None:
                self.start("T4", {}, ctx)
            return self.task.on_incident(inc, ctx)  # type: ignore[attr-defined]
        return f"incident kind {inc.get('kind')} ignored (no T4 active)"

    # ---- per tick -----------------------------------------------------------------------
    def step(self, ctx: TaskContext) -> Optional[TaskStatus]:
        if self.task is None:
            return None
        st = self.task.step(ctx)
        self.last_status = st
        if st.done or st.aborted:
            finished = self.task
            self.task = None
            if self.saved is not None and finished.name == "T4" and st.done:
                bp = self.saved
                self.saved = None
                self.task = bp.task
                self.task.resume()
                log.info("resuming %s at %s after the T4 detour", self.task.name, bp.sub_state)
                return TaskStatus(running=True, sub_state=self.task.m.state, reason=f"resumed after detour from {bp.sub_state}")
            self.saved = None
        return st
