"""Generic state machine runner used by the vehicle layer, every task, and the OCS run lifecycle.

A Machine is a data table (states + transitions); the tests enumerate the table, so a change to the
state model is a change to data that a table-driven test will see.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Iterable, List, Optional

from .state import TaskStateEvent

Guard = Callable[[Any], bool]
Hook = Callable[[Any], None]


@dataclass
class State:
    name: str
    on_enter: Optional[Hook] = None
    on_exit: Optional[Hook] = None
    timeout_s: Optional[float] = None
    on_timeout: Optional[str] = None     # event raised when timeout_s elapses in this state


@dataclass
class Transition:
    event: str
    src: Iterable[str]                  # state names, or "*" for any
    dst: str
    guard: Optional[Guard] = None
    reason: str = ""


@dataclass
class Machine:
    name: str
    states: List[State]
    transitions: List[Transition]
    initial: str
    layer: str = "task"
    on_transition: Optional[Callable[[TaskStateEvent], None]] = None
    clock: Callable[[], float] = time.monotonic

    def __post_init__(self):
        self._states: Dict[str, State] = {s.name: s for s in self.states}
        if self.initial not in self._states:
            raise ValueError(f"{self.name}: initial state {self.initial!r} unknown")
        for t in self.transitions:
            if t.dst not in self._states:
                raise ValueError(f"{self.name}: transition to unknown state {t.dst!r}")
            for s in self._srcs(t):
                if s != "*" and s not in self._states:
                    raise ValueError(f"{self.name}: transition from unknown state {s!r}")
        self.state: str = self.initial
        self.entered_at: float = self.clock()
        self.history: List[TaskStateEvent] = []
        self._last_payload: Any = None

    @staticmethod
    def _srcs(t: Transition) -> List[str]:
        return [t.src] if isinstance(t.src, str) else list(t.src)

    # ---- queries ----------------------------------------------------------
    @property
    def since_s(self) -> float:
        return self.clock() - self.entered_at

    def can(self, event: str, payload: Any = None) -> Optional[Transition]:
        for t in self.transitions:
            if t.event != event:
                continue
            srcs = self._srcs(t)
            if "*" not in srcs and self.state not in srcs:
                continue
            if t.guard is not None and not t.guard(payload):
                continue
            return t
        return None

    def table(self) -> List[dict]:
        """The transition table as data (for tests and documentation)."""
        return [{"event": t.event, "src": self._srcs(t), "dst": t.dst, "guarded": t.guard is not None,
                 "reason": t.reason} for t in self.transitions]

    # ---- actions ------------------------------------------------------------
    def handle(self, event: str, payload: Any = None, reason: str = "") -> bool:
        t = self.can(event, payload)
        if t is None:
            return False
        self._go(t.dst, reason or t.reason or event, payload)
        return True

    def force(self, dst: str, reason: str, payload: Any = None) -> None:
        """Used by the vehicle layer for input-driven precedence moves (kill, override, fault)."""
        if dst not in self._states:
            raise ValueError(dst)
        if dst != self.state:
            self._go(dst, reason, payload)

    def _go(self, dst: str, reason: str, payload: Any) -> None:
        src = self.state
        s_old = self._states[src]
        s_new = self._states[dst]
        if s_old.on_exit:
            s_old.on_exit(payload)
        self.state = dst
        self.entered_at = self.clock()
        ev = TaskStateEvent(layer=self.layer, machine=self.name, src=src, dst=dst, reason=reason)
        self.history.append(ev)
        if self.on_transition:
            self.on_transition(ev)
        if s_new.on_enter:
            s_new.on_enter(payload)

    def tick(self, now: Optional[float] = None) -> Optional[str]:
        """Raise the current state's timeout event if due. Returns the event raised, if any."""
        now = self.clock() if now is None else now
        s = self._states[self.state]
        if s.timeout_s is not None and s.on_timeout and (now - self.entered_at) >= s.timeout_s:
            if self.handle(s.on_timeout, None, reason=f"timeout {s.timeout_s}s in {s.name}"):
                return s.on_timeout
        return None
