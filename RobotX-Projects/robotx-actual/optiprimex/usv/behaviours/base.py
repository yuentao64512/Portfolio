from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional

from ...companion.config import Config
from ...companion.frames import ControlOrigin
from ...companion.state import FcuState, LocalPose, RunContext


@dataclass
class BehaviourContext:
    """What a behaviour may read each tick. Nothing here sends anything."""
    cfg: Config
    now: float
    fcu: FcuState
    pose: Optional[LocalPose]
    origin: Optional[ControlOrigin]
    run: RunContext
    map: Any = None


@dataclass
class BehaviourOut:
    cmd: Any = None            # ThrusterFrame | GuidedTarget | VelocityYawTarget | Hold | None
    done: bool = False
    abort: Optional[str] = None
    info: Dict[str, Any] = field(default_factory=dict)


class Behaviour:
    name = "behaviour"

    def start(self, args: Dict[str, Any], ctx: BehaviourContext) -> None:
        self.args = dict(args)
        self.t_start = ctx.now

    def step(self, ctx: BehaviourContext) -> BehaviourOut:  # pragma: no cover - abstract
        raise NotImplementedError

    def cancel(self) -> None:
        pass
