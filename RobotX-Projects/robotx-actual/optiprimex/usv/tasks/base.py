"""Task = a sub-state Machine where each sub-state owns one behaviour (MODULES tasks/base.py)."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

from ...companion.config import Config
from ...companion.fsm import Machine, State, Transition
from ...companion.state import Event, Hold, RobotXTask, TaskStateEvent
from ..behaviours.base import Behaviour, BehaviourContext, BehaviourOut
from ...companion.state import GuidedTarget, VelocityYawTarget
from ...companion import speed_k as SK


def speed_class(task, sub: str, beh) -> Optional[str]:
    """v0.1.94.3: which live speed knob a stage's output follows - the task's SPEED_CLASS first, else by its behaviour:
    Orbit -> circle, ScanYaw / StationKeep (in place) -> none, anything else that moves -> cruise."""
    own = getattr(task, "SPEED_CLASS", {}) or {}
    if sub in own:
        return own[sub]
    from ..behaviours.orbit import Orbit
    from ..behaviours.scan_yaw import ScanYaw
    from ..behaviours.station_keep import StationKeep
    if isinstance(beh, Orbit):
        return "circle"
    if beh is None or isinstance(beh, (ScanYaw, StationKeep)):
        return None
    return "cruise"


def scale_speed(out, cls: Optional[str], k: float) -> None:
    """v0.1.94.3: the operator's live multiplier on a task command (forward speed; a circle's yaw rate too, so its radius holds)."""
    if out is None or out.cmd is None or not cls or k == 1.0:
        return
    c = out.cmd
    if isinstance(c, GuidedTarget) and c.speed_mps is not None:
        out.cmd = GuidedTarget(c.lat, c.lon, c.speed_mps * k)
    elif isinstance(c, VelocityYawTarget):
        out.cmd = VelocityYawTarget(c.vx_mps * k, c.yaw_rate_rad_s * (k if cls == "circle" else 1.0))
    out.info["speed_k"] = k


def cap_speed(out, cfg, k: float = 1.0) -> None:
    """v0.1.92 (Dmitrii 2026-11-13: "cap the speed ... so if any skipping or freezing is happening, we still good"): every task's GUIDED
    carrot speed and velocity-yaw forward speed limited to limits.max_speed_mps (the tuning knob acts at once). Mode-20 frames keep max_amp;
    the guard's brake / back-off are not task commands and stay as they are; astern is capped by the same magnitude."""
    if out is None or out.cmd is None or cfg is None:
        return
    vmax = float(getattr(getattr(cfg, "limits", None), "max_speed_mps", 0.0) or 0.0)
    if vmax <= 0:
        return
    if k > 1.0:                                          # v0.1.94.3: the live knob lifts the cap with it, never above 1.5 m/s
        vmax = max(vmax, min(SK.V_CEIL, vmax * k))
    c = out.cmd
    if isinstance(c, GuidedTarget) and (c.speed_mps is None or c.speed_mps > vmax):
        out.cmd = GuidedTarget(c.lat, c.lon, vmax)
    elif isinstance(c, VelocityYawTarget) and abs(c.vx_mps) > vmax:
        out.cmd = VelocityYawTarget(vmax if c.vx_mps > 0 else -vmax, c.yaw_rate_rad_s)


@dataclass
class TaskContext:
    b: BehaviourContext                     # what behaviours see
    emit_event: Callable[[Event], None]     # task reports -> event/<v>/<kind>
    log: Callable[[str], None] = lambda s: None


@dataclass
class TaskStatus:
    running: bool = True
    done: bool = False
    aborted: bool = False
    reason: str = ""
    sub_state: str = ""
    out: Optional[BehaviourOut] = None


class Task:
    name = "task"
    robo_task = RobotXTask.NONE
    # sub-state -> factory of the behaviour that runs there (None = no motion, Hold by default)
    BEHAVIOURS: Dict[str, Optional[Callable[[], Behaviour]]] = {}
    # v0.1.26 (ROADMAP WP7): stages START_TASK args.from may name, with what their precondition is (check_entry() tests it)
    ENTRY_POINTS: Dict[str, str] = {}
    # v0.1.68 (WP17 F4): stages whose end is not motion even though a behaviour runs there (a station keep, a scan) - `skip` refuses
    # them with "<STAGE> ends on <this>, not on motion"; a stage without a behaviour is refused the same way (SKIP_DEFAULT)
    SKIP_REFUSE: Dict[str, str] = {}
    # v0.1.85 (WP41): a task that moves the boat - under the profile's guidance.mode underway its GUIDED carrots become the under-way law's
    # velocity-yaw targets (usv/guidance.py) and it starts with the move-first leg; mode fcu (the default) = unchanged
    UNDERWAY = False
    NEEDS_FIELD = False    # v0.1.85 (WP46): a task whose targets come from the course field - refused on a real judge without the RxCourse

    def __init__(self, cfg: Config, on_transition: Optional[Callable[[TaskStateEvent], None]] = None, clock=None):
        self.cfg = cfg
        self.m = self.build_machine(on_transition, clock)
        self._beh: Optional[Behaviour] = None
        self._beh_state: Optional[str] = None
        self.paused = False
        self.started = False
        self.args: Dict[str, Any] = {}
        self.breakpoint: Optional[str] = None
        # v0.1.26 stage stepping (args.step): every stage boundary parks the task; the mission manager reads safe_hold, puts the
        # vehicle in AUTO_SAFE_HOLD with that reason, and RESUME enters the stage. Generic: works for every task.
        self.safe_hold: Optional[str] = None
        self.step_mode = False
        self._step_next: Optional[str] = None
        self._entering = False
        inner = self.m.on_transition

        def _on_transition(ev, _inner=inner):
            if _inner:
                _inner(ev)
            if self.step_mode and not self._entering and ev.dst not in ("DONE", "ABORTED"):
                self._step_next = ev.dst
        self.m.on_transition = _on_transition

    # ---- to implement per task -------------------------------------------------------
    def build_machine(self, on_transition, clock) -> Machine:  # pragma: no cover - abstract
        raise NotImplementedError

    def behaviour_args(self, sub_state: str, ctx: TaskContext) -> Dict[str, Any]:
        return {}

    def on_behaviour_done(self, sub_state: str, ctx: TaskContext) -> None:
        """Called once when the sub-state's behaviour reports done; usually fires the next event."""

    def on_behaviour_abort(self, sub_state: str, reason: str, ctx: TaskContext) -> None:
        self.m.handle("ABORT", reason=reason)

    # ---- lifecycle --------------------------------------------------------------------
    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        self.args = dict(args or {})
        self.started = True
        self.step_mode = bool(self.args.get("step"))

    @property
    def holding(self) -> bool:
        """Paused, or parked at a stage boundary in step mode: a task's own step() must not advance its stage logic."""
        return self.paused or self._step_next is not None

    # ---- args.from (v0.1.26) ------------------------------------------------------------------------------------
    def precheck(self, ctx: TaskContext) -> str:
        """v0.1.30 (WP8c): '' when the task can run with its args now; else why not - START_TASK is then REJECTED at once,
        before the FSM moves (the pool: m0t2 without points failed 8 s later in PLAN). Called on a probe after start(args)."""
        return ""

    def check_entry(self, stage: str, ctx: TaskContext) -> str:
        """'' when the task may start at `stage` now; else why not. Tasks with ENTRY_POINTS override it."""
        return ""

    def enter_from(self, stage: str, ctx: TaskContext) -> str:
        """Jump to an entry stage after start(); '' on success, else the refusal (nothing changed)."""
        if stage not in self.ENTRY_POINTS:
            pts = ", ".join(self.ENTRY_POINTS) or "none"
            return f"{stage} is not an entry point of {self.name} (entry points: {pts})"
        why = self.check_entry(stage, ctx)
        if why:
            return why
        self._entering = True
        try:
            self.m.force(stage, reason=f"from {stage} (operator)")
        finally:
            self._entering = False
        return ""

    # ---- v0.1.68 (WP17 F4): skip - end the current stage's behaviour now (free run only; the command handler checks that) ----------
    def skip_check(self) -> str:
        """'' when `skip` may end the current stage's behaviour now; else why not."""
        st = self.m.state
        if self.is_terminal:
            return f"{self.name} is {st}"
        if self._step_next is not None:
            return f"parked before {self._step_next}: resume enters it"
        if self.paused:
            return f"{st} is held: resume first"
        why = self.SKIP_REFUSE.get(st)
        if why is None and self.BEHAVIOURS.get(st) is None:
            why = "a report" if st.startswith(("REPORT", "ACK")) else "a timer"
        if why:
            return f"{st} ends on {why}, not on motion"
        return ""

    def skip(self, ctx: TaskContext) -> str:
        """The runner's skip: cancel the stage's behaviour, call on_behaviour_done(stage) - the task's own next event fires."""
        why = self.skip_check()
        if why:
            return why
        sub = self.m.state
        if self._beh:
            self._beh.cancel()
        self._beh, self._beh_state = None, None
        self.on_behaviour_done(sub, ctx)
        ctx.log(f"{self.name} {sub}: skip (free run) - the behaviour ended by the operator; now {self.m.state}")
        return f"{sub} ended by skip -> {self.m.state}"

    def report(self, ctx: TaskContext, kind: str, body: Dict[str, Any]) -> None:
        b = dict(body)
        b["event"] = kind
        ctx.emit_event(Event(kind=kind, body=b))

    def pause(self) -> None:
        self.paused = True
        self.breakpoint = self.m.state
        if self._beh:
            self._beh.cancel()
            self._beh = None
            self._beh_state = None

    def resume(self) -> None:
        self.paused = False

    def abort(self, reason: str) -> None:
        if self._beh:
            self._beh.cancel()
            self._beh = None
        self.m.handle("ABORT", reason=reason)

    @property
    def is_terminal(self) -> bool:
        return self.m.state in ("DONE", "ABORTED")

    def step(self, ctx: TaskContext) -> TaskStatus:
        self.m.tick(ctx.b.now)
        st = TaskStatus(sub_state=self.m.state)
        if self.m.state == "DONE":
            st.running, st.done = False, True
            return st
        if self.m.state == "ABORTED":
            st.running, st.aborted = False, True
            st.reason = self.m.history[-1].reason if self.m.history else ""
            return st
        if self.paused:
            return st
        if self._step_next is not None:            # step mode: park at the boundary before the stage's behaviour starts
            nxt, self._step_next = self._step_next, None
            self.safe_hold = f"step: next {nxt}"
            self._beh = None
            st.out = BehaviourOut(cmd=Hold(), info={"step_next": nxt})
            return st
        factory = self.BEHAVIOURS.get(self.m.state)
        if factory is None:
            self._beh = None
            self._beh_state = None
            return st
        if self._beh is None or self._beh_state != self.m.state:
            self._beh = factory()
            self._beh.start(self.behaviour_args(self.m.state, ctx), ctx.b)
            self._beh_state = self.m.state
        out = self._beh.step(ctx.b)
        if self.UNDERWAY:
            self._underway(out, ctx)
        cls = speed_class(self, self.m.state, self._beh)   # v0.1.94.3: the operator's live speed knobs (cruise / circle / gate)
        k = SK.k_of(self.cfg, cls)
        scale_speed(out, cls, k)
        cap_speed(out, self.cfg, k)                      # v0.1.92: limits.max_speed_mps on every task command (was a label only)
        st.out = out
        if out.abort:
            sub = self.m.state
            self._beh = None
            self.on_behaviour_abort(sub, out.abort, ctx)
            st.reason = out.abort
        elif out.done:
            sub = self.m.state
            self._beh = None
            self.on_behaviour_done(sub, ctx)
        st.sub_state = self.m.state
        return st


    # ---- v0.1.85 (WP41): turn under way --------------------------------------------------------------------------------------
    def course_poly(self, ctx: TaskContext):
        """The course polygon (local) the guidance's boundary rule uses: the task's own field when it has one, else the RxCourse on the boat."""
        own = getattr(self, "poly", None)
        if own:
            return own
        from .. import guidance as GD
        return GD.course_local(ctx.b.run, ctx.b.origin)

    def guidance_obstacles(self, ctx: TaskContext):
        from ..gatepath import planner as gp
        return gp.map_obstacles(ctx.b.map, int(((getattr(self, "p", None) or {}).get("min_obs")) or 3))

    def _underway(self, out: BehaviourOut, ctx: TaskContext) -> None:
        from .. import guidance as GD
        if not GD.underway(self.cfg) or out is None:
            return
        if getattr(self, "_steer", None) is None:
            self._steer = GD.Steer(self.cfg, log=ctx.log)
            self._steer.turn_first = getattr(self, "turn_first_deg", None)   # v0.1.92: a task's own turn-first (T1's gate legs)
            self._move_first = GD.MoveFirst(self.cfg, log=ctx.log)
        poly = self.course_poly(ctx)
        obst = self.guidance_obstacles(ctx)
        mf = self._move_first.cmd(ctx.b.now, ctx.b.pose, obst, poly)
        if mf is not None:
            out.cmd = mf
            out.info["guidance"] = {"why": "move-first"}
            return
        beh = self._beh
        pts = getattr(beh, "pts", None)
        end_xy = tuple(pts[-1]) if pts else (getattr(beh, "target_local", None) or None)
        out.cmd = self._steer.shape(out.cmd, ctx.b.now, ctx.b.pose, ctx.b.origin, obst, poly, leg=(self.m.state, id(beh)), end_xy=end_xy)
        out.info["guidance"] = dict(self._steer.last)


def simple_machine(name: str, states: List[str], transitions: List[Transition], initial: str,
                   on_transition, clock, timeouts: Optional[Dict[str, tuple]] = None) -> Machine:
    """Every task machine gets DONE and ABORTED plus a universal ABORT transition."""
    st = [State(s) for s in states]
    for s in st:
        if timeouts and s.name in timeouts:
            s.timeout_s, s.on_timeout = timeouts[s.name]
    if "DONE" not in states:
        st.append(State("DONE"))
    if "ABORTED" not in states:
        st.append(State("ABORTED"))
    tr = list(transitions) + [Transition("ABORT", "*", "ABORTED", reason="abort")]
    return Machine(name=name, layer="task", states=st, transitions=tr, initial=initial,
                   on_transition=on_transition, clock=clock or __import__("time").monotonic)
