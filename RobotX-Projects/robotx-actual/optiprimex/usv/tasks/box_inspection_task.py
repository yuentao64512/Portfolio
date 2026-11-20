"""Guarded task-layer wrapper for the pure box-inspection mission core.

v0.1.27 (ROADMAP WP8): parameters from the profile's `box_inspect:` section (START_TASK args override); the camera
preflight reads the perception health beat, not detections; observing / strafing / yawing are mode-20 frames built by
behaviours/mode20 (the calibrated stern factors, heading hold); a core HOLD becomes a safe hold (AUTO_SAFE_HOLD with the
reason, RESUME goes on); stage stepping (args.step) and entry stages (args.from: OBSERVE, ALIGN, APPROACH) as the other tasks.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

from ...companion.fsm import Transition
from ...companion.frames import wrap180
from ...companion.state import GuidedTarget, Hold, RobotXTask, ThrusterFrame
from ..behaviours import mode20
from ..behaviours.base import BehaviourOut
from ..box_inspection import ActionKind, BoxInspectionMission, BoxState, MissionSnapshot, PointNE
from .base import Task, TaskContext, TaskStatus, simple_machine

DEFAULTS: Dict[str, Any] = {
    "scan_x_n": 0.0, "scan_y_e": 0.0,      # where the search starts (local N / E, m)
    "scan_hdg_deg": None,                  # the search looks this way (None = the heading on arrival)
    "expected_boxes": 3,                   # faces to see stable before choosing
    "target_colour": None,                 # None = the vehicle's rule: GREEN (handbook 3.3.4)
    "align_range_m": 15.0,                 # the align point on the face normal
    "stop_range_m": 1.0,                   # stop this far off the face (decided 2026-11-04)
    "align_pos_tol_m": 1.5,
    "align_timeout_s": 90.0,
    "require_blink": False,                # Task 3 bay indicators are steady
    "observe_s": 4.5,                      # dwell per look (BoxTrack.stable needs a 4 s window)
    "strafe_step_m": 2.0,                  # one strafe leg between looks
    "strafe_len_m": 6.0,                   # the whole search sways at most this far, then HOLD
    "strafe_side": "starboard",            # starboard | port (body frame)
    "strafe_v": 0.2,                       # sway command (mode 20)
    "health_age_s": 3.0,                   # the perception health beat must be this fresh
}
# a step park is not wanted inside the search loop or into / out of a hold (the hold is a safe hold itself)
_NO_PARK = {("OBSERVE", "STRAFE"), ("STRAFE", "OBSERVE")}


class BoxInspectionTask(Task):
    name = "BOX_INSPECT"
    robo_task = RobotXTask.NONE
    BEHAVIOURS: Dict[str, Any] = {}
    ENTRY_POINTS = {
        "OBSERVE": "search from here (no scan point)",
        "ALIGN": "one stable face of the target colour in the map",
        "APPROACH": "that face, and the boat within 3 m of its align point, heading within 15 deg",
    }

    def build_machine(self, on_transition, clock):
        states = [s.value for s in BoxState if s not in (BoxState.DONE, BoxState.ABORTED)]
        transitions = [Transition("TO_" + s.value, "*", s.value, reason="box mission") for s in BoxState]
        return simple_machine("BOX_INSPECT", states, transitions, BoxState.PREFLIGHT.value,
                              on_transition, clock)

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        p = dict(DEFAULTS)
        p.update(self.cfg.extra.get("box_inspect", {}) or {})
        p.update({k: v for k, v in (args or {}).items() if k in DEFAULTS and v is not None})
        self.p = p
        side = str(p["strafe_side"]).lower()
        if side not in ("starboard", "port"):
            raise ValueError("box_inspect strafe_side: starboard | port")
        self.core = BoxInspectionMission(
            PointNE(float(p["scan_x_n"]), float(p["scan_y_e"])),
            expected_boxes=int(p["expected_boxes"]),
            align_range_m=float(p["align_range_m"]),
            stop_range_m=float(p["stop_range_m"]),
            align_pos_tol_m=float(p["align_pos_tol_m"]),
            require_blink=bool(p["require_blink"]),
            observe_s=float(p["observe_s"]),
            strafe_step_m=float(p["strafe_step_m"]),
            strafe_len_m=float(p["strafe_len_m"]),
            strafe_side=1 if side == "starboard" else -1,
            align_timeout_s=float(p["align_timeout_s"]),
            scan_hdg_deg=None if p["scan_hdg_deg"] is None else float(p["scan_hdg_deg"]),
        )
        self.safe_hold: Optional[str] = None
        self._reported = False
        ctx.log(f"BOX_INSPECT started: scan ({p['scan_x_n']}, {p['scan_y_e']}), {p['expected_boxes']} faces, target "
                f"{p['target_colour'] or 'GREEN (rule)'}, stop {p['stop_range_m']} m, strafe {p['strafe_len_m']} m {side}")

    # ---- args.from ------------------------------------------------------------------------------------------------
    def check_entry(self, stage: str, ctx: TaskContext) -> str:
        if ctx.b.pose is None:
            return f"{stage} needs a local pose"
        return ""

    def enter_from(self, stage: str, ctx: TaskContext) -> str:
        if stage not in self.ENTRY_POINTS:
            return super().enter_from(stage, ctx)          # the base's refusal text (lists the entry points)
        why = self.check_entry(stage, ctx) or self.core.enter(stage, self._snapshot(ctx))
        if why:
            return why
        self._entering = True
        try:
            self.m.force(stage, reason=f"from {stage} (operator)")
        finally:
            self._entering = False
        return ""

    def resume(self) -> None:
        super().resume()
        self.core.resume(self.m.clock())
        self._sync_state()

    # ---- helpers --------------------------------------------------------------------------------------------------
    def _sync_state(self) -> None:
        wanted = self.core.state.value
        if self.m.state != wanted:
            src = self.m.state
            self.m.handle("TO_" + wanted)
            if (src, wanted) in _NO_PARK or wanted == "HOLD" or src == "HOLD":
                self._step_next = None

    def _camera_ok(self, ctx: TaskContext) -> bool:
        pmap = ctx.b.map
        if pmap is None:
            return False
        t = max(float(getattr(pmap, "last_alive_t", -1e9)), float(getattr(pmap, "last_ingest_t", -1e9)))
        return ctx.b.now - t <= float(self.p["health_age_s"])

    def _snapshot(self, ctx: TaskContext) -> MissionSnapshot:
        pose = ctx.b.pose
        sim = bool(self.args.get("sim"))
        pmap = ctx.b.map
        tracks = list(getattr(pmap, "box_tracks", {}).values()) if pmap is not None else []
        target_id = self.core.target.box_id if self.core.target else None
        visible = target_id is None or target_id in {b.box_id for b in self.core._stable_boxes(tracks)}
        return MissionSnapshot(
            now=ctx.b.now,
            position=PointNE(pose.x_n, pose.y_e),
            heading_deg=pose.hdg_deg,
            preflight={
                "rtk": sim or ctx.b.fcu.gps_fix >= 3,
                "heading": ctx.b.fcu.hdg_deg is not None,
                "camera": self._camera_ok(ctx),          # v0.1.27: the health beat, not detections (review §4.2)
                "thrusters": sim or ctx.b.fcu.params_ok,
                "ocs": True,  # bridge loss is enforced by the vehicle FSM's safe_hold policy
            },
            tracks=tracks,
            target_colour=self.args.get("target_colour") or self.p.get("target_colour"),
            target_visible=visible,
            ocs_link=True,
            rtk_ok=sim or ctx.b.fcu.gps_fix >= 3,
        )

    def _frame(self, ctx: TaskContext, u: float, v: float, heading_deg: Optional[float]) -> ThrusterFrame:
        g, amp = ctx.b.cfg.gains, float(ctx.b.cfg.limits.max_amp)
        r = 0.0 if heading_deg is None else mode20.yaw_cmd(g, wrap180(heading_deg - ctx.b.pose.hdg_deg))
        return mode20.frame(g, amp, u, v, r)

    def _direct_approach(self, target: PointNE, speed_mps: float, heading_deg: float, ctx: TaskContext) -> ThrusterFrame:
        pose = ctx.b.pose
        if pose is None:
            return ThrusterFrame.zero()
        # Mode-20 approach: speed request is converted conservatively to direct
        # surge authority; the heading loop preserves the calibrated aft-pair
        # convention.  The 10 Hz task tick supplies the 250 ms keepalive margin.
        max_speed = max(0.1, float(ctx.b.cfg.limits.max_speed_mps))
        u = min(float(ctx.b.cfg.limits.max_amp), max(0.08, speed_mps / max_speed * 0.35))
        # v0.1.27: line of sight onto the face normal through `target` (the stop point): steer back by the cross-track
        # error over a look-ahead, so a metre of ALIGN error does not end a metre off the bay's centre line.
        h = math.radians(heading_deg)
        xte = (pose.x_n - target.n) * -math.sin(h) + (pose.y_e - target.e) * math.cos(h)
        look = max(2.0, 0.5 * math.hypot(pose.x_n - target.n, pose.y_e - target.e))
        err = wrap180(heading_deg - math.degrees(math.atan2(xte, look)) - pose.hdg_deg)
        r = max(-0.25, min(0.25, ctx.b.cfg.gains.kpsi * err / 45.0))
        peak = max(abs(u + r), abs(u - r), 1e-9)
        scale = min(1.0, ctx.b.cfg.limits.max_amp / peak)
        return ThrusterFrame((u + r) * scale, 0.0, (u - r) * scale, 0.0)

    # ---- per tick -------------------------------------------------------------------------------------------------
    def step(self, ctx: TaskContext) -> TaskStatus:
        self.m.tick(ctx.b.now)
        if self.m.state == "DONE":
            return TaskStatus(running=False, done=True, sub_state="DONE")
        if self.m.state == "ABORTED":
            return TaskStatus(running=False, aborted=True, reason=self.core.reason, sub_state="ABORTED")
        if self.paused:
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=Hold()))
        pose = ctx.b.pose
        if pose is None:
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=Hold(), info={"reason": "no local pose"}))

        action = self.core.step(self._snapshot(ctx))
        self._sync_state()
        if self.core.hold_new:                     # a hold the operator resumes: AUTO_SAFE_HOLD with the reason
            self.safe_hold = f"BOX_INSPECT {self.core.held_from.value if self.core.held_from else ''}: {self.core.reason}"
            ctx.log(self.safe_hold)
        if self._step_next is not None:            # step mode: park at the boundary (as Task.step does)
            nxt, self._step_next = self._step_next, None
            self.safe_hold = f"step: next {nxt}"
            self.core.resume(ctx.b.now)            # the stage's timer starts when the operator resumes
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=Hold(), info={"step_next": nxt}))

        if action.kind == ActionKind.REPORT:
            self.report(ctx, "box_survey", dict(action.body))
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=None, info=action.body))   # no mode change
        if action.kind == ActionKind.DONE:
            self.report(ctx, "box_mission_complete", dict(action.body))
            return TaskStatus(running=False, done=True, sub_state="DONE", out=BehaviourOut(cmd=Hold()))
        if action.kind in (ActionKind.ALERT, ActionKind.BACK_AWAY):
            self.m.handle("TO_ABORTED")
            cmd = ThrusterFrame(-0.12, 0.0, -0.12, 0.0) if action.kind == ActionKind.BACK_AWAY else Hold()
            self.report(ctx, "alert", dict(action.body))
            return TaskStatus(running=False, aborted=True, reason=self.core.reason, sub_state="ABORTED",
                              out=BehaviourOut(cmd=cmd))
        if action.kind == ActionKind.GOTO and action.target is not None and ctx.b.origin is not None:
            lat, lon = ctx.b.origin.from_local(action.target.n, action.target.e)
            return TaskStatus(sub_state=self.m.state,
                              out=BehaviourOut(cmd=GuidedTarget(lat, lon, action.speed_mps), info=action.body))
        if action.kind == ActionKind.APPROACH and action.target is not None and action.heading_deg is not None:
            cmd = self._direct_approach(action.target, float(action.speed_mps or 0.15), action.heading_deg, ctx)
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=cmd, info=action.body))
        if action.kind == ActionKind.STILL:          # observing: mode 20, zero surge / sway, heading held
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=self._frame(ctx, 0.0, 0.0, action.heading_deg),
                                                                        info=action.body))
        if action.kind == ActionKind.YAW:            # ALIGN on arrival: turn in place to the approach heading
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=self._frame(ctx, 0.0, 0.0, action.heading_deg),
                                                                        info=action.body))
        if action.kind == ActionKind.STRAFE:         # one strafe leg, heading held, side from the core
            v = min(float(self.p["strafe_v"]), float(ctx.b.cfg.limits.max_amp)) * int(action.body.get("side", 1))
            return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=self._frame(ctx, 0.0, v, action.heading_deg),
                                                                        info=action.body))
        return TaskStatus(sub_state=self.m.state, out=BehaviourOut(cmd=Hold(), info=action.body))
