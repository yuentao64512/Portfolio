"""Task 4 Dynamic Incident Response, core chain (ARCHITECTURE 3.3, 10.9):

LISTEN -> (AssistanceRequest) ACK -> TRANSIT (goto the request position) -> READY (loiter, readiness report)
-> WAIT_CONFIRM (ReadinessConfirm) -> DONE.
Advanced KeepOutZone: ack, store the polygon (planner constraint); AllClear: ack, clear.
Disruptive MovingObjectAlert: store the track and keep-out radius; no ack exists in the schema.
v0.1.67 (borrowed from the UAV team's task4 - their invariant 9; docs/TODO_UAV_BORROWINGS.md B4): the ReadinessReport goes out once the
boat has HELD - within `t4.hold_r_m` (3 m) of where READY began, under `t4.hold_v_mps` (0.7 m/s), for `t4.hold_s` (3 s) without a break -
not on the first tick of READY; after `t4.ready_max_s` (20 s) it goes anyway (logged): a judge watching the boat sees it on station.

When the task is started as a breakpoint detour (another task was running) the runner starts it
with args {"incident": ...} so it begins at ACK and DONE returns the runner to the saved task.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

from ...companion.fsm import Transition
from ...companion.state import RobotXTask, utc_iso
from ..behaviours import Goto, StationKeep
from .base import Task, TaskContext, simple_machine


class T4Incident(Task):
    name = "T4"
    robo_task = RobotXTask.T4
    SKIP_REFUSE = {"LISTEN": "a report", "READY": "a timer", "WAIT_CONFIRM": "a report"}   # v0.1.68 (F4)
    BEHAVIOURS = {
        "LISTEN": StationKeep,
        "ACK": None,
        "TRANSIT": Goto,
        "READY": StationKeep,
        "WAIT_CONFIRM": StationKeep,
    }

    def build_machine(self, on_transition, clock):
        return simple_machine(
            "T4", ["LISTEN", "ACK", "TRANSIT", "READY", "WAIT_CONFIRM"],
            [
                Transition("ASSISTANCE", ["LISTEN"], "ACK", reason="AssistanceRequest"),
                Transition("ACKED", ["ACK"], "TRANSIT", reason="IncidentAck sent"),
                Transition("ARRIVED", ["TRANSIT"], "READY", reason="at the request position"),
                Transition("READY_SENT", ["READY"], "WAIT_CONFIRM", reason="ReadinessReport sent"),
                Transition("CONFIRMED", ["WAIT_CONFIRM"], "DONE", reason="ReadinessConfirm received"),
                Transition("CONFIRM_TIMEOUT", ["WAIT_CONFIRM"], "DONE", reason="no ReadinessConfirm, resuming anyway"),
            ],
            initial="LISTEN", on_transition=on_transition, clock=clock,
            timeouts={"WAIT_CONFIRM": (float(self.cfg.extra.get("t4", {}).get("confirm_timeout_s", 120.0)), "CONFIRM_TIMEOUT")},
        )

    def start(self, args: Dict[str, Any], ctx: TaskContext) -> None:
        super().start(args, ctx)
        self.incident: Optional[Dict[str, Any]] = None
        self.keep_out: Optional[Dict[str, Any]] = None
        self.moving_object: Optional[Dict[str, Any]] = None
        self.readiness_seq: Optional[int] = None
        if args.get("incident"):
            self.on_incident(args["incident"], ctx)

    # ---- inputs from the command handler --------------------------------------------------
    def on_incident(self, inc: Dict[str, Any], ctx: TaskContext) -> str:
        """inc = {kind, command_seq, position{lat,lon}, domain, polygon, ...}. Returns a short result."""
        kind = inc.get("kind")
        if kind == "assistance_request":
            if self.m.state not in ("LISTEN",):
                return f"assistance request ignored in {self.m.state}"
            self.incident = dict(inc)
            self.m.handle("ASSISTANCE")
            return "accepted"
        if kind == "keep_out_zone":
            self.keep_out = dict(inc)
            self.report(ctx, "incident", {"kind": "ack", "command_seq": inc.get("command_seq"), "what": "keep_out_zone", "t": utc_iso()})
            return "keep-out stored and acked"
        if kind == "all_clear":
            self.keep_out = None
            self.report(ctx, "incident", {"kind": "ack", "command_seq": inc.get("command_seq"), "what": "all_clear", "t": utc_iso()})
            return "all clear acked"
        if kind == "moving_object":
            self.moving_object = dict(inc)
            return "moving object stored (no ack in the schema)"
        if kind == "readiness_confirm":
            if self.m.state == "WAIT_CONFIRM":
                self.m.handle("CONFIRMED")
                return "confirmed"
            return f"readiness confirm ignored in {self.m.state}"
        return f"unknown incident kind {kind}"

    # ---- sub-state behaviour wiring ---------------------------------------------------------
    def behaviour_args(self, sub_state: str, ctx: TaskContext) -> Dict[str, Any]:
        if sub_state == "TRANSIT" and self.incident:
            pos = self.incident.get("position") or {}
            out = {"lat": pos.get("lat"), "lon": pos.get("lon"),           # v0.1.94: t4.speed_mps (0.3, as simulated) - not the T1 cap
                   "speed_mps": self.incident.get("speed_mps") or float((self.cfg.extra.get("t4", {}) or {}).get("speed_mps", 0.3)),
                   "timeout_s": self.cfg.extra.get("t4", {}).get("transit_timeout_s", 300.0)}
            b = ctx.b                                   # v0.1.54: round the mapped course objects (the 0.60 m clearance rule)
            if b.pose is not None and b.origin is not None and b.map is not None and pos.get("lat") is not None:
                from ..gatepath import planner as gp
                t1 = dict(self.cfg.extra.get("t1", {}) or {})
                clr = gp.Clearance(clear_m=float(t1.get("clear_m", 0.60)), track_m=float(t1.get("track_m", 0.45)))
                goal = b.origin.to_local(float(pos["lat"]), float(pos["lon"]))
                obs = gp.map_obstacles(b.map, int(t1.get("min_obs", 5)))
                t4 = dict(self.cfg.extra.get("t4", {}) or {})
                # v0.1.94 (fork B's T4 sim: 2 contacts in 100 detours, both on well-mapped buoys): the waiting point keeps an extra margin for
                # the arrival swing (checked as a point it sat 1.7 m off and the hull swung to 0.53 m), and a request that finds the boat inside
                # a keep disc (mid-gate) first backs out to the nearest free point
                free = gp.free_point(goal, obs, clr, margin=0.2 + float(t4.get("arrive_margin_m", 0.8)))
                if math.hypot(free[0] - goal[0], free[1] - goal[1]) > 0.05:          # the request next to an object: wait beside it
                    ctx.log(f"T4 the request lies {gp.path_clearance([goal], obs, clr):.2f} m from a course object: waiting "
                            f"{math.hypot(free[0] - goal[0], free[1] - goal[1]):.1f} m off it (the clearance rule)")
                    out.pop("lat"), out.pop("lon")
                    out["x_n"], out["y_e"] = free
                    goal = free
                start = (b.pose.x_n, b.pose.y_e)
                out_of = gp.free_point(start, obs, clr)
                if math.hypot(out_of[0] - start[0], out_of[1] - start[1]) > 0.05:
                    ctx.log(f"T4 the boat is inside a course object's keep disc: out to the nearest free point "
                            f"{math.hypot(out_of[0] - start[0], out_of[1] - start[1]):.1f} m away first")
                    out["via"] = [tuple(out_of)] + gp.via_points(tuple(out_of), goal, obs, clr)
                else:
                    out["via"] = gp.via_points(start, goal, obs, clr)
                if out["via"]:
                    ctx.log(f"T4 transit round {len(out['via'])} corner(s): the course objects on the straight line")
            return out
        return {}

    def step(self, ctx: TaskContext):
        # ACK and READY have side effects once on entry; handled here because behaviours are pure
        if self.m.state == "ACK" and not self.holding:
            self.report(ctx, "incident", {"kind": "ack", "command_seq": (self.incident or {}).get("command_seq"),
                                          "what": "assistance_request", "t": utc_iso()})
            self.m.handle("ACKED")
        elif self.m.state == "READY" and not self.holding and self.readiness_seq is None and self.held(ctx):
            self.readiness_seq = (self.incident or {}).get("command_seq")
            pos = {"lat": ctx.b.fcu.lat, "lon": ctx.b.fcu.lon}
            self.report(ctx, "incident", {"kind": "readiness", "command_seq": self.readiness_seq, "position": pos, "t": utc_iso()})
            self.m.handle("READY_SENT")
        elif self.m.state == "TRANSIT" and not self.holding:
            self.replan_check(ctx)
        return super().step(ctx)

    def held(self, ctx: TaskContext) -> bool:
        """B4: on station - within hold_r_m of READY's anchor, slower than hold_v_mps, for hold_s; or READY older than ready_max_s."""
        t4 = dict(self.cfg.extra.get("t4", {}) or {})
        r_m, v_max = float(t4.get("hold_r_m", 3.0)), float(t4.get("hold_v_mps", 0.7))
        hold_s, max_s = float(t4.get("hold_s", 3.0)), float(t4.get("ready_max_s", 20.0))
        now, pose = ctx.b.now, ctx.b.pose
        if getattr(self, "_ready_t0", None) is None or getattr(self, "_ready_for", None) != (self.incident or {}).get("command_seq"):
            self._ready_t0, self._ready_for = now, (self.incident or {}).get("command_seq")
            self._anchor = (pose.x_n, pose.y_e) if pose is not None else None
            self._still_t0 = None
        if hold_s <= 0:
            return True
        if pose is None or self._anchor is None:
            return now - self._ready_t0 >= max_s
        sog = getattr(pose, "sog_mps", None)
        if sog is None:
            sog = getattr(ctx.b.fcu, "groundspeed", None) or 0.0
        on = math.hypot(pose.x_n - self._anchor[0], pose.y_e - self._anchor[1]) <= r_m and float(sog) <= v_max
        if not on:
            self._still_t0 = None
        elif self._still_t0 is None:
            self._still_t0 = now
        if self._still_t0 is not None and now - self._still_t0 >= hold_s:
            ctx.log(f"T4 on station {now - self._ready_t0:.1f} s after arriving (within {r_m:g} m, < {v_max:g} m/s for {hold_s:g} s): readiness")
            return True
        if now - self._ready_t0 >= max_s:
            ctx.log(f"T4 not settled after {max_s:g} s on station - readiness anyway (the judges see the attempt)")
            return True
        return False

    def replan_check(self, ctx: TaskContext) -> None:
        """v0.1.54: the map fills on the way (the request comes at once): the transit is planned again - at most once a second - when
        the rest of its way now passes a mapped object closer than the clearance rule."""
        b = self._beh
        if self.m.state != "TRANSIT" or b is None or getattr(b, "target_local", None) is None or ctx.b.pose is None:
            return
        if ctx.b.now - getattr(self, "_replan_t", -1e9) < 1.0:
            return
        self._replan_t = ctx.b.now
        from ..gatepath import planner as gp
        t1 = dict(self.cfg.extra.get("t1", {}) or {})
        clr = gp.Clearance(clear_m=float(t1.get("clear_m", 0.60)), track_m=float(t1.get("track_m", 0.45)))
        rest = [(ctx.b.pose.x_n, ctx.b.pose.y_e)] + list(b.via) + [tuple(b.target_local)]
        obs = gp.map_obstacles(ctx.b.map, int(t1.get("min_obs", 5)))
        if gp.path_clearance(gp.resample(rest, 0.25), obs, clr) >= clr.clear_m:
            return
        b.cancel()
        self._beh = None                                 # the next step starts the transit again on the grown map

    def on_behaviour_done(self, sub_state: str, ctx: TaskContext) -> None:
        if sub_state == "TRANSIT":
            self.m.handle("ARRIVED")

    def on_behaviour_abort(self, sub_state: str, reason: str, ctx: TaskContext) -> None:
        if sub_state == "TRANSIT":
            # cannot reach the point: still report readiness from where we are (judges see the attempt)
            ctx.log(f"T4 transit aborted: {reason}; reporting readiness from the current position")
            self.m.handle("ARRIVED", reason=reason)
        else:
            super().on_behaviour_abort(sub_state, reason, ctx)
