"""RunContext: PREPARE / START_RUN / STOP_RUN and the 'nothing from a previous run' rule (3.4.13)."""
from __future__ import annotations

from typing import Tuple

from ..companion.state import Command, Course, RunContext, utc_iso

# Accepted regardless of run ids: the run bookkeeping commands themselves, and every command that REDUCES autonomy.
# An operator must always be able to hold, abort, drop to manual or confirm safe, whatever run the envelope names
# (3.4.13 forbids reusing a previous run's *task* context, not stopping). Starting things stays run-scoped.
RUN_FREE_ACTIONS = ("PREPARE", "CONFIRM_SAFE", "COURSE", "STOP_RUN", "RESET", "HOLD", "ABORT_TASK", "DISABLE_AUTO", "STEP",
                    "SAVE_POINT", "DELETE_POINT", "CLEAR_MAP", "HOME",   # v0.1.29: points are data, not the run
                    "SET_TUNING",                                        # v0.1.49: a knob for the next task start (MANUAL / AUTO_HOLD only)
                    "SET_SPEED_K",                                       # v0.1.94.3: the live speed knobs (any state, at once)
                    "FCU_MODE", "FCU_ARM", "SET_RC6",                    # v0.1.50: the operator's FCU buttons and the RC6 policy, any run
                    "FENCE_APPLY", "FENCE_ENABLE")                       # v0.1.53: the FCU geofence (refused in AUTO_TASK by the handler)


def accepts(run: RunContext, cmd: Command) -> Tuple[bool, str]:
    """Reject anything whose run_id / declaration_seq does not match the current context."""
    if cmd.action in RUN_FREE_ACTIONS:
        return True, ""
    if cmd.declaration_seq is not None and run.declaration_seq is not None and cmd.declaration_seq != run.declaration_seq:
        return False, f"declaration_seq {cmd.declaration_seq} != {run.declaration_seq}"
    if cmd.action == "START_RUN":
        if run.declaration_seq is None:
            return False, "START_RUN before PREPARE"
        return True, ""
    if cmd.run_id is not None:
        if run.run_id is None:
            return False, f"run {cmd.run_id} unknown (no START_RUN)"
        if cmd.run_id != run.run_id:
            return False, f"run_id {cmd.run_id} != {run.run_id}"
    return True, ""


def apply(run: RunContext, cmd: Command) -> Tuple[bool, str]:
    """Apply a run-context command. Returns (changed, reason)."""
    a, args = cmd.action, cmd.args
    if a == "PREPARE":
        seq = args.get("declaration_seq", cmd.declaration_seq)
        if seq is None:
            return False, "PREPARE without declaration_seq"
        run.declaration_seq = int(seq)
        run.run_id = None
        run.tiers = dict(args.get("tiers") or {})
        run.prepared_at = utc_iso()
        run.started_at = run.ended_at = None
        if "course" in args:
            _apply_course(run, args["course"])
        return True, ""
    if a == "COURSE":
        _apply_course(run, args)
        return True, ""
    if a == "START_RUN":
        rid = args.get("run_id", cmd.run_id)
        if rid is None:
            return False, "START_RUN without run_id"
        run.run_id = int(rid)
        run.started_at = utc_iso()
        return True, ""
    if a == "STOP_RUN":
        run.ended_at = utc_iso()
        run.run_id = None
        return True, ""
    if a == "RESET":
        run.ended_at = utc_iso()
        run.run_id = None
        run.declaration_seq = None
        run.tiers = {}
        run.prepared_at = run.started_at = None
        return True, ""
    return False, f"not a run command: {a}"


def _apply_course(run: RunContext, c: dict) -> None:
    run.course = Course(
        course_id=c.get("course_id"),
        boundary=[tuple(p) for p in c.get("boundary", [])],
        uav_transit_area=[tuple(p) for p in c.get("uav_transit_area", [])],
        pinger_hz=c.get("pinger_hz"),
        uav_geofence=[tuple(p) for p in c.get("uav_geofence", [])],
        ceiling_hae_m=c.get("ceiling_hae_m"),
    )
