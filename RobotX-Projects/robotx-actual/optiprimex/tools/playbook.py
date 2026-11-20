"""playbook: the usage patterns of the operator intents, in one place.

Mirrors the two guard tables the live system applies (ocs/cmd_authority.GUARDS on the OCS side, usv/vehicle_fsm on the
boat side) so that a console can say *before* sending an intent whether it will be refused or rejected, and why, and
what to do first. It never replaces those guards: the OCS and the vehicle still decide. Used by ocs_cli (help, next,
hints after a refusal), by the bench TUI (the command panel) and by the scenario runner.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Tuple, Dict, List, Optional, Sequence

from ..ocs import declaration_checks as DC
from ..ocs.cmd_authority import RUN_FREE_TASKS, GUARDS, NEVER
from ..ocs.run_lifecycle import R

R_ORDER = list(R.keys())                       # R0..R6
V_ORDER = ["BOOT", "KILLED", "MANUAL", "AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD", "FAULT"]
V_CODE = {name: f"V{i}" for i, name in enumerate(V_ORDER)}
AUTO = ("AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD")


@dataclass
class Entry:
    action: str
    key: str                      # one-letter key in the TUI
    cli: str                      # ocs_cli spelling
    does: str                     # what it does, one line
    ocs_r: Sequence[str]          # R states where the OCS lifecycle accepts it ("*" = any R the GUARDS allow)
    vehicle_v: Sequence[str]      # V states where the vehicle accepts it ("*" = any, "-" = not a vehicle command)
    needs: str = ""               # extra precondition in words
    then: str = ""                # what happens after
    next: str = ""                # typical next step
    bench_only: bool = False
    args: str = ""


PLAYBOOK: Dict[str, Entry] = {e.action: e for e in [
    Entry("DECLARE", "d", "declare", "publish the RunDeclaration to the judge; PREPARE (+course) to every vehicle; heartbeats start",
          ("R2", "R4", "R6"), ("*",), then="R3 DECLARED; the judge answers with RunStart (fake judge: after --start-delay) -> R5",
          next="enable_auto"),
    Entry("ENABLE_AUTO", "a", "enable_auto", "vehicle MANUAL -> AUTO_HOLD (HOLD mode on the FCU)",
          ("*",), ("MANUAL",), needs="armed, EKF ok, PREPARE seen, no kill / RC override / fault",
          then="V3 AUTO_HOLD; STATE_AUTO in the heartbeat; R3 -> R4 when every declared vehicle is in AUTO",
          next="start_task m4t0 (needs RunStart, R5)"),
    Entry("DISABLE_AUTO", "A", "disable_auto", "vehicle AUTO_HOLD -> MANUAL",
          ("*",), ("AUTO_HOLD",), needs="no task running: abort first (ABORT_TASK) if in AUTO_TASK / SAFE_HOLD",
          then="V2 MANUAL; STATE_MANUAL", next="enable_auto to go back"),
    Entry("START_TASK", "t", "start_task m4t0", "start a mission by its code (m4t0 = T4 built; goto on the bench; m0t0 / m0t1 = no-motion test missions)",
          ("R5",), ("AUTO_HOLD",), needs="a verified RunStart (run_id known to the vehicle)",
          then="V4 AUTO_TASK; m4t0 waits in LISTEN for an incident; m0t* start in S1", next="inject assistance (bench) or wait for the judge; step for m0t0",
          args="mission code m4t0 | goto | m0t0 | m0t1 | m1t1 m1t2 m3t1 m0t2 m0t3 (not built: rejected); +sim = fake FCU only; old names T4 / TEST_AUTO work"),
    Entry("SELECT_TARGET", "-", "select_target red", "m0t4: override the vehicle's GREEN rule (the face colour to approach)",
          ("R5",), ("AUTO_TASK", "AUTO_SAFE_HOLD"), needs="BOX_INSPECT before ALIGN (PREFLIGHT .. ASK_OCS, or held)",
          then="the task approaches that colour's face (exactly one must be stable)", next="resume if held; watch the approach",
          args="red | green | blue"),
    Entry("SAVE_POINT", "-", "save_point A", "store a named point in the boat's points.json (usv_ui's file): the boat's own fix, or --lat/--lon "
          "(the map's drop point) - m0t2 / m0t3 use them by name (--names A,B)",
          ("*",), ("*",), needs="a mission manager on the boat; no lat/lon = a 3D fix on the boat",
          then="the point appears on the map and in MAIN's points list (overwrites a point of that name)", next="mission m0t2 --names A,B,...",
          args="NAME [--lat LAT --lon LON]"),
    Entry("DELETE_POINT", "-", "del_point A", "remove a named point from the boat's points.json",
          ("*",), ("*",), then="gone from the map and the list", next="-", args="NAME"),
    Entry("HOME", "-", "home", "the go-home override: abort whatever runs and drive slowly (0.3 m/s, GUIDED) to the saved point "
          "HOME (else the control origin), then stop and hold - with or without a run",
          ("*",), ("AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD"), needs="an AUTO state (in MANUAL the boat is yours: the transmitter)",
          then="AUTO_TASK RETURN_HOME GO -> DONE -> AUTO_HOLD at HOME", next="manual / disable_auto, or a new mission",
          args="[--speed V]"),
    Entry("FCU_MODE", "-", "fcu_mode HOLD", "v0.1.50: the FCU's mode from a page (MANUAL / HOLD / ...), as Mission Planner would - the FCU "
          "decides; with RC6 latched it also clears the latch (last wins)", ("*",), ("*",), then="the FCU's mode in the bar within a second",
          next="enable_auto (from HOLD)", args="MODE"),
    Entry("FCU_ARM", "-", "arm | disarm", "v0.1.50: ARM / DISARM the FCU from a page; its pre-arm checks decide (the transmitter's kill "
          "still wins)", ("*",), ("*",), then="armed / disarmed in the bar", next="fcu_mode HOLD, enable_auto"),
    Entry("FENCE_APPLY", "-", "fence apply course", "v0.1.53: the USV's FCU geofence - the received course (RxCourse) or the OCS profile's "
          "geofence as one inclusion polygon (MAVLink fence upload, MISSION_ACK), read back and compared within 1e-6 deg; nothing is "
          "enabled by it", ("*",), ("*",), needs="not in AUTO_TASK; a received course (source course); a geofence inside the course",
          then="the ack: 'fence N points uploaded, read back identical' or the FCU's MAV_MISSION_* refusal; status fence.points_n / matches",
          next="fence on", args="course | geofence"),
    Entry("FENCE_ENABLE", "-", "fence on | fence off", "v0.1.53: MAV_CMD_DO_FENCE_ENABLE (a command with COMMAND_ACK, not the FENCE_ENABLE "
          "parameter); off by default - what a breach does is the FCU's FENCE_ACTION (apply_params.py preset)", ("*",), ("*",),
          needs="on: a fence on the FCU (fence apply first), not in AUTO_TASK; off: always",
          then="status fence.enabled from the FCU (SYS_STATUS geofence bit, else the ack)", next="-", args="on | off"),
    Entry("STATION_KEEP", "-", "station_keep hold|loiter", "v0.1.81 (WP37): the autonomous station keep - hold = FCU HOLD (motors neutral, "
          "drifting), loiter = FCU LOITER (a real position hold: AUTO_HOLD and every stage that holds); the safe hold stays HOLD; kept across "
          "runs; default hold until the LOITER drift check on the water (Test Guide W1)", ("*",), ("*",), then="status station_keep",
          next="-", args="hold | loiter"),
    Entry("RC6", "-", "rc6_policy latched|hold", "v0.1.50: the RC6 override's rule - latched: a later page command wins over a high RC6 (the "
          "next flip takes the boat back); hold: while RC6 is high, no autonomy", ("*",), ("*",), then="the ack names the rule",
          next="-", args="latched | hold"),
    Entry("REPORT", "-", "report on|off", "v0.1.53: START / STOP REPORTING - the OCS's heartbeats for every vehicle to RoboCommand at 2 Hz "
          "before a declaration (the handbook: at all times in the course area); off = none now, whatever R (v0.1.54)", ("*",), ("*",),
          then="the judge page's JUDGE RECEIVES shows our heartbeats", next="declare", args="on | off"),
    Entry("USV_READY", "-", "usv ready on|off", "v0.1.82: the boat's PILOT READY - an OCS flag; with ocs.usv_heartbeat button DECLARE refuses the "
          "boat until ▶ HEARTBEAT USV (report on usv) and this; cleared by STOP / RESET", ("*",), ("*",),
          needs="report on usv (its heartbeat to RoboCommand)", then="DECLARE may carry the boat", next="declare", args="on | off"),
    Entry("DECLARE_UAV", "-", "declare uav on|off", "v0.1.64 (WP14): the UAV (UAV1, drone SGIT-53577) in the next declaration - its FCU fence and "
          "FENCE_ALT_MAX, the pilot's call sign; off by default, m1t2 needs it on; kept across runs", ("*",), ("*",),
          needs="the UAV adapter up (launcher UAV on) for the declare itself", then="the next declare carries both vehicles",
          next="uav ready on, declare", args="on | off"),
    Entry("UAV", "-", "uav ready on|off", "v0.1.64 (WP14): the UAV's words - ready on|off (the pilot said ready: the UAV's PREPARE can go "
          "DONE), task N|none (current_task by hand), resend_course (the cached course to the Orin again)", ("*",), ("*",),
          needs="the UAV adapter up", then="the UAV card shows it", next="declare", args="ready on|off | task N|none | resend_course"),
    Entry("CLEAR_MAP", "-", "clear_map", "the boat's perception map forgets every detection (buoys, dock); the camera fills it again",
          ("*",), ("*",), needs="no task running (a task may steer by the map)", then="the map empties; fresh detections reappear within seconds",
          next="-"),
    Entry("TUNE", "-", "tune sway_v 0.3", "v0.1.49: a mission-manager knob into the live config (the next task start reads it); "
          "`tune save` = a profile copy on the boat (logs/launch/<stamp>/usv_tuned.yaml) - never an FCU parameter",
          ("*",), ("MANUAL", "AUTO_HOLD"), needs="no task running (a task keeps the values it started with); a whitelisted key, its range",
          then="the ack says old -> new; the boat window's TUNING card shows it", next="start the task; tune save to keep it",
          args="KEY VALUE | save (ocs_cli tune lists the knobs)"),
    Entry("SPEED", "-", "speed cruise 1.25", "v0.1.94.3: the live speed knobs - cruise (path legs), circle (orbits: speed and yaw rate), "
          "gate (T1 THROUGH), x0.5-2.0; at once, a running task included; the cap lifts with k, never above 1.5 m/s",
          ("*",), ("*",), needs="a mission manager answering", then="the ack shows the three knobs; the console's SPEED row follows",
          next="-", args="cruise | circle | gate | all K | reset"),
    Entry("HOLD", "h", "hold", "pause: AUTO_HOLD / AUTO_TASK -> AUTO_SAFE_HOLD, task kept at its sub-state",
          ("*",), ("AUTO_HOLD", "AUTO_TASK"), then="V5; HOLD on the FCU", next="resume or abort"),
    Entry("RESUME", "r", "resume", "leave SAFE_HOLD: back to the paused task (AUTO_TASK) or to AUTO_HOLD",
          ("*",), ("AUTO_SAFE_HOLD",), needs="the hold cause cleared (no soft fault, no sticky hold)",
          then="V4 or V3", next="-"),
    Entry("SKIP", "-", "skip", "v0.1.68 (free run only): end the current stage's behaviour now - the task's own next event fires (SWAY_SEARCH "
          "ends its pass early); a stage that ends on the map / the UAV / a timer / a report is refused",
          ("*",), ("AUTO_TASK",), needs="the mission manager in free run (--free-run, the lab profile); a motion stage",
          then="the next stage (with --step: parked before it)", next="resume (with --step), or watch the next stage"),
    Entry("SESSION", "-", "session start buoys", "v0.1.69: a dry / wet session - OCS-side (the page server runs it): the observers and feeds "
          "for one stage, its METRICS on the MISSION tab, a result file in logs/ocs/sessions/; nothing moves, the FCU is not touched",
          ("*",), ("*",), needs="bench / lab / pool profile (bench_mode or test_mode); on the course network only `collect`; the page server up",
          then="ocs/state session.current with its metrics; `session stop` writes the result", next="session stop, session result",
          args="start buoys|dockface|windows|uavmap|cannon|collect [--model --tape --path --aim10 --confirm aim --label] | stop | result"),
    Entry("ABORT_TASK", "x", "abort", "drop the task: AUTO_TASK / SAFE_HOLD -> AUTO_HOLD",
          ("*",), ("AUTO_TASK", "AUTO_SAFE_HOLD"), then="V3; TASK_NONE", next="start_task again, or disable_auto"),
    Entry("CONFIRM_SAFE", "c", "confirm_safe", "operator confirmation to leave KILLED or FAULT -> MANUAL",
          ("*",), ("KILLED", "FAULT"), needs="the input cleared first (kill released / FCU stream back / EKF ok)",
          then="V2 MANUAL (DONE ack when entered)", next="enable_auto"),
    Entry("END_RUN", "e", "end_run", "STOP_RUN to every vehicle; run -> R6 ENDED; a running task is aborted",
          ("R3", "R4", "R5"), ("*",), then="R6; nothing from this run is valid any more", next="declare for a new run, or reset"),
    Entry("RESET_RUN", "R", "reset", "both ends back to the start: RESET to every vehicle (task aborted, auto off, run forgotten), OCS -> R2",
          ("R2", "R3", "R4", "R5", "R6"), ("*",), needs="a vehicle in KILLED / FAULT stays there (confirm_safe)",
          then="R2 COURSE_VALID (R1 without a course); vehicle MANUAL", next="declare"),
    Entry("STEP", "s", "step", "advance the running TEST_MANUAL task one stage (TEST_AUTO: skip the rest of its timer)",
          ("*",), ("AUTO_TASK",), needs="start_task m0t0 or m0t1 first", then="S1 -> S2 -> S3 -> DONE (back to AUTO_HOLD)", next="step again, or abort"),
    Entry("GOTO", "g", "goto --x 20 --y 5", "bench-only task: drive the plant to a local point and stop",
          ("*",), ("AUTO_HOLD",), needs="a verified RunStart; guards.allow_goto_task in the vehicle profile",
          then="V4 AUTO_TASK (GOTO) until arrival, then V3", next="-", bench_only=True,
          args="--x --y (local NED metres from frames.origin) or --lat --lon; --speed"),
    Entry("INJECT", "i", "inject <what>", "bench-only: publish what the judge would (run_start, assistance, keep_out, all_clear, moving, confirm)",
          ("*",), ("*",), needs="run_start: R3/R4 (declared); assistance / keep_out / all_clear / moving: vehicle in AUTO (V3-V5), assistance in V3 needs a run; confirm: T4 in WAIT_CONFIRM",
          then="the same cmd/<v>/incident envelope the real chain sends, origin=bench", next="-", bench_only=True,
          args="run_start | assistance --lat --lon --cseq | keep_out | all_clear | moving | confirm --cseq | course"),
]}


# v0.1.54 (Dmitrii: "a better stop button ... hold - disarmed ... clear the current mission state, stop reporting, and then I could
# rerun the sequence"): the macros ocs_cli runs step by step (not intents: no GUARDS row, so not in PLAYBOOK / allowed_now)
STOP_RESET_STEPS = (("cancel_auto", "CANCEL_AUTO", "only while the 3-2-1 after RunStart runs"),
                    ("abort", "ABORT_TASK", "only when a task runs (AUTO_TASK / AUTO_SAFE_HOLD)"),
                    ("disable_auto", "DISABLE_AUTO", "only from AUTO_HOLD (after the abort)"),
                    ("fcu_mode HOLD", "FCU_MODE", "always sent"),
                    ("disarm", "FCU_ARM", "always sent"),
                    ("reset", "RESET_RUN", "R2+ only: RESET to every vehicle, the OCS back to R2"))
MACROS: Dict[str, Entry] = {e.action: e for e in [
    Entry("STOP_RESET", "-", "stop_reset", "STOP & RESET: abort the task -> disable_auto -> fcu_mode HOLD -> disarm -> reset (OCS R2, "
          "vehicles RESET; v0.1.59: reporting stays as it is - heartbeats go on, the boat is still in the course area); every step's result is printed, a refused / not "
          "applicable step does not stop the chain (the FCU commands always go out)", ("*",), ("*",),
          needs="a vehicle in KILLED / FAULT stays there (confirm_safe); the FCU decides HOLD / DISARM",
          then="FCU HOLD + disarmed, vehicle MANUAL (the heartbeat says so), OCS R2 - the run sequence can start again",
          next="declare, arm, enable_auto (the RUN SEQUENCE)"),
]}


@dataclass
class Verdict:
    action: str
    ok: bool
    why: str = ""          # why not (empty when ok)
    hint: str = ""         # what to do first
    sent_as: str = ""      # the vehicle command behind the intent


def _r_index(r: Optional[str]) -> int:
    try:
        return R_ORDER.index(r)
    except (ValueError, TypeError):
        return 0


def check(action: str, ocs_state: Optional[dict], vehicle_id: str = "USV1", task: Optional[str] = None,
          what: Optional[str] = None, status: Optional[dict] = None, ui_active: Optional[bool] = None) -> Verdict:
    """Predict the verdict for an intent from the last ocs/state (and status/<v> if given).

    ocs_state is the body of ocs/state; status the body of status/<v> (for armed / kill / ekf hints); ui_active is whether
    usv_ui on the boat still commands the FCU (its tui/active beat: the mission manager refuses ENABLE_AUTO until `handover`),
    None when the console cannot see the boat broker."""
    a = action.upper()
    if a in NEVER:
        return Verdict(a, False, f"{a} is never a software command (ADR-9)", "use the RC transmitter / kill remote")
    e = PLAYBOOK.get(a)
    if e is None:
        return Verdict(a, False, f"unknown intent {a}", "ocs_cli help")
    if not ocs_state:
        return Verdict(a, False, "no ocs/state seen yet", "is ocs_app running on this broker?")
    r = ocs_state.get("r")
    bench = bool(ocs_state.get("bench_mode"))
    veh = dict((ocs_state.get("vehicles") or {}).get(vehicle_id) or {})
    if status and status.get("state"):          # status/<v> is 2 Hz and fresher than the 1 Hz copy in ocs/state
        veh.update(state=status.get("state"), task_state=status.get("task_state"), current_task=status.get("current_task"),
                   task_name=status.get("task_name"))
    v = veh.get("state")
    run_id = ocs_state.get("run_id")
    r_norm, r_bench, vcmd, kind, bench_only = GUARDS[a]
    sent = vcmd or a
    if bench_only and not bench:
        return Verdict(a, False, f"{a} is bench only and bench mode is off", "ocs_bench.yaml: ocs.bench_mode true (never on the course)", sent)
    r_min = r_bench if bench else r_norm
    test = bool(ocs_state.get("test_mode"))               # v0.1.37 TEST MODE
    rr0 = resolve(task) if (a == "START_TASK" and task) else None
    run_free = test and (a in ("ENABLE_AUTO", "GOTO") or (a == "START_TASK" and rr0 is not None and rr0[1] in RUN_FREE_TASKS))
    if run_free:
        r_min = 0
    if _r_index(r) < r_min:
        return Verdict(a, False, f"{a} needs R{r_min}+ (now {r})", _r_hint(r_min, r), sent)
    # OCS lifecycle transitions that the authority cannot see from the GUARDS table
    if e.ocs_r != ("*",) and r not in e.ocs_r:
        if a == "DECLARE":
            hint = "end_run first" if r in ("R3", "R5") else "wait for the hub connection and the course"
            if r == "R3":
                hint = "already declared: enable_auto, or end_run and declare again"
            return Verdict(a, False, f"DECLARE needs R2, R4 (stalled) or R6, now {r}", hint, sent)
        if a == "END_RUN":
            return Verdict(a, False, f"END_RUN needs R3-R5 (now {r}): nothing to end", "declare first", sent)
        if a == "START_TASK" and not bench and not run_free:
            return Verdict(a, False, f"START_TASK needs R5 RUNNING (now {r})", "declare first, enable_auto, wait for RunStart", sent)
    if a == "RESET_RUN":
        if _r_index(r) < 2:
            return Verdict(a, False, f"nothing to reset in {r}", "wait for the hub link and a course", sent)
        return Verdict(a, True, sent_as=sent)
    if a == "STEP":
        if v != "AUTO_TASK" or not str(veh.get("task_name") or "").startswith("TEST_"):
            return Verdict(a, False, f"STEP needs a running m0t0 / m0t1 (vehicle {v or '?'}, task {veh.get('task_name') or '-'})",
                           "start_task m0t0 first", sent)
        return Verdict(a, True, sent_as=sent)
    if a == "HOME":
        if v not in AUTO:
            return Verdict(a, False, f"HOME acts in AUTO only (vehicle {v or '?'})",
                           "in MANUAL the boat is yours: drive it home with the transmitter" if v == "MANUAL" else "enable_auto first", sent)
        return Verdict(a, True, sent_as=sent)
    if a == "CLEAR_MAP" and v in ("AUTO_TASK",):
        return Verdict(a, False, "a task is running and may steer by the map", "abort (or wait for the end), then clear_map", sent)
    if a in ("SAVE_POINT", "DELETE_POINT", "CLEAR_MAP"):
        if not v or v in ("OFFLINE", "BOOT"):
            return Verdict(a, False, f"no mission manager answering (vehicle {v or '?'})", "start the boat stack (run_usv.sh)", sent)
        return Verdict(a, True, sent_as=sent)
    if a == "SELECT_TARGET":
        if v not in ("AUTO_TASK", "AUTO_SAFE_HOLD") or veh.get("task_name") != "BOX_INSPECT" or veh.get("task_state") not in (
                "PREFLIGHT", "GO_SCAN", "OBSERVE", "STRAFE", "ASK_OCS", "HOLD"):
            return Verdict(a, False,
                           f"SELECT_TARGET needs BOX_INSPECT before ALIGN (vehicle {v or '?'}, task {veh.get('task_name') or '-'}, state {veh.get('task_state') or '-'})",
                           "start_task m0t4 [--target-colour red] (the vehicle picks GREEN by itself)", sent)
        return Verdict(a, True, sent_as=sent)
    if a == "INJECT":
        w = (what or "").lower()
        if w == "run_start" and r not in ("R3", "R4"):
            return Verdict(a, False, f"run_start injection needs R3/R4 (declared, no run yet), now {r}", "declare first", sent)
        if w in ("assistance_request", "assistance", "keep_out_zone", "keep_out", "all_clear", "moving_object", "moving") and v not in AUTO:
            return Verdict(a, False, f"the vehicle must be in AUTO for an incident (now {v})", "enable_auto first", sent)
        if w in ("assistance_request", "assistance") and v == "AUTO_HOLD" and run_id is None:
            return Verdict(a, False, "an AssistanceRequest in AUTO_HOLD auto-starts T4, which needs a run", "wait for RunStart or inject run_start", sent)
        if w in ("readiness_confirm", "confirm") and veh.get("task_state") != "WAIT_CONFIRM":
            return Verdict(a, False, f"nothing waits for a ReadinessConfirm (task_state {veh.get('task_state')})", "inject assistance first and let T4 reach WAIT_CONFIRM", sent)
        return Verdict(a, True, sent_as=sent)
    if a == "FENCE_APPLY" and v == "AUTO_TASK":           # v0.1.53: the vehicle and the OCS refuse a new fence mid-task
        return Verdict(a, False, "FENCE_APPLY is refused in AUTO_TASK (a new fence could stop the running task)", "hold / abort first, or apply before the start", sent)
    if a in ("FENCE_APPLY", "FENCE_ENABLE") and (not v or v in ("OFFLINE",)):
        return Verdict(a, False, f"no mission manager answering (vehicle {v or '?'})", "start the boat stack (run_usv.sh)", sent)
    if a == "TUNE" and what == "save":                    # v0.1.49: saving changes nothing on the boat: any state
        return Verdict(a, True, sent_as=sent)
    # vehicle side
    if e.vehicle_v != ("*",) and v not in e.vehicle_v:
        return Verdict(a, False, f"{sent} is accepted in {'/'.join(e.vehicle_v)} only, vehicle is in {v or '?'}", _v_hint(a, v), sent)
    if a in ("START_TASK", "GOTO"):
        rr = resolve(task or ("GOTO" if a == "GOTO" else "T4"))
        if rr is None:
            return Verdict(a, False, f"unknown mission {task}", "start_task m4t0 | m1t1 | m3t1 | m0t0 .. m0t3 (or goto); `ocs_cli help` lists the codes", sent)
        t = rr[1]
        if mission_for(rr[0]).status == "planned":
            return Verdict(a, False, f"{rr[0]} ({t}) is not built yet (vehicle registry: T4, T1, T3, GOTO, TEST_*)", "start_task m4t0 / m1t1 / m3t1, or goto on the bench", sent)
        if rr[3] and status is not None and status.get("fcu_link") != "fake":
            return Verdict(a, False, f"{rr[0]}sim needs the fake FCU on the boat (usv_ui holds: {status.get('fcu_link') or 'unknown'})",
                           f"start usv_ui with --fcu fake (fcu fake in usv_ui), or start_task {rr[0]} without sim", sent)
        if run_id is None and not (test and t in RUN_FREE_TASKS):
            return Verdict(a, False, "the vehicle needs a verified RunStart (run_id unknown)",
                           "a judged mission: declare, enable_auto, wait for R5" + (" (test mode runs m0t*, goto, home, m0t4 without a run)" if test else ""), sent)
        mis = (rr[0] + ("sim" if rr[3] else "")) if (task or a == "GOTO") else _preselected(ocs_state)
        block = declaration_block(ocs_state, mis, "goto" if a == "GOTO" else "start_task")
        if block:                                                        # v0.1.41 (WP9c) the OCS would refuse it
            return Verdict(a, False, block[0]["text"], "declaration.policy refuse: the OCS refuses this until it is fixed", sent)
    if a == "DECLARE":
        block = declaration_block(ocs_state, None, "declare")
        if block:
            return Verdict(a, False, block[0]["text"], "declaration.policy refuse: the OCS refuses declare until it is fixed", sent)
    if a == "ENABLE_AUTO" and ui_active:
        return Verdict(a, False, "usv_ui ACTIVE on the boat: it commands the FCU", "type `handover` in usv_ui, then enable_auto", sent)
    if a == "ENABLE_AUTO" and status:
        if status.get("kill") or status.get("estop"):
            return Verdict(a, False, "kill / E-stop asserted", "release the kill, then confirm_safe", sent)
        if not status.get("armed", True):
            return Verdict(a, False, "FCU not armed", "arm: the bar's ARM (asks first) or the transmitter", sent)
        if status.get("mode") == "MANUAL":                 # v0.1.29.3 (pool report F6)
            return Verdict(a, False, "the FCU is in MANUAL (RC6 override or the transmitter's mode switch)",
                           "put it in HOLD with the transmitter, then enable_auto", sent)
        if status.get("fault"):
            return Verdict(a, False, f"fault {status.get('fault')}", "clear the cause, then confirm_safe", sent)
        if (ocs_state.get("declaration_seq") or 0) == 0 and not test and ocs_state.get("auto_before_declare") is False:   # v0.1.90.8: only when a profile asks
            return Verdict(a, False, "no PREPARE yet (declaration_seq 0)", "declare first", sent)
    if a == "CONFIRM_SAFE" and status:
        if status.get("kill") or status.get("estop"):
            return Verdict(a, False, "kill / E-stop still asserted: confirmation would be rejected", "release the kill first", sent)
    return Verdict(a, True, sent_as=sent)


def _preselected(ocs_state: Optional[dict]) -> str:
    mi = (ocs_state or {}).get("mission") or {}
    return mi.get("code") or code_of(mi.get("task")) or "m4t0"                # start_task without a task: the preselected, else T4


def declaration_rows(ocs_state: Optional[dict], mission: Optional[str] = None, step: str = "state") -> List[dict]:
    """v0.1.41 (WP9c): the RunDeclaration rows (ocs/declaration_checks) for a mission (default: the preselected one), from ocs/state.
    `declare` and `state` look at the preselected mission; the other steps only at the mission's own rows."""
    rows = DC.from_state(ocs_state, mission if step not in ("declare",) else None, step)
    return rows if step in ("declare", "state") else [x for x in rows if x["scope"] == "mission"]


def declaration_block(ocs_state: Optional[dict], mission: Optional[str] = None, step: str = "state") -> List[dict]:
    """The rows that make the OCS refuse (policy refuse, severity hard)."""
    pol = ((ocs_state or {}).get("declaration_checks") or {}).get("policy") or DC.DEFAULT_POLICY
    return DC.blocking(declaration_rows(ocs_state, mission, step), pol)


def _r_hint(r_min: int, r: Optional[str]) -> str:
    if r_min >= 5:
        return "declare, enable_auto, wait for RunStart (R5)"
    if r_min >= 3:
        return "declare first"
    if r_min >= 2:
        return "wait for the hub connection and a valid course"
    return ""


def _v_hint(action: str, v: Optional[str]) -> str:
    if action == "ENABLE_AUTO" and v in AUTO:
        return "already in auto"
    if action == "CONFIRM_SAFE" and v not in ("KILLED", "FAULT"):
        return "nothing to confirm (only after a kill or a fault)"
    if v in ("KILLED", "FAULT"):
        return "clear the input, then confirm_safe, then enable_auto"
    if v == "MANUAL":
        return "enable_auto first"
    if v == "AUTO_TASK" and action in ("DISABLE_AUTO", "START_TASK", "GOTO"):
        return "abort first" if action == "DISABLE_AUTO" else "abort first, or hold to pause"
    if v == "AUTO_SAFE_HOLD":
        return "resume or abort first"
    if v == "AUTO_HOLD" and action == "RESUME":
        return "nothing is held; start_task or hold"
    if v == "AUTO_HOLD" and action == "ABORT_TASK":
        return "no task running"
    if v == "BOOT":
        return "wait for the FCU heartbeat"
    return ""


def allowed_now(ocs_state: Optional[dict], vehicle_id: str = "USV1", status: Optional[dict] = None,
                ui_active: Optional[bool] = None) -> List[Verdict]:
    return [check(a, ocs_state, vehicle_id, status=status, ui_active=ui_active) for a in PLAYBOOK]


def suggest(ocs_state: Optional[dict], vehicle_id: str = "USV1", ui_active: Optional[bool] = None) -> str:
    """The one obvious next step for a nominal run, from the current R and V (and who commands the FCU, when known).
    v0.1.41 (WP9c): a RunDeclaration row comes first when the OCS would refuse (policy refuse), or rides along (policy warn)."""
    s = _suggest(ocs_state, vehicle_id, ui_active)
    o = ocs_state or {}
    veh = (o.get("vehicles") or {}).get(vehicle_id) or {}
    if not o or o.get("r") in (None, "R0", "R1") or veh.get("state") in ("KILLED", "FAULT", "AUTO_TASK", "AUTO_SAFE_HOLD"):
        return s
    rows = declaration_rows(o)
    if not rows:
        return s
    block = declaration_block(o)
    if block:
        return "declaration check (refused until fixed): " + block[0]["text"]
    return s + "   · check: " + rows[0]["text"]


def _suggest(ocs_state: Optional[dict], vehicle_id: str = "USV1", ui_active: Optional[bool] = None) -> str:
    if not ocs_state:
        return "start ocs_app"
    r = ocs_state.get("r")
    veh = (ocs_state.get("vehicles") or {}).get(vehicle_id) or {}
    v, ts = veh.get("state"), veh.get("task_state")
    if v in ("KILLED", "FAULT"):
        return "clear the input, then confirm_safe"
    if r in ("R0", "R1"):
        return "wait: hub link / course"
    if r == "R6":
        return "declare again (new run), or reset (both ends back to R2 / MANUAL)"
    if r == "R2":
        if ocs_state.get("prepare_pending"):
            return "waiting for PREPARE acks from " + ", ".join(ocs_state["prepare_pending"]) + " (3 s, then an alert)"
        return "declare" + (" (usv_ui is ACTIVE: `handover` there before enable_auto)" if ui_active else "")
    if r == "R4" and ocs_state.get("stalled"):
        return "no RunStart yet: ask the course manager, or redeclare (a refusal is shown on RoboCommand's display)"
    if v == "MANUAL" and ui_active:
        return "usv_ui holds the FCU: type `handover` in usv_ui (on the boat), then enable_auto"
    if v == "MANUAL":
        return "enable_auto"
    if r in ("R3", "R4"):
        return "declared, waiting for the course manager's RunStart (course: ~1 min; fake judge: 2 s; bench: inject run_start)"
    if v == "AUTO_HOLD":
        return "start_task m4t0 | m1t1 | m3t1 | m0t0 .. m0t3, or goto; inject assistance auto-starts m4t0 (T4)"
    if v == "AUTO_TASK" and str(veh.get("task_name") or "").startswith("TEST_"):
        return f"{code_of(veh.get('task_name'))} in {ts}: step (m0t0) or watch the timer (m0t1); hold / abort work too"
    if v == "AUTO_TASK" and ts == "LISTEN":
        return "inject assistance"
    if v == "AUTO_TASK" and ts == "WAIT_CONFIRM":
        return "inject confirm if the judge did not"
    if v == "AUTO_TASK":
        return "watch; hold or abort"
    if v == "AUTO_SAFE_HOLD" and str(veh.get("hold_reason") or "").startswith("step: next "):
        return f"resume -> {str(veh['hold_reason'])[len('step: next '):]}   (stage stepping; abort ends the task)"
    if v == "AUTO_SAFE_HOLD":
        return "resume or abort"
    return "-"


# one line per state, for the consoles (usv_ui MISSION MANAGER box, op-tui VEHICLE box)
V_MEANING = {
    "BOOT": "V0 mission manager up, no FCU heartbeat yet",
    "KILLED": "V1 kill / E-stop asserted: motors dead; release it, then confirm_safe -> MANUAL",
    "MANUAL": "V2 the mission manager commands nothing: usv_ui, the transmitter or nobody drives; enable_auto needs handover + arm + EKF + PREPARE",
    "AUTO_HOLD": "V3 the mission manager holds the FCU in HOLD, no task; start_task / goto from the OCS",
    "AUTO_TASK": "V4 a task behaviour drives (see the task stage); hold pauses, abort drops it, stop in usv_ui takes the FCU back",
    "AUTO_SAFE_HOLD": "V5 paused in HOLD by hold / a soft fault / a keep-out; resume continues the task",
    "FAULT": "V6 a hard fault (FCU_LOST, EKF_BAD, unexpected disarm): clear it, then confirm_safe -> MANUAL",
    "OFFLINE": "the mission manager's last will: it is down",
}
R_MEANING = {
    "R0": "DISCONNECTED: no RoboCommand / judge link",
    "R1": "CONNECTED: judge link up, no valid course yet",
    "R2": "COURSE_VALID: ready to declare (PREPARE goes to the vehicles, R3 when they ack)",
    "R3": "DECLARED: the judge has the declaration, heartbeats run; enable_auto next",
    "R4": "READY: every declared vehicle is in AUTO, waiting for RunStart",
    "R5": "RUNNING: RunStart verified, tasks may start",
    "R6": "ENDED: STOP_RUN sent; declare again or reset",
}
# ---- the mission catalogue: one source for the MISSION box, the MISSION tab and the stage meanings ------------------
# Mission codes (v0.1.19, the operator's words): m<RobotX task><tier>  -  m4t0 = Task 4 core, m1t2 = Task 1 advanced,
# m0tN = our own test missions, `goto` = the bench waypoint.  A `sim` suffix (m1t1sim) is the SAME mission with one more
# guard: the vehicle rejects it unless usv_ui holds the fake FCU (presence fcu_link: fake).  The wire never carries a code:
# the OCS sends `task: T4` plus args {tier: core|advanced, sim: true|false}; the vehicle registry knows T4, GOTO, TEST_*.
@dataclass
class Stage:
    name: str
    meaning: str            # one line, as the consoles show it next to the stage
    behaviour: str = "-"    # the layer-2 behaviour that drives it
    leaves_on: str = ""     # what moves it to the next stage
    judge: str = "-"        # the RoboCommand message in or out at this stage
    detour: str = ""        # v0.1.43: a detour of that stage (LOOK of TRANSIT, APPROACH_LEAD of SEARCH_ENTRY) - shown indented


@dataclass
class Mission:
    code: str                         # m4t0 ... the operator's name (ocs_cli mission m4t0, the MISSION tab)
    task: str                         # the wire task: T4, T1, T3, GOTO, TEST_MANUAL, TEST_AUTO, TEST_PATH, TEST_SWAY
    title: str
    tier: str                         # as shown: Core / Advanced / bench / test
    status: str                       # built | planned | bench | test
    what: Sequence[str]               # two or three lines
    starts: str
    ends: str
    judge_in: str
    judge_out: str
    config: str
    stages: Sequence[Stage]
    note: str = ""
    wire_tier: Optional[str] = None   # what goes into START_TASK args.tier: core | advanced | None (tests, bench)
    hidden: bool = False              # v0.1.86 (WP42 §2.6 fallback, §6 item 3): out of every picker (pages, console, launchers); typed still works


_T1_STAGES = [Stage("WAIT_START", "holding at the start line", "StationKeep", "RunStart"),
              Stage("SEARCH_ENTRY", "looking for the flashing blue entry buoy", "ScanYaw", "entry buoy seen"),
              Stage("APPROACH_LEAD", "no entry in view but a far sighting (a ray): 12 m toward it, then scan again (3 tries)", "FollowPath",
                    "arrived -> SEARCH_ENTRY", detour="SEARCH_ENTRY"),
              Stage("CIRCLE_ENTRY_CW", "one clockwise circle round the entry buoy", "Orbit", "circle closed"),
              Stage("TRANSIT", "gate by gate: pre -> mid -> post, GREEN to port / RED to starboard (loops)", "FollowPath", "no gate left -> EXIT"),
              Stage("LOOK", "a pending light nearer than the next gate: face it, 8 s at most", "ScanYaw (face)", "decided -> TRANSIT", detour="TRANSIT"),
              Stage("SCAN", "no usable gate ahead: turning on the spot to find one (nothing -> safe hold)", "ScanYaw", "gate -> TRANSIT | exit -> EXIT"),
              Stage("EXIT", "to the solid blue exit buoy's circle", "FollowPath", "at the circle"),
              Stage("CIRCLE_EXIT_CCW", "one counter-clockwise circle round the exit buoy", "Orbit", "circle closed -> DONE"),
              Stage("DONE", "course passed", "-", "TASK_DONE -> AUTO_HOLD")]

MISSIONS: Dict[str, Mission] = {m.code: m for m in [
    Mission("m4t0", "T4", "Dynamic Incident Response", "Core", "built",
            ["station-keep and listen; on the judge's AssistanceRequest: break off, ack, drive to the request",
             "position, report readiness, hold until ReadinessConfirm. Core needs no perception (first task end-to-end)."],
            starts="start_task m4t0 (vehicle AUTO_HOLD, run R5); it then waits in LISTEN for the request",
            ends="DONE after ReadinessConfirm -> AUTO_HOLD; abort -> AUTO_HOLD; hold pauses (the detour map survives)",
            judge_in="AssistanceRequest, ReadinessConfirm (Adv: KeepOutZone, AllClear; Disr: MovingObjectAlert)",
            judge_out="IncidentAck (<= 1 s), ReadinessReport",
            config="tasks.t4.speed_mps, arrival_m (usv_pool.yaml)",
            stages=[Stage("LISTEN", "station-keeping, waiting for an AssistanceRequest", "StationKeep", "AssistanceRequest -> ACK"),
                    Stage("ACK", "IncidentAck going out", "-", "ack sent -> TRANSIT", "Ack out"),
                    Stage("TRANSIT", "driving to the request position", "Goto", "arrived -> READY"),
                    Stage("READY", "at the position, ReadinessReport going out", "StationKeep", "sent -> WAIT_CONFIRM", "Report out"),
                    Stage("WAIT_CONFIRM", "holding, waiting for the judge's ReadinessConfirm", "StationKeep", "ReadinessConfirm -> DONE", "Confirm in"),
                    Stage("DONE", "incident served", "-", "TASK_DONE -> AUTO_HOLD")],
            note="Advanced: KeepOutZone -> ACK -> AVOID -> AllClear.  Disruptive: MovingObjectAlert -> STANDOFF >= 10 m (not built)",
            wire_tier="core"),
    Mission("m1t1", "T1", "Safe Passage (gates)", "Core", "built",
            ["enter through the FLASHING BLUE buoy (one clockwise circle), transit the red/green gates keeping GREEN to port",
             "and RED to starboard, exit at the SOLID BLUE buoy (counter-clockwise circle). The boat's own camera finds the buoys."],
            starts="start_task m1t1 [--planner simple|s6] (vehicle AUTO_HOLD, run R5); m1t1sim on the simulated pool first",
            ends="DONE after CIRCLE_EXIT_CCW; abort -> AUTO_HOLD; a scan that finds nothing -> AUTO_SAFE_HOLD (resume scans again)",
            judge_in="-", judge_out="SafePassageReport (entry, exit, each buoy with colour and state), updated as the map changes",
            config="t1.planner simple|s6, circle_r_m 3, circle_v 0.5, transit_v 0.6, gate_d_m 2, min_obs 5 (usv profile)",
            stages=_T1_STAGES,
            note="v0.1.21 (ROADMAP WP2): two planners - simple (gate midpoints) and s6 (pathing_sim's Dubins leg); a gate only crossable with RED to port is refused",
            wire_tier="core"),
    Mission("m1t2", "T1", "Safe Passage (gates) + UAV buoy map", "Advanced", "built",
            ["T1 Advanced (v0.1.66, WP16): the boat waits at the start (WAIT_UAV_MAP) while the UAV surveys and reports the buoys - its",
             "colours come from the UAV (the boat is colour-blind for buoys coloured only from above), its positions from the camera + GPS."],
            starts="declare uav on, uav ready on, declare; start_task m1t2 (needs a fresh UAV status; the declaration's T1 tier = ADVANCED)",
            ends="DONE after CIRCLE_EXIT_CCW; WAIT_UAV_MAP: scan.complete / the drone landed / `uav scan done`; 420 s without an entry -> AUTO_SAFE_HOLD",
            judge_in="-", judge_out="SafePassageReport with the fused colours; the UAV's own report relayed (D13)",
            config="t1.colour_source uav (set by the mission), t1.uav_wait_s 420; uav.assoc_m 1.0 (the link radius)",
            stages=[_T1_STAGES[0], Stage("WAIT_UAV_MAP", "holding outside the field while the UAV surveys (its scan.complete, the drone landed, or uav scan done)",
                                         "StationKeep", "the UAV map is in")] + _T1_STAGES[1:],
            note="v0.1.66: the UAV's buoys (uav/perception/buoys, docs/UAV_BUOY_CONTRACT.md) linked within 1 m to the camera's; UAV-only ones are leads",
            wire_tier="advanced"),
    Mission("scout", "SCOUT", "SCOUT — buoy to buoy, colour-blind, logs the field (declared as Task 1)", "T1 core / advanced (UAV)", "built",
            ["v0.1.79 (WP35): forward until the first buoy locks, then the nearest unvisited buoy by path length (1.6 m off every mapped",
             "object), a 3 m stand-off, 4 s to log it; a slow 360 scan only when the map runs dry, then the unswept lawnmower points; home at the end.",
             "Colour-blind: every buoy logged with its position (logs/scout_<run>.jsonl); the drone's buoys are targets too."],
            starts="start_task scout (AUTO_HOLD, run R5; declared SCOUT -> T1 core, or advanced with `declare uav on`); scoutsim on the fake FCU",
            ends="RETURN to the start pose -> DONE (AUTO_HOLD) when no frontier is left or budget_s (600 s) is up; abort -> AUTO_HOLD",
            judge_in="-", judge_out="SafePassageReport whenever the mapped set changes (buoys only, colour red only when voted)",
            config="scout: standoff_m 3, transit_v 0.6, log_s 4, budget_s 600, lane_m 14, clear_m 1.6, min_obs 3, field_m 30 (usv profile)",
            stages=[Stage("WAIT_START", "holding at the start", "StationKeep", "RunStart"),
                    Stage("WAIT_UAV_MAP", "the drone declared: holding while it surveys (scan.complete, landed, or uav scan done)", "StationKeep", "the UAV map is in"),
                    Stage("ADVANCE", "forward along the heading inside the field (toward its centre near an edge) until the first lock", "FollowPath",
                          "first lock -> PICK | far point -> SCAN"),
                    Stage("PICK", "the nearest unvisited buoy by path length round the clearance rings", "-", "target -> APPROACH | none -> SCAN"),
                    Stage("APPROACH", "to the buoy's stand-off point (3 m, outside every ring, inside the field)", "FollowPath", "arrived -> LOG"),
                    Stage("LOG", "holding 4 s while the map settles; the buoy is logged", "StationKeep", "logged -> PICK"),
                    Stage("SCAN", "the map is exhausted: one slow turn on the spot", "ScanYaw", "a new buoy -> PICK | nothing -> FRONTIER"),
                    Stage("FRONTIER", "to the next unswept lawnmower point", "FollowPath", "arrived -> SCAN | a buoy -> PICK"),
                    Stage("RETURN", "home to the start pose", "FollowPath", "at the start -> DONE"),
                    Stage("DONE", "field logged (summary line)", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.79: the ZED guard (WP36) brakes / backs off / re-plans in every moving stage; t1_safe_passage.py untouched",
            wire_tier=None),
    Mission("m3t1", "T3", "Coordinated Logistics (dock + fire)", "Core", "built",
            ["find the dock line, sway the whole standoff line counting each bay's indicator, commit to the GREEN bay, bow in to the",
             "stop distance (a RED indicator on the way in: back out, search again), report the docking; the RED window lights: water on",
             "that window until it turns GREEN (30 s at most), report the firefighting with its window_id; back out astern."],
            starts="start_task m3t1 [--bay N] (vehicle AUTO_HOLD, run R5, the dock within camera range); m3t1sim on the simulated pool first",
            ends="DONE after UNDOCK -> AUTO_HOLD; abort -> AUTO_HOLD; no dock / GREEN bay found -> AUTO_SAFE_HOLD",
            judge_in="-", judge_out="DockingReport, FirefightingReport (Adv: ResourceRequest - not built)",
            config="t3.standoff_m / standoff2_m (v0.1.56: finger_m 3.5 + hull/2 + clear 0.60 + 1.0 / 0.3 = 5.7 / 5.0 unless set), stop_m 1.5, fire_s 30, sway_v 0.25, enter_v 0.15, reengage_max 2, window_wait_s 10, undock astern (usv profile); gains.sway_*",
            stages=[Stage("WAIT_START", "holding", "StationKeep", "started"),
                    Stage("FIND_DOCK", "turning on the spot until the dock line (three faces) is in the map", "ScanYaw", "dock line found"),
                    Stage("TO_LINE", "to the near end of the standoff line (standoff_m off the dock)", "FollowPath", "on the line"),
                    Stage("SWAY_SEARCH", "sway the whole line, bow to the dock, counting each bay's indicator (RED / GREEN votes)", "SwayTransit",
                          "one bay GREEN -> SWAY_TO_ABEAM (none / a tie: a second pass at 3.5 m; still none: hold)"),
                    Stage("SWAY_TO_ABEAM", "sway along the standoff line, bow to the dock, to abeam of the chosen bay", "SwayTransit", "abeam"),
                    Stage("TURN_IN", "bow to the bay", "DockLos align", "aligned"),
                    Stage("ENTER", "guarded bow-in along the bay centreline to stop_m off the face", "DockLos enter", "stop distance -> DOCKED"),
                    Stage("BACK_OUT", "the bay's indicator RED on the way in (or off its centreline): astern to the standoff line", "DockLos astern",
                          "-> SWAY_SEARCH without that bay (2 at most)", detour="ENTER"),
                    Stage("DOCKED", "holding in the bay", "StationKeep", "-> REPORT_DOCK"),
                    Stage("REPORT_DOCK", "DockingReport going out", "-", "sent -> AIM_FIRE", "Report out"),
                    Stage("AIM_FIRE", "cannon at the window seen lit RED (local/usv/cannon/aim with window_id) until it is GREEN, fire_s at most", "cannon", "GREEN -> REPORT_FIRE"),
                    Stage("REPORT_FIRE", "FirefightingReport going out", "-", "sent -> UNDOCK", "Report out"),
                    Stage("UNDOCK", "astern out of the bay to the standoff line", "DockLos astern", "out -> DONE"),
                    Stage("DONE", "task finished", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.22 (ROADMAP WP3); the dock is read through usv/dockpath (the students' box core, docs/merge/, can feed it); cv/cannon aims on the AGX",
            wire_tier="core"),
    Mission("goto", "GOTO", "bench waypoint", "bench", "bench",
            ["drive to a local N/E point at a speed and stop there. Bench only (guards.allow_goto_task); refused on the course network."],
            starts="goto --x N --y E [--speed V]  (the OCS sends START_TASK GOTO with the point)",
            ends="arrived -> DONE -> AUTO_HOLD; abort -> AUTO_HOLD",
            judge_in="-", judge_out="-", config="guards.allow_goto_task (bench / pool profiles)",
            stages=[Stage("GO", "driving to the point (bench task)", "Goto", "arrived -> DONE"),
                    Stage("DONE", "arrived", "-", "TASK_DONE -> AUTO_HOLD")]),
    Mission("m0t0", "TEST_MANUAL", "state-machine test, by hand", "test", "test",
            ["walks the task machine S1 -> S2 -> S3 -> DONE with no behaviours: nothing moves, no judge traffic.",
             "Each stage advances on `step` (ocs_cli step, or step in usv_ui) - from either side."],
            starts="start_task m0t0 (vehicle AUTO_HOLD, run R5)",
            ends="step in S3 -> DONE -> AUTO_HOLD; abort -> AUTO_HOLD",
            judge_in="-", judge_out="- (the heartbeat keeps TASK_NONE)", config="-",
            stages=[Stage("S1", "test stage 1 of 3: `step` advances it", "none (hold)", "step -> S2"),
                    Stage("S2", "test stage 2 of 3: `step` advances it", "none (hold)", "step -> S3"),
                    Stage("S3", "test stage 3 of 3: `step` finishes it", "none (hold)", "step -> DONE"),
                    Stage("DONE", "task finished", "-", "TASK_DONE -> AUTO_HOLD")]),
    Mission("m0t1", "TEST_AUTO", "state-machine test, timers", "test", "test",
            ["the same S1 -> S2 -> S3 -> DONE on timers (step_s each, 5 s by default), nothing moves, no judge traffic.",
             "Proves START_TASK, the stage events, TASK_DONE and both consoles' stage display without touching anything."],
            starts="start_task m0t1 (vehicle AUTO_HOLD, run R5), or `mission m0t1 --auto` -> starts on RunStart",
            ends="DONE after the third timer -> AUTO_HOLD; abort -> AUTO_HOLD; hold pauses the timer; step skips it",
            judge_in="-", judge_out="- (the heartbeat keeps TASK_NONE)", config="extra.test_auto.step_s (usv_pool.yaml), 5 s",
            stages=[Stage("S1", "test stage 1 of 3, timer running", "none (hold)", "timer -> S2 | step"),
                    Stage("S2", "test stage 2 of 3, timer running", "none (hold)", "timer -> S3 | step"),
                    Stage("S3", "test stage 3 of 3, timer running, then DONE", "none (hold)", "timer -> DONE | step"),
                    Stage("DONE", "task finished", "-", "TASK_DONE -> AUTO_HOLD")]),
    Mission("m0t2", "TEST_PATH", "path test (waypoints -> path -> follow)", "test", "test",
            ["the pool-trial path leg through the final architecture: a path of saved points (or one generated from the CV",
             "map, `pathgen`) is followed by the mission manager's FollowPath behaviour; what usv_ui's `pathto` did by hand."],
            starts="start_task m0t2 [--names EN,G1,..] (vehicle AUTO_HOLD, run R5); no names = the gates in the mission manager's map (the PoR course)",
            ends="last point reached -> DONE -> AUTO_HOLD; abort -> AUTO_HOLD",
            judge_in="-", judge_out="-", config="tasks.test_path (points file or `pathgen` output)",
            stages=[Stage("PLAN", "take the points (file / pathgen) and build the path", "-", "path ready -> FOLLOW"),
                    Stage("FOLLOW", "point by point along the path", "FollowPath", "last point -> DONE"),
                    Stage("DONE", "path done", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.21 (ROADMAP WP2); m0t2sim on world_por = the Proof of Readiness run in simulation"),
    Mission("m0t3", "TEST_SWAY", "sway test (dock line -> sway along it)", "test", "test",
            ["the pool-trial sway leg through the final architecture: sway along a line the CV dock mapper gives (or two saved",
             "points) at a set range; what usv_ui's `swayto` did between waypoints. The first half of docking (m3t1)."],
            starts="start_task m0t3 [--names A,B] (vehicle AUTO_HOLD, run R5); no names = the dock line from the map at t3.standoff_m",
            ends="end of the line -> DONE -> AUTO_HOLD; abort -> AUTO_HOLD",
            judge_in="-", judge_out="-", config="t3.standoff_m, t3.sway_v; gains.sway_f_pos / sway_f_neg / sway_kpsi / sway_kc",
            stages=[Stage("LINE", "take the line (two points / the dock line at the standoff)", "-", "line ready -> TO_START"),
                    Stage("TO_START", "GUIDED to A", "FollowPath", "at A -> SWAY"),
                    Stage("SWAY", "sway along the line, bow held orthogonal (mode 20)", "SwayTransit", "end of line -> DONE"),
                    Stage("DONE", "sway done", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.22 (ROADMAP WP3): the calibrated sway factors (F+ 1.05 / F- 0.877) are in the mission manager's gains now"),
    Mission("m0t6", "TEST_ORBIT", "orbit test (circle a point)", "test", "test",
            ["orbit a point on its own (v0.1.30, ROADMAP WP8c): the Orbit behaviour of m1t1's entry / exit circles, round a",
             "saved point or a map click - radius, cw / ccw, turns, speed. GUIDED carrots on the circle, no mode 20."],
            starts="start_task m0t6 --centre A [--radius 3 --dir cw --turns 1 --speed 0.5] (or --lat / --lon: the map; AUTO_HOLD, run R5)",
            ends="turns x 360 deg wound -> DONE -> AUTO_HOLD; abort -> AUTO_HOLD; orbit timeout -> aborted",
            judge_in="-", judge_out="-", config="orbit: r_m 3, direction cw, turns 1, speed_mps 0.5 (START_TASK args override)",
            stages=[Stage("APPROACH", "GUIDED to the point of the circle nearest the boat", "Goto", "on the circle -> ORBIT"),
                    Stage("ORBIT", "round the centre, carrots on the circle", "Orbit", "turns done -> DONE"),
                    Stage("DONE", "orbit done", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.30; --step / --from APPROACH | ORBIT (ORBIT needs the boat within band_m of the circle)"),
    Mission("m0t8", "TEST_DETECT", "ground test: buoy detection ranges (the boat stands still)", "test", "test",
            ["v0.1.79 (Dmitrii 2026-11-11: check the detection on the ground first): the boat holds still (HOLD, props off on land) while every",
             "buoy the camera maps is logged - range, bearing, sightings; walk a buoy away: the LOST line gives the range it was last seen at."],
            starts="start_task m0t8 [--duration 600] (AUTO_HOLD; test mode: no run needed)",
            ends="duration_s over -> DONE (a summary line per buoy: nearest / farthest / last-seen range); abort -> AUTO_HOLD",
            judge_in="-", judge_out="-", config="test_detect: duration_s 600, lost_s 3, min_obs 2 (logs/detect_<stamp>.jsonl)",
            stages=[Stage("WATCH", "standing still, logging every buoy's range (seen / range / lost / back lines)", "StationKeep", "duration -> DONE"),
                    Stage("DONE", "summary written", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.79: no motion; the mission manager's map does the work (the same path as the missions)"),
    Mission("m0t9", "TEST_GUARD", "ground test: the ZED guard (still; a STOP pulses the motors astern)", "test", "test",
            ["v0.1.80 (WP36; Dmitrii 2026-11-11): the boat holds still while the guard's verdict is logged on every change (clear / SLOW /",
             "STOP with range, bearing extent, pixels, IMU roll / pitch); a STOP sends the scout's BRAKE frame (props in the air!), the back-off is only logged."],
            starts="start_task m0t9 [--duration 600] (AUTO_HOLD; test mode: no run needed) - PROPS IN THE AIR or the boat on a stand",
            ends="duration_s over -> DONE (a summary: the STOPs); abort -> AUTO_HOLD",
            judge_in="-", judge_out="-", config="test_guard: duration_s 600, pulse 0.4 (0 = no motor), pulse_s 1.5; zed_guard: stop_m 1.5, slow_m 3.0",
            stages=[Stage("WATCH", "standing still, logging the guard's verdict; a STOP = one brake pulse", "StationKeep", "duration -> DONE"),
                    Stage("DONE", "summary written", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.80: walk a board toward the bow: the STOP line's range should read 1.5 +- 0.1 m; tilt the camera +-10 deg: it must not move"),
    Mission("m0t10", "TEST_YAW", "yaw couple test: side / aft / both pairs, the measured turn rate (the boat turns)", "test", "test",
            ["v0.1.85 (WP41 T0.5, after the 11 Nov water day: the hull turned ~1.8 deg/s in place): mode-20 frames for duration_s - side pair",
             "opposed (M2 +a, M4 -a), aft pair differential (M1 +a, M3 -a) or both - and the yaw rate from the FCU heading, logged."],
            starts="start_task m0t10 --pair side|aft|both [--amp 0.6 --duration 10 --dir -1] (AUTO_HOLD; test mode only - refused on the course profile)",
            ends="duration_s over -> DONE (a summary: rate_dps) -> AUTO_HOLD; abort -> AUTO_HOLD",
            judge_in="-", judge_out="-", config="test_yaw: pair side, amp 0.6, duration_s 10, settle_s 2 (logs/yaw_<stamp>.jsonl); mode20.yaw_side_gain 0",
            stages=[Stage("TURN", "the yaw frame (mode 20), the heading logged", "StationKeep (frame overridden)", "duration -> DONE"),
                    Stage("DONE", "summary written", "-", "TASK_DONE -> AUTO_HOLD")],
            note="v0.1.85: the morning's water test W0 (2): side, aft, both at 0.6 - three rates for the steering preset"),
    Mission("m0t4", "BOX_INSPECT", "box survey of the dock faces, GREEN chosen, 1 m approach", "test", "built",
            ["the students' box inspection (v0.1.27, ROADMAP WP8): find and geolocate the dock's front faces (3 points per face:",
             "left / mid / right), choose the GREEN one (or --target-colour), align on its normal, revalidate, stop 1 m off it."],
            starts="start_task m0t4sim --scan-x 35 --scan-y 8 [--target-colour red] (vehicle AUTO_HOLD, run R5; world_boxes.yaml for SIL)",
            ends="stop distance reached -> DONE -> AUTO_HOLD; a hold (strafe limit, no target face, ALIGN timeout) -> "
                 "AUTO_SAFE_HOLD, resume goes on; wrong colour at VALIDATE -> back away + abort",
            judge_in="-", judge_out="- (box_survey / box_mission_complete stay in the OCS: ocs/state `internal`)",
            config="box_inspect: in the usv profile (stop_range_m 1, strafe_len_m 6, strafe_side, require_blink false, ...); "
                   "START_TASK args override",
            stages=[Stage("PREFLIGHT", "RTK, heading, the camera's health beat, thrusters", "Hold", "all pass"),
                    Stage("GO_SCAN", "move to the scan point", "Goto", "a face in view / point reached"),
                    Stage("OBSERVE", "turn to scan_hdg_deg, look for 4.5 s (mode 20, zero frames)", "mode 20", "all faces stable / strafe"),
                    Stage("STRAFE", "one 2 m sway leg to strafe_side, heading held", "mode-20 sway", "leg done -> OBSERVE; length used -> on or HOLD"),
                    Stage("ASK_OCS", "box_survey to the OCS; the face: GREEN (or the chosen colour)", "-", "one match -> ALIGN"),
                    Stage("ALIGN", "GUIDED to the 15 m point on the face normal, then yaw in place", "Goto / mode-20 yaw", "within tol and 5 deg; timeout -> HOLD"),
                    Stage("APPROACH", "close in on the normal (line of sight)", "mode-20 surge/yaw", "15 m validation range"),
                    Stage("VALIDATE", "the face still the chosen colour", "mode 20", "colour matches"),
                    Stage("STOP", "taper to stop_range_m off the face", "mode-20 surge/yaw", "face distance reached"),
                    Stage("DONE", "target reached; box_mission_complete to the OCS", "-", "TASK_DONE -> AUTO_HOLD")],
            note="test mission; --step / --from OBSERVE | ALIGN | APPROACH work; built to become m3t1's dock-finding stage later (ROADMAP WP12)"),
]}
HIDDEN = ("goto", "m0t0", "m0t1", "m0t2", "m0t3", "m0t6")   # v0.1.86: the cull as a hide-flag (deletion + the pool profile move = WP42b)
MISSION_ORDER = ["m4t0", "m1t1", "m1t2", "scout", "m3t1", "goto", "m0t0", "m0t1", "m0t2", "m0t3", "m0t4", "m0t6", "m0t8", "m0t9", "m0t10"]
for _c in HIDDEN:
    if _c in MISSIONS:
        MISSIONS[_c].hidden = True


def visible_missions():
    """MISSION_ORDER without the hidden ones (the pickers)."""
    return [c for c in MISSION_ORDER if c in MISSIONS and not MISSIONS[c].hidden]

# code -> (wire task, wire tier); the old wire names stay accepted as aliases (T4 = m4t0, TEST_AUTO = m0t1, T1 = m1t1 ...)
CODES: Dict[str, tuple] = {m.code: (m.task, m.wire_tier) for m in MISSIONS.values()}
CODE_ALIAS: Dict[str, str] = {"m4t1": "m4t0"}   # v0.1.71 (WP27, Dmitrii 2026-11-09): m4t1 renamed m4t0 - "can we go to an incident", the standalone test
_TASK_ALIAS: Dict[str, str] = {}
for _c in MISSION_ORDER:                       # first code per task wins = the core tier
    _TASK_ALIAS.setdefault(MISSIONS[_c].task, _c)

TASK_STAGE_MEANING = {(m.task, st.name): st.meaning for m in MISSIONS.values() for st in m.stages}


def vehicle_words(state: Optional[str], mode: Optional[str], rc6_holds: bool = False, ui_active: bool = False,
                  station_keep: Optional[str] = None) -> Optional[str]:
    """v0.1.81 (WP37 L3, Dmitrii 2026-11-11): what the operator reads beside the vehicle state - V2 with the FCU in HOLD and no RC override
    is "HANDED OVER · HOLD · enable_auto to go" (not "manual mode": the heartbeat's STATE_MANUAL stays right - AUTO would start RunStart);
    "MANUAL (RC)" only when RC6 holds the boat or the FCU is in MANUAL; AUTO_HOLD = "AUTO · LOITER" / "AUTO · HOLD" (the station keep).
    None when nothing more is to be said."""
    if state == "MANUAL":
        if rc6_holds or mode == "MANUAL":
            return "MANUAL (RC)"
        if ui_active:
            return "usv_ui holds the FCU (handover there)"
        if mode == "HOLD":
            return "HANDED OVER · HOLD · enable_auto to go"
        return None
    if state == "AUTO_HOLD":
        return "AUTO · " + ("LOITER" if str(station_keep or "").lower() == "loiter" else "HOLD")
    return None


def resolve(name: Optional[str]):
    """Operator word -> (code, task, wire_tier, sim) or None.  Accepts a code (`m4t0`, `M0T1`, `m1t1sim`), the wire task
    (`T4`, `test_auto`, `GOTO`) or a wire task with the suffix (`T4sim`)."""
    if not name:
        return None
    w = str(name).strip()
    sim = False
    if w.lower().endswith("sim") and len(w) > 3:
        w, sim = w[:-3], True
    code = CODE_ALIAS.get(w.lower(), w.lower())     # v0.1.71 (WP27): m4t1 -> m4t0 (the old word keeps its verdict)
    if code not in MISSIONS:
        code = _TASK_ALIAS.get(w.upper())
        if code is None:
            return None
    m = MISSIONS[code]
    return code, m.task, m.wire_tier, sim


def code_of(task: Optional[str], tier: Optional[str] = None, sim: bool = False) -> str:
    """Wire task (+ tier as the wire or the declaration says it: core | advanced | TIER_CORE | TIER_ADVANCED) -> code."""
    if not task:
        return ""
    t = str(tier or "").lower().replace("tier_", "") or None
    for c in MISSION_ORDER:
        m = MISSIONS[c]
        if m.task == task and (t is None or m.wire_tier in (None, t)):
            return c + ("sim" if sim else "")
    return _TASK_ALIAS.get(str(task).upper(), str(task).lower()) + ("sim" if sim else "")


def mission_for(task: Optional[str], tier: Optional[str] = None) -> Optional[Mission]:
    """The catalogue entry for a wire task (what the vehicle reports) or a code (what the operator typed)."""
    if not task:
        return None
    if task in MISSIONS:
        return MISSIONS[task]
    r = resolve(task)
    if r is None:
        return None
    if tier is None:
        return MISSIONS[r[0]]
    return MISSIONS.get(code_of(r[1], tier)) or MISSIONS[r[0]]


def mission_stage_names(task: Optional[str]) -> List[str]:
    m = mission_for(task)
    return [st.name for st in m.stages] if m else []


def mission_short(task: Optional[str]) -> str:
    """One line for the MISSION box: code, title and the first sentence of `what`."""
    m = mission_for(task)
    return f"{m.code} {m.title}: {m.what[0]}" if m else ""


def how_to_select(task: str, ocs_state: Optional[dict], vehicle_id: str = "USV1", ui_active: Optional[bool] = None,
                  status: Optional[dict] = None):
    """The numbered path from the current state to this mission running, with the exact command per step.

    `task` is a code (m4t0) or a wire task (T4).  Returns (steps, needs): steps = [(num, text, command, note)],
    needs = [(label, ok)] with ok True/False/None (unknown)."""
    m = mission_for(task)
    if m is None:
        return [("-", f"unknown mission {task}", "", "")], []
    code, wire = m.code, m.task
    ocs_state = ocs_state or {}
    r = ocs_state.get("r")
    veh = (ocs_state.get("vehicles") or {}).get(vehicle_id) or {}
    v = veh.get("state")
    running = veh.get("task_name") if v in ("AUTO_TASK", "AUTO_SAFE_HOLD") else None
    bridge = ((status or {}).get("health") or {}).get("bridge_up") if status else None
    needs = [("R5", r == "R5"), ("AUTO_HOLD", v == "AUTO_HOLD"),
             ("handed over", (not ui_active) if ui_active is not None else None),
             ("course", r not in (None, "R0", "R1")), ("bridge", bridge)]
    if m.status == "planned":
        return [("-", f"{code} ({wire}) is not built yet: the stages are the design; start_task {code} is rejected by the vehicle", "", m.note)], needs
    steps: List[tuple] = []
    n = 1
    if running == wire:
        steps.append(("*", f"{code} ({wire}) is running now ({veh.get('task_state')})", "", "hold / abort / end_run / reset apply"))
    elif running:
        steps.append((str(n), f"a task runs ({code_of(running)}): drop it first", "abort", "vehicle -> AUTO_HOLD")); n += 1
    if code == "goto":
        steps.append((str(n), "send the point (bench task)", "goto --x 10 --y 0 --speed 0.6", "START_TASK GOTO with the point")); n += 1
    elif running != wire:
        steps.append((str(n), "preselect it (what start_task runs)", f"mission {code}", "Enter here = into the prompt; `mission %ssim` = fake FCU only" % code)); n += 1
    if r in ("R2", "R6", None, "R0", "R1") and running != wire:
        steps.append((str(n), "declare the run (PREPARE acks -> R3)" if r in ("R2", "R6") else "wait: hub link / course first", "declare" if r in ("R2", "R6") else "", "")); n += 1
    if v == "MANUAL" and running != wire:
        if ui_active:
            steps.append((str(n), "usv_ui holds the FCU: handover there", "handover  (typed in usv_ui)", "then enable_auto")); n += 1
        steps.append((str(n), "vehicle to AUTO_HOLD", "enable_auto", "needs armed, EKF, PREPARE")); n += 1
    if r in ("R3", "R4") and running != wire:
        steps.append((str(n), "wait for RunStart (fake judge: 2 s)", "inject run_start  (bench only)", "-> R5")); n += 1
    if running != wire and code != "goto":
        steps.append((str(n), "start it now", "start_task", f"or start_task {code} directly")); n += 1
        if code != "m0t0":
            steps.append(("", "or arm it for the next run instead", f"mission {code} --auto", "then reset + ready: starts on RunStart"))
    if wire == "T4":
        steps.append((str(n), "T4 waits in LISTEN for the judge", "inject assistance --lat 1.28112 --lon 103.85501 --cseq 1", "fake judge / bench; the real judge sends it")); n += 1
        steps.append(("", "when the boat reports READY", "inject confirm --cseq 1", "the fake judge also confirms by itself"))
    elif wire == "TEST_MANUAL":
        steps.append((str(n), "advance it stage by stage", "step", "x3: S1 -> S2 -> S3 -> DONE; also `step` in usv_ui")); n += 1
    elif wire == "TEST_AUTO":
        steps.append((str(n), "watch the stages change every step_s", "", "step skips a timer; hold pauses it")); n += 1
    if ui_active:
        steps.append(("boat", "usv_ui: type `handover` first; `takeover` / `stop` take the FCU back any time", "", ""))
    else:
        steps.append(("boat", "usv_ui: nothing to type, stay handed over (`takeover` / `stop` drop the vehicle to MANUAL)", "", ""))
    return steps, needs


def task_stage_meaning(task_name, sub_state) -> str:
    return TASK_STAGE_MEANING.get((task_name, sub_state), "")


# the flow of one run as the consoles draw it: (label, how you get there); the current stage is highlighted
FLOW = [("usv_ui ACTIVE", "boat console commands the FCU"), ("handover", "typed in usv_ui: the beat stops"),
        ("MANUAL", "mission manager may take the FCU"), ("declare", "ocs_cli declare -> R3, judge RunStart -> R5"),
        ("enable_auto", "V3 AUTO_HOLD, FCU in HOLD"), ("start_task/goto", "V4 AUTO_TASK"),
        ("hold<->resume", "V5 SAFE_HOLD and back"), ("end_run", "R6"), ("takeover/stop", "typed in usv_ui: ACTIVE again")]
TRANSITIONS = ("handover", "takeover/stop")           # typed in usv_ui; drawn dim, never a "position"


def flow_stage(ocs_state: Optional[dict], vehicle_id: str = "USV1", ui_active: Optional[bool] = None) -> int:
    """Index into FLOW of where the run is now (-1 = unknown)."""
    if ui_active:
        return 0
    if not ocs_state:
        return -1
    r = ocs_state.get("r")
    veh = (ocs_state.get("vehicles") or {}).get(vehicle_id) or {}
    v = veh.get("state")
    if v == "AUTO_SAFE_HOLD":
        return 6
    if v == "AUTO_TASK":
        return 5
    if v == "AUTO_HOLD":
        return 4
    if r == "R6":
        return 7
    if v == "MANUAL":
        return 2 if r in ("R0", "R1", "R2") else 3
    return -1


FLOW_CAPTION = ["boat", "beat off", "V2", "R2>R3", "V3", "V4", "V5", "R6", "to boat"]   # one per FLOW entry, under the cell
FLOW_SPLIT = 3                                                                        # FLOW[:3] = the boat row, FLOW[3:] = the OCS row


def flow_rows(stage: int, wide: bool = True) -> List[List[tuple]]:
    """The two-row snake the consoles draw: [[(label, caption, state)], [...]] with state cur | done | future.
    wide=False shortens the labels for narrow terminals."""
    short = {"start_task/goto": "task/goto", "hold<->resume": "hold/resume", "takeover/stop": "takeover", "usv_ui ACTIVE": "usv_ui"}
    cells = []
    for i, (label, _) in enumerate(FLOW):
        st = "cur" if i == stage else ("done" if (stage >= 0 and i < stage) else "future")
        cells.append((label if wide else short.get(label, label), FLOW_CAPTION[i], st))
    return [cells[:FLOW_SPLIT], cells[FLOW_SPLIT:]]


def flow_line(stage: int, sep: str = " > ") -> List[tuple]:
    """[(text, highlighted)] for one line: usv_ui ACTIVE > handover > MANUAL > ... > takeover/stop."""
    out = []
    for i, (label, _) in enumerate(FLOW):
        out.append((label, i == stage))
        if i < len(FLOW) - 1:
            out.append((sep, False))
    return out


def help_lines() -> List[str]:
    out = ["intent           OCS R      vehicle V            needs / does"]
    for e in PLAYBOOK.values():
        rr = ",".join(e.ocs_r) if e.ocs_r != ("*",) else "any"
        vv = "/".join(e.vehicle_v) if e.vehicle_v != ("*",) else "any"
        tag = " (bench only)" if e.bench_only else ""
        out.append(f"{e.cli:16s} {rr:10s} {vv:20s} {e.does}{tag}")
        if e.needs:
            out.append(f"{'':16s} {'':10s} {'':20s} needs: {e.needs}")
        if e.args:
            out.append(f"{'':16s} {'':10s} {'':20s} args: {e.args}")
        out.append(f"{'':16s} {'':10s} {'':20s} then: {e.then}" + (f"; next: {e.next}" if e.next and e.next != "-" else ""))
    for e in MACROS.values():                                  # v0.1.54: the macros (ocs_cli runs them step by step)
        out.append(f"{e.cli:16s} {'any':10s} {'any':20s} macro: {e.does}")
        out.append(f"{'':16s} {'':10s} {'':20s} then: {e.then}; next: {e.next}")
    out.append("")
    out.append("missions (code = wire task, tier, status): " + "  ".join(f"{c}={MISSIONS[c].task}/{MISSIONS[c].tier}/{MISSIONS[c].status}" for c in MISSION_ORDER))
    out.append("      a `sim` suffix (m1t1sim) = the same mission, rejected unless usv_ui holds the fake FCU; the old names T4 / TEST_AUTO still work")
    out.append("never a software command (ADR-9): " + ", ".join(NEVER))
    out.append("R ladder: " + " ".join(f"{k} {v}" for k, v in R.items()))
    out.append("V ladder: " + " ".join(f"{V_CODE[n]} {n}" for n in V_ORDER) + "; precedence kill > RC override > fault > safe-hold > task > hold")
    out.append("FLOW: " + " > ".join(l for l, _ in FLOW))
    out.append("      usv_ui (the boat console) always owns the FCU link; ACTIVE = it commands, the mission manager refuses enable_auto;")
    out.append("      `handover` typed in usv_ui lets the mission manager take the FCU (you stay in usv_ui); `takeover` / stop / x take it back")
    return out


def cv_source(health: Optional[dict], age_s: Optional[float], cv_publish_up: bool = False) -> Tuple[str, str]:
    """(text, style) for WHERE WE ARE: which perception feeds the boat - `CV sim (AGX, world_pool)` from the sim's health beat
    (fake_perception: source sim, host, world), `CV real (cv_publish)` from the AGX's detector, else `CV none`. Shared by
    op-tui and usv_ui (ROADMAP WP1)."""
    fresh = health is not None and age_s is not None and age_s < 5.0
    if fresh and (health or {}).get("source") == "sim":
        host = str((health or {}).get("host") or "")
        where = "AGX" if "agx" in host.lower() else ("NX" if "usv" in host.lower() or "pixhawk" in host.lower() else (host.split(".")[0] or "?"))
        return f"CV sim ({where}, {(health or {}).get('world') or '?'})", "warn"
    if fresh:
        return f"CV real ({(health or {}).get('detector') or 'cv_publish'})", "ok" if (health or {}).get("ok") else "bad"
    if cv_publish_up:
        return "CV cv_publish up, no health", "warn"
    return "CV none", "dim"
