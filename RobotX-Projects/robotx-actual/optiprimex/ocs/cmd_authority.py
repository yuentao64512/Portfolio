"""cmd_authority: the single command authority (ADR-15, frame 22).

ui/cmd/<v> intents -> guard (run state, vehicle state; normal or bench) -> cmd/<v>/<kind> with intent_id.
Bench mode relaxes the OCS run-state guards and enables injectors; vehicle guards never relax; ARM,
DISARM, kill-clear, leaving MANUAL, FCU modes and actuator values are never commands at all.
"""
from __future__ import annotations

import logging
import os
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from ..companion import fence as F
from ..companion import tuning
from ..companion import speed_k as SK
from ..companion.mqtt import Bus
from . import declaration_checks as DC
from .run_lifecycle import R

log = logging.getLogger("optiprimex.authority")

R_IDX = {v: i for i, v in enumerate(R.values())}

# intent action -> (min R normal, min R bench, vehicle command, cmd topic kind, bench only)
GUARDS: Dict[str, Tuple[int, int, Optional[str], str, bool]] = {
    "DECLARE":      (2, 0, None,           "run",      False),
    "END_RUN":      (3, 0, "STOP_RUN",     "run",      False),
    "RESET_RUN":    (2, 0, "RESET",        "run",      False),
    "STEP":         (0, 0, "STEP",         "run",      False),
    "ENABLE_AUTO":  (2, 0, "ENABLE_AUTO",  "run",      False),
    "DISABLE_AUTO": (0, 0, "DISABLE_AUTO", "run",      False),
    "START_TASK":   (5, 0, "START_TASK",   "run",      False),
    "SELECT_TARGET":(5, 0, "SELECT_TARGET","run",      False),
    "SAVE_POINT":   (0, 0, "SAVE_POINT",   "run",      False),     # v0.1.29: m0t2's points from the OCS (data only, any R)
    "DELETE_POINT": (0, 0, "DELETE_POINT", "run",      False),
    "CLEAR_MAP":    (0, 0, "CLEAR_MAP",    "run",      False),
    "HOME":         (0, 0, "HOME",         "run",      False),     # v0.1.30: the go-home override (any R; the vehicle needs AUTO)     # v0.1.29.1: the boat forgets its detections (not during a task)
    "ABORT_TASK":   (0, 0, "ABORT_TASK",   "run",      False),
    "HOLD":         (0, 0, "HOLD",         "run",      False),
    "RESUME":       (0, 0, "RESUME",       "run",      False),
    "SKIP":         (0, 0, "SKIP",         "run",      False),
    "SESSION":      (0, 0, None,           "run",      False),     # v0.1.69 (WP25): the dry / wet sessions - OCS-side, nothing moves     # v0.1.68 (WP17 F4): free run only (the vehicle says so); refused on the course network
    "CONFIRM_SAFE": (0, 0, "CONFIRM_SAFE", "run",      False),
    "GOTO":         (99, 0, "START_TASK",  "run",      True),
    "INJECT":       (99, 0, None,          "incident", True),
    "MISSION":      (0, 0, None,           "run",      False),
    "CANCEL_AUTO":  (0, 0, None,           "run",      False),     # v0.1.43: during the 3-2-1 after RunStart - no auto-start
    "HOLD_ON_START":(0, 0, None,           "run",      False),
    "REPORT":       (0, 0, None,           "run",      False),     # v0.1.53: START / STOP REPORTING (heartbeats before a declaration)     # v0.1.43: during the 3-2-1 - the mission starts, then HOLD
    "TUNE":         (0, 0, "SET_TUNING",   "run",      False),
    "SPEED":        (0, 0, "SET_SPEED_K",  "run",      False),     # v0.1.94.3 (Dmitrii 14 Nov): the live speed knobs (cruise / circle / gate), any state
    "FCU_MODE":     (0, 0, "FCU_MODE",     "run",      False),     # v0.1.50 (Dmitrii 2026-11-06, ADR-9 lifted for these): the page's
    "FCU_ARM":      (0, 0, "FCU_ARM",      "run",      False),     # mode / ARM / DISARM buttons, as Mission Planner would; the FCU decides
    "FENCE_APPLY":  (0, 0, "FENCE_APPLY",  "run",      False),     # v0.1.53 (Dmitrii 2026-11-06): the USV's FCU geofence from the
    "FENCE_ENABLE": (0, 0, "FENCE_ENABLE", "run",      False),     # map's Apply / on / off (any R; the polygon checked here)
    "RC6":          (0, 0, "SET_RC6",      "run",      False),
    "STATION_KEEP": (0, 0, "SET_STATION_KEEP", "run",  False),     # v0.1.81 (WP37): station_keep hold|loiter (the same gate as rc6_policy)
    "UAV":          (0, 0, None,           "run",      False),     # v0.1.64 (WP14): uav ready on|off / task N|none / resend_course
    "USV_READY":    (0, 0, None,           "run",      False),     # v0.1.82: usv ready on|off (the OCS's gate before DECLARE; nothing to the boat)
    "DECLARE_UAV":  (0, 0, None,           "run",      False),     # v0.1.64 (D7): declare uav on|off (off by default; m1t2 needs it)     # v0.1.50: the RC6 policy latched | hold     # v0.1.49 (WP10k-b): a runtime knob (companion/tuning), MANUAL / AUTO_HOLD only
}
RUN_FREE_TASKS = ("TEST_MANUAL", "TEST_AUTO", "TEST_PATH", "TEST_SWAY", "TEST_ORBIT", "GOTO", "RETURN_HOME", "BOX_INSPECT", "TEST_DETECT", "TEST_GUARD")  # v0.1.37 TEST MODE
MISSION_TASKS = ("T4", "T1", "T3", "GOTO", "TEST_MANUAL", "TEST_AUTO", "TEST_PATH", "TEST_SWAY", "BOX_INSPECT", "TEST_ORBIT", "SCOUT", "TEST_DETECT", "TEST_GUARD", "TEST_YAW")   # v0.1.85: TEST_YAW; v0.1.80: TEST_GUARD; v0.1.79: SCOUT, TEST_DETECT; v0.1.21: + T1, TEST_PATH; v0.1.22: + T3, TEST_SWAY; v0.1.27: + BOX_INSPECT
START_TASKS = ("T1", "T3", "T4", "TEST_MANUAL", "TEST_AUTO", "TEST_PATH", "TEST_SWAY", "BOX_INSPECT", "TEST_ORBIT", "SCOUT", "TEST_DETECT", "TEST_GUARD", "TEST_YAW")   # the wire names START_TASK may carry (schema enum)
MISSION_ARGS = ("tier", "sim", "t4", "lucky", "planner", "names", "bay", "step", "from", "target_colour", "scan_x_n", "scan_y_e", "expected_boxes", "scan_hdg_deg", "strafe_side",
                "lat", "lon", "r_m", "direction", "turns", "speed_mps", "budget_s", "duration_s")
# v0.1.92 (COWORK_HANDOFF addendum 6 §6.3, the "mission modifiers" subpanel): m1t1 / m1t2's START_TASK args, set before ENABLE AUTO.
# The boat reads them as t1 params (t1_safe_passage DEFAULTS: p.update(args)) - so they travel as the boat's types: start_mode str,
# look_lock / circle_exit_tangent bool (False must survive: the MISSION filter keeps these four), scout_entry_s a number.
# v0.1.93.1 (Dmitrii 13 Nov morning): + squeeze (bool, the course default off) and lucky_pairs (one_known | any: the lucky rule's pairs)
MOD_ARGS = ("start_mode", "look_lock", "scout_entry_s", "circle_exit_tangent", "squeeze", "lucky_pairs")
MOD_DEFAULTS = {"start_mode": "advance360", "look_lock": True, "scout_entry_s": 420, "circle_exit_tangent": False,
                "squeeze": False, "lucky_pairs": "one_known"}   # = t1 DEFAULTS 13 Nov + the course profile
MISSION_ARGS = MISSION_ARGS + MOD_ARGS
_ON = ("on", "true", "1", "yes")
_OFF = ("off", "false", "0", "no")


def mod_value(k: str, v):
    """(why, value): one modifier normalised to the boat's type; why = a refusal line (the operator's typo), else ''."""
    if k == "start_mode":
        w = str(v).strip().lower().replace("+", "").replace("-", "").replace("_", "")
        w = {"advance360": "advance360", "advance": "advance360", "straight": "straight"}.get(w)
        return ("", w) if w else (f"start_mode: advance360 | straight (got {v})", None)
    if k == "lucky_pairs":                                   # v0.1.93.1
        w = str(v).strip().lower().replace("-", "_").replace(" ", "_")
        w = {"one_known": "one_known", "one": "one_known", "known": "one_known", "any": "any"}.get(w)
        return ("", w) if w else (f"lucky_pairs: one_known | any (got {v})", None)
    if k in ("look_lock", "circle_exit_tangent", "squeeze"):
        if isinstance(v, bool):
            return "", v
        w = str(v).strip().lower()
        if w in _ON:
            return "", True
        if w in _OFF:
            return "", False
        return f"{k}: on | off (got {v})", None
    if k == "scout_entry_s":
        try:
            f = float(v)
        except (TypeError, ValueError):
            return f"scout_entry_s: seconds, 60 .. 1200 (got {v})", None
        if not 60 <= f <= 1200:
            return f"scout_entry_s: seconds, 60 .. 1200 (got {v})", None
        return "", int(f) if f == int(f) else f
    return f"{k}: not a mission modifier", None


def mod_args(args: Optional[dict]) -> Tuple[str, dict]:
    """(why, {key: value}) for the modifiers present in args (None / '' = not set: the boat's profile decides)."""
    out = {}
    for k in MOD_ARGS:
        v = (args or {}).get(k)
        if v is None or v == "":
            continue
        why, val = mod_value(k, v)
        if why:
            return why, {}
        out[k] = val
    return "", out   # v0.1.30: m0t6   # v0.1.26: + step / from (stage stepping); v0.1.27: m0t4   # v0.1.19: tier / sim from the mission code (the code itself never travels); v0.1.21: m1t1 planner, m0t2 point names
NEVER = ("ARM", "DISARM", "CLEAR_KILL", "SET_MODE", "ACTUATOR", "MANUAL_EXIT")


@dataclass
class IntentResult:
    intent_id: str
    result: str          # accepted | refused
    reason: str = ""
    cmd_msg_id: Optional[str] = None


class CmdAuthority:
    def __init__(self, bus: Bus, lifecycle, vehicles: Dict[str, str], bench_mode: bool, is_course_network: Callable[[], bool],
                 publish_result: Callable[[IntentResult], None], injectors: Optional[Dict[str, Callable[[dict], str]]] = None):
        """vehicles: vehicle_id -> topic token (USV1 -> usv)."""
        self.bus = bus
        self.lifecycle = lifecycle
        self.vehicles = dict(vehicles)
        self._bench_cfg = bench_mode
        self.is_course_network = is_course_network
        self.publish_result = publish_result
        self.injectors = dict(injectors or {})
        self.last_vehicle_state: Dict[str, str] = {}
        self.sent: List[dict] = []
        self.on_declare: Optional[Callable[[dict], Optional[str]]] = None    # returns a refusal reason, or None when accepted
        self.on_end_run: Optional[Callable[[], None]] = None
        self.on_reset_run: Optional[Callable[[], Optional[str]]] = None
        self.on_mission: Optional[Callable[..., Optional[str]]] = None   # (task, auto, extra={tier, sim}) -> refusal or None
        self.mission_task: Optional[str] = None       # the preselected task: start_task without a task runs it
        self.mission_args: dict = {}                  # its tier / sim (v0.1.19)
        # v0.1.41 (WP9c): the RunDeclaration checks - (step, task, args) -> rows (ocs/declaration_checks); refuse or warn by policy
        self.on_check: Optional[Callable[[str, Optional[str], dict], List[dict]]] = None
        self.on_reporting: Optional[Callable[[bool], Optional[str]]] = None   # v0.1.53: REPORT on/off -> refusal or None
        self.on_countdown: Optional[Callable[[str], Optional[str]]] = None
        self.on_uav: Optional[Callable[[str, dict], Tuple[bool, str]]] = None   # v0.1.64: (op, args) -> (ok, text)
        self.on_session: Optional[Callable[[str, dict], Tuple[bool, str]]] = None   # v0.1.69: (op, args) -> (ok, text)
        self.on_start_check: Optional[Callable[[Optional[str], dict], Optional[str]]] = None   # v0.1.66: m1t2 needs the UAV   # v0.1.43: CANCEL_AUTO / HOLD_ON_START -> refusal or None
        self.declaration_policy: str = DC.DEFAULT_POLICY
        # v0.1.53: the OCS profile's `geofence:` (the MAP tab writes it) - re-read at every FENCE_APPLY geofence; ocs_app may set the
        # file, else $OCS_CONFIG, else the geofence the profile had at start (bus.cfg.extra)
        self.geofence_file: Optional[str] = None

    @property
    def bench_mode(self) -> bool:
        return self._bench_cfg and not self.is_course_network()

    # ---- intents --------------------------------------------------------------------------
    def on_intent(self, topic: str, env: dict) -> IntentResult:
        if env.get("kind") != "intent":
            return IntentResult(env.get("msg_id", "?"), "refused", "not an intent")
        intent_id = env["msg_id"]
        body = env["body"]
        action = str(body.get("action", "")).upper()
        vehicle_id = env.get("vehicle_id")
        args = dict(body.get("args") or {})
        if action in NEVER:
            return self._refuse(intent_id, f"{action} is never a GUI command (ADR-9): arm / disarm and the FCU's modes are yours - the transmitter, or usv_ui's typed arm / disarm")
        if action not in GUARDS:
            return self._refuse(intent_id, f"unknown intent {action}: the words are in op-tui's help / the page's HELP tab (the ten words: ready start_task hold resume abort manual end_run reset save_point home)")
        r_norm, r_bench, vcmd, kind, bench_only = GUARDS[action]
        if action == "SKIP" and self.is_course_network():      # v0.1.68: like the bench words - never on the course network
            return self._refuse(intent_id, "SKIP is never used on the course network (free run is a bench mode)")
        if bench_only and not self.bench_mode:
            return self._refuse(intent_id, f"{action} is bench only: start the simulator / bench (scripts/launch.sh -> sim, or scripts/run_sim.sh) to use it")
        r_min = r_bench if self.bench_mode else r_norm
        if getattr(self, "test_mode", False):       # v0.1.37 TEST MODE: enable_auto and the run-free tasks need no run
            t0 = body.get("task") or (self.mission_task if action == "START_TASK" else None)
            if action == "ENABLE_AUTO" or action == "GOTO" or (action == "START_TASK" and t0 in RUN_FREE_TASKS):
                r_min = 0
        if self.lifecycle.r_index < r_min:
            hint = " - declare first (the course runs every task in a judged run)" if action in ("START_TASK", "ENABLE_AUTO", "SELECT_TARGET") else ""
            return self._refuse(intent_id, f"{action} needs R{r_min}+ (now {self.lifecycle.body()['r']} {self.lifecycle.state}){hint}")
        if action == "DECLARE":
            # the lifecycle takes DECLARE from R2, R4 (stalled) and R6 only; refusing here keeps declaration_seq honest
            if not self.lifecycle.m.can("DECLARE"):
                r = self.lifecycle.body()["r"]
                hint = "end_run first" if r in ("R3", "R5") else "wait for the hub link and a valid course"
                return self._refuse(intent_id, f"DECLARE needs R2, R4 or R6 (now {r} {self.lifecycle.state}): {hint}")
            rows = self._check("declare", None, args)
            block = DC.blocking(rows, self.declaration_policy)
            if block:
                return self._refuse(intent_id, f"DECLARE refused (declaration.policy refuse): {DC.summary(block)}")
            if self.on_declare:
                why = self.on_declare(args)
                if why:
                    return self._refuse(intent_id, f"DECLARE refused: {why}")
            return self._accept(intent_id, reason="PREPARE sent; R3 when every vehicle acks" + self._warned(rows))
        if action == "END_RUN":
            if not self.lifecycle.m.can("END"):
                r = self.lifecycle.body()["r"]
                return self._refuse(intent_id, f"END_RUN needs R3-R5 (now {r} {self.lifecycle.state}): nothing to end")
            if self.on_end_run:
                self.on_end_run()
            return self._accept(intent_id)
        if action == "RESET_RUN":
            if self.lifecycle.r_index < 2:
                return self._refuse(intent_id, f"RESET_RUN: nothing to reset in {self.lifecycle.body()['r']} {self.lifecycle.state}")
            why = self.on_reset_run() if self.on_reset_run else None
            if why:
                return self._refuse(intent_id, why)
            return self._accept(intent_id, reason="run context dropped on the OCS and the vehicles")
        if action == "MISSION":
            t = str(args.get("task") or "").upper()
            auto = bool(args.get("auto"))
            if t in ("", "OFF", "NONE"):
                t = None
            elif t not in MISSION_TASKS:
                return self._refuse(intent_id, f"MISSION: unknown task {t} (one of {', '.join(MISSION_TASKS)}, or off)")
            extra = {k: args[k] for k in MISSION_ARGS if args.get(k) not in (None, False, "")}
            mwhy, mods = mod_args(args)                  # v0.1.92: the modifiers in the boat's types (False kept)
            if mwhy:
                return self._refuse(intent_id, f"MISSION refused: {mwhy}")
            extra.update(mods)
            rows = self._check("mission", t, dict(extra, auto=auto)) if t else []
            block = DC.blocking(rows, self.declaration_policy, "mission")
            if block:
                return self._refuse(intent_id, f"MISSION {DC.mission_code(t, extra)} refused (declaration.policy refuse): {DC.summary(block)}")
            why = self.on_mission(t, auto, extra) if self.on_mission else None
            if why:
                return self._refuse(intent_id, why)
            self.mission_task = t
            self.mission_args = extra if t else {}
            tag = (" " + " ".join(f"{k}={v}" for k, v in extra.items())) if (t and extra) else ""
            return self._accept(intent_id, reason=f"mission {t or 'off'}{tag}" + (" (auto-start on RunStart)" if t and auto else "")
                                + self._warned([x for x in rows if x["scope"] == "mission"]))
        if action == "SESSION":                        # v0.1.69 (WP25): bench / lab only; the course network: collect, stop, result
            op, name = str(args.get("op") or ""), str(args.get("name") or "")
            if op == "start" and name != "collect":
                if self.is_course_network():
                    return self._refuse(intent_id, f"session {name} is a bench / lab session: on the course network only `session start "
                                                   f"collect` (and stop / result)")
                if not (self._bench_cfg or getattr(self, "test_mode", False)):
                    return self._refuse(intent_id, f"session {name}: the course profile runs no dry sessions (bench_mode / test_mode off) - "
                                                   f"use the lab or pool profile; `session start collect` works here")
            if not self.on_session:
                return self._refuse(intent_id, "no sessions on this OCS")
            ok, text = self.on_session(op, args)
            return self._accept(intent_id, reason=text) if ok else self._refuse(intent_id, text)
        if action in ("UAV", "DECLARE_UAV"):           # v0.1.64: the UAV's words - the OCS decides, the adapter carries them
            if not self.on_uav:
                return self._refuse(intent_id, "no UAV on this OCS")
            op = "declare" if action == "DECLARE_UAV" else str(args.get("op") or "")
            ok, text = self.on_uav(op, args)
            return self._accept(intent_id, reason=text) if ok else self._refuse(intent_id, text)
        if action == "USV_READY":                      # v0.1.82: the USV's PILOT READY - an OCS flag
            if not getattr(self, "on_usv_ready", None):
                return self._refuse(intent_id, "no USV pilot-ready switch on this OCS")
            ok, text = self.on_usv_ready(bool(args.get("on")))
            return self._accept(intent_id, reason=text) if ok else self._refuse(intent_id, text)
        if action == "REPORT" and args.get("vehicle"):   # v0.1.64: report on|off uav; v0.1.82: usv
            why = self.on_reporting(bool(args.get("on")), args["vehicle"]) if self.on_reporting else "no reporting switch on this OCS"
            who = "the USV" if args["vehicle"] == "usv" else "the UAV"
            return self._refuse(intent_id, why) if why else self._accept(
                intent_id, reason=f"reporting for {who} {'ON: its heartbeats at 2 Hz to RoboCommand' if args.get('on') else 'OFF'}")
        if action == "REPORT":                         # v0.1.53: the OCS's own switch - nothing goes to the vehicle
            why = self.on_reporting(bool(args.get("on"))) if self.on_reporting else "no reporting switch on this OCS"
            return self._refuse(intent_id, why) if why else self._accept(
                intent_id, reason="reporting ON: heartbeats for every vehicle at 2 Hz - watch JUDGE RECEIVES on the judge page"
                if args.get("on") else "reporting OFF: heartbeats stopped now, whatever R (declared or not) - back on with report on "
                "or a new declaration")                                      # v0.1.54: a hard off
        if action in ("CANCEL_AUTO", "HOLD_ON_START"):
            why = self.on_countdown(action) if self.on_countdown else "no auto-start countdown on this OCS"
            if why:
                return self._refuse(intent_id, why)
            return self._accept(intent_id, reason="auto-start cancelled: start_task to start by hand" if action == "CANCEL_AUTO"
                                else "the mission starts at 0, then HOLD (resume to go on)")
        if action == "INJECT":
            what = args.get("what")
            fn = self.injectors.get(what)
            if fn is None:
                return self._refuse(intent_id, f"unknown injector {what}: one of {', '.join(sorted(self.injectors)) or '(none on this profile)'}")
            msg = fn(args)
            return self._accept(intent_id, reason=msg)
        if vehicle_id not in self.vehicles:
            return self._refuse(intent_id, f"unknown vehicle {vehicle_id}: one of {', '.join(sorted(self.vehicles))}")
        task = body.get("task")
        if action == "TUNE":                            # v0.1.49: the whitelist and the range here, the vehicle state here and on the boat
            if args.get("save"):
                args = {"save": True}
            else:
                why, value = tuning.check(args.get("key"), args.get("value"))
                if why:
                    return self._refuse(intent_id, why)
                vs = self.last_vehicle_state.get(vehicle_id)
                if vs not in tuning.TUNE_STATES:
                    return self._refuse(intent_id, f"TUNE refused in {vs or 'an unknown vehicle state'}: knobs change only in MANUAL / AUTO_HOLD "
                                                   f"(a running task keeps the values it started with - abort it, or wait for its end)")
                args = {"key": str(args["key"]), "value": value}
        if action == "SPEED":                           # v0.1.94.3: 0.5-2.0 per knob, checked here and on the boat
            why, ks = SK.check(args)
            if why:
                return self._refuse(intent_id, why)
            args = ks
        if action in ("FENCE_APPLY", "FENCE_ENABLE"):
            why, args = self._fence_args(action, args, self.last_vehicle_state.get(vehicle_id))
            if why:
                return self._refuse(intent_id, why)
            msg_id = self.emit(vehicle_id, kind, vcmd, None, args, intent_id=intent_id)
            note = (f"{len(args['points'])} points ({args['source']}) to the FCU" if action == "FENCE_APPLY"
                    else f"fence {'on' if args['on'] else 'off'} to the FCU")
            return self._accept(intent_id, reason=note, cmd_msg_id=msg_id)
        if action == "START_TASK":
            if not task:
                task = self.mission_task or "T4"                       # the preselected mission, else T4
                for k, v in self.mission_args.items():                 # ... with its tier / sim
                    args.setdefault(k, v)
            if task not in START_TASKS:
                return self._refuse(intent_id, f"START_TASK needs task {'/'.join(START_TASKS)} (got {task})")
            mwhy, mods = mod_args(args)                  # v0.1.92: a typed start_task --look-lock off etc. -> the boat's types
            if mwhy:
                return self._refuse(intent_id, f"START_TASK refused: {mwhy}")
            args.update(mods)
        if action == "GOTO":
            task = "GOTO"
        rows: List[dict] = []
        if action in ("START_TASK", "GOTO"):
            rows = [x for x in self._check("start_task" if action == "START_TASK" else "goto", task, args) if x["scope"] == "mission"]
            block = DC.blocking(rows, self.declaration_policy)
            if block:
                return self._refuse(intent_id, f"{action} {DC.mission_code(task, args)} refused (declaration.policy refuse): {DC.summary(block)}")
            why = self.on_start_check(task, args) if (self.on_start_check and action == "START_TASK") else None   # v0.1.66: m1t2 needs the UAV
            if why:
                return self._refuse(intent_id, f"START_TASK refused: {why}")
        msg_id = self.emit(vehicle_id, kind, vcmd, task, args, intent_id=intent_id)
        return self._accept(intent_id, reason=self._warned(rows).lstrip(" ·"), cmd_msg_id=msg_id)

    # ---- v0.1.53: the FCU geofence -----------------------------------------------------------------------------------------
    def course_boundary(self) -> List:
        """The received RxCourse boundary (handbook 3.4.9: ordered corners, first == last), [] when none came."""
        c = getattr(self.lifecycle, "course", None) or {}
        return list(c.get("boundary") or [])

    def profile_geofence(self) -> List:
        """The OCS profile's `geofence:` as the MAP tab saved it (read now, not at start)."""
        path = self.geofence_file or os.environ.get("OCS_CONFIG")
        if path and os.path.isfile(path):
            try:
                import yaml
                with open(path) as fh:
                    return list((yaml.safe_load(fh) or {}).get("geofence") or [])
            except Exception:
                log.exception("geofence: reading %s", path)
        cfg = getattr(self.bus, "cfg", None)
        return list(((getattr(cfg, "extra", None) or {}).get("geofence")) or [])

    def _fence_args(self, action: str, args: dict, vstate: Optional[str]) -> Tuple[Optional[str], dict]:
        """(refusal, the vehicle's args). FENCE_APPLY {source course|geofence[, points]}: the polygon from the OCS's state (the
        received course, or the profile's geofence - the map may send the points it shows), >= 3 distinct points, and a geofence
        must lie inside the received course when there is one (RoboCommand's rule, 1 m). Not in AUTO_TASK (the vehicle checks it too).
        FENCE_ENABLE {on}: off always; on not in AUTO_TASK."""
        if action == "FENCE_ENABLE":
            on = args.get("on")
            if isinstance(on, str):
                on = {"on": True, "true": True, "1": True, "off": False, "false": False, "0": False}.get(on.lower())
            if not isinstance(on, bool):
                return "FENCE_ENABLE: on | off", args
            if on and vstate == "AUTO_TASK":
                return "fence on refused in AUTO_TASK: a breach would stop the running task - switch it on before the start", args
            return None, {"on": on}
        source = str(args.get("source") or "").lower()
        if source not in F.SOURCES:
            return f"FENCE_APPLY: the source is course | geofence (got {source or 'none'})", args
        if vstate == "AUTO_TASK":
            return "FENCE_APPLY refused in AUTO_TASK: a new fence mid-task could stop the boat - hold / abort first, or apply before the start", args
        course = self.course_boundary()
        if source == "course":
            if len(course) < 3:
                return "fence apply course: no course received yet (RxCourse) - wait for the judge's course, or apply the geofence", args
            pts = course
        else:
            pts = args.get("points") or self.profile_geofence()
            if not pts:
                return "fence apply geofence: the OCS profile has no geofence - draw one on the MAP (geofence, save)", args
        why = F.problem(pts)
        if why:
            return f"fence apply {source}: {why}", args
        if source == "geofence" and len(course) >= 3:
            why = F.outside_course(pts, course)
            if why:
                return f"fence apply geofence refused: {why} (it must lie inside the received course)", args
        ring = [[round(a, 7), round(b, 7)] for a, b in F.ring(pts)]
        return None, {"points": ring, "source": source}

    def _check(self, step: str, task: Optional[str], args: dict) -> List[dict]:
        """v0.1.41 (WP9c): the declaration rows for this step (none without the hook: the unit tests' bare authority)."""
        if not self.on_check:
            return []
        try:
            return list(self.on_check(step, task, dict(args or {})) or [])
        except Exception:
            log.exception("declaration check")
            return []

    def _warned(self, rows: List[dict]) -> str:
        """The accepted intent's reason carries what the checks found (policy warn, or a soft row under refuse)."""
        return (" · check: " + DC.summary(rows)) if rows else ""

    def emit(self, vehicle_id: str, kind: str, action: str, task: Optional[str] = None, args: Optional[dict] = None,
             intent_id: Optional[str] = None, origin: Optional[str] = None) -> str:
        v = self.vehicles[vehicle_id]
        topic = self.bus.cfg.team("cmd", v, kind)
        self.bus.run_id = self.lifecycle.run_id
        self.bus.declaration_seq = self.lifecycle.declaration_seq or None
        body = {"action": action, "task": task, "args": args or {}}
        msg_id = self.bus.publish(topic, "cmd", body, qos=1, intent_id=intent_id, vehicle_id=vehicle_id,
                                  origin=origin or ("bench" if self.bench_mode else "real"))
        self.sent.append({"t": time.time(), "vehicle": vehicle_id, "kind": kind, "action": action, "task": task,
                          "msg_id": msg_id, "intent_id": intent_id})
        log.info("cmd -> %s %s %s %s", vehicle_id, action, task or "", args or "")
        return msg_id

    def _accept(self, intent_id: str, reason: str = "", cmd_msg_id: Optional[str] = None) -> IntentResult:
        r = IntentResult(intent_id, "accepted", reason, cmd_msg_id)
        self.publish_result(r)
        return r

    def _refuse(self, intent_id: str, reason: str) -> IntentResult:
        log.info("intent refused: %s", reason)
        r = IntentResult(intent_id, "refused", reason)
        self.publish_result(r)
        return r
