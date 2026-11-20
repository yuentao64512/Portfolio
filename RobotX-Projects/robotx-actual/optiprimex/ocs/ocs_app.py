"""ocs_app: judge client + team client + run lifecycle + heartbeat publisher + report mapper + command
authority (MODULES 3.4). One process on the OCS laptop; on the bench it runs against fake_robocommand.

Topics it owns on the hub: ocs/state (1 Hz), ocs/intent_result, ocs/rc_log, ocs/alert.
"""
from __future__ import annotations

import argparse
import json
import os
import logging
import signal
import threading
import time
from collections import OrderedDict
from typing import Dict, List, Optional, Tuple

from ..companion import config as cfgmod
from ..companion.config import Config
from ..companion.mqtt import Bus
from ..companion.state import TaskStateEvent, utc_iso
from . import declaration_checks as DC
from .cmd_authority import CmdAuthority, IntentResult
from . import cmd_authority as CA
from . import geofence as GF
from .declaration_checks import mission_code  # noqa: F401  (v0.1.41: moved to declaration_checks, the one table; the name stays here)
from .heartbeat_publisher import HeartbeatPublisher
from .judge_client import SEQ_FILE_DEFAULT, FakeJudgeClient, JudgeClient, NullJudgeClient, seq_file_path
from .report_mapper import ReportMapper, internal_event
from .run_lifecycle import R, RunLifecycle

log = logging.getLogger("optiprimex.ocs")


class OcsApp:
    def __init__(self, cfg: Config, bus: Optional[Bus] = None, judge: Optional[JudgeClient] = None):
        self.cfg = cfg
        o = cfg.extra.get("ocs", {})
        self.team_id: str = o.get("team_id", "SGIT")
        self.vehicles: Dict[str, str] = dict(o.get("vehicles") or {"USV1": "usv"})
        self.declaration_cfg: dict = dict(o.get("declaration") or {})
        self.domain_of: Dict[str, str] = {vid: ("air" if tok == "uav" else "surface") for vid, tok in self.vehicles.items()}
        # v0.1.64 (WP14, D6-D7): the UAV is a vehicle of the profile, declared only with `declare uav on` (off by default); everything
        # UAV below is behind uav_vid (a vehicle with the token uav in `vehicles`) and the toggle - the USV lives without it
        self.uav_vid: Optional[str] = next((v for v, t in self.vehicles.items() if t == "uav"), None)
        # the boat: the first surface vehicle (never "the first key" - a profile written by yaml.safe_dump sorts SGIT-53577 before USV1)
        self.primary_vid: str = next((v for v, t in self.vehicles.items() if t != "uav"), next(iter(self.vehicles)))
        self.uav_cfg: dict = dict(o.get("uav") or {})
        self.declare_uav = bool(self.uav_cfg.get("declare_default", False))
        self.report_uav = False                                         # `report on uav`: heartbeats for an undeclared UAV
        # v0.1.82 (Dmitrii at the venue: "USV heartbeat is autostarted to robocommand, I need it on the button ... pump heartbeat to RC, Pilot
        # Ready and finally Declare"): ocs.usv_heartbeat button = the boat is silent on the wire until ▶ HEARTBEAT USV (`report on usv`),
        # DECLARE refuses it until PILOT READY (`usv ready on`, an OCS flag - nothing goes to the boat; cleared by STOP / RESET);
        # auto (the default, the bench) = the v0.1.59 rule: heartbeats from the first fresh status, no gate
        self.usv_flow = "button" if str(o.get("usv_heartbeat", "auto")).lower() == "button" else "auto"
        self.report_usv = self.usv_flow == "auto"
        self.usv_ready = self.usv_flow == "auto"
        self.auto_before_declare = bool(o.get("auto_before_declare", True))    # v0.1.90.7 / .8: ENABLE AUTO, then DECLARE - everywhere
        self.uav_auto_ready = bool(o.get("uav_pilot_auto", True))      # v0.1.90.5 (Dmitrii: "remove pilot ready buttons"): UAV_READY by itself
        self._uav_ready_t = -1e9
        self.uav_stale_s = float(self.uav_cfg.get("telemetry_stale_s", 5.0))
        self.uav_view: Dict[str, object] = {"last_refusal": None, "last_fence_change": None, "redeclared": 0, "course_fp": None,
                                            "course_forward": None, "last_command": None, "reports": 0, "last_report": None}
        self._uav_course: Optional[dict] = None                         # the RxCourse as a protobuf dict (what his bridge forwards)
        self._uav_course_fp: Optional[str] = None
        self._uav_course_sent_t = 0.0
        self._uav_fwd: Dict[str, dict] = {}                             # RC_FORWARD msg_id -> {kind, seq, auto_ack}
        self._rate_dropped_seen, self._rate_alert_t = 0, 0.0
        self.course_nic = bool(o.get("course_nic", False))
        self.bus = bus or Bus(cfg, client_id=cfg.mqtt.client_id, source="ocs_app")
        self.judge = judge or self._make_judge(o)
        self.countdown_s = float(o.get("autostart_countdown_s", 3.0))      # v0.1.43 (§7d.4): the 3-2-1 before the --auto start
        self.lifecycle = RunLifecycle(stall_s=float(o.get("stall_s", 60.0)), on_transition=self._on_transition, on_alert=self.alert,
                                      prepare_ack_s=float(o.get("prepare_ack_s", 3.0)))
        self.status_fresh_s = float(o.get("status_fresh_s", 3.0))      # declare needs every vehicle heard within this
        self.heartbeats = HeartbeatPublisher(self.judge, list(self.vehicles), hz=float(o.get("heartbeat_hz", 2.0)),
                                             stale_stop_s=float(o.get("stale_stop_s", 10.0)),
                                             stale_by={self.uav_vid: float(self.uav_cfg.get("stale_stop_s", 5.0))} if self.uav_vid else None)
        self.heartbeats.vehicles = self.hb_vehicles()                   # v0.1.64: an undeclared UAV is silent on the wire
        self.reports = ReportMapper(self.judge.publish_report, min_period_s=float(o.get("report_min_period_s", 1.0)))
        self.authority = CmdAuthority(self.bus, self.lifecycle, self.vehicles, bool(o.get("bench_mode", False)),
                                      lambda: self.course_nic or self.judge.network == "course", self._publish_intent_result,
                                      injectors={"run_start": self.inject_run_start, "course": self.inject_course,
                                                 "assistance_request": self.inject_assistance, "keep_out_zone": self.inject_keep_out,
                                                 "all_clear": self.inject_all_clear, "moving_object": self.inject_moving_object,
                                                 "readiness_confirm": self.inject_readiness_confirm})
        self.authority.test_mode = bool(o.get("test_mode", False))      # v0.1.37 (WP10d) TEST MODE
        self.authority.on_declare = self.declare
        self.authority.on_session = self.on_session                   # v0.1.69 (WP25): the sessions (run by the page server)
        self.session_state: Optional[dict] = None
        self.session_t = float("-inf")
        self.authority.on_end_run = self.end_run
        self.authority.on_reset_run = self.reset_run
        self.authority.on_mission = self.set_mission
        self.authority.on_check = self.declaration_rows                 # v0.1.41 (WP9c) the RunDeclaration checks
        self.authority.on_countdown = self.on_countdown                 # v0.1.43: cancel_auto / hold_on_start
        self.authority.on_usv_ready = self.on_usv_ready                 # v0.1.82: usv ready on|off
        self.authority.on_uav = self.on_uav                             # v0.1.64: declare uav / uav ready / uav task / uav resend_course
        self.authority.on_start_check = self.start_check                # v0.1.66: m1t2 needs the UAV
        self.reporting = False                                          # v0.1.53: START REPORTING (heartbeats before a declaration)
        self.hb_hard_off = False                                        # v0.1.54: `report off` holds until report on / a declaration
        # v0.1.59 (B1, handbook 5.3 #8a: heartbeats at all times in the course area): on the judge wire (judge protobuf = rc-test or the
        # course) heartbeats are ON from the first fresh status of each declared-profile vehicle, whatever R / V (MANUAL, KILLED as they
        # are; never fabricated - HeartbeatPublisher sends only for a vehicle with a status younger than stale_stop_s). `report off` is
        # the exception (the bench, the boat out of the water); judge fake keeps the v0.1.53 rule (from the declaration).
        self.attempt, self._attempt_course = 0, None                    # v0.1.59 (B2): declarations published for the current course
        self.reporting_default = bool(o.get("heartbeats_default_on", o.get("judge", "fake") == "protobuf"))
        if self.reporting_default:
            self.reporting = True
            self.heartbeats.enabled = True
        self.authority.on_reporting = self.on_reporting
        self.authority.declaration_policy = DC.policy_of(self.declaration_cfg)
        self.declaration_mandatory = tuple(self.declaration_cfg.get("mandatory") or DC.MANDATORY)
        self.declaration_mandatory_hard = bool(self.declaration_cfg.get("t4_required", False))     # v0.1.44: T4 a modifier (soft)
        self._prepare_msg_ids: Dict[str, str] = {}   # PREPARE msg_id -> vehicle_id while a declaration waits for acks
        self.mission: Dict[str, object] = {"task": None, "auto": False, "started_run": None, "armed_run": None,
                                           "tier": None, "sim": False, "code": None}   # the preselected mission (v0.1.19: + tier / sim / code)
        # v0.1.86 (WP42 §2.1): the launcher's preset in the OCS profile copy - `launch: {preset, mission, task, extra, auto, tiers, record}`:
        # the mission preselected at start (`ocs_cli mission off` still clears it), the declaration's tier row for the run
        self.launch: Dict[str, object] = dict(((cfg.extra or {}).get("launch")) or {})
        self.judge_seen: Dict[str, dict] = {}        # what RoboCommand sent us, last of each kind, with the time (judge_view)
        self._run_dropped_alerted: set = set()
        self._vehicle_run_seen: Dict[str, int] = {}
        self.last_status: Dict[str, dict] = {}
        self.acks: List[dict] = []
        self.rc_log: List[dict] = []
        self.alerts: List[dict] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._cmd_seq_seen: set = set()
        # v0.1.78.8 (Dmitrii 2026-11-09, Team Village: the boat missed course ALPHA - forwarded at 13:02:10, the boat's bridge up at 13:02:11):
        # per boat, the course it acknowledged; re-sent every COURSE_RESEND_S while it is online and has not, and after a status gap
        # (a reconnect / a mission-manager restart may have lost it). A boat that refuses it is not asked again for that course.
        self._course_sync: Dict[str, dict] = {}
        self._status_mono: Dict[str, float] = {}
        self.pending_confirm: Dict[int, str] = {}   # command_seq -> vehicle_id (readiness awaiting confirm)
        self.internal: Dict[str, Dict[str, dict]] = {}   # v0.1.27: vehicle -> kind -> the last OCS-internal event (m0t4)
        # v0.1.43 (OCS_V2_SPEC §6): TIMED OUT - a cmd with no ack within ack_timeout_s; a later ack is logged as late, never silent
        self.ack_timeout_s = float(o.get("ack_timeout_s", 5.0))
        self._ack_lock = threading.Lock()
        self._sent_scanned = 0                           # authority.sent entries already taken into _await_ack
        self._await_ack: Dict[str, dict] = {}            # cmd msg_id -> the authority.sent record, until its first ack
        self._acked_ids: "OrderedDict[str, None]" = OrderedDict()   # ack_of seen (an ack may beat the sweep to a fresh cmd)
        self._timed_out_by_id: Dict[str, dict] = {}
        self.timed_out: List[dict] = []                  # the last TIMED_OUT_KEEP records (ocs/state.timed_out)
        self.timed_out_total = 0
        self.judge.log_cb = self._rc_log
        self.judge.on_link(self._on_judge_link)
        self.judge.on_course(self.on_course)
        self.judge.on_command(self.on_rc_command)
        self.foreign_declaration: Optional[dict] = None              # v0.1.65 (D18): an RxRequest on our topic we did not send
        if hasattr(self.judge, "foreign_cb"):
            self.judge.foreign_cb = self.on_foreign_declaration
        sf = getattr(self.judge, "seq_file", None)                      # v0.1.64 (D9): the last declaration beside the seq counters
        self.decl_file: Optional[str] = os.path.join(os.path.dirname(sf), "rc_last_declaration.json") if sf else None   # (off with them)

    @staticmethod
    def _make_judge(o: dict) -> JudgeClient:
        kind = o.get("judge", "fake")
        # v0.1.23: the judge's address comes from the environment when set (RC_HOST / RC_PORT, loaded by the launchers from
        # network*.env) - the course RoboCommand (192.168.65.2 ...) never goes into a YAML profile
        host = os.environ.get("RC_HOST") or o.get("judge_host", "127.0.0.1")
        port = int(os.environ.get("RC_PORT") or o.get("judge_port", 1883))
        if kind == "fake":
            return FakeJudgeClient(host, port, o.get("team_id", "SGIT"),
                                   rate_cap=float(o.get("msg_rate_cap", 5.0)), network=o.get("judge_network", "bench"),
                                   seq_file=seq_file_path(o.get("rc_seq_file") or SEQ_FILE_DEFAULT))
        if kind == "protobuf":
            from .judge_client import ProtobufJudgeClient
            return ProtobufJudgeClient(host, port, o.get("team_id", "SGIT"), rate_cap=float(o.get("msg_rate_cap", 5.0)),
                                       network=o.get("judge_network", "course"),
                                       seq_file=seq_file_path(o.get("rc_seq_file") or SEQ_FILE_DEFAULT))
        return NullJudgeClient()

    # ---- lifecycle --------------------------------------------------------------------------
    def start(self) -> None:
        ns = self.cfg.mqtt.ns
        self.bus.subscribe(f"{ns}/status/#", self.on_status, qos=0)
        self.bus.subscribe(f"{ns}/relay/usv/status/#", self.on_status, qos=0)
        self.bus.subscribe(f"{ns}/ack/#", self.on_ack, qos=1)
        self.bus.subscribe(f"{ns}/event/#", self.on_event, qos=1)
        self.bus.subscribe(f"{ns}/relay/usv/event/#", self.on_event, qos=1)
        self.bus.subscribe(f"{ns}/ui/cmd/#", self.on_intent, qos=1)
        self.bus.subscribe("ocs/session", self._on_session_state, qos=0, raw=True)   # v0.1.69: the page server's sessions
        self.bus.start()
        self.judge.start()
        self._thread = threading.Thread(target=self._loop, name="ocs-loop", daemon=True)
        self._thread.start()
        self.apply_launch()
        log.info("ocs_app started: team %s vehicles %s judge=%s bench=%s", self.team_id, list(self.vehicles), self.judge.name,
                 self.authority.bench_mode)

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        self.judge.stop()
        self.bus.stop()

    def _loop(self) -> None:
        next_state = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            try:
                self.lifecycle.tick()
                self._countdown_tick()
                self.sweep_acks()
                self.course_resend_tick()                                    # v0.1.78.8
                self.uav_ready_tick()                                        # v0.1.90.5
                self.heartbeats.vehicles = self.hb_vehicles()
                self.heartbeats.tick()
                self.reports.tick()
                self._rate_watch()
                if now >= next_state:
                    next_state = now + 1.0
                    self.publish_state()
            except Exception:
                log.exception("ocs loop")
            time.sleep(0.05)

    # ---- state out --------------------------------------------------------------------------
    def judge_view(self) -> dict:
        """The critical payloads from RoboCommand, last of each, with their age: the operator must see them (handbook 3.4.16)."""
        now = time.monotonic()
        out = {"link": self.judge.connected, "name": self.judge.name}
        c = self.lifecycle.course or {}
        if c:
            out["course"] = {"course_id": c.get("course_id"), "points": len(c.get("boundary") or []), "pinger_hz": c.get("pinger_hz"),
                             "valid": self.lifecycle._course_valid(c), "age_s": round(now - self.judge_seen.get("course", {}).get("t", now), 1)}
        for kind, rec in self.judge_seen.items():
            if kind == "course":
                continue
            d = {k: v for k, v in rec.items() if k != "t"}
            d["age_s"] = round(now - rec["t"], 1)
            out[kind] = d
        out["pending_confirm"] = list(self.pending_confirm.keys())
        return out

    def launch_tiers(self) -> dict:
        """v0.1.86: the preset's tier row ({T1: ADVANCED, ...} or TIER_*), normalised; {} without one."""
        out = {}
        for k, v in dict(self.launch.get("tiers") or {}).items():
            v = str(v).upper()
            out[str(k).upper()] = v if v.startswith("TIER_") else f"TIER_{v}"
        return out

    def apply_launch(self) -> None:
        """v0.1.86 (WP42 §2.1): the preset's mission preselected at start (with --auto when the preset says auto_mission)."""
        task = self.launch.get("task")
        if task and not self.mission.get("task"):
            self.set_mission(str(task), bool(self.launch.get("auto")), dict(self.launch.get("extra") or {}))
            self.authority.mission_task = str(task)                   # v0.1.90: a bare start_task (▶ START) = this mission, not T4
            if self.launch.get("declare_uav") and self.uav_vid:        # v0.1.90.4: an m1t2 preset declares the UAV by itself
                ok, txt = self.on_uav("declare", {"on": True})
                log.info("launch preset %s: declare uav on (%s)", self.launch.get("preset"), "ok" if ok else txt)
            self.authority.mission_args = {k: v for k, v in dict(self.launch.get("extra") or {}).items()
                                           if k in CA.MISSION_ARGS and v not in (None, False, "")}
            self.authority.mission_args.update(self.mission.get("mods") or {})   # v0.1.92: the modifiers (False kept)
            log.info("launch preset %s: mission %s preselected (auto %s), tiers %s", self.launch.get("preset"), self.launch.get("mission"),
                     bool(self.launch.get("auto")), self.launch_tiers() or "profile")

    def set_mission(self, task: Optional[str], auto: bool, extra: Optional[dict] = None) -> Optional[str]:
        extra = dict(extra or {}) if task else {}
        self.mission = {"task": task, "auto": bool(auto and task), "started_run": None, "armed_run": None,
                        "tier": extra.get("tier"), "sim": bool(extra.get("sim")), "code": mission_code(task, extra),
                        "t4": t4_tier(extra.get("t4")),     # v0.1.71 (WP27): None | TIER_NONE | TIER_CORE | TIER_ADVANCED
                        "lucky": bool(extra.get("lucky")),   # v0.1.90: uncertain-colour pairs as gates (T1)
                        "mods": CA.mod_args(extra)[1] if task else {}}   # v0.1.92: the mission modifiers set (the rest = the boat's profile)
        log.info("mission preselected: %s %s auto=%s", task, extra or "", auto)
        if task and auto and self.lifecycle.state == "RUNNING":
            self._mission_trigger("mission preselected during a run")        # RunStart already came: start now (or when AUTO_HOLD)
        return None

    def _mission_trigger(self, why: str) -> None:
        """RunStart verified: start the preselected mission by itself when armed with --auto (once per run), else just say so.
        The vehicle must be in AUTO_HOLD for START_TASK; until it is (the fake judge may start a run before enable_auto),
        the start waits on the next status that shows AUTO_HOLD."""
        t = self.mission.get("task")
        if not t:
            return
        if not self.mission.get("auto"):
            self.alert("mission_ready", {"task": t, "why": why, "action": "start_task to confirm (the mission is preselected)"})
            return
        if self.mission.get("started_run") == self.lifecycle.run_id:
            return
        vid = self.primary_vid
        if (self.last_status.get(vid) or {}).get("state") != "AUTO_HOLD":
            self.mission["armed_run"] = self.lifecycle.run_id
            self.alert("mission_waiting", {"task": t, "why": why, "vehicle_state": (self.last_status.get(vid) or {}).get("state"),
                                           "action": "enable_auto: the mission starts when the vehicle reaches AUTO_HOLD"})
            return
        margs = {k: self.mission[k] for k in ("tier", "sim", "lucky") if self.mission.get(k)}
        margs.update(self.mission.get("mods") or {})                     # v0.1.92: the mission modifiers (look_lock False travels)
        why = self.start_check(t, margs)                                  # v0.1.66: m1t2 needs the UAV
        if why:
            self.mission["started_run"] = self.lifecycle.run_id
            self.alert("mission_refused", {"task": t, "code": self.mission.get("code"), "why": why, "action": why})
            return
        block = DC.blocking([x for x in self.declaration_rows("start_task", t, margs) if x["scope"] == "mission"],
                            self.authority.declaration_policy)
        if block:                                       # v0.1.41: the auto-start obeys the same checks as a typed start_task
            self.mission["started_run"] = self.lifecycle.run_id
            self.alert("mission_refused", {"task": t, "code": self.mission.get("code"), "why": why, "action": DC.summary(block)})
            return
        self.mission["started_run"] = self.lifecycle.run_id
        self.mission["armed_run"] = None
        if self.countdown_s > 0:                        # v0.1.43 (§7d.4): 3-2-1 on the page first; cancel / hold-on-start stay live
            self.mission["countdown"] = {"t0": time.time(), "until": time.time() + self.countdown_s, "s": self.countdown_s,
                                         "hold_on_start": False, "why": why, "vid": vid, "margs": margs}
            self.alert("mission_countdown", {"task": t, "code": self.mission.get("code"), "s": self.countdown_s,
                                             "action": "cancel_auto / hold_on_start during the count"})
            return
        self._fire_mission(vid, t, margs, why)

    def _fire_mission(self, vid: str, t: str, margs: dict, why: str, hold: bool = False) -> None:
        self.authority.emit(vid, "run", "START_TASK", t, margs)
        if hold:                                        # HOLD on start: the run's task is set, the boat waits for resume
            self.authority.emit(vid, "run", "HOLD")
        self.alert("mission_started", {"task": t, "code": self.mission.get("code"), "why": why, "run_id": self.lifecycle.run_id,
                                       "hold": hold})

    def _countdown_tick(self) -> None:
        cd = self.mission.get("countdown")
        if cd and time.time() >= cd["until"]:
            self.mission["countdown"] = None
            self._fire_mission(cd["vid"], self.mission.get("task"), cd["margs"], cd["why"], cd.get("hold_on_start", False))

    def on_reporting(self, on: bool, which: Optional[str] = None) -> Optional[str]:
        """v0.1.53 (Dmitrii: 'start reporting'): heartbeats for every vehicle from now (handbook: at all times in the course area).
        v0.1.54 (Dmitrii, STOP & RESET): off = heartbeats OFF now, whatever R (declared or not) - the judge sees none until
        `report on` or a new declaration (its PREPARE acks) turns them on again - a RunStart of the declaration already made does
        not (`hb_hard_off`). Never fabricated: a vehicle without a fresh status still gets none.
        v0.1.64 (D6): `report on uav` / `report off uav` - heartbeats for the UAV without declaring it (nothing else changes)."""
        if which == "usv":                                              # v0.1.82: the boat's own switch (▶ HEARTBEAT USV)
            self.report_usv = on
            if on:
                self.hb_hard_off = False
                self.heartbeats.enabled = True
            self.heartbeats.vehicles = self.hb_vehicles()
            log.info("reporting for the USV %s", "ON" if on else "OFF")
            return None
        if which:
            if which != "uav" or not self.uav_vid:
                return f"report {'on' if on else 'off'} {which}: only `uav` / `usv` (the UAV must be in the profile)"
            self.report_uav = on
            self.uav_hb_off = not on                                     # v0.1.90.6: the per-vehicle toggle wins over `declare uav on`
            self.heartbeats.vehicles = self.hb_vehicles()
            log.info("reporting for the UAV %s", "ON" if on else "OFF")
            return None
        self.reporting = on
        self.hb_hard_off = not on
        self.heartbeats.enabled = bool(on)
        self.report_usv = bool(on) or self.usv_flow == "auto"           # v0.1.82: START / STOP REPORTING covers the USV too
        self.heartbeats.vehicles = self.hb_vehicles()
        log.info("reporting %s (heartbeats %s)", "ON" if on else "OFF", "on" if self.heartbeats.enabled else "off")   # a log line, not an alert
        return None

    def on_countdown(self, action: str) -> Optional[str]:
        cd = self.mission.get("countdown")
        if not cd:
            return f"{action}: no countdown now (it runs for {self.countdown_s:.0f} s after RunStart with a mission armed --auto)"
        if action == "CANCEL_AUTO":
            self.mission["countdown"] = None
            self.mission["auto"] = False
            self.alert("mission_auto_cancelled", {"task": self.mission.get("task"), "action": "start_task to start by hand"})
        else:
            cd["hold_on_start"] = True
        return None

    def _mission_body(self) -> dict:
        m = {k: v for k, v in self.mission.items() if k != "countdown"}
        cd = self.mission.get("countdown")
        m["countdown"] = ({"left_s": round(max(0.0, cd["until"] - time.time()), 1), "s": cd["s"], "hold_on_start": cd["hold_on_start"]}
                          if cd else None)
        m["countdown_s"] = self.countdown_s
        return m

    def state_body(self) -> dict:
        b = self.lifecycle.body()
        b.update({"judge": self.judge.name, "judge_connected": self.judge.connected, "bench_mode": self.authority.bench_mode, "test_mode": self.authority.test_mode, "auto_before_declare": getattr(self, "auto_before_declare", False), "tiers": (self.lifecycle.declaration or {}).get("tiers"),
                  "course_nic": self.course_nic, "vehicles": {v: {"state": s.get("state"), "robot_state": s.get("robot_state"),
                                                                 "current_task": s.get("current_task"), "task_name": s.get("task_name"), "task_state": s.get("task_state"),
                                                                 "hold_reason": s.get("hold_reason"),
                                                                 "age_s": round(self.heartbeats.age(v), 1)}
                                                             for v, s in sorted(self.last_status.items(),        # v0.1.64: profile order (USV1 first)
                                                                                key=lambda kv: (kv[0] == self.uav_vid, list(self.vehicles).index(kv[0]) if kv[0] in self.vehicles else 99))},
                  "uav": self.uav_state(), "usv_flow": self.usv_flow_state(), "foreign_declaration": self.foreign_declaration,
                  "heartbeats_sent": dict(self.heartbeats.sent), "reports_sent": self.reports.sent, "attempt": self.attempt,
                  "reporting": {"on": self.reporting, "heartbeats": bool(self.heartbeats.enabled),   # v0.1.53
                                "default_on": self.reporting_default,                                   # v0.1.59 (B1)
                                "hb": self.heartbeats.view(),                                           # v0.1.59 (B5): count · state · age · seq
                                "last_report": getattr(self.judge, "last_report", None),                # v0.1.59 (B5): kind · seq (not heartbeats)
                                "off": bool(getattr(self, "hb_hard_off", False))},                     # v0.1.54: report off holds
                  "msg_rate": getattr(getattr(self.judge, "cap", None), "rate", 0) if hasattr(self.judge, "cap") else 0,
                  "rate_dropped": getattr(getattr(self.judge, "cap", None), "dropped", 0) if hasattr(self.judge, "cap") else 0,
                  "stalled": self.lifecycle.stalled, "mission": self._mission_body(), "judge_view": self.judge_view(),
                  "internal": self.internal, "t": utc_iso(),
                  "session": self._session_view(),                     # v0.1.69 (WP25)
                  "declaration_preview": self._preview_safe(),          # v0.1.70 (WP26): the constructor's "as it goes on the wire"
                  "declaration_sent": self.lifecycle.declaration})      # v0.1.70: the last one sent, verbatim (the wire feed)
        with self._ack_lock:                                    # v0.1.43 (OCS_V2_SPEC §6): TIMED OUT commands
            b["timed_out"] = [dict(r) for r in self.timed_out]
            b["timed_out_total"] = self.timed_out_total
            b["ack_timeout_s"] = self.ack_timeout_s
        try:
            b["declaration_checks"] = DC.state_block(self.effective_tiers(), self._declared(), self.authority.declaration_policy,
                                                     self.declaration_mandatory, self.declaration_rows("state"),
                                                     self.declaration_mandatory_hard)
        except Exception:
            log.exception("declaration checks")
        return b

    def _preview_safe(self) -> Optional[dict]:
        try:
            return self.declaration_preview()
        except Exception:
            log.exception("declaration preview")
            return None

    def _session_view(self) -> Optional[dict]:
        if self.session_state is None:
            return None
        s = self.session_state
        return {"host_age_s": round(time.monotonic() - self.session_t, 1), "current": s.get("current"),
                "last": {k: (s.get("last") or {}).get(k) for k in ("name", "result", "files", "stopped")} if s.get("last") else None}

    # ---- v0.1.41 (WP9c): the RunDeclaration checks ------------------------------------------------------------------------
    def _declared(self) -> bool:
        return bool(self.lifecycle.declaration) and self.lifecycle.body()["r"] in DC.JUDGED_R

    def effective_tiers(self, declare_args: Optional[dict] = None) -> dict:
        """The tiers in force: the declaration's while declared (R3-R5), else what `declare` would send now (+ its args)."""
        if self._declared() and declare_args is None:
            return dict(self.lifecycle.declaration.get("tiers") or {})
        tiers = dict(self.declaration_cfg.get("tiers") or DC.DEFAULT_TIERS)
        tiers.update(self.launch_tiers())                                  # v0.1.86: the preset's row wins over the profile's
        tiers = self._t4_flag(self._t1_flag(tiers))                        # v0.1.90.2: the ARMED mission decides T1 (m1t1 CORE / m1t2
                                                                           # ADVANCED) and its --t4 - over the preset's row (17:10 mismatch)
        if declare_args and declare_args.get("tiers"):
            tiers.update(declare_args["tiers"])
        return tiers

    def _t1_flag(self, tiers: dict) -> dict:
        """v0.1.66 (D22b): the declaration's T1 tier follows the preselected mission - m1t2 -> TIER_ADVANCED, m1t1 -> TIER_CORE; nothing
        preselected (or another task) -> the profile's tier."""
        code = str(self.mission.get("code") or "")
        if code.startswith("m1t2") or (code.startswith("scout") and self.declare_uav):     # v0.1.79: scout follows `declare uav`
            tiers["T1"] = "TIER_ADVANCED"
        elif code.startswith("m1t1") or code.startswith("scout"):
            tiers["T1"] = "TIER_CORE"
        return tiers

    def _t4_flag(self, tiers: dict) -> dict:
        """v0.1.44 (OCS_V2_SPEC §5, ROADMAP §7d.3): T4 is a modifier - the armed mission's `t4` flag (on / off) sets the declared T4
        tier (on = TIER_CORE, off = TIER_NONE); no flag = the profile's tier."""
        t4 = self.mission.get("t4")
        if t4 is not None:                                              # v0.1.71: the tier itself (True / False from older callers)
            tiers["T4"] = t4 if isinstance(t4, str) else ("TIER_CORE" if t4 else "TIER_NONE")
        return tiers

    def declaration_rows(self, step: str, task: Optional[str] = None, args: Optional[dict] = None) -> List[dict]:
        """The rows for one step (declare | mission | start_task | goto | state). `declare` checks its tiers and the preselected
        mission; the other steps the mission they name (state: the preselected one)."""
        args = dict(args or {})
        if step in ("declare", "state"):
            tiers = self.effective_tiers(args if step == "declare" else None)
            task, tier, auto = self.mission.get("task"), self.mission.get("tier"), bool(self.mission.get("auto"))
            if str(task or "").upper() == "SCOUT":                     # v0.1.79: the scout's tier = the drone toggle
                tier = "advanced" if self.declare_uav else "core"
        else:
            tiers = self.effective_tiers()
            tier, auto = args.get("tier"), bool(args.get("auto"))
            if str(task or "").upper() == "SCOUT" and not tier:      # v0.1.79: the scout's tier = the drone toggle
                tier = "advanced" if self.declare_uav else "core"
            if step == "mission" and not self._declared():          # v0.1.66 (D22b): the next declaration's T1 follows THIS mission
                code = str(mission_code(task, {"tier": tier}) or "") if task else ""
                if code.startswith("m1t2") or (code.startswith("scout") and self.declare_uav):
                    tiers["T1"] = "TIER_ADVANCED"
                elif code.startswith("m1t1") or code.startswith("scout"):
                    tiers["T1"] = "TIER_CORE"
        rows = DC.evaluate(tiers, task, tier, r=self.lifecycle.body()["r"], step=step, test_mode=self.authority.test_mode,
                           auto=auto, mandatory=self.declaration_mandatory,
                           mandatory_hard=self.declaration_mandatory_hard)
        lt = self.launch_tiers()
        if lt:                                                             # v0.1.86: the preset answered "are you sure?" (soft rows only)
            rows = [x for x in rows if not (x["code"] == "mandatory_none" and x["severity"] == "soft" and
                                            all(lt.get(t) == "TIER_NONE" for t in self.declaration_mandatory if tiers.get(t) == "TIER_NONE"))]
        return rows

    def publish_state(self) -> None:
        self.bus.publish("ocs/state", "telemetry", self.state_body(), qos=0, timeout_s=0.0, vehicle_id="OCS")

    def alert(self, kind: str, data: dict) -> None:
        rec = {"event": "alert", "kind": kind, "t": utc_iso(), **data}
        self.alerts.append(rec)
        log.warning("ALERT %s %s", kind, data)
        try:
            self.bus.publish("ocs/alert", "event", rec, qos=1, timeout_s=0.0, vehicle_id="OCS")
        except Exception:
            pass

    def _rc_log(self, rec: dict) -> None:
        self.rc_log.append(rec)
        try:
            self.bus.publish("ocs/rc_log", "telemetry", rec, qos=0, timeout_s=0.0, vehicle_id="OCS")
        except Exception:
            pass

    def _publish_intent_result(self, r: IntentResult) -> None:
        self.bus.publish("ocs/intent_result", "telemetry", {"intent_id": r.intent_id, "result": r.result, "reason": r.reason,
                                                            "cmd_msg_id": r.cmd_msg_id}, qos=1, timeout_s=0.0, vehicle_id="OCS")

    def _on_transition(self, ev: TaskStateEvent) -> None:
        log.info("run: %s -> %s (%s)", ev.src, ev.dst, ev.reason)
        self.bus.publish("ocs/run_state", "event", ev.body(), qos=1, timeout_s=0.0, vehicle_id="OCS")

    # ---- judge side --------------------------------------------------------------------------
    def _on_judge_link(self, up: bool) -> None:
        self.lifecycle.on_link(up)

    COURSE_NET = {"ALPHA": "192.168.65.", "BRAVO": "192.168.66.", "CHARLIE": "192.168.67.", "DELTA": "192.168.68."}

    def course_checks(self, course: dict, local_addr: Optional[str] = None) -> List[str]:
        """v0.1.82 (WP39): the launcher's course (RC_COURSE) against the RxCourse's course_id and our own address toward RoboCommand
        (the course subnet, handbook 3.4.2) -> the CAUTION texts (each raised once per text)."""
        sel = (os.environ.get("RC_COURSE") or "").upper()
        if sel not in self.COURSE_NET:
            return []
        out = []
        cid = str(course.get("course_id") or "").upper()
        if cid and sel not in cid:
            out.append(f"course mismatch: selected {sel}, RxCourse says {course.get('course_id')}")
        if local_addr is None:
            local_addr = self._local_addr_toward(os.environ.get("RC_HOST"), int(os.environ.get("RC_PORT") or 1883))
        if local_addr and not local_addr.startswith(self.COURSE_NET[sel]):
            out.append(f"OCS address not in the course subnet: {local_addr}, {sel} is {self.COURSE_NET[sel]}0/24")
        seen = getattr(self, "_course_cautions", set())
        for t in out:
            if t not in seen:
                self.alert("course_check", {"level": "caution", "text": t, "action": "the launcher's judge dropdown / the course cable (DHCP)"})
                seen.add(t)
        self._course_cautions = seen
        return out

    @staticmethod
    def _local_addr_toward(host: Optional[str], port: int) -> Optional[str]:
        import socket
        if not host:
            return None
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as so:
                so.connect((host, port))                          # no packet: the routing table picks our address
                return so.getsockname()[0]
        except OSError:
            return None

    def on_course(self, course: dict) -> None:
        pb = course.pop("_pb", None)
        self.judge_seen["course"] = {"t": time.monotonic()}
        try:
            self.course_checks(course)                                  # v0.1.82 (WP39)
        except Exception:
            log.exception("course check failed")
        self.uav_course_in(course, pb)                                  # v0.1.64 (D12): every changed course to the Orin
        if self.lifecycle.on_course(course):
            log.info("course %s accepted (%d boundary points)", course.get("course_id"), len(course.get("boundary", [])))
            self.send_course_to_vehicles()

    COURSE_RESEND_S = 6.0          # > the 5 s ack timeout
    STATUS_GAP_S = 4.0             # status at 2 Hz: a longer gap = the link or the mission manager restarted
    STATUS_FRESH_S = 3.0

    def _course_fp(self) -> Optional[str]:
        c = self.lifecycle.course
        if not c:
            return None
        return json.dumps([c.get("course_id"), c.get("boundary", []), c.get("uav_transit_area", []), c.get("pinger_hz")], sort_keys=True)

    def _course_sent(self, vid: str, msg_id: Optional[str], why: str) -> None:
        s = self._course_sync.setdefault(vid, {"acked": None, "refused": None})
        s.update(msg_id=msg_id, fp_sent=self._course_fp(), t_sent=time.monotonic())
        if why:
            log.info("course %s -> %s again (%s)", (self.lifecycle.course or {}).get("course_id"), vid, why)

    def _course_ack(self, b: dict) -> None:
        for vid, s in self._course_sync.items():
            if s.get("msg_id") and b.get("ack_of") == s["msg_id"]:
                if b.get("result") in ("ACCEPTED", "DONE"):
                    s["acked"] = s.get("fp_sent")
                elif b.get("result") in ("REJECTED", "FAILED"):
                    s["refused"] = s.get("fp_sent")
                    log.warning("course refused by %s: %s - not re-sent (a new course or a reconnect asks again)", vid, b.get("reason", ""))

    def course_resend_tick(self, now: Optional[float] = None) -> List[str]:
        """Re-send COURSE to each boat that is online and has not acknowledged the current course. Returns the vehicles asked."""
        now = time.monotonic() if now is None else now
        fp, asked = self._course_fp(), []
        if fp is None:
            return asked
        for vid in self.vehicles:
            if vid == self.uav_vid:
                continue
            seen = self._status_mono.get(vid)
            if seen is None or now - seen > self.STATUS_FRESH_S:
                continue                                                # offline: wait for its status
            s = self._course_sync.setdefault(vid, {"acked": None, "refused": None})
            if s.get("acked") == fp or s.get("refused") == fp or now - s.get("t_sent", -1e9) < self.COURSE_RESEND_S:
                continue
            self._emit_course(vid, "not acknowledged by the boat")
            asked.append(vid)
        return asked

    def _emit_course(self, vid: str, why: str = "") -> None:
        c = self.lifecycle.course or {}
        args = {"course_id": c.get("course_id"), "boundary": c.get("boundary", []), "uav_transit_area": c.get("uav_transit_area", []),
                "pinger_hz": c.get("pinger_hz"), "uav_geofence": self.declaration_cfg.get("uav_geofence", []),
                "ceiling_hae_m": self.declaration_cfg.get("uav_geofence_max_altitude_hae_m")}
        self._course_sent(vid, self.authority.emit(vid, "course", "COURSE", None, args), why)

    def send_course_to_vehicles(self) -> None:
        c = self.lifecycle.course or {}
        args = {"course_id": c.get("course_id"), "boundary": c.get("boundary", []), "uav_transit_area": c.get("uav_transit_area", []),
                "pinger_hz": c.get("pinger_hz"), "uav_geofence": self.declaration_cfg.get("uav_geofence", []),
                "ceiling_hae_m": self.declaration_cfg.get("uav_geofence_max_altitude_hae_m")}
        for vid in self.vehicles:
            if vid != self.uav_vid:                                     # v0.1.64: the UAV's course is the RxCourse itself (cmd/uav/rc)
                self._course_sent(vid, self.authority.emit(vid, "course", "COURSE", None, args), "")

    def on_rc_command(self, cmd: dict) -> None:
        pb = cmd.pop("_pb", None)
        kind = cmd.get("kind")
        seq = cmd.get("command_seq")
        # v0.1.23: a duplicate is the same seq AND kind AND send time (a QoS-1 redelivery); seq alone is not unique - found live
        # against rc-test: its server and RoboNation's test_client both number from 1, and a restarted RoboCommand starts over
        key = (seq, kind, round(float(cmd["sent_at"]), 3) if cmd.get("sent_at") is not None else None)
        if seq is not None and key in self._cmd_seq_seen:
            return
        if seq is not None:
            self._cmd_seq_seen.add(key)
        rec = {"t": time.monotonic(), "command_seq": seq, "origin": cmd.get("origin", "judge")}
        if kind == "run_start":
            rec.update(run_id=cmd.get("run_id"), declaration_seq=cmd.get("declaration_seq"), ours=self.lifecycle.declaration_seq)
            ok = self.lifecycle.on_run_start(cmd)
            rec["accepted"] = ok
            self.judge_seen["run_start"] = rec
            if ok:
                self.judge.new_run()
                self.heartbeats.enabled = not getattr(self, "hb_hard_off", False)     # v0.1.54: `report off` holds until on / a declaration
                for vid in self.run_vehicles():
                    args = {"run_id": self.lifecycle.run_id}
                    if vid == self.uav_vid:                             # D11: the UAV's current_task after the forward is acked
                        args["uav_task"] = "1" if self.mission.get("code") == "m1t2" else None
                    self.authority.emit(vid, "run", "START_RUN", None, args)
                if self.uav_declared():                                  # RunStart flies the drone (his invariant 16): only a run that declared it
                    self.uav_forward("command", cmd, pb)
                self._mission_trigger("RunStart verified")
            return                                                       # a RunStart we refused: the Orin never hears it
        if kind and kind != "unknown":
            self.uav_forward("command", cmd, pb)                         # v0.1.64 (D12): every RxCommand to the Orin (his bridge's rule)
        pos = cmd.get("position") or {}
        rec.update(domain=cmd.get("domain"), lat=pos.get("lat"), lon=pos.get("lon"),
                   polygon_points=len(cmd.get("polygon") or []) if cmd.get("polygon") else None,
                   heading_deg=cmd.get("heading_deg"), speed_mps=cmd.get("speed_mps"), standoff_m=cmd.get("standoff_m") or cmd.get("distance_m"))
        self.judge_seen[kind or "unknown"] = {k: v for k, v in rec.items() if v is not None}
        self.judge_seen["last_command"] = {"t": rec["t"], "kind": kind, "command_seq": seq}
        if kind in ("assistance_request", "keep_out_zone", "all_clear", "moving_object"):
            domain = cmd.get("domain", "surface")
            targets = [vid for vid, d in self.domain_of.items() if domain in ("all", d) and vid != self.uav_vid]   # the UAV: the forward
            for vid in targets:
                self.authority.emit(vid, "incident", "INCIDENT", None, {k: v for k, v in cmd.items() if k not in ("team_id",)})
            return
        if kind == "readiness_confirm":
            vid = cmd.get("vehicle_id") or self.pending_confirm.get(cmd.get("command_seq")) or self.primary_vid
            if vid != self.uav_vid:                                      # the UAV's ReadinessConfirm: forwarded (task4_mission resumes on it)
                self.authority.emit(vid, "incident", "INCIDENT", None, {"kind": "readiness_confirm", "command_seq": cmd.get("command_seq")})
            return
        log.info("rc command %s ignored", kind)

    # ---- team side -------------------------------------------------------------------------------
    def on_status(self, topic: str, env: dict) -> None:
        vid = env.get("vehicle_id")
        body = env.get("body", {})
        self.last_status[vid] = body
        mono = time.monotonic()                                         # v0.1.78.8: a status gap = the boat may have lost the course
        prev = self._status_mono.get(vid)
        cs = self._course_sync.get(vid)
        if cs is not None and prev is not None and mono - prev > self.STATUS_GAP_S:
            cs["acked"] = cs["refused"] = None                          # a gap: it may have lost it - ask at once
            cs["t_sent"] = -1e9
        elif cs is not None and prev is None and cs.get("acked") is None:
            cs["t_sent"] = -1e9                                         # its first status: what was sent before it was up is lost - at once
        if vid is not None:
            self._status_mono[vid] = mono
        self.heartbeats.update(vid, body)
        if vid is not None and vid == self.uav_vid and body.get("state") != "OFFLINE":
            self._uav_course_sync(body)
        self.lifecycle.vehicle_status(vid, body.get("robot_state", "STATE_UNKNOWN"))
        self.authority.last_vehicle_state[vid] = body.get("state", "?")
        if (self.mission.get("armed_run") is not None and self.mission["armed_run"] == self.lifecycle.run_id
                and body.get("state") == "AUTO_HOLD" and self.lifecycle.state == "RUNNING"):
            self._mission_trigger("vehicle in AUTO_HOLD after RunStart")
        # a vehicle that forgot the run while the OCS still runs it (a reset typed on the boat): say so once per run
        run = body.get("run") or {}
        if run.get("run_id") is not None:
            self._vehicle_run_seen[vid] = run.get("run_id")
        elif (self.lifecycle.state == "RUNNING" and self.lifecycle.run_id is not None and vid != self.uav_vid   # v0.1.65: the UAV's
              and self._vehicle_run_seen.get(vid) == self.lifecycle.run_id):           # adapter going OFFLINE is no boat reset (its own rows say it)
            key = (vid, self.lifecycle.run_id)
            if key not in self._run_dropped_alerted:
                self._run_dropped_alerted.add(key)
                self.alert("vehicle_run_dropped", {"vehicle": vid, "run_id": self.lifecycle.run_id,
                                                   "action": "the vehicle was reset on the boat: ocs_cli reset (both ends) or end_run"})

    # ---- v0.1.43 (OCS_V2_SPEC §6): TIMED OUT ------------------------------------------------------------------------
    TIMED_OUT_KEEP = 10

    def sweep_acks(self, now: Optional[float] = None) -> List[dict]:
        """Every cmd the authority emitted that has no ack after ack_timeout_s becomes a TIMED OUT record (once). Returns the new ones."""
        now = time.time() if now is None else now
        new: List[dict] = []
        if self.ack_timeout_s <= 0:
            return new
        with self._ack_lock:
            sent = self.authority.sent
            if self._sent_scanned > len(sent):              # the list was replaced: scan it again (acked ids are remembered)
                self._sent_scanned = 0
            for e in sent[self._sent_scanned:]:
                if e.get("msg_id") and e["msg_id"] not in self._acked_ids:
                    self._await_ack[e["msg_id"]] = e
            self._sent_scanned = len(sent)
            for mid, e in list(self._await_ack.items()):
                if now - float(e.get("t", now)) < self.ack_timeout_s:
                    continue
                del self._await_ack[mid]
                action = e.get("action") or "?"
                rec = {"msg_id": mid, "intent_id": e.get("intent_id"), "vehicle": e.get("vehicle"), "kind": e.get("kind"),
                       "action": action, "task": e.get("task"), "sent_at": utc_iso(float(e.get("t", now))),
                       "sent_unix": round(float(e.get("t", now)), 2), "t": utc_iso(now),
                       "t_unix": round(now, 2), "timeout_s": self.ack_timeout_s, "late_ack": None,
                       "text": f"command {action} {mid[:8]} TIMED OUT (no ack in {self.ack_timeout_s:g} s): check the link; repeat deliberately"}
                self.timed_out.append(rec)
                self.timed_out = self.timed_out[-self.TIMED_OUT_KEEP:]
                self._timed_out_by_id[mid] = rec
                while len(self._timed_out_by_id) > 200:
                    self._timed_out_by_id.pop(next(iter(self._timed_out_by_id)))
                self.timed_out_total += 1
                new.append(rec)
        for rec in new:
            log.warning("TIMED OUT: %s (cmd %s to %s, intent %s)", rec["text"], rec["msg_id"], rec["vehicle"], rec["intent_id"])
        return new

    def _ack_seen(self, b: dict, now: Optional[float] = None) -> None:
        """An ack clears its cmd's wait; one for a cmd already TIMED OUT is logged as late and noted on the record (nothing silent)."""
        mid = b.get("ack_of")
        if not mid:
            return
        now = time.time() if now is None else now
        late = None
        with self._ack_lock:
            self._await_ack.pop(mid, None)
            self._acked_ids[mid] = None
            while len(self._acked_ids) > 2000:
                self._acked_ids.popitem(last=False)
            rec = self._timed_out_by_id.get(mid)
            if rec is not None and rec.get("late_ack") is None:
                after = round(now - float(rec["sent_unix"]), 1)
                rec["late_ack"] = {"result": b.get("result"), "reason": b.get("reason") or "", "after_s": after, "t": utc_iso(now)}
                rec["text"] += f" · late ack {b.get('result')} after {after:g} s"
                late = rec
        if late is not None:
            log.warning("LATE ack %s for TIMED OUT %s %s (%s)", b.get("result"), late["action"], mid, late["late_ack"])

    def on_ack(self, topic: str, env: dict) -> None:
        self.acks.append(env)
        b = env["body"]
        self._uav_fwd_ack(b)
        self._course_ack(b)                                             # v0.1.78.8
        log.info("ack %s %s %s (%s)", env.get("vehicle_id"), b.get("result"), b.get("state"), b.get("reason", ""))
        self._ack_seen(b)
        vid = self._prepare_msg_ids.get(b.get("ack_of"))
        if vid and b.get("result") in ("DONE", "REJECTED", "FAILED"):
            if self.lifecycle.on_prepare_ack(vid, b.get("result"), b.get("reason", "")):
                self._prepare_msg_ids = {}
                self.judge.publish_request(self.lifecycle.declaration)      # every vehicle prepared: now the judge hears it
                self._save_declaration(self.lifecycle.declaration)
                cid = (self.lifecycle.course or {}).get("course_id")          # v0.1.59 (B2): the n-th declaration in this course slot
                self.attempt = (self.attempt + 1) if cid == self._attempt_course else 1
                self._attempt_course = cid
                self.hb_hard_off = False                                     # v0.1.54: a new declaration ends `report off`
                self.heartbeats.enabled = True                               # R3+: heartbeats start with the declaration (handbook 5.3 item 8)
                log.info("R3: declaration %s published after PREPARE acks", self.lifecycle.declaration_seq)

    def on_event(self, topic: str, env: dict) -> None:
        vid = env.get("vehicle_id")
        body = env.get("body", {})
        if vid is not None and vid == self.uav_vid and body.get("event") == "fence_change":
            self.on_uav_fence_change(body)
            return
        if body.get("event") == "incident" and body.get("kind") == "readiness":
            self.pending_confirm[body.get("command_seq")] = vid
        rec = internal_event(body)
        if rec is not None:                                     # v0.1.27: ours, never a judge report
            self.internal.setdefault(vid, {})[rec["kind"]] = {**rec, "t": utc_iso()}
            log.info("internal %s from %s: %s", rec["kind"], vid, rec)
            return
        self.reports.on_event(vid, body)

    def on_intent(self, topic: str, env: dict) -> None:
        self.authority.on_intent(topic, env)

    # ---- operator actions ---------------------------------------------------------------------
    def declare(self, args: Optional[dict] = None) -> Optional[str]:
        """The operator's declare: every vehicle must have been heard (status younger than status_fresh_s), PREPARE goes to
        each, and the declaration reaches the judge (R3) only when every PREPARE is acked DONE (on_ack). Returns a refusal
        reason, or None when PREPARE went out."""
        vehicles = self.decl_vehicles()                                 # v0.1.64 (D6): computed per run - USV1, + the UAV when declared
        ghosts = [v for v in vehicles if (self.last_status.get(v) or {}).get("nav") == "ghost"]
        if ghosts and (self.course_nic or getattr(self.judge, "network", "") == "course"):   # v0.1.68 (WP17 F6): never a free run judged
            return f"free run boat on the course judge ({', '.join(ghosts)}: nav ghost) - restart it without --free-run"
        if self.mission.get("code") == "m1t2" and self.uav_vid and self.uav_vid not in vehicles:
            return "m1t2 needs declare uav on (T1 Advanced: the UAV maps the buoys)"
        if self.usv_flow == "button":                                   # v0.1.82: ▶ HEARTBEAT USV → PILOT READY → DECLARE
            if not self.report_usv:
                return f"{self.primary_vid}: press ▶ HEARTBEAT USV first (report on usv) - RoboCommand must see its heartbeat"
            if not self.usv_ready:
                return f"{self.primary_vid}: PILOT READY not pressed (usv ready on)"
        stale = [(v, self.heartbeats.age(v)) for v in vehicles if self.heartbeats.age(v) > self.status_fresh_s]
        if stale:
            return "; ".join(f"{v} not heard ({'never' if a == float('inf') else '%.0f s ago' % a}): "
                             + ("the UAV adapter up (launcher UAV on) and the Orin's listener sending?" if v == self.uav_vid
                                else "mission manager up and bridged?") for v, a in stale)
        why, tiers, fence, extra, fence_bad = self._declaration_inputs(vehicles, args)
        if why:
            self.uav_view["last_refusal"] = {"t": utc_iso(), "reason": why}
            self.alert("declaration_refused_uav", {"vehicle": self.uav_vid, "reason": why,
                                                   "action": "see the UAV card; fix it with the pilot, declare again"})
            if fence_bad:                                               # D9: the listener clears the fence (disarmed only)
                self.authority.emit(self.uav_vid, "run", "FENCE_REJECTED", None, {"reason": why})
            return f"UAV: {why}"
        decl = self.lifecycle.declare(list(vehicles), tiers, fence, extra, require_prepare_ack=True)
        self._prepare_msg_ids = {}
        for vid in vehicles:
            msg_id = self.authority.emit(vid, "run", "PREPARE", None, {"declaration_seq": decl["declaration_seq"], "tiers": tiers,
                                                                      "course": self.lifecycle.course or {}})
            self._prepare_msg_ids[msg_id] = vid
        return None

    # ---- v0.1.69 (WP25): the sessions ------------------------------------------------------------------------------------------------
    SESSION_HOST_S = 6.0

    def _on_session_state(self, topic: str, payload: bytes) -> None:
        try:
            b = json.loads(payload.decode("utf-8")) if payload else None
        except Exception:
            return
        if isinstance(b, dict):
            self.session_state, self.session_t = b, time.monotonic()

    def on_session(self, op: str, args: dict) -> Tuple[bool, str]:
        """The authority accepted the word: refuse what the page server would refuse, else hand it over on ocs/session/cmd."""
        from . import sessions as SS
        if op not in ("start", "stop", "result"):
            return False, "session start NAME [options] | session stop | session result"
        if time.monotonic() - self.session_t > self.SESSION_HOST_S:
            return False, ("no session host: the OCS page server (ocs_web) runs the sessions - it is not publishing ocs/session "
                           "(run_ocs.sh / run_sim.sh start it; the page at :8096)")
        cur = (self.session_state or {}).get("current")
        name = str(args.get("name") or "")
        if op == "start":
            opts, why = SS.parse_opts(name, args.get("opts") or {})
            if why:
                return False, why
            if cur:
                return False, f"session {cur.get('name')} is running: session stop first"
            body = {"op": "start", "name": name, "opts": opts}
        elif op == "stop":
            if not cur:
                return False, "no session running"
            body = {"op": "stop"}
        else:
            s = cur or (self.session_state or {}).get("last")
            return (True, SS.result_line(s)) if s else (False, "no session yet")
        self.bus.publish_raw("ocs/session/cmd", json.dumps(dict(body, t=utc_iso())), qos=1)
        return True, (f"session {name} starting on the page server - its METRICS on the MISSION tab" if op == "start"
                      else f"session {cur.get('name')} stopping - the result file follows (session result)")

    def _declaration_inputs(self, vehicles: List[str], args: Optional[dict] = None):
        """v0.1.70 (WP26): what a declaration carries now - (UAV refusal or None, tiers, the UAV geofence, extra, fence_bad). No side
        effect: declare() sends it, declaration_preview() renders it (one source for both)."""
        d = self.declaration_cfg
        tiers = dict(d.get("tiers") or DC.DEFAULT_TIERS)
        tiers.update(self.launch_tiers())                               # v0.1.88: the preset's row (+ the T4 tick box) reaches the wire
        tiers = self._t4_flag(self._t1_flag(tiers))                     # v0.1.90.2: then the armed mission's T1 / T4 - the same order as effective_tiers
        if args and args.get("tiers"):
            tiers.update(args["tiers"])
        extra = {"uav_geofence_max_altitude_hae_m": d.get("uav_geofence_max_altitude_hae_m"),
                 "uav_pilot_callsign": d.get("uav_pilot_callsign")}
        fence = d.get("uav_geofence", [])                               # the bench stand-in when no UAV is declared (as before)
        if self.uav_vid in vehicles:                                    # D8: the FCU's polygon + FENCE_ALT_MAX, never the profile's
            why, fence, ceiling, fence_bad = self.uav_declaration_content()
            if why:
                return why, tiers, fence, extra, fence_bad
            extra = {"uav_geofence_max_altitude_hae_m": ceiling,
                     "uav_pilot_callsign": self.uav_cfg.get("pilot_callsign") or d.get("uav_pilot_callsign")}
        return None, tiers, fence, extra, False

    def declaration_preview(self) -> dict:
        """v0.1.70 (WP26): the RunDeclaration the OCS would send NOW, rendered from the protobuf (rc_wire.encode_request, decoded back
        and as protobuf text) - the RUN tab's constructor shows it beside its inputs; after `declare` the judge receives these fields."""
        vehicles = self.decl_vehicles()
        why, tiers, fence, extra, _bad = self._declaration_inputs(vehicles)
        body = {"kind": "run_declaration", "declaration_seq": self.lifecycle.declaration_seq + 1, "vehicle_ids": list(vehicles),
                "tiers": dict(tiers), "uav_geofence": list(fence), **(extra or {})}
        out = {"vehicles": list(vehicles), "uav_declared": self.uav_vid in vehicles if self.uav_vid else False, "uav_vid": self.uav_vid,
               "tiers": dict(tiers), "uav_refusal": why, "body": body, "fields": None, "text": None, "bytes": None}
        try:
            from . import rc_wire
            from google.protobuf import text_format
            raw = rc_wire.encode_request(self.team_id, 0, body, t=0.0)
            dec = rc_wire.decode_request(raw)
            out["fields"] = {k: dec[k] for k in ("vehicle_ids", "tiers", "uav_geofence", "uav_geofence_max_altitude_hae_m", "uav_pilot_callsign")}
            out["text"] = text_format.MessageToString(rc_wire.REQ.RxRequest.FromString(raw).run_declaration)
            out["bytes"] = len(raw)
        except Exception as exc:                                        # no protobuf on this host: the body alone
            out["why_no_wire"] = f"{type(exc).__name__}: {exc}"
        return out

    def end_run(self) -> None:
        self.uav_view["scan_done"] = None                                # v0.1.66: per attempt
        for vid in self.cmd_vehicles():
            self.authority.emit(vid, "run", "STOP_RUN", None, {})
        self.lifecycle.end()
        self.heartbeats.enabled = self.reporting or (self.usv_flow == "button" and self.report_usv)   # v0.1.53 (v0.1.82: the USV button too)
        if self.usv_flow == "button":
            self.usv_ready = False                                       # v0.1.82: per attempt, as the UAV's

    def reset_run(self) -> Optional[str]:
        """Operator reset: RESET to every vehicle (abort, manual, run context dropped), the OCS back to R2 (or R1 without a course)."""
        self.uav_view["scan_done"] = None                                # v0.1.66: per attempt
        for vid in self.cmd_vehicles():
            self.authority.emit(vid, "run", "RESET", None, {})
        self._prepare_msg_ids = {}
        self.heartbeats.enabled = self.reporting or (self.usv_flow == "button" and self.report_usv)   # v0.1.53 (v0.1.82)
        if self.usv_flow == "button":
            self.usv_ready = False                                       # v0.1.82
        self.pending_confirm.clear()
        if not self.lifecycle.reset():
            return f"nothing to reset in {self.lifecycle.body()['r']} {self.lifecycle.state}"
        return None

    # ---- v0.1.64 (WP14): the UAV, behind uav_vid and `declare uav` ------------------------------------------------------------
    # The UAV team's robocommand_bridge.py, its RoboCommand half, in the team's one client (MERGE_GUIDE §6.1 option B): the declaration
    # list per run (D6-D7), the FCU's fence + ceiling in the declaration with his geofence_problems() first (D8), fence_rejected and the
    # re-declaration on a fence / ceiling change before RunStart only (D9), every changed RxCourse and every RxCommand to the Orin through
    # the adapter (D12), the auto-ack of UAV-targeted Task 4 commands once the Orin has them, the UAV's reports (D13). Nothing here sends
    # anything to the drone but RoboCommand's own traffic and his protocol messages (the adapter speaks them).
    def uav_ready_tick(self) -> None:
        """v0.1.90.5 (Dmitrii at the venue: "remove pilot ready buttons, they confuse during the run"): the UAV's pilot ready by itself -
        declare uav on, its adapter up, no run declared (R1 / R2 / R6: before DECLARE, and again after a STOP & RESET cleared it), not
        ready yet -> UAV_READY on, at most every 5 s. `uav_pilot_auto: false` in the OCS profile = the button again."""
        if not (self.uav_vid and self.declare_uav and self.uav_auto_ready):
            return
        now = time.monotonic()
        if now - self._uav_ready_t < 5.0:
            return
        st = self.last_status.get(self.uav_vid) or {}
        if (st.get("health") or {}).get("pilot_ready") or self.lifecycle.body()["r"] not in ("R1", "R2", "R6") or not self.uav_active():
            return
        self._uav_ready_t = now
        ok, txt = self.on_uav("ready", {"on": True})
        log.info("UAV pilot ready by itself: %s", txt if ok else "refused - " + txt)

    def uav_active(self) -> bool:
        """The adapter is up: a UAV status younger than its stale cut, not its LWT."""
        if not self.uav_vid:
            return False
        st = self.last_status.get(self.uav_vid) or {}
        return st.get("state") not in (None, "OFFLINE") and self.heartbeats.age(self.uav_vid) <= self.uav_stale_s

    def decl_vehicles(self) -> List[str]:
        """D6: who the next declaration carries - every surface vehicle; the UAV only with `declare uav on`."""
        out = [v for v in self.vehicles if v != self.uav_vid]
        if self.uav_vid and self.declare_uav:
            out.append(self.uav_vid)
        return out

    def uav_declared(self) -> bool:
        """The current declaration (R3-R5) carries the UAV."""
        d = self.lifecycle.declaration or {}
        return bool(self.uav_vid) and self.uav_vid in (d.get("vehicle_ids") or []) and self.lifecycle.body()["r"] in DC.JUDGED_R

    def hb_vehicles(self) -> List[str]:
        """D6: heartbeats for the vehicles of the declaration list, or a UAV given `report on uav` explicitly; an undeclared UAV is
        silent on the wire."""
        out = [v for v in self.vehicles if v != self.uav_vid and (self.report_usv or self.usv_declared(v))]   # v0.1.82: the button
        if self.uav_vid and (self.report_uav or self.uav_declared() or (self.declare_uav and not getattr(self, "uav_hb_off", False))):
            out.append(self.uav_vid)                                    # v0.1.90.6: `report off uav` silences it until declared
        return out

    def usv_declared(self, vid: Optional[str] = None) -> bool:
        """v0.1.82: the current declaration (R3-R5) carries this surface vehicle (its heartbeats go on, the handbook's 'at all times')."""
        d = self.lifecycle.declaration or {}
        return (vid or self.primary_vid) in (d.get("vehicle_ids") or []) and self.lifecycle.body()["r"] in DC.JUDGED_R

    def on_usv_ready(self, on: bool) -> Tuple[bool, str]:
        """v0.1.82: usv ready on|off - the boat's PILOT READY (OCS-side)."""
        if on and not self.report_usv:
            return False, "PILOT READY needs the USV's heartbeat first: ▶ HEARTBEAT USV (report on usv)"
        self.usv_ready = on
        return True, ("USV pilot ready ON - DECLARE may carry the boat" if on else "USV pilot ready off - DECLARE refuses the boat")

    def usv_flow_state(self) -> dict:
        v = self.primary_vid
        return {"mode": self.usv_flow, "vehicle_id": v, "heartbeat": self.report_usv, "heartbeats_on": v in self.heartbeats.vehicles
                and self.heartbeats.enabled, "ready": self.usv_ready, "declared": self.usv_declared(v)}

    def run_vehicles(self) -> List[str]:
        """Who START_RUN / the run commands go to: the declaration's vehicles."""
        d = self.lifecycle.declaration or {}
        ids = [v for v in (d.get("vehicle_ids") or []) if v in self.vehicles]
        return ids or [v for v in self.vehicles if v != self.uav_vid]

    def cmd_vehicles(self) -> List[str]:
        """STOP_RUN / RESET: every surface vehicle, the UAV when its adapter is up (no TIMED OUT for a process that is not running)."""
        return [v for v in self.vehicles if v != self.uav_vid or self.uav_active()]

    def uav_declaration_content(self):
        """D8: (refusal or None, the FCU's closed polygon, the ceiling = FENCE_ALT_MAX, fence_bad) from status/uav. fence_bad = the refusal
        is the fence or the ceiling (-> fence_rejected, D9); not for staleness, armed, no polygon, no course."""
        st = self.last_status.get(self.uav_vid) or {}
        h, f = st.get("health") or {}, st.get("fence") or {}
        age = h.get("telemetry_age_s")
        if self.heartbeats.age(self.uav_vid) > self.uav_stale_s or age is None or float(age) > self.uav_stale_s:
            return (f"UAV telemetry stale ({'none' if age is None else '%.0f s' % float(age)}; cut {self.uav_stale_s:g} s): "
                    "the Orin's listener and the adapter up?"), [], None, False
        if st.get("armed"):
            return "the drone is ARMED - a declaration is made only disarmed, on the ground", [], None, False
        fence = [[float(p[0]), float(p[1])] for p in f.get("points") or []]
        if fence and fence[0] != fence[-1]:
            fence.append(list(fence[0]))
        if len(fence) < 4:
            return "the FCU has no single polygon fence to declare (Mission Planner: Fence -> Read / Write)", [], None, False
        c = self.lifecycle.course or {}
        corners = [(float(p[0]), float(p[1])) for p in c.get("boundary") or []]
        transit = [(float(p[0]), float(p[1])) for p in c.get("uav_transit_area") or []]
        if not corners:
            return "no course from RoboCommand to check the UAV geofence against", [], None, False
        ceiling = f.get("ceiling_hae_m")
        if ceiling is None:
            return "no FENCE_ALT_MAX from the drone yet (the ceiling)", [], None, False
        probs = GF.geofence_problems([tuple(p) for p in fence], corners, transit, float(ceiling), GF.MAX_CEILING_HAE_M)
        if probs:
            return "RoboCommand would REJECT it: " + "; ".join(probs), [], None, True
        return None, fence, float(ceiling), False

    def on_uav_fence_change(self, body: dict) -> None:
        """D9: a fence written in Mission Planner (or a new FENCE_ALT_MAX) -> the LAST declaration re-sent with the new polygon in R3-R4;
        after RunStart (R5-R6) not re-declared (A15 assumption), said; the result back to the Orin (fence_change_result)."""
        cid = body.get("id")
        r = self.lifecycle.body()["r"]
        fence = [[float(p[0]), float(p[1])] for p in body.get("vertices") or []]
        if fence and fence[0] != fence[-1]:
            fence.append(list(fence[0]))
        st = self.last_status.get(self.uav_vid) or {}
        ceiling = body.get("ceiling_hae_m")
        if ceiling is None:
            ceiling = (st.get("fence") or {}).get("ceiling_hae_m")
        what = "the new FENCE_ALT_MAX" if body.get("synthetic") else "the Mission Planner fence"
        if not self.uav_declared():
            # his bridge declares the UAV alone here (tiers none); ours does not - the team's declaration is the operator's `declare`
            # (decided unattended 2026-11-08, DEVLOG v0.1.64). The pilot hears whether the next declare would take this fence.
            c = self.lifecycle.course or {}
            probs = GF.geofence_problems([tuple(p) for p in fence], [tuple(p) for p in c.get("boundary") or []],
                                         [tuple(p) for p in c.get("uav_transit_area") or []],
                                         float(ceiling) if ceiling is not None else 0.0, GF.MAX_CEILING_HAE_M) if c.get("boundary") else []
            ok = not probs
            text = (f"{what}: no declaration with the UAV yet - the next declare carries it" if ok else
                    f"{what}: no declaration with the UAV yet, and RoboCommand would REJECT this one ({'; '.join(probs)})")
        elif r in ("R5", "R6"):
            ok, text = False, f"{what}: after RunStart - NOT re-declared (no re-declaration in a run); tell the judges if it matters"
            self.alert("uav_fence_changed_after_runstart", {"vehicle": self.uav_vid, "id": cid, "action": text})
        else:
            c = self.lifecycle.course or {}
            probs = GF.geofence_problems([tuple(p) for p in fence], [tuple(p) for p in c.get("boundary") or []],
                                         [tuple(p) for p in c.get("uav_transit_area") or []],
                                         float(ceiling) if ceiling is not None else 0.0, GF.MAX_CEILING_HAE_M)
            if probs:
                ok, text = False, f"{what}: RoboCommand would REJECT it ({'; '.join(probs)}) - NOT sent; the drone's fence is being cleared"
                self.authority.emit(self.uav_vid, "run", "FENCE_REJECTED", None, {"reason": "; ".join(probs)})
                self.alert("declaration_refused_uav", {"vehicle": self.uav_vid, "reason": "; ".join(probs), "action": "redraw in Mission Planner"})
            else:
                decl = dict(self.lifecycle.declaration, uav_geofence=fence, uav_geofence_max_altitude_hae_m=float(ceiling))
                self.lifecycle.declaration = decl
                seq = self.judge.publish_request(decl)
                self._save_declaration(decl)
                self.uav_view["redeclared"] = int(self.uav_view.get("redeclared") or 0) + 1
                ok = seq is not None
                text = (f"Run Request #{seq} re-sent for {what} ({len(fence)} points from the FCU, ceiling {float(ceiling):.1f} m HAE)"
                        if ok else f"{what}: the re-declaration was rate-capped - declare again")
                self.alert("uav_fence_redeclared", {"vehicle": self.uav_vid, "id": cid, "text": text})
        self.uav_view["last_fence_change"] = {"t": utc_iso(), "id": cid, "ok": ok, "text": text, "r": r}
        if self.uav_active():
            self.authority.emit(self.uav_vid, "run", "FENCE_CHANGE_RESULT", None, {"id": cid, "ok": ok, "text": text})

    # ---- D12: RoboCommand's traffic to the Orin ------------------------------------------------------------------------------
    def uav_course_in(self, course: dict, pb: Optional[dict]) -> None:
        if not self.uav_vid:
            return
        from ..uav.wire import course_fingerprint
        from . import rc_wire
        if pb is None:
            try:
                pb = rc_wire.pb_dict("course", rc_wire.encode_course(course))
            except Exception:
                log.exception("course -> protobuf dict")
                return
        fp = course_fingerprint(pb)
        if fp == self._uav_course_fp:
            return                                                     # his invariant 7: an unchanged course is not forwarded
        self._uav_course, self._uav_course_fp = pb, fp
        self.uav_view["course_fp"] = fp[:12]
        if self.uav_active():
            self._uav_course_send("course changed")

    def _uav_course_send(self, why: str) -> Optional[str]:
        if not self._uav_course or not self.uav_active():
            return None
        self._uav_course_sent_t = time.monotonic()
        mid = self.authority.emit(self.uav_vid, "rc", "RC_FORWARD", None, {"kind": "course", "message": self._uav_course})
        self._uav_fwd[mid] = {"kind": "course", "fp": self._uav_course_fp}
        self.uav_view["course_forward"] = {"t": utc_iso(), "fp": (self._uav_course_fp or "")[:12], "acked": None, "why": why}
        return mid

    def _uav_course_sync(self, st: dict) -> None:
        """A fresh adapter (started after the course came, or restarted) gets the cached course once (his invariant 17 decides on the Orin)."""
        if not self._uav_course or not self._uav_course_fp:
            return
        have = ((st.get("robocommand") or {}).get("course_fp") or "")
        if have != self._uav_course_fp[:12] and time.monotonic() - self._uav_course_sent_t > 16.0:
            self._uav_course_send("the adapter has no course")

    def uav_forward(self, kind: str, cmd: dict, pb: Optional[dict]) -> None:
        if not self.uav_active():
            return
        from ..uav.wire import ACKED_COMMANDS
        from . import rc_wire
        if pb is None:
            try:
                pb = rc_wire.pb_dict("command", rc_wire.encode_command(self.team_id, int(cmd.get("command_seq") or 0), cmd))
            except Exception:
                log.info("command %s not forwarded to the Orin (no protobuf form)", cmd.get("kind"))
                return
        which = next((k for k in ("run_start", "assistance_request", "keep_out_zone", "all_clear", "moving_object_alert", "readiness_confirm")
                      if k in pb), "command")
        target = pb.get("vehicle_id") or cmd.get("vehicle_id")
        for_uav = (target == self.uav_vid) if target else (cmd.get("domain") == "air")
        mid = self.authority.emit(self.uav_vid, "rc", "RC_FORWARD", None, {"kind": "command", "message": pb, "which": which})
        self._uav_fwd[mid] = {"kind": which, "seq": cmd.get("command_seq"), "auto_ack": which in ACKED_COMMANDS and for_uav}
        self.uav_view["last_command"] = {"t": utc_iso(), "which": which, "seq": cmd.get("command_seq"), "acked": None}

    def _uav_fwd_ack(self, b: dict) -> None:
        rec = self._uav_fwd.get(b.get("ack_of") or "")
        if rec is None or b.get("result") not in ("DONE", "FAILED", "REJECTED"):
            return
        self._uav_fwd.pop(b.get("ack_of"), None)
        ok = b.get("result") == "DONE"
        view = self.uav_view.get("course_forward" if rec["kind"] == "course" else "last_command") or {}
        view["acked"] = utc_iso() if ok else False
        if ok and rec.get("auto_ack"):                                 # his ACKED_COMMANDS: the IncidentAck once the Orin has it
            self.reports.on_event(self.uav_vid, {"event": "incident", "kind": "ack", "command_seq": rec.get("seq"), "what": rec["kind"]})
        if not ok:
            self.alert("uav_forward_failed", {"vehicle": self.uav_vid, "what": rec["kind"], "reason": b.get("reason", ""),
                                              "action": "the Orin did not take the " + ("course: uav resend_course" if rec["kind"] == "course"
                                                                                       else "command") + " - is the listener up (14600)?"})

    # ---- the words (ocs_cli / op-tui / the UAV window) ----------------------------------------------------------------------------
    def on_uav(self, op: str, args: dict):
        """(ok, text): declare uav on|off · uav ready on|off · uav task N|none · uav resend_course."""
        if not self.uav_vid:
            return False, "no UAV in this OCS profile (ocs.vehicles has no `uav` vehicle)"
        on = bool(args.get("on"))
        if op == "declare":
            self.declare_uav = on
            note = " (the current declaration is unchanged until the next declare)" if self._declared() else ""
            return True, (f"declare uav ON: the next declaration carries {self.uav_vid} (its FCU fence + FENCE_ALT_MAX, pilot "
                          f"{self.uav_cfg.get('pilot_callsign')}); heartbeats for it now" if on else
                          "declare uav OFF: USV-only declarations; the UAV is silent on the wire") + note
        if not self.uav_active():
            return False, f"the UAV adapter is not running (or {self.uav_vid}'s telemetry is stale): launcher UAV on, the Orin's listener up"
        if op == "ready":
            mid = self.authority.emit(self.uav_vid, "run", "UAV_READY", None, {"on": on})
            return True, ("pilot ready ON - the UAV's PREPARE can go DONE" if on else "pilot ready off") + f" ({mid[:8]})"
        if op == "task":
            t = str(args.get("task") or "none").lower()
            if t not in ("none", "1", "2", "3", "4"):
                return False, "uav task none | 1 | 2 | 3 | 4"
            self.authority.emit(self.uav_vid, "run", "UAV_TASK", None, {"task": t})
            return True, f"UAV current_task -> {t} (by hand; overrides the RunStart rule until `uav task none`)"
        if op == "scan_done":                                            # v0.1.66 (D22c): the boat leaves WAIT_UAV_MAP
            self.uav_view["scan_done"] = utc_iso()
            return True, "uav scan done: the boat leaves WAIT_UAV_MAP (m1t2) - it transits with the UAV map it has"
        if op == "resend_course":
            if not self._uav_course:
                return False, "no RxCourse received yet - wait for RoboCommand's course"
            self._uav_course_send("uav resend_course")
            return True, f"the cached course ({(self._uav_course_fp or '')[:12]}) re-sent to the Orin - its ack on the UAV card"
        return False, f"uav {op}: one of ready on|off, task N|none, resend_course"

    def uav_state(self) -> dict:
        d = self.lifecycle.declaration or {}
        decl = self.uav_declared()
        st = self.last_status.get(self.uav_vid) or {} if self.uav_vid else {}
        return {"vehicle_id": self.uav_vid, "configured": bool(self.uav_vid), "declare": self.declare_uav, "declared": decl,
                "active": self.uav_active(), "report_on": self.report_uav,
                "registration": self.uav_cfg.get("registration"), "pilot_callsign": self.uav_cfg.get("pilot_callsign"),
                "declared_fence": d.get("uav_geofence") if decl else None,
                "declared_ceiling_hae_m": d.get("uav_geofence_max_altitude_hae_m") if decl else None,
                "pilot_ready": bool((st.get("health") or {}).get("pilot_ready")),
                "telemetry_age_s": (st.get("health") or {}).get("telemetry_age_s"),
                "heartbeats_on": self.uav_vid in self.heartbeats.vehicles if self.uav_vid else False,
                "flight": {"armed": st.get("armed"), "flight_phase": st.get("flight_phase"), "mode": st.get("mode"),   # v0.1.89: + mode (RTL)
                           **((st.get("flown") or {}) if isinstance(st.get("flown"), dict) else {})},
                "course": {"boundary": (self.lifecycle.course or {}).get("boundary") or [],          # v0.1.65: the UAV window's map
                           "uav_transit_area": (self.lifecycle.course or {}).get("uav_transit_area") or []},
                **{k: v for k, v in self.uav_view.items()}}

    def start_check(self, task: Optional[str], args: Optional[dict]) -> Optional[str]:
        """v0.1.66 (D22b): START_TASK m1t2 (T1 tier advanced) needs the UAV - declared and its status fresh."""
        if task == "T1" and str((args or {}).get("tier") or "").lower() == "advanced":
            if not self.uav_vid or not self.uav_active():
                return "m1t2 needs the UAV: no fresh UAV status (the adapter, the Orin's listener)"
            if not self.uav_declared() and self.lifecycle.body()["r"] in DC.JUDGED_R:
                return "m1t2 needs the UAV in the declaration (declare uav on, then declare)"
        return None

    def on_foreign_declaration(self, rec: dict) -> None:
        """D18: another RoboCommand client declared for our team (the UAV team's robocommand_bridge.py left on): an ALARM - two owners
        give conflicting declarations (MERGE_GUIDE §6.1)."""
        self.foreign_declaration = rec
        self.alert("foreign_declaration", {"seq": rec.get("seq"), "vehicle_ids": rec.get("vehicle_ids"),
                                           "action": "another client declared for SGIT - stop the UAV team's robocommand_bridge.py (one declaration owner)"})

    def _save_declaration(self, decl: Optional[dict]) -> None:
        if not decl or not self.decl_file:
            return
        try:
            os.makedirs(os.path.dirname(self.decl_file) or ".", exist_ok=True)
            with open(self.decl_file + ".tmp", "w") as fh:
                json.dump({"team_id": self.team_id, "declaration": decl, "t": utc_iso()}, fh, indent=1)
            os.replace(self.decl_file + ".tmp", self.decl_file)
        except OSError as e:
            log.warning("last declaration not saved (%s)", e)

    def _rate_watch(self) -> None:
        """D2: the 5 msg/s cap with two vehicles - a dropped message is said loudly (ocs/alert; the card shows rate_dropped)."""
        cap = getattr(self.judge, "cap", None)
        n = max(int(getattr(cap, "dropped", 0) or 0), int((getattr(self.judge, "stats", {}) or {}).get("dropped", 0) or 0))
        if n > self._rate_dropped_seen:
            if time.monotonic() - self._rate_alert_t > 10.0:
                self._rate_alert_t = time.monotonic()
                self.alert("rate_dropped", {"dropped": n, "vehicles": self.hb_vehicles(),
                                            "action": "the 5 msg/s team cap dropped a message: fewer vehicles reporting, or report off"})
            self._rate_dropped_seen = n

    # ---- bench injectors (refused on the course network by the authority) --------------------------
    def inject_run_start(self, args: dict) -> str:
        self.on_rc_command({"kind": "run_start", "declaration_seq": self.lifecycle.declaration_seq,
                            "run_id": int(args.get("run_id", int(time.time()) % 100000)), "command_seq": -int(time.time() * 1000) % 1000000,
                            "origin": "bench"})
        return f"RunStart injected for declaration_seq {self.lifecycle.declaration_seq}"

    def inject_course(self, args: dict) -> str:
        c = args.get("course") or {"course_id": "BENCH-1", "boundary": [[1.2990, 103.7990], [1.2990, 103.8010], [1.3010, 103.8010], [1.3010, 103.7990]],
                                   "uav_transit_area": [], "pinger_hz": 30000}
        self.on_course(c)
        return f"course {c.get('course_id')} injected"

    def inject_assistance(self, args: dict) -> str:
        self.on_rc_command({"kind": "assistance_request", "command_seq": int(args.get("command_seq", int(time.time()) % 100000)),
                            "position": {"lat": float(args.get("lat", 1.3002)), "lon": float(args.get("lon", 103.8002))},
                            "domain": args.get("domain", "surface"), "origin": "bench"})
        return "AssistanceRequest injected"

    def inject_keep_out(self, args: dict) -> str:
        self.on_rc_command({"kind": "keep_out_zone", "command_seq": int(args.get("command_seq", int(time.time()) % 100000)),
                            "polygon": args.get("polygon", []), "domain": args.get("domain", "surface"), "origin": "bench"})
        return "KeepOutZone injected"

    def inject_all_clear(self, args: dict) -> str:
        self.on_rc_command({"kind": "all_clear", "command_seq": int(args.get("command_seq", int(time.time()) % 100000)),
                            "domain": args.get("domain", "surface"), "origin": "bench"})
        return "AllClear injected"

    def inject_moving_object(self, args: dict) -> str:
        self.on_rc_command({"kind": "moving_object", "command_seq": int(args.get("command_seq", int(time.time()) % 100000)),
                            "position": args.get("position"), "heading_deg": args.get("heading_deg"), "speed_mps": args.get("speed_mps"),
                            "keep_out_radius_m": args.get("keep_out_radius_m", 5.0), "domain": args.get("domain", "surface"), "origin": "bench"})
        return "MovingObjectAlert injected"

    def inject_readiness_confirm(self, args: dict) -> str:
        self.on_rc_command({"kind": "readiness_confirm", "command_seq": args.get("command_seq"), "vehicle_id": args.get("vehicle_id"),
                            "origin": "bench"})
        return "ReadinessConfirm injected"


def t4_tier(v) -> Optional[str]:
    """v0.1.71 (WP27): the mission's --t4 -> the declared T4 tier: off / false -> TIER_NONE, core / on / true -> TIER_CORE, advanced ->
    TIER_ADVANCED; nothing -> None (the profile's tier)."""
    if v in (None, ""):
        return None
    w = str(v).strip().lower()
    if w in ("advanced", "tier_advanced", "2"):
        return "TIER_ADVANCED"
    if w in ("off", "false", "0", "no", "none", "tier_none"):
        return "TIER_NONE"
    return "TIER_CORE"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="optiprimex OCS application")
    ap.add_argument("--config", required=True)
    ap.add_argument("--log-level", default="INFO")
    a = ap.parse_args(argv)
    logging.basicConfig(level=getattr(logging, a.log_level.upper()), format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = cfgmod.load(a.config)
    app = OcsApp(cfg)
    app.authority.geofence_file = a.config                              # v0.1.53: `fence apply geofence` reads the profile's geofence now
    app.start()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        while not stop.is_set():
            time.sleep(0.2)
    finally:
        app.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
