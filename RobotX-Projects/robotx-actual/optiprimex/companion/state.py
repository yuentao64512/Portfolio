"""Canonical data types (MODULES 2, ADR-4). Pure data, no I/O.

Angles are degrees in data, radians inside math. Heading is NED 0-360 clockwise.
Positions in messages are lat/lon, or local (x_n, y_e) with the control origin attached.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


def utc_iso(t: Optional[float] = None) -> str:
    t = time.time() if t is None else t
    s = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t))
    return f"{s}.{int((t % 1) * 1000):03d}Z"


class VState(str, Enum):
    """Layer 1: vehicle state (ARCHITECTURE 3.2). Value = name reported in status."""
    V0 = "BOOT"
    V1 = "KILLED"
    V2 = "MANUAL"
    V3 = "AUTO_HOLD"
    V4 = "AUTO_TASK"
    V5 = "AUTO_SAFE_HOLD"
    V6 = "FAULT"

    @property
    def robot_state(self) -> str:
        if self is VState.V1:
            return "STATE_KILLED"
        if self is VState.V2:
            return "STATE_MANUAL"
        if self in (VState.V3, VState.V4, VState.V5):
            return "STATE_AUTO"
        return "STATE_UNKNOWN"   # V0 / V6: the OCS keeps the last known state (decision 4)


class RobotXTask(str, Enum):
    NONE = "TASK_NONE"
    T1 = "TASK_SAFE_PASSAGE"
    T2 = "TASK_INFRASTRUCTURE_SURVEY"
    T3 = "TASK_COORDINATED_LOGISTICS"
    T4 = "TASK_DYNAMIC_INCIDENT"


TASK_TO_ROBOTX = {"T1": RobotXTask.T1, "T2": RobotXTask.T2, "T3": RobotXTask.T3, "T4": RobotXTask.T4,
                  "GOTO": RobotXTask.NONE}


@dataclass
class FcuState:
    """Last known FCU truth as observed on the MAVLink stream (companion.link)."""
    t_mono: float = 0.0
    t_utc: str = ""
    seen_heartbeat: bool = False
    vehicle_type: Optional[int] = None
    autopilot: Optional[int] = None
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt_m: Optional[float] = None
    hdg_deg: Optional[float] = None
    sog_mps: Optional[float] = None
    cog_deg: Optional[float] = None
    vx_n: float = 0.0
    vy_e: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: Optional[float] = None
    yaw_rate_dps: Optional[float] = None     # v0.1.51: ATTITUDE.yawspeed (None until an ATTITUDE came)
    mode: Optional[int] = None
    mode_name: Optional[str] = None
    armed: Optional[bool] = None
    system_status: Optional[int] = None
    ekf_flags: int = 0
    ekf_pos_ok: bool = False
    gps_fix: int = 0
    sats: int = 0
    hdop: float = 99.0
    batt_v: Optional[float] = None
    batt_pct: Optional[int] = None
    rc: Dict[int, int] = field(default_factory=dict)     # channel -> pwm
    rc_chancount: int = 0
    rc_last_plausible: bool = False      # v0.1.29.3: the LAST RC_CHANNELS had a channel 1-4 in 800..2200 us (rc keeps old values)
    rc_rx_present: Optional[bool] = None  # SYS_STATUS: the FCU reports an RC receiver (bit RC_RECEIVER present) - None = never seen
    rc_rx_healthy: Optional[bool] = None  # ... and healthy (ArduPilot clears it on an RC failsafe)
    rc_ok: bool = False
    rc_detail: str = ""                  # which signal decided rc_ok (status.health.rc_detail; the lab day reads it)
    servo: List[int] = field(default_factory=lambda: [0] * 8)
    kill: bool = False
    estop: bool = False
    ages: Dict[str, float] = field(default_factory=dict)  # msg type -> seconds since last
    statustext: List[str] = field(default_factory=list)
    params_ok: bool = False
    firmware_hash: Optional[str] = None
    lamp: Optional[int] = None               # v0.1.55 (D4): NAMED_VALUE_INT LAMP if the Lua ever sends it - 0 off · 1 red · 2 yellow · 3 green
    fence_sys_present: Optional[bool] = None   # v0.1.53: SYS_STATUS MAV_SYS_STATUS_GEOFENCE present / enabled / healthy (None = never seen)
    fence_sys_enabled: Optional[bool] = None
    fence_sys_healthy: Optional[bool] = None
    fence_breach_status: Optional[int] = None  # v0.1.53: FENCE_STATUS breach_status / breach_count / breach_type (None = never seen)
    fence_breach_count: Optional[int] = None
    fence_breach_type: Optional[int] = None

    def age(self, msg: str, now: Optional[float] = None) -> float:
        return self.ages.get(msg, float("inf"))

    @property
    def rc6_high(self) -> bool:
        return self.rc.get(6, 0) >= 1700

    @property
    def rc5_estop(self) -> bool:
        return self.rc.get(5, 0) >= 1700


@dataclass
class LocalPose:
    x_n: float
    y_e: float
    hdg_deg: float
    co_lat: float
    co_lon: float
    t: float            # monotonic
    t_utc: str = ""
    sog_mps: float = 0.0


RC_SYS_FRESH_S = 3.5                    # SYS_STATUS is 1 Hz: its RC bit counts while younger than this


def rc_health(s: "FcuState", rc_s: float) -> Tuple[bool, str]:
    """(rc_ok, why) - v0.1.29.3 (pool report F1: `RC_LOST` all day on a live transmitter; the real link sent RC_CHANNELS with
    chancount 0 while the channel values were good). RC_CHANNELS must be fresh (no channels = no RC5 / RC6 either), then:
    the FCU's own receiver-health bit (SYS_STATUS RC_RECEIVER, cleared by ArduPilot on an RC failsafe) when it reports one;
    else chancount > 0 or a plausible channel 1-4 in the latest message."""
    age = s.ages.get("RC_CHANNELS", float("inf"))
    age_txt = "never" if age == float("inf") else f"{age:.1f} s"
    head = f"RC_CHANNELS {age_txt}, chancount {s.rc_chancount}"
    if age > rc_s:
        return False, f"{head}: not fresh (> {rc_s:g} s)"
    if s.rc_rx_present and s.ages.get("SYS_STATUS", float("inf")) <= RC_SYS_FRESH_S:
        return bool(s.rc_rx_healthy), f"{head}; SYS_STATUS rc receiver {'healthy' if s.rc_rx_healthy else 'UNHEALTHY (failsafe)'}"
    ok = s.rc_chancount > 0 or s.rc_last_plausible
    return ok, f"{head}; channels {'plausible' if s.rc_last_plausible else 'none'} (no SYS_STATUS rc bit)"


@dataclass
class Health:
    fcu_age_s: float = float("inf")
    rc_ok: bool = False
    rc_detail: str = ""
    kill: bool = False
    estop: bool = False
    perception_age_s: float = float("inf")
    bridge_up: bool = False
    clock_ok: bool = True
    loop_slow: bool = False
    faults: List[str] = field(default_factory=list)
    tick_ms: Optional[dict] = None
    guard_ms: Optional[dict] = None    # v0.1.86.8 (WP43 §0): the guard pipeline {pipe_ms_p50, pipe_ms_p95, net_ms_p50, n}     # v0.1.86.4 (WP43 §0): the mission manager's tick duration {p50, p95, max, slow_per_min, n} over 60 s

    def body(self) -> dict:
        d = asdict(self)
        d["fcu_age_s"] = None if d["fcu_age_s"] == float("inf") else round(d["fcu_age_s"], 2)
        d["perception_age_s"] = None if d["perception_age_s"] == float("inf") else round(d["perception_age_s"], 2)
        return d


@dataclass
class Course:
    course_id: Optional[str] = None
    boundary: List[Tuple[float, float]] = field(default_factory=list)
    uav_transit_area: List[Tuple[float, float]] = field(default_factory=list)
    pinger_hz: Optional[float] = None
    uav_geofence: List[Tuple[float, float]] = field(default_factory=list)
    ceiling_hae_m: Optional[float] = None


@dataclass
class RunContext:
    run_id: Optional[int] = None
    declaration_seq: Optional[int] = None
    course: Course = field(default_factory=Course)
    tiers: Dict[str, str] = field(default_factory=dict)
    prepared_at: Optional[str] = None
    started_at: Optional[str] = None
    ended_at: Optional[str] = None

    def body(self) -> dict:
        return {"run_id": self.run_id, "declaration_seq": self.declaration_seq,
                "course_id": self.course.course_id, "tiers": dict(self.tiers)}


@dataclass
class ThrusterFrame:
    """m1 PORT, m2 STERN, m3 STBD, m4 BOW, each in [-1, 1]."""
    m1: float = 0.0
    m2: float = 0.0
    m3: float = 0.0
    m4: float = 0.0

    def as_list(self) -> List[float]:
        return [self.m1, self.m2, self.m3, self.m4]

    @staticmethod
    def zero() -> "ThrusterFrame":
        return ThrusterFrame()


@dataclass
class GuidedTarget:
    lat: float
    lon: float
    speed_mps: Optional[float] = None


@dataclass
class VelocityYawTarget:
    vx_mps: float
    yaw_rate_rad_s: float


@dataclass
class Hold:
    """Behaviour output: hold position via the adapter (HOLD mode)."""
    pass


class Loiter(Hold):
    """v0.1.81 (WP37): the autonomous station keep - a real position hold (ArduRover LOITER, mode 5) instead of HOLD (motors neutral,
    drifting) when `station_keep hold|loiter` says loiter. A Hold to everyone else (muted stages, the guard): only the adapter differs."""
    pass


def station_keep_mode(cfg) -> str:
    """v0.1.81: the station keep in force - the live word (`station_keep hold|loiter`) or the profile's station_keep.mode; hold by default
    (Dmitrii 2026-11-11: hold until the LOITER drift check on the water has passed)."""
    m = str((((getattr(cfg, "extra", None) or {}).get("station_keep")) or {}).get("mode") or "hold").lower()
    return m if m in ("hold", "loiter") else "hold"


@dataclass
class Command:
    env: dict
    action: str
    task: Optional[str] = None
    args: Dict[str, Any] = field(default_factory=dict)

    @property
    def msg_id(self) -> str:
        return self.env["msg_id"]

    @property
    def run_id(self) -> Optional[int]:
        return self.env.get("run_id")

    @property
    def declaration_seq(self) -> Optional[int]:
        return self.env.get("declaration_seq")

    @property
    def intent_id(self) -> Optional[str]:
        return self.env.get("intent_id")

    @staticmethod
    def from_env(env: dict) -> "Command":
        b = env["body"]
        return Command(env=env, action=b["action"], task=b.get("task"), args=dict(b.get("args") or {}))


@dataclass
class Ack:
    ack_of: str
    result: str          # ACCEPTED | REJECTED | DONE | FAILED
    reason: str = ""
    state: str = ""

    def body(self) -> dict:
        return {"ack_of": self.ack_of, "result": self.result, "reason": self.reason, "state": self.state}


@dataclass
class Event:
    kind: str            # task_state | RobotX reports | box_survey | box_mission_complete | alert
    body: Dict[str, Any]

    def topic_suffix(self) -> str:
        return self.kind


@dataclass
class TaskStateEvent:
    layer: str           # vehicle | task | run
    machine: str
    src: Optional[str]
    dst: str
    reason: str
    t_utc: str = field(default_factory=utc_iso)

    def body(self) -> dict:
        return {"event": "task_state", "layer": self.layer, "machine": self.machine,
                "from": self.src, "to": self.dst, "reason": self.reason, "t": self.t_utc}


@dataclass
class VehicleState:
    """The in-process truth of one vehicle; status_body() is its status/<v> projection (5.3)."""
    state: VState = VState.V0
    current_task: str = RobotXTask.NONE.value
    task_name: Optional[str] = None
    task_state: Optional[str] = None
    fcu: FcuState = field(default_factory=FcuState)
    local: Optional[LocalPose] = None
    health: Health = field(default_factory=Health)
    fault: Optional[str] = None
    run: RunContext = field(default_factory=RunContext)
    hold_reason: Optional[str] = None
    tui_active: bool = False
    profile: str = ""                 # the mission manager's YAML profile (usv_pool.yaml / usv_lab.yaml), for the consoles
    lab: bool = False                 # any guard relaxed (LAB GUARDS banner)
    link_loss_action: str = "continue"
    fcu_link: Optional[str] = None    # v0.1.19: what usv_ui's presence beat says it holds: fake | real (None = no beat) - the sim guard
    rc6: Optional[dict] = None        # v0.1.50: {policy latched|hold, raw (the switch high), latched (it holds the boat)}
    tuning: Optional[dict] = None     # v0.1.49: the runtime knobs (companion/tuning.view) for local/<v>/mm/state - never bridged
    cmd: Optional[dict] = None        # v0.1.55 (D1): the commanded side (mission_manager._cmd_view) - local/<v>/mm/state only
    fence: Optional[dict] = None      # v0.1.53: the FCU geofence (companion/fence.FenceBook.view) - null until the mission manager ticks
    nav: Optional[str] = None         # v0.1.68 (WP17 F6): "ghost" in free run (the position is the ghost plant's), else absent
    motion: Optional[str] = None      # v0.1.68: "muted" in free run (mode requests only to the FCU), else absent
    free_run: Optional[dict] = None   # v0.1.68: {ghost, armed_policy, armed, sent, skip {ok, why}, gps_note?, banner?}
    station_keep: Optional[str] = None   # v0.1.81 (WP37): hold | loiter - the autonomous station keep in force
    guard: Optional[dict] = None      # v0.1.80 (WP36): the ZED guard's view {chip clear|SLOW|STOP|BRAKE|BACKOFF|lost|off, range_m, age_s, caution}

    def robot_state(self) -> str:
        return self.state.robot_state

    def status_body(self) -> dict:
        f = self.fcu
        d = self._status_body(f)
        if self.nav == "ghost" and self.free_run:          # v0.1.68 (F6): the ghost's position everywhere a position is read
            g = self.free_run.get("ghost") or {}
            d.update({"lat": g.get("lat"), "lon": g.get("lon"), "hdg_deg": g.get("hdg_deg"), "sog_mps": g.get("sog_mps")})
        if self.nav:
            d["nav"], d["motion"], d["free_run"] = self.nav, self.motion, self.free_run
        if self.guard is not None:                          # v0.1.80 (WP36): the GUARD chip (the OCS bar, the boat window)
            d["guard"] = self.guard
        if self.station_keep is not None:                   # v0.1.81 (WP37): the HOLD / LOITER switch
            d["station_keep"] = self.station_keep
        return d

    def _status_body(self, f) -> dict:
        return {
            "state": self.state.value,
            "robot_state": self.robot_state(),
            "current_task": self.current_task,
            "task_name": self.task_name,
            "task_state": self.task_state,
            "mode": f.mode_name,
            "armed": f.armed,
            "kill": f.kill,
            "estop": f.estop,
            "lat": f.lat, "lon": f.lon,
            "hdg_deg": f.hdg_deg,
            "sog_mps": f.sog_mps,
            "roll_deg": round(f.roll_deg, 1), "pitch_deg": round(f.pitch_deg, 1),
            "yaw_rate_dps": None if f.yaw_rate_dps is None else round(f.yaw_rate_dps, 1),       # v0.1.51: the HUD's r   # v0.1.26: the judge's heartbeat (handbook 3.4.11)
            "altitude_hae_m": None,
            "flight_phase": None,
            "health": self.health.body(),
            "fault": self.fault,
            "hold_reason": self.hold_reason,
            "fcu_link": self.fcu_link,
            "rc6": self.rc6,
            "fence": self.fence,
            "run": self.run.body(),
            "local": None if self.local is None else {"x_n": round(self.local.x_n, 2), "y_e": round(self.local.y_e, 2),
                                                      "co_lat": self.local.co_lat, "co_lon": self.local.co_lon},
        }

    def local_body(self) -> dict:
        """Full state for local/<v>/mm/state (5 Hz, never bridged)."""
        d = self.status_body()
        d["ui_active"] = self.tui_active          # usv_ui beats tui/active: it commands the FCU, ENABLE_AUTO is refused
        d["profile"], d["lab"], d["link_loss_action"] = self.profile, self.lab, self.link_loss_action   # v0.1.18: the consoles' WHERE WE ARE
        d["fcu"] = {k: v for k, v in asdict(self.fcu).items() if k not in ("statustext",)}
        d["fcu"]["ages"] = {k: round(v, 3) for k, v in self.fcu.ages.items()}
        d["tuning"] = self.tuning
        d["cmd"] = self.cmd                       # v0.1.55 (D1): frame / target / hdg_goal_deg, all null when nothing commands
        return d


# ---- perception and map (ADR-12, D1: body-frame observations in, local-frame entities out) ---------------------------
@dataclass
class ObsItem:
    """One detected object in the camera's body frame (FRD): bearing positive to starboard, elevation positive up. range_m 0
    (null on the wire) = a bearing without a range (v0.1.34: beyond the depth limit) - the map keeps it as a ray."""
    id: int
    cls: str
    colour: str = "unknown"
    conf: float = 0.0
    range_m: float = 0.0
    bearing_deg: float = 0.0
    elev_deg: float = 0.0
    quality: Dict[str, Any] = field(default_factory=dict)
    lit: Optional[bool] = None

    @staticmethod
    def from_body(b: dict) -> "ObsItem":
        return ObsItem(id=int(b.get("id", 0)), cls=str(b.get("class", b.get("cls", "buoy"))), colour=str(b.get("colour", "unknown")),
                       conf=float(b.get("confidence", b.get("conf", 0.0))), range_m=float(b.get("range_m") or 0.0),   # null = bearing only (v0.1.34)
                       bearing_deg=float(b.get("bearing_deg", 0.0)), elev_deg=float(b.get("elev_deg", 0.0)),
                       quality=dict(b.get("quality") or {}), lit=b.get("lit"))

    def body(self) -> dict:
        d = {"id": self.id, "class": self.cls, "colour": self.colour, "confidence": round(self.conf, 3),
             "range_m": round(self.range_m, 3) if self.range_m > 0.0 else None,
             "bearing_deg": round(self.bearing_deg, 2), "elev_deg": round(self.elev_deg, 2), "quality": self.quality}
        if self.lit is not None:
            d["lit"] = self.lit
        return d


@dataclass
class Observation:
    """What a detector publishes on local/<v>/perception/<kind> (ARCHITECTURE 5.4). t_cam is unix seconds at capture."""
    t_cam: float
    kind: str = "buoys"                     # buoys | dock | window | aruco
    camera: str = "zed2i_left"
    frame: str = "body_FRD"
    pose_hint: Optional[dict] = None        # {lat, lon, hdg_deg, t} as seen by the perception node, provenance only
    items: List[ObsItem] = field(default_factory=list)
    source: str = "usv"
    t_utc: str = ""

    @staticmethod
    def from_body(b: dict, kind: Optional[str] = None) -> "Observation":
        return Observation(t_cam=float(b.get("t_cam", 0.0)), kind=str(kind or b.get("kind", "buoys")), camera=str(b.get("camera", "")),
                           frame=str(b.get("frame", "body_FRD")), pose_hint=b.get("pose_hint"),
                           items=[ObsItem.from_body(i) for i in (b.get("items") or [])], source=str(b.get("source", "usv")),
                           t_utc=str(b.get("t", b.get("t_utc", ""))))

    def body(self) -> dict:
        return {"t": self.t_utc or utc_iso(), "t_cam": round(self.t_cam, 3), "frame": self.frame, "camera": self.camera, "kind": self.kind,
                "pose_hint": self.pose_hint, "items": [i.body() for i in self.items], "source": self.source}


@dataclass
class MapEntity:
    """A tracked object in the local frame; sigma_m is the running residual, n_obs the evidence."""
    id: int
    kind: str                               # buoy | dock | window | aruco
    colour: str
    x_n: float
    y_e: float
    sigma_m: float
    n_obs: int
    t_first: float
    t_last: float
    state: str = "tracked"                  # tracked | stale
    source: str = "usv"
    colour_votes: Dict[str, float] = field(default_factory=dict)   # v0.1.34: weighted by range (PerceptionMap.vote_weight)
    light: str = "?"                        # flash | steady | off | ? (from the colour votes, PerceptionMap.classify_light)
    colour_now: str = "unknown"             # the colour of the latest sighting (a Task 3 window RED -> GREEN)

    def body(self) -> dict:
        return {"id": self.id, "kind": self.kind, "colour": self.colour, "x_n": round(self.x_n, 2), "y_e": round(self.y_e, 2),
                "sigma_m": round(self.sigma_m, 2), "n_obs": self.n_obs, "age_s": None, "state": self.state, "source": self.source,
                "light": self.light, "colour_now": self.colour_now}


@dataclass
class Gate:
    red: MapEntity
    green: MapEntity
    mid_n: float
    mid_e: float
    through_hdg_deg: float                  # heading that passes the gate with red to starboard
    width_m: float
    passed: bool = False
    lucky: bool = False                     # v0.1.90: a pair of uncertain colour taken as a gate (the `lucky` flag)

    def body(self) -> dict:
        return {"red": self.red.id, "green": self.green.id, "mid_n": round(self.mid_n, 2), "mid_e": round(self.mid_e, 2),
                "through_hdg_deg": round(self.through_hdg_deg, 1), "width_m": round(self.width_m, 2), "passed": self.passed,
                **({"lucky": True} if self.lucky else {})}
