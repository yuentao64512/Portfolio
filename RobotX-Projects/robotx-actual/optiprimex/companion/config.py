"""One YAML per node, validated at start; no magic numbers elsewhere (MODULES 3.1 config.py)."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from .. import TEAM_NS


@dataclass
class MavlinkCfg:
    endpoint: str = "udpin:127.0.0.1:14550"
    baud: int = 921600
    sysid: int = 200
    compid: int = 191
    target_sysid: int = 1
    # message id -> Hz, requested with SET_MESSAGE_INTERVAL after the first heartbeat
    intervals: Dict[str, float] = field(default_factory=lambda: {
        "HEARTBEAT": 1, "GLOBAL_POSITION_INT": 10, "ATTITUDE": 10, "GPS_RAW_INT": 2,
        "SYS_STATUS": 1, "RC_CHANNELS": 5, "SERVO_OUTPUT_RAW": 5, "EKF_STATUS_REPORT": 2,
        "NAMED_VALUE_INT": 2,
    })
    mode_ack_timeout_s: float = 1.0


@dataclass
class MqttCfg:
    host: str = "127.0.0.1"
    port: int = 1882                 # v0.1.90.12: the hub's default (1883 = the drone's map port on the NX)
    user: Optional[str] = None
    password: Optional[str] = None
    ns: str = TEAM_NS
    client_id: str = "usv-mm"
    keepalive_s: int = 15
    connect_timeout_s: float = 5.0
    bridge_name: str = "ocs"  # $SYS/broker/connection/<bridge_name>/state on the local broker


@dataclass
class TimeoutsCfg:
    fcu_s: float = 2.0          # FCU stream silence -> FCU_LOST
    rc_s: float = 2.0           # RC_CHANNELS silence -> RC_LOST
    perception_s: float = 0.0   # 0 = perception not required (bench B1/B2)
    cmd_ack_s: float = 0.5
    approach_s: float = 60.0
    hold_reassert_s: float = 5.0
    mode_ack_s: float = 0.2     # v0.1.86.5 (WP43 Tier 1.4): the adapter's longest wait for a mode ack in the tick (was 0.5)
    mode_backoff_s: float = 2.0 # v0.1.86.5: after a refused / unacked mode M, M is not asked again for this long (one log line)
    tui_active_s: float = 2.0
    loop_slow_s: float = 0.2
    incident_ack_s: float = 1.0


@dataclass
class LimitsCfg:
    max_amp: float = 0.6
    max_speed_mps: float = 1.0
    corridor_m: float = 6.0
    goto_arrive_r_m: float = 1.0
    goto_settle_r_m: float = 2.5
    goto_settle_gs_mps: float = 0.3
    goto_settle_s: float = 2.0
    goto_timeout_s: float = 120.0


@dataclass
class GainsCfg:
    stern_factor: float = 0.957
    kpsi: float = 0.6
    kc: float = 0.15
    sway_f_pos: float = 1.05           # v0.1.22: calibrated stern factors for sway (pool 2026-10-21, usv_ui swaymix): to starboard
    sway_f_neg: float = 0.877          #   ... and to port; M2 = F * v, M4 = v (F > 1: M4 = v / F so neither exceeds |v|)
    sway_kpsi: float = 0.6             # mode-20 yaw command per 45 deg of heading error (usv_ui --sway-kpsi, pool-proven)
    yaw_side_gain: float = 0.0         # v0.1.85 (WP41 T0.5): the side pair's share of a mode-20 yaw command (M2 += k r, M4 -= k r); 0 = unchanged
    sway_kc: float = 0.15              # mode-20 surge command per metre of cross-track along the bow axis (usv_ui --sway-kc)
    sway_rmax: float = 0.3
    sway_umax: float = 0.25


@dataclass
class FramesCfg:
    cam_forward_m: float = 0.0
    cam_right_m: float = 0.0
    cam_yaw_deg: float = 0.0
    origin_lat: Optional[float] = None   # control origin; None = first good fix
    origin_lon: Optional[float] = None
    pose_buffer_s: float = 5.0


@dataclass
class MapCfg:
    assoc_gate_m: float = 2.0          # a new observation within this of a tracked entity of the same kind updates it
    assoc_gate_a: float = 0.0          # v0.1.92 (Cowork consult): > 0 = a ranged buoy sighting's gate a + b * range, clamped [min, max] (the
    assoc_gate_b: float = 0.08         #   course profile: 0.35 + 0.08 r in 0.8..1.6 m - markers 1.4-2.8 m apart, 2.0 merged them), and within
    assoc_gate_min: float = 0.8        #   assoc_colour_m a sighting lit red / green / blue never joins an entity decided another of those
    assoc_gate_max: float = 1.6
    assoc_colour_m: float = 0.0        # v0.1.92: > 0 = the colour separation within this range (the course profile: 10)
    assoc_gate_window_m: float = 0.5   # the same for Task 3 windows (two per bay, ~1.2 m apart)
    max_age_s: float = 30.0            # not seen for this long -> stale (kept, not used for gates/dock)
    drop_age_s: float = 120.0          # stale for this long -> dropped
    ema_n_max: int = 10                # position EMA weight 1/min(n_obs, ema_n_max)
    gate_width_min_m: float = 1.5
    gate_width_max_m: float = 6.0
    dock_max_age_s: float = 10.0       # dock faces older than this do not take part in the line fit
    min_conf: float = 0.3              # observations below this confidence are ignored
    min_obs: int = 1                   # v0.1.31 (WP9a): entities seen fewer times stay internal - never in the digest, the map, gates,
                                       # markers, the dock line or a task's queries (the profiles set 3: profile section `perception:`)
    forget_s: float = 0.0              # v0.1.31: such a young entity not seen again for this long is dropped (0 = only the normal ageing)
    ray_ttl_s: float = 30.0            # v0.1.34 (WP10g): a bearing-only sighting (a ray) is kept this long (pathing spec 5.3: 30 s)
    ray_ang_tol_deg: float = 3.0       # a ray refreshed within 2x this; dropped when a tracked buoy lies within this of it
    tri_min_angle_deg: float = 10.0    # two rays of one colour crossing at least this much -> a point (triangulation)
    tri_max_range_m: float = 60.0      # ... no further than this from either boat position
    vote_full_m: float = 15.0          # v0.1.34: a colour seen this close counts 1 vote ...
    vote_far_m: float = 25.0           # ... falling linearly to vote_far_w at this range (far frames miscolour: the 2026-11-02 bug came
    vote_far_w: float = 0.25           #     back with the real 51 deg camera - an unlit obstacle at 22 m voted blue)
    triangulate: bool = False          # decided 2026-11-05 (Dmitrii): the pathing_sim way - rays are leads, a position only from depth;
                                       # true = crossing rays make points (CV_TRAIN's ask; kept for tests: perception.triangulate)


@dataclass
class RatesCfg:
    tick_hz: float = 10.0
    status_hz: float = 2.0
    local_hz: float = 5.0
    map_hz: float = 2.0


@dataclass
class GuardsCfg:
    """Vehicle-side guards. These are never relaxed by the OCS bench mode."""
    require_armed_for_auto: bool = True
    require_ekf_for_auto: bool = True
    require_prepare_for_auto: bool = False  # v0.1.90.8 (Dmitrii: "uniform rule everywhere"): ENABLE AUTO before DECLARE (True = R3+ first)
    require_run_for_task: bool = True       # run_id must be known (R5)
    v1_exit_requires_confirm: bool = True   # KILLED -> MANUAL needs CONFIRM_SAFE from the OCS
    v6_exit_requires_confirm: bool = True
    allow_goto_task: bool = False           # bench one-shot GOTO task (never on the course)
    test_mode: bool = False                 # v0.1.37 (WP10d): the run-free tasks (RUN_FREE_TASKS) start in AUTO without a declaration or
                                            # RunStart, and enable_auto needs no PREPARE; the judged missions keep the run (course: false)
    ignore_faults: List[str] = field(default_factory=list)   # LAB ONLY (usv_lab.yaml): faults still reported in status.health but
                                            # never hard/soft faults - e.g. EKF_BAD indoors (no GPS), RC_LOST with the transmitter off
    rc_lost_action: str = "report"          # v0.1.29.3: RC_LOST (the transmitter lost): report = shown only (as before) | hold =
                                            # a soft fault (a running task -> AUTO_SAFE_HOLD, enable_auto refused). Decided 2026-11-04:
                                            # hold on pool + course, switched on after the lab day shows rc ok / rc NO correctly
    link_loss_action: str = "continue"      # what a running task does when the boat broker's bridge to the hub drops (MODULES S6):
                                            # continue (default: the task goes on, QoS 1 events queue) | safe_hold (V5 after link_loss_s;
                                            # the hold clears when the link is back, the operator resumes)
    link_loss_s: float = 5.0
    rc6_policy: str = "latched"             # v0.1.50 (Dmitrii 2026-11-06): RC6 high = MANUAL at once; latched = a later page command
                                            # (enable_auto, an FCU mode / arm, or a mode change seen on the FCU) clears it - last wins,
                                            # the next flip takes the boat back; hold = while RC6 is high, no autonomy (the v0.1.29 rule)
    ekf_bad_hard_s: float = 3.0             # v0.1.59 (A2): EKF_BAD is a soft fault (V5 hold) until continuous this long, then hard (V6);
                                            # 0 = hard at once (the pre-v0.1.59 rule)
    cmd_max_age_s: float = 10.0             # v0.1.43 (OCS_V2_SPEC §6): a cmd whose envelope sent_at is older than this is REJECTED
                                            # `stale` - the bridge delivers queued QoS-1 commands after a link loss (clocks within
                                            # 1 s); 0 = not checked. A cmd without a readable sent_at is accepted (logged)


@dataclass
class Config:
    vehicle_id: str = "USV1"
    vehicle: str = "usv"               # topic token: usv | uav
    node: str = "mission_manager"
    log_dir: str = "./logs"
    mavlink: MavlinkCfg = field(default_factory=MavlinkCfg)
    mqtt: MqttCfg = field(default_factory=MqttCfg)
    timeouts: TimeoutsCfg = field(default_factory=TimeoutsCfg)
    limits: LimitsCfg = field(default_factory=LimitsCfg)
    gains: GainsCfg = field(default_factory=GainsCfg)
    frames: FramesCfg = field(default_factory=FramesCfg)
    map: MapCfg = field(default_factory=MapCfg)
    rates: RatesCfg = field(default_factory=RatesCfg)
    guards: GuardsCfg = field(default_factory=GuardsCfg)
    extra: Dict[str, Any] = field(default_factory=dict)   # node-specific sections (ocs, sim)
    profile: str = ""                  # the YAML file name this config came from (load() sets it; shown by the consoles)

    # ---- topic helpers -------------------------------------------------
    def team(self, *parts: str) -> str:
        return "/".join([self.mqtt.ns, *parts])

    def local(self, *parts: str) -> str:
        return "/".join(["local", self.vehicle, *parts])


_SECTIONS = {
    "mavlink": MavlinkCfg, "mqtt": MqttCfg, "timeouts": TimeoutsCfg, "limits": LimitsCfg,
    "gains": GainsCfg, "frames": FramesCfg, "map": MapCfg, "rates": RatesCfg, "guards": GuardsCfg,
}


def _build(cls, data: Dict[str, Any], path: str):
    fields = {f.name: f for f in dataclasses.fields(cls)}
    unknown = set(data) - set(fields)
    if unknown:
        raise ValueError(f"config: unknown keys in {path}: {sorted(unknown)}")
    kwargs = {}
    for k, v in data.items():
        f = fields[k]
        if dataclasses.is_dataclass(f.type) if isinstance(f.type, type) else False:
            kwargs[k] = _build(f.type, v or {}, f"{path}.{k}")
        else:
            kwargs[k] = v
    return cls(**kwargs)


def from_dict(data: Dict[str, Any]) -> Config:
    data = dict(data or {})
    sections = {}
    for name, cls in _SECTIONS.items():
        sections[name] = _build(cls, data.pop(name, {}) or {}, name)
    extra = data.pop("extra", {}) or {}
    # anything else top-level that is not a Config field goes to extra (ocs:, sim:, ...)
    known = {f.name for f in dataclasses.fields(Config)}
    for k in list(data):
        if k not in known:
            extra[k] = data.pop(k)
    per = extra.get("perception")                    # v0.1.31 (WP9a): `perception: {min_obs, forget_s}` = the map's ghost filter
    if isinstance(per, dict):
        for k in ("min_obs", "forget_s", "triangulate", "ray_ttl_s", "max_age_s", "drop_age_s", "assoc_gate_a", "assoc_gate_b",
                  "assoc_gate_min", "assoc_gate_max", "assoc_colour_m"):    # v0.1.92: + the memory
            if k in per:
                setattr(sections["map"], k, type(getattr(sections["map"], k))(per[k]))
    cfg = Config(**data, **sections, extra=extra)
    _validate(cfg)
    return cfg


def load(path: str | Path) -> Config:
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    cfg = from_dict(data)
    cfg.profile = Path(path).name
    return cfg


def _validate(cfg: Config) -> None:
    from .health import FAULTS
    bad = [f for f in cfg.guards.ignore_faults if f not in FAULTS]
    if bad:
        raise ValueError(f"config: guards.ignore_faults has unknown faults {bad}; known: {list(FAULTS)}")
    if cfg.vehicle not in ("usv", "uav"):
        raise ValueError("config.vehicle must be usv or uav")
    if cfg.guards.rc_lost_action not in ("report", "hold"):
        raise ValueError("config: guards.rc_lost_action must be report or hold")
    if cfg.guards.link_loss_action not in ("continue", "safe_hold"):
        raise ValueError("config: guards.link_loss_action must be continue or safe_hold")
    if cfg.guards.cmd_max_age_s < 0:
        raise ValueError("config: guards.cmd_max_age_s must be >= 0 (0 = not checked)")
    if cfg.map.min_obs < 1 or cfg.map.forget_s < 0:
        raise ValueError("config: perception.min_obs >= 1 and perception.forget_s >= 0")
    if not (0 < cfg.limits.max_amp <= 1.0):
        raise ValueError("limits.max_amp must be in (0, 1]")
    if cfg.rates.tick_hz <= 0 or cfg.rates.status_hz <= 0:
        raise ValueError("rates must be positive")
    if cfg.timeouts.fcu_s <= 0:
        raise ValueError("timeouts.fcu_s must be positive")
