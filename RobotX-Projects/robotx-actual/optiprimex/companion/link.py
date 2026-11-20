"""MavLink: the companion's view of the FCU (MODULES 3.1 link.py). Never arms.

A background thread reads the stream and maintains FcuState with per-message ages. Senders are the
few messages the adapters need; every mode change waits for COMMAND_ACK.
"""
from __future__ import annotations

import logging
import os
import queue
import threading
import time
from typing import Callable, Dict, List, Optional

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

from . import fence as F
from .config import Config
from .state import FcuState, ThrusterFrame, rc_health, utc_iso

log = logging.getLogger("optiprimex.link")
MAV = mavutil.mavlink

ROVER_MODES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 6: "FOLLOW", 7: "SIMPLE",
               8: "DOCK", 9: "CIRCLE", 10: "AUTO", 11: "RTL", 12: "SMART_RTL", 15: "GUIDED",
               16: "INITIALISING", 17: "CUSTOM_AUTO", 20: "CUSTOM_MANUAL"}
MODE_BY_NAME = {v: k for k, v in ROVER_MODES.items()}

MSG_IDS = {
    "HEARTBEAT": MAV.MAVLINK_MSG_ID_HEARTBEAT,
    "GLOBAL_POSITION_INT": MAV.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
    "ATTITUDE": MAV.MAVLINK_MSG_ID_ATTITUDE,
    "GPS_RAW_INT": MAV.MAVLINK_MSG_ID_GPS_RAW_INT,
    "SYS_STATUS": MAV.MAVLINK_MSG_ID_SYS_STATUS,
    "RC_CHANNELS": MAV.MAVLINK_MSG_ID_RC_CHANNELS,
    "SERVO_OUTPUT_RAW": MAV.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
    "EKF_STATUS_REPORT": MAV.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
    "NAMED_VALUE_INT": MAV.MAVLINK_MSG_ID_NAMED_VALUE_INT,
    "FENCE_STATUS": MAV.MAVLINK_MSG_ID_FENCE_STATUS,          # v0.1.53: a profile may ask for it (mavlink.intervals); ArduPilot streams it
}
GEOFENCE_BIT = MAV.MAV_SYS_STATUS_GEOFENCE                    # v0.1.53: SYS_STATUS sensor bit 0x100000
MISSION_KINDS = ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK", "MISSION_COUNT", "MISSION_ITEM_INT")
FENCE_TYPE = MAV.MAV_MISSION_TYPE_FENCE
EKF_POS_HORIZ_ABS = MAV.EKF_POS_HORIZ_ABS                # 0x10
RC_RECEIVER_BIT = 0x10000                # MAV_SYS_STATUS_SENSOR_RC_RECEIVER (v0.1.29.3)
# v0.1.54.1: 0x100 is EKF_PRED_POS_HORIZ_REL (a relative-only prediction, no absolute fix); ABS is 0x200. The old 0x100 let
# a relative-only predicted position pass the AUTO guard on a real Pixhawk. Taken from pymavlink's enum, pinned by tests/test_v01541.py.
EKF_PRED_POS_HORIZ_REL = MAV.EKF_PRED_POS_HORIZ_REL      # 0x100 - never counts as a position
EKF_PRED_POS_HORIZ_ABS = MAV.EKF_PRED_POS_HORIZ_ABS      # 0x200

# SET_POSITION_TARGET type masks
MASK_POSITION_ONLY = 0b0000_1111_1111_1000   # use x, y, z only
MASK_VELOCITY_YAWRATE = 1479                  # use vx, vy, vz + yaw_rate (from the CV scripts)


class MavLink:
    def __init__(self, cfg: Config, endpoint: Optional[str] = None):
        self.cfg = cfg
        self.endpoint = endpoint or cfg.mavlink.endpoint
        self._conn = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._state = FcuState()
        self._last: Dict[str, float] = {}
        self._cbs: Dict[str, List[Callable]] = {}
        self._acks: Dict[int, int] = {}            # command -> result
        self._ack_event = threading.Event()
        self._intervals_requested = False
        self.target_system = cfg.mavlink.target_sysid
        self.target_component = 1
        self.stats = {"rx": 0, "tx": 0}
        self._mission_q: "queue.Queue" = queue.Queue()   # v0.1.53: fence-protocol replies addressed to us (the rx thread fills it)
        self._fence_lock = threading.Lock()              # one fence transaction at a time

    # ---- lifecycle ---------------------------------------------------------------
    def start(self) -> None:
        self._conn = mavutil.mavlink_connection(self.endpoint, source_system=self.cfg.mavlink.sysid,
                                                source_component=self.cfg.mavlink.compid, baud=self.cfg.mavlink.baud,
                                                autoreconnect=True)
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mavlink-rx", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.5)
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass

    def on(self, msg_type: str, cb: Callable) -> None:
        self._cbs.setdefault(msg_type, []).append(cb)

    def state(self) -> FcuState:
        """A snapshot with ages computed now."""
        now = time.monotonic()
        with self._lock:
            s = FcuState(**{k: v for k, v in self._state.__dict__.items()})
            s.rc = dict(self._state.rc)
            s.servo = list(self._state.servo)
            s.statustext = list(self._state.statustext[-5:])
            s.ages = {k: now - t for k, t in self._last.items()}
        s.rc_ok, s.rc_detail = rc_health(s, self.cfg.timeouts.rc_s)
        return s

    # ---- rx -----------------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.2)
            except Exception as e:  # pragma: no cover - transport hiccups
                log.debug("recv error: %s", e)
                time.sleep(0.05)
                continue
            if msg is None:
                continue
            self.stats["rx"] += 1
            kind = msg.get_type()
            if kind == "BAD_DATA":
                continue
            now = time.monotonic()
            with self._lock:
                self._last[kind] = now
                self._state.t_mono = now
                self._state.t_utc = utc_iso()
                self._apply(kind, msg)
            if kind == "HEARTBEAT" and not self._intervals_requested:
                self._intervals_requested = True
                self.request_intervals()
            for cb in self._cbs.get(kind, ()):
                try:
                    cb(msg)
                except Exception:
                    log.exception("callback for %s failed", kind)

    def _apply(self, kind: str, msg) -> None:
        s = self._state
        if kind == "HEARTBEAT":
            if msg.get_srcSystem() != self.target_system and msg.type == MAV.MAV_TYPE_GCS:
                return
            s.seen_heartbeat = True
            s.vehicle_type = int(msg.type)
            s.autopilot = int(msg.autopilot)
            s.armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
            s.mode = int(msg.custom_mode)
            s.mode_name = ROVER_MODES.get(s.mode, f"MODE{s.mode}")
            s.system_status = int(msg.system_status)
        elif kind == "GLOBAL_POSITION_INT":
            if msg.lat != 0 or msg.lon != 0:
                s.lat, s.lon = msg.lat / 1e7, msg.lon / 1e7
            s.alt_m = msg.alt / 1000.0
            s.vx_n, s.vy_e = msg.vx / 100.0, msg.vy / 100.0
            s.sog_mps = (s.vx_n ** 2 + s.vy_e ** 2) ** 0.5
            if msg.hdg != 65535:
                s.hdg_deg = msg.hdg / 100.0
        elif kind == "ATTITUDE":
            import math
            s.roll_deg, s.pitch_deg = math.degrees(msg.roll), math.degrees(msg.pitch)
            s.yaw_rate_dps = math.degrees(msg.yawspeed)          # v0.1.51 (WP10k-c item 26): r for the HUD (ATTITUDE.yawspeed)
            s.yaw_deg = math.degrees(msg.yaw) % 360.0
            if s.hdg_deg is None:
                s.hdg_deg = s.yaw_deg
        elif kind == "GPS_RAW_INT":
            s.gps_fix = int(msg.fix_type)
            s.sats = 0 if msg.satellites_visible == 255 else int(msg.satellites_visible)
            s.hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.0
        elif kind == "SYS_STATUS":
            present = int(getattr(msg, "onboard_control_sensors_present", 0) or 0)
            if present & RC_RECEIVER_BIT:
                s.rc_rx_present = True
                s.rc_rx_healthy = bool(int(getattr(msg, "onboard_control_sensors_health", 0) or 0) & RC_RECEIVER_BIT)
            else:
                s.rc_rx_present = False
            if present & GEOFENCE_BIT:                                  # v0.1.53: the FCU's own geofence flags
                s.fence_sys_present = True
                s.fence_sys_enabled = bool(int(getattr(msg, "onboard_control_sensors_enabled", 0) or 0) & GEOFENCE_BIT)
                s.fence_sys_healthy = bool(int(getattr(msg, "onboard_control_sensors_health", 0) or 0) & GEOFENCE_BIT)
            else:
                s.fence_sys_present, s.fence_sys_enabled, s.fence_sys_healthy = False, None, None
            s.batt_v = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
            s.batt_pct = None if msg.battery_remaining == -1 else int(msg.battery_remaining)
        elif kind == "RC_CHANNELS":
            s.rc_chancount = int(msg.chancount)
            s.rc_last_plausible = any(800 <= int(getattr(msg, f"chan{i}_raw", 0) or 0) <= 2200 for i in range(1, 5))
            for i in range(1, 17):
                v = getattr(msg, f"chan{i}_raw", 0)
                if v and v != 65535:
                    s.rc[i] = int(v)
        elif kind == "SERVO_OUTPUT_RAW":
            s.servo = [int(getattr(msg, f"servo{i}_raw", 0)) for i in range(1, 9)]
        elif kind == "EKF_STATUS_REPORT":
            s.ekf_flags = int(msg.flags)
            s.ekf_pos_ok = bool(msg.flags & EKF_POS_HORIZ_ABS) or bool(msg.flags & EKF_PRED_POS_HORIZ_ABS)
        elif kind == "NAMED_VALUE_INT":
            name = msg.name.rstrip("\x00") if isinstance(msg.name, str) else msg.name.decode(errors="ignore").rstrip("\x00")
            if name == "KILL":
                s.kill = bool(msg.value)
            elif name == "LAMP":                                        # v0.1.55 (D4): the lamp read-back hook (the Lua does not send it yet)
                s.lamp = int(msg.value)
            elif name == "ESTOP":
                s.estop = bool(msg.value)
        elif kind == "STATUSTEXT":
            text = msg.text if isinstance(msg.text, str) else msg.text.decode(errors="ignore")
            s.statustext.append(text.rstrip("\x00"))
            s.statustext = s.statustext[-20:]
        elif kind == "COMMAND_ACK":
            tc = int(getattr(msg, "target_component", 0) or 0)
            if tc not in (0, self.cfg.mavlink.compid):
                return               # addressed to another component of our sysid (usv_ui is 200/190, we are 200/191)
            self._acks[int(msg.command)] = int(msg.result)
            self._ack_event.set()
        elif kind == "PARAM_VALUE":
            s.params_ok = True
        elif kind == "FENCE_STATUS":                                    # v0.1.53
            s.fence_breach_status = int(msg.breach_status)
            s.fence_breach_count = int(msg.breach_count)
            s.fence_breach_type = int(msg.breach_type)
        elif kind in MISSION_KINDS:                                     # v0.1.53: the fence protocol's replies, to the waiting caller
            if int(getattr(msg, "mission_type", 0) or 0) != FENCE_TYPE:
                return
            ts, tc = int(getattr(msg, "target_system", 0) or 0), int(getattr(msg, "target_component", 0) or 0)
            if ts not in (0, self.cfg.mavlink.sysid) or tc not in (0, self.cfg.mavlink.compid):
                return               # a reply to another program (usv_ui, Mission Planner through a router) - not ours
            self._mission_q.put(msg)

    # ---- tx -----------------------------------------------------------------------
    def request_intervals(self) -> None:
        for name, hz in self.cfg.mavlink.intervals.items():
            mid = MSG_IDS.get(name)
            if mid is None or hz <= 0:
                continue
            self._conn.mav.command_long_send(self.target_system, self.target_component,
                                             MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, int(1e6 / hz), 0, 0, 0, 0, 0)
            self.stats["tx"] += 1

    def _wait_ack(self, command: int, timeout_s: float) -> Optional[int]:
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout_s:
            if command in self._acks:
                return self._acks.pop(command)
            self._ack_event.wait(0.05)
            self._ack_event.clear()
        return None

    def set_mode(self, mode: int | str, timeout_s: Optional[float] = None) -> bool:
        """DO_SET_MODE with COMMAND_ACK wait. Returns True on MAV_RESULT_ACCEPTED."""
        if isinstance(mode, str):
            mode = MODE_BY_NAME[mode]
        self._acks.pop(MAV.MAV_CMD_DO_SET_MODE, None)
        self._conn.mav.command_long_send(self.target_system, self.target_component, MAV.MAV_CMD_DO_SET_MODE, 0,
                                         MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, int(mode), 0, 0, 0, 0, 0)
        self.stats["tx"] += 1
        res = self._wait_ack(MAV.MAV_CMD_DO_SET_MODE, timeout_s or self.cfg.mavlink.mode_ack_timeout_s)
        return res == MAV.MAV_RESULT_ACCEPTED

    def arm(self, on: bool, timeout_s: float = 2.0) -> Optional[int]:
        """v0.1.50: MAV_CMD_COMPONENT_ARM_DISARM (the operator's page button, Dmitrii 2026-11-06); the FCU's own pre-arm checks decide.
        Returns the MAV_RESULT, None when no ack came."""
        self._acks.pop(MAV.MAV_CMD_COMPONENT_ARM_DISARM, None)
        self._conn.mav.command_long_send(self.target_system, self.target_component, MAV.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                         1.0 if on else 0.0, 0, 0, 0, 0, 0, 0)
        self.stats["tx"] += 1
        return self._wait_ack(MAV.MAV_CMD_COMPONENT_ARM_DISARM, timeout_s)

    def send_actuator_frame(self, frame: ThrusterFrame) -> None:
        controls = [max(-1.0, min(1.0, float(v))) for v in frame.as_list()] + [0.0] * 4
        self._conn.mav.set_actuator_control_target_send(int(time.monotonic() * 1e6) & 0xFFFFFFFFFFFFFFFF, 0,
                                                        self.target_system, self.target_component, controls)
        self.stats["tx"] += 1

    def send_position_target_global(self, lat: float, lon: float, alt_m: float = 0.0) -> None:
        self._conn.mav.set_position_target_global_int_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF, self.target_system, self.target_component,
            MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK_POSITION_ONLY,
            int(lat * 1e7), int(lon * 1e7), float(alt_m), 0, 0, 0, 0, 0, 0, 0, 0)
        self.stats["tx"] += 1

    def send_velocity_yaw(self, vx_mps: float, yaw_rate_rad_s: float) -> None:
        self._conn.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF, self.target_system, self.target_component,
            MAV.MAV_FRAME_BODY_NED, MASK_VELOCITY_YAWRATE, 0, 0, 0, float(vx_mps), 0, 0, 0, 0, 0, 0,
            float(yaw_rate_rad_s))
        self.stats["tx"] += 1

    def do_change_speed(self, mps: float, timeout_s: float = 1.0) -> bool:
        self._acks.pop(MAV.MAV_CMD_DO_CHANGE_SPEED, None)
        self._conn.mav.command_long_send(self.target_system, self.target_component, MAV.MAV_CMD_DO_CHANGE_SPEED, 0,
                                         0, float(mps), -1, 0, 0, 0, 0)
        self.stats["tx"] += 1
        return self._wait_ack(MAV.MAV_CMD_DO_CHANGE_SPEED, timeout_s) == MAV.MAV_RESULT_ACCEPTED

    def param_request(self, name: str) -> None:
        self._conn.mav.param_request_read_send(self.target_system, self.target_component, name.encode()[:16], -1)
        self.stats["tx"] += 1

    def command_long(self, cmd: int, *params: float, timeout_s: float = 1.0) -> Optional[int]:
        p = list(params) + [0] * (7 - len(params))
        self._acks.pop(cmd, None)
        self._conn.mav.command_long_send(self.target_system, self.target_component, cmd, 0, *p[:7])
        self.stats["tx"] += 1
        return self._wait_ack(cmd, timeout_s)

    # ---- v0.1.53: the FCU geofence (MAVLink mission protocol, mission_type FENCE; Dmitrii 2026-11-06) --------------------------
    # Runs on the caller's thread (the command handler's fence worker), never on the tick: the rx thread hands the replies over
    # through _mission_q. MAVLink 2 only: MAVLink 1 has no mission_type and the FCU would take the items as a MISSION (UAV lesson).
    def _mav2(self) -> bool:
        return str(getattr(MAV, "WIRE_PROTOCOL_VERSION", "")) == "2.0"

    def _mission_wait(self, kinds, timeout_s: float, seq: Optional[int] = None):
        t_end = time.monotonic() + timeout_s
        while True:
            left = t_end - time.monotonic()
            if left <= 0:
                return None
            try:
                msg = self._mission_q.get(timeout=min(left, 0.2))
            except queue.Empty:
                continue
            if msg.get_type() in kinds and (seq is None or int(getattr(msg, "seq", -1)) == seq):
                return msg

    def _drain_mission(self) -> None:
        while True:
            try:
                self._mission_q.get_nowait()
            except queue.Empty:
                return

    def fence_upload(self, points, timeout_s: float = 2.0, retries: int = 3):
        """One inclusion polygon: MISSION_COUNT(FENCE) -> MISSION_REQUEST(_INT) seq i -> MISSION_ITEM_INT
        (MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, param1 = the vertex count, x / y = deg * 1e7) -> MISSION_ACK. Replaces the FCU's
        whole fence (the fence list is not incremental). Returns (ok, text); the text names the FCU's MAV_MISSION_* result."""
        if not self._mav2():
            return False, "MAVLink 1 on this link: the fence needs MAVLink 2 (mission_type) - set MAVLINK20=1"
        pts = F.ring(points)
        n = len(pts)
        cmd = MAV.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        with self._fence_lock:
            for attempt in range(1, retries + 1):
                self._drain_mission()
                self._conn.mav.mission_count_send(self.target_system, self.target_component, n, FENCE_TYPE)
                self.stats["tx"] += 1
                while True:
                    msg = self._mission_wait(("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"), timeout_s)
                    if msg is None:
                        break                                    # no progress: start again from MISSION_COUNT
                    if msg.get_type() == "MISSION_ACK":
                        r = int(msg.type)
                        if r == MAV.MAV_MISSION_ACCEPTED:
                            return True, f"{n} points accepted (MISSION_ACK MAV_MISSION_ACCEPTED)"
                        return False, f"the FCU refused: {F.MISSION_RESULT.get(r, f'MAV_MISSION result {r}')}"
                    seq = int(msg.seq)
                    if not 0 <= seq < n:
                        continue
                    lat, lon = pts[seq]
                    self._conn.mav.mission_item_int_send(self.target_system, self.target_component, seq, MAV.MAV_FRAME_GLOBAL, cmd,
                                                         0, 0, float(n), 0, 0, 0, int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0,
                                                         FENCE_TYPE)
                    self.stats["tx"] += 1
                log.info("fence upload: no MISSION_ACK (attempt %d / %d)", attempt, retries)
        return False, f"no MISSION_ACK after {retries} attempts"

    def fence_download(self, timeout_s: float = 2.0):
        """MISSION_REQUEST_LIST(FENCE) -> MISSION_COUNT -> MISSION_REQUEST_INT seq i -> MISSION_ITEM_INT, then our MISSION_ACK.
        Returns ([(command, param1, lat, lon), ...], "") or (None, why)."""
        if not self._mav2():
            return None, "MAVLink 1 on this link: the fence needs MAVLink 2"
        with self._fence_lock:
            self._drain_mission()
            self._conn.mav.mission_request_list_send(self.target_system, self.target_component, FENCE_TYPE)
            self.stats["tx"] += 1
            msg = self._mission_wait(("MISSION_COUNT",), timeout_s)
            if msg is None:
                return None, "no MISSION_COUNT (the read-back timed out)"
            items = []
            for seq in range(int(msg.count)):
                it = None
                for _ in range(2):                               # one resend of the request, then give up
                    self._conn.mav.mission_request_int_send(self.target_system, self.target_component, seq, FENCE_TYPE)
                    self.stats["tx"] += 1
                    it = self._mission_wait(("MISSION_ITEM_INT",), timeout_s, seq=seq)
                    if it is not None:
                        break
                if it is None:
                    return None, f"no MISSION_ITEM_INT for item {seq} (the read-back is incomplete)"
                items.append((int(it.command), float(it.param1), it.x / 1e7, it.y / 1e7))
            self._conn.mav.mission_ack_send(self.target_system, self.target_component, MAV.MAV_MISSION_ACCEPTED, FENCE_TYPE)
            self.stats["tx"] += 1
            return items, ""

    def fence_enable(self, on: bool, timeout_s: float = 2.0) -> Optional[int]:
        """MAV_CMD_DO_FENCE_ENABLE (param1 1 = on, 0 = off) with COMMAND_ACK - a command, not the FENCE_ENABLE parameter. The MAV_RESULT,
        None when no ack came."""
        return self.command_long(MAV.MAV_CMD_DO_FENCE_ENABLE, 1.0 if on else 0.0, timeout_s=timeout_s)
