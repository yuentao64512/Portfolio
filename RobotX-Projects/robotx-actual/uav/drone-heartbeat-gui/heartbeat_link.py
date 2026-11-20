"""
MAVLink heartbeat link: connects to a MAVLink endpoint and tracks HEARTBEAT
status in a background thread.

This module knows nothing about how the status is displayed — it just
exposes a thread-safe snapshot that any GUI (terminal, web, desktop...) can
poll and render.
"""

import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

# pymavlink 2.4.50+ defaults to the "all" dialect - ask for ArduPilot's, the
# default of earlier versions and what this code is built and tested
# against. Must happen before pymavlink is imported.
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")

from pymavlink import mavutil

MAV_AUTOPILOT = {v: k for k, v in vars(mavutil.mavlink).items() if k.startswith("MAV_AUTOPILOT_")}
MAV_TYPE = {v: k for k, v in vars(mavutil.mavlink).items() if k.startswith("MAV_TYPE_")}
MAV_STATE = {v: k for k, v in vars(mavutil.mavlink).items() if k.startswith("MAV_STATE_")}
MAV_SEVERITY = {v: k.replace("MAV_SEVERITY_", "") for k, v in vars(mavutil.mavlink).items() if k.startswith("MAV_SEVERITY_")}

# SYS_STATUS sensor-health bits we care about, mapped to short display names
# (matches mavlink's MAV_SYS_STATUS_SENSOR_* bitmask; anything not listed here
# is skipped rather than guessed at).
SENSOR_HEALTH_BITS = {
    "MAV_SYS_STATUS_SENSOR_3D_GYRO": "gyro",
    "MAV_SYS_STATUS_SENSOR_3D_ACCEL": "accel",
    "MAV_SYS_STATUS_SENSOR_3D_MAG": "mag",
    "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE": "baro",
    "MAV_SYS_STATUS_SENSOR_GPS": "gps",
    "MAV_SYS_STATUS_SENSOR_RC_RECEIVER": "rc",
    "MAV_SYS_STATUS_AHRS": "ahrs",
    "MAV_SYS_STATUS_SENSOR_BATTERY": "battery",
    "MAV_SYS_STATUS_PREARM_CHECK": "prearm",
    "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS": "motor_outputs",
    "MAV_SYS_STATUS_TERRAIN": "terrain",
    "MAV_SYS_STATUS_LOGGING": "logging",
    "MAV_SYS_STATUS_GEOFENCE": "geofence",
}

# EKF_STATUS_REPORT.flags bits (ArduPilot-specific), mapped to short names.
# For EKF_GOOD_BITS, a SET bit means that estimate is healthy, so we flag it
# as unhealthy when the bit is CLEAR. EKF_BAD_BITS are the opposite: these
# flag a degraded state, so a SET bit IS the problem (e.g. CONST_POS_MODE
# means the EKF has given up on a real position estimate).
EKF_GOOD_BITS = {
    "EKF_ATTITUDE": "attitude",
    "EKF_VELOCITY_HORIZ": "vel_horiz",
    "EKF_VELOCITY_VERT": "vel_vert",
    "EKF_POS_HORIZ_REL": "pos_horiz_rel",
    "EKF_POS_HORIZ_ABS": "pos_horiz_abs",
    "EKF_POS_VERT_ABS": "pos_vert_abs",
    "EKF_POS_VERT_AGL": "pos_vert_agl",
    "EKF_PRED_POS_HORIZ_REL": "pred_pos_horiz_rel",
    "EKF_PRED_POS_HORIZ_ABS": "pred_pos_horiz_abs",
}
EKF_BAD_BITS = {
    "EKF_CONST_POS_MODE": "const_pos_mode",
    "EKF_UNINITIALIZED": "uninitialized",
}

# Explicit per-message rates (Hz) requested via MAV_CMD_SET_MESSAGE_INTERVAL
# once connected. A blanket REQUEST_DATA_STREAM doesn't reliably turn these on
# for every ArduPilot version/link; asking for each message by id does.
STREAM_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS: 4,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 2,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 5,
}

# Sane bounds for a raw RC PWM value; anything outside this is treated as "no
# data" rather than displayed as a bogus number.
RC_PWM_MIN = 800
RC_PWM_MAX = 2200

# GPS_RAW_INT.fix_type -> short label.
GPS_FIX_LABELS = {
    0: "NO GPS",
    1: "NO FIX",
    2: "2D FIX",
    3: "3D FIX",
    4: "DGPS",
    5: "RTK FLOAT",
    6: "RTK FIXED",
    7: "STATIC",
    8: "PPP",
}

MAG_CAL_STATUS_LABELS = {
    0: "NOT_STARTED",
    1: "WAITING_TO_START",
    2: "RUNNING_STEP_ONE",
    3: "RUNNING_STEP_TWO",
    4: "SUCCESS",
    5: "FAILED",
    6: "BAD_ORIENTATION",
    7: "BAD_RADIUS",
}

# Newer MAVLink (pymavlink 2.4.50+) renamed 6/7 to FAILED_ORIENTATION /
# FAILED_RADIUS and added more specific failures.
MAG_CAL_STATUS_LABELS.update({8: "FAILED_OFFSETS", 9: "FAILED_DIAG_SCALING", 10: "FAILED_RESIDUALS_HIGH"})

# By number, not name: the numbers are the same in every pymavlink version,
# the names aren't (see above). 4 = success, 5 and up = failed.
MAG_CAL_TERMINAL_STATUSES = {
    4,  # SUCCESS
    5,  # FAILED
    6,  # BAD_ORIENTATION / FAILED_ORIENTATION
    7,  # BAD_RADIUS / FAILED_RADIUS
    8,  # FAILED_OFFSETS (newer MAVLink)
    9,  # FAILED_DIAG_SCALING (newer MAVLink)
    10,  # FAILED_RESIDUALS_HIGH (newer MAVLink)
}

CALIBRATION_KINDS = ("accel", "compass")


def is_autopilot_heartbeat(msg) -> bool:
    """True for a heartbeat from the flight controller itself, not a GCS/hub.

    On a MAVProxy-hubbed link, MAVProxy's own periodic heartbeat (and any
    other GCS on the same link) arrives on the same stream as the aircraft's.
    Treating one of those as "the vehicle" would show a GCS's system/component
    id instead of the FCU's, which then makes every targeted command (and
    stream request) go to the wrong place.
    """
    return (
        getattr(msg, "type", None) != mavutil.mavlink.MAV_TYPE_GCS
        and getattr(msg, "autopilot", None) != mavutil.mavlink.MAV_AUTOPILOT_INVALID
    )


@dataclass(frozen=True)
class HeartbeatSnapshot:
    """A point-in-time, read-only view of the heartbeat link."""

    connected: bool
    last_heartbeat_age: Optional[float]
    system_id: Optional[int]
    component_id: Optional[int]
    autopilot: Optional[int]
    vehicle_type: Optional[int]
    system_status: Optional[int]
    armed: bool
    mode_name: Optional[str]
    heartbeat_count: int
    hz: float
    last_error: Optional[str]

    @property
    def autopilot_name(self) -> str:
        return MAV_AUTOPILOT.get(self.autopilot, str(self.autopilot)) if self.autopilot is not None else "-"

    @property
    def vehicle_type_name(self) -> str:
        return MAV_TYPE.get(self.vehicle_type, str(self.vehicle_type)) if self.vehicle_type is not None else "-"

    @property
    def system_status_name(self) -> str:
        return MAV_STATE.get(self.system_status, str(self.system_status)) if self.system_status is not None else "-"


@dataclass(frozen=True)
class AttitudeData:
    roll_deg: float
    pitch_deg: float
    yaw_deg: float


@dataclass(frozen=True)
class BatteryData:
    voltage_v: Optional[float]
    current_a: Optional[float]
    remaining_pct: Optional[int]


@dataclass(frozen=True)
class GpsData:
    fix_type: int
    satellites_visible: int

    @property
    def fix_label(self) -> str:
        return GPS_FIX_LABELS.get(self.fix_type, str(self.fix_type))


@dataclass(frozen=True)
class CompassCalProgress:
    compass_id: int
    completion_pct: int
    status: int

    @property
    def status_name(self) -> str:
        return MAG_CAL_STATUS_LABELS.get(self.status, str(self.status))


@dataclass(frozen=True)
class CalibrationSnapshot:
    """What's currently calibrating (accel or compass), if anything."""

    kind: Optional[str]  # "accel", "compass", or None if nothing has been started
    started_at: Optional[float]
    finished: bool
    success: Optional[bool]  # None while still in progress

    # accel: ArduPilot doesn't report a numeric percentage for this, only
    # STATUSTEXT prompts - so progress here is the latest prompt plus a count
    # of "place vehicle" prompts seen (approximate, not an exact fraction).
    accel_step_text: Optional[str]
    accel_step_count: int

    # compass: MAG_CAL_PROGRESS gives a real completion_pct per compass.
    compass_progress: Dict[int, CompassCalProgress]


@dataclass(frozen=True)
class StatusMessage:
    severity: int
    text: str
    timestamp: float

    @property
    def severity_name(self) -> str:
        return MAV_SEVERITY.get(self.severity, str(self.severity))


@dataclass(frozen=True)
class TelemetrySnapshot:
    """Everything besides the core heartbeat/connection status: attitude,
    sensor/EKF health, battery, RC inputs, servo outputs, and recent
    status-text messages."""

    attitude: Optional[AttitudeData]
    attitude_age: Optional[float]

    unhealthy_sensors: List[str]
    ekf_unhealthy: List[str]
    cpu_load_pct: Optional[float]
    battery: Optional[BatteryData]
    sys_status_age: Optional[float]

    rc_channels: List[Optional[int]]  # raw PWM values, index 0 = channel 1; None = no data
    rc_rssi: Optional[int]
    rc_age: Optional[float]

    servo_channels: List[int]  # raw PWM values, index 0 = servo/motor 1
    servo_age: Optional[float]

    gps: Optional[GpsData]
    gps_age: Optional[float]

    recent_messages: List[StatusMessage]  # newest last


class NotificationReceiver:
    """Receives plain-UDP notifications from other programs on this machine
    (e.g. geofence_udp_listener.py) for the GUI's message log.

    Needed because MAVLink can't carry them here: MAVProxy forwards what a
    program on one of its output ports sends ONLY to the FCU, never to its
    other output ports, and ArduPilot doesn't send a message back down the
    link it came in on. A STATUSTEXT from the listener never reaches this GUI.

    Each datagram is JSON: {"severity": <MAV_SEVERITY int>, "text": "..."}.
    Bound to 127.0.0.1 only, so nothing off this machine can post to it.
    """

    def __init__(self, port: int):
        self.port = port
        self._lock = threading.Lock()
        self._pending: List[Tuple[int, str]] = []
        self._stop_event = threading.Event()
        self._sock: Optional[socket.socket] = None

    def start(self) -> Optional[str]:
        """Bind and start receiving. Returns an error message instead of
        raising, so a busy port costs the notifications, not the whole GUI."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("127.0.0.1", self.port))
        except OSError as e:
            return f"notification port {self.port} unavailable ({e}) - fence notifications won't show"
        sock.settimeout(0.5)
        self._sock = sock
        threading.Thread(target=self._run, daemon=True).start()
        return None

    def stop(self) -> None:
        self._stop_event.set()

    def drain(self) -> List[Tuple[int, str]]:
        """Messages received since the last call, oldest first."""
        with self._lock:
            messages, self._pending = self._pending, []
        return messages

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            try:
                msg = json.loads(data.decode("utf-8"))
                severity = int(msg.get("severity", mavutil.mavlink.MAV_SEVERITY_INFO))
                text = str(msg["text"])
            except (ValueError, KeyError, TypeError, UnicodeDecodeError, AttributeError):
                continue
            with self._lock:
                self._pending.append((severity, text))
        self._sock.close()


class MavproxyLaunchError(RuntimeError):
    pass


class MavproxyProcess:
    """Launches and manages a MAVProxy subprocess that bridges the FCU link to a
    UDP output this GUI (or anything else) can read, so the user doesn't have
    to start MAVProxy by hand in a separate terminal first.

    Runs MAVProxy in --daemon mode (no interactive console of its own) since
    this GUI is the interactive front-end.
    """

    def __init__(self, master: str, out: str, extra_args: Optional[List[str]] = None):
        self.master = master
        self.out = out
        self.extra_args = extra_args or []
        self._process: Optional[subprocess.Popen] = None

    def start(self) -> None:
        if self._process is not None:
            return

        mavproxy_cmd = shutil.which("mavproxy.py") or shutil.which("mavproxy")
        if mavproxy_cmd is None:
            raise MavproxyLaunchError(
                "mavproxy.py was not found on PATH. Install it with 'pip3 install MAVProxy' "
                "or activate the environment it's installed in."
            )

        cmd = [mavproxy_cmd, f"--master={self.master}", f"--out={self.out}", "--daemon", *self.extra_args]
        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL,
            )
        except OSError as e:
            raise MavproxyLaunchError(f"failed to start mavproxy: {e}") from e

    def stop(self) -> None:
        if self._process is None:
            return
        self._process.terminate()
        try:
            self._process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._process.kill()
            self._process.wait(timeout=5)
        self._process = None

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None


class ScriptProcess:
    """Runs another Python script (e.g. geofence_udp_listener.py) as its own
    subprocess, so a GUI can start it without the two codes being combined.
    The script's output goes to a log file, since it can't share the GUI's
    terminal.
    """

    def __init__(self, script_path: str, args: List[str], log_path: str):
        self.script_path = script_path
        self.args = args
        self.log_path = log_path
        self._process: Optional[subprocess.Popen] = None
        self._log_file = None

    def start(self) -> Optional[str]:
        """Start (or restart, after it exited) the script. Returns an error
        message instead of raising, so the caller can just display it."""
        if self.is_running():
            return None
        if not os.path.isfile(self.script_path):
            return f"{self.script_path} not found - is geofence-tools/ next to drone-heartbeat-gui/?"
        self._close_log()
        try:
            self._log_file = open(self.log_path, "a", encoding="utf-8")
            self._process = subprocess.Popen(
                [sys.executable, "-u", self.script_path, *self.args],
                stdout=self._log_file,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
            )
        except OSError as e:
            self._close_log()
            return f"failed to start {os.path.basename(self.script_path)}: {e}"
        return None

    def stop(self) -> None:
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=5)
        self._process = None
        self._close_log()

    def is_running(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def exit_code(self) -> Optional[int]:
        """The exit code once it has stopped, None while running or never started."""
        return self._process.poll() if self._process is not None else None

    def _close_log(self) -> None:
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None


class HeartbeatMonitor:
    """Connects to a MAVLink endpoint and tracks HEARTBEAT status on a background thread."""

    def __init__(self, connection_string: str, heartbeat_timeout: float = 3.0):
        self.connection_string = connection_string
        self.heartbeat_timeout = heartbeat_timeout

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._reconnect_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._master = None  # the active mavutil connection, set while connected

        self._connected = False
        self._last_heartbeat_time: Optional[float] = None
        self._system_id: Optional[int] = None
        self._component_id: Optional[int] = None
        self._autopilot: Optional[int] = None
        self._vehicle_type: Optional[int] = None
        self._system_status: Optional[int] = None
        self._armed = False
        self._mode_name: Optional[str] = None
        self._heartbeat_count = 0
        self._hz = 0.0
        self._hz_window: List[float] = []
        self._last_error: Optional[str] = None
        self._streams_requested = False

        self._attitude: Optional[AttitudeData] = None
        self._attitude_time: Optional[float] = None

        self._unhealthy_sensors: List[str] = []
        self._ekf_unhealthy: List[str] = []
        self._cpu_load_pct: Optional[float] = None
        self._battery: Optional[BatteryData] = None
        self._sys_status_time: Optional[float] = None

        self._rc_channels: List[Optional[int]] = []
        self._rc_rssi: Optional[int] = None
        self._rc_time: Optional[float] = None

        self._servo_channels: List[int] = []
        self._servo_time: Optional[float] = None

        self._gps: Optional[GpsData] = None
        self._gps_time: Optional[float] = None

        self._cal_kind: Optional[str] = None
        self._cal_started_at: Optional[float] = None
        self._cal_finished = False
        self._cal_success: Optional[bool] = None
        self._cal_accel_step_text: Optional[str] = None
        self._cal_accel_step_count = 0
        self._cal_compass_mask = 0
        self._cal_compass_progress: Dict[int, CompassCalProgress] = {}

        self._recent_messages: List[StatusMessage] = []
        self._max_messages = 50

        # Extra per-message callbacks (see add_message_listener) - the GUI
        # doesn't use these; other scripts built on this monitor do.
        self._message_listeners: List[Callable[[object], None]] = []

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._reconnect_event.set()

    def add_message_listener(self, callback: Callable[[object], None]) -> None:
        """Also hand every received MAVLink message to callback (called on the
        link thread). Lets another script reuse this monitor's single
        connection and heartbeat tracking instead of opening its own."""
        self._message_listeners.append(callback)

    def active_link(self) -> Tuple[object, Optional[int], Optional[int]]:
        """(mavutil connection or None, FCU system id, FCU component id), for
        sending messages this class has no dedicated method for."""
        with self._lock:
            return self._master, self._system_id, self._component_id

    def request_reconnect(self, connection_string: Optional[str] = None) -> None:
        """Drop the current link and reopen it, optionally on a new connection string."""
        with self._lock:
            if connection_string:
                self.connection_string = connection_string
            self._last_error = None
        self._reconnect_event.set()

    def send_arm(self, arm: bool) -> bool:
        """Send a MAV_CMD_COMPONENT_ARM_DISARM. Returns False if there's no active link."""
        with self._lock:
            master = self._master
            target_system = self._system_id or 1
            target_component = self._component_id or 1
        if master is None:
            return False
        master.mav.command_long_send(
            target_system,
            target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1 if arm else 0,
            0, 0, 0, 0, 0, 0,
        )
        return True

    def send_mode(self, mode_name: str) -> Tuple[bool, str]:
        """Request a flight-mode change by name (e.g. GUIDED, LOITER, HOLD).

        Deliberately doesn't use pymavlink's master.set_mode()/mode_mapping():
        those key off pymavlink's own internal "first heartbeat wins" sysid
        lock, which a GCS/hub heartbeat reaching us before the real FCU's
        could win. Using our own filtered system/component id and vehicle
        type (both already gated through is_autopilot_heartbeat) keeps this
        targeted at the FCU no matter what else is on the link.
        """
        with self._lock:
            master = self._master
            target_system = self._system_id
            target_component = self._component_id
            vehicle_type = self._vehicle_type
        if master is None or target_system is None or target_component is None:
            return False, "not connected"

        mode_name = mode_name.upper()
        mapping = mavutil.mode_mapping_byname(vehicle_type) if vehicle_type is not None else None

        if not mapping or mode_name not in mapping:
            available = ", ".join(sorted(mapping)) if mapping else "none yet (wait for a heartbeat)"
            return False, f"unknown mode '{mode_name}' - available: {available}"

        try:
            master.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mapping[mode_name],
                0, 0, 0, 0, 0,
            )
        except Exception as e:
            return False, f"failed to send mode change: {e}"
        return True, f"mode change to {mode_name} requested"

    def start_calibration(self, kind: str) -> Tuple[bool, str]:
        """Start an accelerometer or compass calibration.

        accel: MAV_CMD_PREFLIGHT_CALIBRATION (param5=1) - a standard 6-position
        calibration. ArduPilot drives it entirely through STATUSTEXT prompts
        ("Place vehicle on its LEFT side...") with no numeric progress field,
        so progress here is inferred from counting those prompts.

        compass: MAV_CMD_DO_START_MAG_CAL - ArduPilot's onboard calibration,
        which does report real progress via MAG_CAL_PROGRESS/MAG_CAL_REPORT.
        """
        kind = kind.lower()
        if kind not in CALIBRATION_KINDS:
            return False, f"unknown calibration '{kind}' - available: {', '.join(CALIBRATION_KINDS)}"

        with self._lock:
            master = self._master
            target_system = self._system_id
            target_component = self._component_id
            if self._cal_kind is not None and not self._cal_finished:
                return False, f"a {self._cal_kind} calibration is already in progress"
        if master is None or target_system is None or target_component is None:
            return False, "not connected"

        try:
            if kind == "accel":
                master.mav.command_long_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
                    0,
                    0, 0, 0, 0,
                    1,  # param5: start accelerometer calibration
                    0, 0,
                )
            else:
                master.mav.command_long_send(
                    target_system,
                    target_component,
                    mavutil.mavlink.MAV_CMD_DO_START_MAG_CAL,
                    0,
                    0,  # param1: compass bitmask, 0 = all enabled compasses
                    1,  # param2: retry on failure
                    1,  # param3: autosave
                    0, 0, 0, 0,
                )
        except Exception as e:
            return False, f"failed to start calibration: {e}"

        with self._lock:
            self._cal_kind = kind
            self._cal_started_at = time.time()
            self._cal_finished = False
            self._cal_success = None
            self._cal_accel_step_text = None
            self._cal_accel_step_count = 0
            self._cal_compass_mask = 0
            self._cal_compass_progress = {}

        return True, f"{kind} calibration started"

    def calibration_snapshot(self) -> CalibrationSnapshot:
        """Thread-safe, read-only view of the current/last calibration."""
        with self._lock:
            return CalibrationSnapshot(
                kind=self._cal_kind,
                started_at=self._cal_started_at,
                finished=self._cal_finished,
                success=self._cal_success,
                accel_step_text=self._cal_accel_step_text,
                accel_step_count=self._cal_accel_step_count,
                compass_progress=dict(self._cal_compass_progress),
            )

    def send_reboot(self, force: bool = False) -> Tuple[bool, str]:
        """Reboot the flight controller. Refused while armed unless force=True."""
        with self._lock:
            master = self._master
            target_system = self._system_id
            target_component = self._component_id
            armed = self._armed
        if master is None or target_system is None or target_component is None:
            return False, "not connected"
        if armed and not force:
            return False, "refused: vehicle is armed - disarm first"

        try:
            master.mav.command_long_send(
                target_system,
                target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0,
                1,  # param1: reboot autopilot
                0, 0, 0, 0, 0, 0,
            )
        except Exception as e:
            return False, f"failed to send reboot: {e}"
        return True, "reboot command sent - the link will drop and reconnect"

    def snapshot(self) -> HeartbeatSnapshot:
        """Thread-safe, read-only view of the current link status."""
        now = time.time()
        with self._lock:
            age = (now - self._last_heartbeat_time) if self._last_heartbeat_time is not None else None
            connected = self._connected and age is not None and age <= self.heartbeat_timeout
            return HeartbeatSnapshot(
                connected=connected,
                last_heartbeat_age=age,
                system_id=self._system_id,
                component_id=self._component_id,
                autopilot=self._autopilot,
                vehicle_type=self._vehicle_type,
                system_status=self._system_status,
                armed=self._armed,
                mode_name=self._mode_name,
                heartbeat_count=self._heartbeat_count,
                hz=self._hz,
                last_error=self._last_error,
            )

    def _run(self) -> None:
        while not self._stop_event.is_set():
            self._reconnect_event.clear()
            try:
                master = mavutil.mavlink_connection(self.connection_string)
            except Exception as e:
                with self._lock:
                    self._last_error = f"Connection failed: {e}"
                    self._connected = False
                time.sleep(2)
                continue

            with self._lock:
                self._master = master
                self._last_error = None
                self._connected = False
                self._last_heartbeat_time = None
                self._streams_requested = False

            while not self._stop_event.is_set() and not self._reconnect_event.is_set():
                msg = master.recv_match(blocking=True, timeout=1.0)
                now = time.time()

                if msg is None:
                    with self._lock:
                        if self._last_heartbeat_time is not None and now - self._last_heartbeat_time > self.heartbeat_timeout:
                            self._connected = False
                    continue

                for listener in list(self._message_listeners):
                    try:
                        listener(msg)
                    except Exception:
                        pass  # a broken listener must never take down the link thread

                msg_type = msg.get_type()
                if msg_type == "HEARTBEAT":
                    if is_autopilot_heartbeat(msg):
                        self._handle_heartbeat(msg, now)
                        self._request_streams_once(master)
                elif msg_type == "ATTITUDE":
                    self._handle_attitude(msg, now)
                elif msg_type == "SYS_STATUS":
                    self._handle_sys_status(msg, now)
                elif msg_type == "EKF_STATUS_REPORT":
                    self._handle_ekf_status(msg)
                elif msg_type in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
                    self._handle_rc_channels(msg, now)
                elif msg_type == "SERVO_OUTPUT_RAW":
                    self._handle_servo_output(msg, now)
                elif msg_type == "GPS_RAW_INT":
                    self._handle_gps(msg, now)
                elif msg_type == "MAG_CAL_PROGRESS":
                    self._handle_mag_cal_progress(msg)
                elif msg_type == "MAG_CAL_REPORT":
                    self._handle_mag_cal_report(msg)
                elif msg_type == "STATUSTEXT":
                    self._handle_statustext(msg, now)

            with self._lock:
                self._master = None
            try:
                master.close()
            except Exception:
                pass

    def _request_streams_once(self, master) -> None:
        with self._lock:
            if self._streams_requested:
                return
            self._streams_requested = True
            system_id = self._system_id
            component_id = self._component_id
        if system_id is None or component_id is None:
            return
        for message_id, hz in STREAM_RATES_HZ.items():
            try:
                master.mav.command_long_send(
                    system_id,
                    component_id,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    message_id,
                    1_000_000.0 / hz,
                    0, 0, 0, 0, 0,
                )
            except Exception:
                pass

    def _handle_heartbeat(self, msg, now: float) -> None:
        try:
            mode_name = mavutil.mode_string_v10(msg)
        except Exception:
            mode_name = None

        with self._lock:
            self._connected = True
            self._last_heartbeat_time = now
            self._system_id = msg.get_srcSystem()
            self._component_id = msg.get_srcComponent()
            self._autopilot = msg.autopilot
            self._vehicle_type = msg.type
            self._system_status = msg.system_status
            self._armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            self._mode_name = mode_name
            self._heartbeat_count += 1

            self._hz_window.append(now)
            cutoff = now - 5.0
            self._hz_window = [t for t in self._hz_window if t >= cutoff]
            if len(self._hz_window) >= 2:
                self._hz = (len(self._hz_window) - 1) / (self._hz_window[-1] - self._hz_window[0])

    def _handle_attitude(self, msg, now: float) -> None:
        attitude = AttitudeData(
            roll_deg=math.degrees(msg.roll),
            pitch_deg=math.degrees(msg.pitch),
            yaw_deg=math.degrees(msg.yaw),
        )
        with self._lock:
            self._attitude = attitude
            self._attitude_time = now

    def _handle_sys_status(self, msg, now: float) -> None:
        enabled = msg.onboard_control_sensors_enabled
        health = msg.onboard_control_sensors_health
        unhealthy = []
        for const_name, short_name in SENSOR_HEALTH_BITS.items():
            bit = getattr(mavutil.mavlink, const_name, None)
            if bit is None:
                continue
            if (enabled & bit) and not (health & bit):
                unhealthy.append(short_name)

        voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
        current = msg.current_battery / 100.0 if msg.current_battery != -1 else None
        remaining = msg.battery_remaining if msg.battery_remaining != -1 else None

        with self._lock:
            self._unhealthy_sensors = unhealthy
            self._cpu_load_pct = msg.load / 10.0
            self._battery = BatteryData(voltage_v=voltage, current_a=current, remaining_pct=remaining)
            self._sys_status_time = now

    def _handle_ekf_status(self, msg) -> None:
        flags = msg.flags
        unhealthy = []
        for const_name, short_name in EKF_GOOD_BITS.items():
            bit = getattr(mavutil.mavlink, const_name, None)
            if bit is None:
                continue
            if not (flags & bit):
                unhealthy.append(short_name)
        for const_name, short_name in EKF_BAD_BITS.items():
            bit = getattr(mavutil.mavlink, const_name, None)
            if bit is None:
                continue
            if flags & bit:
                unhealthy.append(short_name)
        with self._lock:
            self._ekf_unhealthy = unhealthy

    def _handle_rc_channels(self, msg, now: float) -> None:
        # RC_CHANNELS carries its own chancount; RC_CHANNELS_RAW (older/deprecated,
        # still sent by some setups) always has exactly 8. Either way, a channel
        # with no receiver connected reads as 0 or UINT16_MAX (65535), and noise
        # outside the plausible PWM range is treated as "no data" rather than
        # shown as a bogus number - matches RC_PWM_MIN/MAX.
        count = getattr(msg, "chancount", None) or 8
        channels: List[Optional[int]] = []
        for i in range(1, count + 1):
            value = getattr(msg, f"chan{i}_raw", None)
            if value is None or value in (0, 65535) or not (RC_PWM_MIN <= value <= RC_PWM_MAX):
                channels.append(None)
            else:
                channels.append(value)
        with self._lock:
            self._rc_channels = channels
            self._rc_rssi = getattr(msg, "rssi", None)
            self._rc_time = now

    def _handle_servo_output(self, msg, now: float) -> None:
        channels = []
        i = 1
        while True:
            value = getattr(msg, f"servo{i}_raw", None)
            if value is None:
                break
            channels.append(value)
            i += 1
        with self._lock:
            self._servo_channels = channels
            self._servo_time = now

    def _handle_gps(self, msg, now: float) -> None:
        gps = GpsData(fix_type=int(msg.fix_type), satellites_visible=int(msg.satellites_visible))
        with self._lock:
            self._gps = gps
            self._gps_time = now

    def _handle_statustext(self, msg, now: float) -> None:
        text = msg.text
        if isinstance(text, bytes):
            text = text.decode("utf-8", errors="replace")
        text = text.rstrip("\x00")
        entry = StatusMessage(severity=msg.severity, text=text, timestamp=now)
        with self._lock:
            self._recent_messages.append(entry)
            if len(self._recent_messages) > self._max_messages:
                self._recent_messages = self._recent_messages[-self._max_messages:]

            # ArduPilot drives accel calibration purely through STATUSTEXT
            # prompts - no numeric progress field exists for it.
            if self._cal_kind == "accel" and not self._cal_finished:
                lowered = text.lower()
                if "place vehicle" in lowered:
                    self._cal_accel_step_count += 1
                    self._cal_accel_step_text = text
                elif "calibration successful" in lowered:
                    self._cal_finished = True
                    self._cal_success = True
                    self._cal_accel_step_text = text
                elif "calibration failed" in lowered or "calibration cancelled" in lowered:
                    self._cal_finished = True
                    self._cal_success = False
                    self._cal_accel_step_text = text

    def _handle_mag_cal_progress(self, msg) -> None:
        progress = CompassCalProgress(
            compass_id=int(msg.compass_id),
            completion_pct=int(msg.completion_pct),
            status=int(msg.cal_status),
        )
        with self._lock:
            if self._cal_kind != "compass":
                return
            self._cal_compass_mask |= int(msg.cal_mask)
            self._cal_compass_progress[progress.compass_id] = progress

    def _handle_mag_cal_report(self, msg) -> None:
        with self._lock:
            if self._cal_kind != "compass":
                return
            self._cal_compass_mask |= int(msg.cal_mask)
            self._cal_compass_progress[int(msg.compass_id)] = CompassCalProgress(
                compass_id=int(msg.compass_id),
                completion_pct=100,
                status=int(msg.cal_status),
            )
            expected_ids = [i for i in range(8) if self._cal_compass_mask & (1 << i)]
            if expected_ids and all(
                i in self._cal_compass_progress and self._cal_compass_progress[i].status in MAG_CAL_TERMINAL_STATUSES
                for i in expected_ids
            ):
                self._cal_finished = True
                self._cal_success = all(
                    self._cal_compass_progress[i].status == mavutil.mavlink.MAG_CAL_SUCCESS for i in expected_ids
                )

    def telemetry_snapshot(self) -> TelemetrySnapshot:
        """Thread-safe, read-only view of everything beyond core heartbeat status."""
        now = time.time()
        with self._lock:
            return TelemetrySnapshot(
                attitude=self._attitude,
                attitude_age=(now - self._attitude_time) if self._attitude_time is not None else None,
                unhealthy_sensors=list(self._unhealthy_sensors),
                ekf_unhealthy=list(self._ekf_unhealthy),
                cpu_load_pct=self._cpu_load_pct,
                battery=self._battery,
                sys_status_age=(now - self._sys_status_time) if self._sys_status_time is not None else None,
                rc_channels=list(self._rc_channels),
                rc_rssi=self._rc_rssi,
                rc_age=(now - self._rc_time) if self._rc_time is not None else None,
                servo_channels=list(self._servo_channels),
                servo_age=(now - self._servo_time) if self._servo_time is not None else None,
                gps=self._gps,
                gps_age=(now - self._gps_time) if self._gps_time is not None else None,
                recent_messages=list(self._recent_messages),
            )
