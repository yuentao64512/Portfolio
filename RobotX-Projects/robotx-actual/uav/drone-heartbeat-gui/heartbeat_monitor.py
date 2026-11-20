#!/usr/bin/env python3
"""
Terminal GUI for the FCU <-> Orin NX MAVLink heartbeat link, with a command
line for controlling the monitor and sending basic MAVLink commands.

Run this on the Orin NX while MAVProxy is already bridging the Holybro 6X
Pro FCU. Point it at one of MAVProxy's output endpoints (add one in the
MAVProxy console with `output add 127.0.0.1:14550` if you don't already
have a free one).

The MAVLink connection, HEARTBEAT decoding, and command sending all live in
heartbeat_link.py — this file only renders status and turns typed commands
into calls on that module.
"""

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from rich.console import Group
from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Input, RichLog, Static

from heartbeat_link import (
    CalibrationSnapshot,
    HeartbeatMonitor,
    HeartbeatSnapshot,
    MAV_SEVERITY,
    MavproxyLaunchError,
    MavproxyProcess,
    NotificationReceiver,
    ScriptProcess,
    TelemetrySnapshot,
)
from pymavlink import mavutil  # after heartbeat_link, which sets the dialect first

# camera_link.py lives in the sibling camera-capture-gui/ project (separate,
# standalone code per the original request) - add it to the path so it can
# be imported here without duplicating it.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "camera-capture-gui"))
from camera_link import CameraLink, CameraStatus  # noqa: E402

REFRESH_INTERVAL = 0.25  # seconds

# Outputs the auto-launched MAVProxy sends to besides this GUI's own:
# geofence listener, spare tools, Mission Planner on the GCS laptop.
DEFAULT_MAVPROXY_EXTRA_OUTS = ["127.0.0.1:14551", "127.0.0.1:14552", "192.168.1.100:14550", "127.0.0.1:14553"]
# ...and 14553: the flight recorder ('record'), so it can run during a mission (14552).

# The flight recorder is its own script in the sibling camera-capture-gui/
# folder; 'record' only launches it as a separate process.
RECORDER_SCRIPT = Path(__file__).resolve().parent.parent / "camera-capture-gui" / "flight_recorder.py"

# The geofence listener is its own script in the sibling geofence-tools/
# folder; this GUI only launches it as a separate process.
GEOFENCE_DIR = Path(__file__).resolve().parent.parent / "geofence-tools"
LISTENER_SCRIPT = GEOFENCE_DIR / "geofence_udp_listener.py"
LISTENER_RETRY_S = 10.0  # wait this long before restarting a listener that stopped
MISSION_CLEAR_TIMEOUT_S = 5.0  # 'mission clear': wait this long for the FCU's MISSION_ACK
# 'fence preview': ask again after FENCE_PREVIEW_RESEND_S without a reply, give up after FENCE_PREVIEW_TIMEOUT_S.
FENCE_PREVIEW_RESEND_S = 1.0
FENCE_PREVIEW_TIMEOUT_S = 5.0
# Altitude line: GLOBAL_POSITION_INT (above home) + a downward DISTANCE_SENSOR
# if fitted. Asked for at this rate when it isn't arriving, retried every
# ALTITUDE_RETRY_S; a reading older than ALTITUDE_STALE_S isn't shown.
ALTITUDE_STREAM_HZ = 4
ALTITUDE_RETRY_S = 10.0
ALTITUDE_STALE_S = 3.0

# The lawnmower mission is its own script in the sibling lawnmower-mission/
# folder; the 'mission' command only launches it as a separate process.
MISSION_SCRIPT = Path(__file__).resolve().parent.parent / "lawnmower-mission" / "lawnmower_mission.py"
# 'mission2' / 'mission3': the same mission at other altitudes/speeds, each its own script.
EXTRA_MISSIONS = {
    "mission2": (MISSION_SCRIPT.with_name("mission2.py"), "5 m", "10 m, 3 m/s", "15 s"),
    "mission3": (MISSION_SCRIPT.with_name("mission3.py"), "15 m", "7 m/s", "30 s"),
    "task4": (MISSION_SCRIPT.with_name("task4_mission.py"), "5 m", "10 m, 3 m/s", "15 s"),
}
# RoboCommand's RunStart (shown in the log by the geofence listener) starts this
# mission by itself: arm, hover 5 m for 15 s, mission2's lawnmower at 3 m/s.
RUNSTART_MISSION_SCRIPT = MISSION_SCRIPT.with_name("runstart_mission.py")
RUNSTART_TASK4_SCRIPT = MISSION_SCRIPT.with_name("task4_mission.py")  # 'runstart task4': same flight, reacts to Task 4
RUNSTART_PREFIX = "RoboCommand: RUN START - run "
# 'quit clear': deletes the FCU's mission + fence through the spare MAVProxy
# output, after the other scripts have stopped and before MAVProxy does.
FCU_CLEAR_SCRIPT = Path(__file__).resolve().parent / "fcu_clear.py"
FCU_CLEAR_CONNECTION = "udp:127.0.0.1:14552"

SEVERITY_STYLE = {
    0: "bold white on red",  # EMERGENCY
    1: "bold red",  # ALERT
    2: "bold red",  # CRITICAL
    3: "red",  # ERROR
    4: "yellow",  # WARNING
    5: "yellow",  # NOTICE
    6: "white",  # INFO
    7: "dim",  # DEBUG
}

PWM_BAR_MIN = 1000
PWM_BAR_MAX = 2000
PWM_BAR_WIDTH = 20
SERVO_CHANNELS_SHOWN = 4  # this airframe only has 4 motors (M1-M4)

HELP_TEXT = """\
Commands:
  help                 show this list
  reconnect            drop and reopen the current link
  connect <string>      switch to a different MAVLink connection string
  timeout <seconds>    change the heartbeat-lost timeout
  arm                  arm the vehicle (press Enter again, blank, to confirm)
  disarm               disarm the vehicle
  mode                 show the current flight mode
  mode <name>          request a flight-mode change (e.g. mode GUIDED)
  <name>               shortcut: just type a mode name directly (e.g. GUIDED)
  calibrate            show a numbered menu - then just type 1 or 2
  calibrate <type>     start a calibration directly (accel, compass)
  reboot               reboot the flight controller (press Enter again to confirm)
  camera               open the USB camera
  camera stop          close it (frees the device for another program)
  photo                save a photo (after 'camera' has been opened)
  photo <name>         save a photo under a specific filename
  mission              fly the lawnmower mission (press Enter again to confirm)
  mission preview      plan + upload the mission only, no flying (view it in Mission Planner)
  mission stop         stop the mission script (use the RC to take over the drone itself)
  mission clear        delete the mission stored on the FCU (press Enter again to confirm) - fence/rally kept
  fence clear          delete all fence points stored on the FCU (press Enter again to confirm) - mission/rally kept
  fence preview        read the fence stored on the FCU and show its coordinates (read only - nothing changes)
  mission2             hover 5 m for 15 s, lawnmower at 10 m AGL, 3 m/s, CV signal (press Enter again to confirm)
  mission3             same lawnmower mission at 15 m AGL, 7 m/s (press Enter again to confirm)
  mission2/3 preview   plan + upload that mission only, no flying ('mission stop' stops either)
  task4                mission2's flight, and it flies to assistance requests and around keep-out zones by itself (confirms)
  task4 preview        plan + upload that mission only, no flying
  runstart             show what RoboCommand's RunStart will fly
  runstart task4       RunStart flies the task4 mission (until the GUI restarts)
  runstart normal      RunStart flies the normal RunStart mission (the default)
  record              record video + the FCU's flight data for every frame
  record stop          stop recording (finishes the video file)
  quit                 exit
  quit clear           delete the mission AND fence from the FCU, then exit (press Enter again to confirm)
"""

# Compact one-line-each version shown permanently in the dashboard (below),
# so commands are visible without having to type 'help' first.
COMMANDS_SUMMARY = [
    ("help", "full command list"),
    ("reconnect", "drop/reopen the link"),
    ("connect <string>", "switch connection"),
    ("timeout <seconds>", "heartbeat-lost timeout"),
    ("arm / disarm", "arm (confirms) / disarm"),
    ("mode [name]", "show/change flight mode"),
    ("<name>", "mode name directly, e.g. GUIDED"),
    ("calibrate", "numbered accel/compass menu"),
    ("reboot", "reboot the FCU (confirms)"),
    ("camera / camera stop", "open/close USB camera"),
    ("photo [name]", "save a photo"),
    ("mission", "lawnmower mission (confirms)"),
    ("mission preview/stop", "upload only / stop script"),
    ("mission clear", "delete FCU mission (confirms)"),
    ("fence clear", "delete FCU fence points (confirms)"),
    ("fence preview", "show FCU fence coordinates"),
    ("mission2 / mission3", "10 m 3 m/s / 15 m 7 m/s"),
    ("task4", "mission2 + reacts to Task 4 (confirms)"),
    ("runstart task4/normal", "what RunStart flies"),
    ("record / record stop", "video + FCU flight log"),
    ("quit", "exit"),
    ("quit clear", "delete FCU mission+fence, exit"),
]


def _section_header(title: str) -> Text:
    return Text(title, style="bold underline")


def render_status_section(snapshot: HeartbeatSnapshot, connection_string: str) -> Text:
    if snapshot.connected:
        status_word, status_style = "CONNECTED", "bold green"
    elif snapshot.last_error:
        status_word, status_style = "CONNECTION ERROR", "bold red"
    elif snapshot.last_heartbeat_age is None:
        status_word, status_style = "WAITING FOR HEARTBEAT", "bold yellow"
    else:
        status_word, status_style = "LOST", "bold red"

    out = Text()
    out.append_text(_section_header("STATUS"))
    out.append("\n")
    out.append("  MODE: ")
    out.append(snapshot.mode_name or "-", style="bold cyan")
    out.append("   ")
    out.append("ARMED" if snapshot.armed else "disarmed", style="bold red" if snapshot.armed else "dim")
    out.append("\n")
    out.append(f"  link {connection_string}   ")
    out.append(status_word, style=status_style)
    age = f"{snapshot.last_heartbeat_age:.1f}s ago" if snapshot.last_heartbeat_age is not None else "never"
    out.append(f"   last hb {age}   {snapshot.hz:.1f}Hz   {snapshot.heartbeat_count} seen\n")
    out.append(f"  sysid {snapshot.system_id if snapshot.system_id is not None else '-'}")
    out.append(f"  compid {snapshot.component_id if snapshot.component_id is not None else '-'}")
    out.append(f"  {snapshot.autopilot_name}  {snapshot.vehicle_type_name}  {snapshot.system_status_name}")
    if snapshot.last_error:
        out.append(f"\n  error: {snapshot.last_error}", style="red")
    return out


def render_attitude_health_section(telemetry: TelemetrySnapshot, altitude: "dict | None" = None) -> Text:
    out = Text()
    out.append_text(_section_header("ATTITUDE & HEALTH"))
    out.append("\n")

    if telemetry.attitude is not None:
        a = telemetry.attitude
        out.append(f"  roll {a.roll_deg:+6.1f}  pitch {a.pitch_deg:+6.1f}  yaw {a.yaw_deg:+6.1f}\n")
    else:
        out.append("  attitude: no data yet\n", style="dim")

    if altitude is not None:
        now = time.monotonic()
        rel, rng = altitude.get("rel"), altitude.get("range")
        out.append("  altitude: ")
        if rel is not None and now - rel[1] <= ALTITUDE_STALE_S:
            out.append(f"{rel[0]:.1f} m above home", style="bold")
        else:
            out.append("no data yet", style="dim")
        if rng is not None and now - rng[1] <= ALTITUDE_STALE_S:
            out.append(f"   rangefinder {rng[0]:.2f} m")
        out.append("\n")

    out.append("  gps: ")
    if telemetry.gps is not None:
        g = telemetry.gps
        if g.fix_type >= 4:
            gps_style = "green"
        elif g.fix_type == 3:
            gps_style = "green" if g.satellites_visible >= 8 else "yellow"
        else:
            gps_style = "bold red"
        out.append(f"{g.fix_label}  {g.satellites_visible} sats", style=gps_style)
        out.append("\n")
    else:
        out.append("no data yet\n", style="dim")

    out.append("  battery: ")
    if telemetry.battery is not None:
        b = telemetry.battery
        parts = []
        if b.voltage_v is not None:
            parts.append(f"{b.voltage_v:.2f}V")
        if b.current_a is not None:
            parts.append(f"{b.current_a:.1f}A")
        if b.remaining_pct is not None:
            parts.append(f"{b.remaining_pct}%")
        out.append(" / ".join(parts) if parts else "unknown", style=None if parts else "dim")
    else:
        out.append("no data yet", style="dim")
    out.append("   cpu ")
    out.append(f"{telemetry.cpu_load_pct:.0f}%" if telemetry.cpu_load_pct is not None else "-")
    out.append("\n")

    out.append("  sensors: ")
    if telemetry.sys_status_age is None:
        out.append("no data yet\n", style="dim")
    elif telemetry.unhealthy_sensors:
        out.append("UNHEALTHY: " + ", ".join(telemetry.unhealthy_sensors) + "\n", style="bold red")
    else:
        out.append("all healthy\n", style="green")

    out.append("  ekf: ")
    if telemetry.ekf_unhealthy:
        out.append("UNHEALTHY: " + ", ".join(telemetry.ekf_unhealthy), style="bold red")
    else:
        out.append("healthy", style="green")
    return out


def render_rc_section(telemetry: TelemetrySnapshot) -> Text:
    out = Text()
    out.append_text(_section_header("RC INPUTS"))
    out.append("\n")
    if not telemetry.rc_channels:
        out.append("  no RC data yet", style="dim")
        return out

    for i, value in enumerate(telemetry.rc_channels, start=1):
        if value is None:
            out.append(f"  ch{i:<2d}   -us |{'.' * PWM_BAR_WIDTH}|\n", style="dim")
            continue
        frac = max(0.0, min(1.0, (value - PWM_BAR_MIN) / (PWM_BAR_MAX - PWM_BAR_MIN)))
        filled = int(round(frac * PWM_BAR_WIDTH))
        bar = "#" * filled + "." * (PWM_BAR_WIDTH - filled)
        out.append(f"  ch{i:<2d} {value:4d}us |{bar}|\n")

    if telemetry.rc_rssi is not None and telemetry.rc_rssi != 255:
        out.append(f"  rssi {telemetry.rc_rssi}", style="dim")
    return out


def render_servos_section(telemetry: TelemetrySnapshot) -> Text:
    out = Text()
    out.append_text(_section_header("MOTOR / SERVO OUTPUTS"))
    out.append("\n")
    if not telemetry.servo_channels:
        out.append("  no motor/servo data yet", style="dim")
        return out

    for i, value in enumerate(telemetry.servo_channels[:SERVO_CHANNELS_SHOWN], start=1):
        frac = max(0.0, min(1.0, (value - PWM_BAR_MIN) / (PWM_BAR_MAX - PWM_BAR_MIN)))
        filled = int(round(frac * PWM_BAR_WIDTH))
        bar = "#" * filled + "." * (PWM_BAR_WIDTH - filled)
        out.append(f"  M{i:<2d} {value:4d}us |{bar}|\n")
    return out


def render_calibration_section(cal: CalibrationSnapshot) -> Text:
    out = Text()
    out.append_text(_section_header("CALIBRATION"))
    out.append("\n")

    out.append(f"  {cal.kind}: ")
    if cal.finished:
        out.append("SUCCESS" if cal.success else "FAILED", style="bold green" if cal.success else "bold red")
    else:
        out.append("IN PROGRESS", style="bold yellow")
    out.append("\n")

    if cal.kind == "accel":
        step_style = "dim" if not cal.accel_step_text else ("green" if cal.finished and cal.success else ("red" if cal.finished else "cyan"))
        out.append(f"  {cal.accel_step_text or 'waiting for the first prompt...'}\n", style=step_style)
        if not cal.finished:
            out.append(
                f"  step {cal.accel_step_count} of 6 (approximate - ArduPilot has no exact % for accel cal)",
                style="dim",
            )
    elif cal.kind == "compass":
        if not cal.compass_progress:
            out.append("  waiting for progress...", style="dim")
        else:
            lines = []
            for compass_id in sorted(cal.compass_progress):
                p = cal.compass_progress[compass_id]
                filled = int(round(p.completion_pct / 100 * PWM_BAR_WIDTH))
                bar = "#" * filled + "." * (PWM_BAR_WIDTH - filled)
                lines.append(f"  compass {compass_id} |{bar}| {p.completion_pct:3d}%  {p.status_name}")
            out.append("\n".join(lines))
    return out


def render_commands_section() -> Text:
    out = Text()
    out.append_text(_section_header("COMMANDS"))
    out.append("\n")
    for name, desc in COMMANDS_SUMMARY:
        out.append(f"  {name:<18}", style="bold")
        out.append(f"{desc}\n")
    return out


def render_camera_section(status: CameraStatus) -> Text:
    out = Text()
    out.append_text(_section_header("CAMERA"))
    out.append("\n")

    out.append(f"  device {status.device}   ")
    out.append("OPEN" if status.opened else "NOT OPEN", style="bold green" if status.opened else "bold red")
    out.append("\n")

    if status.width and status.height:
        out.append(f"  resolution {status.width}x{status.height}   ")
    else:
        out.append("  resolution -   ")
    out.append(f"fps {status.fps:.1f}   frames seen {status.frame_count}")

    if status.last_saved_path:
        out.append(f"\n  last photo: {status.last_saved_path}", style="green")
    if status.last_error:
        out.append(f"\n  error: {status.last_error}", style="red")
    return out


def render_recording_section(status: dict) -> Text:
    """Live status of flight_recorder.py, from the status file it keeps."""
    out = Text()
    out.append_text(_section_header("RECORDING"))
    out.append("\n")
    if status.get("state") != "recording":
        out.append("  starting camera..." if status.get("state") == "starting" else f"  {status.get('state', '?')}",
                   style="yellow")
        if status.get("error"):
            out.append(f"\n  error: {status['error']}", style="red")
        return out

    seconds = int(status.get("video_duration_s", 0))
    out.append("  REC ", style="bold white on red")
    out.append(" ")
    out.append(f"{seconds // 3600:02d}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}", style="bold")
    out.append(f"   {status.get('size_bytes', 0) / 1e6:.1f} MB   {status.get('frames', 0)} frames @ {status.get('fps', 0):g} fps")
    camera_fps = status.get("camera_fps", 0)
    if camera_fps and camera_fps < status.get("fps", 0) * 0.9:
        out.append(f" (camera {camera_fps:.0f})", style="yellow")
    out.append("\n")

    att, pos = status.get("attitude") or {}, status.get("position") or {}

    def val(d, key, unit):
        return f"{float(d[key]):.1f}{unit}" if d.get(key) not in (None, "") else "-"

    if status.get("fcu_connected") and att.get("roll_deg") not in (None, ""):
        out.append(f"  roll {val(att, 'roll_deg', '°')}  pitch {val(att, 'pitch_deg', '°')}  yaw {val(att, 'yaw_deg', '°')}  "
                   f"hdg {val(att, 'heading_deg', '°')}\n")
        out.append(f"  x {val(pos, 'x_north_m', ' m')}  y {val(pos, 'y_east_m', ' m')}  z {val(pos, 'z_down_m', ' m')}  "
                   f"alt {val(pos, 'alt_rel_home_m', ' m')}   FCU data in {status.get('rows_with_fcu', 0)}/{status.get('frames', 0)} frames")
    else:
        out.append("  NO FCU DATA - video only (check MAVProxy output 127.0.0.1:14553)", style="bold red")
    out.append(f"\n  {status.get('video', '')}", style="dim")
    return out


def render_dashboard(
    snapshot: HeartbeatSnapshot,
    telemetry: TelemetrySnapshot,
    connection_string: str,
    calibration: CalibrationSnapshot,
    camera: "CameraStatus | None" = None,
    recording: "dict | None" = None,
    altitude: "dict | None" = None,
) -> Group:
    parts = [
        render_status_section(snapshot, connection_string),
        Text(),
        render_attitude_health_section(telemetry, altitude),
        Text(),
        render_rc_section(telemetry),
        Text(),
        render_servos_section(telemetry),
    ]
    if calibration.kind is not None:
        parts += [Text(), render_calibration_section(calibration)]
    if camera is not None:
        parts += [Text(), render_camera_section(camera)]
    if recording is not None:
        parts += [Text(), render_recording_section(recording)]
    return Group(*parts)


class HeartbeatApp(App):
    CSS = """
    Screen {
        background: $surface;
    }
    #main-row {
        height: auto;
    }
    #dashboard {
        width: 1fr;
        height: auto;
        padding: 0 1;
    }
    #commands {
        width: 44;
        height: auto;
        padding: 0 1;
    }
    #log {
        height: 1fr;
        padding: 0 1;
        border-top: solid $panel;
    }
    #command {
        border: none;
        background: $surface;
    }
    """

    def __init__(
        self,
        connection_string: str,
        heartbeat_timeout: float,
        mavproxy_proc: "MavproxyProcess | None" = None,
        camera_device: str = "0",
        camera_output_dir: "Path | None" = None,
        camera_rotation_deg: float = 0.0,
        notify_port: "int | None" = None,
        listener_proc: "ScriptProcess | None" = None,
        runstart_mission: bool = True,
    ):
        super().__init__()
        self.monitor = HeartbeatMonitor(connection_string, heartbeat_timeout=heartbeat_timeout)
        self.notifications = NotificationReceiver(notify_port) if notify_port else None
        self.mavproxy_proc = mavproxy_proc
        # Started from refresh_status once the FCU heartbeat is seen - the
        # listener exits if it can't reach the FCU when it starts.
        self.listener_proc = listener_proc
        self._listener_was_running = False
        self._listener_retry_at = 0.0
        self._listener_last_error: str | None = None
        # Started by the 'mission' command; its progress arrives through the
        # same notification port as the listener's.
        self._notify_port = notify_port
        self.mission_proc: "ScriptProcess | None" = None
        self._mission_exit_reported = True
        # RoboCommand RunStart -> RUNSTART_MISSION_SCRIPT, once per run id.
        self._runstart_mission = runstart_mission
        self._runstart_script = RUNSTART_MISSION_SCRIPT
        self._runstart_runs: set = set()
        # 'mission clear': the FCU's MISSION_ACK arrives on the link thread.
        self._mission_clear_listening = False
        self._mission_clear_waiting = False
        # 'fence clear': the same, for the fence list.
        self._fence_clear_listening = False
        self._fence_clear_waiting = False
        # 'fence preview': the fence read in progress ({"count", "items", "progress_at"}) or None.
        self._fence_preview_listening = False
        self._fence_preview: "dict | None" = None
        self._fence_preview_timer = None
        # 'quit clear': main() deletes the FCU's mission + fence on the way out.
        self.clear_on_quit = False
        # Altitude line: {"rel"/"range": (metres, monotonic time)}, filled on the link thread.
        self._altitude: dict = {}
        self._altitude_request_at = 0.0
        self.monitor.add_message_listener(self._on_altitude_message)
        # Started by 'record'; its live status comes from a small JSON file it
        # keeps up to date, its final summary through the notification port.
        self.recorder_proc: "ScriptProcess | None" = None
        self._recorder_exit_reported = True
        self._shown_message_count = 0
        self._pending_confirm: str | None = None  # e.g. "arm", awaiting a blank Enter
        self._pending_menu: dict[str, str] | None = None  # e.g. calibration number -> kind
        self._last_calibration_finished = False

        # Camera is NOT started here - only when the user types 'camera', per
        # the request to not change anything else about this GUI's existing
        # startup behavior.
        self.camera: "CameraLink | None" = None
        self._camera_device = camera_device
        self._camera_output_dir = camera_output_dir or (Path(__file__).resolve().parent / "captures")
        self._camera_rotation_deg = camera_rotation_deg
        self._record_status_path = self._camera_output_dir / "recording_status.json"

    def compose(self) -> ComposeResult:
        with Horizontal(id="main-row"):
            yield Static(id="dashboard")
            yield Static(render_commands_section(), id="commands")
        yield RichLog(id="log", wrap=True, markup=True)
        yield Input(placeholder="command>", id="command")

    def on_mount(self) -> None:
        log = self.query_one("#log", RichLog)
        if self.mavproxy_proc is not None:
            log.write(
                f"MAVProxy started (master={self.mavproxy_proc.master}, out={self.mavproxy_proc.out})."
            )
        self.monitor.start()
        if self.notifications is not None:
            error = self.notifications.start()
            if error:
                log.write(Text(error, style="yellow"))
        log.write("Heartbeat monitor started. Type 'help' for commands.")
        self.set_interval(REFRESH_INTERVAL, self.refresh_status)
        self.query_one("#command", Input).focus()

    def _check_listener(self, connected: bool) -> None:
        proc = self.listener_proc
        if proc is None or proc.is_running():
            return
        log = self.query_one("#log", RichLog)
        now = time.monotonic()
        if self._listener_was_running:
            self._listener_was_running = False
            self._listener_retry_at = now + LISTENER_RETRY_S
            log.write(Text(
                f"Geofence listener stopped (exit code {proc.exit_code()}) - details in {proc.log_path}. "
                f"Retrying in {LISTENER_RETRY_S:g} s.",
                style="yellow",
            ))
            return
        if not connected or now < self._listener_retry_at:
            return
        error = proc.start()
        if error:
            if error != self._listener_last_error:
                log.write(Text(error, style="yellow"))
            self._listener_last_error = error
            self._listener_retry_at = now + LISTENER_RETRY_S
            return
        self._listener_was_running = True
        self._listener_last_error = None
        log.write(f"Geofence listener started (log: {proc.log_path}).")

    def refresh_status(self) -> None:
        snapshot = self.monitor.snapshot()
        telemetry = self.monitor.telemetry_snapshot()
        calibration = self.monitor.calibration_snapshot()
        camera_status = self.camera.status() if self.camera is not None else None
        recording_status = None
        if self.recorder_proc is not None and self.recorder_proc.is_running():
            try:
                recording_status = json.loads(self._record_status_path.read_text())
            except (OSError, ValueError):
                recording_status = {"state": "starting"}

        self.query_one("#dashboard", Static).update(
            render_dashboard(snapshot, telemetry, self.monitor.connection_string, calibration, camera_status,
                             recording_status, self._altitude)
        )
        if snapshot.connected:
            self._request_altitude_stream()

        new_messages = telemetry.recent_messages[self._shown_message_count:]
        if new_messages:
            log = self.query_one("#log", RichLog)
            for m in new_messages:
                style = SEVERITY_STYLE.get(m.severity, "")
                log.write(Text(f"[{m.severity_name}] {m.text}", style=style))
            self._shown_message_count = len(telemetry.recent_messages)

        self._check_listener(snapshot.connected)
        # Checked before draining notifications, reported after them, so the
        # script's own last message comes first in the log.
        mission_just_ended = (
            self.mission_proc is not None and not self._mission_exit_reported and not self.mission_proc.is_running()
        )
        recorder_just_ended = (
            self.recorder_proc is not None and not self._recorder_exit_reported and not self.recorder_proc.is_running()
        )

        if self.notifications is not None:
            notes = self.notifications.drain()
            if notes:
                log = self.query_one("#log", RichLog)
                for severity, text in notes:
                    name = MAV_SEVERITY.get(severity, str(severity))
                    log.write(Text(f"[{name}] {text}", style=SEVERITY_STYLE.get(severity, "")))
                    if text.startswith(RUNSTART_PREFIX):
                        self._on_run_start(text[len(RUNSTART_PREFIX):].split(" ", 1)[0], log)

        if mission_just_ended:
            self._mission_exit_reported = True
            self.query_one("#log", RichLog).write(
                f"Mission script finished (exit code {self.mission_proc.exit_code()}, log: {self.mission_proc.log_path})."
            )
        if recorder_just_ended:
            self._recorder_exit_reported = True
            if self.recorder_proc.exit_code():
                self.query_one("#log", RichLog).write(Text(
                    f"Flight recorder stopped with an error (exit code {self.recorder_proc.exit_code()}, "
                    f"details in {self.recorder_proc.log_path}).", style="yellow"))

        if calibration.finished and not self._last_calibration_finished:
            log = self.query_one("#log", RichLog)
            word = "SUCCESS" if calibration.success else "FAILED"
            style = "bold green" if calibration.success else "bold red"
            log.write(Text(f"{calibration.kind} calibration {word}", style=style))
        self._last_calibration_finished = calibration.finished

    def _on_altitude_message(self, msg) -> None:
        # link thread
        msg_type = msg.get_type()
        if msg_type not in ("GLOBAL_POSITION_INT", "DISTANCE_SENSOR"):
            return
        _, system_id, _ = self.monitor.active_link()
        if system_id is not None and msg.get_srcSystem() != system_id:
            return
        now = time.monotonic()
        if msg_type == "GLOBAL_POSITION_INT":
            self._altitude["rel"] = (msg.relative_alt / 1000.0, now)
        elif msg.orientation == mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270:  # facing down
            self._altitude["range"] = (msg.current_distance / 100.0, now)

    def _request_altitude_stream(self) -> None:
        """Ask the FCU for GLOBAL_POSITION_INT if it isn't arriving."""
        now = time.monotonic()
        rel = self._altitude.get("rel")
        if (rel is not None and now - rel[1] <= ALTITUDE_STALE_S) or now < self._altitude_request_at:
            return
        self._altitude_request_at = now + ALTITUDE_RETRY_S
        master, system_id, component_id = self.monitor.active_link()
        if master is None or system_id is None:
            return
        try:
            master.mav.command_long_send(
                system_id, component_id or 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 1_000_000.0 / ALTITUDE_STREAM_HZ, 0, 0, 0, 0, 0,
            )
        except Exception:
            pass

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id != "command":
            return
        text = event.value.strip()
        event.input.value = ""
        log = self.query_one("#log", RichLog)

        if self._pending_menu is not None:
            menu, self._pending_menu = self._pending_menu, None
            if text in menu:
                log.write(f"> {text}")
                ok, message = self.monitor.start_calibration(menu[text])
                log.write(message)
                return
            log.write("calibration menu cancelled")
            # fall through - whatever they typed instead is handled as a normal command

        if self._pending_confirm is not None:
            pending, self._pending_confirm = self._pending_confirm, None
            if text == "":
                self.confirm_pending(pending, log)
                return
            log.write(f"{pending} cancelled")
            # fall through - whatever they typed instead is handled as a normal command

        if not text:
            return
        log.write(f"> {text}")
        self.dispatch_command(text, log)

    def confirm_pending(self, action: str, log: RichLog) -> None:
        if action == "arm":
            ok = self.monitor.send_arm(True)
            log.write("ARM command sent" if ok else "Not connected - can't arm")
        elif action == "reboot":
            ok, message = self.monitor.send_reboot()
            log.write(message)
        elif action == "mission":
            self.start_mission(["--yes"], log)
        elif action == "mission clear":
            self.clear_mission(log)
        elif action == "fence clear":
            self.clear_fence(log)
        elif action in EXTRA_MISSIONS:
            self.start_mission(["--yes"], log, EXTRA_MISSIONS[action][0])
        elif action == "quit clear":
            problem = self._quit_clear_problem()  # checked again: the drone may have armed meanwhile
            if problem:
                log.write(Text(problem, style="yellow"))
                return
            self.clear_on_quit = True
            self.exit()

    def _quit_clear_problem(self) -> "str | None":
        snapshot = self.monitor.snapshot()
        if not snapshot.connected:
            return "Not connected to the FCU - can't delete anything. Plain 'quit' just closes."
        if snapshot.armed:
            return "The drone is ARMED - 'quit clear' only works disarmed. Land and disarm first (or plain 'quit')."
        return None

    def _mission_clear_problem(self) -> "str | None":
        """Why the FCU's mission can't be cleared right now, or None."""
        snapshot = self.monitor.snapshot()
        if not snapshot.connected:
            return "Not connected - can't clear the mission."
        if self.mission_proc is not None and self.mission_proc.is_running():
            return "The mission script is running - 'mission stop' first."
        if snapshot.armed and (snapshot.mode_name or "").upper() == "AUTO":
            return "The drone is flying the mission in AUTO - switch mode first (e.g. LOITER or RTL), then clear it."
        if self._mission_clear_waiting:
            return "Already clearing - waiting for the FCU's reply."
        return None

    def clear_mission(self, log: RichLog) -> None:
        """Delete the mission stored on the FCU (MISSION_CLEAR_ALL for the
        mission list only - the fence and rally points are separate lists and
        stay as they are). The FCU's reply is reported when it arrives."""
        problem = self._mission_clear_problem()  # checked again: things may have changed since the command
        if problem:
            log.write(Text(problem, style="yellow"))
            return
        master, system_id, component_id = self.monitor.active_link()
        if master is None or system_id is None:
            log.write("Not connected - can't clear the mission.")
            return
        if not self._mission_clear_listening:
            self.monitor.add_message_listener(self._on_mission_clear_ack)
            self._mission_clear_listening = True
        self._mission_clear_waiting = True
        try:
            # mission_type left at its default 0 = the mission (not fence/rally)
            master.mav.mission_clear_all_send(system_id, component_id or 1)
        except Exception as e:
            self._mission_clear_waiting = False
            log.write(Text(f"Couldn't send the mission clear: {e}", style="red"))
            return
        log.write("Clearing the mission on the FCU...")
        self.set_timer(MISSION_CLEAR_TIMEOUT_S, self._mission_clear_timed_out)

    def _on_mission_clear_ack(self, msg) -> None:
        # link thread
        if not self._mission_clear_waiting or msg.get_type() != "MISSION_ACK":
            return
        if getattr(msg, "mission_type", 0) != mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
            return
        _, system_id, _ = self.monitor.active_link()
        if system_id is not None and msg.get_srcSystem() != system_id:
            return
        self._mission_clear_waiting = False
        self.call_from_thread(self._mission_clear_done, msg.type)

    def _mission_clear_done(self, result: int) -> None:
        log = self.query_one("#log", RichLog)
        if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            log.write(Text(
                "Mission cleared on the FCU (fence and rally points kept). "
                "In Mission Planner: Plan > Read to refresh its map.", style="green"))
        else:
            entry = mavutil.mavlink.enums["MAV_MISSION_RESULT"].get(result)
            log.write(Text(f"The FCU refused to clear the mission: {entry.name if entry else result}", style="red"))

    def _mission_clear_timed_out(self) -> None:
        if not self._mission_clear_waiting:
            return
        self._mission_clear_waiting = False
        self.query_one("#log", RichLog).write(Text(
            "No reply from the FCU to the mission clear - check in Mission Planner (Plan > Read).", style="yellow"))

    def _fence_clear_problem(self) -> "str | None":
        """Why the FCU's fence can't be cleared right now, or None."""
        snapshot = self.monitor.snapshot()
        if not snapshot.connected:
            return "Not connected - can't clear the fence."
        if snapshot.armed:
            return "The drone is ARMED - 'fence clear' only works disarmed. Land and disarm first."
        if self.mission_proc is not None and self.mission_proc.is_running():
            return "The mission script is running - 'mission stop' first."
        if self._fence_clear_waiting:
            return "Already clearing - waiting for the FCU's reply."
        return None

    def clear_fence(self, log: RichLog) -> None:
        """Delete every fence point stored on the FCU (MISSION_CLEAR_ALL for
        the fence list only - the mission, rally points and FENCE_* params
        stay as they are). The FCU's reply is reported when it arrives."""
        problem = self._fence_clear_problem()  # checked again: things may have changed since the command
        if problem:
            log.write(Text(problem, style="yellow"))
            return
        master, system_id, component_id = self.monitor.active_link()
        if master is None or system_id is None:
            log.write("Not connected - can't clear the fence.")
            return
        if not master.mavlink20():
            # mission_type is a MAVLink 2 field: over MAVLink 1 the FCU would clear the MISSION instead
            log.write(Text("The link is MAVLink 1 - 'fence clear' needs MAVLink 2, so nothing was sent. "
                           "Clear the fence in Mission Planner instead.", style="red"))
            return
        if not self._fence_clear_listening:
            self.monitor.add_message_listener(self._on_fence_clear_ack)
            self._fence_clear_listening = True
        self._fence_clear_waiting = True
        try:
            master.mav.mission_clear_all_send(system_id, component_id or 1, mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        except Exception as e:
            self._fence_clear_waiting = False
            log.write(Text(f"Couldn't send the fence clear: {e}", style="red"))
            return
        log.write("Clearing the fence on the FCU...")
        self.set_timer(MISSION_CLEAR_TIMEOUT_S, self._fence_clear_timed_out)

    def _on_fence_clear_ack(self, msg) -> None:
        # link thread
        if not self._fence_clear_waiting or msg.get_type() != "MISSION_ACK":
            return
        if getattr(msg, "mission_type", 0) != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            return
        _, system_id, _ = self.monitor.active_link()
        if system_id is not None and msg.get_srcSystem() != system_id:
            return
        self._fence_clear_waiting = False
        self.call_from_thread(self._fence_clear_done, msg.type)

    def _fence_clear_done(self, result: int) -> None:
        log = self.query_one("#log", RichLog)
        if result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            log.write(Text(
                "Fence cleared on the FCU (mission, rally points and FENCE_* params kept). The drone has no "
                "polygon fence until a new one is sent - the missions won't fly without one. "
                "In Mission Planner: Plan > Fence > Read to refresh its map.", style="green"))
        else:
            entry = mavutil.mavlink.enums["MAV_MISSION_RESULT"].get(result)
            log.write(Text(f"The FCU refused to clear the fence: {entry.name if entry else result}", style="red"))

    def _fence_clear_timed_out(self) -> None:
        if not self._fence_clear_waiting:
            return
        self._fence_clear_waiting = False
        self.query_one("#log", RichLog).write(Text(
            "No reply from the FCU to the fence clear - check in Mission Planner (Plan > Fence > Read).", style="yellow"))

    def preview_fence(self, log: RichLog) -> None:
        """Read every fence item back from the FCU (MISSION_REQUEST_LIST for the
        fence list) and show the coordinates. Read only - nothing on the FCU
        changes. The replies arrive on the link thread, one item per request."""
        if self._fence_preview is not None:
            log.write(Text("Already reading the fence - waiting for the FCU's reply.", style="yellow"))
            return
        master, system_id, component_id = self.monitor.active_link()
        if master is None or system_id is None or not self.monitor.snapshot().connected:
            log.write("Not connected - can't read the fence.")
            return
        if not master.mavlink20():
            # mission_type is a MAVLink 2 field: over MAVLink 1 the FCU would send the MISSION instead
            log.write(Text("The link is MAVLink 1 - 'fence preview' needs MAVLink 2, so nothing was sent. "
                           "Read the fence in Mission Planner instead (Plan > Fence > Read).", style="red"))
            return
        if not self._fence_preview_listening:
            self.monitor.add_message_listener(self._on_fence_preview_message)
            self._fence_preview_listening = True
        self._fence_preview = {"count": None, "items": [], "progress_at": time.monotonic()}
        try:
            master.mav.mission_request_list_send(system_id, component_id or 1, mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        except Exception as e:
            self._fence_preview = None
            log.write(Text(f"Couldn't ask the FCU for its fence: {e}", style="red"))
            return
        log.write("Reading the fence from the FCU...")
        self._fence_preview_timer = self.set_interval(FENCE_PREVIEW_RESEND_S, self._fence_preview_tick)

    def _on_fence_preview_message(self, msg) -> None:
        # link thread
        state = self._fence_preview
        if state is None or msg.get_type() not in ("MISSION_COUNT", "MISSION_ITEM_INT"):
            return
        if getattr(msg, "mission_type", 0) != mavutil.mavlink.MAV_MISSION_TYPE_FENCE:
            return
        master, system_id, component_id = self.monitor.active_link()
        if master is None or msg.get_srcSystem() != system_id:
            return
        # Through MAVProxy the FCU's replies to the geofence listener and Mission Planner arrive here too.
        if (msg.target_system, msg.target_component) != (master.mav.srcSystem, master.mav.srcComponent):
            return
        if msg.get_type() == "MISSION_COUNT":
            if state["count"] is not None:
                return
            state["count"] = msg.count
        elif state["count"] is not None and msg.seq == len(state["items"]):
            state["items"].append((msg.command, msg.param1, msg.x / 1e7, msg.y / 1e7))
        else:
            return
        state["progress_at"] = time.monotonic()
        try:
            if len(state["items"]) < state["count"]:
                master.mav.mission_request_int_send(system_id, component_id or 1, len(state["items"]),
                                                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
                return
            master.mav.mission_ack_send(system_id, component_id or 1, mavutil.mavlink.MAV_MISSION_ACCEPTED,
                                        mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        except Exception:
            return  # the GUI-side timer asks again
        self._fence_preview = None
        self.call_from_thread(self._fence_preview_done, state["items"])

    def _fence_preview_tick(self) -> None:
        state = self._fence_preview
        if state is None:
            self._stop_fence_preview_timer()
            return
        idle = time.monotonic() - state["progress_at"]
        if idle >= FENCE_PREVIEW_TIMEOUT_S:
            self._fence_preview = None
            self._stop_fence_preview_timer()
            self.query_one("#log", RichLog).write(Text(
                "No reply from the FCU to the fence read - try again, or check in Mission Planner (Plan > Fence > Read).",
                style="yellow"))
            return
        master, system_id, component_id = self.monitor.active_link()
        if master is None or system_id is None:
            return
        try:  # a request or reply went missing: ask again for what's next
            if state["count"] is None:
                master.mav.mission_request_list_send(system_id, component_id or 1,
                                                     mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
            else:
                master.mav.mission_request_int_send(system_id, component_id or 1, len(state["items"]),
                                                    mavutil.mavlink.MAV_MISSION_TYPE_FENCE)
        except Exception:
            pass

    def _stop_fence_preview_timer(self) -> None:
        if self._fence_preview_timer is not None:
            self._fence_preview_timer.stop()
            self._fence_preview_timer = None

    def _fence_preview_done(self, items: list) -> None:
        self._stop_fence_preview_timer()
        log = self.query_one("#log", RichLog)
        if not items:
            log.write(Text("Fence preview: the FCU has no fence points stored.", style="yellow"))
            return
        mav = mavutil.mavlink
        lines = [f"Fence preview: {len(items)} item(s) on the FCU (lat, lon):"]
        i = 0
        while i < len(items):
            command, param1, lat, lon = items[i]
            if command in (mav.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, mav.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION):
                kind = "inclusion" if command == mav.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION else "exclusion"
                count = max(1, int(param1))
                lines.append(f"  {kind} polygon, {count} points:")
                for n, (_c, _p, vlat, vlon) in enumerate(items[i:i + count], 1):
                    lines.append(f"    {n:>2}  {vlat:.7f}, {vlon:.7f}")
                i += count
                continue
            if command in (mav.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION, mav.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION):
                kind = "inclusion" if command == mav.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION else "exclusion"
                lines.append(f"  {kind} circle: centre {lat:.7f}, {lon:.7f}, radius {param1:g} m")
            elif command == mav.MAV_CMD_NAV_FENCE_RETURN_POINT:
                lines.append(f"  return point: {lat:.7f}, {lon:.7f}")
            else:
                lines.append(f"  item {i}: command {command} at {lat:.7f}, {lon:.7f}")
            i += 1
        log.write("\n".join(lines))

    def _on_run_start(self, run_id: str, log: RichLog) -> None:
        """RoboCommand started the run: fly RUNSTART_MISSION_SCRIPT (once per run)."""
        if not self._runstart_mission:
            log.write(Text("RunStart: auto mission is off (--no-runstart-mission) - start a mission by hand.", style="yellow"))
            return
        if run_id in self._runstart_runs:
            return
        self._runstart_runs.add(run_id)
        snapshot = self.monitor.snapshot()
        if not snapshot.connected:
            problem = "not connected to the FCU"
        elif snapshot.armed:
            problem = "the drone is already ARMED"
        elif self.mission_proc is not None and self.mission_proc.is_running():
            problem = "a mission script is already running"
        else:
            problem = None
        if problem:
            log.write(Text(f"RunStart (run {run_id}): mission NOT started - {problem}.", style="bold red"))
            return
        task4_note = ", reacting to Task 4 by itself" if self._runstart_script == RUNSTART_TASK4_SCRIPT else ""
        log.write(Text(f"RunStart (run {run_id}): starting the RunStart mission - arm, hover 5 m for 15 s, then "
                       f"mission2's lawnmower (10 m) at 3 m/s{task4_note}. Flip the RC mode switch to take over at any time.",
                       style="bold yellow"))
        self.start_mission(["--yes"], log, self._runstart_script)

    def start_mission(self, extra_args: list, log: RichLog, script: Path = MISSION_SCRIPT) -> None:
        if self.mission_proc is not None and self.mission_proc.is_running():
            log.write("A mission script is already running - 'mission stop' first.")
            return
        notify = f"127.0.0.1:{self._notify_port}" if self._notify_port else ""
        proc = ScriptProcess(str(script), [*extra_args, "--notify", notify], log_path=str(script.with_suffix(".log")))
        error = proc.start()
        if error:
            log.write(Text(error, style="red"))
            return
        self.mission_proc = proc
        self._mission_exit_reported = False
        log.write(f"Mission script started (log: {proc.log_path}).")

    def dispatch_command(self, text: str, log: RichLog) -> None:
        parts = text.split()
        cmd = parts[0].lower()
        args = parts[1:]

        if cmd in ("quit", "q", "exit") and args and args[0].lower() == "clear":
            problem = self._quit_clear_problem()
            if problem:
                log.write(Text(problem, style="yellow"))
                return
            self._pending_confirm = "quit clear"
            log.write(Text(
                "This closes the GUI and deletes the mission AND the fence stored on the FCU, so Mission Planner "
                "shows none after its next Read. The drone then has no polygon fence until a new one is sent. "
                "Press Enter again to confirm, or type anything else to cancel.",
                style="yellow",
            ))
            return
        if cmd in ("quit", "q", "exit"):
            self.exit()
        elif cmd == "help":
            log.write(HELP_TEXT)
        elif cmd == "reconnect":
            self.monitor.request_reconnect()
            log.write("Reconnecting...")
        elif cmd == "connect":
            if not args:
                log.write("usage: connect <mavlink-connection-string>")
                return
            self.monitor.request_reconnect(args[0])
            log.write(f"Switching connection to {args[0]}")
        elif cmd == "timeout":
            if not args:
                log.write("usage: timeout <seconds>")
                return
            try:
                self.monitor.heartbeat_timeout = float(args[0])
                log.write(f"Heartbeat timeout set to {args[0]}s")
            except ValueError:
                log.write("timeout must be a number")
        elif cmd == "disarm":
            ok = self.monitor.send_arm(False)
            log.write("DISARM command sent" if ok else "Not connected - can't disarm")
        elif cmd == "arm":
            if args and args[0].lower() == "confirm":
                ok = self.monitor.send_arm(True)
                log.write("ARM command sent" if ok else "Not connected - can't arm")
            else:
                self._pending_confirm = "arm"
                log.write("This sends a real ARM command to the vehicle. Press Enter again to confirm, or type anything else to cancel.")
        elif cmd == "mode":
            if not args:
                current = self.monitor.snapshot().mode_name
                log.write(f"current mode: {current}" if current else "current mode: unknown (no heartbeat yet)")
                return
            ok, message = self.monitor.send_mode(args[0])
            log.write(message)
        elif cmd == "calibrate":
            if not args:
                self._pending_menu = {"1": "accel", "2": "compass"}
                log.write("Available calibrations:\n  1  accel\n  2  compass\nType the number to start.")
                return
            ok, message = self.monitor.start_calibration(args[0])
            log.write(message)
        elif cmd == "reboot":
            self._pending_confirm = "reboot"
            log.write(
                "This reboots the flight controller - the link will drop and reconnect. "
                "Press Enter again to confirm, or type anything else to cancel."
            )
        elif cmd == "camera":
            if args and args[0].lower() == "stop":
                if self.camera is None:
                    log.write("Camera isn't open.")
                    return
                self.camera.stop()
                self.camera = None
                log.write("Camera closing - device will be free for another program in a moment.")
                return
            if self.camera is not None:
                log.write("Camera is already open.")
                return
            if self.recorder_proc is not None and self.recorder_proc.is_running():
                log.write("The flight recorder is using the camera - 'record stop' first.")
                return
            self.camera = CameraLink(self._camera_device, self._camera_output_dir, rotation_deg=self._camera_rotation_deg)
            self.camera.start()
            log.write(f"Opening camera (device {self._camera_device}, saving to {self._camera_output_dir})...")
        elif cmd == "photo":
            if self.camera is None:
                log.write("Camera isn't open yet - type 'camera' first.")
                return
            ok, message = self.camera.capture(args[0] if args else None)
            log.write(Text(message, style="green" if ok else "red"))
        elif cmd == "mission":
            sub = args[0].lower() if args else ""
            if sub == "stop":
                if self.mission_proc is None or not self.mission_proc.is_running():
                    log.write("No mission script is running.")
                    return
                self.mission_proc.stop()
                log.write(Text(
                    "Mission script stopped. This does NOT change the flight mode - if the drone is flying, "
                    "take over with the RC mode switch (or type a mode, e.g. LOITER).",
                    style="yellow",
                ))
            elif sub == "preview":
                self.start_mission(["--upload-only"], log)
            elif sub == "clear":
                problem = self._mission_clear_problem()
                if problem:
                    log.write(Text(problem, style="yellow"))
                    return
                self._pending_confirm = "mission clear"
                log.write(Text(
                    "This deletes the mission stored on the FCU (the lawnmower route, or anything uploaded from "
                    "Mission Planner). The fence and rally points are NOT touched. Press Enter again to confirm, "
                    "or type anything else to cancel.",
                    style="yellow",
                ))
            elif sub == "":
                if self.mission_proc is not None and self.mission_proc.is_running():
                    log.write("A mission script is already running - 'mission stop' first.")
                    return
                self._pending_confirm = "mission"
                log.write(Text(
                    "This ARMS the drone and flies the lawnmower mission: take off to 5 m, hover 30 s, "
                    "mow inside the fence at 3 m/s, return home and land (then GUIDED, waypoints cleared). The RC mode switch must be in GUIDED; "
                    "flip it to take over at any time. Press Enter again to confirm, or type anything else to cancel.",
                    style="yellow",
                ))
            else:
                log.write("usage: mission | mission preview | mission stop | mission clear")
        elif cmd in EXTRA_MISSIONS:
            script, altitude, speed, hover = EXTRA_MISSIONS[cmd]
            sub = args[0].lower() if args else ""
            if sub in ("stop", "clear"):
                self.dispatch_command(f"mission {sub}", log)  # the same script slot / FCU mission
            elif sub == "preview":
                self.start_mission(["--upload-only"], log, script)
            elif sub == "":
                if self.mission_proc is not None and self.mission_proc.is_running():
                    log.write("A mission script is already running - 'mission stop' first.")
                    return
                self._pending_confirm = cmd
                log.write(Text(
                    f"This ARMS the drone and flies {cmd}: take off to {altitude}, hover {hover}, "
                    f"mow inside the fence at {speed}, return home and land (then GUIDED, waypoints cleared). "
                    "The RC mode switch must be in GUIDED; flip it to take over at any time. "
                    "Press Enter again to confirm, or type anything else to cancel.",
                    style="yellow",
                ))
            else:
                log.write(f"usage: {cmd} | {cmd} preview | {cmd} stop | {cmd} clear")
        elif cmd == "runstart":
            choice = args[0].lower() if len(args) == 1 else ""
            if choice == "task4":
                self._runstart_script = RUNSTART_TASK4_SCRIPT
            elif choice == "normal":
                self._runstart_script = RUNSTART_MISSION_SCRIPT
            elif args:
                log.write("usage: runstart | runstart task4 | runstart normal")
                return
            log.write(f"RunStart flies {self._runstart_script.name}.")
        elif cmd == "fence":
            if len(args) == 1 and args[0].lower() == "preview":
                self.preview_fence(log)
                return
            if not args or args[0].lower() != "clear" or len(args) > 1:
                log.write("usage: fence clear | fence preview")
                return
            problem = self._fence_clear_problem()
            if problem:
                log.write(Text(problem, style="yellow"))
                return
            self._pending_confirm = "fence clear"
            log.write(Text(
                "This deletes ALL fence points stored on the FCU (polygons, circles, return point). The mission, "
                "rally points and FENCE_* params are NOT touched. A running laptop publisher, or a new course "
                "from RoboCommand, puts a fence back. Press Enter again to confirm, or type anything else to cancel.",
                style="yellow",
            ))
        elif cmd == "record":
            recording = self.recorder_proc is not None and self.recorder_proc.is_running()
            if args and args[0].lower() == "stop":
                if not recording:
                    log.write("Not recording.")
                    return
                log.write("Stopping the recording - finishing the video file...")
                self.recorder_proc.stop()  # SIGTERM: the recorder finishes the files, then exits
                return
            if args:
                log.write("usage: record | record stop")
                return
            if recording:
                log.write("Already recording - 'record stop' to stop.")
                return
            if self.camera is not None:
                log.write("The camera is open here ('camera') and only one program can use it - 'camera stop' first, then 'record'.")
                return
            try:
                self._record_status_path.unlink()
            except OSError:
                pass
            notify = f"127.0.0.1:{self._notify_port}" if self._notify_port else ""
            proc = ScriptProcess(
                str(RECORDER_SCRIPT),
                ["--device", self._camera_device, "--rotate", str(self._camera_rotation_deg),
                 "--output-dir", str(self._camera_output_dir), "--status-file", str(self._record_status_path),
                 "--notify", notify],
                log_path=str(RECORDER_SCRIPT.with_suffix(".log")),
            )
            error = proc.start()
            if error:
                log.write(Text(error, style="red"))
                return
            self.recorder_proc = proc
            self._recorder_exit_reported = False
            log.write(f"Recording video + FCU flight data to {self._camera_output_dir} - 'record stop' to finish.")
        elif not args:
            # Not a recognized command and it's a single word - try it as a
            # bare mode name (e.g. typing "GUIDED" directly, no "mode" prefix).
            # Only report it if it's actually a real mode, so a typo of some
            # other command (e.g. "reconect") still says "Unknown command"
            # instead of a confusing "unknown mode" message.
            ok, message = self.monitor.send_mode(cmd)
            log.write(message if ok else f"Unknown command: {cmd} (type 'help')")
        else:
            log.write(f"Unknown command: {cmd} (type 'help')")


def run_fcu_clear() -> None:
    """'quit clear': delete the FCU's mission + fence (fcu_clear.py), printing
    its result in the terminal the GUI was started from."""
    print("Deleting the mission and fence stored on the FCU...", flush=True)
    try:
        subprocess.run([sys.executable, "-u", str(FCU_CLEAR_SCRIPT), "--connection", FCU_CLEAR_CONNECTION], timeout=60)
    except subprocess.TimeoutExpired:
        print("fcu_clear.py timed out - check in Mission Planner (Plan > Read, Fence > Read).", flush=True)
    except OSError as e:
        print(f"Couldn't run {FCU_CLEAR_SCRIPT.name}: {e}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--connection",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string this GUI listens on (default: %(default)s). "
        "With --master, this is also where the auto-launched MAVProxy sends its output.",
    )
    parser.add_argument(
        "--master",
        default="udp:0.0.0.0:14555",
        help="MAVProxy is launched automatically with this as its --master connection "
        "(default: %(default)s). Use --no-mavproxy if MAVProxy is already running.",
    )
    parser.add_argument(
        "--no-mavproxy",
        action="store_true",
        help="Don't launch MAVProxy - connect to one that is already running instead.",
    )
    parser.add_argument(
        "--mavproxy-out",
        action="append",
        default=None,
        help="Extra output for the auto-launched MAVProxy, on top of this GUI's own; repeat for more "
        f"(default: {' '.join(DEFAULT_MAVPROXY_EXTRA_OUTS)} - geofence listener, spare tools, Mission Planner on the GCS laptop)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=3.0,
        help="Seconds without a HEARTBEAT before the link is considered lost (default: %(default)s)",
    )
    parser.add_argument(
        "--camera-device",
        default="0",
        help="USB camera device for the 'camera' command: an index (e.g. 0) or a path "
        "(e.g. /dev/video2). Not opened until you type 'camera'. Default: %(default)s",
    )
    parser.add_argument(
        "--camera-output-dir",
        default=None,
        help="Directory to save photos in (default: a 'captures' folder next to this script)",
    )
    parser.add_argument(
        "--camera-rotate",
        type=float,
        default=0.0,
        help="Rotate the camera feed this many degrees clockwise, to correct a "
        "physically mis-mounted camera (default: %(default)s, no rotation)",
    )
    parser.add_argument(
        "--notify-port",
        type=int,
        default=14650,
        help="Local UDP port where other programs on this machine (e.g. geofence_udp_listener.py) "
        "send notifications for the message log. 0 disables it (default: %(default)s)",
    )
    parser.add_argument(
        "--no-listener",
        action="store_true",
        help="Don't launch geofence_udp_listener.py (it is started automatically by default).",
    )
    parser.add_argument(
        "--listener-connection",
        default="udp:127.0.0.1:14551",
        help="MAVProxy output the auto-launched geofence listener connects to (default: %(default)s)",
    )
    parser.add_argument(
        "--listener-preset",
        default=str(GEOFENCE_DIR / "preset_fence_example.csv"),
        help="Preset fence CSV the listener restores when the publisher stops (default: %(default)s)",
    )
    parser.add_argument(
        "--no-runstart-mission",
        action="store_true",
        help="Don't fly runstart_mission.py when RoboCommand sends RunStart (it does by default), e.g. on the bench.",
    )
    args = parser.parse_args()

    listener_proc = None
    if not args.no_listener:
        listener_proc = ScriptProcess(
            str(LISTENER_SCRIPT),
            [
                "--connection", args.listener_connection,
                "--preset-fence-file", args.listener_preset,
                "--notify", f"127.0.0.1:{args.notify_port}" if args.notify_port else "",
                "--vehicle-id", "SGIT-53577",  # the id RoboCommand knows this drone by
            ],
            log_path=str(LISTENER_SCRIPT.with_suffix(".log")),
        )

    mavproxy_proc = None
    if args.master and not args.no_mavproxy:
        out_target = args.connection.split(":", 1)[1] if args.connection.startswith("udp:") else args.connection
        extra_outs = args.mavproxy_out if args.mavproxy_out is not None else DEFAULT_MAVPROXY_EXTRA_OUTS
        mavproxy_proc = MavproxyProcess(args.master, out=out_target, extra_args=[f"--out={o}" for o in extra_outs])
        try:
            mavproxy_proc.start()
        except MavproxyLaunchError as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)

    app = HeartbeatApp(
        args.connection,
        args.timeout,
        mavproxy_proc=mavproxy_proc,
        camera_device=args.camera_device,
        camera_output_dir=Path(args.camera_output_dir) if args.camera_output_dir else None,
        camera_rotation_deg=args.camera_rotate,
        notify_port=args.notify_port or None,
        listener_proc=listener_proc,
        runstart_mission=not args.no_runstart_mission,
    )
    try:
        app.run()
    finally:
        app.monitor.stop()
        if app.notifications is not None:
            app.notifications.stop()
        if listener_proc is not None:
            listener_proc.stop()
        if app.mission_proc is not None:
            app.mission_proc.stop()
        if app.recorder_proc is not None:
            app.recorder_proc.stop()  # finishes the video file before MAVProxy goes
        if app.clear_on_quit:
            run_fcu_clear()  # while MAVProxy is still up, after the listener/mission have stopped
        if mavproxy_proc is not None:
            mavproxy_proc.stop()
        if app.camera is not None:
            app.camera.stop()


if __name__ == "__main__":
    main()
