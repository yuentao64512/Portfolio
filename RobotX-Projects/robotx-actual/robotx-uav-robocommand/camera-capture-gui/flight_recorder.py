#!/usr/bin/env python3
"""
Flight recorder: records video from the USB camera and, for every video
frame, the drone's flight state as reported by the FCU at that moment.

Files written (same name stem, in --output-dir):
  flight_<date>_<time>.mp4             the video, always 960x720 (4:3)
  flight_<date>_<time>_frames.csv      ONE ROW PER VIDEO FRAME: frame number, its
                                       time in the video, and the FCU's latest
                                       reading of every degree of freedom -
                                       position x/y/z (north/east/down, m) and
                                       velocities, roll/pitch/yaw (deg) and
                                       their rates (deg/s), heading (deg, 0-360),
                                       lat/lon/altitude, speeds, accelerations,
                                       flight mode and armed state, plus the
                                       FCU clock time the frame was captured at
                                       and the FCU heartbeat it falls in
  flight_<date>_<time>_fcu_raw.jsonl   every FCU message used, exactly as received,
                                       at full rate - the untouched source data

The video is on the real clock: frames go into the file at a fixed rate,
one repeated when the camera is slower than that and skipped when faster,
so video time is flight time (repeat=1 in the CSV marks a repeated frame).

True readings only: every value comes straight from an FCU message (only
unit conversions - rad to deg, mm to m - are applied). Nothing is
interpolated or estimated. Each group of values also carries the FCU's own
timestamp (time_boot_ms) and how old that reading was when the frame was
captured; a value the FCU hasn't sent is left blank, never filled in.

Synced to the FCU clock and heartbeat: on every FCU heartbeat (1 Hz) the
recorder sends a MAVLink TIMESYNC request and the FCU answers with its own
clock (the time_boot_ms clock), so each frame row gets fcu_time_ms - the FCU
clock at the moment that frame was captured - with its error bound
(fcu_clock_err_ms = half the round trip, from the quickest TIMESYNC of the
last 10 s), and heartbeat = which FCU heartbeat it falls in. If the FCU
doesn't answer TIMESYNC, fcu_time_ms comes from the FCU message timestamps
instead (fcu_clock_sync=oneway, error unknown - at least the link delay).

FCU data comes through the drone GUI's heartbeat code (heartbeat_link.py's
HeartbeatMonitor) on its own MAVProxy output (default udp:127.0.0.1:14553),
so it can run alongside the lawnmower mission (14552).

While recording it keeps a small status file up to date (--status-file) -
duration, file size, frames, the latest attitude - which the drone GUI shows.
Stops cleanly on Ctrl+C / SIGTERM ('record stop' in the GUI), finishing the
video file so it plays.

Usage:
    python3 flight_recorder.py --device 0
"""

import argparse
import csv
import json
import math
import os
import signal
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path

os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")  # before pymavlink loads - see heartbeat_link.py

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "drone-heartbeat-gui"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cv2  # noqa: E402
from pymavlink import mavutil  # noqa: E402

from camera_link import CameraLink  # noqa: E402
from heartbeat_link import HeartbeatMonitor, is_autopilot_heartbeat  # noqa: E402

# Stream rates (Hz) asked of the FCU while recording, so the log is dense
# enough for video. Restored afterwards to what the GUI / other scripts ask
# for (RESTORE_RATES_HZ) - the FCU keeps one rate per message.
RECORD_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 30,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED: 15,
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU: 15,
}
RESTORE_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,  # the drone GUI's rate
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,  # the mission / listener rate
}
LOGGED_TYPES = {"ATTITUDE", "LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "VFR_HUD", "SCALED_IMU"}
# FCU clock sync: a TIMESYNC request on every FCU heartbeat; the quickest
# answer of the last CLOCK_WINDOW_S gives the FCU-minus-Orin clock offset.
CLOCK_TYPES = {"HEARTBEAT", "TIMESYNC"}
CLOCK_WINDOW_S = 10.0
CLOCK_JUMP_S = 1.0  # offset changed by more than this = the FCU rebooted: start the sync again
G = 9.80665
CAMERA_CHECK_S = 3.0  # after opening, wait up to this long for the first frames
MIN_FILE_FPS, MAX_FILE_FPS, DEFAULT_FILE_FPS = 5.0, 60.0, 30.0  # video file rate: the camera's own, if sane
STATUS_INTERVAL_S = 0.5
# The video is always recorded at this size (4:3). The camera is asked for it;
# if it gives another size anyway, frames are converted (to_record_size).
RECORD_WIDTH, RECORD_HEIGHT = 960, 720


def to_record_size(frame, pad: bool = False):
    """The frame at exactly RECORD_WIDTH x RECORD_HEIGHT (4:3), never stretched:
    another shape is centre-cropped to 4:3 (pad=False), or fitted inside with
    black bars (pad=True - after --rotate, so nothing is cut off), then scaled."""
    height, width = frame.shape[:2]
    if (width, height) == (RECORD_WIDTH, RECORD_HEIGHT):
        return frame
    aspect = RECORD_WIDTH / RECORD_HEIGHT
    if pad:
        scale = min(RECORD_WIDTH / width, RECORD_HEIGHT / height)
        new_w = min(RECORD_WIDTH, max(1, round(width * scale)))
        new_h = min(RECORD_HEIGHT, max(1, round(height * scale)))
        frame = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
        top, left = (RECORD_HEIGHT - new_h) // 2, (RECORD_WIDTH - new_w) // 2
        return cv2.copyMakeBorder(frame, top, RECORD_HEIGHT - new_h - top, left, RECORD_WIDTH - new_w - left,
                                  cv2.BORDER_CONSTANT, value=0)
    if width / height > aspect:  # wider than 4:3 (e.g. 16:9) - trim the sides
        crop_w = round(height * aspect)
        x = (width - crop_w) // 2
        frame = frame[:, x:x + crop_w]
    elif width / height < aspect:  # taller than 4:3 - trim top and bottom
        crop_h = round(width / aspect)
        y = (height - crop_h) // 2
        frame = frame[y:y + crop_h]
    interpolation = cv2.INTER_AREA if frame.shape[1] > RECORD_WIDTH else cv2.INTER_LINEAR
    return cv2.resize(frame, (RECORD_WIDTH, RECORD_HEIGHT), interpolation=interpolation)

CSV_COLUMNS = [
    # frame = video frame number; camera_frame = which camera frame it shows;
    # repeat = 1 when the camera was slower than the file and this frame repeats the previous one
    "frame", "video_time_s", "camera_frame", "repeat", "orin_time_unix",
    # the FCU clock (time_boot_ms) when the frame was captured, +/- error, and how it was synced (timesync/oneway)
    "fcu_time_ms", "fcu_clock_err_ms", "fcu_clock_sync",
    # position / velocity (LOCAL_POSITION_NED: metres from the EKF origin, north-east-down)
    "x_north_m", "y_east_m", "z_down_m", "vx_north_mps", "vy_east_mps", "vz_down_mps",
    "local_pos_fcu_ms", "local_pos_age_ms",
    # attitude (ATTITUDE)
    "roll_deg", "pitch_deg", "yaw_deg", "roll_rate_dps", "pitch_rate_dps", "yaw_rate_dps",
    "attitude_fcu_ms", "attitude_age_ms",
    # global position + heading (GLOBAL_POSITION_INT)
    "heading_deg", "lat_deg", "lon_deg", "alt_msl_m", "alt_rel_home_m",
    "global_pos_fcu_ms", "global_pos_age_ms",
    # speeds (VFR_HUD)
    "groundspeed_mps", "climb_mps", "throttle_pct", "vfr_age_ms",
    # accelerations (SCALED_IMU, body frame)
    "accel_x_mps2", "accel_y_mps2", "accel_z_mps2", "imu_fcu_ms", "imu_age_ms",
    # from the heartbeat
    "mode", "armed",
    # heartbeat = number of FCU heartbeats since recording started (1 Hz), age = time since the last one
    "heartbeat", "heartbeat_age_ms",
]


class Notifier:
    """Prints, and sends each line to the drone GUI's message log."""

    def __init__(self, target: str):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._target = None
        if target:
            host, _, port = target.rpartition(":")
            self._target = (host or "127.0.0.1", int(port))

    def __call__(self, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_INFO) -> None:
        print(text, flush=True)
        if self._target is not None:
            try:
                self._sock.sendto(json.dumps({"severity": severity, "text": text}).encode("utf-8"), self._target)
            except OSError:
                pass


class FcuData:
    """Latest FCU message of each logged type (with the time it arrived), and
    the raw log of every one of them. Fed by HeartbeatMonitor's link thread."""

    def __init__(self, monitor: HeartbeatMonitor):
        self.monitor = monitor
        self._lock = threading.Lock()
        self._latest = {}  # type -> (msg, received unix time)
        self._raw = None
        self._t0 = None  # video start, for the raw log's video_time_s
        self.raw_count = 0
        self._timesync_sent = {}  # ts1 of our TIMESYNC requests -> Orin time sent
        self._timesync = deque()  # (Orin time answered, round trip s, FCU-minus-Orin clock offset s)
        self._oneway = deque()  # (Orin time, offset) from FCU message timestamps, largest offset first - fallback
        self.heartbeats = 0  # FCU heartbeats since recording started
        self._heartbeat_at = None
        monitor.add_message_listener(self._on_message)

    def start_raw_log(self, path: Path, t0: float) -> None:
        with self._lock:
            self._raw = open(path, "w", encoding="utf-8")
            self._t0 = t0
            self.heartbeats = 0

    def close_raw_log(self) -> None:
        with self._lock:
            if self._raw is not None:
                self._raw.close()
                self._raw = None

    def _on_message(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type not in LOGGED_TYPES and msg_type not in CLOCK_TYPES:
            return
        master, system_id, _ = self.monitor.active_link()
        if system_id is not None and msg.get_srcSystem() != system_id:
            return  # not the FCU
        now = time.time()
        extra = {}
        if msg_type == "HEARTBEAT":
            if system_id is None or not is_autopilot_heartbeat(msg):
                return
            self._send_timesync(master)
        elif msg_type == "TIMESYNC":
            with self._lock:
                sent = self._timesync_sent.pop(msg.ts1, None)
            if msg.tc1 == 0 or sent is None:
                return  # a request, or the answer to someone else's
            extra = {"request_sent_unix": round(sent, 6), "round_trip_ms": round((now - sent) * 1000, 3)}
        with self._lock:
            if msg_type == "HEARTBEAT":
                self.heartbeats += 1
                self._heartbeat_at = now
            elif msg_type == "TIMESYNC":
                # tc1 = the FCU clock (ns since boot) between our send and its answer
                if now - sent <= CLOCK_JUMP_S:
                    offset = msg.tc1 / 1e9 - (sent + now) / 2
                    if self._timesync and abs(offset - self._timesync[-1][2]) > CLOCK_JUMP_S:
                        self._timesync.clear()
                    self._timesync.append((now, now - sent, offset))
                    while now - self._timesync[0][0] > CLOCK_WINDOW_S:
                        self._timesync.popleft()
            else:
                self._latest[msg_type] = (msg, now)
                boot_ms = getattr(msg, "time_boot_ms", None)
                if boot_ms is not None:
                    offset = boot_ms / 1000 - now  # the least delayed message gives the largest offset
                    if self._oneway and offset < self._oneway[0][1] - CLOCK_JUMP_S:
                        self._oneway.clear()
                    while self._oneway and self._oneway[-1][1] <= offset:
                        self._oneway.pop()
                    self._oneway.append((now, offset))
                    while now - self._oneway[0][0] > CLOCK_WINDOW_S:
                        self._oneway.popleft()
            if self._raw is not None:
                record = msg.to_dict()
                record.pop("mavpackettype", None)
                if "type" in record:  # HEARTBEAT's own MAV_TYPE field - kept, renamed so it doesn't replace "type"
                    record["mav_type"] = record.pop("type")
                self._raw.write(json.dumps({"orin_time_unix": round(now, 4), "video_time_s": round(now - self._t0, 4),
                                            "type": msg_type, **record, **extra}) + "\n")
                self.raw_count += 1

    def _send_timesync(self, master) -> None:
        """Ask the FCU for its clock (sent on every FCU heartbeat, on the link thread)."""
        if master is None:
            return
        ts1 = time.time_ns()  # our id for the answer - the FCU sends it back unchanged
        sent = time.time()
        try:
            master.mav.timesync_send(0, ts1)
        except Exception:
            return
        with self._lock:
            for key in [k for k, t in self._timesync_sent.items() if sent - t > CLOCK_WINDOW_S]:
                del self._timesync_sent[key]
            self._timesync_sent[ts1] = sent

    def sync_columns(self, now: float) -> dict:
        """The FCU clock and heartbeat columns for a frame captured at Orin time now."""
        row = {}
        with self._lock:
            if self._heartbeat_at is not None:
                row.update(heartbeat=self.heartbeats, heartbeat_age_ms=f"{(now - self._heartbeat_at) * 1000:.0f}")
            recent = [s for s in self._timesync if now - s[0] <= CLOCK_WINDOW_S]
            if recent:
                _, round_trip, offset = min(recent, key=lambda s: s[1])
                row.update(fcu_time_ms=f"{(now + offset) * 1000:.1f}", fcu_clock_err_ms=f"{round_trip * 500:.1f}",
                           fcu_clock_sync="timesync")
            elif self._oneway and now - self._oneway[-1][0] <= CLOCK_WINDOW_S:
                row.update(fcu_time_ms=f"{(now + self._oneway[0][1]) * 1000:.1f}", fcu_clock_sync="oneway")
        return row

    def latest(self):
        with self._lock:
            return dict(self._latest)

    def flush(self) -> None:
        with self._lock:
            if self._raw is not None:
                self._raw.flush()


def set_rates(monitor: HeartbeatMonitor, rates: dict) -> bool:
    master, system_id, component_id = monitor.active_link()
    if master is None or system_id is None:
        return False
    for message_id, hz in rates.items():
        try:
            master.mav.command_long_send(system_id, component_id or 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                         message_id, 1_000_000.0 / hz, 0, 0, 0, 0, 0)
        except Exception:
            return False
    return True


def frame_row(index: int, video_time: float, now: float, latest: dict, monitor: HeartbeatMonitor) -> dict:
    row = {"frame": index, "video_time_s": f"{video_time:.4f}", "orin_time_unix": f"{now:.4f}"}

    def age(msg_type):
        return f"{(now - latest[msg_type][1]) * 1000:.0f}"

    if "LOCAL_POSITION_NED" in latest:
        m = latest["LOCAL_POSITION_NED"][0]
        row.update(x_north_m=f"{m.x:.3f}", y_east_m=f"{m.y:.3f}", z_down_m=f"{m.z:.3f}",
                   vx_north_mps=f"{m.vx:.3f}", vy_east_mps=f"{m.vy:.3f}", vz_down_mps=f"{m.vz:.3f}",
                   local_pos_fcu_ms=m.time_boot_ms, local_pos_age_ms=age("LOCAL_POSITION_NED"))
    if "ATTITUDE" in latest:
        m = latest["ATTITUDE"][0]
        row.update(roll_deg=f"{math.degrees(m.roll):.3f}", pitch_deg=f"{math.degrees(m.pitch):.3f}",
                   yaw_deg=f"{math.degrees(m.yaw):.3f}", roll_rate_dps=f"{math.degrees(m.rollspeed):.3f}",
                   pitch_rate_dps=f"{math.degrees(m.pitchspeed):.3f}", yaw_rate_dps=f"{math.degrees(m.yawspeed):.3f}",
                   attitude_fcu_ms=m.time_boot_ms, attitude_age_ms=age("ATTITUDE"))
    if "GLOBAL_POSITION_INT" in latest:
        m = latest["GLOBAL_POSITION_INT"][0]
        row.update(heading_deg="" if m.hdg == 65535 else f"{m.hdg / 100:.2f}",
                   lat_deg=f"{m.lat / 1e7:.7f}", lon_deg=f"{m.lon / 1e7:.7f}",
                   alt_msl_m=f"{m.alt / 1000:.3f}", alt_rel_home_m=f"{m.relative_alt / 1000:.3f}",
                   global_pos_fcu_ms=m.time_boot_ms, global_pos_age_ms=age("GLOBAL_POSITION_INT"))
    if "VFR_HUD" in latest:
        m = latest["VFR_HUD"][0]
        row.update(groundspeed_mps=f"{m.groundspeed:.3f}", climb_mps=f"{m.climb:.3f}", throttle_pct=m.throttle,
                   vfr_age_ms=age("VFR_HUD"))
    if "SCALED_IMU" in latest:
        m = latest["SCALED_IMU"][0]
        row.update(accel_x_mps2=f"{m.xacc * G / 1000:.3f}", accel_y_mps2=f"{m.yacc * G / 1000:.3f}",
                   accel_z_mps2=f"{m.zacc * G / 1000:.3f}", imu_fcu_ms=m.time_boot_ms, imu_age_ms=age("SCALED_IMU"))
    snap = monitor.snapshot()
    if snap.connected:
        row.update(mode=snap.mode_name or "", armed=int(snap.armed))
    return row


def write_status(path: Path, status: dict) -> None:
    if not path:
        return
    tmp = path.with_suffix(".tmp")
    try:
        tmp.write_text(json.dumps(status))
        os.replace(tmp, path)
    except OSError:
        pass


def fmt_duration(seconds: float) -> str:
    seconds = int(seconds)
    return f"{seconds // 3600:02d}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udp:127.0.0.1:14553", help="MAVProxy output for FCU data (default: %(default)s)")
    parser.add_argument("--device", default="0", help="camera: an index (e.g. 0) or a path (e.g. /dev/video2) (default: %(default)s)")
    parser.add_argument("--rotate", type=float, default=0.0, help="rotate frames this many degrees clockwise (default: %(default)s)")
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "recordings",
                        help="where to save the video and logs (default: %(default)s)")
    parser.add_argument("--status-file", type=Path, default=None, help="JSON status file for the drone GUI to show")
    parser.add_argument("--notify", default="127.0.0.1:14650", help="drone GUI notification port, empty = off (default: %(default)s)")
    args = parser.parse_args()

    say = Notifier(args.notify)
    stop = threading.Event()

    def on_signal(_signum, _frame):
        stop.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    monitor = HeartbeatMonitor(args.connection)
    fcu = FcuData(monitor)
    monitor.start()
    rotator = CameraLink(args.device, args.output_dir, rotation_deg=args.rotate) if args.rotate else None

    status = {"state": "starting", "started_at": time.time()}
    write_status(args.status_file, status)

    target = int(args.device) if args.device.isdigit() else args.device
    cap = cv2.VideoCapture(target)
    if not cap.isOpened():
        say(f"Recording FAILED: couldn't open camera '{args.device}' - is it in use (GUI 'camera', camera_view.py)?",
            mavutil.mavlink.MAV_SEVERITY_ERROR)
        write_status(args.status_file, dict(status, state="error", error=f"couldn't open camera '{args.device}'"))
        monitor.stop()
        sys.exit(1)
    # Ask the camera itself for 960x720 (4:3); any other size it gives is converted below.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, RECORD_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, RECORD_HEIGHT)

    # Check the camera really delivers frames, and get the frame size.
    warm = []
    check_until = time.time() + CAMERA_CHECK_S
    while len(warm) < 5 and time.time() < check_until and not stop.is_set():
        ok, frame = cap.read()
        if ok and frame is not None:
            warm.append(frame)
    if len(warm) < 2:
        say("Recording FAILED: the camera isn't delivering frames", mavutil.mavlink.MAV_SEVERITY_ERROR)
        write_status(args.status_file, dict(status, state="error", error="camera isn't delivering frames"))
        cap.release()
        monitor.stop()
        sys.exit(1)
    # The video file has one fixed frame rate, but a camera's real rate varies
    # (low light, USB load). So frames go into the file on the real clock:
    # one per 1/fps seconds - a frame is repeated when the camera is slow and
    # skipped when it's fast. Video time therefore IS real time, and the
    # frames CSV says which camera frame each video frame is (repeat=1 marks
    # a repeated one).
    reported = cap.get(cv2.CAP_PROP_FPS)
    fps = reported if MIN_FILE_FPS <= reported <= MAX_FILE_FPS else DEFAULT_FILE_FPS
    camera_h, camera_w = warm[-1].shape[:2]
    first = to_record_size(warm[-1])
    first = to_record_size(rotator._rotate_frame(first), pad=True) if rotator else first
    height, width = first.shape[:2]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = time.strftime("flight_%Y%m%d_%H%M%S")
    video_path = args.output_dir / f"{stem}.mp4"
    csv_path = args.output_dir / f"{stem}_frames.csv"
    raw_path = args.output_dir / f"{stem}_fcu_raw.jsonl"
    writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        say(f"Recording FAILED: couldn't create {video_path}", mavutil.mavlink.MAV_SEVERITY_ERROR)
        write_status(args.status_file, dict(status, state="error", error=f"couldn't create {video_path}"))
        cap.release()
        monitor.stop()
        sys.exit(1)

    csv_file = open(csv_path, "w", newline="", encoding="utf-8")
    rows = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
    rows.writeheader()
    t0 = time.time()
    fcu.start_raw_log(raw_path, t0)
    say(f"Recording: {width}x{height} at {fps:.1f} fps -> {video_path.name} (+ flight log {csv_path.name})")
    if (camera_w, camera_h) != (RECORD_WIDTH, RECORD_HEIGHT):
        say(f"Recording: the camera gives {camera_w}x{camera_h}, not {RECORD_WIDTH}x{RECORD_HEIGHT} - "
            "frames are centre-cropped to 4:3 and scaled", mavutil.mavlink.MAV_SEVERITY_WARNING)

    frames = rows_with_fcu = camera_frames = 0
    rows_with_clock, rows_timesync, worst_clock_err = 0, 0, 0.0
    rates_set, rates_set_at = False, None
    fcu_warned = False
    clock_said = clock_warned = False
    last_status = last_flush = 0.0
    error = None
    row = {}
    try:
        while not stop.is_set():
            ok, frame = cap.read()
            now = time.time()
            if not ok or frame is None:
                error = "lost the camera"
                break
            camera_frames += 1
            due = int((now - t0) * fps) + 1  # video frames that should exist by now
            if frames < due:
                frame = to_record_size(frame)
                if rotator:
                    frame = to_record_size(rotator._rotate_frame(frame), pad=True)
                row = frame_row(frames, frames / fps, now, fcu.latest(), monitor)
                row.update(fcu.sync_columns(now))
                for repeat in range(due - frames):
                    writer.write(frame)
                    rows.writerow(dict(row, frame=frames, video_time_s=f"{frames / fps:.4f}",
                                       camera_frame=camera_frames, repeat=int(repeat > 0)))
                    frames += 1
                    if "attitude_fcu_ms" in row:
                        rows_with_fcu += 1
                    if "fcu_time_ms" in row:
                        rows_with_clock += 1
                    if row.get("fcu_clock_sync") == "timesync":
                        rows_timesync += 1
                        worst_clock_err = max(worst_clock_err, float(row["fcu_clock_err_ms"]))
            # else: the camera is ahead of the file's rate - this frame is skipped

            # Once the FCU is known, and once more 2 s later: HeartbeatMonitor
            # asks for its own (lower) rates when it connects, which could
            # otherwise land after ours.
            if not rates_set:
                rates_set = set_rates(monitor, RECORD_RATES_HZ)
                rates_set_at = now
            elif rates_set_at is not None and now - rates_set_at > 2.0:
                set_rates(monitor, RECORD_RATES_HZ)
                rates_set_at = None
            if not fcu_warned and now - t0 > 10 and rows_with_fcu == 0:
                fcu_warned = True
                say(f"Recording: NO FCU DATA after 10 s on {args.connection} - video is recording, but the flight log "
                    "is empty. Is MAVProxy sending to that port?", mavutil.mavlink.MAV_SEVERITY_WARNING)
            if not clock_said and row.get("fcu_clock_sync") == "timesync":
                clock_said = True
                say(f"Recording: synced to the FCU clock (TIMESYNC on every heartbeat, +/-{row['fcu_clock_err_ms']} ms)")
            elif not clock_said and not clock_warned and now - t0 > 10 and row.get("fcu_clock_sync") == "oneway":
                clock_warned = True
                say("Recording: the FCU isn't answering TIMESYNC - frame FCU times come from the message timestamps "
                    "(one-way, error unknown)", mavutil.mavlink.MAV_SEVERITY_WARNING)

            if now - last_flush > 1.0:
                csv_file.flush()
                fcu.flush()
                last_flush = now
            if now - last_status > STATUS_INTERVAL_S:
                last_status = now
                write_status(args.status_file, {
                    "state": "recording", "video": str(video_path), "log": str(csv_path),
                    "elapsed_s": now - t0, "video_duration_s": frames / fps, "frames": frames, "fps": fps,
                    "size_bytes": video_path.stat().st_size if video_path.exists() else 0,
                    "rows_with_fcu": rows_with_fcu, "fcu_messages": fcu.raw_count,
                    "camera_fps": camera_frames / max(now - t0, 1e-6),
                    "fcu_connected": monitor.snapshot().connected,
                    "attitude": {k: row.get(k) for k in ("roll_deg", "pitch_deg", "yaw_deg", "heading_deg")},
                    "position": {k: row.get(k) for k in ("x_north_m", "y_east_m", "z_down_m", "alt_rel_home_m")},
                })
    finally:
        writer.release()
        cap.release()
        csv_file.close()
        fcu.close_raw_log()
        if rates_set:
            set_rates(monitor, RESTORE_RATES_HZ)
            time.sleep(0.2)  # let the rate-restore commands go out before the link closes
        monitor.stop()

    size = video_path.stat().st_size if video_path.exists() else 0
    duration = frames / fps
    elapsed = time.time() - t0
    camera_fps = camera_frames / max(elapsed, 1e-6)
    coverage = (100.0 * rows_with_fcu / frames) if frames else 0.0
    final = {"state": "error" if error else "stopped", "error": error, "video": str(video_path), "log": str(csv_path),
             "raw": str(raw_path), "elapsed_s": elapsed, "video_duration_s": duration, "frames": frames,
             "fps": fps, "camera_fps": camera_fps, "size_bytes": size, "rows_with_fcu": rows_with_fcu,
             "fcu_messages": fcu.raw_count}
    write_status(args.status_file, final)
    severity = mavutil.mavlink.MAV_SEVERITY_WARNING if error or coverage < 99 else mavutil.mavlink.MAV_SEVERITY_INFO
    slow = f" (camera gave {camera_fps:.0f} fps - frames repeated to keep real time)" if camera_fps < fps * 0.9 else ""
    clock_pct = (100.0 * rows_with_clock / frames) if frames else 0.0
    clock = ("not synced" if rows_with_clock == 0
             else f"TIMESYNC, worst +/-{worst_clock_err:.1f} ms" if rows_timesync == rows_with_clock
             else f"TIMESYNC on {rows_timesync}, one-way on {rows_with_clock - rows_timesync}")
    say(f"Recording {'STOPPED (' + error + ')' if error else 'saved'}: {video_path.name} - {fmt_duration(duration)}, "
        f"{size / 1e6:.1f} MB, {frames} frames at {fps:g} fps{slow}; flight log {csv_path.name} - FCU data in "
        f"{coverage:.0f}% of frames, {fcu.raw_count} raw FCU messages; FCU clock on {clock_pct:.0f}% of frames "
        f"({clock})", severity)
    sys.exit(1 if error else 0)


if __name__ == "__main__":
    main()
