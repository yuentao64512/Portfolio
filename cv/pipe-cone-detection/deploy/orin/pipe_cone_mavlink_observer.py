#!/usr/bin/env python3
"""Fuse Pixhawk EKF pose with ZED cone observations into a local 2D map.

Inputs:
  * MAVLink LOCAL_POSITION_NED and ATTITUDE/ATTITUDE_QUATERNION from the FCU.
  * UDP JSON cone observations from live_cone_map_yolo26_zed.py.

The map frame is ArduPilot local NED projected onto North/East. Camera
measurements use the ZED optical frame: x right, y down, z forward.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import select
import socket
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from collections import deque

from pymavlink import mavutil

WGS84_RADIUS_M = 6_378_137.0
MAP_STREAM = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


@dataclass
class Pose:
    north_m: float = 0.0
    east_m: float = 0.0
    down_m: float = 0.0
    yaw_rad: float = 0.0
    position_monotonic: float = 0.0
    attitude_monotonic: float = 0.0
    source: str = "none"


@dataclass
class Landmark:
    landmark_id: int
    cone_class: str
    north_m: float
    east_m: float
    observations: int
    first_seen_utc: str
    last_seen_utc: str
    last_seen_monotonic: float
    last_camera_frame: int
    history: list[tuple[float, float]] = field(default_factory=list)


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mavlink", default="/dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--listen-host", default="127.0.0.1")
    p.add_argument("--listen-port", type=int, default=15250)
    p.add_argument("--output", default="/home/jetson/pipe_cones/logs/map")
    p.add_argument("--camera-forward-m", type=float, default=0.0)
    p.add_argument("--camera-right-m", type=float, default=0.0)
    p.add_argument("--camera-yaw-deg", type=float, default=0.0,
                   help="camera forward yaw clockwise from vehicle forward")
    p.add_argument("--fixed-position", action="store_true",
                   help="bench test only: use a fixed local position instead of FCU position")
    p.add_argument("--fixed-north-m", type=float, default=0.0)
    p.add_argument("--fixed-east-m", type=float, default=0.0)
    p.add_argument("--pose-max-age-s", type=float, default=0.50)
    p.add_argument("--association-gate-m", type=float, default=3.0)
    p.add_argument("--memory-length", type=int, default=10,
                   help="number of recent detections retained per cone")
    p.add_argument("--decay", type=float, default=0.92,
                   help="weight multiplier for each older detection")
    p.add_argument("--track-timeout-s", type=float, default=5.0,
                   help="remove unseen cones after N seconds; 0 latches them permanently")
    p.add_argument("--min-confidence", type=float, default=0.15)
    p.add_argument("--min-valid-fraction", type=float, default=0.10)
    p.add_argument("--max-range-m", type=float, default=40.0)
    p.add_argument("--status-period-s", type=float, default=2.0)
    args = p.parse_args()
    if args.memory_length < 1:
        p.error("--memory-length must be at least 1")
    if not 0.0 < args.decay <= 1.0:
        p.error("--decay must be in (0, 1]")
    if args.track_timeout_s < 0.0:
        p.error("--track-timeout-s cannot be negative")
    return args


def quaternion_yaw(q1, q2, q3, q4):
    return math.atan2(2.0 * (q1 * q4 + q2 * q3), 1.0 - 2.0 * (q3 * q3 + q4 * q4))


def request_streams(link):
    """Request 20 Hz pose telemetry; do not change the FCU's internal EKF rate."""
    target_system = link.target_system or 1
    target_component = link.target_component or 1
    for message_id, interval_us in (
        (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 50_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 200_000),
        (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 500_000),
    ):
        link.mav.command_long_send(
            target_system, target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval_us, 0, 0, 0, 0, 0,
        )


def recv_match_safe(link, **kwargs):
    """Skip a malformed/cache-conflicting packet without dropping the serial link."""
    try:
        return link.recv_match(**kwargs)
    except (TypeError, ValueError, AttributeError) as error:
        print(f"Skipped invalid MAVLink packet: {error}", flush=True)
        return None


def wait_heartbeat_safe(link, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        message = recv_match_safe(link, type="HEARTBEAT", blocking=False)
        if message is not None and message.get_srcSystem() not in {0, 255}:
            link.target_system = message.get_srcSystem()
            link.target_component = message.get_srcComponent()
            return message
        time.sleep(0.01)
    return None


def sample_rate(state, kind, boot_ms, now):
    """Count unique FCU samples using boot timestamps, independent of polling."""
    samples = state.setdefault("sample_clocks", {}).setdefault(kind, deque(maxlen=51))
    if samples and boot_ms <= samples[-1][0]:
        if boot_ms == samples[-1][0]:
            return False
        samples.clear()  # reboot/wrap: start a new measurement window
    samples.append((boot_ms, now))
    return True


def measured_rate(state, kind):
    samples = state.get("sample_clocks", {}).get(kind, [])
    if len(samples) < 2 or time.monotonic()-samples[-1][1] > 2.0:
        return 0.0
    return 1000.0*(len(samples)-1)/(samples[-1][0]-samples[0][0])


def update_pose(link, pose, state, on_sample=None):
    while True:
        msg = recv_match_safe(link, blocking=False)
        if msg is None:
            return
        now = time.monotonic()
        kind = msg.get_type()
        if msg.get_srcSystem() != link.target_system or msg.get_srcComponent() != link.target_component:
            continue
        if kind in {"LOCAL_POSITION_NED", "GLOBAL_POSITION_INT", "ATTITUDE", "ATTITUDE_QUATERNION"}:
            if not sample_rate(state, kind, int(msg.time_boot_ms), now):
                continue
        old_position_time = pose.position_monotonic
        if kind == "LOCAL_POSITION_NED":
            if pose.source == "GLOBAL_POSITION_INT_LOCAL_ORIGIN":
                continue  # Keep the run's established origin; never mix map frames.
            pose.north_m = float(msg.x)
            pose.east_m = float(msg.y)
            pose.down_m = float(msg.z)
            pose.position_monotonic = now
            pose.source = "LOCAL_POSITION_NED"
        elif kind == "ATTITUDE":
            pose.yaw_rad = float(msg.yaw)
            pose.attitude_monotonic = now
        elif kind == "ATTITUDE_QUATERNION":
            pose.yaw_rad = quaternion_yaw(msg.q1, msg.q2, msg.q3, msg.q4)
            pose.attitude_monotonic = now
        elif kind == "EKF_STATUS_REPORT":
            state["ekf_flags"] = int(msg.flags)
        elif kind == "GLOBAL_POSITION_INT":
            lat_deg = msg.lat / 1e7
            lon_deg = msg.lon / 1e7
            state["lat_deg"] = lat_deg
            state["lon_deg"] = lon_deg
            if abs(lat_deg) > 1e-9 or abs(lon_deg) > 1e-9:
                if "origin_lat_deg" not in state:
                    state["origin_lat_deg"] = lat_deg
                    state["origin_lon_deg"] = lon_deg
                # Prefer native local NED whenever it has arrived recently.
                if pose.source in {"none", "GLOBAL_POSITION_INT_LOCAL_ORIGIN"}:
                    origin_lat = state["origin_lat_deg"]
                    origin_lon = state["origin_lon_deg"]
                    pose.north_m = math.radians(lat_deg - origin_lat) * WGS84_RADIUS_M
                    pose.east_m = (
                        math.radians(lon_deg - origin_lon)
                        * WGS84_RADIUS_M
                        * math.cos(math.radians(origin_lat))
                    )
                    pose.down_m = -float(msg.relative_alt) / 1000.0
                    pose.position_monotonic = now
                    pose.source = "GLOBAL_POSITION_INT_LOCAL_ORIGIN"
        elif kind == "GPS_RAW_INT":
            state["gps_fix_type"] = int(msg.fix_type)
            state["satellites_visible"] = int(msg.satellites_visible) if msg.satellites_visible != 255 else None
            state["gps_received_monotonic"] = now
        if on_sample and (pose.position_monotonic != old_position_time or kind in {"ATTITUDE", "ATTITUDE_QUATERNION"}):
            state["observer_clock"] = kind if pose.source in {"none", "FIXED_BENCH_POSITION"} else ("LOCAL_POSITION_NED" if pose.source == "LOCAL_POSITION_NED" else "GLOBAL_POSITION_INT")
            on_sample(kind, int(msg.time_boot_ms))


def body_to_map(forward_m, right_m, yaw_rad):
    north = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
    east = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
    return north, east


def transform_observation(obs, pose, args):
    camera_yaw = math.radians(args.camera_yaw_deg)
    cam_forward = float(obs["camera_z_forward_m"])
    cam_right = float(obs["camera_x_right_m"])
    body_forward = args.camera_forward_m + cam_forward * math.cos(camera_yaw) - cam_right * math.sin(camera_yaw)
    body_right = args.camera_right_m + cam_forward * math.sin(camera_yaw) + cam_right * math.cos(camera_yaw)
    dn, de = body_to_map(body_forward, body_right, pose.yaw_rad)
    return pose.north_m + dn, pose.east_m + de, body_forward, body_right


def decay_average(history, decay):
    weighted_north = weighted_east = weight_total = 0.0
    for age, (north_m, east_m) in enumerate(reversed(history)):
        weight = decay ** age
        weighted_north += weight * north_m
        weighted_east += weight * east_m
        weight_total += weight
    return weighted_north / weight_total, weighted_east / weight_total


def associate(
    landmarks, cone_class, north_m, east_m, gate_m, timestamp,
    now_monotonic, camera_frame, memory_length=50, decay=0.92,
):
    candidates = [
        # Match against the latest observation, not the lagging display average.
        (math.hypot((item.history[-1][0] if item.history else item.north_m) - north_m,
                    (item.history[-1][1] if item.history else item.east_m) - east_m), item)
        for item in landmarks
        if item.cone_class == cone_class and item.last_camera_frame != camera_frame
    ]
    distance, match = min(candidates, default=(math.inf, None), key=lambda pair: pair[0])
    if match is None or distance > gate_m:
        next_id = max((item.landmark_id for item in landmarks), default=0) + 1
        match = Landmark(
            next_id, cone_class, north_m, east_m, 1, timestamp, timestamp,
            now_monotonic, camera_frame, [(north_m, east_m)],
        )
        landmarks.append(match)
        return match, "new"
    match.history.append((north_m, east_m))
    if len(match.history) > memory_length:
        del match.history[:-memory_length]
    match.north_m, match.east_m = decay_average(match.history, decay)
    match.observations += 1
    match.last_seen_utc = timestamp
    match.last_seen_monotonic = now_monotonic
    match.last_camera_frame = camera_frame
    return match, "updated"


def expire_tracks(landmarks, now_monotonic, timeout_s):
    if timeout_s <= 0.0:
        return False
    before = len(landmarks)
    landmarks[:] = [item for item in landmarks if now_monotonic - item.last_seen_monotonic <= timeout_s]
    return before != len(landmarks)


def offset_to_geographic(latitude_deg, longitude_deg, north_m, east_m):
    """Apply a short North/East offset to a WGS84 latitude/longitude."""
    latitude = latitude_deg + math.degrees(north_m / WGS84_RADIUS_M)
    longitude_scale = WGS84_RADIUS_M * math.cos(math.radians(latitude_deg))
    if abs(longitude_scale) < 1e-9:
        return None, None
    longitude = longitude_deg + math.degrees(east_m / longitude_scale)
    return latitude, longitude


def landmark_payload(item, pose, state):
    payload = {
        "trace": item.history,
        "spread_rms_m": math.sqrt(
            sum((n-item.north_m)**2 + (e-item.east_m)**2 for n, e in item.history)
            / len(item.history)
        ) if item.history else None,
        **{key: value for key, value in asdict(item).items()
           if key not in {"last_seen_monotonic", "last_camera_frame", "history"}},
    }
    latitude = state.get("lat_deg")
    longitude = state.get("lon_deg")
    if latitude is not None and longitude is not None and (state.get("gps_fix_type") or 0) >= 3:
        cone_lat, cone_lon = offset_to_geographic(
            latitude, longitude,
            item.north_m - pose.north_m,
            item.east_m - pose.east_m,
        )
        payload["latitude_deg"] = cone_lat
        payload["longitude_deg"] = cone_lon
    else:
        payload["latitude_deg"] = None
        payload["longitude_deg"] = None
    return payload


def write_map(path, landmarks, pose, state):
    payload = {
        "frame": "LOCAL_NED_2D",
        "updated_utc": datetime.now(timezone.utc).isoformat(),
        "vehicle": {
            "north_m": pose.north_m,
            "east_m": pose.east_m,
            "yaw_deg": math.degrees(pose.yaw_rad),
            "pose_source": pose.source,
            "latitude_deg": state.get("lat_deg"),
            "longitude_deg": state.get("lon_deg"),
        },
        "origin": {"latitude_deg": state.get("origin_lat_deg"), "longitude_deg": state.get("origin_lon_deg")},
        "ekf_flags": state.get("ekf_flags"),
        "vehicle_trace": state.get("vehicle_trace", []),
        "diagnostics": {"position_age_s": time.monotonic() - pose.position_monotonic if pose.position_monotonic else None,
                        "attitude_age_s": time.monotonic() - pose.attitude_monotonic if pose.attitude_monotonic else None,
                        "ground_truth_error_m": None,
                        "note": "Cone spread and innovations measure consistency, not absolute accuracy."},
        "gps": {"fix_type": state.get("gps_fix_type"), "satellites_visible": state.get("satellites_visible"),
                "latitude_deg": state.get("lat_deg"), "longitude_deg": state.get("lon_deg"),
                "age_s": time.monotonic() - state["gps_received_monotonic"] if "gps_received_monotonic" in state else None},
        "rates_hz": {
            "cv": round(float(state.get("cv_rate_hz", 0.0)), 2),
            "observer": round(measured_rate(state, state.get("observer_clock")), 2),
            "position": round(measured_rate(state, "LOCAL_POSITION_NED" if pose.source == "LOCAL_POSITION_NED" else "GLOBAL_POSITION_INT"), 2),
            "attitude": round(measured_rate(state, "ATTITUDE"), 2),
        },
        "observer_rate_source": state.get("observer_clock", "waiting for FCU state"),
        "landmarks": [landmark_payload(item, pose, state) for item in landmarks],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(payload, indent=2))
    temporary.replace(path)
    # Per-message chunks keep a large landmark map below UDP datagram limits.
    encoded = json.dumps(payload, separators=(",", ":")).encode()
    import base64
    transfer = time.monotonic_ns()
    chunks = [encoded[i:i+30000] for i in range(0, len(encoded), 30000)]
    for index, chunk in enumerate(chunks):
        MAP_STREAM.sendto(json.dumps({"id": transfer, "index": index, "count": len(chunks),
                                     "data": base64.b64encode(chunk).decode()}).encode(), ("127.0.0.1", 15260))


def main():
    args = parse_args()
    run_dir = Path(args.output) / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    link = mavutil.mavlink_connection(args.mavlink, baud=args.baud, autoreconnect=True)
    heartbeat = wait_heartbeat_safe(link, 10.0)
    if heartbeat is None:
        raise RuntimeError(f"No MAVLink heartbeat on {args.mavlink}")
    request_streams(link)
    print("Requested 20 Hz pose telemetry (50,000 us); displayed rates are measured from FCU timestamps.", flush=True)
    if args.fixed_position:
        print(
            "WARNING: fixed observer position enabled for stationary bench testing; "
            "no fake GPS is sent to the FCU.",
            flush=True,
        )

    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind((args.listen_host, args.listen_port))
    udp.setblocking(False)
    pose, state, landmarks = Pose(), {}, []
    accepted = rejected = 0
    map_path = run_dir / "pipe_cone_map.json"
    last_status = 0.0
    pose_output = (run_dir / "vehicle_state.csv").open("w", newline="")
    pose_log = csv.writer(pose_output)
    pose_log.writerow([
        "utc", "source", "north_m", "east_m", "yaw_deg", "step_m",
        "latitude_deg", "longitude_deg", "gps_fix_type", "satellites_visible",
        "position_age_s", "attitude_age_s", "ekf_flags", "trigger", "fcu_time_boot_ms",
    ])

    def on_sample(kind, boot_ms):
        if args.fixed_position:
            pose.north_m, pose.east_m, pose.down_m = args.fixed_north_m, args.fixed_east_m, 0.0
            pose.position_monotonic = pose.attitude_monotonic
            pose.source = "FIXED_BENCH_POSITION"
        trace = state.setdefault("vehicle_trace", [])
        step = math.hypot(pose.north_m-trace[-1][0], pose.east_m-trace[-1][1]) if trace else 0.0
        position_event = kind in {"LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"} or args.fixed_position
        if position_event and pose.position_monotonic:
            trace.append([pose.north_m, pose.east_m])
            del trace[:-args.memory_length]
        pose_log.writerow([
            datetime.now(timezone.utc).isoformat(), pose.source, pose.north_m, pose.east_m,
            math.degrees(pose.yaw_rad), step, state.get("lat_deg"), state.get("lon_deg"),
            state.get("gps_fix_type"), state.get("satellites_visible"),
            time.monotonic()-pose.position_monotonic if pose.position_monotonic else None,
            time.monotonic()-pose.attitude_monotonic if pose.attitude_monotonic else None,
            state.get("ekf_flags"), kind, boot_ms,
        ])
        pose_output.flush()
        write_map(map_path, landmarks, pose, state)

    with (run_dir / "pipe_cone_geolocation.csv").open("w", newline="") as output:
        log = csv.writer(output)
        log.writerow([
            "received_utc", "camera_timestamp_ns", "class", "confidence", "range_m",
            "vehicle_north_m", "vehicle_east_m", "vehicle_yaw_deg",
            "body_forward_m", "body_right_m", "cone_north_m", "cone_east_m",
            "vehicle_latitude_deg", "vehicle_longitude_deg",
            "cone_latitude_deg", "cone_longitude_deg", "gps_fix_type", "satellites_visible",
            "landmark_id", "landmark_observations",
        ])
        print(f"MAVLink heartbeat: system={link.target_system} component={link.target_component}", flush=True)
        print(f"Listening for cone observations on udp://{args.listen_host}:{args.listen_port}", flush=True)
        while True:
            # Expire before association so stale tracks cannot steal new detections.
            if expire_tracks(landmarks, time.monotonic(), args.track_timeout_s):
                write_map(map_path, landmarks, pose, state)
            update_pose(link, pose, state, on_sample)
            # Wake on serial bytes or CV datagrams; timeout only services status.
            ready, _, _ = select.select([udp, link.port], [], [], 0.5)
            if udp in ready:
                raw, _ = udp.recvfrom(65535)
                received = datetime.now(timezone.utc).isoformat()
                try:
                    obs = json.loads(raw.decode("utf-8"))
                    now = time.monotonic()
                    previous_cv_time = state.get("last_cv_monotonic") if obs.get("frame") != state.get("last_cv_frame") else None
                    if previous_cv_time is not None and now > previous_cv_time:
                        instant_cv_rate = 1.0 / (now - previous_cv_time)
                        previous_cv_rate = float(state.get("cv_rate_hz", instant_cv_rate))
                        state["cv_rate_hz"] = 0.8 * previous_cv_rate + 0.2 * instant_cv_rate
                    if obs.get("frame") != state.get("last_cv_frame"):
                        state["last_cv_monotonic"] = now
                        state["last_cv_frame"] = obs.get("frame")
                    pose_age = max(now - pose.position_monotonic, now - pose.attitude_monotonic)
                    confidence = float(obs["confidence"])
                    valid_fraction = float(obs["valid_fraction"])
                    range_m = float(obs["range_m"])
                    if pose_age > args.pose_max_age_s:
                        raise ValueError(f"stale EKF pose ({pose_age:.2f}s)")
                    if confidence < args.min_confidence or valid_fraction < args.min_valid_fraction:
                        raise ValueError("measurement quality below threshold")
                    if not 0.0 < range_m <= args.max_range_m:
                        raise ValueError("range outside configured bounds")
                    north, east, body_forward, body_right = transform_observation(obs, pose, args)
                    landmark, _ = associate(
                        landmarks, str(obs["class"]), north, east,
                        args.association_gate_m, received, now,
                        int(obs.get("frame", -1)), args.memory_length, args.decay,
                    )
                    vehicle_lat = state.get("lat_deg")
                    vehicle_lon = state.get("lon_deg")
                    cone_lat = cone_lon = None
                    if vehicle_lat is not None and vehicle_lon is not None and (state.get("gps_fix_type") or 0) >= 3:
                        cone_lat, cone_lon = offset_to_geographic(
                            vehicle_lat, vehicle_lon,
                            north - pose.north_m,
                            east - pose.east_m,
                        )
                    log.writerow([
                        received, obs.get("timestamp_ns"), obs["class"], confidence, range_m,
                        pose.north_m, pose.east_m, math.degrees(pose.yaw_rad),
                        body_forward, body_right, north, east,
                        vehicle_lat, vehicle_lon, cone_lat, cone_lon,
                        state.get("gps_fix_type"), state.get("satellites_visible"),
                        landmark.landmark_id, landmark.observations,
                    ])
                    output.flush()
                    accepted += 1
                    write_map(map_path, landmarks, pose, state)
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
                    rejected += 1
                    print(f"Rejected cone observation: {error}", flush=True)
            if expire_tracks(landmarks, time.monotonic(), args.track_timeout_s):
                write_map(map_path, landmarks, pose, state)
            now = time.monotonic()
            if now - last_status >= args.status_period_s:
                pos_age = now - pose.position_monotonic if pose.position_monotonic else math.inf
                att_age = now - pose.attitude_monotonic if pose.attitude_monotonic else math.inf
                print(
                    f"pose[{pose.source}] N={pose.north_m:.2f} E={pose.east_m:.2f} yaw={math.degrees(pose.yaw_rad):.1f}deg "
                    f"age(pos/att)={pos_age:.2f}/{att_age:.2f}s "
                    f"gps_fix={state.get('gps_fix_type')} sats={state.get('satellites_visible')} "
                    f"cv={state.get('cv_rate_hz', 0.0):.1f}Hz observer={measured_rate(state, state.get('observer_clock')):.1f}Hz[{state.get('observer_clock')}] "
                    f"landmarks={len(landmarks)} accepted={accepted} rejected={rejected}",
                    flush=True,
                )
                last_status = now


if __name__ == "__main__":
    main()
