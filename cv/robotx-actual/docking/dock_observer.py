#!/usr/bin/env python3
"""dock_observer: detect the red dock face, measure ZED range, geolocate it from FCU GPS/heading, show a camera +
north-up 2D map and write runs/latest/latest_status.json (+ dock_observations.csv) for cv_publish / the map.

Derived 2026-10-29 from the students' docking_cv/dock_mapper_sender.py (sha256 1140a18a...): PERCEPTION ONLY - the
water-cannon aiming (camera -> cannon offsets, HMAC packets to the Pi) moved to ../cannon/cannon_sender.py, which reads
the status file this program writes. Same detector, red mask, ZED measurement, frames and geolocation code.

Frames written per target: forward_m/right_m/down_m (camera), body_forward_m/body_right_m (rotated by camera yaw,
camera origin), fcu_forward_m/fcu_right_m (+ camera offsets from the FCU), north/east offsets and lat/lon.
This program is read-only with respect to the FCU. It sends no MAVLink commands and nothing over the network.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import socket
import sys
import time
import uuid
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
BUOYS = ROOT.parent / "buoys"                     # the buoy observer snapshot: ZED open, mask measurement, FCU telemetry
sys.path.insert(0, str(BUOYS))

from actual_buoy_observer import (  # noqa: E402
    PixhawkTelemetry,
    discover_mavlink,
    measure_mask,
    offset_to_gps,
    open_zed,
)

DEFAULT_CONFIG = ROOT / "dock_config.json"
DEFAULT_WEIGHTS = ROOT / "models" / "dock_partial_best.engine"
DEFAULT_OUTPUT = ROOT / "runs"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def load_config(path: str) -> dict:
    cfg = json.loads(Path(path).read_text())
    required = ("camera", "red_detection", "depth", "telemetry", "map")
    missing = [key for key in required if key not in cfg]
    if missing:
        raise ValueError(f"Missing config sections: {missing}")
    return cfg


def red_mask_for_box(frame: np.ndarray, xyxy, cfg: dict):
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
    x1, x2 = max(0, x1), min(width, x2)
    y1, y2 = max(0, y1), min(height, y2)
    if x2 - x1 < 8 or y2 - y1 < 8:
        return np.zeros((height, width), np.uint8), 0.0, 0

    roi = frame[y1:y2, x1:x2]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    sat_min = int(cfg["saturation_min"])
    val_min = int(cfg["value_min"])
    low1 = np.array([int(cfg["hue_low_1"]), sat_min, val_min], np.uint8)
    high1 = np.array([int(cfg["hue_high_1"]), 255, 255], np.uint8)
    low2 = np.array([int(cfg["hue_low_2"]), sat_min, val_min], np.uint8)
    high2 = np.array([int(cfg["hue_high_2"]), 255, 255], np.uint8)
    red = cv2.bitwise_or(
        cv2.inRange(hsv, low1, high1),
        cv2.inRange(hsv, low2, high2),
    )
    red = cv2.morphologyEx(
        red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )
    red = cv2.morphologyEx(
        red, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)
    )
    pixels = int(np.count_nonzero(red))
    fraction = pixels / float(max(red.size, 1))
    full = np.zeros((height, width), np.uint8)
    full[y1:y2, x1:x2] = red
    return full, fraction, pixels


def measurement_from_red_box(xyz, red_mask, cfg):
    expanded = cv2.dilate(red_mask, np.ones((7, 7), np.uint8), iterations=1)
    return measure_mask(
        xyz,
        expanded.astype(np.float32) / 255.0,
        min_valid_fraction=float(cfg["minimum_valid_fraction"]),
        max_depth_m=float(cfg["maximum_m"]),
    )


def camera_measurement_to_frames(measurement, cfg):
    """Camera-frame measurement -> body frame (camera yaw) and FCU-relative offsets. The water-cannon frame
    (cannon offsets from the camera) is applied by cannon/cannon_sender.py from body_forward_m / body_right_m."""
    camera = cfg["camera"]
    yaw = math.radians(float(camera["yaw_deg"]))
    c, s = math.cos(yaw), math.sin(yaw)

    relative_body_forward = measurement.forward_m * c - measurement.right_m * s
    relative_body_right = measurement.forward_m * s + measurement.right_m * c

    fcu_forward = float(camera["forward_from_fcu_m"]) + relative_body_forward
    fcu_right = float(camera["right_from_fcu_m"]) + relative_body_right

    return {
        "body_forward_m": relative_body_forward,
        "body_right_m": relative_body_right,
        "fcu_forward_m": fcu_forward,
        "fcu_right_m": fcu_right,
        "body_bearing_deg": math.degrees(
            math.atan2(relative_body_right, relative_body_forward)
        ),
    }


def target_geolocation(frames, telemetry):
    lat = telemetry.get("lat")
    lon = telemetry.get("lon")
    yaw = telemetry.get("yaw_rad")
    if lat is None or lon is None or yaw is None:
        return None
    north = frames["fcu_forward_m"] * math.cos(yaw) - frames["fcu_right_m"] * math.sin(yaw)
    east = frames["fcu_forward_m"] * math.sin(yaw) + frames["fcu_right_m"] * math.cos(yaw)
    target_lat, target_lon = offset_to_gps(lat, lon, north, east)
    return {
        "north_offset_m": north,
        "east_offset_m": east,
        "latitude_deg": target_lat,
        "longitude_deg": target_lon,
    }


def telemetry_is_fresh(telemetry, max_age_s):
    return (
        telemetry.get("position_age_s") is not None
        and telemetry["position_age_s"] <= max_age_s
        and telemetry.get("attitude_age_s") is not None
        and telemetry["attitude_age_s"] <= max_age_s
        and telemetry.get("heartbeat_age_s") is not None
        and telemetry["heartbeat_age_s"] <= max_age_s
        and (telemetry.get("gps_fix") or 0) >= 3
    )


def draw_map(telemetry, target, status, radius_m, size=620):
    canvas = np.full((size, size, 3), (26, 31, 38), np.uint8)
    center = np.array([size // 2, size // 2], dtype=float)
    scale = (size * 0.40) / max(float(radius_m), 1.0)

    for metres in (5, 10, 15, 20, 25):
        if metres <= radius_m:
            cv2.circle(canvas, tuple(center.astype(int)), int(metres * scale),
                       (65, 72, 82), 1)
            cv2.putText(canvas, f"{metres}m",
                        (int(center[0] + metres * scale + 3), int(center[1] - 3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (145, 155, 168), 1)

    cv2.line(canvas, (int(center[0]), 35), (int(center[0]), size - 35),
             (70, 78, 88), 1)
    cv2.line(canvas, (35, int(center[1])), (size - 35, int(center[1])),
             (70, 78, 88), 1)
    cv2.putText(canvas, "N", (int(center[0] - 8), 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 2)
    cv2.putText(canvas, "E", (size - 28, int(center[1] + 7)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 230, 230), 2)

    yaw = telemetry.get("yaw_rad")
    if yaw is None:
        yaw = 0.0
    heading_end = center + np.array(
        [math.sin(yaw), -math.cos(yaw)]
    ) * 55.0
    cv2.arrowedLine(canvas, tuple(center.astype(int)),
                    tuple(heading_end.astype(int)), (80, 210, 255), 4,
                    tipLength=0.28)
    cv2.circle(canvas, tuple(center.astype(int)), 9, (80, 210, 255), -1)
    cv2.putText(canvas, "BOAT", (int(center[0] + 12), int(center[1] - 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (80, 210, 255), 1)

    if target is not None:
        east = float(target["east_offset_m"])
        north = float(target["north_offset_m"])
        point = center + np.array([east, -north]) * scale
        point = np.clip(point, 20, size - 20).astype(int)
        cv2.circle(canvas, tuple(point), 12, (0, 0, 255), -1)
        cv2.circle(canvas, tuple(point), 17, (220, 220, 255), 2)
        cv2.line(canvas, tuple(center.astype(int)), tuple(point),
                 (80, 80, 230), 2)
        cv2.putText(canvas, "RED DOCK", (int(point[0] + 14), int(point[1] - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (80, 100, 255), 2)

    lines = [
        f"STATUS: {status}",
        f"Boat lat: {telemetry.get('lat', 'NA')}",
        f"Boat lon: {telemetry.get('lon', 'NA')}",
        f"Heading: {math.degrees(yaw):.1f} deg",
    ]
    if target is not None:
        lines += [
            f"Dock lat: {target['latitude_deg']:.8f}",
            f"Dock lon: {target['longitude_deg']:.8f}",
            f"body right: {target['body_right_m']:+.2f} m",
            f"body fwd: {target['body_forward_m']:+.2f} m",
        ]
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (18, size - 180 + index * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (225, 225, 225), 1)
    return canvas


def combine_camera_and_map(frame, map_frame):
    target_h = frame.shape[0]
    map_w = max(320, int(map_frame.shape[1] * target_h / map_frame.shape[0]))
    side = cv2.resize(map_frame, (map_w, target_h), interpolation=cv2.INTER_AREA)
    return cv2.hconcat([frame, side])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="runs/<stamp>/ + runs/latest/latest_status.json go here")
    parser.add_argument("--mavlink", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--resolution",
                        choices=["hd2k", "hd1080", "hd720", "vga"],
                        default="hd720")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--depth-mode",
                        choices=["neural", "neural_light", "ultra"],
                        default="neural")
    parser.add_argument("--conf", type=float, default=0.50)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--allow-no-fcu", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--cannon", choices=("red", "green"), default=None,
                        help="v0.1.61: also measure the cannon's target window of this colour into --cannon-out every frame")
    parser.add_argument("--cannon-out", default=None, help="v0.1.61: the target file (default: ../cannon/cannon_config.json target_file)")
    return parser.parse_args()


def run_self_test(cfg):
    class FakeMeasurement:
        forward_m = 10.0
        right_m = -15.0

    frames = camera_measurement_to_frames(FakeMeasurement(), cfg)
    assert abs(frames["body_right_m"] + 15.0) < 1e-9
    assert abs(frames["body_forward_m"] - 10.0) < 1e-9
    assert frames["body_bearing_deg"] < 0

    geo = target_geolocation(
        frames, {"lat": 1.3, "lon": 103.8, "yaw_rad": 0.0}
    )
    assert geo["north_offset_m"] > 0
    assert geo["east_offset_m"] < 0

    frame = np.zeros((100, 120, 3), np.uint8)
    frame[20:80, 30:90] = (0, 0, 255)
    mask, fraction, pixels = red_mask_for_box(
        frame, [30, 20, 90, 80], cfg["red_detection"]
    )
    assert fraction > 0.90 and pixels > 3000 and mask.any()

    map_frame = draw_map(
        {"lat": 1.3, "lon": 103.8, "yaw_rad": 0.0},
        {**frames, **geo},
        "SELF_TEST",
        cfg["map"]["radius_m"],
        400,
    )
    assert map_frame.shape == (400, 400, 3)
    print(json.dumps({
        "self_test": "PASS",
        "left_example_body_right_m": frames["body_right_m"],
        "forward_example_body_forward_m": frames["body_forward_m"],
        "bearing_deg": frames["body_bearing_deg"],
    }))


def main():
    args = parse_args()
    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
        return

    session = str(uuid.uuid4())

    endpoint = discover_mavlink() if args.mavlink == "auto" else args.mavlink
    telemetry_reader = None
    if endpoint:
        telemetry_reader = PixhawkTelemetry(endpoint, args.baud)
        telemetry_reader.start()
    elif not args.allow_no_fcu:
        raise RuntimeError(
            "No FCU MAVLink endpoint found. Connect the FCU or use "
            "--mavlink udpin:0.0.0.0:14550. Camera-only testing requires "
            "--allow-no-fcu and will never send an actionable OK target."
        )

    import pyzed.sl as sl

    model = YOLO(args.weights, task="detect")
    camera = open_zed(
        args.resolution,
        args.fps,
        args.depth_mode,
        float(cfg["depth"]["maximum_m"]),
    )
    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = 50
    runtime.texture_confidence_threshold = 100
    image, cloud = sl.Mat(), sl.Mat()

    out_root = Path(args.output)
    run_dir = out_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    latest_dir = out_root / "latest"
    latest_dir.mkdir(parents=True, exist_ok=True)
    csv_path = run_dir / "dock_observations.csv"
    fields = [
        "utc", "frame", "status", "confidence", "red_fraction",
        "range_m", "forward_m", "right_m",
        "body_forward_m", "body_right_m", "body_bearing_deg", "fcu_forward_m", "fcu_right_m",
        "vehicle_latitude_deg", "vehicle_longitude_deg",
        "vehicle_heading_deg", "latitude_deg", "longitude_deg",
        "north_offset_m", "east_offset_m",
    ]
    csv_file = csv_path.open("w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=fields)
    writer.writeheader()

    cannon = None
    if args.cannon:                                           # v0.1.61: the cannon's target from this camera (one owner)
        sys.path.insert(1, str(ROOT.parent / "cannon"))
        import aim_cv
        cannon = aim_cv.TargetWriter(args.cannon, args.cannon_out, source="dock_observer", max_depth_m=float(cfg["depth"]["maximum_m"]))
        print(f"cannon target: {args.cannon} -> {cannon.path}", flush=True)
    sequence = 0
    try:
        while not args.frames or sequence < args.frames:
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.02)
                continue
            camera.retrieve_image(image, sl.VIEW.LEFT)
            camera.retrieve_measure(cloud, sl.MEASURE.XYZRGBA)
            frame = image.get_data()[:, :, :3].copy()
            xyz = cloud.get_data()
            if cannon is not None:
                cannon.frame(frame, xyz)
            result = model.predict(
                frame,
                conf=args.conf,
                imgsz=args.imgsz,
                device=0,
                verbose=False,
            )[0]
            telemetry = (
                telemetry_reader.snapshot() if telemetry_reader else {}
            )

            boxes_seen = 0
            red_seen = 0
            candidates = []
            if result.boxes is not None:
                for box in result.boxes:
                    boxes_seen += 1
                    xyxy = box.xyxy[0].detach().cpu().numpy()
                    red_mask, red_fraction, red_pixels = red_mask_for_box(
                        frame, xyxy, cfg["red_detection"]
                    )
                    is_red = (
                        red_fraction >= float(
                            cfg["red_detection"]["minimum_red_fraction"]
                        )
                        and red_pixels >= int(
                            cfg["red_detection"]["minimum_red_pixels"]
                        )
                    )
                    measurement = None
                    if is_red:
                        red_seen += 1
                        measurement = measurement_from_red_box(
                            xyz, red_mask, cfg["depth"]
                        )
                    x1, y1, x2, y2 = map(int, xyxy)
                    colour = (0, 0, 255) if is_red else (0, 180, 255)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                    label = (
                        f"dock {float(box.conf.item()):.2f} "
                        f"red={red_fraction:.2f}"
                    )
                    cv2.putText(
                        frame, label, (max(0, x1), max(28, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, colour, 2,
                    )
                    if measurement is not None:
                        candidates.append((
                            measurement.range_m,
                            float(box.conf.item()),
                            red_fraction,
                            measurement,
                        ))

            status = "NO_DOCK"
            target = None
            if boxes_seen and not red_seen:
                status = "DOCK_NOT_RED"
            elif red_seen and not candidates:
                status = "RED_DOCK_NO_RELIABLE_DEPTH"
            elif candidates:
                _, confidence, red_fraction, measurement = min(
                    candidates, key=lambda item: item[0]
                )
                frames = camera_measurement_to_frames(measurement, cfg)
                fresh = telemetry_is_fresh(
                    telemetry, float(cfg["telemetry"]["maximum_age_s"])
                )
                geo = target_geolocation(frames, telemetry) if fresh else None
                if geo is None:
                    status = "FCU_GPS_HEADING_STALE"
                else:
                    status = "OK"
                    target = {
                        "class_name": "dock",
                        "color": "red",
                        "confidence": confidence,
                        "red_fraction": red_fraction,
                        **asdict(measurement),
                        **frames,
                        **geo,
                        "vehicle_latitude_deg": telemetry["lat"],
                        "vehicle_longitude_deg": telemetry["lon"],
                        "vehicle_heading_deg": math.degrees(
                            telemetry["yaw_rad"]
                        ),
                    }
                    cv2.putText(
                        frame,
                        (
                            f"fwd={target['body_forward_m']:+.2f}m "
                            f"right={target['body_right_m']:+.2f}m "
                            f"bearing={target['body_bearing_deg']:+.1f}deg"
                        ),
                        (15, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 0, 255), 2,
                    )

            body = {
                "protocol": "dock_observer.v1",
                "session": session,
                "sequence": sequence,
                "sent_unix_ns": time.time_ns(),
                "sent_utc": utc_now(),
                "source_host": socket.gethostname(),
                "status": status,
                "target": target,
            }

            map_frame = draw_map(
                telemetry,
                target,
                status,
                float(cfg["map"]["radius_m"]),
            )
            combined = combine_camera_and_map(frame, map_frame)
            row = {field: "" for field in fields}
            row.update(utc=body["sent_utc"], frame=sequence, status=status)
            if target:
                for key_name in fields:
                    if key_name in target:
                        row[key_name] = target[key_name]
            writer.writerow(row)
            csv_file.flush()

            status_tmp = latest_dir / "latest_status.json.tmp"
            status_tmp.write_text(json.dumps(body, indent=2))
            os.replace(status_tmp, latest_dir / "latest_status.json")
            if sequence == 0 or sequence % 5 == 0:
                cv2.imwrite(str(latest_dir / "latest_combined.jpg"), combined)
            if sequence == 0 or sequence % 15 == 0:
                print(
                    f"seq={sequence} status={status} target={target}",
                    flush=True,
                )

            if not args.headless:
                cv2.imshow("Red dock + FCU map - Q/Esc", combined)
                if cv2.waitKey(1) & 255 in (27, ord("q")):
                    break
            sequence += 1
    finally:
        if cannon is not None:
            cannon.stopped()
        stopped = {
            "protocol": "dock_observer.v1",
            "session": session,
            "sequence": sequence + 1,
            "sent_unix_ns": time.time_ns(),
            "sent_utc": utc_now(),
            "source_host": socket.gethostname(),
            "status": "OBSERVER_STOPPED",
            "target": None,
        }
        try:
            status_tmp = latest_dir / "latest_status.json.tmp"
            status_tmp.write_text(json.dumps(stopped, indent=2))
            os.replace(status_tmp, latest_dir / "latest_status.json")
        except OSError:
            pass
        csv_file.close()
        camera.close()
        if telemetry_reader:
            telemetry_reader.close()
        if not args.headless:
            cv2.destroyAllWindows()
    print(f"Logs: {run_dir}")


if __name__ == "__main__":
    main()
