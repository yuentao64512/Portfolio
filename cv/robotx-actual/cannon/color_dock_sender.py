#!/usr/bin/env python3
"""Track green regions with ZED 2i colour + depth and aim the Pi servos.

This active sender uses only green image segmentation and ZED depth. Coordinates
are relative to the water-cannon reference:
x_right_m is positive right, y_forward_m is forward, z_up_m is upward.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
import socket
import time



import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent

DEFAULT_CONFIG = ROOT / "color_dock_config.json"
@dataclass
class GreenMeasurement:
    range_m: float
    forward_m: float
    right_m: float
    down_m: float
    bearing_deg: float
    valid_pixels: int
    valid_fraction: float
    depth_spread_m: float


def open_zed(resolution, fps, depth_mode, depth_max_m):
    import pyzed.sl as sl
    resolutions = {
        "hd2k": sl.RESOLUTION.HD2K,
        "hd1080": sl.RESOLUTION.HD1080,
        "hd720": sl.RESOLUTION.HD720,
        "vga": sl.RESOLUTION.VGA,
    }
    modes = {
        "neural": sl.DEPTH_MODE.NEURAL,
        "neural_light": sl.DEPTH_MODE.NEURAL_LIGHT,
        "ultra": sl.DEPTH_MODE.ULTRA,
    }
    allowed = {
        "hd2k": {15}, "hd1080": {15, 30},
        "hd720": {15, 30, 60}, "vga": {15, 30, 60, 100},
    }
    if fps not in allowed[resolution]:
        raise ValueError(f"{resolution} supports FPS {sorted(allowed[resolution])}")
    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = resolutions[resolution]
    init.camera_fps = fps
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.depth_mode = modes[depth_mode]
    init.depth_minimum_distance = 0.30
    init.depth_maximum_distance = float(depth_max_m)
    error = camera.open(init)
    if error != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Could not open ZED: {error}")
    return camera


def measure_mask(xyz, mask, min_valid_fraction=0.10, max_depth_m=25.0):
    binary = (mask > 0.5).astype(np.uint8)
    area = int(binary.sum())
    if area < 80:
        return None
    inner = cv2.erode(binary, np.ones((5, 5), np.uint8), iterations=1)
    if int(inner.sum()) < 40:
        inner = binary
    valid = (inner.astype(bool)
             & np.isfinite(xyz[:, :, :3]).all(axis=2)
             & (xyz[:, :, 2] > 0.30)
             & (xyz[:, :, 2] < float(max_depth_m)))
    fraction = float(valid.sum() / max(int(inner.sum()), 1))
    points = xyz[:, :, :3][valid]
    if len(points) < 30 or fraction < float(min_valid_fraction):
        return None
    z = points[:, 2]
    median_z = float(np.median(z))
    mad = float(np.median(np.abs(z - median_z)))
    gate = max(0.08, 3.0 * 1.4826 * mad)
    points = points[np.abs(z - median_z) <= gate]
    if len(points) < 30:
        return None
    spread = float(np.percentile(points[:, 2], 90)
                   - np.percentile(points[:, 2], 10))
    if spread > max(0.35, 0.18 * median_z):
        return None
    right, down, forward = np.median(points, axis=0)
    return GreenMeasurement(
        range_m=float(np.linalg.norm([right, down, forward])),
        forward_m=float(forward),
        right_m=float(right),
        down_m=float(down),
        bearing_deg=math.degrees(math.atan2(float(right), float(forward))),
        valid_pixels=int(len(points)),
        valid_fraction=fraction,
        depth_spread_m=spread,
    )


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    requigreen = ("network", "camera_to_cannon", "detection", "green_hsv",
                "depth", "output")
    missing = [name for name in requigreen if name not in cfg]
    if missing:
        raise ValueError(f"Missing config sections: {missing}")
    return cfg


def signed_packet(body, key):
    raw = json.dumps(body, separators=(",", ":"), sort_keys=True).encode()
    signature = hmac.new(key, raw, hashlib.sha256).hexdigest()
    return json.dumps({"body": body, "hmac_sha256": signature},
                      separators=(",", ":")).encode()


def green_mask(frame, cfg):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    green = cfg["green_hsv"]
    low1 = np.array([green["hue_low_1"], green["saturation_min"],
                     green["value_min"]], np.uint8)
    high1 = np.array([green["hue_high_1"], 255, 255], np.uint8)
    low2 = np.array([green["hue_low_2"], green["saturation_min"],
                     green["value_min"]], np.uint8)
    high2 = np.array([green["hue_high_2"], 255, 255], np.uint8)
    mask = cv2.bitwise_or(cv2.inRange(hsv, low1, high1),
                          cv2.inRange(hsv, low2, high2))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((5, 5), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((11, 11), np.uint8))
    return mask


def green_components(mask, minimum_pixels):
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    components = []
    for label in range(1, count):
        x, y, width, height, area = stats[label]
        if int(area) < int(minimum_pixels) or width < 8 or height < 8:
            continue
        component = np.zeros_like(mask)
        component[labels == label] = 255
        bbox_area = max(int(width * height), 1)
        fraction = float(area) / bbox_area
        components.append({
            "mask": component,
            "bbox": [int(x), int(y), int(x + width), int(y + height)],
            "pixels": int(area),
            "fraction": fraction,
        })
    return components


def cannon_coordinates(measurement, cfg):
    mount = cfg["camera_to_cannon"]
    yaw = math.radians(float(mount["camera_yaw_deg"]))
    c, s = math.cos(yaw), math.sin(yaw)
    forward = measurement.forward_m * c - measurement.right_m * s
    right = measurement.forward_m * s + measurement.right_m * c
    y_forward = forward + float(mount["camera_forward_from_cannon_m"])
    x_right = right + float(mount["camera_right_from_cannon_m"])
    z_up = -measurement.down_m + float(mount["camera_up_from_cannon_m"])
    horizontal = math.hypot(x_right, y_forward)
    return {
        "x_right_m": x_right,
        "y_forward_m": y_forward,
        "z_up_m": z_up,
        "horizontal_m": horizontal,
        "range_m": math.hypot(horizontal, z_up),
        "bearing_deg": math.degrees(math.atan2(x_right, y_forward)),
        "elevation_deg": math.degrees(math.atan2(z_up, horizontal)),
    }


def direction_text(x_right):
    if abs(x_right) < 0.05:
        return "CENTGREEN"
    return f"{abs(x_right):.2f}m {'RIGHT' if x_right > 0 else 'LEFT'}"


def make_body(session, sequence, status, target):
    return {
        "protocol": "dock_color_aim.v1",
        "session": session,
        "sequence": sequence,
        "sent_unix_ns": time.time_ns(),
        "sent_utc": utc_now(),
        "source_host": socket.gethostname(),
        "status": status,
        "target": target,
    }


def synthetic_target():
    return {
        "class_name": "dock", "color": "green",
        "confidence": 1.0, "color_fraction": 0.8,
        "x_right_m": -0.20, "y_forward_m": 5.0, "z_up_m": 0.10,
        "horizontal_m": math.hypot(0.20, 5.0),
        "range_m": math.sqrt(0.20 ** 2 + 5.0 ** 2 + 0.10 ** 2),
        "bearing_deg": math.degrees(math.atan2(-0.20, 5.0)),
        "elevation_deg": math.degrees(math.atan2(0.10, math.hypot(0.20, 5.0))),
    }


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--resolution", choices=["hd2k", "hd1080", "hd720", "vga"],
                        default="hd720")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--depth-mode", choices=["neural", "neural_light", "ultra"],
                        default="neural")
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--synthetic-send", action="store_true")
    return parser.parse_args()


def run_self_test(cfg):
    frame = np.zeros((160, 240, 3), np.uint8)
    frame[30:130, 60:180] = (0, 255, 0)
    components = green_components(green_mask(frame, cfg), 80)
    assert len(components) == 1
    assert components[0]["fraction"] > 0.90

    class Measurement:
        forward_m = 5.0
        right_m = -0.20
        down_m = -0.10
    coords = cannon_coordinates(Measurement(), cfg)
    assert coords["x_right_m"] < 0 and coords["bearing_deg"] < 0
    print(json.dumps({"self_test": "PASS", "green_only": True,
                      "model_loaded": False, "green_fraction": components[0]["fraction"],
                      "example": coords}, indent=2))


def main():
    args = parse_args()
    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
        return

    key = Path(cfg["network"]["key_file"]).read_text().strip().encode()
    if len(key) < 32:
        raise RuntimeError("HMAC key must contain at least 32 characters")
    destination = (str(cfg["network"]["pi_ip"]),
                   int(cfg["network"]["pi_port"]))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session = str(uuid.uuid4())
    if args.synthetic_send:
        for sequence in range(12):
            body = make_body(session, sequence, "OK", synthetic_target())
            sock.sendto(signed_packet(body, key), destination)
            time.sleep(0.07)
        print(json.dumps({"synthetic_send": "PASS", "green_only": True,
                          "packets": 12, "destination": destination,
                          "target": body["target"]}, indent=2))
        sock.close()
        return

    import pyzed.sl as sl
    camera = open_zed(args.resolution, args.fps, args.depth_mode,
                      float(cfg["depth"]["maximum_m"]))
    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = 100
    runtime.texture_confidence_threshold = 100
    image, cloud = sl.Mat(), sl.Mat()

    output_root = Path(cfg["output"]["directory"])
    run_dir = output_root / datetime.now().strftime("%Y%m%d_%H%M%S")
    latest_dir = output_root / "latest"
    run_dir.mkdir(parents=True, exist_ok=True)
    latest_dir.mkdir(parents=True, exist_ok=True)
    log_file = (run_dir / "green_observations.jsonl").open("a", buffering=1)
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
            mask = green_mask(frame, cfg)
            minimum_pixels = max(200, int(cfg["detection"]["minimum_color_pixels"]))
            components = green_components(mask, minimum_pixels)
            candidates = []
            for component in components:
                expanded = cv2.dilate(component["mask"],
                                      np.ones((7, 7), np.uint8), iterations=1)
                measurement = measure_mask(
                    xyz, expanded.astype(np.float32) / 255.0,
                    min_valid_fraction=float(cfg["depth"]["minimum_valid_fraction"]),
                    max_depth_m=float(cfg["depth"]["maximum_m"]),
                )
                x1, y1, x2, y2 = component["bbox"]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"GREEN area={component['pixels']}",
                            (x1, max(28, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, (0, 255, 0), 2)
                if measurement is None:
                    continue
                coords = cannon_coordinates(measurement, cfg)
                color_score = min(1.0, 0.5 + 0.5 * component["fraction"])
                candidates.append({
                    "class_name": "dock", "color": "green",
                    "confidence": color_score,
                    "color_fraction": component["fraction"],
                    "green_pixels": component["pixels"],
                    "bbox_xyxy": component["bbox"],
                    "valid_depth_pixels": measurement.valid_pixels,
                    "valid_depth_fraction": measurement.valid_fraction,
                    "depth_spread_m": measurement.depth_spread_m,
                    **coords,
                })

            if candidates:
                target = min(candidates, key=lambda item: item["range_m"])
                status = "OK"
                text = (f"GREEN {direction_text(target['x_right_m'])} "
                        f"forward={target['y_forward_m']:.2f}m "
                        f"up={target['z_up_m']:+.2f}m")
                cv2.putText(frame, text, (15, 38), cv2.FONT_HERSHEY_SIMPLEX,
                            0.70, (0, 255, 0), 2)
            else:
                target = None
                status = ("GREEN_NO_RELIABLE_DEPTH" if components
                          else "NO_GREEN_TARGET")

            body = make_body(session, sequence, status, target)
            sock.sendto(signed_packet(body, key), destination)
            log_file.write(json.dumps(body, separators=(",", ":")) + "\n")
            temporary = latest_dir / "latest_status.json.tmp"
            temporary.write_text(json.dumps(body, indent=2))
            os.replace(temporary, latest_dir / "latest_status.json")
            if sequence % 5 == 0:
                cv2.imwrite(str(latest_dir / "latest.jpg"), frame)
            if sequence % 15 == 0:
                print(f"seq={sequence} status={status} green_components="
                      f"{len(components)} target={target} "
                      f"dest={destination[0]}:{destination[1]}", flush=True)
            if not args.headless:
                cv2.imshow("ZED green-only target - Q/Esc", frame)
                if cv2.waitKey(1) & 255 in (27, ord("q")):
                    break
            sequence += 1
    finally:
        try:
            body = make_body(session, sequence + 1, "SENDER_STOPPED", None)
            sock.sendto(signed_packet(body, key), destination)
        except OSError:
            pass
        log_file.close()
        camera.close()
        sock.close()
        if not args.headless:
            cv2.destroyAllWindows()
    print(f"Logs: {run_dir}")


if __name__ == "__main__":
    main()

