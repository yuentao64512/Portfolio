#!/usr/bin/env python3
"""Read-only buoy detector: YOLO26 segmentation + ZED depth + Pixhawk GPS.

This program never arms the vehicle and never sends movement commands.  It
reports the visible buoy surface in the ZED optical frame and projects that
offset from the current Pixhawk position using the Pixhawk yaw estimate.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl
from pymavlink import mavutil
from ultralytics import YOLO

EARTH_M = 6_378_137.0
WEIGHTS = "/home/optiprime-agxorin/actual_buoy/best.pt"
OUTPUT = "/home/optiprime-agxorin/actual_buoy/observer_logs"
DEFAULT_MAVLINK = os.environ.get(
    "ACTUAL_BUOY_MAVLINK", "udpin:0.0.0.0:14550"
)


@dataclass(frozen=True)
class BuoyMeasurement:
    range_m: float
    forward_m: float
    right_m: float
    down_m: float
    bearing_deg: float
    valid_pixels: int
    valid_fraction: float
    depth_spread_m: float


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def discover_mavlink():
    patterns = [
        "/dev/serial/by-id/*ArduPilot*",
        "/dev/serial/by-id/*Pixhawk*",
        "/dev/serial/by-id/*Cube*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    for pattern in patterns:
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


class PixhawkTelemetry:
    """Background read-only telemetry cache; only requests message intervals."""

    def __init__(self, endpoint, baud=115200):
        self.endpoint = endpoint
        self.link = mavutil.mavlink_connection(endpoint, baud=baud, autoreconnect=True)
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.state = {
            "heartbeat": None, "armed": None, "mode": None,
            "lat": None, "lon": None, "relative_alt_m": None,
            "yaw_rad": None, "gps_fix": None, "satellites": None,
            "position_time": 0.0, "attitude_time": 0.0, "gps_time": 0.0,
        }
        self.thread = None

    def start(self, heartbeat_timeout=8.0):
        hb = self.link.wait_heartbeat(timeout=heartbeat_timeout)
        if hb is None:
            raise RuntimeError(f"No Pixhawk heartbeat on {self.endpoint}")
        self.link.target_system = hb.get_srcSystem()
        self.link.target_component = hb.get_srcComponent()
        for msg_id, interval_us in (
            (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 500_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 200_000),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100_000),
        ):
            self.link.mav.command_long_send(
                self.link.target_system, self.link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, interval_us, 0, 0, 0, 0, 0,
            )
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            try:
                msg = self.link.recv_match(blocking=True, timeout=0.2)
            except (TypeError, ValueError, AttributeError, OSError):
                continue
            if msg is None:
                continue
            now = time.monotonic()
            kind = msg.get_type()
            with self.lock:
                if kind == "HEARTBEAT":
                    self.state["heartbeat"] = now
                    self.state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    self.state["mode"] = mavutil.mode_string_v10(msg)
                    self.state["autopilot"] = int(msg.autopilot)
                    self.state["vehicle_type"] = int(msg.type)
                elif kind == "GLOBAL_POSITION_INT":
                    lat, lon = msg.lat / 1e7, msg.lon / 1e7
                    if abs(lat) > 1e-9 or abs(lon) > 1e-9:
                        self.state.update(lat=lat, lon=lon,
                                          relative_alt_m=msg.relative_alt / 1000.0,
                                          position_time=now)
                    if msg.hdg != 65535:
                        self.state.update(yaw_rad=math.radians(msg.hdg / 100.0),
                                          attitude_time=now)
                elif kind == "ATTITUDE":
                    self.state.update(yaw_rad=float(msg.yaw), attitude_time=now)
                elif kind == "GPS_RAW_INT":
                    self.state.update(gps_fix=int(msg.fix_type),
                                      satellites=None if msg.satellites_visible == 255 else int(msg.satellites_visible),
                                      gps_time=now)

    def snapshot(self):
        with self.lock:
            s = dict(self.state)
        now = time.monotonic()
        s["position_age_s"] = now - s["position_time"] if s["position_time"] else None
        s["attitude_age_s"] = now - s["attitude_time"] if s["attitude_time"] else None
        s["heartbeat_age_s"] = now - s["heartbeat"] if s["heartbeat"] else None
        return s

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=1.0)
        self.link.close()


def offset_to_gps(lat, lon, north_m, east_m):
    out_lat = lat + math.degrees(north_m / EARTH_M)
    scale = EARTH_M * math.cos(math.radians(lat))
    if abs(scale) < 1e-6:
        raise ValueError("Longitude scale is invalid near the pole")
    out_lon = lon + math.degrees(east_m / scale)
    return out_lat, out_lon


def measure_mask(xyz, mask, min_valid_fraction=0.25, max_depth_m=20.0):
    """Median visible surface point from an eroded segmentation mask."""
    binary = (mask > 0.5).astype(np.uint8)
    area = int(binary.sum())
    if area < 80:
        return None
    kernel = np.ones((5, 5), np.uint8)
    inner = cv2.erode(binary, kernel, iterations=1)
    if inner.sum() < 40:
        inner = binary
    valid = (inner.astype(bool) & np.isfinite(xyz[:, :, :3]).all(axis=2)
             & (xyz[:, :, 2] > 0.30) & (xyz[:, :, 2] < float(max_depth_m)))
    fraction = float(valid.sum() / max(int(inner.sum()), 1))
    points = xyz[:, :, :3][valid]
    if len(points) < 30 or fraction < min_valid_fraction:
        return None
    z = points[:, 2]
    med_z = float(np.median(z))
    mad = float(np.median(np.abs(z - med_z)))
    gate = max(0.08, 3.0 * 1.4826 * mad)
    points = points[np.abs(z - med_z) <= gate]
    if len(points) < 30:
        return None
    spread = float(np.percentile(points[:, 2], 90) - np.percentile(points[:, 2], 10))
    if spread > max(0.35, 0.18 * med_z):
        return None
    right, down, forward = np.median(points, axis=0)
    return BuoyMeasurement(
        range_m=float(np.linalg.norm([right, down, forward])),
        forward_m=float(forward), right_m=float(right), down_m=float(down),
        bearing_deg=math.degrees(math.atan2(float(right), float(forward))),
        valid_pixels=int(len(points)), valid_fraction=fraction,
        depth_spread_m=spread,
    )


def buoy_geolocation(measurement, telemetry, camera_forward_m=0.0,
                     camera_right_m=0.0, camera_yaw_deg=0.0):
    required = (telemetry.get("lat"), telemetry.get("lon"), telemetry.get("yaw_rad"))
    if any(value is None for value in required):
        return None
    cam_yaw = math.radians(camera_yaw_deg)
    body_forward = camera_forward_m + measurement.forward_m * math.cos(cam_yaw) - measurement.right_m * math.sin(cam_yaw)
    body_right = camera_right_m + measurement.forward_m * math.sin(cam_yaw) + measurement.right_m * math.cos(cam_yaw)
    yaw = telemetry["yaw_rad"]
    north = body_forward * math.cos(yaw) - body_right * math.sin(yaw)
    east = body_forward * math.sin(yaw) + body_right * math.cos(yaw)
    lat, lon = offset_to_gps(telemetry["lat"], telemetry["lon"], north, east)
    return {"latitude_deg": lat, "longitude_deg": lon,
            "north_offset_m": north, "east_offset_m": east,
            "body_forward_m": body_forward, "body_right_m": body_right}


def open_zed(resolution, fps, depth_mode, depth_max_m=20.0):
    resolutions = {"hd2k": sl.RESOLUTION.HD2K, "hd1080": sl.RESOLUTION.HD1080,
                   "hd720": sl.RESOLUTION.HD720, "vga": sl.RESOLUTION.VGA}
    modes = {"neural": sl.DEPTH_MODE.NEURAL, "neural_light": sl.DEPTH_MODE.NEURAL_LIGHT,
             "ultra": sl.DEPTH_MODE.ULTRA}
    allowed = {"hd2k": {15}, "hd1080": {15, 30}, "hd720": {15, 30, 60}, "vga": {15, 30, 60, 100}}
    if fps not in allowed[resolution]:
        raise ValueError(f"{resolution} supports FPS values {sorted(allowed[resolution])}")
    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = resolutions[resolution]
    init.camera_fps = fps
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.depth_mode = modes[depth_mode]
    init.depth_minimum_distance = 0.30
    if not 1.0 <= depth_max_m <= 40.0:
        raise ValueError("depth_max_m must be between 1 and 40 metres")
    init.depth_maximum_distance = float(depth_max_m)
    error = camera.open(init)
    if error != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Could not open ZED: {error}")
    return camera


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--weights", default=WEIGHTS)
    p.add_argument("--output", default=OUTPUT)
    p.add_argument(
        "--mavlink",
        default=DEFAULT_MAVLINK,
        help=(
            f"pymavlink endpoint (default: {DEFAULT_MAVLINK}); "
            "use 'auto' to discover a local USB/serial Pixhawk"
        ),
    )
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--resolution", choices=["hd2k", "hd1080", "hd720", "vga"], default="hd2k")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--depth-mode", choices=["neural", "neural_light", "ultra"], default="neural")
    p.add_argument("--conf", type=float, default=0.60)
    p.add_argument("--imgsz", type=int, default=768)
    p.add_argument("--frames", type=int, default=0)
    p.add_argument("--camera-forward-m", type=float, default=0.0)
    p.add_argument("--camera-right-m", type=float, default=0.0)
    p.add_argument("--camera-yaw-deg", type=float, default=0.0)
    p.add_argument("--telemetry-max-age-s", type=float, default=0.75)
    p.add_argument("--headless", action="store_true")
    p.add_argument("--allow-no-pixhawk", action="store_true", help="bench camera test only; GPS output will be unavailable")
    p.add_argument("--self-test", action="store_true")
    return p.parse_args()


def run_self_test():
    lat, lon = offset_to_gps(1.3000, 103.8000, 10.0, 5.0)
    assert lat > 1.3000 and lon > 103.8000
    fake = np.zeros((20, 20, 3), np.float32); fake[:, :, 2] = 4.0; fake[:, :, 0] = 0.5
    result = measure_mask(fake, np.ones((20, 20), np.float32), 0.2)
    assert result and abs(result.forward_m - 4.0) < 1e-6
    print(json.dumps({"self_test": "PASS", "offset_lat": lat, "offset_lon": lon, "range_m": result.range_m}))


def main():
    args = parse_args()
    if args.self_test:
        run_self_test(); return
    endpoint = discover_mavlink() if args.mavlink == "auto" else args.mavlink
    telemetry = None
    if endpoint:
        telemetry = PixhawkTelemetry(endpoint, args.baud); telemetry.start()
    elif not args.allow_no_pixhawk:
        raise RuntimeError("No Pixhawk device found. Connect it or pass --mavlink ENDPOINT. For camera-only bench use, pass --allow-no-pixhawk.")

    run_dir = Path(args.output) / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.weights, task="segment")
    camera = open_zed(args.resolution, args.fps, args.depth_mode)
    runtime = sl.RuntimeParameters(); runtime.confidence_threshold = 50; runtime.texture_confidence_threshold = 100
    image, cloud = sl.Mat(), sl.Mat()
    analysed = 0
    csv_file = (run_dir / "buoy_observations.csv").open("w", newline="")
    fields = ["utc", "frame", "confidence", "range_m", "forward_m", "right_m", "down_m", "bearing_deg",
              "valid_fraction", "valid_pixels", "depth_spread_m", "vehicle_lat", "vehicle_lon", "vehicle_yaw_deg",
              "gps_fix", "satellites", "buoy_lat", "buoy_lon", "north_offset_m", "east_offset_m", "status"]
    writer = csv.DictWriter(csv_file, fieldnames=fields); writer.writeheader()
    try:
        while not args.frames or analysed < args.frames:
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.02); continue
            camera.retrieve_image(image, sl.VIEW.LEFT)
            camera.retrieve_measure(cloud, sl.MEASURE.XYZRGBA)
            frame = image.get_data()[:, :, :3].copy()
            xyz = cloud.get_data()
            result = model.predict(frame, conf=args.conf, imgsz=args.imgsz, device=0, verbose=False)[0]
            tel = telemetry.snapshot() if telemetry else {}
            candidates = []
            if result.boxes is not None and result.masks is not None:
                masks = result.masks.data.cpu().numpy()
                for i, box in enumerate(result.boxes):
                    mask = cv2.resize(masks[i], (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)
                    measurement = measure_mask(xyz, mask)
                    if measurement:
                        candidates.append((measurement.range_m, float(box.conf.item()), measurement, mask, box))
            selected = min(candidates, default=None, key=lambda item: item[0])
            status = "NO_RELIABLE_BUOY"
            row = {key: "" for key in fields}; row.update(utc=utc_now(), frame=analysed, status=status)
            if selected:
                _, confidence, measurement, mask, box = selected
                fresh = (tel.get("position_age_s") is not None and tel["position_age_s"] <= args.telemetry_max_age_s
                         and tel.get("attitude_age_s") is not None and tel["attitude_age_s"] <= args.telemetry_max_age_s
                         and (tel.get("gps_fix") or 0) >= 3)
                geo = buoy_geolocation(measurement, tel, args.camera_forward_m, args.camera_right_m, args.camera_yaw_deg) if fresh else None
                status = "OK" if geo else "BUOY_DEPTH_OK_GPS_UNAVAILABLE"
                row.update(confidence=confidence, **asdict(measurement),
                           vehicle_lat=tel.get("lat", ""), vehicle_lon=tel.get("lon", ""),
                           vehicle_yaw_deg=math.degrees(tel["yaw_rad"]) if tel.get("yaw_rad") is not None else "",
                           gps_fix=tel.get("gps_fix", ""), satellites=tel.get("satellites", ""), status=status)
                if geo:
                    row.update(buoy_lat=geo["latitude_deg"], buoy_lon=geo["longitude_deg"],
                               north_offset_m=geo["north_offset_m"], east_offset_m=geo["east_offset_m"])
                colour = np.zeros_like(frame); colour[:, :, 1] = (mask > 0.5).astype(np.uint8) * 180
                frame = cv2.addWeighted(frame, 1.0, colour, 0.45, 0)
                x1,y1,x2,y2 = map(int, box.xyxy[0].cpu().numpy())
                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.putText(frame, f"buoy {measurement.range_m:.2f}m bearing {measurement.bearing_deg:+.1f}deg", (max(0,x1),max(35,y1-10)), 0, 0.7,(0,255,0),2)
            writer.writerow(row); csv_file.flush()
            payload = {"updated_utc": row["utc"], "frame": analysed, "status": status,
                       "measurement": ({k: row[k] for k in fields if row[k] != ""}), "telemetry": tel}
            tmp = run_dir / "latest_status.json.tmp"; tmp.write_text(json.dumps(payload, indent=2)); os.replace(tmp, run_dir / "latest_status.json")
            if analysed == 0 or analysed % 15 == 0:
                cv2.imwrite(str(run_dir / "latest_preview.jpg"), frame)
                print(f"frame={analysed} status={status} range={row['range_m'] or 'NA'} buoy_gps={row['buoy_lat'] or 'NA'},{row['buoy_lon'] or 'NA'}", flush=True)
            if not args.headless:
                cv2.imshow("Actual buoy observer - Q/Esc", frame)
                if cv2.waitKey(1) & 255 in (27, ord('q')): break
            analysed += 1
    finally:
        csv_file.close(); camera.close()
        if telemetry: telemetry.close()
        if not args.headless: cv2.destroyAllWindows()
    print(f"Logs: {run_dir}")


if __name__ == "__main__":
    main()
