#!/usr/bin/env python3
"""Read-only CSV and event recorder for the four-terminal Gazebo/OCS stack.

Subscribes to the existing Gazebo transport bus, boat odometry, and the four
thruster command topics. It never publishes control or actuator commands.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import threading
import time
import uuid

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry

from . import topics
from .bus import JsonBus


JOINTS = ("motor_port_joint", "motor_stbd_joint", "thruster_front_joint", "thruster_rear_joint")
THRUSTER_LABELS = ("port", "starboard", "bow", "stern")
BUS_TOPICS = (
    topics.DECLARATION, topics.RUN_START, topics.TEAM_GO, topics.CONTROL_STOP,
    topics.COURSE, topics.UAV_REPORT, topics.USV_UAV_REPORT, topics.ROUTE_CHANGE,
    topics.STATUS_UAV, topics.STATUS_USV, topics.USV_CV, topics.USV_ROUTE,
    topics.USV_DRIVE_STATUS, topics.heartbeat("UAV1"), topics.heartbeat("USV1"),
)
COLUMNS = (
    "wall_time_utc", "elapsed_s", "run_id", "tier",
    "uav_state", "uav_stage", "uav_x_m", "uav_y_m", "uav_z_m",
    "uav_heartbeat_age_s", "uav_camera_frames", "coverage_pct", "buoy_count",
    "usv_state", "usv_stage", "usv_x_m", "usv_y_m", "usv_z_m",
    "usv_yaw_rad", "usv_odom_age_s", "usv_heartbeat_age_s",
    "usv_camera_frames", "usv_markers", "drive_state", "route_index",
    "route_total", "distance_to_waypoint_m", "thrust_port", "thrust_starboard",
    "thrust_bow", "thrust_stern", "last_thrust_age_s",
)


def utc_iso(timestamp: float) -> str:
    return datetime.fromtimestamp(timestamp, timezone.utc).isoformat(timespec="milliseconds")


class RunRecorder:
    def __init__(self, output: Path, package: Path, sample_hz: float = 5.0):
        if sample_hz <= 0 or sample_hz > 50:
            raise ValueError("sample_hz must be in (0, 50]")
        self.package = package
        self.started_wall = time.time()
        self.started_mono = time.monotonic()
        stamp = datetime.now().strftime("run_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:6]
        self.directory = output / stamp
        self.directory.mkdir(parents=True, exist_ok=False)
        self.csv_file = (self.directory / "telemetry.csv").open("w", newline="", encoding="utf-8")
        self.events_file = (self.directory / "events.jsonl").open("w", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=COLUMNS)
        self.csv_writer.writeheader()
        self.bus = JsonBus()
        self.lock = threading.Lock()
        self.latest: dict[str, dict] = {}
        self.odom: dict | None = None
        self.odom_at = 0.0
        self.thrust: dict[str, float] = {}
        self.thrust_at = 0.0
        self.run_id = ""
        self.tier = ""
        self.event_count = 0
        self.sample_count = 0
        self.odom_count = 0
        self.thrust_updates = 0
        self.report_count = 0
        self.peak_abs_thrust = 0.0
        self.closed = False
        self.sample_period = 1.0 / sample_hz
        (self.directory / "metadata.json").write_text(json.dumps({
            "source": "RoboCommand Gazebo Transport, read-only recorder",
            "started_utc": utc_iso(self.started_wall), "sample_hz": sample_hz,
            "bus_topics": list(BUS_TOPICS),
            "odom_topic": "/model/usv_four_thruster/odometry",
            "thruster_topics": [f"/model/usv_four_thruster/joint/{joint}/cmd_thrust" for joint in JOINTS],
            "thrust_units": "Gazebo joint cmd_thrust values; not measured force or ESC PWM",
        }, indent=2), encoding="utf-8")
        config = package / "config/task1_world.yaml"
        if config.is_file():
            shutil.copy2(config, self.directory / "task1_world.yaml")
        for topic in BUS_TOPICS:
            if not self.bus.subscribe(topic, lambda msg, selected=topic: self.on_bus(selected, msg)):
                raise RuntimeError(f"Could not subscribe to {topic}")
        if not self.bus.node.subscribe(Odometry, "/model/usv_four_thruster/odometry", self.on_odom):
            raise RuntimeError("Could not subscribe to boat odometry")
        self._thrust_callbacks = []
        for joint, label in zip(JOINTS, THRUSTER_LABELS):
            topic = f"/model/usv_four_thruster/joint/{joint}/cmd_thrust"
            def callback(msg: Double, *_: object, selected=label) -> None:
                self.on_thrust(selected, float(msg.data))
            self._thrust_callbacks.append(callback)
            if not self.bus.node.subscribe(Double, topic, callback):
                raise RuntimeError(f"Could not subscribe to {topic}")

    def on_bus(self, topic: str, message: dict) -> None:
        wall = time.time()
        with self.lock:
            if self.closed:
                return
            self.latest[topic] = message
            self.events_file.write(json.dumps({"received_utc": utc_iso(wall),
                                               "topic": topic, "message": message},
                                              separators=(",", ":")) + "\n")
            self.event_count += 1
            if topic == topics.UAV_REPORT:
                self.report_count += 1
            if topic == topics.RUN_START:
                self.run_id = str(message.get("run_id", ""))
                self.tier = str(message.get("tier", ""))
            elif topic == topics.DECLARATION:
                self.tier = str(message.get("tier", self.tier))
            if self.event_count % 10 == 0:
                self.events_file.flush()

    def on_odom(self, message: Odometry, *_: object) -> None:
        position, q = message.pose.position, message.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        with self.lock:
            if self.closed:
                return
            self.odom = {"x": position.x, "y": position.y, "z": position.z, "yaw": yaw}
            self.odom_at = time.time()
            self.odom_count += 1

    def on_thrust(self, label: str, value: float) -> None:
        with self.lock:
            if self.closed:
                return
            self.thrust[label] = value
            self.thrust_at = time.time()
            self.thrust_updates += 1
            self.peak_abs_thrust = max(self.peak_abs_thrust, abs(value))

    @staticmethod
    def _age(now: float, message: dict | None) -> float | str:
        if not message:
            return ""
        try:
            return round(max(0., now-float(message["ts"])), 3)
        except (KeyError, TypeError, ValueError):
            return ""

    def sample(self) -> None:
        now = time.time()
        with self.lock:
            latest = dict(self.latest)
            odom = dict(self.odom) if self.odom else None
            odom_at = self.odom_at
            thrust = dict(self.thrust)
            thrust_at = self.thrust_at
            run_id, tier = self.run_id, self.tier
        uav_hb = latest.get(topics.heartbeat("UAV1"), {})
        usv_hb = latest.get(topics.heartbeat("USV1"), {})
        uav_status = latest.get(topics.STATUS_UAV, {})
        usv_status = latest.get(topics.STATUS_USV, {})
        drive = latest.get(topics.USV_DRIVE_STATUS, {})
        uav_pos = uav_hb.get("position", {})
        usv_pos = usv_hb.get("position", {})
        row = {"wall_time_utc": utc_iso(now), "elapsed_s": round(time.monotonic()-self.started_mono, 3),
               "run_id": run_id, "tier": tier,
               "uav_state": uav_hb.get("state", ""), "uav_stage": uav_hb.get("stage", ""),
               "uav_x_m": uav_pos.get("x", ""), "uav_y_m": uav_pos.get("y", ""),
               "uav_z_m": uav_pos.get("z", ""),
               "uav_heartbeat_age_s": self._age(now, uav_hb),
               "uav_camera_frames": uav_status.get("frames", ""),
               "coverage_pct": uav_status.get("coverage_pct", ""),
               "buoy_count": uav_status.get("n_items", ""),
               "usv_state": usv_hb.get("state", ""), "usv_stage": usv_hb.get("stage", ""),
               "usv_x_m": odom.get("x", "") if odom else usv_pos.get("x", ""),
               "usv_y_m": odom.get("y", "") if odom else usv_pos.get("y", ""),
               "usv_z_m": odom.get("z", "") if odom else usv_pos.get("z", ""),
               "usv_yaw_rad": odom.get("yaw", "") if odom else "",
               "usv_odom_age_s": round(now-odom_at, 3) if odom_at else "",
               "usv_heartbeat_age_s": self._age(now, usv_hb),
               "usv_camera_frames": usv_status.get("cv_frames", ""),
               "usv_markers": usv_status.get("cv_detections", ""),
               "drive_state": drive.get("state", ""),
               "route_index": drive.get("route_index", ""),
               "route_total": drive.get("route_total", ""),
               "distance_to_waypoint_m": drive.get("distance_m", ""),
               "last_thrust_age_s": round(now-thrust_at, 3) if thrust_at else ""}
        for label in THRUSTER_LABELS:
            row[f"thrust_{label}"] = thrust.get(label, "")
        self.csv_writer.writerow(row)
        self.sample_count += 1
        if self.sample_count % 5 == 0:
            self.csv_file.flush()

    def run(self, duration: float | None = None) -> None:
        print(f"RECORDER ready: {self.directory}", flush=True)
        print("Start/declaration/go in the other terminals; Ctrl+C here after stopping them.", flush=True)
        try:
            next_sample = time.monotonic()
            while duration is None or time.monotonic()-self.started_mono < duration:
                now = time.monotonic()
                if now >= next_sample:
                    self.sample()
                    next_sample += self.sample_period
                    if next_sample < now:
                        next_sample = now + self.sample_period
                time.sleep(min(.05, max(.005, next_sample-time.monotonic())))
        except KeyboardInterrupt:
            pass
        finally:
            self.close()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        with self.lock:
            self.csv_file.flush()
            self.events_file.flush()
            self.csv_file.close()
            self.events_file.close()
            latest = dict(self.latest)
            summary = {"started_utc": utc_iso(self.started_wall),
                       "ended_utc": utc_iso(time.time()),
                       "elapsed_s": round(time.monotonic()-self.started_mono, 3),
                       "run_id": self.run_id, "tier": self.tier,
                       "telemetry_rows": self.sample_count, "bus_events": self.event_count,
                       "boat_odometry_messages": self.odom_count,
                       "thruster_command_messages": self.thrust_updates,
                       "peak_abs_gazebo_thrust_command": round(self.peak_abs_thrust, 4),
                       "uav_buoy_reports": self.report_count,
                       "last_uav_status": latest.get(topics.STATUS_UAV),
                       "last_usv_status": latest.get(topics.STATUS_USV),
                       "last_drive_status": latest.get(topics.USV_DRIVE_STATUS),
                       "note": "Recorded messages and requested Gazebo thrust; not an OCS score or physical output"}
        copied = []
        console_dir = self.directory / "console_logs"
        for source in sorted((self.package / "logs").glob("*.log")):
            if source.stat().st_mtime >= self.started_wall - 2:
                console_dir.mkdir(exist_ok=True)
                shutil.copy2(source, console_dir / source.name)
                copied.append(source.name)
        summary["console_logs_copied"] = copied
        (self.directory / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
        print(f"RECORDER saved: {self.directory}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    default_package = Path(__file__).resolve().parents[1]
    parser.add_argument("--output", type=Path, default=default_package / "logs/recordings")
    parser.add_argument("--sample-hz", type=float, default=5.)
    parser.add_argument("--duration", type=float, help="Optional automatic stop for a short smoke test")
    args = parser.parse_args()
    RunRecorder(args.output, default_package, args.sample_hz).run(args.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
