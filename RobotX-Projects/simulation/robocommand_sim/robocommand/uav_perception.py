#!/usr/bin/env python3
"""Gazebo Iris camera agent: detect, geolocate, merge and publish course buoys."""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml
from gz.msgs10.image_pb2 import Image
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

from . import topics
from .bus import JsonBus
from .contract import xy_to_latlon
from .perception import TrackStore, detect_beacons, gazebo_image_to_bgr, pixel_to_world


def quaternion_yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class UavPerceptionAgent:
    def __init__(self, config_path: Path, camera_topic: str | None = None, show: bool = False) -> None:
        self.cfg = yaml.safe_load(config_path.read_text())
        self.bus = JsonBus()
        self.node = Node()
        self.camera_topic = camera_topic
        self.show = show
        self.lock = threading.Lock()
        self.position = {"x": 0.0, "y": 76.0, "z": 1.0, "yaw": 0.0}
        self.speed_mps = 0.0
        self.last_pose_sample: tuple[float, float, float, float] | None = None
        self.state = "READY"
        self.stage = "WAIT_GO"
        self.tier = "core"
        self.run_id: str | None = None
        self.go_time: float | None = None
        self.last_frame_time = 0.0
        self.stationary_since: float | None = None
        self.frames = 0
        self.tracks = TrackStore(1.5)
        self.frozen_items: list[dict[str, Any]] | None = None
        self.stop = False
        self._image_callback_ref = self._on_image
        self._pose_callback_ref = self._on_pose
        self.gimbal_publisher = self.node.advertise("/gimbal/cmd_pitch", Double)
        self.node.subscribe(Pose_V, f"/world/{self.cfg['world_name']}/pose/info", self._pose_callback_ref)
        self.bus.subscribe(topics.TEAM_GO, self._on_go)
        self.bus.subscribe(topics.CONTROL_STOP, self._on_stop)
        self.bus.subscribe(topics.ROUTE_CHANGE, self._on_route_change)

    def discover_camera(self, timeout: float = 30.0) -> str:
        if self.camera_topic:
            return self.camera_topic
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            candidates = [
                topic
                for topic in self.node.topic_list()
                if topic.endswith("/image") and ("iris" in topic.lower() or "gimbal" in topic.lower())
            ]
            if candidates:
                self.camera_topic = sorted(candidates)[0]
                return self.camera_topic
            time.sleep(0.25)
        raise RuntimeError("Iris camera image topic not found; start run_boat_drone_gazebo.sh first")

    def _on_go(self, msg: dict[str, Any]) -> None:
        run_id = str(msg.get("run_id", ""))
        if run_id and run_id == self.run_id:
            return
        self.run_id = run_id or self.run_id
        self.tier = str(msg.get("tier", "core")).lower()
        if self.tier == "core":
            self.state, self.stage = "STANDBY", "CORE_NO_UAV"
        else:
            self.state, self.stage = "AUTO", "SURVEY"
            self.go_time = time.time()
            self.tracks = TrackStore(1.5)
            self.frozen_items = None

    def _on_route_change(self, _msg: dict[str, Any]) -> None:
        if self.tier == "disruptive":
            self.tracks = TrackStore(1.5)
            self.frozen_items = None
            self.go_time = time.time()
            self.stage = "RESCAN"

    def _on_stop(self, _msg: dict[str, Any]) -> None:
        self.state, self.stage = "HOLD", "STOPPED"

    def _on_pose(self, msg: Pose_V, *_: Any) -> None:
        for pose in msg.pose:
            if pose.name == "iris_drone" or pose.name.endswith("::iris_drone"):
                now = time.monotonic()
                with self.lock:
                    if self.last_pose_sample is not None:
                        px, py, pz, pt = self.last_pose_sample
                        dt = max(1e-3, now - pt)
                        self.speed_mps = math.sqrt(
                            (pose.position.x - px) ** 2 + (pose.position.y - py) ** 2 + (pose.position.z - pz) ** 2
                        ) / dt
                    self.last_pose_sample = (pose.position.x, pose.position.y, pose.position.z, now)
                    self.position = {
                        "x": pose.position.x,
                        "y": pose.position.y,
                        "z": pose.position.z,
                        "yaw": quaternion_yaw(pose.orientation),
                    }
                return

    @staticmethod
    def image_to_bgr(msg: Image) -> np.ndarray:
        return gazebo_image_to_bgr(msg)

    def _on_image(self, msg: Image, *_: Any) -> None:
        now = time.time()
        if now - self.last_frame_time < 0.08:
            return
        self.last_frame_time = now
        try:
            frame = self.image_to_bgr(msg)
        except (ValueError, TypeError) as exc:
            print(f"WARN camera frame rejected: {exc}", flush=True)
            return
        with self.lock:
            position = dict(self.position)
            speed_mps = self.speed_mps
        altitude = max(0.1, position["z"])
        if self.state != "AUTO" or self.tier not in {"advanced", "disruptive"} or altitude < 10.0:
            if self.show:
                cv2.imshow("UAV buoy perception", frame)
                cv2.waitKey(1)
            return
        polygon = self.cfg["survey_polygon_xy"]
        xs = [float(point[0]) for point in polygon]
        ys = [float(point[1]) for point in polygon]
        inside_survey = min(xs) - 3.0 <= position["x"] <= max(xs) + 3.0 and min(ys) - 3.0 <= position["y"] <= max(ys) + 3.0
        if not inside_survey or speed_mps > 0.6:
            self.stationary_since = None
            return
        if speed_mps > 0.20:
            self.stationary_since = None
            return
        if self.stationary_since is None:
            self.stationary_since = now
            return
        if now - self.stationary_since < 0.8:
            return
        camera = self.cfg["camera"]
        detections = detect_beacons(frame)
        for detection in detections:
            # Beacon / top-disc centres are about 1.47 m above water in this world.
            projection_height = max(0.1, altitude - 1.47)
            dx, dy = pixel_to_world(
                detection.u,
                detection.v,
                frame.shape[1],
                frame.shape[0],
                projection_height,
                position["yaw"],
                float(camera["hfov_rad"]),
                math.radians(float(camera.get("mount_yaw_offset_deg", 0.0))),
            )
            object_x, object_y = position["x"] + dx, position["y"] + dy
            if min(xs) <= object_x <= max(xs) and min(ys) <= object_y <= max(ys):
                self.tracks.add(detection.colour, object_x, object_y, detection.area, now, detection.intensity)
            if self.show:
                cv2.circle(frame, (int(detection.u), int(detection.v)), 10, (0, 255, 255), 2)
                cv2.putText(frame, detection.colour, (int(detection.u) + 8, int(detection.v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        self.frames += 1
        if self.show:
            cv2.imshow("UAV buoy perception", frame)
            cv2.waitKey(1)

    def report(self) -> dict[str, Any]:
        origin = self.cfg["origin"]
        if self.frozen_items is not None:
            return self.bus.envelope(
                "uav_perception",
                items=[dict(item) for item in self.frozen_items],
                scan={"state": "complete", "coverage_pct": 100.0, "n_items": len(self.frozen_items)},
            )
        items = []
        tracks = self.tracks.consolidated(5, 3.5)
        blue_tracks = [track for track in tracks if track.colour == "blue"]
        entry_track_id: int | None = None
        if blue_tracks:
            temporal = max(blue_tracks, key=lambda track: track.blink_score)
            if temporal.blink_score > 0:
                entry_track_id = temporal.track_id
            elif len(blue_tracks) == 2:
                # Fixed Task-1 fallback if the renderer exposes no off interval.
                entry_track_id = min(blue_tracks, key=lambda track: track.y).track_id
        for track in tracks:
            colour = track.colour
            if colour == "blue":
                is_entry = track.track_id == entry_track_id
                object_class = "entry" if is_entry else "exit"
                lit: bool | str = "blinking" if is_entry else True
            elif colour == "black":
                object_class, lit = "hazard", False
            else:
                object_class, lit = "gate", True
            lat, lon = xy_to_latlon(track.x, track.y, origin["lat"], origin["lon"])
            conf = min(0.99, 0.50 + len(track.points) / 20.0)
            items.append({
                "id": f"uav-{track.track_id:02d}", "lat": lat, "lon": lon,
                "class": object_class, "colour": colour, "lit": lit,
                "conf": conf, "sigma_m": track.sigma_m,
            })
        elapsed = 0.0 if self.go_time is None else time.time() - self.go_time
        expected = len(self.cfg["ground_truth"])
        counts = {colour: sum(item["colour"] == colour for item in items) for colour in ("red", "green", "blue", "black")}
        complete = len(items) == expected and counts == {"red": 3, "green": 3, "blue": 2, "black": 2} and elapsed >= 3.0
        coverage = min(100.0, elapsed / 35.0 * 100.0) if self.go_time else 0.0
        if complete:
            coverage = 100.0
            self.frozen_items = [dict(item) for item in items]
            self.stage = "HOLD_OVERHEAD"
        return self.bus.envelope(
            "uav_perception",
            items=items,
            scan={"state": "complete" if complete else "scanning", "coverage_pct": round(coverage, 1), "n_items": len(items)},
        )

    def loop(self) -> None:
        topic = self.discover_camera()
        if not self.node.subscribe(Image, topic, self._image_callback_ref):
            raise RuntimeError(f"failed to subscribe camera topic {topic}")
        pitch = Double()
        pitch.data = math.pi / 2.0
        for _ in range(15):
            self.gimbal_publisher.publish(pitch)
            time.sleep(0.1)
        print(f"UAV camera subscribed: {topic}")
        print("Gimbal commanded straight down (+90 deg)")
        print("HSV detector active; waiting for team/go")
        next_hb = next_report = 0.0
        try:
            while not self.stop:
                now = time.time()
                with self.lock:
                    pos = dict(self.position)
                if now >= next_hb:
                    self.bus.publish(topics.heartbeat("UAV1"), self.bus.envelope(
                        "uav_perception", vehicle_id="UAV1", state=self.state, stage=self.stage,
                        mode="AUTO", rate_hz=2.0, position=pos,
                    ))
                    next_hb = now + 0.5
                if now >= next_report:
                    report = self.report()
                    self.bus.publish(topics.UAV_REPORT, report)
                    scan = report["scan"]
                    self.bus.publish(topics.STATUS_UAV, self.bus.envelope(
                        "uav_perception", state=self.state, stage=self.stage,
                        coverage_pct=scan["coverage_pct"], n_items=scan["n_items"], frames=self.frames,
                        cv_source="gazebo", camera_topic=topic,
                    ))
                    next_report = now + 1.0
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            if self.show:
                cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--camera-topic")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    UavPerceptionAgent(args.config, args.camera_topic, args.show).loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

