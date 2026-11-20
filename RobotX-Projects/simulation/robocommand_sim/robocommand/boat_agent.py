#!/usr/bin/env python3
"""USV mission state machine: wait for UAV map, validate and plan safe passage."""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Any

import yaml
from gz.msgs10.image_pb2 import Image

from . import topics
from .bus import JsonBus
from .contract import latlon_to_xy, validate_report
from .perception import detect_usv_markers, gazebo_image_to_bgr


class BoatAgent:
    def __init__(self, config_path: Path) -> None:
        self.cfg = yaml.safe_load(config_path.read_text())
        self.bus = JsonBus()
        self.lock = threading.Lock()
        self.state = "READY"
        self.stage = "WAIT_GO"
        self.tier = "core"
        self.route: list[dict[str, Any]] = []
        self.latest_map: dict[str, Any] | None = None
        self.run_id = ""
        self.route_key = ""
        self.drive_status: dict[str, Any] | None = None
        self.last_route_publish = 0.0
        self.awaiting_rescan = False
        self.position = {"x": 0.0, "y": 37.0, "z": 0.12}
        self.cv_topic = "/usv/front_camera/image"
        self.cv_frames = 0
        self.cv_detections = 0
        self.cv_last_frame = 0.0
        self.cv_started = 0.0
        self.cv_last_log = 0.0
        self._image_callback_ref = self._on_sim_image
        if not self.bus.node.subscribe(Image, self.cv_topic, self._image_callback_ref):
            raise RuntimeError(f"Could not subscribe simulated USV camera: {self.cv_topic}")
        self.bus.subscribe(topics.TEAM_GO, self._go)
        self.bus.subscribe(topics.USV_UAV_REPORT, self._map)
        self.bus.subscribe(topics.CONTROL_STOP, self._stop)
        self.bus.subscribe(topics.ROUTE_CHANGE, self._route_change)
        self.bus.subscribe(topics.USV_DRIVE_STATUS, self._drive_status)

    def _go(self, msg: dict[str, Any]) -> None:
        self.tier = str(msg.get("tier", "core")).lower()
        self.run_id = str(msg.get("run_id", ""))
        self.route = []
        self.route_key = ""
        self.awaiting_rescan = False
        self.state = "AUTO"
        if self.tier == "core":
            self.stage = "T1_CORE_SIM_CV_WAIT"
            print(f"CORE: waiting for Gazebo boat camera {self.cv_topic}", flush=True)
        else:
            self.stage = "WAIT_UAV_MAP"
            print(f"{self.tier.upper()}: WAIT_UAV_MAP; Gazebo boat CV also active", flush=True)
        self.cv_started = time.time()

    def _on_sim_image(self, msg: Image, *_: Any) -> None:
        now = time.time()
        if now - self.cv_last_frame < 0.2:
            return
        try:
            frame = gazebo_image_to_bgr(msg)
            detections = detect_usv_markers(frame)
        except (ValueError, TypeError) as exc:
            print(f"USV simulated camera frame rejected: {exc}", flush=True)
            return
        with self.lock:
            self.cv_frames += 1
            self.cv_last_frame = now
            self.cv_detections = len(detections)
            if self.state == "AUTO" and self.tier == "core":
                self.stage = "T1_CORE_SIM_CV"
        if self.state != "AUTO":
            return
        observation = self.bus.envelope(
            "usv_gazebo_cv", tier=self.tier, camera_topic=self.cv_topic,
            frame=self.cv_frames, width=frame.shape[1], height=frame.shape[0],
            detections=[{
                "colour": item.colour,
                "center_px": [round(item.u, 1), round(item.v, 1)],
                "area_px": round(item.area, 1),
                "bearing_deg": round(math.degrees(math.atan(
                    (item.u - frame.shape[1] / 2.0)
                    / (frame.shape[1] / (2.0 * math.tan(1.65 / 2.0)))
                )), 1),
                "confidence": round(item.confidence, 2),
            } for item in detections],
        )
        self.bus.publish(topics.USV_CV, observation)
        if now - self.cv_last_log >= 2.0:
            colours = [item.colour for item in detections]
            print(f"SIM_USV_CV frames={self.cv_frames} detections={colours} topic={self.cv_topic}", flush=True)
            self.cv_last_log = now

    def _stop(self, _msg: dict[str, Any]) -> None:
        self.state, self.stage = "HOLD", "STOPPED"
        self.route = []
        self.route_key = ""
        print("Boat state -> HOLD")

    def _route_change(self, _msg: dict[str, Any]) -> None:
        if self.tier == "disruptive":
            self.stage = "WAIT_UAV_MAP"
            self.latest_map = None
            self.route = []
            self.route_key = ""
            self.awaiting_rescan = True
            print("Route changed: old route invalidated, WAIT_UAV_MAP")

    def _drive_status(self, msg: dict[str, Any]) -> None:
        if str(msg.get("run_id", "")) != self.run_id or self.state != "AUTO":
            return
        with self.lock:
            self.drive_status = msg
            self.position = dict(msg.get("position", self.position))
            if self.route and msg.get("state") in {"MOVING", "DONE", "NO_ODOM"}:
                self.stage = {"MOVING": "ROUTE_FOLLOWING", "DONE": "ROUTE_COMPLETE", "NO_ODOM": "DRIVE_NO_ODOM"}[msg["state"]]

    def _map(self, msg: dict[str, Any]) -> None:
        if self.state != "AUTO":
            return
        if self.tier not in {"advanced", "disruptive"}:
            return
        ok, reason = validate_report(msg)
        if not ok:
            print(f"Rejected UAV map: {reason}")
            return
        if self.awaiting_rescan:
            if msg["scan"].get("state") != "complete":
                self.awaiting_rescan = False
            return
        if msg["scan"].get("state") != "complete":
            self.stage = "WAIT_UAV_MAP"
            return
        route = self.plan_route(msg["items"])
        if not route:
            self.stage = "WAIT_UAV_MAP"
            return
        route_key = str([(round(p["x"], 2), round(p["y"], 2)) for p in route])
        if route_key == self.route_key:
            return
        with self.lock:
            self.latest_map = msg
            self.route = route
            self.route_key = route_key
            self.stage = "ROUTE_READY"
        self.bus.publish_reliable(topics.USV_ROUTE, self.bus.envelope(
            "boat_agent", run_id=self.run_id, tier=self.tier, route=route,
        ))
        self.last_route_publish = time.monotonic()
        print("Safe-passage route ready:")
        for index, point in enumerate(route, 1):
            print(f"  {index:02d} {point['action']:<14} x={point['x']:.2f} y={point['y']:.2f}")
        print("Simulation route published for Gazebo boat driver", flush=True)

    def plan_route(self, items: list[dict[str, Any]]) -> list[dict[str, Any]]:
        origin = self.cfg["origin"]
        converted = []
        for item in items:
            x, y = latlon_to_xy(float(item["lat"]), float(item["lon"]), origin["lat"], origin["lon"])
            converted.append({**item, "x": x, "y": y})
        entries = [i for i in converted if i["class"] == "entry" and i["colour"] == "blue"]
        exits = [i for i in converted if i["class"] == "exit" and i["colour"] == "blue"]
        greens = sorted([i for i in converted if i["colour"] == "green"], key=lambda i: i["y"])
        reds = sorted([i for i in converted if i["colour"] == "red"], key=lambda i: i["y"])
        if len(entries) != 1 or len(exits) != 1 or not greens or len(greens) != len(reds):
            print(f"Incomplete map: entry={len(entries)} exit={len(exits)} green={len(greens)} red={len(reds)}")
            return []
        route: list[dict[str, Any]] = []
        entry = entries[0]
        # Approach from the outside east of the buoy course, then join the
        # entrance from the south. This avoids a diagonal cut through gates.
        route.extend([
            {"action": "TRANSIT_EAST", "x": 12.0, "y": 37.0},
            {"action": "TRANSIT_SOUTH", "x": 12.0, "y": -36.0},
            {"action": "APPROACH_ENTRY", "x": entry["x"] + 3.0, "y": -36.0},
        ])
        radius = 3.0
        for degree in range(0, -361, -60):
            angle = math.radians(degree)
            route.append({"action": "ENTRY_CW", "x": entry["x"] + radius * math.cos(angle), "y": entry["y"] + radius * math.sin(angle)})
        for gate_index, (green, red) in enumerate(zip(greens, reds), 1):
            route.append({"action": f"GATE_{gate_index}", "x": (green["x"] + red["x"]) / 2, "y": (green["y"] + red["y"]) / 2})
        exit_item = exits[0]
        for degree in range(0, 361, 60):
            angle = math.radians(degree)
            route.append({"action": "EXIT_CCW", "x": exit_item["x"] + radius * math.cos(angle), "y": exit_item["y"] + radius * math.sin(angle)})
        return route

    def loop(self) -> None:
        print(f"USV Gazebo CV subscribed: {self.cv_topic}", flush=True)
        next_hb = next_status = 0.0
        try:
            while True:
                now = time.time()
                if now >= next_hb:
                    self.bus.publish(topics.heartbeat("USV1"), self.bus.envelope(
                        "boat_agent", vehicle_id="USV1", state=self.state, stage=self.stage,
                        mode="AUTO", rate_hz=2.0, position=self.position,
                    ))
                    next_hb = now + 0.5
                if now >= next_status:
                    if self.state == "AUTO" and self.route and now - self.last_route_publish >= 1.0:
                        self.bus.publish(topics.USV_ROUTE, self.bus.envelope(
                            "boat_agent", run_id=self.run_id, tier=self.tier, route=self.route,
                        ))
                        self.last_route_publish = now
                    if self.state == "AUTO" and self.tier == "core" and self.cv_started and now - self.cv_started > 5.0 and now - self.cv_last_frame > 5.0:
                        self.stage = "SIM_CAMERA_NO_FRAMES"
                    self.bus.publish(topics.STATUS_USV, self.bus.envelope(
                        "boat_agent", state=self.state, tier=self.tier, stage=self.stage, route_points=len(self.route),
                        route_index=(self.drive_status or {}).get("route_index"),
                        drive_state=(self.drive_status or {}).get("state"),
                        cv_source="gazebo", cv_frames=self.cv_frames, cv_detections=self.cv_detections,
                        cv_age_s=round(now - self.cv_last_frame, 1) if self.cv_last_frame else None,
                    ))
                    next_status = now + 1.0
                time.sleep(0.05)
        except KeyboardInterrupt:
            self.state, self.stage = "HOLD", "STOPPED"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    BoatAgent(args.config).loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

