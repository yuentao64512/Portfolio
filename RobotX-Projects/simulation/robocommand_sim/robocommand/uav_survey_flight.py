#!/usr/bin/env python3
"""ArduCopter SITL lawnmower survey, triggered by RoboCommand team/go."""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Any

import yaml
from pymavlink import mavutil

from . import topics
from .bus import JsonBus


class SurveyFlight:
    def __init__(self, config_path: Path, connection: str, hover_s: float) -> None:
        self.cfg = yaml.safe_load(config_path.read_text())
        self.connection_string = connection
        self.hover_s = hover_s
        self.bus = JsonBus()
        self.go_event = threading.Event()
        self.stop_event = threading.Event()
        self.rescan_event = threading.Event()
        self.tier = "core"
        self.bus.subscribe(topics.TEAM_GO, self._go)
        self.bus.subscribe(topics.CONTROL_STOP, lambda _m: self.stop_event.set())
        self.bus.subscribe(topics.ROUTE_CHANGE, lambda _m: self.rescan_event.set())

    def _go(self, msg: dict[str, Any]) -> None:
        self.tier = str(msg.get("tier", "core")).lower()
        if self.tier in {"advanced", "disruptive"}:
            self.go_event.set()

    @staticmethod
    def set_mode(link: Any, mode: str) -> None:
        mapping = link.mode_mapping() or {}
        if mode not in mapping:
            raise RuntimeError(f"Copter did not advertise {mode}")
        link.mav.set_mode_send(link.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mapping[mode])
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb is not None and mavutil.mode_string_v10(hb) == mode:
                return
        raise RuntimeError(f"Copter did not enter {mode}")

    @staticmethod
    def arm(link: Any) -> None:
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0, 0, 0, 0, 0, 0, 0,
        )
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline:
            hb = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if hb and hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                return
        raise RuntimeError("Copter did not arm")

    @staticmethod
    def takeoff(link: Any, altitude: float) -> None:
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude,
        )
        deadline = time.monotonic() + 35.0
        while time.monotonic() < deadline:
            pos = link.recv_match(type=["LOCAL_POSITION_NED", "GLOBAL_POSITION_INT"], blocking=True, timeout=0.5)
            height = None
            if pos and pos.get_type() == "LOCAL_POSITION_NED":
                height = -float(pos.z)
            elif pos and pos.get_type() == "GLOBAL_POSITION_INT":
                height = float(pos.relative_alt) / 1000.0
            if height is not None and height >= altitude - 1.0:
                return
        raise RuntimeError("takeoff altitude not reached")

    @staticmethod
    def send_local_position(link: Any, north: float, east: float, down: float) -> None:
        # Use position only; ignore velocity, acceleration, yaw and yaw-rate.
        mask = 0x0DF8
        link.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask,
            north, east, down,
            0, 0, 0, 0, 0, 0, 0, 0,
        )

    def fly_to(self, link: Any, north: float, east: float, down: float) -> None:
        deadline = time.monotonic() + 55.0
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                self.set_mode(link, "LOITER")
                raise RuntimeError("operator stop -> LOITER")
            self.send_local_position(link, north, east, down)
            pos = link.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.2)
            if pos and math.sqrt((pos.x - north) ** 2 + (pos.y - east) ** 2 + (pos.z - down) ** 2) < 1.2:
                end = time.monotonic() + self.hover_s
                while time.monotonic() < end:
                    if self.stop_event.is_set():
                        self.set_mode(link, "LOITER")
                        raise RuntimeError("operator stop -> LOITER")
                    self.send_local_position(link, north, east, down)
                    time.sleep(0.1)
                return
        raise RuntimeError(f"waypoint timeout north={north} east={east}")

    def fly_survey(self, link: Any, altitude: float) -> None:
        # Verified on this Harmonic plugin: local-NED north == Gazebo world y,
        # local-NED east == Gazebo world x, and the origin is the world origin.
        waypoints = self.cfg["survey_waypoints_xy"]
        for index, (world_x, world_y) in enumerate(waypoints, 1):
            print(f"survey waypoint {index}/{len(waypoints)} world=({world_x},{world_y})", flush=True)
            self.fly_to(link, world_y, world_x, -altitude)

    def loop(self) -> None:
        print("UAV flight agent waiting for advanced/disruptive team/go")
        while not self.go_event.wait(0.25):
            if self.stop_event.is_set():
                return
        altitude = float(self.cfg["camera"]["survey_altitude_m"])
        link = mavutil.mavlink_connection(self.connection_string, source_system=252)
        if link.wait_heartbeat(timeout=20) is None:
            raise RuntimeError("no ArduCopter heartbeat")
        print(f"connected to Copter system={link.target_system}")
        link.mav.request_data_stream_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1,
        )
        for message_id in (32, 33):  # LOCAL_POSITION_NED, GLOBAL_POSITION_INT
            link.mav.command_long_send(
                link.target_system, link.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                message_id, 100_000, 0, 0, 0, 0, 0,
            )
        self.set_mode(link, "GUIDED")
        self.arm(link)
        self.takeoff(link, altitude)
        try:
            self.fly_survey(link, altitude)
            print("initial survey complete; holding overhead")
            if self.tier == "disruptive":
                while not self.stop_event.is_set():
                    if self.rescan_event.wait(0.25):
                        self.rescan_event.clear()
                        print("route changed; starting disruptive rescan")
                        self.fly_survey(link, altitude)
            else:
                while not self.stop_event.is_set():
                    time.sleep(0.25)
            self.set_mode(link, "LOITER")
        finally:
            link.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760")
    parser.add_argument("--hover-s", type=float, default=3.0)
    args = parser.parse_args()
    SurveyFlight(args.config, args.connection, args.hover_s).loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

