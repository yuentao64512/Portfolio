#!/usr/bin/env python3
"""Simulation-only Gazebo four-thruster follower for RoboCommand route points."""

from __future__ import annotations

import argparse
import math
import threading
import time

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node

from . import topics
from .bus import JsonBus


JOINTS = (
    "motor_port_joint", "motor_stbd_joint",
    "thruster_front_joint", "thruster_rear_joint",
)


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def body_error(x: float, y: float, yaw: float, target_x: float, target_y: float) -> tuple[float, float]:
    dx, dy = target_x - x, target_y - y
    return math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy


class SimBoatDriver:
    def __init__(self, model: str = "usv_four_thruster") -> None:
        self.node = Node()
        self.bus = JsonBus()
        self.lock = threading.Lock()
        self.model = model
        self.publishers = {
            joint: self.node.advertise(f"/model/{model}/joint/{joint}/cmd_thrust", Double)
            for joint in JOINTS
        }
        self.pose: tuple[float, float, float, float] | None = None
        self.velocity = (0.0, 0.0)
        self.pose_time = 0.0
        self.run_id = ""
        self.tier = ""
        self.route: list[dict] = []
        self.index = 0
        self.active = False
        self.stopped = False
        self.last_status = 0.0
        self.last_log = 0.0
        self.last_route_time = 0.0
        self._odom_callback_ref = self._odom
        if not self.node.subscribe(Odometry, f"/model/{model}/odometry", self._odom_callback_ref):
            raise RuntimeError("Gazebo boat odometry subscription failed")
        self.bus.subscribe(topics.TEAM_GO, self._go)
        self.bus.subscribe(topics.USV_ROUTE, self._route)
        self.bus.subscribe(topics.ROUTE_CHANGE, self._change)
        self.bus.subscribe(topics.CONTROL_STOP, self._stop)

    def _odom(self, msg: Odometry, *_: object) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        now = time.monotonic()
        with self.lock:
            if self.pose is not None and self.pose_time:
                dt = now - self.pose_time
                if 0.02 <= dt <= 0.5:
                    measured_x = (p.x - self.pose[0]) / dt
                    measured_y = (p.y - self.pose[1]) / dt
                    self.velocity = (0.7 * self.velocity[0] + 0.3 * measured_x,
                                     0.7 * self.velocity[1] + 0.3 * measured_y)
            self.pose = p.x, p.y, p.z, yaw
            self.pose_time = now

    def _go(self, msg: dict) -> None:
        tier = str(msg.get("tier", "")).lower()
        if tier not in {"advanced", "disruptive"}:
            return
        with self.lock:
            self.run_id = str(msg.get("run_id", ""))
            self.tier = tier
            self.route = []
            self.index = 0
            self.active = False
            self.stopped = False
        self.zero()
        print(f"SIM_DRIVER waiting for {tier} route", flush=True)

    def _route(self, msg: dict) -> None:
        points = msg.get("route")
        with self.lock:
            if self.stopped or not self.run_id or msg.get("run_id") != self.run_id or not isinstance(points, list) or not points:
                return
            if not all(isinstance(p, dict) and math.isfinite(float(p["x"])) and math.isfinite(float(p["y"])) for p in points):
                return
            if self.active and self.route == points:
                self.last_route_time = time.monotonic()
                return
            if self.route == points and self.index >= len(self.route):
                return
            self.route = points
            self.index = 0
            self.active = True
            self.last_route_time = time.monotonic()
        print(f"SIM_DRIVER accepted {len(points)} waypoints run_id={self.run_id}", flush=True)

    def _change(self, _msg: dict) -> None:
        if self.tier == "disruptive":
            with self.lock:
                self.route = []
                self.active = False
                self.index = 0
            self.zero()
            print("SIM_DRIVER route invalidated; thrust zero", flush=True)

    def _stop(self, _msg: dict) -> None:
        with self.lock:
            self.route = []
            self.active = False
            self.stopped = True
        self.zero()
        print("SIM_DRIVER HOLD; thrust zero", flush=True)

    def send(self, surge: float = 0.0, sway: float = 0.0) -> None:
        values = (surge, surge, sway, sway)
        for joint, value in zip(JOINTS, values):
            self.publishers[joint].publish(Double(data=float(value)))

    def zero(self) -> None:
        self.send()

    def step(self) -> dict:
        with self.lock:
            pose = self.pose
            velocity = self.velocity
            age = time.monotonic() - self.pose_time
            run_id, tier = self.run_id, self.tier
            active, index, route = self.active, self.index, self.route
            route_age = time.monotonic() - self.last_route_time
        state = "WAIT_ROUTE"
        distance = None
        if pose is None or age > 0.7:
            state = "NO_ODOM"
            self.zero()
        elif active and route_age > 3.0:
            state = "STALE_ROUTE"
            self.zero()
            with self.lock:
                self.active = False
        elif not active or index >= len(route):
            state = "DONE" if route and index >= len(route) else "WAIT_ROUTE"
            self.zero()
        else:
            x, y, z, yaw = pose
            point = route[index]
            distance = math.hypot(point["x"] - x, point["y"] - y)
            if distance < (1.5 if point["action"].startswith("GATE_") else 2.0):
                with self.lock:
                    self.index += 1
                    index = self.index
                self.zero()
                if index >= len(route):
                    state = "DONE"
                    with self.lock:
                        self.active = False
                else:
                    state = "MOVING"
            else:
                forward, left = body_error(x, y, yaw, point["x"], point["y"])
                velocity_forward = math.cos(yaw) * velocity[0] + math.sin(yaw) * velocity[1]
                velocity_left = -math.sin(yaw) * velocity[0] + math.cos(yaw) * velocity[1]
                # SDF lateral thrusters: positive command moves body-right.
                surge = clamp(3.0 * forward - 9.0 * velocity_forward, 14.0)
                sway = clamp(-3.0 * left + 9.0 * velocity_left, 10.0)
                self.send(surge, sway)
                state = "MOVING"
        position = {"x": round(pose[0], 2), "y": round(pose[1], 2), "z": round(pose[2], 2)} if pose else {}
        return {"run_id": run_id, "tier": tier, "state": state, "route_index": index,
                "route_total": len(route), "distance_m": round(distance, 2) if distance is not None else None,
                "position": position}

    def loop(self) -> None:
        print(f"SIM_DRIVER Gazebo-only model={self.model}; no Pixhawk connection", flush=True)
        try:
            while True:
                status = self.step()
                now = time.monotonic()
                if now - self.last_status >= 0.5:
                    self.bus.publish(topics.USV_DRIVE_STATUS, self.bus.envelope("sim_boat_driver", **status))
                    self.last_status = now
                if status["state"] == "MOVING" and now - self.last_log >= 2.0:
                    print(f"SIM_DRIVER waypoint={status['route_index']+1}/{status['route_total']} position={status['position']} distance={status['distance_m']}m", flush=True)
                    self.last_log = now
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            for _ in range(5):
                self.zero()
                time.sleep(0.05)
            print("SIM_DRIVER stopped; all four thrust commands zero", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="usv_four_thruster")
    args = parser.parse_args()
    SimBoatDriver(args.model).loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
