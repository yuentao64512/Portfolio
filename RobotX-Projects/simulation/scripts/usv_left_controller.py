#!/usr/bin/env python3
"""Move the four-thruster USV left without an intentional yaw command."""

from __future__ import annotations

import argparse
import math
import threading
import time

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node


class LeftCrabController:
    def __init__(self, model: str):
        self.node = Node()
        self.model = model
        self.lock = threading.Lock()
        self.pose = None

        self.publishers = {}
        for joint in ("motor_port_joint", "motor_stbd_joint",
                      "thruster_front_joint", "thruster_rear_joint"):
            topic = f"/model/{model}/joint/{joint}/cmd_thrust"
            self.publishers[joint] = self.node.advertise(topic, Double)

        self.node.subscribe(
            Odometry,
            f"/model/{model}/odometry",
            self._odometry_callback,
        )

    def _odometry_callback(self, msg: Odometry):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self.lock:
            self.pose = (p.x, p.y, p.z, yaw)

    def send(self, front_command: float, rear_command: float):
        # The model's positive lateral direction is -Y. With the boat facing
        # +X, left is +Y, so both commands are negative. The SDF gives the
        # front and rear lateral thrusters identical orientations, therefore
        # equal signs produce the same global leftward force.
        values = {
            "motor_port_joint": 0.0,
            "motor_stbd_joint": 0.0,
            "thruster_front_joint": front_command,
            "thruster_rear_joint": rear_command,
        }
        for joint, value in values.items():
            msg = Double(data=float(value))
            self.publishers[joint].publish(msg)

    def stop(self):
        self.send(0.0, 0.0)

    def report(self, label: str):
        with self.lock:
            pose = self.pose
        if pose is None:
            print(f"{label}: odometry not received")
            return
        x, y, z, yaw = pose
        print(f"{label}: x={x:+.3f} y={y:+.3f} z={z:+.3f} "
              f"yaw={math.degrees(yaw):+.2f} deg")

    def run(self, duration: float, magnitude: float, rear_gain: float):
        self.report("start")
        print(f"Crabbing left for {duration:.1f}s; yaw command = 0; "
              f"rear trim = {rear_gain:.2f}")
        end = time.monotonic() + duration
        while time.monotonic() < end:
            left_command = -abs(magnitude)
            self.send(left_command, left_command * rear_gain)
            time.sleep(0.05)
        self.stop()
        time.sleep(0.25)
        self.report("stopped")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="usv_four_thruster")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--magnitude", type=float, default=7.0,
                        help="Lateral thrust magnitude; default: 7")
    parser.add_argument("--rear-gain", type=float, default=1.15,
                        help="Rear/front thrust ratio; default: 1.15")
    args = parser.parse_args()

    controller = LeftCrabController(args.model)
    try:
        controller.run(args.duration, args.magnitude, args.rear_gain)
    except KeyboardInterrupt:
        controller.stop()


if __name__ == "__main__":
    main()
