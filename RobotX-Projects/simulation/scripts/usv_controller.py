#!/usr/bin/env python3
"""Small Gazebo Sim controller for the two USV propulsion layouts.

The command values are direct ``cmd_thrust`` values. Run with ``--demo`` for
an unattended motion check, or omit it and enter ``surge sway yaw`` commands.
"""

from __future__ import annotations

import argparse
import threading
import time

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node


class USVController:
    def __init__(self, model_name: str, four_thruster: bool):
        self.model_name = model_name
        self.four_thruster = four_thruster
        self.node = Node()
        self.lock = threading.Lock()
        self.current_pose = (0.0, 0.0, 0.0)

        joints = ["motor_port_joint", "motor_stbd_joint"]
        if four_thruster:
            joints += ["thruster_front_joint", "thruster_rear_joint"]
        self.publishers = {}
        for joint in joints:
            topic = f"/model/{model_name}/joint/{joint}/cmd_thrust"
            self.publishers[joint] = self.node.advertise(topic, Double)

        self.node.subscribe(
            Odometry,
            f"/model/{model_name}/odometry",
            self._odometry_callback,
        )

    def _odometry_callback(self, msg: Odometry):
        pose = msg.pose
        with self.lock:
            self.current_pose = (pose.position.x, pose.position.y, pose.position.z)

    def send(self, port: float = 0.0, stbd: float = 0.0,
             sway_front: float = 0.0, sway_rear: float = 0.0):
        values = {
            "motor_port_joint": port,
            "motor_stbd_joint": stbd,
        }
        if self.four_thruster:
            values.update({
                "thruster_front_joint": sway_front,
                "thruster_rear_joint": sway_rear,
            })
        for joint, value in values.items():
            msg = Double()
            msg.data = float(value)
            self.publishers[joint].publish(msg)

    def drive(self, surge: float, sway: float, yaw: float):
        # Equal stern commands produce surge; differential commands yaw.
        # Equal bow/stern lateral commands produce crab motion.
        self.send(
            port=surge - yaw,
            stbd=surge + yaw,
            sway_front=sway,
            sway_rear=sway,
        )

    def stop(self):
        self.send()

    def print_pose(self):
        with self.lock:
            x, y, z = self.current_pose
        print(f"pose x={x:+.3f} y={y:+.3f} z={z:+.3f}")

    def demo(self, seconds: float):
        phases = [
            ("forward", 0.30 * seconds, 10.0, 0.0, 0.0),
            ("reverse", 0.20 * seconds, -8.0, 0.0, 0.0),
        ]
        if self.four_thruster:
            phases.append(("crab", 0.30 * seconds, 0.0, 7.0, 0.0))
        phases.append(("yaw", 0.20 * seconds, 0.0, 0.0, 4.0))

        print(f"Starting {self.model_name} motion demo")
        start = time.monotonic()
        for name, duration, surge, sway, yaw in phases:
            print(f"  {name}: {duration:.1f}s")
            end = time.monotonic() + duration
            while time.monotonic() < end:
                self.drive(surge, sway, yaw)
                time.sleep(0.05)
        self.stop()
        elapsed = time.monotonic() - start
        time.sleep(0.25)
        print(f"Completed in {elapsed:.1f}s")
        self.print_pose()

    def interactive(self):
        print("Enter: surge sway yaw (example: 8 0 0; Ctrl-D to stop)")
        print("Values are direct thrust commands; use 0 0 0 to stop.")
        try:
            while True:
                line = input("> ").strip()
                if not line:
                    continue
                surge, sway, yaw = (float(value) for value in line.split()[:3])
                self.drive(surge, sway if self.four_thruster else 0.0, yaw)
                self.print_pose()
        except (EOFError, KeyboardInterrupt):
            pass
        finally:
            self.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", default="usv_four_thruster")
    parser.add_argument("--two-thruster", action="store_true",
                        help="Use only the port/starboard propulsion pair")
    parser.add_argument("--demo", action="store_true",
                        help="Run the unattended motion sequence")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Demo duration budget in seconds")
    args = parser.parse_args()
    controller = USVController(args.model, not args.two_thruster)
    if args.demo:
        controller.demo(args.duration)
    else:
        controller.interactive()


if __name__ == "__main__":
    main()
