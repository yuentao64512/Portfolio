#!/usr/bin/env python3
"""Listen to Pixhawk RC/mixer output and mirror all four movements to Gazebo.

The receiver drives the Pixhawk; this script does not inject RC overrides. It
listens for SERVO_OUTPUT_RAW and maps SERVO1..4 to the four-thruster Gazebo
model. Use --send to enable Gazebo publishing; without it the script only
prints/logs what it hears.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node
from pymavlink import mavutil


DEFAULT_DEVICE = "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_200029001351333531343838-if00"
NEUTRAL = 1500
THRUST_SCALE = 35.0


def norm(pwm: int, trim: int = NEUTRAL) -> float:
    if not pwm:
        return 0.0
    return max(-1.0, min(1.0, (pwm - trim) / 500.0))


class Bridge:
    def __init__(self, link, args):
        self.link, self.args = link, args
        self.system, self.component = link.target_system, link.target_component
        self.last_servo = {}
        self.last_servo_time = 0.0
        self.last_print = 0.0
        self.pose = None
        self.node = Node()
        self.pub = {
            name: self.node.advertise(f"/model/{args.model}/joint/{joint}/cmd_thrust", Double)
            for name, joint in {
                "front": "thruster_front_joint",
                "port": "motor_port_joint",
                "rear": "thruster_rear_joint",
                "stbd": "motor_stbd_joint",
            }.items()
        }
        self.node.subscribe(Odometry, f"/model/{args.model}/odometry", self._odom)

    def _odom(self, msg):
        self.pose = msg.pose.position

    def zero(self):
        for p in self.pub.values():
            p.publish(Double(data=0.0))

    def drain(self):
        while True:
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            typ = msg.get_type()
            if typ == "SERVO_OUTPUT_RAW":
                self.last_servo = {
                    i: int(getattr(msg, f"servo{i}_raw", 0) or 0)
                    for i in range(1, 5)
                }
                self.last_servo_time = time.monotonic()
            elif typ == "RC_CHANNELS" and time.monotonic() - self.last_print > 0.5:
                channels = [int(getattr(msg, f"chan{i}_raw", 0) or 0) for i in range(1, 9)]
                print(f"RC1..8={channels}", flush=True)
                self.last_print = time.monotonic()

    def publish(self):
        if time.monotonic() - self.last_servo_time > self.args.timeout:
            self.zero()
            return
        # Native OmniPlus mapping: servo1 front, servo2 port, servo3 rear,
        # servo4 starboard. The Gazebo joints use the same sign convention.
        values = {
            "front": norm(self.last_servo.get(1, 0)) * self.args.thrust_scale,
            "port": norm(self.last_servo.get(2, 0)) * self.args.thrust_scale,
            "rear": norm(self.last_servo.get(3, 0)) * self.args.thrust_scale * self.args.rear_trim,
            "stbd": norm(self.last_servo.get(4, 0)) * self.args.thrust_scale,
        }
        if self.args.send:
            for name, value in values.items():
                self.pub[name].publish(Double(data=float(value)))
        if time.monotonic() - self.last_print > 0.5:
            print(f"SERVO={self.last_servo} GAZEBO={values} send={self.args.send}", flush=True)
            self.last_print = time.monotonic()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--device", default=DEFAULT_DEVICE)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--model", default="usv_four_thruster")
    p.add_argument("--send", action="store_true", help="publish thruster commands to Gazebo")
    p.add_argument("--duration", type=float, default=0.0, help="seconds; 0 runs until Ctrl-C")
    p.add_argument("--rate", type=float, default=30.0)
    p.add_argument("--timeout", type=float, default=0.5, help="zero Gazebo if servo telemetry is stale")
    p.add_argument("--thrust-scale", type=float, default=THRUST_SCALE)
    p.add_argument("--rear-trim", type=float, default=1.0)
    args = p.parse_args()
    print(f"Opening Pixhawk: {args.device}")
    link = mavutil.mavlink_connection(args.device, baud=args.baud, autoreconnect=True, source_system=250)
    heartbeat = link.wait_heartbeat(timeout=10)
    print(f"Heartbeat system={link.target_system} component={link.target_component} type={heartbeat.type}")
    link.mav.command_long_send(
        link.target_system, link.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
        int(1_000_000 / args.rate), 0, 0, 0, 0, 0,
    )
    bridge = Bridge(link, args)
    if not args.send:
        print("DRY RUN: RC and servo telemetry will be displayed; Gazebo remains zeroed.")
    else:
        print("LIVE: Pixhawk servo outputs are being sent to Gazebo. Ctrl-C stops and zeros thrust.")
    end = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while end is None or time.monotonic() < end:
            bridge.drain()
            bridge.publish()
            time.sleep(1.0 / args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.zero()
        print("Stopped; Gazebo thrust zeroed.")


if __name__ == "__main__":
    main()
