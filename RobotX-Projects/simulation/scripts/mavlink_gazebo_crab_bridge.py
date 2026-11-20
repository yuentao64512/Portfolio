#!/usr/bin/env python3
"""Drive Gazebo thrusters from Pixhawk MAVLink servo-output feedback.

Data path:

    MAV_CMD_DO_SET_SERVO -> Pixhawk -> SERVO_OUTPUT_RAW -> Gazebo cmd_thrust

This is for the four-thruster Gazebo model. It does not use the Pixhawk's
normal port/starboard Rover mixer for crab motion; channels 2 and 4 are read
as direct/manual outputs and mapped to the bow/stern lateral joints.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node
from pymavlink import mavutil


DEFAULT_DEVICE = (
    "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_200029001351333531343838-if00"
)
NEUTRAL_PWM = 1500
MIN_PWM = 1000
MAX_PWM = 2000


def command_plan(args) -> tuple[float, float, int, int]:
    front = max(-1.0, min(1.0, -abs(args.magnitude) * args.front_sign))
    rear = max(-1.0, min(1.0,
                           -abs(args.magnitude) * args.rear_trim * args.rear_sign))
    front_pwm = int(round(NEUTRAL_PWM + front * 500))
    rear_pwm = int(round(NEUTRAL_PWM + rear * 500))
    return front, rear, front_pwm, rear_pwm


class MavlinkGazeboBridge:
    def __init__(self, connection, args):
        self.connection = connection
        self.args = args
        self.system = connection.target_system
        self.component = connection.target_component
        self.started = time.monotonic()
        self.state = "CONNECTED"
        self.lock = threading.Lock()
        self.pose = None
        self.servo_pwm: dict[int, int] = {}
        self.node = Node()
        self.publishers = {
            name: self.node.advertise(
                f"/model/{args.model}/joint/{joint}/cmd_thrust", Double
            )
            for name, joint in {
                "port": "motor_port_joint",
                "stbd": "motor_stbd_joint",
                "front": "thruster_front_joint",
                "rear": "thruster_rear_joint",
            }.items()
        }
        self.node.subscribe(
            Odometry, f"/model/{args.model}/odometry", self._odometry_callback
        )
        self.log_file = args.log.open("w", encoding="utf-8")

    def log(self, record: str, **fields) -> None:
        item = {
            "t_unix": time.time(),
            "t_monotonic": time.monotonic() - self.started,
            "record": record,
            "state": self.state,
            **fields,
        }
        self.log_file.write(json.dumps(item, default=str,
                                       separators=(",", ":")) + "\n")
        self.log_file.flush()

    def set_state(self, state: str, reason: str) -> None:
        self.state = state
        self.log("state", reason=reason)
        print(f"state={state}: {reason}")

    def _odometry_callback(self, message: Odometry) -> None:
        p = message.pose.position
        q = message.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self.lock:
            self.pose = {"x": p.x, "y": p.y, "z": p.z, "yaw": yaw}

    def pose_snapshot(self):
        with self.lock:
            return None if self.pose is None else dict(self.pose)

    def wait_for_gazebo(self, timeout: float = 10.0) -> bool:
        self.set_state("WAITING_GAZEBO", "waiting for Gazebo odometry")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.pose_snapshot() is not None:
                self.log("gazebo_ready", pose=self.pose_snapshot())
                return True
            time.sleep(0.1)
        return False

    def get_param(self, name: str, timeout: float = 3.0):
        self.connection.mav.param_request_read_send(
            self.system, self.component, name.encode("ascii"), -1
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            message = self.connection.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.25
            )
            if message is None:
                continue
            value = message.param_id
            if isinstance(value, bytes):
                value = value.decode("ascii", errors="ignore")
            if value.rstrip("\x00") == name:
                return float(message.param_value)
        return None

    def check_output_map(self) -> bool:
        self.set_state("CHECKING_OUTPUT_MAP", "checking direct lateral outputs")
        ok = True
        for channel in (self.args.front_channel, self.args.rear_channel):
            function = self.get_param(f"SERVO{channel}_FUNCTION")
            self.log("output_map", channel=channel, function=function)
            if function is None or int(function) != 0:
                print(
                    f"SERVO{channel}_FUNCTION={function}; expected 0. Refusing.",
                    file=sys.stderr,
                )
                ok = False
        return ok

    def drain_mavlink(self) -> None:
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            message_type = message.get_type()
            if message_type == "SERVO_OUTPUT_RAW":
                outputs = {}
                for channel in range(1, 17):
                    value = getattr(message, f"servo{channel}_raw", 0)
                    outputs[channel] = int(value)
                with self.lock:
                    self.servo_pwm.update(outputs)
                self.log("mavlink", message=message_type, data=message.to_dict())
            elif message_type in {"HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"}:
                self.log("mavlink", message=message_type, data=message.to_dict())

    def normalized(self, channel: int) -> float:
        with self.lock:
            pwm = self.servo_pwm.get(channel, 0)
        if pwm <= 0:
            return 0.0
        return max(-1.0, min(1.0, (pwm - NEUTRAL_PWM) / 500.0))

    def publish_from_fcu_outputs(self) -> dict[str, float]:
        # The two propulsion channels are held neutral for this crab test.
        values = {
            "port": 0.0,
            "stbd": 0.0,
            "front": self.normalized(self.args.front_channel) * self.args.thrust_scale,
            "rear": self.normalized(self.args.rear_channel) * self.args.thrust_scale,
        }
        for name, value in values.items():
            self.publishers[name].publish(Double(data=float(value)))
        self.log("gazebo_command", **values)
        return values

    def send_servo(self, channel: int, pwm: int) -> None:
        pwm = max(MIN_PWM, min(MAX_PWM, int(pwm)))
        self.connection.mav.command_long_send(
            self.system, self.component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            float(channel), float(pwm), 0, 0, 0, 0, 0,
        )
        self.log("command", command="MAV_CMD_DO_SET_SERVO",
                 channel=channel, pwm=pwm)

    def send_pair(self, front_pwm: int, rear_pwm: int) -> None:
        self.send_servo(self.args.front_channel, front_pwm)
        self.send_servo(self.args.rear_channel, rear_pwm)

    def neutralize(self) -> None:
        self.set_state("STOPPING", "sending neutral PWM and zero Gazebo thrust")
        for _ in range(4):
            self.send_pair(NEUTRAL_PWM, NEUTRAL_PWM)
            self.drain_mavlink()
            self.publishers["port"].publish(Double(data=0.0))
            self.publishers["stbd"].publish(Double(data=0.0))
            self.publishers["front"].publish(Double(data=0.0))
            self.publishers["rear"].publish(Double(data=0.0))
            time.sleep(0.1)

    def run(self) -> int:
        if not self.wait_for_gazebo():
            self.set_state("FAULT", "Gazebo odometry was not received")
            return 5
        if not self.check_output_map():
            self.set_state("FAULT", "front/rear outputs are not direct outputs")
            return 3

        front, rear, front_pwm, rear_pwm = command_plan(self.args)
        self.log("command_plan", direction="left", front_normalized=front,
                 rear_normalized=rear, front_pwm=front_pwm, rear_pwm=rear_pwm,
                 rear_trim=self.args.rear_trim,
                 thrust_scale=self.args.thrust_scale)
        print(f"FCU->Gazebo plan: front={front_pwm}us rear={rear_pwm}us "
              f"rear_trim={self.args.rear_trim:.2f}")

        if not self.args.send:
            self.set_state("DRY_RUN", "no MAVLink commands or Gazebo thrust sent")
            return 0

        self.set_state("COMMANDING", "FCU servo commands drive Gazebo outputs")
        start_pose = self.pose_snapshot()
        end = time.monotonic() + self.args.duration
        try:
            while time.monotonic() < end:
                self.send_pair(front_pwm, rear_pwm)
                self.drain_mavlink()
                self.publish_from_fcu_outputs()
                time.sleep(1.0 / self.args.rate)
        finally:
            self.neutralize()
        self.log("motion_result", start_pose=start_pose,
                 end_pose=self.pose_snapshot())
        self.set_state("COMPLETE", "interval ended and Gazebo thrust was zeroed")
        return 0

    def close(self) -> None:
        self.log_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--model", default="usv_four_thruster")
    parser.add_argument("--front-channel", type=int, default=2)
    parser.add_argument("--rear-channel", type=int, default=4)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--magnitude", type=float, default=0.20)
    parser.add_argument("--rear-trim", type=float, default=1.4)
    parser.add_argument("--front-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--rear-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--thrust-scale", type=float, default=35.0,
                        help="full-scale PWM to Gazebo cmd_thrust; default 35")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--log", type=Path, default=Path("mavlink_gazebo_crab.jsonl"))
    parser.add_argument("--send", action="store_true",
                        help="send FCU commands and drive Gazebo; default dry-run")
    args = parser.parse_args()

    if not 0 < args.magnitude <= 1:
        parser.error("--magnitude must be greater than 0 and at most 1")
    if args.duration <= 0 or args.rate <= 0:
        parser.error("--duration and --rate must be positive")
    if args.front_channel == args.rear_channel:
        parser.error("front and rear channels must be different")

    args.log.parent.mkdir(parents=True, exist_ok=True)
    print(f"Opening {args.device}")
    try:
        connection = mavutil.mavlink_connection(
            args.device, baud=args.baud, autoreconnect=True, source_system=250
        )
        heartbeat = connection.wait_heartbeat(timeout=10)
    except Exception as exc:
        print(f"Could not receive FCU heartbeat: {exc}", file=sys.stderr)
        return 2

    print(f"Heartbeat: system={connection.target_system} "
          f"component={connection.target_component} "
          f"type={heartbeat.type} mode={mavutil.mode_string_v10(heartbeat)}")
    bridge = MavlinkGazeboBridge(connection, args)
    bridge.log("logger_start", direction="left", send=args.send,
               rear_trim=args.rear_trim)
    try:
        return bridge.run()
    except KeyboardInterrupt:
        bridge.set_state("STOPPING", "operator interrupted")
        if args.send:
            bridge.neutralize()
        return 130
    finally:
        bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
