#!/usr/bin/env python3
"""Relay OmniCustomGuided commands through a Pixhawk into Gazebo.

Control path:

  normalized CLI command -> MAVLink RC_CHANNELS_OVERRIDE -> Pixhawk mode 17
  -> SERVO_OUTPUT_RAW -> Gazebo four-thruster cmd_thrust topics

This targets the locally extracted custom Rover firmware where mode 17 is
``OmniCustomGuided`` (``OCGD``).  It is a bench/simulation bridge, not a
hardware-in-the-loop electrical isolator: arming the Pixhawk can also drive
real MAIN/AUX outputs.  The default is therefore a non-actuating dry run.
Use ``--send --arm`` only after disconnecting propellers/ESC power.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry
from gz.transport13 import Node
from pymavlink import mavutil


def install_pymavlink_instance_cache_guard() -> None:
    """Repair missing indexed-message cache state in this pymavlink build."""
    original = mavutil.add_message
    if getattr(original, "_crabwalk_guarded", False):
        return

    def guarded_add_message(messages, message_type, message):
        existing = messages.get(message_type)
        if (
            getattr(message, "_instance_field", None) is not None
            and existing is not None
            and getattr(existing, "_instances", None) is None
        ):
            existing._instances = {}
        return original(messages, message_type, message)

    guarded_add_message._crabwalk_guarded = True
    mavutil.add_message = guarded_add_message


DEFAULT_DEVICE = (
    "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_200029001351333531343838-if00"
)
CUSTOM_MODE = 17
CUSTOM_MODE_NAME = "OmniCustomGuided"
IGNORE_OVERRIDE = 65535
RELEASE_OVERRIDE = 0
EXPECTED_FUNCTIONS = {1: 33, 2: 34, 3: 35, 4: 36}
OUTPUT_TO_GAZEBO = {
    1: ("port", +1.0),
    2: ("front", +1.0),
    # Custom firmware emits -forward for motor 3 because the physical
    # starboard propulsion installation is counter-oriented.  Gazebo's port
    # and starboard cmd_thrust convention expects equal signs for surge.
    3: ("stbd", -1.0),
    4: ("rear", +1.0),
}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class Calibration:
    minimum: int
    trim: int
    maximum: int

    def pwm(self, normalized: float) -> int:
        value = clamp(normalized, -1.0, 1.0)
        span = self.maximum - self.trim if value >= 0 else self.trim - self.minimum
        return int(round(self.trim + value * span))

    def normalized(self, pwm: int) -> float:
        if pwm <= 0:
            return 0.0
        span = self.maximum - self.trim if pwm >= self.trim else self.trim - self.minimum
        if span <= 0:
            return 0.0
        return clamp((pwm - self.trim) / span, -1.0, 1.0)


class CrabwalkGuidedBridge:
    def __init__(self, connection, heartbeat, args):
        self.connection = connection
        self.args = args
        self.system = connection.target_system
        self.component = connection.target_component
        self.heartbeat = heartbeat
        self.initial_mode = int(heartbeat.custom_mode)
        self.mode_changed = False
        self.armed_by_script = False
        self.override_active = False
        self.cleaned_up = False
        self.input_channels = None
        self.input_calibration = None
        self.started = time.monotonic()
        self.servo_pwm: dict[int, int] = {}
        self.servo_received_at = 0.0
        self.pose = None
        self.lock = threading.Lock()
        self.node = Node()
        joints = {
            "port": "motor_port_joint",
            "stbd": "motor_stbd_joint",
            "front": "thruster_front_joint",
            "rear": "thruster_rear_joint",
        }
        self.publishers = {
            name: self.node.advertise(
                f"/model/{args.model}/joint/{joint}/cmd_thrust", Double
            )
            for name, joint in joints.items()
        }
        self.node.subscribe(
            Odometry, f"/model/{args.model}/odometry", self._odometry_callback
        )
        self.log_file = args.log.open("w", encoding="utf-8")

    def _odometry_callback(self, message: Odometry) -> None:
        position = message.pose.position
        orientation = message.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
        )
        with self.lock:
            self.pose = {
                "x": position.x,
                "y": position.y,
                "z": position.z,
                "yaw": yaw,
            }

    def log(self, record: str, **fields) -> None:
        item = {
            "time": time.time(),
            "elapsed": time.monotonic() - self.started,
            "record": record,
            **fields,
        }
        self.log_file.write(json.dumps(item, default=str, separators=(",", ":")) + "\n")
        self.log_file.flush()

    def pose_snapshot(self):
        with self.lock:
            return None if self.pose is None else dict(self.pose)

    def get_param(self, name: str, timeout: float = 2.0):
        self.connection.mav.param_request_read_send(
            self.system, self.component, name.encode("ascii"), -1
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.2
            )
            if message is None:
                continue
            param_id = message.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="ignore")
            if param_id.rstrip("\x00") == name:
                return float(message.param_value)
        return None

    def require_int_param(self, name: str) -> int:
        value = self.get_param(name)
        if value is None:
            raise RuntimeError(f"Pixhawk did not return required parameter {name}")
        return int(round(value))

    def set_param(self, name: str, value: int, timeout: float = 3.0) -> None:
        self.connection.mav.param_set_send(
            self.system,
            self.component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.connection.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.2
            )
            if message is None:
                continue
            param_id = message.param_id
            if isinstance(param_id, bytes):
                param_id = param_id.decode("ascii", errors="ignore")
            if param_id.rstrip("\x00") != name:
                continue
            actual = int(round(float(message.param_value)))
            if actual != value:
                raise RuntimeError(f"{name} was acknowledged as {actual}, expected {value}")
            self.log("parameter_set", name=name, value=value)
            print(f"Set {name}={value}")
            return
        raise RuntimeError(f"Pixhawk did not acknowledge {name}={value}")

    def configure_fcu(self) -> int:
        if self.is_armed():
            raise RuntimeError("refusing persistent parameter changes while armed")
        required = {
            "FRAME_CLASS": 2,
            "FRAME_TYPE": 3,
            **{
                f"SERVO{channel}_FUNCTION": function
                for channel, function in EXPECTED_FUNCTIONS.items()
            },
        }
        for name, expected in required.items():
            actual = self.require_int_param(name)
            if actual != expected:
                self.set_param(name, expected)
            else:
                print(f"Keep {name}={expected}")
        print("FCU configuration saved. Reboot/power-cycle the Pixhawk, then rerun.")
        return 0

    def load_configuration(self):
        frame_class = self.require_int_param("FRAME_CLASS")
        frame_type = self.require_int_param("FRAME_TYPE")
        if frame_class != 2 or frame_type != 3:
            raise RuntimeError(
                f"FRAME_CLASS={frame_class}, FRAME_TYPE={frame_type}; mode 17's motor "
                "vector requires FRAME_CLASS=2 and FRAME_TYPE=3 (OmniPlus). "
                "Run './crabwalk_guided.py --configure-fcu' while disarmed, then reboot."
            )
        input_channels = {
            "steering": self.require_int_param("RCMAP_ROLL"),
            "forward": self.require_int_param("RCMAP_THROTTLE"),
            "lateral": self.require_int_param("RCMAP_YAW"),
        }
        if any(channel < 1 or channel > 8 for channel in input_channels.values()):
            raise RuntimeError(f"RC input channel outside MAVLink override range: {input_channels}")
        if len(set(input_channels.values())) != 3:
            raise RuntimeError(f"RC input channels overlap: {input_channels}")

        input_calibration = {}
        for axis, channel in input_channels.items():
            input_calibration[axis] = Calibration(
                self.require_int_param(f"RC{channel}_MIN"),
                self.require_int_param(f"RC{channel}_TRIM"),
                self.require_int_param(f"RC{channel}_MAX"),
            )

        output_calibration = {}
        for channel, expected_function in EXPECTED_FUNCTIONS.items():
            actual = self.require_int_param(f"SERVO{channel}_FUNCTION")
            if actual != expected_function:
                raise RuntimeError(
                    f"SERVO{channel}_FUNCTION={actual}, expected {expected_function}; "
                    "this is not the validated custom four-thruster mapping"
                )
            output_calibration[channel] = Calibration(
                self.require_int_param(f"SERVO{channel}_MIN"),
                self.require_int_param(f"SERVO{channel}_TRIM"),
                self.require_int_param(f"SERVO{channel}_MAX"),
            )

        gcs_system = self.get_param("MAV_GCS_SYSID")
        if gcs_system is not None and int(round(gcs_system)) != self.args.source_system:
            raise RuntimeError(
                f"MAV_GCS_SYSID={int(round(gcs_system))}, but this script uses "
                f"source system {self.args.source_system}; RC override would be ignored"
            )
        self.log(
            "configuration",
            input_channels=input_channels,
            input_calibration=input_calibration,
            output_calibration=output_calibration,
            output_functions=EXPECTED_FUNCTIONS,
        )
        return input_channels, input_calibration, output_calibration

    def wait_for_gazebo(self, timeout: float = 8.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.pose_snapshot() is not None:
                return True
            time.sleep(0.1)
        return False

    def drain_mavlink(self) -> None:
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            message_type = message.get_type()
            if message_type == "HEARTBEAT":
                self.heartbeat = message
            elif message_type == "SERVO_OUTPUT_RAW":
                values = {
                    channel: int(getattr(message, f"servo{channel}_raw", 0))
                    for channel in range(1, 5)
                }
                with self.lock:
                    self.servo_pwm.update(values)
                    self.servo_received_at = time.monotonic()
                self.log("servo_output", pwm=values)
            elif message_type in {"COMMAND_ACK", "STATUSTEXT"}:
                self.log("mavlink", message=message_type, data=message.to_dict())

    def request_servo_telemetry(self) -> None:
        interval_us = int(1_000_000 / self.args.rate)
        self.connection.mav.command_long_send(
            self.system,
            self.component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )
        self.log("request_message_interval", message_id=36, interval_us=interval_us)

    def is_armed(self) -> bool:
        return bool(
            self.heartbeat.base_mode
            & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    def wait_for_mode(self, mode: int, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.drain_mavlink()
            if int(self.heartbeat.custom_mode) == mode:
                return True
            time.sleep(0.05)
        return False

    def request_mode(self, mode: int) -> bool:
        self.connection.mav.set_mode_send(
            self.system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode,
        )
        success = self.wait_for_mode(mode)
        self.log("mode_request", mode=mode, success=success)
        if success and mode != self.initial_mode:
            self.mode_changed = True
        return success

    def arm(self) -> bool:
        self.connection.mav.command_long_send(
            self.system,
            self.component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            self.drain_mavlink()
            if self.is_armed():
                self.armed_by_script = True
                self.log("armed")
                return True
            time.sleep(0.1)
        return False

    def disarm(self) -> None:
        self.connection.mav.command_long_send(
            self.system,
            self.component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0.0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            self.drain_mavlink()
            if not self.is_armed():
                break
            time.sleep(0.1)
        self.log("disarm", success=not self.is_armed())

    def send_override(
        self,
        channels: dict[str, int],
        calibration: dict[str, Calibration],
        forward: float,
        lateral: float,
        steering: float,
    ) -> None:
        raw = [IGNORE_OVERRIDE] * 8
        commands = {
            "forward": forward,
            "lateral": lateral,
            "steering": steering,
        }
        pwm = {}
        for axis, value in commands.items():
            channel = channels[axis]
            pwm[axis] = calibration[axis].pwm(value)
            raw[channel - 1] = pwm[axis]
        self.connection.mav.rc_channels_override_send(
            self.system, self.component, *raw
        )
        self.override_active = True
        self.log("rc_override", normalized=commands, pwm=pwm)

    def release_override(self, channels: dict[str, int]) -> None:
        raw = [IGNORE_OVERRIDE] * 8
        for channel in channels.values():
            raw[channel - 1] = RELEASE_OVERRIDE
        self.connection.mav.rc_channels_override_send(
            self.system, self.component, *raw
        )
        self.override_active = False
        self.log("rc_override_release")

    def publish_outputs(self, calibration: dict[int, Calibration]):
        with self.lock:
            age = time.monotonic() - self.servo_received_at
            pwm = dict(self.servo_pwm)
        if age > self.args.telemetry_timeout:
            raise RuntimeError(
                f"SERVO_OUTPUT_RAW is stale ({age:.2f}s); Gazebo was zeroed"
            )
        thrust = {}
        for channel, (name, sign) in OUTPUT_TO_GAZEBO.items():
            value = calibration[channel].normalized(pwm.get(channel, 0))
            thrust[name] = value * sign * self.args.thrust_scale
        for name, value in thrust.items():
            self.publishers[name].publish(Double(data=float(value)))
        self.log("gazebo_thrust", **thrust)
        return thrust

    def zero_gazebo(self) -> None:
        for publisher in self.publishers.values():
            publisher.publish(Double(data=0.0))

    def neutralize(
        self,
        channels: dict[str, int],
        calibration: dict[str, Calibration],
    ) -> None:
        for _ in range(8):
            self.send_override(channels, calibration, 0.0, 0.0, 0.0)
            self.zero_gazebo()
            time.sleep(0.05)
        self.release_override(channels)
        self.zero_gazebo()

    def run(self) -> int:
        if self.args.configure_fcu:
            return self.configure_fcu()
        channels, input_calibration, output_calibration = self.load_configuration()
        self.input_channels = channels
        self.input_calibration = input_calibration
        print(f"Input map: {channels}")
        print(f"Verified output functions: {EXPECTED_FUNCTIONS}")
        print("Waiting for Gazebo odometry ...")
        if not self.wait_for_gazebo():
            raise RuntimeError(
                f"no /model/{self.args.model}/odometry; start the four-thruster world first"
            )
        print(f"Gazebo ready at {self.pose_snapshot()}")
        print(
            f"Plan: mode {CUSTOM_MODE} ({CUSTOM_MODE_NAME}), "
            f"forward={self.args.forward:+.2f}, lateral={self.args.lateral:+.2f}, "
            f"steering={self.args.steering:+.2f}, duration={self.args.duration:.1f}s"
        )

        if not self.args.send:
            print("DRY RUN: no mode change, RC override, arming, or Gazebo thrust sent.")
            print("Use --send --arm only with propellers and ESC power disconnected.")
            self.log("dry_run", pose=self.pose_snapshot())
            return 0

        initially_armed = self.is_armed()
        if initially_armed and not self.args.allow_armed:
            raise RuntimeError(
                "Pixhawk is already armed; refusing without explicit --allow-armed"
            )
        if not initially_armed and not self.args.arm:
            raise RuntimeError("--send requires an armed Pixhawk or explicit --arm")

        self.request_servo_telemetry()
        self.send_override(channels, input_calibration, 0.0, 0.0, 0.0)
        time.sleep(0.35)
        if not self.request_mode(CUSTOM_MODE):
            raise RuntimeError(f"Pixhawk did not enter custom mode {CUSTOM_MODE}")
        if self.args.arm and not initially_armed and not self.arm():
            raise RuntimeError("Pixhawk refused to arm; no movement command was sent")

        start_pose = self.pose_snapshot()
        deadline = time.monotonic() + self.args.duration
        print("Relaying Pixhawk SERVO_OUTPUT_RAW to Gazebo; Ctrl-C stops safely.")
        try:
            while time.monotonic() < deadline:
                self.send_override(
                    channels,
                    input_calibration,
                    self.args.forward,
                    self.args.lateral,
                    self.args.steering,
                )
                self.drain_mavlink()
                if int(self.heartbeat.custom_mode) != CUSTOM_MODE:
                    raise RuntimeError(
                        f"mode changed to {self.heartbeat.custom_mode}; Gazebo was zeroed"
                    )
                self.publish_outputs(output_calibration)
                time.sleep(1.0 / self.args.rate)
        finally:
            self.cleanup()

        end_pose = self.pose_snapshot()
        self.log("complete", start_pose=start_pose, end_pose=end_pose)
        print(f"Complete. Start pose: {start_pose}")
        print(f"          End pose: {end_pose}")
        return 0

    def emergency_stop(self) -> None:
        self.zero_gazebo()
        self.log("emergency_gazebo_zero")

    def cleanup(self) -> None:
        if self.cleaned_up:
            return
        self.cleaned_up = True
        self.zero_gazebo()
        if (
            self.override_active
            and self.input_channels is not None
            and self.input_calibration is not None
        ):
            self.neutralize(self.input_channels, self.input_calibration)
        if self.armed_by_script and self.args.disarm_after:
            self.disarm()
        if self.mode_changed and self.args.restore_mode and not self.is_armed():
            restored = self.request_mode(self.initial_mode)
            self.log("mode_restore", mode=self.initial_mode, success=restored)
        self.zero_gazebo()

    def close(self) -> None:
        try:
            self.cleanup()
        except Exception as exc:
            print(f"Cleanup warning: {exc}", file=sys.stderr)
            self.zero_gazebo()
        finally:
            self.log_file.close()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--source-system",
        type=int,
        default=255,
        help="MAVLink sender ID; must match MAV_GCS_SYSID",
    )
    parser.add_argument("--model", default="usv_four_thruster")
    parser.add_argument("--forward", type=float, default=0.0, help="-1..1")
    parser.add_argument(
        "--lateral",
        type=float,
        default=-0.25,
        help="-1..1; negative is left crab motion in this Gazebo model",
    )
    parser.add_argument("--steering", type=float, default=0.0, help="-1..1")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--thrust-scale", type=float, default=35.0)
    parser.add_argument("--telemetry-timeout", type=float, default=1.0)
    parser.add_argument(
        "--log", type=Path, default=Path("crabwalk_guided.jsonl")
    )
    parser.add_argument(
        "--send",
        action="store_true",
        help="enable RC commands and Gazebo output relay; default is dry-run",
    )
    parser.add_argument(
        "--configure-fcu",
        action="store_true",
        help="while disarmed, persist the OmniPlus frame and Motor1..4 output mapping",
    )
    parser.add_argument(
        "--arm",
        action="store_true",
        help="arm the physical Pixhawk (disconnect propellers/ESC power first)",
    )
    parser.add_argument(
        "--allow-armed",
        action="store_true",
        help="permit use when the Pixhawk was already armed",
    )
    parser.add_argument(
        "--disarm-after",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="disarm when this script armed the FCU (default: yes)",
    )
    parser.add_argument(
        "--restore-mode",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="restore the original mode after a disarmed run (default: yes)",
    )
    args = parser.parse_args()
    for name in ("forward", "lateral", "steering"):
        value = getattr(args, name)
        if not -1.0 <= value <= 1.0:
            parser.error(f"--{name} must be between -1 and 1")
    if args.duration <= 0 or args.rate <= 0 or args.telemetry_timeout <= 0:
        parser.error("--duration, --rate, and --telemetry-timeout must be positive")
    if args.thrust_scale <= 0:
        parser.error("--thrust-scale must be positive")
    if not 1 <= args.source_system <= 255:
        parser.error("--source-system must be between 1 and 255")
    if args.arm and not args.send:
        parser.error("--arm requires --send")
    if args.configure_fcu and (args.send or args.arm or args.allow_armed):
        parser.error("--configure-fcu cannot be combined with --send/--arm/--allow-armed")
    args.log.parent.mkdir(parents=True, exist_ok=True)
    return args


def main() -> int:
    args = parse_args()
    install_pymavlink_instance_cache_guard()
    print(f"Opening Pixhawk: {args.device}")
    try:
        connection = mavutil.mavlink_connection(
            args.device,
            baud=args.baud,
            autoreconnect=True,
            source_system=args.source_system,
        )
        heartbeat = connection.wait_heartbeat(timeout=10)
    except Exception as exc:
        print(f"Could not receive Pixhawk heartbeat: {exc}", file=sys.stderr)
        return 2

    print(
        f"Heartbeat system={connection.target_system} "
        f"component={connection.target_component} type={heartbeat.type} "
        f"custom_mode={heartbeat.custom_mode} "
        f"armed={bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}"
    )
    if heartbeat.type != mavutil.mavlink.MAV_TYPE_SURFACE_BOAT:
        print(
            f"Refusing FCU type {heartbeat.type}; expected MAV_TYPE_SURFACE_BOAT",
            file=sys.stderr,
        )
        return 3

    bridge = CrabwalkGuidedBridge(connection, heartbeat, args)
    bridge.log("start", arguments=vars(args))
    try:
        return bridge.run()
    except KeyboardInterrupt:
        print("Interrupted; zeroing Gazebo thrust.", file=sys.stderr)
        bridge.emergency_stop()
        return 130
    except Exception as exc:
        bridge.log("failure", error=repr(exc))
        bridge.emergency_stop()
        print(f"crabwalk_guided failed: {exc}", file=sys.stderr)
        return 1
    finally:
        bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
