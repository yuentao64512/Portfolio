#!/usr/bin/env python3
"""Drive the four-thruster Gazebo USV with ArduPilot's native OmniPlus mixer.

The control path is RC_CHANNELS_OVERRIDE -> Rover OmniPlus mixer ->
SERVO_OUTPUT_RAW -> Gazebo cmd_thrust topics.

Native OmniPlus output mapping:
  SERVO1/motor1 -> bow lateral thruster
  SERVO2/motor2 -> port stern thruster
  SERVO3/motor3 -> stern lateral thruster
  SERVO4/motor4 -> starboard stern thruster

The default path uses MANUAL mode because native Rover OmniPlus lateral input
is consumed there from the Rover yaw channel (RCMAP_YAW). The --guided path
uses SET_POSITION_TARGET_LOCAL_NED body-frame velocity targets and requires
the matching local ArduPilot Guided extension. The script is dry-run by
default and never arms unless --arm is explicitly supplied.
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
IGNORE_OVERRIDE = 65535
RELEASE_OVERRIDE = 0
MOTOR_FUNCTIONS = {1: 33, 2: 34, 3: 35, 4: 36}


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def pwm_from_normalized(value: float) -> int:
    return int(round(NEUTRAL_PWM + clamp(value, -1.0, 1.0) * 500.0))


class OmniPlusBridge:
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
        self.heartbeat = None
        self.armed_by_script = False
        self.mode_channel: int | None = None
        self.mode_pwm: int | None = None
        self.node = Node()
        self.publishers = {
            name: self.node.advertise(
                f"/model/{args.model}/joint/{joint}/cmd_thrust", Double
            )
            for name, joint in {
                "front": "thruster_front_joint",
                "port": "motor_port_joint",
                "rear": "thruster_rear_joint",
                "stbd": "motor_stbd_joint",
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
        self.log_file.write(
            json.dumps(item, default=str, separators=(",", ":")) + "\n"
        )
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

    def resolve_input_channels(self) -> dict[str, int]:
        requested = {
            "steering": self.args.steering_channel,
            "throttle": self.args.throttle_channel,
            "lateral": self.args.lateral_channel,
        }
        defaults = {"steering": 1, "throttle": 3, "lateral": 4}
        parameters = {
            "steering": "RCMAP_ROLL",
            "throttle": "RCMAP_THROTTLE",
            # Rover's native OmniPlus path binds channel_lateral to the
            # RCMapper yaw channel. RCMAP_LATERAL is not used by Rover's
            # manual mode mixer here.
            "lateral": "RCMAP_YAW",
        }
        channels = {}
        for axis, parameter in parameters.items():
            value = self.get_param(parameter)
            if requested[axis] is not None:
                channels[axis] = requested[axis]
            elif value is not None and 1 <= int(value) <= 16:
                channels[axis] = int(value)
            else:
                channels[axis] = defaults[axis]
            self.log(
                "input_map",
                axis=axis,
                parameter=parameter,
                parameter_value=value,
                channel=channels[axis],
            )
        if len(set(channels.values())) != 3:
            raise RuntimeError(f"steering/throttle/lateral channels overlap: {channels}")
        self.resolve_manual_mode_override(set(channels.values()))
        return channels

    def resolve_manual_mode_override(self, input_channels: set[int]) -> None:
        """Find a valid PWM slot for MANUAL on the configured mode channel.

        Leaving MODE_CH uncontrolled is unsafe for this bench setup: a stale
        override can select HOLD, where Rover deliberately commands neutral
        OmniPlus outputs.
        """
        mode_channel = self.get_param("MODE_CH")
        if mode_channel is None or not 1 <= int(mode_channel) <= 8:
            self.mode_channel = None
            self.mode_pwm = None
            self.log("mode_input", mode_channel=mode_channel, mode_pwm=None)
            return
        self.mode_channel = int(mode_channel)
        if self.mode_channel in input_channels:
            raise RuntimeError(
                f"MODE_CH={self.mode_channel} overlaps steering/throttle/lateral input"
            )

        mode_mapping = self.connection.mode_mapping() or {}
        manual_mode_id = mode_mapping.get("MANUAL")
        manual_slot = None
        if manual_mode_id is not None:
            for slot in range(1, 7):
                configured = self.get_param(f"MODE{slot}")
                if configured is not None and int(configured) == int(manual_mode_id):
                    manual_slot = slot
                    break

        # A 6-position switch slot has these standard PWM ranges.  Select the
        # middle of the matching slot so the mode decoder is unambiguous.
        slot_pwm = {1: 1100, 2: 1295, 3: 1425, 4: 1555, 5: 1685, 6: 1850}
        self.mode_pwm = slot_pwm.get(manual_slot or 1)
        self.log(
            "mode_input",
            mode_channel=self.mode_channel,
            manual_mode_id=manual_mode_id,
            manual_slot=manual_slot,
            mode_pwm=self.mode_pwm,
        )

    def check_omniplus_config(self) -> bool:
        self.set_state("CHECKING_OMNIPLUS_CONFIG", "checking native OmniPlus parameters")
        frame_type = self.get_param("FRAME_TYPE")
        frame_class = self.get_param("FRAME_CLASS")
        self.log("frame_config", frame_class=frame_class, frame_type=frame_type)
        ok = frame_type is not None and int(frame_type) == 3
        if frame_class is not None and int(frame_class) != 2:
            ok = False
        if not ok:
            print(
                f"Expected FRAME_CLASS=2 and FRAME_TYPE=3; got "
                f"FRAME_CLASS={frame_class}, FRAME_TYPE={frame_type}.",
                file=sys.stderr,
            )

        for channel, expected_function in MOTOR_FUNCTIONS.items():
            function = self.get_param(f"SERVO{channel}_FUNCTION")
            self.log(
                "output_map",
                channel=channel,
                function=function,
                expected_function=expected_function,
            )
            if function is None or int(function) != expected_function:
                print(
                    f"SERVO{channel}_FUNCTION={function}; expected "
                    f"{expected_function} (OmniPlus motor{channel}).",
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
            elif message_type == "HEARTBEAT":
                self.heartbeat = message
                self.log("mavlink", message=message_type, data=message.to_dict())
            elif message_type in {"COMMAND_ACK", "STATUSTEXT"}:
                self.log("mavlink", message=message_type, data=message.to_dict())

    def is_armed(self) -> bool:
        heartbeat = self.heartbeat
        return bool(
            heartbeat is not None
            and (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        )

    def wait_for_mode(self, mode: str, timeout: float = 5.0) -> bool:
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.drain_mavlink()
            if self.heartbeat is not None and mavutil.mode_string_v10(self.heartbeat) == mode:
                return True
            time.sleep(0.1)
        return False

    def request_manual(self) -> bool:
        return self.request_mode("MANUAL", "requesting MANUAL mode for native lateral input")

    def request_guided(self) -> bool:
        return self.request_mode("GUIDED", "requesting GUIDED mode for velocity setpoints")

    def request_mode(self, mode: str, reason: str) -> bool:
        self.set_state(f"REQUESTING_{mode}", reason)
        mapping = self.connection.mode_mapping() or {}
        mode_id = mapping.get(mode)
        if mode_id is None:
            raise RuntimeError(f"FCU did not advertise a {mode} mode")
        self.connection.mav.set_mode_send(
            self.system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        if not self.wait_for_mode(mode):
            return False
        self.log("mode", mode=mode)
        return True

    def arm(self) -> bool:
        self.set_state("ARMING", "arming requested by --arm")
        self.connection.mav.command_long_send(
            self.system, self.component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1.0, 0.0, 0, 0, 0, 0, 0,
        )
        end = time.monotonic() + 8.0
        while time.monotonic() < end:
            self.drain_mavlink()
            if self.is_armed():
                self.armed_by_script = True
                self.log("armed")
                return True
            time.sleep(0.1)
        return False

    def release_safety(self) -> bool:
        """Explicitly release the Pixhawk hardware safety state."""
        self.set_state("RELEASING_SAFETY", "releasing safety by explicit request")
        command = mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE
        self.connection.mav.command_long_send(
            self.system, self.component, command, 0,
            mavutil.mavlink.SAFETY_SWITCH_STATE_DANGEROUS,
            0, 0, 0, 0, 0, 0,
        )
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            message = self.connection.recv_match(
                type="COMMAND_ACK", blocking=True, timeout=0.25
            )
            if message is None or message.command != command:
                continue
            self.log("safety", command=command, result=message.result)
            return message.result in {
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
            }
        self.log("safety", command=command, result="no_ack")
        return False

    def disarm(self) -> None:
        self.connection.mav.command_long_send(
            self.system, self.component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0.0, 0.0, 0, 0, 0, 0, 0,
        )
        self.log("disarm_requested")

    def send_rc_override(self, channels: dict[str, int], values: dict[str, int]) -> None:
        raw = [IGNORE_OVERRIDE] * 8
        for axis, pwm in values.items():
            raw[channels[axis] - 1] = max(MIN_PWM, min(MAX_PWM, int(pwm)))
        if self.mode_channel is not None and self.mode_pwm is not None:
            raw[self.mode_channel - 1] = self.mode_pwm
        self.connection.mav.rc_channels_override_send(
            self.system, self.component, *raw
        )
        self.log("command", command="RC_CHANNELS_OVERRIDE", values=values)

    def prime_rc_override(self, channels: dict[str, int]) -> None:
        # Hold the configured MANUAL mode slot while priming neutral inputs.
        values = {
            "steering": NEUTRAL_PWM,
            "throttle": NEUTRAL_PWM,
            "lateral": NEUTRAL_PWM,
        }
        raw = [IGNORE_OVERRIDE] * 8
        for axis, pwm in values.items():
            raw[channels[axis] - 1] = pwm
        if self.mode_channel is not None and self.mode_pwm is not None:
            raw[self.mode_channel - 1] = self.mode_pwm
        self.connection.mav.rc_channels_override_send(
            self.system, self.component, *raw
        )
        self.log(
            "command",
            command="RC_CHANNELS_OVERRIDE_PRIME",
            values={**values, "mode_pwm": self.mode_pwm},
        )
        time.sleep(0.5)
        self.drain_mavlink()

    def release_rc_override(self, channels: dict[str, int]) -> None:
        raw = [IGNORE_OVERRIDE] * 8
        for channel in channels.values():
            raw[channel - 1] = RELEASE_OVERRIDE
        if self.mode_channel is not None:
            raw[self.mode_channel - 1] = RELEASE_OVERRIDE
        self.connection.mav.rc_channels_override_send(
            self.system, self.component, *raw
        )
        self.log("command", command="RC_CHANNELS_OVERRIDE_RELEASE")

    def send_guided_velocity(self, forward_speed: float, lateral_speed: float) -> None:
        """Send a body-frame velocity target while holding the current yaw."""
        type_mask = (
            (1 << 0) | (1 << 1) | (1 << 2) |  # ignore position x/y/z
            (1 << 5) |                         # ignore vz
            (1 << 6) | (1 << 7) | (1 << 8) |  # ignore acceleration
            (1 << 9) | (1 << 10) | (1 << 11)  # ignore force/yaw/yaw-rate
        )
        self.connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.system,
            self.component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            type_mask,
            0.0, 0.0, 0.0,
            float(forward_speed), float(lateral_speed), 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0,
        )
        self.log(
            "command",
            command="SET_POSITION_TARGET_LOCAL_NED",
            frame="BODY_NED",
            forward_speed=forward_speed,
            lateral_speed=lateral_speed,
            yaw="hold_current",
        )

    def neutralize_guided(self) -> None:
        self.set_state("STOPPING", "sending zero Guided velocity and zeroing Gazebo thrust")
        for _ in range(8):
            self.send_guided_velocity(0.0, 0.0)
            self.drain_mavlink()
            self.zero_gazebo()
            time.sleep(0.05)
        self.zero_gazebo()

    def normalized(self, channel: int) -> float:
        with self.lock:
            pwm = self.servo_pwm.get(channel, 0)
        if pwm <= 0:
            return 0.0
        return clamp((pwm - NEUTRAL_PWM) / 500.0, -1.0, 1.0)

    def publish_from_fcu_outputs(self) -> dict[str, float]:
        values = {
            "front": self.normalized(1) * self.args.thrust_scale,
            "port": self.normalized(2) * self.args.thrust_scale,
            "rear": self.normalized(3) * self.args.thrust_scale * self.args.rear_gazebo_trim,
            "stbd": self.normalized(4) * self.args.thrust_scale,
        }
        for name, value in values.items():
            self.publishers[name].publish(Double(data=float(value)))
        self.log("gazebo_command", **values)
        return values

    def zero_gazebo(self) -> None:
        for publisher in self.publishers.values():
            publisher.publish(Double(data=0.0))

    def neutralize(self, channels: dict[str, int]) -> None:
        self.set_state("STOPPING", "centering native inputs and zeroing Gazebo thrust")
        neutral = {"steering": NEUTRAL_PWM, "throttle": NEUTRAL_PWM, "lateral": NEUTRAL_PWM}
        for _ in range(8):
            self.send_rc_override(channels, neutral)
            self.drain_mavlink()
            self.zero_gazebo()
            time.sleep(0.05)
        self.release_rc_override(channels)
        self.zero_gazebo()

    def run(self) -> int:
        channels = None
        if not self.args.guided:
            channels = self.resolve_input_channels()
            self.log("resolved_input_channels", **channels)
        if not self.check_omniplus_config():
            self.set_state("FAULT", "FCU is not configured as native OmniPlus")
            return 3
        if not self.wait_for_gazebo():
            self.set_state("FAULT", "Gazebo odometry was not received")
            return 5
        requested_mode = "GUIDED" if self.args.guided else "MANUAL"
        mode_ok = self.request_guided() if self.args.guided else self.request_manual()
        if not mode_ok:
            self.set_state("FAULT", f"FCU did not enter {requested_mode} mode")
            return 6

        if self.args.send:
            if self.args.release_safety and not self.release_safety():
                self.set_state("FAULT", "Pixhawk did not release hardware safety")
                return 9
            if self.args.guided:
                self.send_guided_velocity(0.0, 0.0)
                self.drain_mavlink()
                time.sleep(0.5)
            else:
                self.prime_rc_override(channels)
                # With FS_THR_ENABLE enabled and no physical receiver, Rover can
                # latch an RC-loss stop before the first override arrives.  The
                # override clears that latch, but MANUAL must be re-entered after
                # the input is valid to hand control back to the pilot path.
                if not self.request_manual():
                    self.set_state("FAULT", "FCU did not re-enter MANUAL after RC override")
                    return 10
            if self.args.arm:
                if not self.arm():
                    self.set_state("FAULT", "FCU refused to arm; no movement was commanded")
                    return 7
            elif not self.is_armed():
                self.set_state("FAULT", "--send requires an armed FCU or --arm")
                print("Refusing to move while disarmed. Add --arm for this simulation test.", file=sys.stderr)
                return 8

        if self.args.guided:
            command = None
            self.log(
                "command_plan",
                mode="GUIDED",
                forward_speed=self.args.forward_speed,
                lateral_speed=self.args.lateral_speed,
                direction="left" if self.args.lateral_speed < 0 else "right",
                rear_gazebo_trim=self.args.rear_gazebo_trim,
                thrust_scale=self.args.thrust_scale,
            )
            print(
                f"OmniPlus plan: GUIDED BODY_NED, "
                f"vx={self.args.forward_speed:.2f} m/s, "
                f"vy={self.args.lateral_speed:.2f} m/s, yaw=hold-current"
            )
        else:
            lateral_pwm = pwm_from_normalized(self.args.lateral)
            command = {
                "steering": NEUTRAL_PWM,
                "throttle": NEUTRAL_PWM,
                "lateral": lateral_pwm,
            }
            self.log(
                "command_plan",
                mode="MANUAL",
                direction="left" if self.args.lateral < 0 else "right",
                lateral=self.args.lateral,
                lateral_pwm=lateral_pwm,
                rear_gazebo_trim=self.args.rear_gazebo_trim,
                thrust_scale=self.args.thrust_scale,
            )
            print(
                f"OmniPlus plan: MANUAL, steering=1500, throttle=1500, "
                f"lateral={lateral_pwm}us, rear Gazebo trim={self.args.rear_gazebo_trim:.2f}"
            )
        if not self.args.send:
            self.set_state("DRY_RUN", "no Guided setpoint, RC override, or Gazebo thrust sent")
            return 0

        self.set_state("COMMANDING", "native OmniPlus outputs drive Gazebo thrusters")
        start_pose = self.pose_snapshot()
        end = time.monotonic() + self.args.duration
        try:
            while time.monotonic() < end:
                if self.args.guided:
                    self.send_guided_velocity(
                        self.args.forward_speed, self.args.lateral_speed
                    )
                else:
                    self.send_rc_override(channels, command)
                self.drain_mavlink()
                self.publish_from_fcu_outputs()
                time.sleep(1.0 / self.args.rate)
        finally:
            if self.args.guided:
                self.neutralize_guided()
            else:
                self.neutralize(channels)
            if self.armed_by_script and self.args.disarm_after:
                self.disarm()
        self.log("motion_result", start_pose=start_pose, end_pose=self.pose_snapshot())
        self.set_state("COMPLETE", "interval ended and Gazebo thrust was zeroed")
        return 0

    def close(self) -> None:
        self.log_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument(
        "--source-system", type=int, default=255,
        help="MAVLink sender id; must match FCU MAV_GCS_SYSID for RC override",
    )
    parser.add_argument("--model", default="usv_four_thruster")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--lateral", type=float, default=-0.20,
                        help="native lateral input, -1..1; negative is left for this SDF")
    parser.add_argument("--guided", action="store_true",
                        help="use GUIDED BODY_NED velocity targets; requires the local Guided extension")
    parser.add_argument("--forward-speed", type=float, default=0.0,
                        help="GUIDED body-frame forward velocity in m/s")
    parser.add_argument("--lateral-speed", type=float, default=-0.30,
                        help="GUIDED body-frame right velocity in m/s; negative is left")
    parser.add_argument("--steering-channel", type=int, default=None)
    parser.add_argument("--throttle-channel", type=int, default=None)
    parser.add_argument("--lateral-channel", type=int, default=None)
    parser.add_argument("--rear-gazebo-trim", type=float, default=1.0,
                        help="optional Gazebo-only rear multiplier; native FCU output remains unchanged")
    parser.add_argument("--thrust-scale", type=float, default=35.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--log", type=Path, default=Path("mavlink_omniplus_gazebo.jsonl"))
    parser.add_argument("--send", action="store_true",
                        help="send RC overrides and drive Gazebo; default is dry-run")
    parser.add_argument("--arm", action="store_true",
                        help="request normal FCU arming; only meaningful with --send")
    parser.add_argument(
        "--release-safety", action="store_true",
        help="release Pixhawk hardware safety; use only with props disconnected",
    )
    parser.add_argument("--disarm-after", action=argparse.BooleanOptionalAction, default=True,
                        help="disarm if this script armed the FCU; default: yes")
    args = parser.parse_args()

    if not -1.0 <= args.lateral <= 1.0:
        parser.error("--lateral must be between -1 and 1")
    if abs(args.forward_speed) > 5.0 or abs(args.lateral_speed) > 5.0:
        parser.error("GUIDED speeds must be between -5 and 5 m/s")
    if args.duration <= 0 or args.rate <= 0:
        parser.error("--duration and --rate must be positive")
    if not 1 <= args.source_system <= 255:
        parser.error("--source-system must be between 1 and 255")
    if args.rear_gazebo_trim < 0 or args.thrust_scale <= 0:
        parser.error("--rear-gazebo-trim must be non-negative and --thrust-scale positive")
    for name in ("steering_channel", "throttle_channel", "lateral_channel"):
        value = getattr(args, name)
        if value is not None and not 1 <= value <= 8:
            parser.error(f"--{name.replace('_', '-')} must be between 1 and 8")

    args.log.parent.mkdir(parents=True, exist_ok=True)
    print(f"Opening {args.device}")
    try:
        # The FCU currently has MAV_GCS_SYSID=255.  ArduPilot accepts
        # RC_CHANNELS_OVERRIDE only from that configured GCS system id; using
        # the old source id 250 made every manual input look like no RC input.
        connection = mavutil.mavlink_connection(
            args.device, baud=args.baud, autoreconnect=True,
            source_system=args.source_system,
        )
        heartbeat = connection.wait_heartbeat(timeout=10)
    except Exception as exc:
        print(f"Could not receive FCU heartbeat: {exc}", file=sys.stderr)
        return 2

    print(
        f"Heartbeat: system={connection.target_system} "
        f"component={connection.target_component} type={heartbeat.type} "
        f"mode={mavutil.mode_string_v10(heartbeat)}"
    )
    bridge = OmniPlusBridge(connection, args)
    bridge.heartbeat = heartbeat
    bridge.log("logger_start", send=args.send, arm=args.arm, lateral=args.lateral)
    try:
        return bridge.run()
    except KeyboardInterrupt:
        bridge.set_state("STOPPING", "operator interrupted")
        if args.send:
            try:
                if args.guided:
                    bridge.neutralize_guided()
                else:
                    channels = bridge.resolve_input_channels()
                    bridge.neutralize(channels)
                if bridge.armed_by_script and args.disarm_after:
                    bridge.disarm()
            except Exception as exc:
                print(f"Emergency neutralization failed: {exc}", file=sys.stderr)
        return 130
    except Exception as exc:
        bridge.log("exception", error=repr(exc))
        print(f"OmniPlus bridge failed: {exc}", file=sys.stderr)
        return 1
    finally:
        bridge.close()


if __name__ == "__main__":
    raise SystemExit(main())
