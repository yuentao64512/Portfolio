#!/usr/bin/env python3
"""Guarded MAVLink crab command for a four-thruster USV.

The FCU must have the bow and stern lateral ESCs connected to explicitly
selected SERVO outputs whose SERVOx_FUNCTION is 0 (disabled/manual). This
script then uses MAV_CMD_DO_SET_SERVO repeatedly at a low rate. It does not
arm the vehicle and it does not change parameters.

The default mode is dry-run. Use --send only after checking the output map,
disconnecting propellers for the first test, and confirming the command signs
with the hull secured. The state/telemetry stream is written to JSONL for
state-machine extraction.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise SystemExit(
        "pymavlink is missing. Install it with: python3 -m pip install pymavlink"
    ) from exc


DEFAULT_DEVICE = (
    "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_200029001351333531343838-if00"
)
NEUTRAL_PWM = 1500
MIN_PWM = 1000
MAX_PWM = 2000


def command_plan(args) -> tuple[float, float, int, int]:
    left = -abs(args.magnitude)
    front = max(-1.0, min(1.0, left * args.front_sign))
    rear = max(-1.0, min(1.0, left * args.rear_trim * args.rear_sign))
    front_pwm = int(round(NEUTRAL_PWM + front * 500))
    rear_pwm = int(round(NEUTRAL_PWM + rear * 500))
    return front, rear, front_pwm, rear_pwm


class CrabController:
    def __init__(self, connection, args):
        self.connection = connection
        self.args = args
        self.system = connection.target_system
        self.component = connection.target_component
        self.state = "CONNECTED"
        self.log_file = args.log.open("w", encoding="utf-8")

    def log(self, record: str, **fields) -> None:
        item = {
            "t_unix": time.time(),
            "t_monotonic": time.monotonic() - self.args.started,
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

    def drain_telemetry(self) -> None:
        while True:
            message = self.connection.recv_match(blocking=False)
            if message is None:
                return
            message_type = message.get_type()
            if message_type in {
                "HEARTBEAT", "ATTITUDE", "LOCAL_POSITION_NED", "VFR_HUD",
                "GLOBAL_POSITION_INT", "SERVO_OUTPUT_RAW", "STATUSTEXT",
                "COMMAND_ACK",
            }:
                self.log("mavlink", message=message_type, data=message.to_dict())

    def get_param(self, name: str, timeout: float = 3.0):
        self.connection.mav.param_request_read_send(
            self.system, self.component, name.encode("ascii"), -1
        )
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            message = self.connection.recv_match(
                type="PARAM_VALUE", blocking=True, timeout=0.25
            )
            if message is not None:
                param_id = message.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode("ascii", errors="ignore")
                if param_id.rstrip("\x00") == name:
                    return float(message.param_value)
        return None

    def check_output_map(self) -> bool:
        self.set_state("CHECKING_OUTPUT_MAP", "reading selected SERVOx_FUNCTION values")
        ok = True
        for channel in (self.args.front_channel, self.args.rear_channel):
            function = self.get_param(f"SERVO{channel}_FUNCTION")
            self.log("output_map", channel=channel, function=function)
            if function is None:
                print(f"SERVO{channel}_FUNCTION was not received", file=sys.stderr)
                ok = False
            elif int(function) != 0:
                print(
                    f"SERVO{channel}_FUNCTION={int(function)}; expected 0 for "
                    "safe direct servo commands. Refusing to send.",
                    file=sys.stderr,
                )
                ok = False
        return ok

    def send_servo(self, channel: int, pwm: int) -> None:
        pwm = max(MIN_PWM, min(MAX_PWM, int(pwm)))
        self.connection.mav.command_long_send(
            self.system,
            self.component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            float(channel),
            float(pwm),
            0, 0, 0, 0, 0,
        )
        self.log("command", command="MAV_CMD_DO_SET_SERVO",
                 channel=channel, pwm=pwm)

    def command_pair(self, front_pwm: int, rear_pwm: int) -> None:
        self.send_servo(self.args.front_channel, front_pwm)
        self.send_servo(self.args.rear_channel, rear_pwm)

    def stop(self) -> None:
        self.set_state("STOPPING", "commanding neutral PWM to both lateral outputs")
        for _ in range(3):
            self.command_pair(NEUTRAL_PWM, NEUTRAL_PWM)
            self.drain_telemetry()
            time.sleep(0.1)

    def run(self) -> int:
        if not self.check_output_map():
            self.set_state("FAULT", "output map is missing or not manual/disabled")
            return 3

        heartbeat = self.connection.messages.get("HEARTBEAT")
        armed = bool(
            heartbeat
            and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
        self.log("preflight", armed=armed,
                 mode=mavutil.mode_string_v10(heartbeat) if heartbeat else None)
        if armed and not self.args.allow_armed:
            self.set_state("FAULT", "vehicle is armed; pass --allow-armed explicitly")
            print("Refusing to command an armed vehicle without --allow-armed.",
                  file=sys.stderr)
            return 4

        front, rear, front_pwm, rear_pwm = command_plan(self.args)
        self.log("command_plan", direction="left", front_normalized=front,
                 rear_normalized=rear, front_pwm=front_pwm, rear_pwm=rear_pwm,
                 rear_trim=self.args.rear_trim)
        print(
            f"left crab plan: front={front_pwm}us rear={rear_pwm}us "
            f"(rear trim {self.args.rear_trim:.2f})"
        )

        if not self.args.send:
            self.set_state("DRY_RUN", "no MAVLink actuator messages sent")
            return 0

        self.set_state("COMMANDING", "sending repeated MAV_CMD_DO_SET_SERVO")
        end = time.monotonic() + self.args.duration
        try:
            while time.monotonic() < end:
                self.command_pair(front_pwm, rear_pwm)
                self.drain_telemetry()
                time.sleep(1.0 / self.args.rate)
        finally:
            self.stop()
        self.set_state("COMPLETE", "crab interval ended and outputs were neutralized")
        return 0

    def close(self) -> None:
        self.log_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--front-channel", type=int, required=True,
                        help="1-based SERVO output channel for the bow lateral ESC")
    parser.add_argument("--rear-channel", type=int, required=True,
                        help="1-based SERVO output channel for the stern lateral ESC")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--magnitude", type=float, default=0.20,
                        help="normalized front command, 0..1; default 0.20")
    parser.add_argument("--rear-trim", type=float, default=1.4,
                        help="stern/front magnitude ratio; default 1.4")
    parser.add_argument("--front-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--rear-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    parser.add_argument("--rate", type=float, default=10.0,
                        help="command repeat rate in Hz; default 10")
    parser.add_argument("--log", type=Path, default=Path("fcu_crab_state.jsonl"))
    parser.add_argument("--send", action="store_true",
                        help="actually send actuator commands; default is dry-run")
    parser.add_argument("--allow-armed", action="store_true",
                        help="required to send while the FCU reports armed")
    args = parser.parse_args()

    if not 0 < args.magnitude <= 1:
        parser.error("--magnitude must be greater than 0 and at most 1")
    if args.duration <= 0 or args.rate <= 0:
        parser.error("--duration and --rate must be positive")
    if args.front_channel == args.rear_channel:
        parser.error("front and rear channels must be different")

    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.started = time.monotonic()
    if not args.send:
        front, rear, front_pwm, rear_pwm = command_plan(args)
        dry_run_records = [
            {"t_unix": time.time(), "record": "state", "state": "DRY_RUN",
             "reason": "no MAVLink actuator messages sent"},
            {"t_unix": time.time(), "record": "command_plan", "state": "DRY_RUN",
             "direction": "left", "front_normalized": front,
             "rear_normalized": rear, "front_pwm": front_pwm,
             "rear_pwm": rear_pwm, "rear_trim": args.rear_trim},
        ]
        with args.log.open("w", encoding="utf-8") as log_file:
            for record in dry_run_records:
                log_file.write(json.dumps(record, separators=(",", ":")) + "\n")
        print(
            f"DRY RUN: no MAVLink messages sent; front={front_pwm}us "
            f"rear={rear_pwm}us (rear trim {args.rear_trim:.2f})"
        )
        print(f"Saved dry-run state log to {args.log}")
        return 0

    print(f"Opening {args.device}")
    try:
        connection = mavutil.mavlink_connection(
            args.device, baud=args.baud, autoreconnect=True, source_system=250
        )
        heartbeat = connection.wait_heartbeat(timeout=10)
    except Exception as exc:
        print(f"Could not receive an FCU heartbeat: {exc}", file=sys.stderr)
        return 2

    print(
        f"Heartbeat: system={connection.target_system} "
        f"component={connection.target_component} "
        f"type={heartbeat.type} mode={mavutil.mode_string_v10(heartbeat)}"
    )
    controller = CrabController(connection, args)
    controller.log("logger_start", direction="left", rear_trim=args.rear_trim,
                   send=args.send)
    try:
        return controller.run()
    except KeyboardInterrupt:
        controller.set_state("STOPPING", "operator interrupted command loop")
        if args.send:
            controller.stop()
        return 130
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
