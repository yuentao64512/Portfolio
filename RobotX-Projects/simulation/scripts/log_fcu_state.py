#!/usr/bin/env python3
"""Receive-only MAVLink logger for extracting an FCU state machine.

This program never arms, changes parameters, overrides RC, or sends actuator
commands. It writes one JSON object per line so the log can be replayed or
converted into a state-machine diagram later.
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
LOGGED_TYPES = {
    "HEARTBEAT",
    "ATTITUDE",
    "GLOBAL_POSITION_INT",
    "LOCAL_POSITION_NED",
    "VFR_HUD",
    "SYS_STATUS",
    "RC_CHANNELS",
    "SERVO_OUTPUT_RAW",
    "STATUSTEXT",
}


def armed_from_heartbeat(message) -> bool | None:
    if message.get_type() != "HEARTBEAT":
        return None
    return bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def mode_from_heartbeat(message) -> str | None:
    if message.get_type() != "HEARTBEAT":
        return None
    try:
        return mavutil.mode_string_v10(message)
    except Exception:
        return str(message.custom_mode)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0,
                        help="Seconds to log; 0 means until Ctrl+C")
    parser.add_argument("--output", type=Path, default=Path("fcu_state.jsonl"))
    args = parser.parse_args()

    print(f"Opening {args.device} (receive-only logger)")
    try:
        connection = mavutil.mavlink_connection(
            args.device,
            baud=args.baud,
            autoreconnect=True,
            source_system=250,
        )
        heartbeat = connection.wait_heartbeat(timeout=10)
    except Exception as exc:
        print(f"Could not receive an FCU heartbeat: {exc}", file=sys.stderr)
        return 2

    print(
        f"Heartbeat from system={connection.target_system} "
        f"component={connection.target_component} "
        f"type={heartbeat.type} mode={mode_from_heartbeat(heartbeat)}"
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    deadline = None if args.duration == 0 else started + args.duration
    state = "ARMED" if armed_from_heartbeat(heartbeat) else "DISARMED"

    with args.output.open("w", encoding="utf-8") as log_file:
        metadata = {
            "t_unix": time.time(),
            "record": "logger_start",
            "state": state,
            "device": args.device,
            "target_system": connection.target_system,
            "target_component": connection.target_component,
        }
        log_file.write(json.dumps(metadata, separators=(",", ":")) + "\n")

        try:
            while deadline is None or time.monotonic() < deadline:
                message = connection.recv_match(blocking=True, timeout=1)
                if message is None or message.get_type() not in LOGGED_TYPES:
                    continue

                record = {
                    "t_unix": time.time(),
                    "t_monotonic": time.monotonic() - started,
                    "record": "mavlink",
                    "message": message.get_type(),
                    "state": state,
                    "data": message.to_dict(),
                }
                if message.get_type() == "HEARTBEAT":
                    state = "ARMED" if armed_from_heartbeat(message) else "DISARMED"
                    record["state"] = state
                    record["mode"] = mode_from_heartbeat(message)
                log_file.write(json.dumps(record, default=str,
                                           separators=(",", ":")) + "\n")
                log_file.flush()
        except KeyboardInterrupt:
            print("Stopping logger")

    print(f"Saved FCU state log to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
