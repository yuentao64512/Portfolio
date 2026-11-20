#!/usr/bin/env python3
"""Request and verify an ArduPilot FCU flight mode without actuator output."""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


DEFAULT_DEVICE = (
    "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_200029001351333531343838-if00"
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", default="GUIDED")
    args = parser.parse_args()
    requested = args.mode.upper()

    try:
        connection = mavutil.mavlink_connection(
            args.device, baud=args.baud, autoreconnect=True, source_system=250
        )
        heartbeat = connection.wait_heartbeat(timeout=10)
    except Exception as exc:
        print(f"Could not receive FCU heartbeat: {exc}", file=sys.stderr)
        return 2

    current = mavutil.mode_string_v10(heartbeat)
    print(f"Current FCU mode: {current}")
    mapping = connection.mode_mapping() or {}
    if requested not in mapping:
        print(f"FCU does not advertise {requested}; available: "
              f"{', '.join(sorted(mapping))}", file=sys.stderr)
        return 3

    if current.upper() != requested:
        connection.set_mode(requested)
    end = time.monotonic() + 5.0
    while time.monotonic() < end:
        message = connection.recv_match(
            type="HEARTBEAT", blocking=True, timeout=0.5
        )
        if message is None:
            continue
        current = mavutil.mode_string_v10(message)
        if current.upper() == requested:
            print(f"Verified FCU mode: {current}")
            return 0

    print(f"FCU did not reach {requested} mode", file=sys.stderr)
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
