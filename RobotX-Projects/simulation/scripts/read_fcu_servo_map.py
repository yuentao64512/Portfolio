#!/usr/bin/env python3
"""Read and print the FCU SERVO1_FUNCTION..SERVO16_FUNCTION map."""

from __future__ import annotations

import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise SystemExit(
        "pymavlink is missing. Install it with: python3 -m pip install pymavlink"
    ) from exc


DEFAULT_DEVICE = (
    "/dev/serial/by-id/usb-ArduPilot_Pixhawk6X_200029001351333531343838-if00"
)


def parameter_name(message) -> str:
    value = message.param_id
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="ignore")
    return value.rstrip("\x00")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default=DEFAULT_DEVICE)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    try:
        connection = mavutil.mavlink_connection(
            args.device, baud=args.baud, autoreconnect=True, source_system=250
        )
        connection.wait_heartbeat(timeout=10)
    except Exception as exc:
        print(f"Could not receive an FCU heartbeat: {exc}", file=sys.stderr)
        return 2

    names = {f"SERVO{channel}_FUNCTION" for channel in range(1, 17)}
    connection.mav.param_request_list_send(
        connection.target_system, connection.target_component
    )
    values = {}
    end = time.monotonic() + args.timeout
    while time.monotonic() < end and names - values.keys():
        message = connection.recv_match(
            type="PARAM_VALUE", blocking=True, timeout=0.5
        )
        if message is None:
            continue
        name = parameter_name(message)
        if name in names:
            values[name] = float(message.param_value)

    print("channel  function  use")
    print("-------  --------  ---------------------------")
    for channel in range(1, 17):
        name = f"SERVO{channel}_FUNCTION"
        function = values.get(name)
        use = "free for guarded DO_SET_SERVO test" if function == 0 else "configured"
        if function is None:
            use = "not received"
        print(f"{channel:7d}  {str(function):>8}  {use}")
    return 0 if len(values) == len(names) else 3


if __name__ == "__main__":
    raise SystemExit(main())
