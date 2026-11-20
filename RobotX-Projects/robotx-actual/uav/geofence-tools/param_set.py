#!/usr/bin/env python3
"""
Dynamically read/change ArduPilot parameters over MAVLink, including
geofence parameters (FENCE_ENABLE, FENCE_TYPE, FENCE_ACTION, etc.).

Shows the current value before changing it, asks for confirmation (unless
--yes), then sets it and verifies the vehicle's own echo confirms the new
value actually took - ArduPilot can silently reject or clamp a bad value,
so "message sent" alone isn't proof anything changed.

Examples:
    # read only
    python3 param_set.py --connection udp:127.0.0.1:14550 --get FENCE_ENABLE

    # set one param
    python3 param_set.py --connection udp:127.0.0.1:14550 --set FENCE_ENABLE=1

    # set several at once
    python3 param_set.py --set FENCE_ENABLE=1 --set FENCE_ACTION=1 --set FENCE_RADIUS=100

    # see common fence params at a glance
    python3 param_set.py --fence-summary
"""

import argparse
import sys

from mavlink_param_link import MavlinkLinkError, connect_and_wait_for_heartbeat, get_param, set_param

COMMON_FENCE_PARAMS = [
    "FENCE_ENABLE",
    "FENCE_TYPE",
    "FENCE_ACTION",
    "FENCE_ALT_MAX",
    "FENCE_RADIUS",
    "FENCE_MARGIN",
    "FENCE_TOTAL",
]


def parse_assignment(text: str):
    if "=" not in text:
        raise argparse.ArgumentTypeError(f"expected NAME=VALUE, got '{text}'")
    name, value = text.split("=", 1)
    name = name.strip().upper()
    try:
        value = float(value.strip())
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{value}' in '{text}' isn't a number")
    return name, value


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udp:127.0.0.1:14550", help="MAVLink connection string (default: %(default)s)")
    parser.add_argument("--get", action="append", default=[], metavar="NAME", help="read a parameter (repeatable)")
    parser.add_argument("--set", action="append", default=[], type=parse_assignment, metavar="NAME=VALUE", help="set a parameter (repeatable)")
    parser.add_argument("--fence-summary", action="store_true", help="print the current value of common FENCE_* parameters and exit")
    parser.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation before each set")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait for each response (default: %(default)s)")
    args = parser.parse_args()

    if args.fence_summary:
        args.get = COMMON_FENCE_PARAMS + args.get

    if not args.get and not args.set:
        parser.error("nothing to do - pass --get NAME, --set NAME=VALUE, or --fence-summary")

    try:
        vehicle = connect_and_wait_for_heartbeat(args.connection)
    except MavlinkLinkError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Connected to system {vehicle.system_id}/component {vehicle.component_id} on {args.connection}\n")

    for name in args.get:
        try:
            value = get_param(vehicle, name, timeout_s=args.timeout)
            print(f"{name} = {value:g}")
        except MavlinkLinkError as e:
            print(f"{name}: {e}")

    if args.set:
        print()
    for name, value in args.set:
        try:
            current = get_param(vehicle, name, timeout_s=args.timeout)
            print(f"{name}: currently {current:g}, requesting {value:g}")
        except MavlinkLinkError as e:
            print(f"{name}: could not read current value ({e}) - continuing anyway")

        if not args.yes:
            answer = input(f"  Set {name} to {value:g}? [y/N] ").strip().lower()
            if answer != "y":
                print("  skipped")
                continue

        ok, message, _confirmed = set_param(vehicle, name, value, timeout_s=args.timeout)
        print(("  OK: " if ok else "  FAILED: ") + message)


if __name__ == "__main__":
    main()
