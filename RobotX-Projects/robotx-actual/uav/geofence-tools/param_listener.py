#!/usr/bin/env python3
"""
Passively watch for ArduPilot parameter changes over MAVLink and print them
live - useful for confirming a change made elsewhere (Mission Planner,
param_set.py, another GCS) actually reached the vehicle, or just for
watching FENCE_* params while testing.

By default requests the full parameter list once at startup (so you see
every current value up front), then keeps listening for any further
PARAM_VALUE messages - including ones the vehicle sends unprompted whenever
a parameter actually changes.

Examples:
    python3 param_listener.py --connection udp:127.0.0.1:14550
    python3 param_listener.py --filter "FENCE_*"
    python3 param_listener.py --filter "FENCE_*" --no-initial-dump
"""

import argparse
import fnmatch
import sys
import time

from mavlink_param_link import MavlinkLinkError, connect_and_wait_for_heartbeat, listen_params

RESET = "\x1b[0m"
DIM = "\x1b[2m"
GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udp:127.0.0.1:14550", help="MAVLink connection string (default: %(default)s)")
    parser.add_argument("--filter", default="*", help="glob pattern to limit which params are shown, e.g. 'FENCE_*' (default: %(default)s)")
    parser.add_argument("--no-initial-dump", dest="initial_dump", action="store_false", help="don't request/print the full current param list at startup - only show live changes")
    args = parser.parse_args()

    try:
        vehicle = connect_and_wait_for_heartbeat(args.connection)
    except MavlinkLinkError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Connected to system {vehicle.system_id}/component {vehicle.component_id} on {args.connection}")
    print(f"Watching params matching '{args.filter}'. Ctrl+C to stop.\n")

    seen = {}
    initial_dump_done = {"flag": not args.initial_dump}

    def on_param(name, value, index, total):
        # listen_params() deliberately doesn't filter - it calls this for
        # EVERY param, matching or not, so completion tracking below (which
        # needs to see index reach total-1 for the true full list, not just
        # the filtered subset) stays correct regardless of where FENCE_*
        # params happen to fall in the vehicle's param ordering.
        now = time.strftime("%H:%M:%S")
        matches = fnmatch.fnmatch(name, args.filter)

        if not initial_dump_done["flag"]:
            if matches:
                print(f"{DIM}[{now}] {name} = {value:g}  ({index + 1}/{total}){RESET}")
            if index + 1 >= total:
                initial_dump_done["flag"] = True
                print(f"{DIM}-- initial dump complete, now watching for live changes --{RESET}\n")
            return

        if not matches:
            return

        previous = seen.get(name)
        seen[name] = value
        if previous is None:
            print(f"[{now}] {name} = {value:g}  (first seen)")
        elif previous != value:
            print(f"{YELLOW}[{now}] {name} changed: {previous:g} -> {value:g}{RESET}")
        else:
            print(f"{GREEN}[{now}] {name} = {value:g}  (unchanged){RESET}")

    try:
        listen_params(vehicle, on_param, request_all_first=args.initial_dump)
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
