#!/usr/bin/env python3
"""
Ground-station publisher: sends geofence vertices over a plain UDP wireless
link to the Orin's geofence_udp_listener.py, repeatedly, for as long as
this keeps running.

This does NOT speak MAVLink at all - it's a simple JSON-over-UDP channel
for a GCS laptop that has no direct MAVLink path to the vehicle (just
WiFi/Tailscale to the Orin). The Orin, which DOES have a MAVLink link via
MAVProxy, receives this and does the actual fence upload.

While running, the same fence is re-sent every --interval seconds as a
liveness signal - the listener uses this to detect when you've stopped
publishing. On Ctrl+C, one final "bye" message is sent so the listener can
revert to its preset fence immediately, instead of waiting out the timeout.

Usage:
    python3 geofence_udp_publisher.py --listener-host 192.168.1.153 --fence-file your_fence.csv
"""

import argparse
import json
import socket
import sys
import time
from pathlib import Path

from fence_csv import load_fence_csv, load_fence_max_alt


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listener-host", required=True, help="IP address of the Orin running geofence_udp_listener.py")
    parser.add_argument("--listener-port", type=int, default=14600, help="UDP port the listener is bound to (default: %(default)s)")
    parser.add_argument("--fence-file", required=True, type=Path, help="CSV of 'lat,lon' vertices")
    parser.add_argument("--exclusion", action="store_true", help="publish as an exclusion zone instead of inclusion")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between repeated sends (default: %(default)s)")
    parser.add_argument(
        "--fence-type",
        type=int,
        default=None,
        help="exact FENCE_TYPE bitmask to set on the FCU (1=max alt, 2=circle, 4=polygon; e.g. 7 = all three). "
        "If omitted, the listener just adds the polygon bit to whatever FENCE_TYPE already was",
    )
    parser.add_argument("--enable-fence", action="store_true", help="also set FENCE_ENABLE=1 on the FCU once the fence is uploaded")
    args = parser.parse_args()

    if args.fence_type is not None and not 0 <= args.fence_type <= 255:
        parser.error("--fence-type must be a bitmask between 0 and 255")

    try:
        vertices = load_fence_csv(args.fence_file)
        max_alt = load_fence_max_alt(args.fence_file)
    except (OSError, ValueError) as e:
        print(f"Error reading {args.fence_file}: {e}", file=sys.stderr)
        sys.exit(1)
    if len(vertices) < 3:
        print(f"Error: need at least 3 vertices, got {len(vertices)}", file=sys.stderr)
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.listener_host, args.listener_port)
    base_payload = {
        "type": "fence",
        "vertices": [[v.lat, v.lon] for v in vertices],
        "inclusion": not args.exclusion,
        "enable": args.enable_fence,
    }
    if args.fence_type is not None:
        base_payload["fence_type"] = args.fence_type
    if max_alt is not None:
        base_payload["max_alt"] = max_alt
        if args.fence_type is not None and not args.fence_type & 1:
            print(f"Warning: the fence file sets max_alt={max_alt:g} m but --fence-type {args.fence_type} "
                  "has no max-altitude bit (1), so ArduPilot won't enforce it.", file=sys.stderr)

    kind = "EXCLUSION" if args.exclusion else "INCLUSION"
    if args.fence_type is not None:
        type_desc = f"FENCE_TYPE={args.fence_type}"
    elif max_alt is not None:
        type_desc = "FENCE_TYPE += polygon and max-alt bits"
    else:
        type_desc = "FENCE_TYPE += polygon bit"
    alt_desc = f", FENCE_ALT_MAX={max_alt:g} m" if max_alt is not None else ""
    enable_desc = ", FENCE_ENABLE=1" if args.enable_fence else ""
    print(f"Publishing {len(vertices)}-vertex {kind} fence ({type_desc}{alt_desc}{enable_desc}) to "
          f"{args.listener_host}:{args.listener_port} every {args.interval:g}s. "
          f"Ctrl+C to stop (and revert the vehicle's fence to its preset).")

    seq = 0
    try:
        while True:
            seq += 1
            payload = dict(base_payload, seq=seq, timestamp=time.time())
            sock.sendto(json.dumps(payload).encode("utf-8"), target)
            print(f"  sent (seq {seq})", end="\r", flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nStopping - sending 'bye' so the listener reverts immediately...")
        sock.sendto(json.dumps({"type": "bye"}).encode("utf-8"), target)
        time.sleep(0.2)  # give the packet a moment to actually go out before the process exits
        print("Stopped.")


if __name__ == "__main__":
    main()
