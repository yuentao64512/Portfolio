#!/usr/bin/env python3
"""
Ground-station script: upload a polygon geofence to the UAV over MAVLink.

Reads vertices from a CSV file (one "lat,lon" per line - see
your_fence.csv), uploads them as an ArduPilot inclusion (or exclusion)
polygon fence via the MAVLink mission protocol, then downloads the fence
back from the vehicle and compares it point-by-point against what was sent,
so "upload succeeded" means verified, not just "a message went out".

This changes what the vehicle will do if it nears the fence boundary -
by default it asks for confirmation before uploading. Does NOT enable the
fence by itself unless you pass --enable-fence; uploading geometry and
switching the fence on are kept as separate, deliberate steps.

Examples:
    python3 geofence_publisher.py --connection udp:127.0.0.1:14550 --fence-file your_fence.csv
    python3 geofence_publisher.py --fence-file my_fence.csv --enable-fence
    python3 geofence_publisher.py --fence-file my_fence.csv --exclusion -y
"""

import argparse
import sys
from pathlib import Path

from mavlink_param_link import (
    MavlinkLinkError,
    connect_and_wait_for_heartbeat,
    download_fence,
    get_param,
    load_fence_csv,
    set_param,
    upload_fence_polygon,
)

# ArduPilot's FENCE_TYPE bitmask (firmware convention, not a MAVLink wire
# constant - verify against your firmware's parameter docs if in doubt):
# bit 0 (1) = max altitude, bit 1 (2) = circle, bit 2 (4) = polygon.
FENCE_TYPE_POLYGON_BIT = 4


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udp:127.0.0.1:14550", help="MAVLink connection string (default: %(default)s)")
    parser.add_argument("--fence-file", required=True, type=Path, help="CSV file of 'lat,lon' vertices, one per line")
    parser.add_argument("--exclusion", action="store_true", help="upload as an exclusion zone instead of an inclusion (allowed) zone")
    parser.add_argument("--enable-fence", action="store_true", help="after a verified upload, also set FENCE_ENABLE=1 (and ensure the polygon bit is set in FENCE_TYPE)")
    parser.add_argument("-y", "--yes", action="store_true", help="don't ask for confirmation before uploading")
    parser.add_argument("--timeout", type=float, default=5.0, help="seconds to wait per protocol step (default: %(default)s)")
    args = parser.parse_args()

    try:
        vertices = load_fence_csv(args.fence_file)
    except (OSError, ValueError) as e:
        print(f"Error reading {args.fence_file}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Loaded {len(vertices)} point(s) from {args.fence_file}:")
    for i, v in enumerate(vertices, start=1):
        print(f"  {i}. {v.lat:.7f}, {v.lon:.7f}")
    closes_itself = len(vertices) >= 2 and vertices[0].lat == vertices[-1].lat and vertices[0].lon == vertices[-1].lon
    unique_count = len(vertices) - 1 if closes_itself else len(vertices)
    if closes_itself:
        print(f"(last point repeats the first - treated as an explicit close; "
              f"{unique_count} unique vertices will actually be uploaded, ArduPilot auto-closes the polygon)")
    print()

    kind = "EXCLUSION" if args.exclusion else "INCLUSION"
    if not args.yes:
        answer = input(f"Upload this {unique_count}-vertex {kind} fence to the vehicle at {args.connection}? [y/N] ").strip().lower()
        if answer != "y":
            print("Cancelled.")
            sys.exit(0)

    try:
        vehicle = connect_and_wait_for_heartbeat(args.connection)
    except MavlinkLinkError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Connected to system {vehicle.system_id}/component {vehicle.component_id}\n")

    ok, message = upload_fence_polygon(vehicle, vertices, inclusion=not args.exclusion, timeout_s=args.timeout)
    print(("Upload OK: " if ok else "Upload FAILED: ") + message)
    if not ok:
        sys.exit(1)

    print("\nVerifying by downloading the fence back from the vehicle...")
    try:
        downloaded = download_fence(vehicle, timeout_s=args.timeout)
    except MavlinkLinkError as e:
        print(f"Verification FAILED: could not download the fence back: {e}", file=sys.stderr)
        sys.exit(1)

    expected = vertices[:-1] if closes_itself else vertices
    mismatches = []
    if len(downloaded) != len(expected):
        mismatches.append(f"vertex count: expected {len(expected)}, got {len(downloaded)} back")
    for i, (exp, got) in enumerate(zip(expected, downloaded)):
        if abs(exp.lat - got.lat) > 1e-6 or abs(exp.lon - got.lon) > 1e-6:
            mismatches.append(f"vertex {i + 1}: expected ({exp.lat:.7f}, {exp.lon:.7f}), got ({got.lat:.7f}, {got.lon:.7f})")

    if mismatches:
        print("Verification FAILED - downloaded fence doesn't match what was sent:")
        for m in mismatches:
            print(f"  {m}")
        sys.exit(1)
    print(f"Verified: all {len(downloaded)} vertices match exactly what was uploaded.")

    if args.enable_fence:
        print("\nEnabling fence...")
        try:
            fence_type = get_param(vehicle, "FENCE_TYPE", timeout_s=args.timeout)
            if not (int(fence_type) & FENCE_TYPE_POLYGON_BIT):
                new_type = int(fence_type) | FENCE_TYPE_POLYGON_BIT
                ok, msg, _ = set_param(vehicle, "FENCE_TYPE", new_type, timeout_s=args.timeout)
                print(f"  FENCE_TYPE {int(fence_type)} -> {new_type} (added polygon bit): " + ("OK" if ok else "FAILED: " + msg))
        except MavlinkLinkError as e:
            print(f"  could not check/update FENCE_TYPE: {e}")

        ok, msg, _ = set_param(vehicle, "FENCE_ENABLE", 1, timeout_s=args.timeout)
        print(("  FENCE_ENABLE=1: OK" if ok else "  FENCE_ENABLE=1: FAILED: " + msg))


if __name__ == "__main__":
    main()
