#!/usr/bin/env python3
"""
Send the RobotX 2026 RunDeclaration to RoboCommand (GCS laptop), through
robocommand_bridge.py.

RoboCommand starts a run only after an accepted RunDeclaration: once before
every run, the team declares its vehicles and the tier it is attempting for
each task, and - with a UAV - the UAV's geofence, the geofence's ceiling in
metres above the WGS84 ellipsoid (HAE), and the pilot's MyDroneFleets call
sign.

This script only signs the request with the vehicles, task tiers, pilot call
sign and the UAV's vehicle id, and hands it to robocommand_bridge.py on this
laptop (--bridge-port). The bridge adds the rest from the drone itself:
  - the geofence: the polygon currently on the FCU (what Mission Planner shows);
  - the ceiling: the FCU's FENCE_ALT_MAX (set in Mission Planner);
checks it the way RoboCommand does (geofence closed with 4+ points, every
point within the course or UAV transit area to 1 m, ceiling above 0 and at
most 50 m HAE) and publishes it. A refused declaration gets NO reply from
RoboCommand, so when the check fails it is NOT sent: the fence is cleared
from the FCU and the listener's startup FENCE_* params are put back. Either
way the bridge's answer is printed here (and shown in the drone GUI).

Declare once per run, and only if your team's operator station doesn't
already: the RunDeclaration covers the whole team (USV, UUV, UAV). Needs the
bridge running, with the drone GUI on the Orin and the drone disarmed.

The bridge saves each declaration it sends to last_run_declaration.json: when
the FCU's fence or FENCE_ALT_MAX is later changed in Mission Planner, it
re-sends it the same way.

Usage:
    python3 run_declaration.py --vehicles USV1 UAV1 --task1 advanced
(the pilot call sign is DEFAULT_PILOT_CALLSIGN below; --pilot-callsign for another pilot)
"""

import argparse
import json
import math
import socket
import sys
import time
from pathlib import Path

from robocommand_bridge import DEFAULT_DECLARE_PORT

DEFAULT_PILOT_CALLSIGN = "Calwin-U2332301"  # MyDroneFleets call sign: first name + UAPL number
DEFAULT_VEHICLE_ID = "SGIT-53577"  # the drone's RoboCommand vehicle name
# The last RunDeclaration the bridge sent - it re-sends it with the new
# geofence / ceiling when the FCU's fence or FENCE_ALT_MAX changes.
LAST_DECLARATION_FILE = Path(__file__).resolve().parent / "last_run_declaration.json"
TIERS = {"none": "TIER_NONE", "core": "TIER_CORE", "advanced": "TIER_ADVANCED", "disruptive": "TIER_DISRUPTIVE"}
GEOFENCE_TOLERANCE_M = 1.0  # RoboCommand's: how far outside the course/transit area a point may be
EDGE_STEP_M = 0.5
BRIDGE_RESEND_S = 1.0  # re-send the request to the bridge this often until it answers


def _xy(p, origin):
    """Flat metres east/north of origin."""
    return ((p[1] - origin[1]) * 111320.0 * math.cos(math.radians(origin[0])), (p[0] - origin[0]) * 111320.0)


def _inside(p, ring) -> bool:
    inside = False
    for (x1, y1), (x2, y2) in zip(ring, ring[1:]):
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def _outside_by(p, ring) -> float:
    """0 inside the ring, else metres to its nearest edge."""
    if _inside(p, ring):
        return 0.0
    best = math.inf
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        dx, dy = bx - ax, by - ay
        t = 0.0 if dx == dy == 0 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / (dx * dx + dy * dy)))
        best = min(best, math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)))
    return best


def _closed(points):
    return points if points and points[0] == points[-1] else points + points[:1]


def geofence_problems(fence, corners, transit, ceiling, max_ceiling) -> list:
    """Why RoboCommand would refuse this UAV geofence (empty: it's fine)."""
    problems = []
    if len(fence) < 4:
        problems.append(f"the geofence has {len(fence)} point(s), needs at least 4 (first point repeated at the end)")
    elif fence[0] != fence[-1]:
        problems.append("the geofence isn't closed: its first and last points must be the same")
    elif corners:
        origin = corners[0]
        areas = [[_xy(p, origin) for p in _closed(ring)] for ring in (corners, transit) if len(ring) >= 3]
        pts = [_xy(p, origin) for p in fence]
        worst, where = 0.0, None
        for i, (a, b) in enumerate(zip(pts, pts[1:])):
            steps = max(1, math.ceil(math.dist(a, b) / EDGE_STEP_M))
            for k in range(steps + 1):
                q = (a[0] + (b[0] - a[0]) * k / steps, a[1] + (b[1] - a[1]) * k / steps)
                out = min(_outside_by(q, area) for area in areas)
                if out > worst:
                    worst = out
                    where = f"point {i + 1}" if k == 0 else f"point {i + 2}" if k == steps else f"edge {i + 1}-{i + 2}"
        if worst > GEOFENCE_TOLERANCE_M:
            problems.append(f"the geofence leaves the course and UAV transit area by {worst:.1f} m at {where}")
    if not 0 < ceiling <= max_ceiling:
        problems.append(f"the ceiling is {ceiling:.1f} m HAE: it must be above 0 and at most {max_ceiling:g} m HAE")
    return problems


def save_last_declaration(declaration: dict) -> None:
    try:
        LAST_DECLARATION_FILE.write_text(json.dumps(dict(declaration, saved_at=time.time()), indent=2))
    except OSError as e:
        print(f"Warning: couldn't save {LAST_DECLARATION_FILE.name} ({e}) - the bridge can't re-send it with a "
              "Mission Planner fence")


def load_last_declaration():
    """The last RunDeclaration sent from this laptop, or None."""
    try:
        return json.loads(LAST_DECLARATION_FILE.read_text())
    except (OSError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--broker", default=None, help="not used - robocommand_bridge.py sends it to its own broker")
    parser.add_argument("--port", type=int, default=None, help="not used - robocommand_bridge.py sends it to its own broker")
    parser.add_argument("--bridge-port", type=int, default=DEFAULT_DECLARE_PORT,
                        help="local UDP port robocommand_bridge.py takes Run Requests on (default: %(default)s)")
    parser.add_argument("--pilot-callsign", default=DEFAULT_PILOT_CALLSIGN,
                        help="the UAV pilot's MyDroneFleets call sign (first name + UAPL number) (default: %(default)s)")
    parser.add_argument("--vehicles", nargs="+", default=[DEFAULT_VEHICLE_ID], help="vehicle ids in this run (default: %(default)s)")
    for n in range(1, 5):
        parser.add_argument(f"--task{n}", choices=sorted(TIERS), default="none", help=f"Task {n} tier (default: %(default)s)")
    parser.add_argument("--uav-vehicle-id", default=DEFAULT_VEHICLE_ID, help="the UAV's vehicle id, for its heartbeat (default: %(default)s)")
    parser.add_argument("--wait", type=float, default=15.0, help="seconds to wait for the bridge's answer (default: %(default)s)")
    parser.add_argument("--yes", action="store_true", help="don't ask before sending")
    args = parser.parse_args()

    has_uav = any(v.upper().startswith("UAV") or v == args.uav_vehicle_id for v in args.vehicles)
    tiers = {n: getattr(args, f"task{n}") for n in range(1, 5)}
    print(f"RunDeclaration:\n  vehicles: {' '.join(args.vehicles)}\n  tiers: "
          + ", ".join(f"Task {n} {tiers[n]}" for n in range(1, 5))
          + (f"\n  UAV {args.uav_vehicle_id}: pilot {args.pilot_callsign}; geofence = the FCU's fence, ceiling = "
             "FENCE_ALT_MAX (added by robocommand_bridge.py)" if has_uav else ""))
    print("Send it only if your team's operator station isn't sending its own for this run.")
    if not args.yes and input("Send this RunDeclaration? [y/N] ").strip().lower() not in ("y", "yes"):
        sys.exit("Not sent.")

    request_id = f"decl-{time.time_ns()}"
    request = {"type": "declare", "id": request_id, "vehicles": args.vehicles, "tiers": tiers,
               "uav_vehicle_id": args.uav_vehicle_id,
               "uav": {"pilot_callsign": args.pilot_callsign} if has_uav else None}
    data = json.dumps(request).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.2)
    target = ("127.0.0.1", args.bridge_port)
    deadline, last_sent = time.time() + args.wait, 0.0
    while time.time() < deadline:
        if time.time() - last_sent >= BRIDGE_RESEND_S:
            try:
                sock.sendto(data, target)
            except OSError:
                pass
            last_sent = time.time()
        try:
            reply = json.loads(sock.recvfrom(65536)[0].decode("utf-8"))
        except (socket.timeout, OSError, ValueError, UnicodeDecodeError):
            continue
        if reply.get("type") == "declare_result" and reply.get("id") == request_id:
            print(reply.get("text"))
            sys.exit(0 if reply.get("ok") else 1)
    sys.exit(f"Error: no answer from robocommand_bridge.py on 127.0.0.1:{args.bridge_port} - is the bridge running "
             "on this laptop?")


if __name__ == "__main__":
    main()
