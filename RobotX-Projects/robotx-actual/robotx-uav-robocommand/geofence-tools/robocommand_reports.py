#!/usr/bin/env python3
"""
Send a UAV task report to RoboCommand from the Orin.

The report goes to the geofence listener on this Orin (UDP 14600, local
only), which relays it to robocommand_bridge.py on the GCS laptop until the
bridge confirms; the bridge checks it against RoboNation's schema and
publishes it as an RxReport from UAV1. The result shows in the drone GUI's
message log ("RoboCommand: ... sent" / "... REJECTED").

This only sends what it is given - it detects nothing itself. Use it from
the command line, or import send_report() from perception code.

Reports (RobotX 2026 RoboCommand):
  safe-passage       Task 1 Advanced/Disruptive: entry/exit buoys + every
                     detected buoy and its light. Re-send whenever detections change.
  resource-delivery  Tasks 2/3 Advanced/Disruptive: sent by the UAV when it
                     starts a delivery, echoing the colours it was asked for.
  readiness          Task 4: on station after an AssistanceRequest. The listener
                     sends this by itself once the drone holds on the requested
                     position - this is the manual fallback.
  incident-ack       Task 4: acknowledge a command. The bridge sends this by itself
                     for UAV-targeted commands - this is the manual fallback.

Examples:
    python3 robocommand_reports.py safe-passage --entry 1.28101,103.85500 --exit 1.28110,103.85512 \\
        --buoy 1.28101,103.85500,blue --buoy 1.28105,103.85505,red --buoy 1.28110,103.85512,steady-blue
    python3 robocommand_reports.py resource-delivery --task 2 --resource any --circle green
    python3 robocommand_reports.py readiness --command-seq 7
"""

import argparse
import json
import socket
import sys
import time
import uuid

BEACON_STATES = {
    "off": "BEACON_STATE_OFF",
    "red": "BEACON_STATE_FLASHING_RED",
    "green": "BEACON_STATE_FLASHING_GREEN",
    "blue": "BEACON_STATE_FLASHING_BLUE",
    "steady-blue": "BEACON_STATE_STEADY_BLUE",
}
COLORS = {"red": "COLOR_RED", "green": "COLOR_GREEN", "blue": "COLOR_BLUE", "any": "COLOR_ANY"}
DELIVERY_TASKS = {"2": "TASK_INFRA_SURVEY_REPAIR", "3": "TASK_COORDINATED_LOGISTICS"}


def send_report(kind: str, body: dict, host: str = "127.0.0.1", port: int = 14600, timeout_s: float = 3.0) -> bool:
    """Hand a report to the listener. True once the listener has it - the
    drone GUI's log then says whether RoboCommand got it."""
    report_id = f"orin-{uuid.uuid4().hex[:12]}"
    data = json.dumps({"type": "uav_report", "id": report_id, "kind": kind, "body": body}).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    deadline = time.time() + timeout_s
    try:
        while time.time() < deadline:
            sock.sendto(data, (host, port))
            try:
                reply = json.loads(sock.recvfrom(4096)[0].decode("utf-8"))
            except (socket.timeout, ValueError, UnicodeDecodeError, ConnectionResetError):
                continue
            if reply.get("type") == "uav_report_queued" and reply.get("id") == report_id:
                return True
    finally:
        sock.close()
    return False


def _latlon(text: str) -> dict:
    lat, lon = (float(v) for v in text.split(","))
    return {"latitude": lat, "longitude": lon}


def _buoy(text: str) -> dict:
    parts = text.split(",")
    if len(parts) != 3 or parts[2].lower() not in BEACON_STATES:
        raise argparse.ArgumentTypeError(f"buoy must be LAT,LON,STATE with STATE one of {', '.join(BEACON_STATES)}")
    return {"position": _latlon(",".join(parts[:2])), "state": BEACON_STATES[parts[2].lower()]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--listener", default="127.0.0.1:14600", help="geofence listener host:port (default: %(default)s)")
    sub = parser.add_subparsers(dest="report", required=True)

    sp = sub.add_parser("safe-passage", help="Task 1: buoy field with entry/exit")
    sp.add_argument("--entry", type=_latlon, required=True, help="FLASHING_BLUE (entry) buoy LAT,LON")
    sp.add_argument("--exit", type=_latlon, required=True, help="STEADY_BLUE (exit) buoy LAT,LON")
    sp.add_argument("--buoy", type=_buoy, action="append", default=[], help="LAT,LON,STATE - repeat for every detected buoy")

    rd = sub.add_parser("resource-delivery", help="Tasks 2/3: starting a delivery")
    rd.add_argument("--task", choices=sorted(DELIVERY_TASKS), required=True)
    rd.add_argument("--resource", choices=sorted(COLORS), required=True, help="tin colour (any = Advanced tier)")
    rd.add_argument("--circle", choices=["red", "green", "blue"], required=True, help="delivery circle colour")

    for name, help_text in (("readiness", "Task 4: on station (manual fallback)"),
                            ("incident-ack", "Task 4: acknowledge a command (manual fallback)")):
        p = sub.add_parser(name, help=help_text)
        p.add_argument("--command-seq", type=int, required=True, help="seq of the RoboCommand command")

    args = parser.parse_args()
    host, _, port = args.listener.rpartition(":")

    if args.report == "safe-passage":
        kind, body = "safe_passage", {"entry_position": args.entry, "exit_position": args.exit, "buoys": args.buoy}
    elif args.report == "resource-delivery":
        kind, body = "resource_delivery", {"task": DELIVERY_TASKS[args.task], "resource_color": COLORS[args.resource],
                                           "delivery_circle_color": COLORS[args.circle]}
    elif args.report == "readiness":
        kind, body = "readiness", {"command_seq": args.command_seq}
    else:
        kind, body = "incident_ack", {"command_seq": args.command_seq}

    if send_report(kind, body, host or "127.0.0.1", int(port)):
        print(f"{args.report} handed to the listener - the drone GUI's log shows when RoboCommand has it.")
    else:
        sys.exit(f"Error: the geofence listener at {args.listener} didn't answer - is the drone GUI running?")


if __name__ == "__main__":
    main()
