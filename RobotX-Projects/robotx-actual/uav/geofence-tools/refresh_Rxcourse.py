#!/usr/bin/env python3
"""
Ask robocommand_bridge.py to re-send the RxCourse it has cached.

The bridge forwards an RxCourse to the Orin only when its content changes
(course_id, pinger_freq_hz, corners and uav_transit_area; sent_at is ignored),
so a repeat of the retained course does not overwrite a fence you planned by
hand. This script is the explicit way to reapply the cached course anyway,
for example after geofence_udp_listener.py restarted or you cleared the FCU
fence - without restarting the bridge.

The bridge listens on --control-port (default 14616, localhost only). This
sends it a small JSON request and waits for the listener's acknowledgement.

Usage:
    python refresh_Rxcourse.py
    python refresh_Rxcourse.py --host 127.0.0.1 --port 14616 --timeout 20
"""

import argparse
import json
import socket
import sys
import time

from robocommand_bridge import DEFAULT_CONTROL_PORT

RESEND_INTERVAL_S = 0.75


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="bridge address (default: %(default)s)")
    parser.add_argument("--port", type=int, default=DEFAULT_CONTROL_PORT,
                        help="bridge course-reapply UDP port (default: %(default)s)")
    parser.add_argument("--timeout", type=float, default=20.0,
                        help="seconds to wait for the bridge/listener answer (default: %(default)s)")
    args = parser.parse_args()

    request_id = f"refresh-{time.time_ns()}"
    request = json.dumps({"type": "course_reapply", "id": request_id}).encode("utf-8")
    target = (args.host, args.port)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.2)

    deadline, last_sent = time.time() + args.timeout, 0.0
    print(f"Asking the bridge at {args.host}:{args.port} to reapply its cached RxCourse...")
    while time.time() < deadline:
        if time.time() - last_sent >= RESEND_INTERVAL_S:
            try:
                sock.sendto(request, target)
            except OSError as e:
                print(f"  send failed: {e}")
            last_sent = time.time()
        try:
            reply = json.loads(sock.recvfrom(65536)[0].decode("utf-8"))
        except (socket.timeout, OSError, ValueError, UnicodeDecodeError):
            continue
        if reply.get("type") != "course_reapply_result" or reply.get("id") != request_id:
            continue  # not ours (e.g. a late reply to an older request)
        print(reply.get("text") or reply.get("error") or json.dumps(reply))
        sys.exit(0 if reply.get("ok") and reply.get("acked") else 1)

    sys.exit(f"Error: no answer from robocommand_bridge.py on {args.host}:{args.port} - "
             "is the bridge running, with this --control-port?")


if __name__ == "__main__":
    main()
