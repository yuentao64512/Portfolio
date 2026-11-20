#!/usr/bin/env python3
"""
Delete the mission and/or the geofence stored on the flight controller.

The FCU keeps its mission and fence (and Mission Planner shows them) until
they are deleted - closing the drone GUI doesn't touch them. This deletes
them:

  - from the drone GUI: 'quit clear' runs this through MAVProxy just before
    the GUI shuts MAVProxy down;
  - on its own, with the GUI closed: it then talks to the FCU directly on the
    port the FCU sends to (udp 14555 - nothing else may be using it, so close
    the GUI / MAVProxy first).

Refuses while the drone is armed. Only the stored lists are deleted; the
FENCE_* parameters (FENCE_ENABLE, FENCE_TYPE, FENCE_ALT_MAX, ...) are left
as they are. Until a new fence is sent (the fence publisher, or the
RoboCommand course when the GUI starts) the drone has no polygon fence.
Mission Planner keeps showing its last copy until it reads them again
(Plan > Read, and Fence > Read) once it is connected.

Usage:
    python3 fcu_clear.py                    # mission + fence, GUI closed
    python3 fcu_clear.py --what mission     # mission only
    python3 fcu_clear.py --what mission fence rally
"""

import argparse
import os
import socket
import sys
import time

os.environ["MAVLINK20"] = "1"  # mission_type (fence / rally) is a MAVLink 2 field
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")  # before pymavlink loads - see heartbeat_link.py

from pymavlink import mavutil  # noqa: E402

M = mavutil.mavlink
LISTS = {"mission": M.MAV_MISSION_TYPE_MISSION, "fence": M.MAV_MISSION_TYPE_FENCE, "rally": M.MAV_MISSION_TYPE_RALLY}
REPLY_TIMEOUT_S = 3.0
TRIES = 3


def udp_port_in_use(connection: str) -> bool:
    """True if a udp/udpin connection's port is already bound by another
    program. Checked with a plain bind (no SO_REUSEADDR) first, because on
    Linux a second reusable bind can succeed and silently take the FCU's
    packets away from MAVProxy."""
    kind, _, rest = connection.partition(":")
    if kind not in ("udp", "udpin"):
        return False
    host, _, port = rest.rpartition(":")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((host or "0.0.0.0", int(port)))
        return False
    except OSError:
        return True
    finally:
        sock.close()


def wait_autopilot_heartbeat(master, timeout_s: float):
    """The FCU's own heartbeat (not MAVProxy's / a ground station's)."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is not None and msg.type != M.MAV_TYPE_GCS and msg.autopilot != M.MAV_AUTOPILOT_INVALID:
            return msg
    return None


def wait_reply(master, msg_type: str, system_id: int, mission_type: int, timeout_s: float):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type=msg_type, blocking=True, timeout=0.3)
        if msg is not None and msg.get_srcSystem() == system_id and getattr(msg, "mission_type", 0) == mission_type:
            return msg
    return None


def clear_list(master, system_id: int, component_id: int, mission_type: int):
    """(ok, what the FCU said)."""
    for _ in range(TRIES):
        master.mav.mission_clear_all_send(system_id, component_id, mission_type)
        ack = wait_reply(master, "MISSION_ACK", system_id, mission_type, REPLY_TIMEOUT_S)
        if ack is not None:
            entry = M.enums["MAV_MISSION_RESULT"].get(ack.type)
            return ack.type == M.MAV_MISSION_ACCEPTED, entry.name if entry else str(ack.type)
    return False, "no reply"


def stored_count(master, system_id: int, component_id: int, mission_type: int):
    """How many items the FCU now holds in that list, or None (no reply)."""
    for _ in range(TRIES):
        master.mav.mission_request_list_send(system_id, component_id, mission_type)
        msg = wait_reply(master, "MISSION_COUNT", system_id, mission_type, REPLY_TIMEOUT_S)
        if msg is not None:
            # the FCU now expects the items to be fetched - end that cleanly
            master.mav.mission_ack_send(system_id, component_id, M.MAV_MISSION_ACCEPTED, mission_type)
            return msg.count
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udpin:0.0.0.0:14555",
                        help="MAVLink connection (default: %(default)s = straight from the FCU, GUI closed; "
                             "the GUI uses its spare MAVProxy output udp:127.0.0.1:14552)")
    parser.add_argument("--what", nargs="+", choices=sorted(LISTS), default=["mission", "fence"],
                        help="what to delete (default: mission fence)")
    parser.add_argument("--timeout", type=float, default=10.0, help="seconds to wait for the FCU heartbeat (default: %(default)s)")
    args = parser.parse_args()

    busy_hint = ("Is the drone GUI (or MAVProxy) still running? Close it first, or use 'quit clear' in the GUI."
                 if "14555" in args.connection else "Another program (e.g. the lawnmower mission) is using it.")
    if udp_port_in_use(args.connection):
        print(f"Can't open {args.connection}: the port is already in use - nothing was deleted.")
        print(busy_hint)
        return 1
    try:
        master = mavutil.mavlink_connection(args.connection, source_system=255, source_component=190)
    except OSError as e:
        print(f"Can't open {args.connection}: {e}")
        print(busy_hint)
        return 1

    print(f"Waiting for the FCU on {args.connection}...")
    heartbeat = wait_autopilot_heartbeat(master, args.timeout)
    if heartbeat is None:
        print("No heartbeat from the FCU - nothing was deleted.")
        return 1
    system_id, component_id = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
    if heartbeat.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
        print("The drone is ARMED - nothing was deleted. Land and disarm first.")
        return 2

    all_ok = True
    for name in args.what:
        mission_type = LISTS[name]
        ok, result = clear_list(master, system_id, component_id, mission_type)
        if not ok:
            all_ok = False
            print(f"  {name}: NOT deleted ({result})")
            continue
        count = stored_count(master, system_id, component_id, mission_type)
        if count is None:
            print(f"  {name}: deleted (the FCU accepted it; couldn't read back the count)")
        elif count == 0:
            print(f"  {name}: deleted (0 items left)")
        elif name == "mission" and count == 1:
            print(f"  {name}: deleted (1 item left - the home position, which ArduPilot always keeps)")
        else:
            all_ok = False
            print(f"  {name}: the FCU accepted the delete but still reports {count} items")
    if "fence" in args.what and all_ok:
        print("The FCU has no polygon fence now (FENCE_* parameters unchanged) - send a new fence before flying a mission.")
    print("Mission Planner keeps showing its last copy until it reads again (Plan > Read, Fence > Read).")
    master.close()
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())
