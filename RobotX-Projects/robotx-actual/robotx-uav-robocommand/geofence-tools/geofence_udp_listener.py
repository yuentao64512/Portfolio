#!/usr/bin/env python3
"""
Orin-side listener: receives geofence vertices over plain UDP from
geofence_udp_publisher.py (running on the ground GCS), uploads them to the
flight controller via MAVLink, and verifies the upload by downloading it
back - same upload+verify behavior as geofence_publisher.py, just triggered
by network packets instead of a one-shot CLI run.

Announces what happened to the drone GUI (heartbeat_monitor.py) by sending
it a small UDP notification on --notify (default 127.0.0.1:14650), which the
GUI prints in its message log. This can't go over MAVLink: MAVProxy forwards
what this script sends only to the FCU, never to the GUI's output port, and
ArduPilot doesn't echo it back down the same link. It is ALSO sent as a
MAVLink STATUSTEXT, which ArduPilot passes on to any other link the FCU has
(e.g. a telemetry radio to a ground station).

After each verified upload of fence data - from the publisher (test fence)
or RoboCommand's course - it turns on the polygon fence only: FENCE_TYPE=4
(polygon; no max-altitude or circle fence) and FENCE_ENABLE=1, so the new
polygon is actually being checked by ArduPilot, not just sitting in storage.
FENCE_ALT_MAX is left as it is.

When this listener is stopped or killed (Ctrl+C, SIGTERM - e.g. the drone
GUI quitting - or its terminal closing), it puts FENCE_TYPE, FENCE_ENABLE and
FENCE_ALT_MAX back to the values they had when it started (armed or not).
Only kill -9 can't be caught.

If the publisher goes quiet (--publisher-timeout seconds with nothing
received) or sends an explicit "bye" on clean shutdown, this reverts to the
preset: the polygon from --preset-fence-file, plus FENCE_TYPE and
FENCE_ENABLE restored to the values they had when this listener started.

It also receives RoboCommand (RobotX 2026) messages forwarded by
robocommand_bridge.py on the GCS laptop, on the same port: shown in the
drone GUI's message log and saved to --robocommand-state, see
robocommand_handler.py. They never change anything on the FCU.

This needs its OWN MAVProxy output, separate from whatever port the drone
GUI is using (e.g. if the GUI uses udp:127.0.0.1:14550, add another with
`output add 127.0.0.1:14551` in the MAVProxy console, or an extra --out
flag when launching it) - two processes can't both bind the same UDP port.

Usage:
    python3 geofence_udp_listener.py \\
        --connection udp:127.0.0.1:14551 \\
        --listen-port 14600 \\
        --preset-fence-file preset_fence_example.csv
"""

import argparse
import itertools
import json
import os
import signal
import socket
import sys
import time
from pathlib import Path

os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")  # pymavlink 2.4.50+ otherwise picks "all"

from pymavlink import mavutil  # noqa: E402

from mavlink_param_link import (
    FenceVertex,
    MavlinkLinkError,
    _param_id_str,
    Vehicle,
    connect_and_wait_for_heartbeat,
    download_fence,
    clear_fence,
    download_fence_items,
    get_param,
    load_fence_csv,
    set_param,
    upload_fence_polygon,
)
from course_fence import CourseFenceError, course_fence
from robocommand_handler import RoboCommandHandler
from uav_telemetry import UavTelemetry, request_streams
from heartbeat_link import is_autopilot_heartbeat  # on the path via mavlink_param_link

BRIDGE_TIMEOUT_S = 10.0  # stop sending UAV telemetry when the bridge's hellos stop
EXTERNAL_FENCE_SETTLE_S = 1.0  # after another program (Mission Planner) wrote the fence, read it back this much later
EXTERNAL_FENCE_RETRY_S = 5.0  # ...and again this much later if that read failed
FENCE_POLL_S = 10.0  # while disarmed, re-read the FCU's fence this often to catch writes made over other links
FENCE_ALT_MAX_POLL_S = 10.0  # re-read FENCE_ALT_MAX this often (it is also picked up whenever the FCU announces it)
REPORT_RESEND_S = 0.5  # UAV reports: re-send to the bridge this often until confirmed...
REPORT_RETRY_FOR_S = 30.0  # ...for up to this long

STATUSTEXT_MAX_BYTES = 50  # MAVLink STATUSTEXT's basic text field length
# This listener's own MAVLink component id (MAV_COMP_ID_ONBOARD_COMPUTER), so
# its fence upload/verify only takes the FCU's replies meant for it - not
# those to Mission Planner / MAVProxy reading the fence at the same time.
LISTENER_COMPONENT_ID = 191

# ArduPilot FENCE_TYPE bitmask convention: 1 = max alt, 2 = circle, 4 = polygon.
FENCE_TYPE_POLYGON_BIT = 4
FENCE_TYPE_MAX_ALT_BIT = 1


# Where the drone GUI listens for notifications - set from --notify in main(),
# None when disabled.
_notify_target = None
_notify_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def announce(vehicle: Vehicle, text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_INFO) -> None:
    print(text)
    vehicle.master.mav.statustext_send(severity, text.encode("utf-8")[:STATUSTEXT_MAX_BYTES])
    if _notify_target is not None:
        try:
            _notify_sock.sendto(json.dumps({"severity": severity, "text": text}).encode("utf-8"), _notify_target)
        except OSError:
            pass  # GUI not running - the terminal print above still has it


def notify_gui(text: str, severity: int = mavutil.mavlink.MAV_SEVERITY_INFO) -> None:
    """Like announce(), but only to this terminal and the drone GUI - used for
    RoboCommand messages, which are no business of the FCU."""
    print(text)
    if _notify_target is not None:
        try:
            _notify_sock.sendto(json.dumps({"severity": severity, "text": text}).encode("utf-8"), _notify_target)
        except OSError:
            pass


def _same_fence(a, b, tolerance_deg: float = 1e-6) -> bool:
    """The same polygon (to ~10 cm, like the upload verify), ignoring a repeated closing point."""
    def open_ring(points):
        pts = [(float(lat), float(lon)) for lat, lon in points]
        if len(pts) >= 2 and abs(pts[0][0] - pts[-1][0]) < tolerance_deg and abs(pts[0][1] - pts[-1][1]) < tolerance_deg:
            pts = pts[:-1]
        return pts
    a, b = open_ring(a), open_ring(b)
    return len(a) == len(b) and all(abs(p[0] - q[0]) < tolerance_deg and abs(p[1] - q[1]) < tolerance_deg for p, q in zip(a, b))


def _same_point(a: FenceVertex, b: FenceVertex, tolerance_deg: float = 1e-7) -> bool:
    return abs(a.lat - b.lat) < tolerance_deg and abs(a.lon - b.lon) < tolerance_deg


def apply_fence(
    vehicle: Vehicle,
    vertices,
    inclusion: bool,
    label: str,
    timeout_s: float,
    fence_type: int,
    fence_enable: "int | None",
    fence_alt_max: "float | None" = None,
) -> bool:
    """Upload + verify the polygon, then set FENCE_ALT_MAX (if given),
    FENCE_TYPE, and FENCE_ENABLE (if given). Params are only touched after the
    polygon is verified, so a failed upload never leaves the FCU checking a
    fence it doesn't actually have."""
    ok, msg = upload_fence_polygon(vehicle, vertices, inclusion=inclusion, timeout_s=timeout_s)
    if not ok:
        announce(vehicle, f"Geofence {label}: upload FAILED - {msg}"[:200], severity=mavutil.mavlink.MAV_SEVERITY_ERROR)
        return False

    try:
        downloaded = download_fence(vehicle, timeout_s=timeout_s)
    except MavlinkLinkError as e:
        announce(vehicle, f"Geofence {label}: verify FAILED - {e}"[:200], severity=mavutil.mavlink.MAV_SEVERITY_ERROR)
        return False

    expected = vertices[:-1] if (len(vertices) >= 2 and _same_point(vertices[0], vertices[-1])) else vertices
    mismatch = len(downloaded) != len(expected) or any(
        not _same_point(a, b, tolerance_deg=1e-6) for a, b in zip(expected, downloaded)
    )
    if mismatch:
        # the detail goes to this terminal / the listener log only
        print(f"  expected {len(expected)} vertices, the FCU returned {len(downloaded)}: "
              + "; ".join(f"#{i} sent {a.lat:.7f},{a.lon:.7f} got {b.lat:.7f},{b.lon:.7f}"
                          for i, (a, b) in enumerate(zip(expected, downloaded)) if not _same_point(a, b, tolerance_deg=1e-6)))
        announce(vehicle, f"Geofence {label}: verify MISMATCH after upload", severity=mavutil.mavlink.MAV_SEVERITY_ERROR)
        return False

    # FENCE_ALT_MAX first, so the altitude limit is already right by the time
    # FENCE_TYPE switches its bit on.
    params = [("FENCE_ALT_MAX", fence_alt_max)] if fence_alt_max is not None else []
    params.append(("FENCE_TYPE", fence_type))
    if fence_enable is not None:
        params.append(("FENCE_ENABLE", fence_enable))
    for name, value in params:
        ok, msg, _ = set_param(vehicle, name, value, timeout_s=timeout_s)
        if not ok:
            announce(vehicle, f"Geofence {label}: {name} FAILED - {msg}"[:200], severity=mavutil.mavlink.MAV_SEVERITY_ERROR)
            return False

    # Kept short deliberately: STATUSTEXT's text field is a hard 50 bytes,
    # so this must fit even with the longest label and a two-digit vertex
    # count ("Geofence RESET-TIMEOUT: 99v T=7 EN=1 A=100 OK" is 45). Full detail
    # (if any) is still in the local stdout print inside announce().
    enable_part = f" EN={fence_enable}" if fence_enable is not None else ""
    alt_part = f" A={fence_alt_max:g}" if fence_alt_max is not None else ""
    announce(vehicle, f"Geofence {label}: {len(downloaded)}v T={fence_type}{enable_part}{alt_part} OK")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udp:127.0.0.1:14551", help="MAVLink connection string - must be a DIFFERENT MAVProxy output than the drone GUI's (default: %(default)s)")
    parser.add_argument("--listen-host", default="0.0.0.0", help="address to bind for incoming UDP fence updates (default: %(default)s)")
    parser.add_argument("--listen-port", type=int, default=14600, help="UDP port to bind (default: %(default)s)")
    parser.add_argument("--preset-fence-file", required=True, type=Path, help="CSV fence to restore when the publisher stops (required - no guessed default)")
    parser.add_argument("--publisher-timeout", type=float, default=5.0, help="seconds without a packet before reverting to the preset fence (default: %(default)s)")
    parser.add_argument("--timeout", type=float, default=5.0, help="MAVLink protocol timeout per step (default: %(default)s)")
    parser.add_argument("--notify", default="127.0.0.1:14650", help="host:port of the drone GUI's notification port; empty string disables (default: %(default)s)")
    parser.add_argument("--robocommand-state", type=Path, default=Path(__file__).resolve().parent / "robocommand_state.json",
                        help="where to keep the latest RoboCommand messages from robocommand_bridge.py (default: %(default)s)")
    parser.add_argument("--vehicle-id", default="UAV1", help="this drone's RoboCommand vehicle id (default: %(default)s)")
    parser.add_argument("--no-course-fence", action="store_true",
                        help="don't turn RoboCommand's course into the FCU fence (done by default, while disarmed)")
    parser.add_argument("--course-max-alt", type=float, default=20.0,
                        help="not applied any more - fence data only turns on the polygon fence, FENCE_ALT_MAX is left "
                             "as it is (default: %(default)s)")
    args = parser.parse_args()
    robocommand = RoboCommandHandler(notify_gui, args.robocommand_state, vehicle_id=args.vehicle_id)

    global _notify_target
    if args.notify:
        host, _, port = args.notify.rpartition(":")
        try:
            _notify_target = (host or "127.0.0.1", int(port))
        except ValueError:
            parser.error(f"--notify must look like host:port, got '{args.notify}'")

    try:
        preset_vertices = load_fence_csv(args.preset_fence_file)
    except (OSError, ValueError) as e:
        print(f"Error reading {args.preset_fence_file}: {e}", file=sys.stderr)
        sys.exit(1)
    if len(preset_vertices) < 3:
        print(f"Error: preset fence needs at least 3 vertices, got {len(preset_vertices)}", file=sys.stderr)
        sys.exit(1)

    try:
        vehicle = connect_and_wait_for_heartbeat(args.connection, source_component=LISTENER_COMPONENT_ID)
    except MavlinkLinkError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Connected to system {vehicle.system_id}/component {vehicle.component_id} on {args.connection}")

    # Snapshot the FCU's own fence params now - these are the "preset" values
    # a reset restores. Refuse to run if they can't be read, since a reset
    # without them couldn't actually put things back the way they were.
    try:
        preset_type = int(get_param(vehicle, "FENCE_TYPE", timeout_s=args.timeout))
        preset_enable = int(get_param(vehicle, "FENCE_ENABLE", timeout_s=args.timeout))
        preset_alt_max = get_param(vehicle, "FENCE_ALT_MAX", timeout_s=args.timeout)
    except MavlinkLinkError as e:
        print(f"Error: couldn't read preset FENCE_TYPE/FENCE_ENABLE/FENCE_ALT_MAX from the FCU: {e}", file=sys.stderr)
        sys.exit(1)
    print(f"Preset params captured: FENCE_TYPE={preset_type} FENCE_ENABLE={preset_enable} FENCE_ALT_MAX={preset_alt_max:g}")
    # Kept apart from preset_* (which the course fence replaces): what goes
    # back on the FCU when this listener stops.
    original_params = (("FENCE_ALT_MAX", preset_alt_max), ("FENCE_TYPE", preset_type), ("FENCE_ENABLE", preset_enable))
    params_touched = [False]

    def restore_original_params() -> None:
        if not params_touched[0]:
            return
        for sig in (signal.SIGINT, signal.SIGTERM, getattr(signal, "SIGHUP", None)):
            if sig is not None:
                signal.signal(sig, signal.SIG_IGN)  # a second stop signal must not cut the restore short
        print("Restoring the original fence settings...")
        for name, value in original_params:
            ok, msg, _ = set_param(vehicle, name, value, timeout_s=args.timeout)
            if not ok:
                announce(vehicle, f"Geofence RESTORE: {name} FAILED - {msg}"[:200], severity=mavutil.mavlink.MAV_SEVERITY_ERROR)
                return
        alt_max, fence_type, fence_enable = (value for _, value in original_params)
        announce(vehicle, f"Geofence RESTORED: T={fence_type} EN={fence_enable} A={alt_max:g} OK")

    def on_stop_signal(_signum, _frame):
        raise KeyboardInterrupt  # stop the same way as Ctrl+C, so the settings are restored

    signal.signal(signal.SIGTERM, on_stop_signal)
    if hasattr(signal, "SIGHUP"):
        signal.signal(signal.SIGHUP, on_stop_signal)  # its terminal was closed

    def reset(label: str) -> bool:
        params_touched[0] = True
        ok = apply_fence(vehicle, preset_vertices, True, label, args.timeout, preset_type, preset_enable, preset_alt_max)
        if ok:
            robocommand.fence_changed([(v.lat, v.lon) for v in preset_vertices])
        return ok

    def clear_rejected_fence(reason: str) -> None:
        """The bridge found RoboCommand would reject the Run Request: delete the
        fence from the FCU and put the startup FENCE_* params back."""
        if telemetry.armed:
            notify_gui(f"RoboCommand would REJECT the Run Request ({reason}) - the drone is ARMED, so its fence is "
                       "NOT cleared", mavutil.mavlink.MAV_SEVERITY_WARNING)
            return
        params_touched[0] = True
        ok, msg = clear_fence(vehicle, timeout_s=args.timeout)
        if not ok:
            announce(vehicle, f"Geofence REJECTED-CLEAR: FAILED - {msg}"[:200], severity=mavutil.mavlink.MAV_SEVERITY_ERROR)
            return
        robocommand.fence_changed([])
        for name, value in original_params:
            ok, msg, _ = set_param(vehicle, name, value, timeout_s=args.timeout)
            if not ok:
                announce(vehicle, f"Geofence REJECTED: {name} FAILED - {msg}"[:200], severity=mavutil.mavlink.MAV_SEVERITY_ERROR)
                return
        alt_max, fence_type, fence_enable = (value for _, value in original_params)
        notify_gui(f"RoboCommand would REJECT the Run Request ({reason}) - fence cleared from the FCU, startup params "
                   f"back: FENCE_TYPE={fence_type} FENCE_ENABLE={fence_enable} FENCE_ALT_MAX={alt_max:g}. "
                   "Mission Planner: Plan > Fence > Read to refresh", mavutil.mavlink.MAV_SEVERITY_WARNING)
        announce(vehicle, f"Geofence REJECTED: cleared T={fence_type} EN={fence_enable} OK")

    # What fence the FCU has right now, for RoboCommand's course check - not
    # needed for anything else, so a failed read is just skipped.
    try:
        robocommand.fence_changed([(v.lat, v.lon) for v in download_fence(vehicle, timeout_s=args.timeout)])
    except MavlinkLinkError:
        pass

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.listen_host, args.listen_port))
    sock.settimeout(0.2)  # short, so the UAV telemetry below goes out on time
    print(f"Listening for fence updates on {args.listen_host}:{args.listen_port}")
    print(f"Preset fence ({len(preset_vertices)} vertices) restores after {args.publisher_timeout:g}s of silence, or an explicit 'bye'.\n")

    last_received = None
    last_vertices_key = None
    reverted = True  # no publisher has spoken yet - nothing to revert FROM

    # UAV telemetry for RoboCommand's heartbeat, sent to robocommand_bridge.py
    # once a second while it is around (it says hello every 2 s).
    telemetry = UavTelemetry(vehicle.system_id, is_autopilot_heartbeat)
    request_streams(vehicle.master, vehicle.system_id, vehicle.component_id)
    bridge_addr, bridge_seen, telemetry_sent = None, 0.0, 0.0

    # UAV reports for RoboCommand (ReadinessReport from the on-station check
    # below; SafePassageReport etc. from robocommand_reports.py on this Orin),
    # relayed to the bridge and re-sent until it confirms publishing them.
    relay = {}  # report id -> [datagram, first sent, last sent, label]
    relay_ids = itertools.count(1)

    def queue_report(kind: str, body: dict, label: str, report_id=None) -> None:
        report_id = report_id or f"orin-{int(time.time())}-{next(relay_ids)}"
        if report_id in relay:
            return
        data = json.dumps({"type": "uav_report", "id": report_id, "kind": kind, "body": body}).encode("utf-8")
        relay[report_id] = [data, time.time(), 0.0, label]

    # The fence from RoboCommand's course (course + UAV transit area as one
    # polygon): uploaded only while the drone is disarmed - a course that
    # arrives in flight waits for the landing - and from then on it is the
    # fence a reset returns to. While the laptop's publisher is active, its
    # fence stays and the course fence takes over when it stops.
    pending_course_fence = None
    course_fence_key, course_retry_at, course_wait_noted = None, 0.0, False

    # A fence written by another program (Mission Planner, the GUI's 'fence
    # clear'): seen as the FCU's MISSION_ACK to that program, then read back
    # here. A single inclusion polygon goes to the bridge with the UAV
    # telemetry ("fence_change") until the bridge reports what it did with it
    # - it re-sends the Run Request with that geofence.
    external_fence_at = None
    fence_polled_at = time.time()  # plus a re-read every FENCE_POLL_S, for writes whose reply never reaches here
    fence_change = None
    fence_change_ids = itertools.count(int(time.time()))

    # The FCU's FENCE_ALT_MAX, sent with the UAV telemetry: the bridge builds
    # the Run Request's ceiling from it (ceiling = FENCE_ALT_MAX).
    fence_alt_max, fence_alt_max_read_at = preset_alt_max, time.time()
    rejections_handled = set()  # ids of the bridge's fence_rejected messages already acted on

    try:
        while True:
            if pending_course_fence is not None and time.time() >= course_retry_at:
                fresh = telemetry.heartbeat_time is not None and time.time() - telemetry.heartbeat_time < 3.0
                publisher_active = last_received is not None and not reverted
                course_type = FENCE_TYPE_POLYGON_BIT  # the polygon fence only; FENCE_ALT_MAX left as it is
                if publisher_active:
                    preset_vertices, preset_type, preset_enable, preset_alt_max = pending_course_fence, course_type, 1, None
                    course_fence_key, pending_course_fence = tuple((v.lat, v.lon) for v in preset_vertices), None
                    notify_gui("RoboCommand: course fence ready - it replaces the laptop's fence when that publisher stops")
                elif fresh and not telemetry.armed:
                    params_touched[0] = True
                    if apply_fence(vehicle, pending_course_fence, True, "COURSE", args.timeout, course_type, 1):
                        preset_vertices, preset_type, preset_enable, preset_alt_max = pending_course_fence, course_type, 1, None
                        course_fence_key = tuple((v.lat, v.lon) for v in preset_vertices)
                        robocommand.fence_changed([(v.lat, v.lon) for v in preset_vertices])
                        pending_course_fence, course_wait_noted = None, False
                    else:
                        # Tried ONCE - no retry loop re-writing the fence while you edit it in Mission
                        # Planner. Note the fence the FCU really holds now (the bridge declares that one).
                        course_fence_key = tuple((v.lat, v.lon) for v in pending_course_fence)
                        pending_course_fence, course_wait_noted = None, False
                        try:
                            items = download_fence_items(vehicle, timeout_s=args.timeout)
                            inclusion = [(lat, lon) for c, _p, lat, lon in items
                                         if c == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION]
                            robocommand.fence_changed(inclusion if len(inclusion) == len(items) else [])
                        except MavlinkLinkError:
                            pass
                        notify_gui("RoboCommand: the course fence was tried once and NOT confirmed - set the fence in "
                                   "Mission Planner (Fence > Read to see what the drone has, then Write)",
                                   mavutil.mavlink.MAV_SEVERITY_WARNING)
                elif not course_wait_noted:
                    course_wait_noted = True
                    notify_gui("RoboCommand: new course received while armed (or FCU not heard) - the fence will "
                               "update after landing", mavutil.mavlink.MAV_SEVERITY_WARNING)

            while True:  # take in whatever the FCU sent since the last pass
                mav_msg = vehicle.master.recv_match(blocking=False)
                if mav_msg is None:
                    break
                telemetry.update(mav_msg)
                if (mav_msg.get_type() == "MISSION_ACK" and mav_msg.get_srcSystem() == vehicle.system_id
                        and getattr(mav_msg, "mission_type", 0) == mavutil.mavlink.MAV_MISSION_TYPE_FENCE
                        and mav_msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED
                        and getattr(mav_msg, "target_component", None) != LISTENER_COMPONENT_ID):
                    external_fence_at = time.time() + EXTERNAL_FENCE_SETTLE_S  # the FCU accepted someone else's fence
                if (mav_msg.get_type() == "PARAM_VALUE" and mav_msg.get_srcSystem() == vehicle.system_id
                        and _param_id_str(mav_msg.param_id) == "FENCE_ALT_MAX"):
                    fence_alt_max = float(mav_msg.param_value)  # e.g. set in Mission Planner
            if time.time() - fence_alt_max_read_at >= FENCE_ALT_MAX_POLL_S:
                fence_alt_max_read_at = time.time()
                if telemetry.heartbeat_time is not None and time.time() - telemetry.heartbeat_time < 3.0:
                    try:
                        fence_alt_max = get_param(vehicle, "FENCE_ALT_MAX", timeout_s=1.0)
                    except MavlinkLinkError:
                        pass  # keep the last value; tried again next time
            items, polled = None, False
            if external_fence_at is not None and time.time() >= external_fence_at:
                external_fence_at = None
                try:
                    items = download_fence_items(vehicle, timeout_s=args.timeout)
                except MavlinkLinkError as e:
                    notify_gui(f"Geofence changed by another program, but reading it back failed ({e}) - retrying",
                               mavutil.mavlink.MAV_SEVERITY_WARNING)
                    external_fence_at = time.time() + EXTERNAL_FENCE_RETRY_S
                    items = None
            elif (time.time() - fence_polled_at >= FENCE_POLL_S and pending_course_fence is None and not telemetry.armed
                  and telemetry.heartbeat_time is not None and time.time() - telemetry.heartbeat_time < 3.0):
                # Also re-read the fence now and then: a Mission Planner write over a link that doesn't
                # pass through this MAVProxy (USB, telemetry radio, straight to the FCU) sends no reply here.
                fence_polled_at = time.time()
                try:
                    items, polled = download_fence_items(vehicle, timeout_s=args.timeout), True
                except MavlinkLinkError:
                    pass  # tried again next time
            if items is not None:
                polygons, i = [], 0
                while i < len(items):
                    command, count = items[i][0], max(1, int(items[i][1]))
                    if command == mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                        polygons.append([(lat, lon) for _c, _p, lat, lon in items[i:i + count]])
                        i += count
                    else:
                        i += 1
                if polled and _same_fence(polygons[0] if len(polygons) == 1 else [], robocommand.fence):
                    items = None  # re-read, unchanged - nothing to do
            if items is not None:
                robocommand.fence_changed(polygons[0] if len(polygons) == 1 else [])
                if len(polygons) == 1:
                    fence_change = {"id": next(fence_change_ids), "vertices": [list(p) for p in polygons[0]]}
                    notify_gui(f"Geofence changed by another program (e.g. Mission Planner): {len(polygons[0])}-point "
                               "polygon - sending it to RoboCommand in a new Run Request (through the bridge)")
                else:
                    fence_change = None
                    notify_gui(f"Geofence changed by another program: {len(polygons)} inclusion polygons on the FCU "
                               "- RoboCommand takes exactly one, so no new Run Request is sent",
                               mavutil.mavlink.MAV_SEVERITY_WARNING)
            if bridge_addr is not None and time.time() - bridge_seen < BRIDGE_TIMEOUT_S and time.time() - telemetry_sent >= 1.0:
                report = telemetry.report(args.vehicle_id)
                if report is not None:
                    report["fence_alt_max"] = fence_alt_max
                    report["fcu_fence"] = [list(p) for p in robocommand.fence]  # the bridge declares this polygon
                    if fence_change is not None:
                        report["fence_change"] = fence_change
                    try:
                        sock.sendto(json.dumps(report).encode("utf-8"), bridge_addr)
                    except OSError:
                        pass
                    if report.get("latitude") is not None:
                        for seq in robocommand.readiness_due(report["latitude"], report["longitude"],
                                                             report.get("spd_mps", 0.0),
                                                             report["flight_phase"] == "FLIGHT_PHASE_AIRBORNE"):
                            queue_report("readiness", {"command_seq": seq}, f"ReadinessReport for #{seq}")
                telemetry_sent = time.time()

            for report_id, entry in list(relay.items()):
                if time.time() - entry[1] > REPORT_RETRY_FOR_S:
                    del relay[report_id]
                    notify_gui(f"RoboCommand: {entry[3]} NOT sent - the GCS bridge never confirmed it "
                               "(is robocommand_bridge.py running?)", mavutil.mavlink.MAV_SEVERITY_ERROR)
                elif bridge_addr is not None and time.time() - entry[2] >= REPORT_RESEND_S:
                    entry[2] = time.time()
                    try:
                        sock.sendto(entry[0], bridge_addr)
                    except OSError:
                        pass

            try:
                data, _addr = sock.recvfrom(65536)
            except socket.timeout:
                data = None

            now = time.time()

            if data is not None:
                try:
                    msg = json.loads(data.decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue

                if msg.get("type") == "bye":
                    if not reverted:
                        print("Publisher sent 'bye' - reverting to preset fence...")
                        if reset("RESET-CLOSED"):
                            reverted = True
                    last_received = None
                    last_vertices_key = None
                    continue

                if msg.get("type") == "fence":
                    last_received = now
                    try:
                        vertices = [FenceVertex(lat=p[0], lon=p[1]) for p in msg["vertices"]]
                        # The polygon fence only, switched on: the publisher's
                        # max_alt / fence_type / enable aren't applied.
                        fence_type, fence_enable, fence_alt_max = FENCE_TYPE_POLYGON_BIT, 1, None
                    except (KeyError, TypeError, ValueError):
                        continue
                    inclusion = bool(msg.get("inclusion", True))
                    key = (tuple((v.lat, v.lon) for v in vertices), inclusion, fence_type, fence_enable, fence_alt_max)
                    if key != last_vertices_key:
                        params_touched[0] = True
                        if apply_fence(vehicle, vertices, inclusion, "UPDATE", args.timeout, fence_type, fence_enable, fence_alt_max):
                            last_vertices_key = key
                            robocommand.fence_changed([(v.lat, v.lon) for v in vertices])
                    reverted = False
                    continue

                if msg.get("type") in ("robocommand_hello", "robocommand"):
                    bridge_addr, bridge_seen = _addr, now  # where the UAV telemetry goes

                if msg.get("type") == "robocommand":
                    # From robocommand_bridge.py on the GCS laptop. Acknowledge
                    # every copy (the bridge re-sends until it hears back), act
                    # on each message once. Doesn't count as publisher liveness,
                    # so it falls through to the timeout check below.
                    try:
                        sock.sendto(json.dumps({"type": "robocommand_ack", "id": msg.get("id")}).encode("utf-8"), _addr)
                    except OSError:
                        pass
                    if robocommand.is_new(msg.get("id")):
                        robocommand.handle(msg)
                        if msg.get("kind") == "course" and not args.no_course_fence:
                            body = msg.get("message") or {}
                            try:
                                fence = course_fence(
                                    [(p.get("latitude", 0.0), p.get("longitude", 0.0)) for p in body.get("corners", [])],
                                    [(p.get("latitude", 0.0), p.get("longitude", 0.0)) for p in body.get("uav_transit_area", [])])
                            except CourseFenceError as e:
                                notify_gui(f"RoboCommand: can't make a fence from the course - {e}",
                                           mavutil.mavlink.MAV_SEVERITY_WARNING)
                            else:
                                vertices = [FenceVertex(lat=lat, lon=lon) for lat, lon in fence]
                                # Same course again (bridge restarted): only if the FCU has no fence now
                                # (e.g. after 'fence clear') - a fence edited in Mission Planner stays.
                                if tuple((v.lat, v.lon) for v in vertices) != course_fence_key or not robocommand.fence:
                                    pending_course_fence, course_retry_at, course_wait_noted = vertices, 0.0, False

                if msg.get("type") == "uav_report" and _addr[0] in ("127.0.0.1", "::1"):
                    # From robocommand_reports.py on this Orin (only local
                    # programs may report for the UAV): confirm receipt, relay.
                    body = msg.get("body") if isinstance(msg.get("body"), dict) else {}
                    queue_report(str(msg.get("kind")), body, f"{msg.get('kind')} report", report_id=msg.get("id"))
                    try:
                        sock.sendto(json.dumps({"type": "uav_report_queued", "id": msg.get("id")}).encode("utf-8"), _addr)
                    except OSError:
                        pass

                if msg.get("type") == "fence_rejected":
                    # From the bridge: RoboCommand would reject the Run Request. Acknowledge
                    # every copy (the bridge re-sends until it hears back), act once.
                    try:
                        sock.sendto(json.dumps({"type": "robocommand_ack", "id": msg.get("id")}).encode("utf-8"), _addr)
                    except OSError:
                        pass
                    if msg.get("id") not in rejections_handled:
                        rejections_handled.add(msg.get("id"))
                        clear_rejected_fence(str(msg.get("reason")))

                if msg.get("type") == "bridge_notice":
                    # From the bridge, e.g. the Run Request re-sent for a new FENCE_ALT_MAX.
                    notify_gui(f"RoboCommand: {msg.get('text')}", mavutil.mavlink.MAV_SEVERITY_INFO if msg.get("ok")
                               else mavutil.mavlink.MAV_SEVERITY_ERROR)

                if msg.get("type") == "fence_change_result":
                    # From the bridge: what it did with the fence_change.
                    if fence_change is not None and msg.get("id") == fence_change["id"]:
                        fence_change = None
                        notify_gui(f"RoboCommand: {msg.get('text')}", mavutil.mavlink.MAV_SEVERITY_INFO if msg.get("ok")
                                   else mavutil.mavlink.MAV_SEVERITY_ERROR)

                if msg.get("type") == "uav_report_ack":
                    entry = relay.pop(msg.get("id"), None)
                    if entry is not None:
                        if msg.get("ok"):
                            notify_gui(f"RoboCommand: {entry[3]} sent")
                        else:
                            notify_gui(f"RoboCommand: {entry[3]} REJECTED by the bridge - {msg.get('error')}",
                                       mavutil.mavlink.MAV_SEVERITY_ERROR)

            if last_received is not None and not reverted and (now - last_received) > args.publisher_timeout:
                print(f"No fence update for {args.publisher_timeout:g}s - publisher presumed closed, reverting to preset...")
                if reset("RESET-TIMEOUT"):
                    reverted = True
                last_vertices_key = None
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        restore_original_params()


if __name__ == "__main__":
    main()
