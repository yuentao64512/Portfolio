#!/usr/bin/env python3
"""
Lawnmower mission inside the geofence that is on the flight controller.

What it does, in order:
  1. Connects through the drone GUI's heartbeat code (heartbeat_link.py's
     HeartbeatMonitor) and waits for the FCU's heartbeat.
  2. Reads the fence back FROM THE FCU (the one geofence_udp_listener.py
     uploaded) - not from a file - and checks it is switched on.
  3. Plans a lawnmower path inside the fence, a safety margin in from every
     edge, starting from where the drone is sitting (home).
  4. Uploads it to the FCU as a normal ArduPilot mission and reads it back to
     verify, so Mission Planner shows the path (Plan tab > Read):
        take off -> hover --hover s above home -> lawnmower legs at --speed
        -> back to home -> land
     After it lands: back to GUIDED and the waypoints are cleared.
  5. Checks the RC flight-mode switch is in GUIDED, arms, takes off to
     --altitude in GUIDED, then switches to AUTO. From here ArduPilot flies
     the whole mission itself; this script only watches and reports.

ArduPilot and the pilot stay in charge the whole time:
  - Flip the RC mode switch (or change mode in Mission Planner) at any
    moment and this script stops immediately - it never changes the mode
    back, never overrides RC, never forces arming past ArduPilot's checks.
  - If this script or the Orin dies after AUTO, the FCU finishes the mission
    on its own; the fence, failsafes and RC takeover all still work.
  - It refuses to fly unless the fence is on, the RC is on and its switch is
    in GUIDED, and the switch has another (non GUIDED/AUTO) mode to take
    over with.

Usage:
    python3 lawnmower_mission.py --upload-only   # plan + upload, no flying
    python3 lawnmower_mission.py                 # fly it (asks to confirm)
"""

import os

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")  # pymavlink 2.4.50+ otherwise picks "all"

import argparse
import json
import signal
import socket
import sys
import time

from pymavlink import mavutil

from lawnmower_planner import PlanError, plan_lawnmower
from mission_link import MavlinkLinkError, MissionItem, MissionLink, fence_zones_from_items

SEV_INFO = mavutil.mavlink.MAV_SEVERITY_INFO
SEV_WARNING = mavutil.mavlink.MAV_SEVERITY_WARNING
SEV_ERROR = mavutil.mavlink.MAV_SEVERITY_ERROR

FENCE_TYPE_MAX_ALT_BIT = 1
FENCE_TYPE_CIRCLE_BIT = 2  # ArduPilot's own circle, FENCE_RADIUS around home
FENCE_TYPE_POLYGON_BIT = 4
ALT_HEADROOM_M = 2.0  # mission altitude must be at least this far under FENCE_ALT_MAX

# Mission layout (item numbers), see build_mission().
HOME_SEQ, TAKEOFF_SEQ, HOVER_SEQ, SPEED_SEQ, FIRST_WP_SEQ = 0, 1, 2, 3, 4

# ArduPilot's flight-mode switch PWM bands -> FLTMODE1..6.
FLTMODE_PWM_UPPER = [1230, 1360, 1490, 1620, 1749]
# The RC mode switch is watched directly (RC_CHANNELS, ~10 Hz) as well as the
# flight mode (HEARTBEAT, ~1 Hz), so a takeover is seen at once. A reading
# older than this isn't used - the FCU's own RC failsafe covers a lost RC.
RC_FRESH_S = 1.0

FRAME_RELATIVE = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT


class MissionAbort(RuntimeError):
    """Stop before (or instead of) handing the flight to ArduPilot."""


class Handover(Exception):
    """The pilot / Mission Planner / ArduPilot took control - stand down."""


class Notifier:
    """Prints, and also sends each line to the drone GUI's message log
    (same UDP JSON format geofence_udp_listener.py uses)."""

    def __init__(self, target: str):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._target = None
        if target:
            host, _, port = target.rpartition(":")
            self._target = (host or "127.0.0.1", int(port))

    def __call__(self, text: str, severity: int = SEV_INFO) -> None:
        print(text, flush=True)
        if self._target is not None:
            try:
                self._sock.sendto(json.dumps({"severity": severity, "text": text}).encode("utf-8"), self._target)
            except OSError:
                pass  # GUI not running - the print above still has it


def result_name(result) -> str:
    if result is None:
        return "no reply"
    entry = mavutil.mavlink.enums["MAV_RESULT"].get(result)
    return entry.name.replace("MAV_RESULT_", "") if entry else str(result)


def flight_mode_slot(pwm: int) -> int:
    for slot, upper in enumerate(FLTMODE_PWM_UPPER, start=1):
        if pwm <= upper:
            return slot
    return 6


def build_mission(plan, home, altitude_m: float, speed_ms: float, hover_s: float, hover_altitude_m: "float | None" = None):
    """The mission items, plus {item number: progress message} for the
    items worth announcing in the GUI log. hover_altitude_m: take off and
    hover at this height instead (the legs stay at altitude_m)."""
    home_lat, home_lon = home
    hover_alt = hover_altitude_m or altitude_m
    items = [
        MissionItem(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, mavutil.mavlink.MAV_FRAME_GLOBAL, lat=home_lat, lon=home_lon),
        MissionItem(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, FRAME_RELATIVE, alt=hover_alt),
        MissionItem(mavutil.mavlink.MAV_CMD_NAV_LOITER_TIME, FRAME_RELATIVE, param1=hover_s, lat=home_lat, lon=home_lon, alt=hover_alt),
        # param1 1 = ground speed; param3 -1 = leave throttle alone
        MissionItem(mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, FRAME_RELATIVE, param1=1, param2=speed_ms, param3=-1),
    ]
    messages = {HOVER_SEQ: f"Mission: AUTO - hovering {hover_s:g} s above home at {hover_alt:g} m"}
    for wp in plan.waypoints:
        if wp.kind == "leg_start":
            messages[len(items)] = f"Mission: lawnmower leg {wp.leg}/{plan.leg_count}"
        items.append(MissionItem(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, FRAME_RELATIVE, lat=wp.lat, lon=wp.lon, alt=altitude_m))
    # The first item after the last leg ends: a corner hop on the way home, or home itself.
    last_leg_end = max(i for i, wp in enumerate(plan.waypoints) if wp.kind == "leg_end")
    messages[FIRST_WP_SEQ + last_leg_end + 1] = "Mission: lawnmower done - returning home"
    items.append(MissionItem(mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, FRAME_RELATIVE, lat=home_lat, lon=home_lon, alt=altitude_m))
    messages[len(items)] = "Mission: landing at home"
    items.append(MissionItem(mavutil.mavlink.MAV_CMD_NAV_LAND, FRAME_RELATIVE, lat=home_lat, lon=home_lon))
    return items, messages


def check_fence(link: MissionLink, args, say):
    """Read the fence off the FCU. Returns (zones, FENCE_TYPE, FENCE_RADIUS);
    raises (or, for --upload-only, just warns) if it wouldn't actually
    protect this flight."""
    fence_enable = int(link.get_param("FENCE_ENABLE"))
    fence_type = int(link.get_param("FENCE_TYPE"))
    fence_alt_max = link.get_param("FENCE_ALT_MAX")
    fence_action = int(link.get_param("FENCE_ACTION"))
    fence_radius = link.get_param("FENCE_RADIUS") if fence_type & FENCE_TYPE_CIRCLE_BIT else 0.0
    zones = fence_zones_from_items(link.download_items(mavutil.mavlink.MAV_MISSION_TYPE_FENCE))
    circle_note = f", {fence_radius:g} m circle around home" if fence_radius > 0 else ""
    say(f"Mission: FCU fence: {zones.describe()}{circle_note} - FENCE_ENABLE={fence_enable} FENCE_TYPE={fence_type} "
        f"FENCE_ALT_MAX={fence_alt_max:g} m FENCE_ACTION={fence_action}")

    problems = []
    if fence_enable != 1:
        problems.append("the fence isn't switched on (FENCE_ENABLE=0) - publish it with --enable-fence")
    if not fence_type & FENCE_TYPE_POLYGON_BIT:
        problems.append(f"FENCE_TYPE={fence_type} doesn't include polygons/zones (4), so ArduPilot isn't checking them")
    if fence_type & FENCE_TYPE_MAX_ALT_BIT:
        if args.altitude > fence_alt_max - ALT_HEADROOM_M:
            problems.append(f"altitude {args.altitude:g} m is too close to FENCE_ALT_MAX {fence_alt_max:g} m")
    else:
        say("Mission: warning - the max-altitude fence is off (FENCE_TYPE has no 1 bit)", SEV_WARNING)
    if fence_action == 0:
        say("Mission: warning - FENCE_ACTION=0, a fence breach is only reported, nothing is done about it", SEV_WARNING)

    for problem in problems:
        if args.upload_only:
            say(f"Mission: warning - {problem} (fine for a preview, not for flying)", SEV_WARNING)
        else:
            raise MissionAbort(problem)
    return zones, fence_type, fence_radius


def wait_for_position(link: MissionLink, timeout_s: float = 5.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        pos = link.position()
        if pos is not None and pos.age_s < 2.0 and (pos.lat, pos.lon) != (0.0, 0.0):
            return pos
        time.sleep(0.2)
    raise MissionAbort("no GPS position from the FCU (GLOBAL_POSITION_INT) - wait for a GPS fix")


def check_preflight(link: MissionLink, mapping: dict) -> list:
    """Everything that must be true before arming. Raises MissionAbort.
    Returns the RC switch's takeover modes, for the go-ahead message."""
    snap = link.monitor.snapshot()
    telemetry = link.monitor.telemetry_snapshot()
    names = {number: name for name, number in mapping.items()}

    if snap.armed:
        raise MissionAbort("the drone is already armed - land and disarm before starting a mission")
    if snap.mode_name != "GUIDED":
        raise MissionAbort(f"flight mode is {snap.mode_name} - flip the RC mode switch to GUIDED first")

    channel = int(link.get_param("FLTMODE_CH"))
    if channel <= 0:
        raise MissionAbort("FLTMODE_CH=0: no RC mode switch is set up, so there'd be no RC takeover")
    pwm = telemetry.rc_channels[channel - 1] if len(telemetry.rc_channels) >= channel else None
    if pwm is None or telemetry.rc_age is None or telemetry.rc_age > 2.0:
        raise MissionAbort(f"no RC input on channel {channel} - the RC must be on (it's how you take over)")
    slot_modes = {slot: int(link.get_param(f"FLTMODE{slot}")) for slot in range(1, 7)}
    slot = flight_mode_slot(pwm)
    if slot_modes[slot] != mapping["GUIDED"]:
        raise MissionAbort(
            f"the RC mode switch is in position {slot} ({names.get(slot_modes[slot], slot_modes[slot])}), "
            "not GUIDED - flip it to GUIDED"
        )
    takeover = sorted({names.get(m, str(m)) for m in slot_modes.values()} - {"GUIDED", "AUTO"})
    if not takeover:
        raise MissionAbort("every RC mode-switch position is GUIDED or AUTO - set one to e.g. LOITER for takeover")

    if telemetry.gps is None or telemetry.gps.fix_type < 3:
        raise MissionAbort("GPS has no 3D fix")
    return takeover


class RcSwitch:
    """The RC flight-mode switch: its channel, the position the mission
    started from (GUIDED), and each position's mode name."""

    def __init__(self, channel: int, slot: int, slot_names: dict):
        self.channel = channel
        self.slot = slot
        self.slot_names = slot_names


def read_rc_switch(link: MissionLink, mapping: dict) -> RcSwitch:
    """Where the RC mode switch is now - after check_preflight, which has
    already made sure there is one and that it's on GUIDED."""
    names = {number: name for name, number in mapping.items()}
    channel = int(link.get_param("FLTMODE_CH"))
    slot_names = {slot: names.get(int(link.get_param(f"FLTMODE{slot}")), "?") for slot in range(1, 7)}
    telemetry = link.monitor.telemetry_snapshot()
    pwm = telemetry.rc_channels[channel - 1] if len(telemetry.rc_channels) >= channel else None
    if pwm is None:
        raise MissionAbort(f"no RC input on channel {channel} - the RC must be on (it's how you take over)")
    slot = flight_mode_slot(pwm)
    if slot_names[slot] != "GUIDED":
        raise Handover(f"RC mode switch moved to position {slot} ({slot_names[slot]})")
    return RcSwitch(channel, slot, slot_names)


def switch_moved(link: MissionLink, switch: "RcSwitch | None") -> "str | None":
    """What happened, if the RC mode switch has left the mission's position."""
    if switch is None:
        return None
    telemetry = link.monitor.telemetry_snapshot()
    if telemetry.rc_age is None or telemetry.rc_age > RC_FRESH_S or len(telemetry.rc_channels) < switch.channel:
        return None
    pwm = telemetry.rc_channels[switch.channel - 1]
    if pwm is None:
        return None
    slot = flight_mode_slot(pwm)
    if slot == switch.slot:
        return None
    return f"RC mode switch moved to position {slot} ({switch.slot_names.get(slot, '?')})"


def guard(link: MissionLink, expected_mode: str, switch: "RcSwitch | None" = None, require_armed: bool = True) -> None:
    """Raise Handover the moment anyone else takes control."""
    snap = link.monitor.snapshot()
    if not snap.connected:
        raise MissionAbort("lost the FCU heartbeat - the drone holds in GUIDED; take over with the RC")
    moved = switch_moved(link, switch)
    if moved:
        raise Handover(moved)
    if require_armed and not snap.armed:
        raise Handover("the drone disarmed")
    if snap.mode_name != expected_mode:
        raise Handover(f"mode changed to {snap.mode_name}")


def wait_until(predicate, timeout_s: float, link: MissionLink, expected_mode: str, poll_s: float = 0.2,
               switch: "RcSwitch | None" = None) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if predicate():
            return True
        guard(link, expected_mode, switch)
        time.sleep(poll_s)
    return predicate()


def finish_mission(link: MissionLink, say, switch: "RcSwitch | None" = None) -> None:
    """After a COMPLETE mission (landed, disarmed): AUTO -> GUIDED, then the
    waypoints deleted from the FCU (the fence stays). The mode is left alone
    if the pilot / Mission Planner already changed it."""
    moved = switch_moved(link, switch)
    mode = link.monitor.snapshot().mode_name
    if moved:
        say(f"Mission: {moved} - flight mode left as it is", SEV_WARNING)
    elif mode != "AUTO":
        say(f"Mission: mode is already {mode} - left as it is")
    else:
        ok, message = link.monitor.send_mode("GUIDED")
        deadline = time.time() + 5
        while ok and link.monitor.snapshot().mode_name != "GUIDED" and time.time() < deadline:
            time.sleep(0.2)
        mode = link.monitor.snapshot().mode_name
        if mode == "GUIDED":
            say("Mission: mode changed AUTO -> GUIDED")
        else:
            say(f"Mission: couldn't change to GUIDED ({message}) - still {mode}", SEV_WARNING)
    try:
        link.clear_mission()
        say("Mission: waypoints cleared from the FCU (fence kept) - Mission Planner: Plan > Read to refresh")
    except MavlinkLinkError as e:
        say(f"Mission: couldn't clear the waypoints: {e}", SEV_WARNING)


def watch_mission(link: MissionLink, messages: dict, land_seq: int, say, switch: "RcSwitch | None" = None) -> int:
    """After AUTO: report progress until landed, or stand down on takeover."""
    last_seq = None
    lost_since = None
    while True:
        time.sleep(0.5)
        snap = link.monitor.snapshot()
        if not snap.connected:
            if lost_since is None:
                lost_since = time.time()
                say("Mission: lost the FCU heartbeat - ArduPilot keeps flying the mission on its own; "
                    "take over with the RC if needed", SEV_WARNING)
            elif time.time() - lost_since > 120:
                say("Mission: no heartbeat for 2 min - script exiting (the FCU is still in charge)", SEV_ERROR)
                return 1
            continue
        if lost_since is not None:
            say("Mission: heartbeat back")
            lost_since = None

        if not snap.armed:
            if last_seq is not None and last_seq >= land_seq:
                say("Mission: COMPLETE - landed at home and disarmed")
                finish_mission(link, say, switch)
                return 0
            say("Mission: the drone disarmed before the mission finished - script stopped", SEV_WARNING)
            return 0
        moved = switch_moved(link, switch)
        if moved:
            say(f"Mission: {moved} - script stopped, you have control", SEV_WARNING)
            return 0
        if snap.mode_name != "AUTO":
            say(f"Mission: mode changed to {snap.mode_name} (RC, Mission Planner or a failsafe) - "
                "script stopped, you have control", SEV_WARNING)
            return 0

        seq = link.mission_current()
        if seq is not None and seq != last_seq:
            last_seq = seq
            if seq in messages:
                say(messages[seq])


def run(link: MissionLink, args, say) -> int:
    say(f"Mission: connecting through the heartbeat monitor on {args.connection}...")
    vehicle = link.connect()
    snap = link.monitor.snapshot()
    mapping = mavutil.mode_mapping_byname(snap.vehicle_type) or {}
    if "GUIDED" not in mapping or "AUTO" not in mapping:
        raise MissionAbort("this vehicle type has no GUIDED/AUTO modes - is it ArduCopter?")
    say(f"Mission: heartbeat OK from system {vehicle.system_id}, mode {snap.mode_name}, "
        f"{'ARMED' if snap.armed else 'disarmed'}")

    zones, fence_type, fence_radius = check_fence(link, args, say)
    pos = wait_for_position(link)
    if fence_radius > 0:
        # ArduPilot's own circle fence is centred on home - where the drone is now.
        zones.inclusion_circles.append(((pos.lat, pos.lon), fence_radius))
    plan = plan_lawnmower(zones, (pos.lat, pos.lon), spacing_m=args.spacing, margin_m=args.margin,
                          sweep_bearing_deg=args.sweep_bearing)
    for warning in plan.warnings:
        say(f"Mission: warning - {warning}", SEV_WARNING)
    flight_s = args.hover + plan.path_length_m / args.speed + args.altitude * 3
    say(f"Mission: planned {plan.leg_count} legs, {plan.spacing_m:.1f} m apart, along bearing "
        f"{plan.sweep_bearing_deg:.0f} deg, {args.margin:g} m inside the fence - {plan.path_length_m:.0f} m, "
        f"about {flight_s / 60:.1f} min")

    items, messages = build_mission(plan, (pos.lat, pos.lon), args.altitude, args.speed, args.hover, args.hover_altitude)
    link.upload_mission(items)
    link.verify_mission(items)
    say(f"Mission: uploaded and verified ({len(items)} items) - see it in Mission Planner: Plan tab > Read")
    if args.upload_only:
        return 0

    takeover_modes = check_preflight(link, mapping)
    switch = read_rc_switch(link, mapping)
    say(f"Mission: preflight OK - RC switch in GUIDED; take over anytime with the RC "
        f"({', '.join(takeover_modes)}) or Mission Planner")
    if not args.yes:
        if not sys.stdin.isatty():
            raise MissionAbort("not confirmed - pass --yes (the drone GUI does this after you confirm there)")
        if input("Type 'fly' to arm and start the mission: ").strip().lower() != "fly":
            raise MissionAbort("not confirmed")

    # --- GUIDED: arm and take off ------------------------------------------
    # Before every command it sends, the script checks the pilot hasn't taken
    # over (RC switch / mode), so it never overrides an RC takeover.
    guard(link, "GUIDED", switch, require_armed=False)
    result = link.command_long(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        texts = link.recent_statustexts()
        raise MissionAbort(f"ArduPilot refused to arm ({result_name(result)})" + (f": {texts[-1]}" if texts else ""))
    deadline = time.time() + 5
    while not link.monitor.snapshot().armed:
        if time.time() > deadline:
            raise MissionAbort("arm was accepted but the drone never reported armed")
        guard(link, "GUIDED", switch, require_armed=False)
        time.sleep(0.1)
    say("Mission: ARMED")

    takeoff_alt = args.hover_altitude or args.altitude  # the hover height (the legs climb to --altitude in AUTO)
    guard(link, "GUIDED", switch)
    result = link.command_long(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, takeoff_alt)
    if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        # Never send a disarm here: with no reply ("lost" ack) the drone may
        # already be climbing, and a disarm in the air stops the motors. On the
        # ground ArduPilot disarms by itself after DISARM_DELAY.
        raise MissionAbort(f"ArduPilot didn't accept the takeoff ({result_name(result)}) - if the drone is still on "
                           "the ground ArduPilot disarms it by itself; otherwise take over with the RC")
    say(f"Mission: taking off to {takeoff_alt:g} m")

    def high_enough():
        p = link.position()
        return p is not None and p.relative_alt_m >= takeoff_alt * 0.9

    if not wait_until(high_enough, args.takeoff_timeout, link, "GUIDED", switch=switch):
        raise MissionAbort(f"didn't reach {takeoff_alt:g} m within {args.takeoff_timeout:g} s - "
                           "the drone holds in GUIDED; take over with the RC")

    # --- hand over to ArduPilot: AUTO from the hover item -------------------
    guard(link, "GUIDED", switch)
    link.set_current_item(HOVER_SEQ)
    wait_until(lambda: link.mission_current() == HOVER_SEQ, 3.0, link, "GUIDED", switch=switch)
    guard(link, "GUIDED", switch)  # last check right before AUTO - never switch to AUTO over a takeover
    ok, message = link.monitor.send_mode("AUTO")
    if not ok or not wait_until(lambda: link.monitor.snapshot().mode_name == "AUTO", 5.0, link, "GUIDED", switch=switch):
        raise MissionAbort(f"couldn't switch to AUTO ({message}) - the drone holds in GUIDED; take over with the RC")
    say("Mission: handed to ArduPilot (AUTO) - it flies the rest itself")
    return watch_mission(link, messages, len(items) - 1, say, switch)


def main(defaults: "dict | None" = None):
    """defaults: other option defaults, e.g. {"altitude": 10.0, "speed": 5.0}
    from mission2.py / mission3.py. Anything typed on the command line wins."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--connection", default="udp:127.0.0.1:14552", help="MAVProxy output to use (default: %(default)s)")
    parser.add_argument("--altitude", type=float, default=5.0, help="mission altitude above home, m (default: %(default)s)")
    parser.add_argument("--speed", type=float, default=3.0, help="ground speed along the legs, m/s (default: %(default)s)")
    parser.add_argument("--hover", type=float, default=30.0, help="seconds to hover after take-off (default: %(default)s)")
    parser.add_argument("--hover-altitude", type=float, default=None,
                        help="take off and hover at this height above home, m, then fly the legs at --altitude "
                        "(default: --altitude)")
    parser.add_argument("--spacing", type=float, default=4.0, help="max distance between legs, m (default: %(default)s)")
    parser.add_argument("--margin", type=float, default=3.0, help="stay this far inside the fence, m (default: %(default)s)")
    parser.add_argument("--sweep-bearing", type=float, default=None,
                        help="compass bearing the legs run along, deg (default: along the fence's longest edge)")
    parser.add_argument("--takeoff-timeout", type=float, default=30.0, help="seconds allowed to reach altitude (default: %(default)s)")
    parser.add_argument("--upload-only", action="store_true", help="plan and upload the mission, but don't arm or fly")
    parser.add_argument("--yes", action="store_true", help="don't ask for confirmation before arming")
    parser.add_argument("--notify", default="127.0.0.1:14650", help="drone GUI notification port, empty = off (default: %(default)s)")
    if defaults:
        parser.set_defaults(**defaults)
    args = parser.parse_args()
    if args.altitude <= 0 or args.speed <= 0 or args.hover < 0:
        parser.error("--altitude and --speed must be positive, --hover can't be negative")
    if args.hover_altitude is not None and not 0 < args.hover_altitude <= args.altitude:
        parser.error("--hover-altitude must be positive and at most --altitude")

    say = Notifier(args.notify)
    link = MissionLink(args.connection)

    def on_sigterm(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_sigterm)

    code = 1
    try:
        code = run(link, args, say)
    except Handover as e:
        say(f"Mission: {e} - script stopped, you have control", SEV_WARNING)
        code = 0
    except (MissionAbort, MavlinkLinkError, PlanError) as e:
        say(f"Mission ABORTED: {e}", SEV_ERROR)
    except KeyboardInterrupt:
        mode = link.monitor.snapshot().mode_name
        say(f"Mission script stopped - the drone was left in {mode}; take over with the RC if needed", SEV_WARNING)
    finally:
        link.close()
    sys.exit(code)


if __name__ == "__main__":
    main()
