#!/usr/bin/env python3
"""
Task 4 mission: flies the same mission as runstart_mission.py (5 m, hover 15 s,
lawnmower at 10 m and 3 m/s, home, land) and also reacts by itself to what
RoboCommand sends the UAV during it:

  AssistanceRequest (Core): flies to the position and holds there. The listener
    sends the ReadinessReport by itself once the drone has held on station.
    After RoboCommand's ReadinessConfirm (or CONFIRM_TIMEOUT_S without one) it
    goes back to finish the survey.
  KeepOutZone (Advanced): if the drone's route comes too close, it stops and
    flies a new route that stays --margin metres clear. The blocked parts of
    the survey are skipped, and flown after the All Clear. If home is in the
    zone, it holds outside until the All Clear.
  AllClear (Advanced): flies whatever the zone had blocked.
  MovingObjectAlert (Disruptive) is NOT handled: only shown in the GUI log.

The commands come from robocommand_state.json, which the listener keeps. The
IncidentAck and ReadinessReport are already sent by the bridge and the listener.

ArduPilot and the pilot stay in charge exactly as in the other missions: flip
the RC mode switch (or change mode in Mission Planner) at any moment and this
script stops immediately. To change route it goes AUTO -> GUIDED (the drone
stops and holds), uploads and verifies the new mission, then goes back to AUTO.
If anything fails there the drone keeps holding in GUIDED and the script stops.
It never sends RC overrides or position commands, only mode changes and
missions. A separate thread watches the RC switch and the mode all the time and
remembers a takeover, so even flipping the switch away and straight back while
a new route is being worked out still stops the script for good.

Also sends the CV start signal (see README.md), "mission": "task4", with the
extra phase "assist" while it is dealing with an assistance request.

Usage (every lawnmower_mission.py option works, and overrides these):
    python3 task4_mission.py --upload-only   # plan + upload, no flying
    python3 task4_mission.py                 # fly it (asks to confirm)
"""

import json
import math
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lawnmower_mission as lm  # noqa: E402
import runstart_mission  # noqa: E402
from lawnmower_planner import PlanError  # noqa: E402
from mission_link import MavlinkLinkError, MissionItem  # noqa: E402
from task4_route import ASSIST_MIN_CLEARANCE_M, Piece, RouteMap  # noqa: E402

HOVER_ALTITUDE_M = 5.0
HOVER_S = 15.0
ALTITUDE_M = 10.0
SPEED_MS = 3.0

STATE_PATH = Path(__file__).resolve().parent.parent / "geofence-tools" / "robocommand_state.json"
UAV_TYPE = "TYPE_UAV"
UAV_VEHICLE_ID = "SGIT-53577"  # the id ReadinessConfirm names (robocommand_bridge.py's DEFAULT_VEHICLE_ID)

ON_STATION_M = 3.0  # same as robocommand_handler.py's READY_RADIUS_M
CONFIRM_TIMEOUT_S = 60.0  # on station this long with no ReadinessConfirm: carry on with the survey
LOW_LANDING_M = 3.0  # already landing and this low: too late for an assistance request
TAKEOVER_POLL_S = 0.05
LOITER_UNLIM = lm.mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM
NAV_WAYPOINT = lm.mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
NAV_LAND = lm.mavutil.mavlink.MAV_CMD_NAV_LAND
DO_CHANGE_SPEED = lm.mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED

_started_at = time.time()
_flight = {}  # what lawnmower_mission.run() planned, kept by the two wrappers below
_plan_lawnmower, _build_mission = lm.plan_lawnmower, lm.build_mission


def _plan_and_keep(fence, home, spacing_m, margin_m, sweep_bearing_deg):
    _flight.update(zones=fence, margin=margin_m)
    return _plan_lawnmower(fence, home, spacing_m=spacing_m, margin_m=margin_m, sweep_bearing_deg=sweep_bearing_deg)


def _build_and_keep(plan, home, altitude_m, speed_ms, hover_s, hover_altitude_m=None):
    items, messages = _build_mission(plan, home, altitude_m, speed_ms, hover_s, hover_altitude_m)
    _flight.update(plan=plan, home=home, altitude=altitude_m, speed=speed_ms, items=items)
    return items, messages


def read_state() -> "dict | None":
    try:
        return json.loads(STATE_PATH.read_text())
    except (OSError, ValueError):
        return None  # not there yet - the listener writes it atomically


def _distance_m(a, b) -> float:
    return math.hypot(math.radians(b[0] - a[0]) * 6378137.0,
                      math.radians(b[1] - a[1]) * 6378137.0 * math.cos(math.radians(a[0])))


class Task4:
    def __init__(self, link, messages, land_seq, say, switch):
        self.link, self.messages, self.land_seq, self.say, self.switch = link, messages, land_seq, say, switch
        self.items = _flight["items"]
        self.home, self.altitude, self.speed = _flight["home"], _flight["altitude"], _flight["speed"]
        self.pieces, self.leg_ends = [], {}  # stretches of survey still to fly; item number of each one's end -> stretch
        previous = None
        for i, wp in enumerate(_flight["plan"].waypoints):
            if wp.kind == "leg_end":
                piece = Piece((previous.lat, previous.lon), (wp.lat, wp.lon))
                self.pieces.append(piece)
                self.leg_ends[lm.FIRST_WP_SEQ + i] = piece
            previous = wp
        self.keep_outs = []  # [(centre, radius m)] active for the UAV
        self.seen_requests = set()
        self.requests = []  # waiting assistance requests
        self.assist = None  # the one being dealt with
        self.station_seq = self.arrived_at = None
        self.last_seq = None
        self.rerouted = False
        self.holding_for_home = False
        self.phase = "hover"
        self.takeover = None  # what the pilot did, once seen - never cleared

    def watch_takeover(self, stop: threading.Event) -> None:
        """Runs on its own thread: remembers the first sign of a takeover."""
        while not stop.wait(TAKEOVER_POLL_S):
            moved = lm.switch_moved(self.link, self.switch)
            snap = self.link.monitor.snapshot()
            if moved:
                self.takeover = moved
            elif snap.connected and snap.mode_name not in ("AUTO", "GUIDED"):
                self.takeover = f"mode changed to {snap.mode_name}"
            if self.takeover:
                return

    def _guard(self, expected_mode: str) -> None:
        if self.takeover:
            raise lm.Handover(self.takeover)
        lm.guard(self.link, expected_mode, self.switch)

    def _map(self) -> RouteMap:
        return RouteMap(_flight["zones"], _flight["margin"], self.keep_outs)

    def _position(self):
        pos = self.link.position()
        if pos is None or pos.age_s > 2.0:
            raise lm.MissionAbort("no fresh position from the FCU - take over with the RC")
        return pos

    # --- changing route ---------------------------------------------------

    def _change_mode(self, mode: str) -> None:
        ok, message = self.link.monitor.send_mode(mode)
        deadline = time.time() + 5.0
        while ok and time.time() < deadline:
            if self.takeover:
                raise lm.Handover(self.takeover)
            if self.link.monitor.snapshot().mode_name == mode:
                return
            time.sleep(0.05)
        raise lm.MissionAbort(f"couldn't change to {mode} ({message}) - take over with the RC")

    def reroute(self, reason: str) -> None:
        """Stop, upload a new route for what is needed now, carry on in AUTO."""
        self._guard("AUTO")
        self._change_mode("GUIDED")
        self.say(f"Mission: {reason} - holding in GUIDED while the route changes")
        pos = self._position()
        here = (pos.lat, pos.lon)
        route_map = self._map()
        points = None
        while self.assist is not None and points is None:
            points = route_map.assist_path(here, self.assist["target"])
            if points is None:
                self.say(f"Mission: can't reach assistance request #{self.assist['seq']} any more - dropped", lm.SEV_WARNING)
                self._next_request()
        goes_home = False
        pieces = self.pieces
        if points is None:
            try:
                survey = route_map.survey_path(here, self.pieces, self.home)
            except PlanError as e:
                raise lm.MissionAbort(f"no safe new route ({e}) - the drone holds in GUIDED; take over with the RC")
            points, pieces, goes_home = survey.points, survey.pieces, survey.goes_home
            for warning in survey.warnings:
                self.say(f"Mission: {warning}", lm.SEV_WARNING)
        items, messages, leg_ends, station_seq, land_seq = self._items(here, points, goes_home)
        try:
            self.link.upload_mission(items)
            self.link.verify_mission(items)
        except MavlinkLinkError as e:
            raise lm.MissionAbort(f"new route not uploaded ({e}) - the drone holds in GUIDED; take over with the RC")
        self._guard("GUIDED")
        self.link.set_current_item(1)
        if not lm.wait_until(lambda: self.link.mission_current() == 1, 3.0, self.link, "GUIDED", switch=self.switch):
            raise lm.MissionAbort("the FCU didn't start the new route - the drone holds in GUIDED; take over with the RC")
        self.items, self.messages, self.leg_ends, self.pieces = items, messages, leg_ends, pieces
        self.station_seq, self.land_seq = station_seq, land_seq
        self.last_seq, self.arrived_at = None, None
        self.rerouted = True
        self.holding_for_home = self.assist is None and not goes_home
        self._guard("GUIDED")  # last check right before AUTO - never over a takeover
        self._change_mode("AUTO")
        if self.holding_for_home:
            self.say("Mission: home is blocked by a keep-out zone - holding after the survey until the All Clear; "
                     "take over with the RC if needed", lm.SEV_WARNING)

    def _items(self, here, points, goes_home):
        frame = lm.FRAME_RELATIVE
        items = [MissionItem(NAV_WAYPOINT, lm.mavutil.mavlink.MAV_FRAME_GLOBAL, lat=self.home[0], lon=self.home[1]),
                 MissionItem(NAV_WAYPOINT, frame, lat=here[0], lon=here[1], alt=self.altitude),  # stop, climb if needed
                 MissionItem(DO_CHANGE_SPEED, frame, param1=1, param2=self.speed, param3=-1)]
        messages, leg_ends = {}, {}
        station_seq = land_seq = None
        legs = sum(p.kind == "leg_end" for p in points)
        leg = 0
        for point in points:
            lat, lon = point.position
            if point.kind == "target":
                station_seq = len(items)
                messages[station_seq] = f"Mission: flying to assistance request #{self.assist['seq']}"
                items.append(MissionItem(LOITER_UNLIM, frame, lat=lat, lon=lon, alt=self.altitude))
                continue
            if point.kind == "leg_start":
                leg += 1
                messages[len(items)] = f"Mission: lawnmower stretch {leg}/{legs}"
            if point.kind == "leg_end":
                leg_ends[len(items)] = point.piece
            items.append(MissionItem(NAV_WAYPOINT, frame, lat=lat, lon=lon, alt=self.altitude))
        if self.assist is None and goes_home:
            messages[len(items)] = "Mission: lawnmower done - returning home"
            items.append(MissionItem(NAV_WAYPOINT, frame, lat=self.home[0], lon=self.home[1], alt=self.altitude))
            land_seq = len(items)
            messages[land_seq] = "Mission: landing at home"
            items.append(MissionItem(NAV_LAND, frame, lat=self.home[0], lon=self.home[1]))
        elif self.assist is None:
            lat, lon = points[-1].position if points else here
            items.append(MissionItem(LOITER_UNLIM, frame, lat=lat, lon=lon, alt=self.altitude))
        return items, messages, leg_ends, station_seq, land_seq

    # --- what RoboCommand sent ---------------------------------------------

    def _route_still_ok(self, route_map, here) -> bool:
        first = max(self.last_seq or 1, 1)
        points = [(i.lat, i.lon) for i in self.items[first:] if i.command != DO_CHANGE_SPEED and (i.lat or i.lon)]
        return route_map.path_ok(here, points)

    def _on_keep_outs(self, active) -> None:
        added = [z for z in active if z not in self.keep_outs]
        lifted = len(active) < len(self.keep_outs)
        self.keep_outs = active
        route_map = self._map()
        if self.assist is not None and route_map.clearance(self.assist["target"]) < ASSIST_MIN_CLEARANCE_M:
            self.say(f"Mission: assistance request #{self.assist['seq']} is now in a keep-out zone - dropped", lm.SEV_WARNING)
            self._next_request()
            self.reroute("a keep-out zone changed the plan")
            return
        pos = self._position()
        if added and not self._route_still_ok(route_map, (pos.lat, pos.lon)):
            centre, radius = added[-1]
            self.reroute(f"keep-out zone {radius:g} m around {centre[0]:.6f}, {centre[1]:.6f} is on the route")
        elif added:
            self.say("Mission: keep-out zone is clear of the route - carrying on")
        elif lifted and self.assist is None and (self.holding_for_home or self._blocked_stretches()):
            self.reroute("All Clear - flying what the keep-out zone blocked")

    def _blocked_stretches(self) -> bool:
        return any(piece not in self.leg_ends.values() for piece in self.pieces)

    def _next_request(self) -> None:
        self.assist = self.requests.pop(0) if self.requests else None
        self.arrived_at = None

    def _on_request(self, request) -> None:
        pos = self._position()
        if self.land_seq is not None and (self.last_seq or 0) >= self.land_seq and pos.relative_alt_m < LOW_LANDING_M:
            self.say(f"Mission: assistance request #{request['seq']} came too late - already landing", lm.SEV_WARNING)
            return
        if self._map().assist_path((pos.lat, pos.lon), request["target"]) is None:
            self.say(f"Mission: assistance request #{request['seq']} refused - no safe way to {request['target'][0]:.6f}, "
                     f"{request['target'][1]:.6f} (fence or keep-out zone); survey carries on", lm.SEV_WARNING)
            return
        self.assist = request
        self.reroute(f"assistance request #{request['seq']}")

    def _check_state(self, state, pos) -> None:
        active = [(tuple(z["center"]), float(z["radius_m"])) for z in state.get("keep_out_zones", [])
                  if z.get("vehicle_type") == UAV_TYPE and z.get("center") and float(z.get("radius_m", 0)) > 0]
        if active != self.keep_outs:
            self._on_keep_outs(active)
        for req in state.get("assistance_requests", []):
            seq = req.get("command_seq")
            if req.get("vehicle_type") != UAV_TYPE or not req.get("position") or seq in self.seen_requests:
                continue
            self.seen_requests.add(seq)
            if req.get("received_at", 0) >= _started_at:
                self.requests.append({"seq": seq, "target": tuple(req["position"])})
        if self.assist is None and self.requests:
            self._on_request(self.requests.pop(0))
        if self.assist is not None:
            self._on_station(state, pos)

    def _on_station(self, state, pos) -> None:
        if self.arrived_at is None:
            if self.last_seq == self.station_seq and _distance_m((pos.lat, pos.lon), self.assist["target"]) <= ON_STATION_M:
                self.arrived_at = time.time()
                self.say(f"Mission: on station for assistance request #{self.assist['seq']} - holding for ReadinessConfirm")
            return
        ready = any(req.get("command_seq") == self.assist["seq"] and req.get("readiness_sent")
                    for req in state.get("assistance_requests", []))
        confirmed = ready and any(c.get("vehicle_id") == UAV_VEHICLE_ID and c.get("received_at", 0) >= self.arrived_at
                                  for c in state.get("readiness_confirms", []))
        waited = time.time() - self.arrived_at
        if confirmed or waited >= CONFIRM_TIMEOUT_S:
            if not confirmed:
                self.say(f"Mission: no ReadinessConfirm after {waited:.0f} s - carrying on", lm.SEV_WARNING)
            seq = self.assist["seq"]
            self._next_request()
            self.reroute(f"assistance request #{seq} done")

    # --- watching ------------------------------------------------------------

    def _progress(self, seq) -> None:
        if seq == self.last_seq or seq is None:
            return
        self.last_seq = seq
        for end in [e for e in self.leg_ends if seq > e]:
            piece = self.leg_ends.pop(end)
            if piece in self.pieces:
                self.pieces.remove(piece)
        if seq in self.messages:
            self.say(self.messages[seq])

    def _phase_now(self) -> str:
        if self.assist is not None:
            return "assist"
        if self.land_seq is not None and (self.last_seq or 0) >= self.land_seq:
            return "landing"
        return "hover" if not self.rerouted and (self.last_seq or 0) <= lm.HOVER_SEQ else "lawnmower"

    def watch(self) -> int:
        """lawnmower_mission.watch_mission's job, plus reacting to Task 4."""
        lost_since = None
        while True:
            time.sleep(0.5)
            snap = self.link.monitor.snapshot()
            if not snap.connected:
                if lost_since is None:
                    lost_since = time.time()
                    self.say("Mission: lost the FCU heartbeat - ArduPilot keeps flying the mission on its own; "
                             "take over with the RC if needed", lm.SEV_WARNING)
                elif time.time() - lost_since > 120:
                    self.say("Mission: no heartbeat for 2 min - script exiting (the FCU is still in charge)", lm.SEV_ERROR)
                    return 1
                continue
            if lost_since is not None:
                self.say("Mission: heartbeat back")
                lost_since = None
            if not snap.armed:
                if self.land_seq is not None and (self.last_seq or 0) >= self.land_seq:
                    self.say("Mission: COMPLETE - landed at home and disarmed")
                    if self.pieces:
                        self.say(f"Mission: {len(self.pieces)} stretch(es) of the survey weren't flown", lm.SEV_WARNING)
                    lm.finish_mission(self.link, self.say, self.switch)
                    return 0
                self.say("Mission: the drone disarmed before the mission finished - script stopped", lm.SEV_WARNING)
                return 0
            moved = self.takeover or lm.switch_moved(self.link, self.switch)
            if moved:
                self.say(f"Mission: {moved} - script stopped, you have control", lm.SEV_WARNING)
                return 0
            if snap.mode_name != "AUTO":
                self.say(f"Mission: mode changed to {snap.mode_name} (RC, Mission Planner or a failsafe) - "
                         "script stopped, you have control", lm.SEV_WARNING)
                return 0
            self._progress(self.link.mission_current())
            self.phase = self._phase_now()
            state = read_state()
            if state is not None:
                self._check_state(state, self._position())


def watch_with_task4(link, messages, land_seq, say, switch=None):
    task4 = Task4(link, messages, land_seq, say, switch)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()

    def send(active, phase):
        try:
            sock.sendto(json.dumps({"mission": "task4", "active": active, "phase": phase}).encode("utf-8"),
                        runstart_mission.CV_TOPIC)
        except OSError:
            pass  # a problem with the CV signal must never affect the mission

    def publish():
        while not stop.is_set():
            send(True, task4.phase)
            stop.wait(runstart_mission.CV_PERIOD_S)

    if read_state() is None:
        say(f"Mission: warning - no {STATE_PATH.name} yet: Task 4 commands only arrive if the listener is running",
            lm.SEV_WARNING)
    say(f"Mission: Task 4 handling ON. CV signal ON (UDP {runstart_mission.CV_TOPIC[0]}:{runstart_mission.CV_TOPIC[1]})")
    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    threading.Thread(target=task4.watch_takeover, args=(stop,), daemon=True).start()
    try:
        return task4.watch()
    finally:
        stop.set()
        thread.join(timeout=2.0)
        for _ in range(3):  # UDP can drop one
            send(False, "done")
        sock.close()
        say("Mission: CV signal OFF")


def main():
    lm.plan_lawnmower, lm.build_mission = _plan_and_keep, _build_and_keep
    lm.watch_mission = watch_with_task4
    lm.main({"hover_altitude": HOVER_ALTITUDE_M, "hover": HOVER_S, "altitude": ALTITUDE_M, "speed": SPEED_MS})


if __name__ == "__main__":
    main()
