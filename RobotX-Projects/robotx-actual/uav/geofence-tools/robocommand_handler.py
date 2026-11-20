"""
Orin-side handling of RoboCommand messages that robocommand_bridge.py (on
the GCS laptop) forwards to geofence_udp_listener.py.

Plain JSON in - the bridge already decoded RoboNation's protobuf messages -
so this needs no protobuf library on the Orin.

For every message it:
  - prints it in the drone GUI's message log (same notification port the
    fence messages use), highlighting the ones aimed at this UAV;
  - keeps the latest state in a JSON file (--robocommand-state): course,
    run start, assistance requests, active keep-out zones, moving objects -
    so other scripts can read it.
When the course arrives (and whenever the fence changes) it also checks the
fence on the FCU against RoboCommand's rule for the UAV geofence: every
point must be inside the course boundary or the UAV transit area.

It deliberately does NOT change anything on the FCU - no fence edits, no
mode changes. A keep-out zone pushed into the fence mid-flight could trip
the fence action (e.g. land) over water; acting on these stays a deliberate
step for the operator.
"""

import json
import math
import os
import time
from collections import deque
from pathlib import Path
from typing import Callable, List, Optional, Sequence, Tuple

# MAV_SEVERITY values, so the GUI colours them like FCU messages.
SEV_WARNING = 4
SEV_NOTICE = 5
SEV_INFO = 6
SEV_DEBUG = 7

UAV_TYPE = "TYPE_UAV"
EDGE_SAMPLE_M = 0.5
ON_BOUNDARY_M = 0.1

# Task 4 ReadinessReport: the UAV counts as "on station" for an assistance
# request once it has held, airborne, within this distance of the requested
# position and below this speed, for this long.
READY_RADIUS_M = 3.0
READY_SPEED_MPS = 0.7
READY_HOLD_S = 3.0

LatLon = Tuple[float, float]


def _latlon(point: Optional[dict]) -> Optional[LatLon]:
    if not point:
        return None
    return float(point.get("latitude", 0.0)), float(point.get("longitude", 0.0))


def _fmt(point: Optional[LatLon]) -> str:
    return f"{point[0]:.6f}, {point[1]:.6f}" if point else "?"


def _short_type(vehicle_type: str) -> str:
    return vehicle_type.replace("TYPE_", "") if vehicle_type else "?"


class _Flat:
    """Flat metres around an origin - plenty for a course a few km across."""

    def __init__(self, origin: LatLon):
        self.lat0, self.lon0 = origin
        self.cos = math.cos(math.radians(self.lat0))

    def xy(self, p: LatLon) -> Tuple[float, float]:
        return (math.radians(p[1] - self.lon0) * 6378137.0 * self.cos, math.radians(p[0] - self.lat0) * 6378137.0)


def _inside_or_on(p, poly) -> bool:
    inside = False
    n = len(poly)
    for i in range(n):
        (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % n]
        if (y1 > p[1]) != (y2 > p[1]) and p[0] < x1 + (p[1] - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
        # On the boundary counts as inside.
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((p[0] - x1) * dx + (p[1] - y1) * dy) / ((dx * dx + dy * dy) or 1.0)))
        if math.hypot(p[0] - (x1 + dx * t), p[1] - (y1 + dy * t)) <= ON_BOUNDARY_M:
            return True
    return inside


def fence_outside_points(fence: Sequence[LatLon], areas: List[List[LatLon]]) -> List[LatLon]:
    """Points along the fence's edges (corners plus every EDGE_SAMPLE_M) that
    are in none of the areas - empty if the whole fence fits."""
    areas = [a for a in areas if len(a) >= 3]
    if not areas or len(fence) < 2:
        return []
    flat = _Flat(fence[0])
    polys = [[flat.xy(p) for p in a] for a in areas]
    closed = list(fence) if fence[0] == fence[-1] else list(fence) + [fence[0]]
    bad = []
    for a, b in zip(closed, closed[1:]):
        ax, ay = flat.xy(a)
        bx, by = flat.xy(b)
        steps = max(1, math.ceil(math.hypot(bx - ax, by - ay) / EDGE_SAMPLE_M))
        for i in range(steps):
            t = i / steps
            p = (ax + (bx - ax) * t, ay + (by - ay) * t)
            if not any(_inside_or_on(p, poly) for poly in polys):
                bad.append(a if i == 0 else (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    return bad


class RoboCommandHandler:
    def __init__(self, notify: Callable[[str, int], None], state_path: Path, vehicle_id: str = "UAV1"):
        self.notify = notify
        self.state_path = state_path
        self.vehicle_id = vehicle_id
        self._seen = deque(maxlen=500)  # message ids - the bridge re-sends until acknowledged
        self._on_station_since = {}  # assistance request seq -> when the drone got there (for readiness_due)
        self.fence: Optional[List[LatLon]] = None
        self.state = {
            "course": None,
            "run_start": None,
            "assistance_requests": [],
            "keep_out_zones": [],
            "moving_objects": [],
            "readiness_confirms": [],
            "updated_at": None,
        }

    # --- called by the listener -------------------------------------------

    def is_new(self, msg_id) -> bool:
        if msg_id is None:
            return True
        if msg_id in self._seen:
            return False
        self._seen.append(msg_id)
        return True

    def handle(self, msg: dict) -> None:
        kind, body = msg.get("kind"), msg.get("message") or {}
        try:
            if kind == "course":
                self._on_course(body)
            elif kind == "command":
                self._on_command(body)
            else:
                self.notify(f"RoboCommand: unknown message kind '{kind}'", SEV_DEBUG)
                return
        except (TypeError, ValueError, KeyError) as e:
            self.notify(f"RoboCommand: couldn't read a {kind} message ({e})", SEV_WARNING)
            return
        self._save()

    def readiness_due(self, lat: float, lon: float, speed_mps: float, airborne: bool) -> List[int]:
        """Command seqs of UAV assistance requests the drone has now reached
        and held (see READY_*), each returned once - for the listener to send
        a ReadinessReport. Never moves the drone: getting there is up to the
        pilot or a mission."""
        due, now = [], time.time()
        for req in self.state["assistance_requests"]:
            seq = req.get("command_seq")
            if req.get("vehicle_type") != UAV_TYPE or req.get("readiness_sent") or not req.get("position"):
                continue
            flat = _Flat((lat, lon))
            x, y = flat.xy(tuple(req["position"]))
            on_station = airborne and math.hypot(x, y) <= READY_RADIUS_M and speed_mps <= READY_SPEED_MPS
            if not on_station:
                self._on_station_since.pop(seq, None)
                continue
            since = self._on_station_since.setdefault(seq, now)
            if now - since >= READY_HOLD_S:
                req["readiness_sent"] = True
                self._on_station_since.pop(seq, None)
                due.append(seq)
                self.notify(f"RoboCommand: on station for assistance request #{seq} - sending ReadinessReport", SEV_NOTICE)
        if due:
            self._save()
        return due

    def fence_changed(self, vertices: Sequence[Tuple[float, float]]) -> None:
        """The listener applied a new fence (update or reset) - recheck it."""
        self.fence = [(float(lat), float(lon)) for lat, lon in vertices]
        if self.state["course"]:
            self._check_fence()

    # --- messages -----------------------------------------------------------

    def _on_course(self, body: dict) -> None:
        corners = [_latlon(p) for p in body.get("corners", [])]
        transit = [_latlon(p) for p in body.get("uav_transit_area", [])]
        self.state["course"] = {
            "course_id": body.get("course_id", ""),
            "pinger_freq_hz": body.get("pinger_freq_hz", 0),
            "corners": corners,
            "uav_transit_area": transit,
            "sent_at": body.get("sent_at"),
        }
        transit_note = f", UAV transit area {len(transit)} corners" if transit else ", no UAV transit area"
        self.notify(f"RoboCommand: course {body.get('course_id', '?')} - boundary {len(corners)} corners{transit_note}", SEV_INFO)
        self._check_fence()

    def _on_command(self, body: dict) -> None:
        seq = body.get("seq", 0)
        target = body.get("vehicle_id", "")
        if target and target != self.vehicle_id:
            self.notify(f"RoboCommand: command #{seq} is addressed to {target}, not {self.vehicle_id}", SEV_DEBUG)
            return
        now = time.time()

        if "run_start" in body:
            rs = body["run_start"]
            self.state["run_start"] = dict(rs, command_seq=seq, received_at=now)
            self.notify(f"RoboCommand: RUN START - run {rs.get('run_id', '?')} (declaration #{rs.get('declaration_seq', '?')})", SEV_NOTICE)

        elif "assistance_request" in body:
            ar = body["assistance_request"]
            vtype = ar.get("vehicle_type", "")
            pos = _latlon(ar.get("position"))
            self.state["assistance_requests"].append({"position": pos, "vehicle_type": vtype, "command_seq": seq, "received_at": now})
            if vtype == UAV_TYPE:
                self.notify(f"RoboCommand: ASSISTANCE REQUEST for the UAV - go to {_fmt(pos)}, loiter, then report ready "
                            f"(command #{seq})", SEV_WARNING)
            else:
                self.notify(f"RoboCommand: assistance request for the {_short_type(vtype)} (command #{seq}) - not this drone", SEV_INFO)

        elif "keep_out_zone" in body:
            kz = body["keep_out_zone"]
            vtype = kz.get("vehicle_type", "")
            centre = _latlon(kz.get("center"))
            radius = float(kz.get("radius_m", 0.0))
            self.state["keep_out_zones"].append({"center": centre, "radius_m": radius, "vehicle_type": vtype,
                                                 "command_seq": seq, "received_at": now})
            if vtype == UAV_TYPE:
                self.notify(f"RoboCommand: KEEP-OUT ZONE for the UAV - stay {radius:g} m clear of {_fmt(centre)} "
                            f"until All Clear (command #{seq})", SEV_WARNING)
            else:
                self.notify(f"RoboCommand: keep-out zone for the {_short_type(vtype)} (command #{seq}) - not this drone", SEV_INFO)

        elif "all_clear" in body:
            vtype = body["all_clear"].get("vehicle_type", "")
            before = len(self.state["keep_out_zones"])
            self.state["keep_out_zones"] = [z for z in self.state["keep_out_zones"] if z["vehicle_type"] != vtype]
            lifted = before - len(self.state["keep_out_zones"])
            who = "the UAV" if vtype == UAV_TYPE else f"the {_short_type(vtype)}"
            self.notify(f"RoboCommand: ALL CLEAR for {who} - {lifted} keep-out zone(s) lifted (command #{seq})",
                        SEV_NOTICE if vtype == UAV_TYPE else SEV_INFO)

        elif "moving_object_alert" in body:
            mo = body["moving_object_alert"]
            affected = mo.get("affected_vehicle_types", [])
            pos = _latlon(mo.get("position"))
            radius = float(mo.get("keep_out_radius_m", 0.0)) or 5.0  # 5 m is RoboCommand's default
            entry = {"position": pos, "heading_deg": mo.get("heading_deg", 0.0), "speed_mps": mo.get("speed_mps", 0.0),
                     "keep_out_radius_m": radius, "affected_vehicle_types": affected, "command_seq": seq, "received_at": now}
            self.state["moving_objects"] = (self.state["moving_objects"] + [entry])[-20:]
            text = (f"MOVING OBJECT at {_fmt(pos)}, heading {float(mo.get('heading_deg', 0.0)):.0f} deg at "
                    f"{float(mo.get('speed_mps', 0.0)):.1f} m/s - keep {radius:g} m away (command #{seq})")
            if UAV_TYPE in affected:
                self.notify(f"RoboCommand: {text}", SEV_WARNING)
            else:
                types = ", ".join(_short_type(t) for t in affected) or "?"
                self.notify(f"RoboCommand: {text} - affects {types}, not this drone", SEV_INFO)

        elif "readiness_confirm" in body:
            rc = body["readiness_confirm"]
            self.state["readiness_confirms"].append(dict(rc, command_seq=seq, received_at=now))
            self.notify(f"RoboCommand: readiness confirmed for {rc.get('vehicle_id', '?')} - back to normal operations "
                        f"(command #{seq})", SEV_INFO)

        else:
            self.notify(f"RoboCommand: command #{seq} with no body this version knows", SEV_DEBUG)

    # --- helpers -------------------------------------------------------------

    def _check_fence(self) -> None:
        course = self.state["course"]
        if not course or not self.fence:
            return
        areas = [course["corners"], course["uav_transit_area"]]
        bad = fence_outside_points(self.fence, areas)
        if bad:
            self.notify(f"RoboCommand: the FCU fence goes OUTSIDE the course and UAV transit area (e.g. at {_fmt(bad[0])}) "
                        "- RoboCommand would refuse a run declaration with this geofence", SEV_WARNING)
        else:
            self.notify("RoboCommand: the FCU fence fits inside the course / UAV transit area", SEV_INFO)

    def _save(self) -> None:
        self.state["updated_at"] = time.time()
        tmp = self.state_path.with_suffix(".tmp")
        try:
            tmp.write_text(json.dumps(self.state, indent=2))
            os.replace(tmp, self.state_path)  # atomic, so a reader never sees half a file
        except OSError as e:
            self.notify(f"RoboCommand: couldn't save {self.state_path}: {e}", SEV_WARNING)
