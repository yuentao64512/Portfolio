"""
MAVLink side of the lawnmower mission, built on the drone GUI's own
HeartbeatMonitor (drone-heartbeat-gui/heartbeat_link.py).

The monitor owns the one connection: it tracks the heartbeat (connected,
armed, flight mode, GPS, RC inputs) exactly as the GUI does, and hands every
message it receives to this module too, so nothing here opens a second
connection or duplicates the heartbeat logic.

On top of that this adds what a mission needs: reading parameters, reading
the fence, uploading and verifying a mission, sending commands and watching
mission progress. Parameter reads reuse geofence-tools/mavlink_param_link.py.

IMPORTANT: MAVLINK20=1 must be set before pymavlink is first imported -
the mission messages need the MAVLink2 `mission_type` field.
"""

import os

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")  # pymavlink 2.4.50+ otherwise picks "all"

import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_ROOT / "drone-heartbeat-gui"))
sys.path.insert(0, str(_ROOT / "geofence-tools"))

from pymavlink import mavutil  # noqa: E402

from heartbeat_link import HeartbeatMonitor  # noqa: E402
from lawnmower_planner import FenceZones  # noqa: E402
from mavlink_param_link import MavlinkLinkError, Vehicle, get_param  # noqa: E402

DEFAULT_TIMEOUT_S = 5.0

# Messages the request/response code below waits for. Everything else is
# left to HeartbeatMonitor.
_PROTOCOL_TYPES = {
    "PARAM_VALUE",
    "MISSION_COUNT",
    "MISSION_ITEM_INT",
    "MISSION_REQUEST",
    "MISSION_REQUEST_INT",
    "MISSION_ACK",
    "COMMAND_ACK",
}

# Extra streams the mission needs, on top of what HeartbeatMonitor asks for.
MISSION_STREAM_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_MISSION_CURRENT: 2,
}

FENCE_CMD_INCLUSION_POLYGON = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
FENCE_CMD_EXCLUSION_POLYGON = mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
FENCE_CMD_INCLUSION_CIRCLE = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION
FENCE_CMD_EXCLUSION_CIRCLE = mavutil.mavlink.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
FENCE_CMD_RETURN_POINT = mavutil.mavlink.MAV_CMD_NAV_FENCE_RETURN_POINT


@dataclass(frozen=True)
class MissionItem:
    command: int
    frame: int
    param1: float = 0.0
    param2: float = 0.0
    param3: float = 0.0
    param4: float = 0.0
    lat: float = 0.0
    lon: float = 0.0
    alt: float = 0.0
    autocontinue: int = 1


@dataclass(frozen=True)
class Position:
    lat: float
    lon: float
    relative_alt_m: float
    age_s: float


class _QueueMaster:
    """Looks like a pymavlink connection (`.mav` + `.recv_match`) to code that
    expects one, e.g. mavlink_param_link.get_param, but reads from the
    messages HeartbeatMonitor passes on instead of the socket - the monitor's
    thread is the only one reading the socket."""

    def __init__(self, monitor: HeartbeatMonitor):
        self._monitor = monitor
        self._queue: "queue.Queue" = queue.Queue()

    def put(self, msg) -> None:
        self._queue.put(msg)

    def flush(self) -> None:
        while True:
            try:
                self._queue.get_nowait()
            except queue.Empty:
                return

    @property
    def mav(self):
        master, _, _ = self._monitor.active_link()
        if master is None:
            raise MavlinkLinkError("no active link to the FCU")
        return master.mav

    def recv_match(self, type=None, blocking=True, timeout=None):
        wanted = {type} if isinstance(type, str) else set(type) if type else None
        deadline = time.time() + (timeout if timeout is not None else 1e9)
        while True:
            remaining = deadline - time.time() if blocking else 0
            if remaining < 0:
                return None
            try:
                msg = self._queue.get(timeout=remaining) if blocking else self._queue.get_nowait()
            except queue.Empty:
                return None
            if wanted is None or msg.get_type() in wanted:
                return msg


class MissionLink:
    def __init__(self, connection_string: str, heartbeat_timeout: float = 3.0):
        self.monitor = HeartbeatMonitor(connection_string, heartbeat_timeout=heartbeat_timeout)
        self._master = _QueueMaster(self.monitor)
        self.vehicle: Optional[Vehicle] = None

        self._lock = threading.Lock()
        self._position: Optional[Tuple[float, float, float, float]] = None  # lat, lon, rel alt, time
        self._mission_current: Optional[int] = None
        self._last_reached: Optional[int] = None
        self._statustexts: List[str] = []

        self.monitor.add_message_listener(self._on_message)

    # --- connection -------------------------------------------------------

    def connect(self, timeout_s: float = 15.0) -> Vehicle:
        """Start the heartbeat monitor and wait for the FCU's heartbeat."""
        self.monitor.start()
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            snap = self.monitor.snapshot()
            if snap.connected and snap.system_id is not None:
                self.vehicle = Vehicle(master=self._master, system_id=snap.system_id, component_id=snap.component_id)
                self._request_streams()
                return self.vehicle
            time.sleep(0.2)
        raise MavlinkLinkError(f"no FCU heartbeat on {self.monitor.connection_string} within {timeout_s:.0f}s")

    def close(self) -> None:
        self.monitor.stop()

    def _request_streams(self) -> None:
        for message_id, hz in MISSION_STREAM_RATES_HZ.items():
            self.command_long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, message_id, 1_000_000.0 / hz, wait_ack=False)

    def _on_message(self, msg) -> None:
        """Called on the monitor's link thread for every message."""
        snap_sysid = self.vehicle.system_id if self.vehicle is not None else None
        if snap_sysid is not None and msg.get_srcSystem() != snap_sysid:
            return  # e.g. Mission Planner or MAVProxy, not the FCU
        msg_type = msg.get_type()
        if msg_type in _PROTOCOL_TYPES:
            self._master.put(msg)
        elif msg_type == "GLOBAL_POSITION_INT":
            with self._lock:
                self._position = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0, time.time())
        elif msg_type == "MISSION_CURRENT":
            with self._lock:
                self._mission_current = int(msg.seq)
        elif msg_type == "MISSION_ITEM_REACHED":
            with self._lock:
                self._last_reached = int(msg.seq)
        elif msg_type == "STATUSTEXT":
            text = msg.text.decode("utf-8", "replace") if isinstance(msg.text, bytes) else msg.text
            with self._lock:
                self._statustexts = (self._statustexts + [text.rstrip("\x00")])[-20:]

    # --- state ------------------------------------------------------------

    def position(self) -> Optional[Position]:
        with self._lock:
            if self._position is None:
                return None
            lat, lon, alt, t = self._position
        return Position(lat=lat, lon=lon, relative_alt_m=alt, age_s=time.time() - t)

    def mission_current(self) -> Optional[int]:
        with self._lock:
            return self._mission_current

    def recent_statustexts(self) -> List[str]:
        with self._lock:
            return list(self._statustexts)

    # --- parameters / commands -------------------------------------------

    def get_param(self, name: str, timeout_s: float = DEFAULT_TIMEOUT_S) -> float:
        self._master.flush()
        return get_param(self._require_vehicle(), name, timeout_s=timeout_s)

    def command_long(self, command: int, *params: float, wait_ack: bool = True, timeout_s: float = DEFAULT_TIMEOUT_S) -> Optional[int]:
        """Send a COMMAND_LONG. Returns the COMMAND_ACK result (None if not
        waited for or no ack arrived)."""
        vehicle = self._require_vehicle()
        p = list(params) + [0.0] * (7 - len(params))
        if wait_ack:
            self._master.flush()
        self._master.mav.command_long_send(vehicle.system_id, vehicle.component_id, command, 0, *p)
        if not wait_ack:
            return None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self._master.recv_match(type="COMMAND_ACK", timeout=max(0.0, deadline - time.time()))
            if msg is not None and msg.command == command:
                return int(msg.result)
        return None

    def set_current_item(self, seq: int) -> None:
        vehicle = self._require_vehicle()
        self._master.mav.mission_set_current_send(vehicle.system_id, vehicle.component_id, seq)

    # --- mission protocol ------------------------------------------------

    def download_items(self, mission_type: int, timeout_s: float = DEFAULT_TIMEOUT_S) -> list:
        """Every MISSION_ITEM_INT of one mission type (fence, mission...)."""
        vehicle = self._require_vehicle()
        self._master.flush()
        self._master.mav.mission_request_list_send(vehicle.system_id, vehicle.component_id, mission_type)
        count = None
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self._master.recv_match(type="MISSION_COUNT", timeout=1.0)
            if msg is not None and getattr(msg, "mission_type", mission_type) == mission_type:
                count = msg.count
                break
        if count is None:
            raise MavlinkLinkError("no MISSION_COUNT reply - download timed out")

        items = []
        for seq in range(count):
            for _attempt in range(3):
                self._master.mav.mission_request_int_send(vehicle.system_id, vehicle.component_id, seq, mission_type)
                item_deadline = time.time() + timeout_s / 2
                msg = None
                while time.time() < item_deadline:
                    msg = self._master.recv_match(type="MISSION_ITEM_INT", timeout=0.5)
                    if msg is not None and msg.seq == seq and getattr(msg, "mission_type", mission_type) == mission_type:
                        break
                    msg = None
                if msg is not None:
                    items.append(msg)
                    break
            else:
                raise MavlinkLinkError(f"no MISSION_ITEM_INT for item {seq} - download incomplete")
        self._master.mav.mission_ack_send(
            vehicle.system_id, vehicle.component_id, mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type
        )
        return items

    def upload_mission(self, items: List[MissionItem], timeout_s: float = DEFAULT_TIMEOUT_S, max_retries: int = 3) -> None:
        """Replace the FCU's mission with items (item 0 is home - ArduPilot
        overwrites it with the real home position). Raises on failure."""
        vehicle = self._require_vehicle()
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        count = len(items)
        for _attempt in range(max_retries):
            self._master.flush()
            self._master.mav.mission_count_send(vehicle.system_id, vehicle.component_id, count, mission_type)
            deadline = time.time() + timeout_s
            while time.time() < deadline:
                msg = self._master.recv_match(type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], timeout=1.0)
                if msg is None or getattr(msg, "mission_type", mission_type) != mission_type:
                    continue
                if msg.get_type() == "MISSION_ACK":
                    if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                        return
                    raise MavlinkLinkError(f"FCU rejected the mission (MISSION_ACK type={msg.type})")
                if msg.seq >= count:
                    continue
                item = items[msg.seq]
                self._master.mav.mission_item_int_send(
                    vehicle.system_id,
                    vehicle.component_id,
                    msg.seq,
                    item.frame,
                    item.command,
                    0,  # current
                    item.autocontinue,
                    item.param1,
                    item.param2,
                    item.param3,
                    item.param4,
                    int(round(item.lat * 1e7)),
                    int(round(item.lon * 1e7)),
                    item.alt,
                    mission_type,
                )
                deadline = time.time() + timeout_s  # progress - keep going
        raise MavlinkLinkError(f"no MISSION_ACK after {max_retries} attempt(s) - mission upload did not complete")

    def clear_mission(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        """Delete the FCU's mission (MISSION_CLEAR_ALL, mission list only -
        the fence and rally points stay). Raises on failure."""
        vehicle = self._require_vehicle()
        mission_type = mavutil.mavlink.MAV_MISSION_TYPE_MISSION
        self._master.flush()
        self._master.mav.mission_clear_all_send(vehicle.system_id, vehicle.component_id, mission_type)
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self._master.recv_match(type="MISSION_ACK", timeout=max(0.0, deadline - time.time()))
            if msg is None or getattr(msg, "mission_type", mission_type) != mission_type:
                continue
            if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                return
            raise MavlinkLinkError(f"FCU refused to clear the mission (MISSION_ACK type={msg.type})")
        raise MavlinkLinkError("no MISSION_ACK - mission clear timed out")

    def verify_mission(self, items: List[MissionItem]) -> None:
        """Download the mission back and compare it item by item. Raises on
        any difference - "uploaded" only means the handshake finished."""
        downloaded = self.download_items(mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        if len(downloaded) != len(items):
            raise MavlinkLinkError(f"mission read back has {len(downloaded)} items, expected {len(items)}")
        for seq in range(1, len(items)):  # item 0 is home, rewritten by the FCU
            want, got = items[seq], downloaded[seq]
            if (
                got.command != want.command
                or abs(got.x - int(round(want.lat * 1e7))) > 1
                or abs(got.y - int(round(want.lon * 1e7))) > 1
                or abs(got.z - want.alt) > 0.05
                or abs(got.param1 - want.param1) > 0.01
                or abs(got.param2 - want.param2) > 0.01
            ):
                raise MavlinkLinkError(f"mission item {seq} read back differently from what was uploaded")

    def _require_vehicle(self) -> Vehicle:
        if self.vehicle is None:
            raise MavlinkLinkError("not connected - call connect() first")
        return self.vehicle


def fence_zones_from_items(items: list) -> FenceZones:
    """Every zone in a downloaded fence: inclusion/exclusion polygons and
    circles. Raises on anything it doesn't recognise rather than planning a
    path that ignores part of the fence."""
    zones = FenceZones()
    polygon_lists = {
        FENCE_CMD_INCLUSION_POLYGON: zones.inclusion_polygons,
        FENCE_CMD_EXCLUSION_POLYGON: zones.exclusion_polygons,
    }
    circle_lists = {
        FENCE_CMD_INCLUSION_CIRCLE: zones.inclusion_circles,
        FENCE_CMD_EXCLUSION_CIRCLE: zones.exclusion_circles,
    }
    current: Optional[List[Tuple[float, float]]] = None
    remaining = 0
    for item in items:
        point = (item.x / 1e7, item.y / 1e7)
        if item.command in polygon_lists:
            # A polygon is param1 consecutive vertex items of the same command.
            if remaining == 0:
                remaining = int(item.param1)
                current = []
                polygon_lists[item.command].append(current)
            current.append(point)
            remaining -= 1
        elif item.command in circle_lists:
            circle_lists[item.command].append((point, float(item.param1)))
        elif item.command != FENCE_CMD_RETURN_POINT:
            raise MavlinkLinkError(f"the FCU fence contains an unknown zone type (MAV_CMD {item.command})")
    if remaining:
        raise MavlinkLinkError("the FCU fence has an incomplete polygon")
    if not zones.inclusion_polygons and not zones.inclusion_circles:
        raise MavlinkLinkError("no inclusion fence on the FCU - publish one with geofence_udp_publisher.py first")
    return zones
