"""
Shared MAVLink backend for the geofence/parameter tools: connecting,
reading/writing ArduPilot parameters, watching for parameter changes, and
uploading/downloading a polygon geofence.

This module knows nothing about CLI argument parsing or output formatting -
it just does the MAVLink protocol work and returns plain data, so each
command-line tool on top of it stays small.

IMPORTANT: this must set MAVLINK20=1 before pymavlink is imported anywhere
in the process. Fence upload/download uses MISSION_COUNT/MISSION_ITEM_INT/
MISSION_REQUEST_INT/MISSION_ACK with the `mission_type` field, which only
exists on the MAVLink2 dialect - pymavlink's default dialect is the older
v1.0-generated one and silently lacks that field, which would otherwise make
every fence message go out with mission_type missing/wrong and the vehicle
would never recognize it as a fence upload at all.
"""

import os

os.environ.setdefault("MAVLINK20", "1")
os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")  # pymavlink 2.4.50+ otherwise picks "all"

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional, Tuple

from pymavlink import mavutil

# is_autopilot_heartbeat() already exists and is tested in the drone GUI -
# reuse it rather than duplicating the same filtering logic here.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "drone-heartbeat-gui"))
from heartbeat_link import is_autopilot_heartbeat  # noqa: E402

DEFAULT_TIMEOUT_S = 5.0


class MavlinkLinkError(RuntimeError):
    pass


@dataclass
class Vehicle:
    master: object  # mavutil.mavfile
    system_id: int
    component_id: int


def connect_and_wait_for_heartbeat(connection_string: str, timeout_s: float = 15.0,
                                   source_component: "int | None" = None) -> Vehicle:
    """Connect and block until a real autopilot heartbeat (not a GCS/hub one)
    is seen, so every command below has a correct, verified target.

    source_component: this program's own MAVLink component id (default:
    pymavlink's 0). A unique one lets the fence upload/download below tell
    the FCU's replies to THIS program from its replies to Mission Planner or
    MAVProxy, which arrive on the same MAVProxy output."""
    if source_component is None:
        master = mavutil.mavlink_connection(connection_string)
    else:
        master = mavutil.mavlink_connection(connection_string, source_system=255, source_component=source_component)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is not None and is_autopilot_heartbeat(msg):
            return Vehicle(master=master, system_id=msg.get_srcSystem(), component_id=msg.get_srcComponent())
    raise MavlinkLinkError(f"no autopilot heartbeat received on {connection_string} within {timeout_s:.0f}s")


# --- Parameters ---------------------------------------------------------


def get_param(vehicle: Vehicle, name: str, timeout_s: float = DEFAULT_TIMEOUT_S) -> float:
    name_bytes = name.encode("utf-8")
    vehicle.master.mav.param_request_read_send(vehicle.system_id, vehicle.component_id, name_bytes, -1)

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = vehicle.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if _param_id_str(msg.param_id) == name:
            return float(msg.param_value)
    raise MavlinkLinkError(f"no PARAM_VALUE for '{name}' within {timeout_s:.0f}s")


def set_param(
    vehicle: Vehicle, name: str, value: float, timeout_s: float = DEFAULT_TIMEOUT_S
) -> Tuple[bool, str, Optional[float]]:
    """Set a parameter and wait for the vehicle's own PARAM_VALUE echo to
    confirm it actually took - ArduPilot can silently clamp or reject a
    value, so the echo (not just "message sent") is what's trusted here.

    Always sends MAV_PARAM_TYPE_REAL32: ArduPilot's own convention is that
    every parameter goes out as float32 on the wire regardless of its real
    underlying type, with the firmware handling the actual cast internally.
    Returns (ok, message, confirmed_value).
    """
    name_bytes = name.encode("utf-8")
    vehicle.master.mav.param_set_send(
        vehicle.system_id,
        vehicle.component_id,
        name_bytes,
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    )

    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = vehicle.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if _param_id_str(msg.param_id) != name:
            continue
        confirmed = float(msg.param_value)
        if abs(confirmed - value) < 1e-6:
            return True, f"{name} set to {confirmed:g}", confirmed
        return False, f"{name} echoed {confirmed:g}, not the requested {value:g} (rejected or clamped)", confirmed
    return False, f"no confirmation for '{name}' within {timeout_s:.0f}s", None


def listen_params(
    vehicle: Vehicle,
    on_param: Callable[[str, float, int, int], None],
    request_all_first: bool = True,
    stop_check: Optional[Callable[[], bool]] = None,
) -> None:
    """Blocking loop: calls on_param(name, value, index, total) for EVERY
    PARAM_VALUE seen - unfiltered. Runs until stop_check() returns True (if
    given) or forever.

    Deliberately does not filter by name here: a caller that only wants to
    print (say) FENCE_* params still needs to see every param's index/total
    to correctly detect "the initial full-list dump just finished" - if
    filtering happened in here, that moment would depend on whether the
    LAST param in the vehicle's full list happens to match the filter,
    which it usually won't. Filter inside on_param instead.
    """
    if request_all_first:
        vehicle.master.mav.param_request_list_send(vehicle.system_id, vehicle.component_id)

    while stop_check is None or not stop_check():
        msg = vehicle.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.0)
        if msg is None:
            continue
        name = _param_id_str(msg.param_id)
        on_param(name, float(msg.param_value), int(msg.param_index), int(msg.param_count))


def _param_id_str(param_id) -> str:
    if isinstance(param_id, bytes):
        param_id = param_id.decode("utf-8", errors="replace")
    return param_id.rstrip("\x00")


# --- Geofence (polygon, via the MISSION protocol's FENCE mission type) --

# FenceVertex/load_fence_csv live in fence_csv.py (no pymavlink dependency,
# so a GCS-only script can use them without installing the MAVLink stack).
# Re-exported here so existing `from mavlink_param_link import FenceVertex,
# load_fence_csv` imports keep working unchanged.
from fence_csv import FenceVertex, load_fence_csv, same_point as _same_point  # noqa: E402


def _addressed_to_us(vehicle: Vehicle, msg) -> bool:
    """ArduPilot addresses every mission/fence reply to whoever asked. Through
    MAVProxy, the replies to other programs (Mission Planner reading the
    fence, MAVProxy's own fence module...) reach this connection too - mixing
    those in gave false 'verify MISMATCH' results."""
    mav = vehicle.master.mav
    target = (getattr(msg, "target_system", mav.srcSystem), getattr(msg, "target_component", mav.srcComponent))
    return target == (mav.srcSystem, mav.srcComponent)


def upload_fence_polygon(
    vehicle: Vehicle,
    vertices: List[FenceVertex],
    inclusion: bool = True,
    timeout_s: float = DEFAULT_TIMEOUT_S,
    max_retries: int = 3,
) -> Tuple[bool, str]:
    """Upload a single polygon fence, replacing whatever fence is currently
    on the vehicle (ArduPilot's fence mission type is not incremental - a
    fresh MISSION_COUNT upload replaces the previous contents).

    Standard MAVLink mission protocol handshake:
      GCS --MISSION_COUNT(mission_type=FENCE)--> vehicle
      vehicle --MISSION_REQUEST_INT(seq=i)--> GCS   (repeated per item)
      GCS --MISSION_ITEM_INT(seq=i)--> vehicle
      vehicle --MISSION_ACK--> GCS   (once all items received)

    A trailing vertex equal to the first is treated as an explicit "close
    the polygon" marker and dropped before upload - ArduPilot auto-closes
    polygons itself (connects the last vertex back to the first), so
    uploading it again as a real vertex would be redundant.
    """
    if len(vertices) >= 2 and _same_point(vertices[0], vertices[-1]):
        vertices = vertices[:-1]

    if len(vertices) < 3:
        return False, f"need at least 3 distinct vertices for a polygon, got {len(vertices)}"

    command = (
        mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION
        if inclusion
        else mavutil.mavlink.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION
    )
    mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    count = len(vertices)

    for attempt in range(max_retries):
        vehicle.master.mav.mission_count_send(
            vehicle.system_id, vehicle.component_id, count, mission_type
        )

        sent = set()
        deadline = time.time() + timeout_s
        acked = False
        ack_result = None

        while time.time() < deadline:
            msg = vehicle.master.recv_match(
                type=["MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"], blocking=True, timeout=1.0
            )
            if msg is None:
                continue
            if getattr(msg, "mission_type", mission_type) != mission_type:
                continue
            if not _addressed_to_us(vehicle, msg):
                continue

            if msg.get_type() == "MISSION_ACK":
                acked = True
                ack_result = msg.type
                break

            seq = msg.seq
            if seq >= count:
                continue
            vertex = vertices[seq]
            vehicle.master.mav.mission_item_int_send(
                vehicle.system_id,
                vehicle.component_id,
                seq,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                command,
                0,  # current
                0,  # autocontinue
                float(count),  # param1: vertex count of THIS polygon
                0, 0, 0,  # param2-4 unused for fence vertices
                int(round(vertex.lat * 1e7)),
                int(round(vertex.lon * 1e7)),
                0.0,  # z unused
                mission_type,
            )
            sent.add(seq)
            deadline = time.time() + timeout_s  # reset the timeout on progress

        if acked:
            if ack_result == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                return True, f"fence uploaded: {count} vertices accepted"
            return False, f"vehicle rejected the fence (MAV_MISSION_ACK type={ack_result})"

        # Timed out with no ACK - retry the whole upload from scratch.

    return False, f"no MISSION_ACK after {max_retries} attempt(s) - upload did not complete"


def clear_fence(vehicle: Vehicle, timeout_s: float = DEFAULT_TIMEOUT_S) -> Tuple[bool, str]:
    """Delete every fence item on the vehicle (MISSION_CLEAR_ALL for the fence
    list only - the mission and rally points stay). Returns (ok, message)."""
    mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    vehicle.master.mav.mission_clear_all_send(vehicle.system_id, vehicle.component_id, mission_type)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        msg = vehicle.master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)
        if (msg is None or getattr(msg, "mission_type", mission_type) != mission_type
                or not _addressed_to_us(vehicle, msg)):
            continue
        if msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            return True, "fence cleared"
        return False, f"vehicle refused to clear the fence (MAV_MISSION_ACK type={msg.type})"
    return False, "no MISSION_ACK - fence clear timed out"


def download_fence(vehicle: Vehicle, timeout_s: float = DEFAULT_TIMEOUT_S) -> List[FenceVertex]:
    """Read back whatever polygon fence is currently on the vehicle, for
    verifying an upload actually took effect as intended."""
    return [FenceVertex(lat=lat, lon=lon) for _command, _param1, lat, lon in download_fence_items(vehicle, timeout_s)]


def download_fence_items(vehicle: Vehicle, timeout_s: float = DEFAULT_TIMEOUT_S) -> List[Tuple[int, float, float, float]]:
    """Every fence item on the vehicle as (command, param1, lat, lon) - the
    command tells inclusion/exclusion polygon vertices, circles and the
    return point apart (param1: a polygon's vertex count, a circle's radius)."""
    mission_type = mavutil.mavlink.MAV_MISSION_TYPE_FENCE
    vehicle.master.mav.mission_request_list_send(vehicle.system_id, vehicle.component_id, mission_type)

    deadline = time.time() + timeout_s
    count = None
    while time.time() < deadline:
        msg = vehicle.master.recv_match(type="MISSION_COUNT", blocking=True, timeout=1.0)
        if (msg is not None and getattr(msg, "mission_type", mission_type) == mission_type
                and _addressed_to_us(vehicle, msg)):
            count = msg.count
            break
    if count is None:
        raise MavlinkLinkError("no MISSION_COUNT response - fence download timed out")
    if count == 0:
        return []

    items = {}
    for seq in range(count):
        vehicle.master.mav.mission_request_int_send(vehicle.system_id, vehicle.component_id, seq, mission_type)
        item_deadline = time.time() + timeout_s
        while time.time() < item_deadline:
            msg = vehicle.master.recv_match(type="MISSION_ITEM_INT", blocking=True, timeout=1.0)
            if (msg is not None and getattr(msg, "mission_type", mission_type) == mission_type and msg.seq == seq
                    and _addressed_to_us(vehicle, msg)):
                items[seq] = (msg.command, msg.param1, msg.x / 1e7, msg.y / 1e7)
                break
        else:
            raise MavlinkLinkError(f"no MISSION_ITEM_INT for seq={seq} - fence download incomplete")

    vehicle.master.mav.mission_ack_send(
        vehicle.system_id, vehicle.component_id, mavutil.mavlink.MAV_MISSION_ACCEPTED, mission_type
    )
    return [items[i] for i in range(count)]
