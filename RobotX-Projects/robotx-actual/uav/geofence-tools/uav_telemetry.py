"""
Collects the FCU telemetry RoboCommand's UAV heartbeat needs, from the
MAVLink messages geofence_udp_listener.py already receives, and turns it
into a plain dict the listener sends to robocommand_bridge.py on the GCS
laptop (which publishes it to RoboCommand as a Heartbeat).

state: STATE_AUTO while the FCU is in AUTO or GUIDED (the RC sits in GUIDED,
the mission flies AUTO), STATE_MANUAL in any other mode.
altitude_hae_m: height above the WGS84 ellipsoid, as RoboCommand wants it -
the EKF's altitude (MSL) corrected by the GPS's own ellipsoid/MSL difference.
Falls back to MSL, and says so, if the GPS doesn't report ellipsoid height.
"""

import math
import time
from typing import Optional

from pymavlink import mavutil

AUTO_MODES = {"AUTO", "GUIDED"}
HEARTBEAT_STALE_S = 3.0
AIRBORNE_REL_ALT_M = 1.0  # only used if the FCU doesn't send EXTENDED_SYS_STATE

# Streams this needs. Same rates the drone GUI / mission script ask for, so
# requesting them again (the FCU has one rate per message) changes nothing.
STREAM_RATES_HZ = {
    mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT: 5,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE: 10,
    mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 1,
}


def request_streams(master, system_id: int, component_id: int) -> None:
    for message_id, hz in STREAM_RATES_HZ.items():
        master.mav.command_long_send(
            system_id, component_id, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, 1_000_000.0 / hz, 0, 0, 0, 0, 0,
        )


class UavTelemetry:
    def __init__(self, system_id: int, is_autopilot_heartbeat):
        self.system_id = system_id
        self._is_autopilot_heartbeat = is_autopilot_heartbeat
        self.mode: Optional[str] = None
        self.armed = False
        self.heartbeat_time: Optional[float] = None
        self.position = None  # (lat, lon, alt_msl_m, rel_alt_m, vx, vy, hdg_cdeg)
        self.attitude = None  # (roll_deg, pitch_deg, yaw_deg)
        self.geoid_sep_m: Optional[float] = None  # ellipsoid height - MSL height, from the GPS
        self.landed_state: Optional[int] = None

    def update(self, msg) -> None:
        if msg.get_srcSystem() != self.system_id:
            return
        t = msg.get_type()
        if t == "HEARTBEAT":
            if self._is_autopilot_heartbeat(msg):
                try:
                    self.mode = mavutil.mode_string_v10(msg)
                except Exception:
                    self.mode = None
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.heartbeat_time = time.time()
        elif t == "GLOBAL_POSITION_INT":
            self.position = (msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0, msg.relative_alt / 1000.0,
                             msg.vx / 100.0, msg.vy / 100.0, msg.hdg)
        elif t == "ATTITUDE":
            self.attitude = (math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw))
        elif t == "GPS_RAW_INT":
            alt_ellipsoid = getattr(msg, "alt_ellipsoid", 0)  # MAVLink2 extension; 0 = not reported
            if alt_ellipsoid and msg.fix_type >= 3:
                self.geoid_sep_m = (alt_ellipsoid - msg.alt) / 1000.0
        elif t == "EXTENDED_SYS_STATE":
            self.landed_state = msg.landed_state

    def report(self, vehicle_id: str) -> Optional[dict]:
        """The heartbeat contents, or None while the FCU heartbeat is stale
        (better to send nothing than an out-of-date state)."""
        if self.heartbeat_time is None or time.time() - self.heartbeat_time > HEARTBEAT_STALE_S:
            return None
        out = {
            "type": "uav_telemetry",
            "vehicle_id": vehicle_id,
            "state": "STATE_AUTO" if self.mode in AUTO_MODES else "STATE_MANUAL",
            "mode": self.mode,
            "armed": self.armed,
            "flight_phase": "FLIGHT_PHASE_AIRBORNE" if self._airborne() else "FLIGHT_PHASE_GROUNDED",
            "timestamp": time.time(),
        }
        if self.position is not None and (self.position[0], self.position[1]) != (0.0, 0.0):
            lat, lon, alt_msl, _rel, vx, vy, hdg = self.position
            out.update(latitude=lat, longitude=lon, spd_mps=math.hypot(vx, vy))
            if hdg != 65535:
                out["heading_deg"] = hdg / 100.0
            elif self.attitude is not None:
                out["heading_deg"] = self.attitude[2] % 360.0
            if self.geoid_sep_m is not None:
                out.update(altitude_hae_m=alt_msl + self.geoid_sep_m, altitude_source="hae")
            else:
                out.update(altitude_hae_m=alt_msl, altitude_source="msl (GPS gives no ellipsoid height)")
        if self.attitude is not None:
            out.update(roll_deg=self.attitude[0], pitch_deg=self.attitude[1])
            out.setdefault("heading_deg", self.attitude[2] % 360.0)
        return out

    def _airborne(self) -> bool:
        if not self.armed:
            return False
        if self.landed_state is not None and self.landed_state != mavutil.mavlink.MAV_LANDED_STATE_UNDEFINED:
            return self.landed_state != mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
        return self.position is not None and self.position[3] > AIRBORNE_REL_ALT_M
