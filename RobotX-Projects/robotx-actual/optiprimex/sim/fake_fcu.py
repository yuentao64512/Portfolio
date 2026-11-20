#!/usr/bin/env python3
"""fake_fcu: a kinematic fake ArduRover (omni-usv-v2 flavour) for the laptop bench.

Lineage: control_v2 bench fake_fcu.py (frozen checkpoint) (message handling) + a planar plant so GUIDED targets and
mode-20 frames move the boat. Not a simulator of ArduPilot; it answers what the companion uses:

  HEARTBEAT, PARAM_*, SET_MESSAGE_INTERVAL, DO_SET_MODE / SET_MODE (0 MANUAL, 4 HOLD, 15 GUIDED,
  20 CUSTOM_MANUAL, 17 with --custom), DO_CHANGE_SPEED, SET_ACTUATOR_CONTROL_TARGET (mode 20, armed,
  250 ms timeout), SET_POSITION_TARGET_GLOBAL_INT / LOCAL_NED (GUIDED), COMPONENT_ARM_DISARM (v0.1.50:
  as ArduPilot - DISARM accepted, ARM refused while the kill / E-stop / RC5 is on; was refused always before the pages' ARM button).

Streams: HEARTBEAT, GLOBAL_POSITION_INT, ATTITUDE, GPS_RAW_INT, SYS_STATUS, RC_CHANNELS,
SERVO_OUTPUT_RAW, EKF_STATUS_REPORT, NAMED_VALUE_INT KILL/ESTOP, STATUSTEXT; FENCE_STATUS 1 Hz while a fence is loaded.

v0.1.53 the geofence: the MAVLink mission protocol for mission_type FENCE - upload (MISSION_COUNT -> MISSION_REQUEST_INT ->
MISSION_ITEM_INT -> MISSION_ACK; inclusion polygon vertices only, param1 = the vertex count, else MAV_MISSION_UNSUPPORTED /
INVALID_PARAM1), read-back (MISSION_REQUEST_LIST -> MISSION_COUNT -> MISSION_REQUEST(_INT) -> MISSION_ITEM_INT), MISSION_CLEAR_ALL;
MAV_CMD_DO_FENCE_ENABLE with COMMAND_ACK (on with no polygon = FAILED: the fake's choice); SYS_STATUS MAV_SYS_STATUS_GEOFENCE
(present = a polygon, enabled, healthy = no breach). A breach (enabled, the plant outside the polygon) is reported in FENCE_STATUS
and, when armed, switches to HOLD once per breach - as ArduPilot Rover with FENCE_ACTION 2 (Hold) would; nothing else is enforced.
Control: {"set": {"fence_reject": 4}} = answer the next upload with that MAV_MISSION result (a test hook), {"get": "state"} has fence.

Control channel (UDP JSON lines on --control-port):
  {"set": {"kill": true}}            {"set": {"estop": false}}      {"set": {"armed": true}}
  {"set": {"rc6": 1900}}             {"set": {"rc5": 1000}}         {"set": {"mode": 0}}   (RC8 position)
  {"set": {"ekf_ok": false}}  (or "rel": a relative-only EKF, v0.1.54.1)         {"set": {"gps_fix": 0}}        {"set": {"rc_lost": true}}
                                     (v0.1.68: gps_fix < 3 = no GLOBAL_POSITION_INT, GPS_RAW_INT lat / lon 0, GUIDED refused - ArduRover)
  {"set": {"rc_sys": "healthy"}}     (SYS_STATUS RC_RECEIVER bit: absent (default) | healthy | unhealthy = an RC failsafe)
  {"set": {"rc_chancount": 0}}       (RC_CHANNELS.chancount; 0 = what the real Pixhawk sent through usv_ui's share, pool 2026-11-04)
  {"sticks": {"surge": 0.5, "yaw": -0.3, "sway": 0}}   (v0.1.32, the simulator's arrow keys: the transmitter's sticks, -1..1; they
                                     move the boat in MANUAL (mode 0) only, powered; 0.5 s without a new one = released = zero)
  {"pause_stream": 3.0}              {"teleport": {"lat": .., "lon": ..}}   {"get": "state"}
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import threading
import time
from typing import Dict, List, Optional

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

MAV = mavutil.mavlink
from .plant import R_EARTH, Plant, frame_uvr, guided_ur, velocity_ur  # noqa: E402,F401  (v0.1.68: moved to sim/plant.py, unchanged)
MODE_NAMES = {0: "MANUAL", 4: "HOLD", 15: "GUIDED", 17: "CUSTOM_AUTO", 20: "CUSTOM_MANUAL", 10: "AUTO", 5: "LOITER", 11: "RTL"}

BASE_PARAMS = {
    "FRAME_CLASS": 2, "FRAME_TYPE": 5, "SERVO1_FUNCTION": 33, "SERVO2_FUNCTION": 34, "SERVO3_FUNCTION": 35,
    "SERVO4_FUNCTION": 36, "MOT_PWM_TYPE": 0, "MOT_THR_MAX": 100, "MOT_THR_MIN": 0, "MOT_SLEWRATE": 50,
    "MODE_CH": 8, "MODE1": 0, "MODE2": 0, "MODE3": 0, "MODE4": 4, "MODE5": 0, "MODE6": 15, "INITIAL_MODE": 4,
    "FS_THR_ENABLE": 0, "FS_GCS_ENABLE": 0, "ARMING_REQUIRE": 1, "WP_SPEED": 2, "CRUISE_SPEED": 2,
    "AHRS_EKF_TYPE": 3, "EK3_SRC1_POSXY": 3, "MAV_SYSID": 1, "NET_ENABLE": 0,
}


class FakeFcu:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.conns = [mavutil.mavlink_connection(a.conn, source_system=1, source_component=1)]
        for p in [x for x in (a.mirror or "").split(",") if x.strip()]:
            self.conns.append(mavutil.mavlink_connection(f"udpout:127.0.0.1:{int(p)}", source_system=1, source_component=1))
        self.plant = Plant(a.lat, a.lon, a.hdg, hull=getattr(a, "hull", "ideal"), str_ramp_s=getattr(a, "str_ramp_s", 0.0))
        self.mode = int(a.mode)
        self.armed = bool(a.armed)
        self.kill = self.estop = False
        self.ekf_ok = True
        self.gps_fix = int(getattr(a, "gps_fix", 3))   # v0.1.68 (F9): --gps-fix N; < 3 = no position sent, GUIDED refused
        self.rc_lost = False
        self.rc_sys = "absent"          # v0.1.29.3: SYS_STATUS RC_RECEIVER bit - absent | healthy | unhealthy
        self.rc_chancount = 16
        self.sticks = (0.0, 0.0, 0.0)   # v0.1.32: surge, sway, yaw from the simulator's arrow keys (MANUAL only)
        self.sticks_t = -1e9
        self.rc = {i: 1500 for i in range(1, 17)}
        self.rc[5] = 1000  # RC5 SF motor E-stop (high = stop)
        self.rc[6] = 1000  # RC6 MANUAL override (high = override)
        self.rc[8] = 1500  # RC8 mode switch position
        self.rc[9] = 1000  # RC9 arm
        self.batt_v = 16.2
        self.wp_speed = float(BASE_PARAMS["WP_SPEED"])
        self.frame = [0.0, 0.0, 0.0, 0.0]
        self.frame_t = 0.0
        self.guided_target = None          # (lat, lon)
        self.vel_target = None             # (vx, yaw_rate)
        self.vel_target_t = 0.0
        self.intervals: Dict[int, float] = {MAV.MAVLINK_MSG_ID_HEARTBEAT: 1.0}
        self.next_due: Dict[int, float] = {}
        self.pause_until = 0.0
        self.params = dict(BASE_PARAMS)
        self.counters = {"rx": 0, "arm_requests": 0, "frames": 0, "mode_changes": 0, "targets": 0}
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.ctrl: Optional[socket.socket] = None
        self.last_ctrl_addr = None
        self.fence: List = []              # v0.1.53: the inclusion polygon [(lat, lon), ...] (empty = no fence)
        self.fence_rx: Optional[dict] = None   # an upload in progress {n, items, src}
        self.fence_enabled = False
        self.fence_breach = 0
        self.fence_breach_count = 0
        self.fence_breach_t = 0
        self.fence_reject: Optional[int] = None   # test hook: the next upload is answered with this MAV_MISSION result
        self.fence_status_due = 0.0

    # ---- helpers --------------------------------------------------------------------------
    def log(self, msg: str) -> None:
        print(f"[fake_fcu {time.monotonic() - self.t0:7.2f}] {msg}", flush=True)

    def boot_ms(self) -> int:
        return int((time.monotonic() - self.t0) * 1000) & 0xFFFFFFFF

    def send(self, fn, *args) -> None:
        for c in self.conns:
            try:
                getattr(c.mav, fn)(*args)
            except Exception as e:  # pragma: no cover
                self.log(f"send {fn} failed: {e}")

    def statustext(self, text: str, severity: int = MAV.MAV_SEVERITY_INFO) -> None:
        self.send("statustext_send", severity, text.encode()[:50])

    def set_mode(self, mode: int, why: str = "") -> bool:
        allowed = {0, 4, 15, 10, 5, 11}
        if self.a.custom:
            allowed |= {17, 20}
        if mode not in allowed:
            return False
        if mode == 15 and self.gps_fix < 3:            # v0.1.68 (WP17 F9): as ArduRover - GUIDED needs a position estimate
            self.log(f"mode GUIDED refused: no position (gps_fix {self.gps_fix}) {why}")
            return False
        if mode != self.mode:
            self.counters["mode_changes"] += 1
            self.log(f"mode {MODE_NAMES.get(self.mode, self.mode)} -> {MODE_NAMES.get(mode, mode)} {why}")
            self.mode = mode
            self.guided_target = None
            self.vel_target = None
            self.statustext(f"Mode {MODE_NAMES.get(mode, mode)}")
        return True

    # ---- control channel ----------------------------------------------------------------------
    def control_loop(self) -> None:
        self.ctrl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl.bind(("127.0.0.1", self.a.control_port))
        self.ctrl.settimeout(0.2)
        while not self.stop.is_set():
            try:
                data, addr = self.ctrl.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break
            for line in data.decode(errors="ignore").splitlines():
                if not line.strip():
                    continue
                try:
                    msg = json.loads(line)
                except Exception:
                    continue
                reply = self.apply_control(msg)
                if reply is not None:
                    try:
                        self.ctrl.sendto(json.dumps(reply).encode(), addr)
                    except OSError:
                        pass

    def apply_control(self, msg: dict) -> Optional[dict]:
        with self.lock:
            if "set" in msg:
                s = msg["set"]
                if "kill" in s:
                    self.kill = bool(s["kill"])
                if "estop" in s:
                    self.estop = bool(s["estop"])
                if "armed" in s:
                    self.armed = bool(s["armed"])
                    self.log(f"armed={self.armed} (control)")
                if "rc5" in s:
                    self.rc[5] = int(s["rc5"])
                if "rc6" in s:
                    self.rc[6] = int(s["rc6"])
                if "rc8" in s:
                    self.rc[8] = int(s["rc8"])
                if "mode" in s:
                    self.set_mode(int(s["mode"]), "(RC8 / control)")
                if "ekf_ok" in s:
                    self.ekf_ok = "rel" if s["ekf_ok"] == "rel" else bool(s["ekf_ok"])   # v0.1.54.1: "rel" = relative-only
                if "gps_fix" in s:
                    self.gps_fix = int(s["gps_fix"])
                if "rc_lost" in s:
                    self.rc_lost = bool(s["rc_lost"])
                if s.get("rc_sys") in ("absent", "healthy", "unhealthy"):
                    self.rc_sys = s["rc_sys"]
                if "rc_chancount" in s:
                    self.rc_chancount = int(s["rc_chancount"])
                if "batt_v" in s:
                    self.batt_v = float(s["batt_v"])
                if "wp_speed" in s:
                    self.wp_speed = float(s["wp_speed"])
                if "fence_reject" in s:
                    self.fence_reject = None if s["fence_reject"] is None else int(s["fence_reject"])
                return {"ok": True}
            if "pause_stream" in msg:
                self.pause_until = time.monotonic() + float(msg["pause_stream"])
                return {"ok": True}
            if "teleport" in msg:
                t = msg["teleport"]
                self.plant.x, self.plant.y = self.plant.to_local(float(t["lat"]), float(t["lon"]))
                if "hdg" in t:
                    self.plant.psi = math.radians(float(t["hdg"]))
                self.plant.u = self.plant.v = self.plant.r = 0.0
                return {"ok": True}
            if "sticks" in msg:
                st = msg["sticks"] or {}
                clip = lambda x: max(-1.0, min(1.0, float(x or 0.0)))
                self.sticks = (clip(st.get("surge")), clip(st.get("sway")), clip(st.get("yaw")))
                self.sticks_t = time.monotonic()
                return {"ok": True, "mode": self.mode}
            if msg.get("get") == "state":
                return self.state_dict()
        return None

    def state_dict(self) -> dict:
        p = self.plant
        return {"lat": p.lat, "lon": p.lon, "hdg_deg": p.hdg_deg, "u": p.u, "v": p.v, "r": p.r,
                "x_n": p.x, "y_e": p.y, "mode": self.mode, "mode_name": MODE_NAMES.get(self.mode, str(self.mode)),
                "armed": self.armed, "kill": self.kill, "estop": self.estop, "ekf_ok": self.ekf_ok,
                "gps_fix": self.gps_fix, "rc6": self.rc[6], "rc5": self.rc[5], "guided_target": self.guided_target,
                "counters": dict(self.counters), "frame": list(self.frame), "wp_speed": self.wp_speed,
                "sticks": list(self.sticks) if time.monotonic() - self.sticks_t <= 0.5 else [0.0, 0.0, 0.0],
                "fence": {"points": [list(p) for p in self.fence], "enabled": self.fence_enabled, "breach": self.fence_breach,
                          "breach_count": self.fence_breach_count}}

    # ---- MAVLink rx ------------------------------------------------------------------------------
    def rx_loop(self) -> None:
        main = self.conns[0]
        while not self.stop.is_set():
            try:
                msg = main.recv_match(blocking=True, timeout=0.1)
            except Exception:
                time.sleep(0.02)
                continue
            if msg is None:
                continue
            self.counters["rx"] += 1
            with self.lock:
                self.handle(msg)

    def handle(self, msg) -> None:
        t = msg.get_type()
        if t == "COMMAND_LONG":
            cmd = int(msg.command)
            if cmd == MAV.MAV_CMD_SET_MESSAGE_INTERVAL:
                mid, us = int(msg.param1), float(msg.param2)
                self.intervals[mid] = (1e6 / us) if us > 0 else 0.0
                self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED)
            elif cmd == MAV.MAV_CMD_DO_SET_MODE:
                ok = self.set_mode(int(msg.param2), "(GCS)")
                self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED if ok else MAV.MAV_RESULT_FAILED)
            elif cmd == MAV.MAV_CMD_DO_CHANGE_SPEED:
                self.wp_speed = float(msg.param2)
                self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED)
            elif cmd == MAV.MAV_CMD_COMPONENT_ARM_DISARM:
                self.counters["arm_requests"] += 1
                on = float(msg.param1) >= 0.5
                if on and (self.kill or self.estop or self.rc[5] >= 1700):
                    self.log("ARM refused: the kill / E-stop is on")
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_DENIED)
                else:
                    self.armed = on
                    self.log("ARMED (a MAVLink request)" if on else "DISARMED (a MAVLink request)")
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED)
            elif cmd == MAV.MAV_CMD_DO_FENCE_ENABLE:                 # v0.1.53: 1 on, 0 off, 2 floor off (no floor on a boat)
                p1 = int(round(float(msg.param1)))
                if p1 == 1 and not self.fence:
                    self.log("fence on refused: no polygon loaded")
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_FAILED)
                elif p1 in (0, 1):
                    self.fence_enabled = p1 == 1
                    if not self.fence_enabled:
                        self.fence_breach = 0
                    self.log(f"fence {'ENABLED' if self.fence_enabled else 'disabled'} (DO_FENCE_ENABLE)")
                    self.statustext(f"Fence {'enabled' if self.fence_enabled else 'disabled'}")
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED)
                elif p1 == 2:
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED)
                else:
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_DENIED)
            elif cmd == MAV.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
                self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED)
            elif cmd == MAV.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:     # v0.1.28: as ArduPilot - refused armed, else silent ~3 s, HOLD
                if self.armed or int(msg.param1) != 1:
                    self.log("reboot refused (armed)" if self.armed else "shutdown refused (only reboot)")
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_DENIED)
                else:
                    self.counters["reboots"] = self.counters.get("reboots", 0) + 1
                    self.log("REBOOT: stream off 3 s, then HOLD")
                    self.send("command_ack_send", cmd, MAV.MAV_RESULT_ACCEPTED)
                    self.pause_until = time.monotonic() + 3.0
                    self.mode = 4
                    self.frame, self.guided_target, self.vel_target = [0.0, 0.0, 0.0, 0.0], None, None
            else:
                self.send("command_ack_send", cmd, MAV.MAV_RESULT_UNSUPPORTED)
        elif t == "SET_MODE":
            ok = self.set_mode(int(msg.custom_mode), "(SET_MODE)")
            self.send("command_ack_send", MAV.MAV_CMD_DO_SET_MODE, MAV.MAV_RESULT_ACCEPTED if ok else MAV.MAV_RESULT_FAILED)
        elif t == "SET_ACTUATOR_CONTROL_TARGET":
            self.counters["frames"] += 1
            if self.mode == 20:
                self.frame = [float(x) for x in msg.controls[:4]]
                self.frame_t = time.monotonic()
        elif t == "SET_POSITION_TARGET_GLOBAL_INT":
            self.counters["targets"] += 1
            if self.mode == 15:
                self.guided_target = (msg.lat_int / 1e7, msg.lon_int / 1e7)
                self.vel_target = None
        elif t == "SET_POSITION_TARGET_LOCAL_NED":
            self.counters["targets"] += 1
            if self.mode == 15:
                self.vel_target = (float(msg.vx), float(msg.yaw_rate))
                self.vel_target_t = time.monotonic()
                self.guided_target = None
        elif t in ("MISSION_COUNT", "MISSION_ITEM_INT", "MISSION_REQUEST_LIST", "MISSION_REQUEST_INT", "MISSION_REQUEST",
                   "MISSION_CLEAR_ALL", "MISSION_ACK"):
            self.fence_msg(t, msg)
        elif t == "PARAM_REQUEST_LIST":
            items = list(self.params.items())
            for i, (k, v) in enumerate(items):
                self.send("param_value_send", k.encode()[:16], float(v), MAV.MAV_PARAM_TYPE_REAL32, len(items), i)
        elif t == "PARAM_REQUEST_READ":
            name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            name = name.rstrip("\x00")
            if name in self.params:
                idx = list(self.params).index(name)
                self.send("param_value_send", name.encode()[:16], float(self.params[name]), MAV.MAV_PARAM_TYPE_REAL32, len(self.params), idx)
        elif t == "PARAM_SET":
            name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode(errors="ignore")
            name = name.rstrip("\x00")
            self.params[name] = float(msg.param_value)
            idx = list(self.params).index(name)
            self.send("param_value_send", name.encode()[:16], float(msg.param_value), MAV.MAV_PARAM_TYPE_REAL32, len(self.params), idx)

    # ---- v0.1.53: the fence (mission_type FENCE) ------------------------------------------------------------------------------
    def fence_msg(self, t: str, msg) -> None:
        if int(getattr(msg, "mission_type", 0) or 0) != MAV.MAV_MISSION_TYPE_FENCE:
            return                                           # missions / rally: not this fake's business (no answer, as before)
        src = (msg.get_srcSystem(), msg.get_srcComponent())   # every answer goes to whoever asked
        F = MAV.MAV_MISSION_TYPE_FENCE
        if t == "MISSION_COUNT":
            n = int(msg.count)
            if n == 0:
                self.fence, self.fence_rx, self.fence_breach = [], None, 0
                self.send("mission_ack_send", src[0], src[1], MAV.MAV_MISSION_ACCEPTED, F)
                return
            self.fence_rx = {"n": n, "items": {}, "src": src}
            self.send("mission_request_int_send", src[0], src[1], 0, F)
        elif t == "MISSION_ITEM_INT":
            rx = self.fence_rx
            if rx is None:
                return
            seq, n = int(msg.seq), rx["n"]
            if int(msg.command) != MAV.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
                res = MAV.MAV_MISSION_UNSUPPORTED                # the fake holds one inclusion polygon, nothing else
            elif int(round(float(msg.param1))) != n:
                res = MAV.MAV_MISSION_INVALID_PARAM1             # param1 = the polygon's vertex count
            elif seq != len(rx["items"]):
                self.send("mission_request_int_send", src[0], src[1], len(rx["items"]), F)   # out of order: ask again
                return
            else:
                rx["items"][seq] = (msg.x / 1e7, msg.y / 1e7)
                if len(rx["items"]) < n:
                    self.send("mission_request_int_send", src[0], src[1], seq + 1, F)
                    return
                res = self.fence_reject if self.fence_reject is not None else MAV.MAV_MISSION_ACCEPTED
                self.fence_reject = None
                if res == MAV.MAV_MISSION_ACCEPTED:
                    self.fence = [rx["items"][i] for i in range(n)]
                    self.fence_breach = 0
                    self.log(f"fence uploaded: {n} points")
            self.fence_rx = None
            self.send("mission_ack_send", src[0], src[1], res, F)
        elif t == "MISSION_REQUEST_LIST":
            self.send("mission_count_send", src[0], src[1], len(self.fence), F)
        elif t in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            seq = int(msg.seq)
            if 0 <= seq < len(self.fence):
                lat, lon = self.fence[seq]
                self.send("mission_item_int_send", src[0], src[1], seq, MAV.MAV_FRAME_GLOBAL, MAV.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION,
                          0, 0, float(len(self.fence)), 0, 0, 0, int(round(lat * 1e7)), int(round(lon * 1e7)), 0.0, F)
            else:
                self.send("mission_ack_send", src[0], src[1], MAV.MAV_MISSION_INVALID_SEQUENCE, F)
        elif t == "MISSION_CLEAR_ALL":
            self.fence, self.fence_rx, self.fence_breach, self.fence_enabled = [], None, 0, False
            self.send("mission_ack_send", src[0], src[1], MAV.MAV_MISSION_ACCEPTED, F)

    def fence_inside(self) -> bool:
        pts, p = self.fence, self.plant
        y, x = p.lat, p.lon
        c = False
        for i in range(len(pts)):
            (y1, x1), (y2, x2) = pts[i], pts[i - 1]
            if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
                c = not c
        return c

    def fence_check(self, now: float) -> None:
        """A breach = enabled, a polygon, the plant outside it: FENCE_STATUS reports it; armed -> HOLD once per breach (FENCE_ACTION 2)."""
        if not (self.fence_enabled and len(self.fence) >= 3):
            self.fence_breach = 0
            return
        out = not self.fence_inside()
        if out and not self.fence_breach:
            self.fence_breach, self.fence_breach_t = 1, self.boot_ms()
            self.fence_breach_count = (self.fence_breach_count + 1) & 0xFFFF
            self.statustext("Fence Breached", MAV.MAV_SEVERITY_CRITICAL)
            self.log("FENCE BREACH" + (" -> HOLD (FENCE_ACTION hold)" if self.armed else " (disarmed: report only)"))
            if self.armed:
                self.set_mode(4, "(fence breach)")
        elif not out and self.fence_breach:
            self.fence_breach = 0
            self.log("fence breach cleared")

    # ---- plant + streams --------------------------------------------------------------------------
    def physics(self, dt: float) -> None:
        p = self.plant
        u_t = v_t = r_t = 0.0
        now = time.monotonic()
        powered = self.armed and not self.kill and not self.estop and self.rc[5] < 1700
        if powered:
            if self.mode == 0:                     # v0.1.32: MANUAL = the transmitter's sticks (the simulator's arrow keys)
                if now - self.sticks_t <= 0.5:
                    su, sv, sr = self.sticks
                    u_t, v_t, r_t = su * Plant.U_MAX * 0.6, sv * Plant.V_MAX, sr * Plant.R_MAX
            elif self.mode == 20:
                if now - self.frame_t <= 0.25:
                    u_t, v_t, r_t = frame_uvr(self.frame)
            elif self.mode == 15:
                if self.guided_target is not None:
                    u_t, r_t = guided_ur(p, self.guided_target, self.wp_speed)
                elif self.vel_target is not None and now - self.vel_target_t <= 3.0:
                    u_t, r_t = velocity_ur(self.vel_target)
        p.step(dt, u_t, v_t, r_t)

    def stream(self, now: float) -> None:
        if now < self.pause_until:
            return
        defaults = {MAV.MAVLINK_MSG_ID_HEARTBEAT: 1, MAV.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 4,
                    MAV.MAVLINK_MSG_ID_ATTITUDE: 4, MAV.MAVLINK_MSG_ID_GPS_RAW_INT: 1,
                    MAV.MAVLINK_MSG_ID_SYS_STATUS: 1, MAV.MAVLINK_MSG_ID_RC_CHANNELS: 2,
                    MAV.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 2, MAV.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 1,
                    MAV.MAVLINK_MSG_ID_NAMED_VALUE_INT: 2}
        self.fence_check(now)
        if self.fence and now >= self.fence_status_due:       # v0.1.53: FENCE_STATUS 1 Hz while a polygon is loaded
            self.fence_status_due = now + 1.0
            self.send("fence_status_send", 1 if self.fence_breach else 0, self.fence_breach_count,
                      MAV.FENCE_BREACH_BOUNDARY if self.fence_breach else MAV.FENCE_BREACH_NONE, self.fence_breach_t)
        for mid, hz_default in defaults.items():
            hz = self.intervals.get(mid, hz_default)
            if hz <= 0:
                continue
            if now >= self.next_due.get(mid, 0.0):
                self.next_due[mid] = now + 1.0 / hz
                self.emit(mid)

    def emit(self, mid: int) -> None:
        p = self.plant
        ms = self.boot_ms()
        if mid == MAV.MAVLINK_MSG_ID_HEARTBEAT:
            base = MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (MAV.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
            self.send("heartbeat_send", MAV.MAV_TYPE_SURFACE_BOAT, MAV.MAV_AUTOPILOT_ARDUPILOTMEGA, base, self.mode,
                      MAV.MAV_STATE_ACTIVE if self.armed else MAV.MAV_STATE_STANDBY)
        elif mid == MAV.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            if self.gps_fix < 3:                       # v0.1.68 (F9): no fix = no position (the bench indoors)
                return
            vn, ve = p.vn_ve
            self.send("global_position_int_send", ms, int(p.lat * 1e7), int(p.lon * 1e7), 0, 0,
                      int(vn * 100), int(ve * 100), 0, int(p.hdg_deg * 100))
        elif mid == MAV.MAVLINK_MSG_ID_ATTITUDE:
            self.send("attitude_send", ms, 0.0, 0.0, p.psi if p.psi <= math.pi else p.psi - 2 * math.pi, 0.0, 0.0, p.r)
        elif mid == MAV.MAVLINK_MSG_ID_GPS_RAW_INT:
            fix = self.gps_fix >= 3                    # v0.1.68 (F9): lat / lon 0 without a fix, as a receiver with none
            self.send("gps_raw_int_send", ms * 1000, self.gps_fix, int(p.lat * 1e7) if fix else 0, int(p.lon * 1e7) if fix else 0, 0, 120, 150,
                      int(math.hypot(*p.vn_ve) * 100), int(p.hdg_deg * 100), 14 if self.gps_fix >= 3 else 3)
        elif mid == MAV.MAVLINK_MSG_ID_SYS_STATUS:
            bit = 0x10000 if self.rc_sys != "absent" else 0          # MAV_SYS_STATUS_SENSOR_RC_RECEIVER
            g = MAV.MAV_SYS_STATUS_GEOFENCE                           # v0.1.53: present = a polygon, enabled, healthy = no breach
            gp = g if self.fence else 0
            ge = g if (self.fence and self.fence_enabled) else 0
            gh = g if (self.fence and not self.fence_breach) else 0
            self.send("sys_status_send", bit | gp, bit | ge, (bit if self.rc_sys == "healthy" else 0) | gh, 120, int(self.batt_v * 1000), -1, 80,
                      0, 0, 0, 0, 0, 0)
        elif mid == MAV.MAVLINK_MSG_ID_RC_CHANNELS:
            if self.rc_lost:
                return
            ch = [self.rc.get(i, 0) for i in range(1, 19)]
            self.send("rc_channels_send", ms, self.rc_chancount, *ch, 255)
        elif mid == MAV.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            pwm = [int(1500 + 400 * x) for x in self.frame] + [1500] * 4
            self.send("servo_output_raw_send", ms * 1000, 0, *pwm)
        elif mid == MAV.MAVLINK_MSG_ID_EKF_STATUS_REPORT:
            # v0.1.33: no GPS fix = no EKF position, as on the real Pixhawk (the sim day: GPS lost gave no EKF_BAD)
            # v0.1.54.1: the real bits - POS_HORIZ_REL 0x08 + POS_HORIZ_ABS 0x10 + PRED_POS_HORIZ_REL 0x100 + PRED_POS_HORIZ_ABS 0x200
            # (the old 0x10 | 0x100 hid the link's wrong constant); ekf "rel" = a relative-only EKF (no absolute position)
            if self.ekf_ok == "rel":
                flags = 0x01 | 0x02 | 0x04 | 0x08 | 0x100
            else:
                flags = (0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x100 | 0x200) if (self.ekf_ok and self.gps_fix >= 3) else 0x01
            self.send("ekf_status_report_send", flags, 0.1, 0.1, 0.1, 0.1, 0.0)
        elif mid == MAV.MAVLINK_MSG_ID_NAMED_VALUE_INT:
            self.send("named_value_int_send", ms, b"KILL", 1 if self.kill else 0)
            self.send("named_value_int_send", ms, b"ESTOP", 1 if self.estop else 0)

    def run(self) -> None:
        self.log(f"fake FCU on {self.a.conn} custom={self.a.custom} armed={self.armed} mode={MODE_NAMES.get(self.mode)} "
                 f"control udp:{self.a.control_port}")
        threading.Thread(target=self.rx_loop, name="fcu-rx", daemon=True).start()
        threading.Thread(target=self.control_loop, name="fcu-ctrl", daemon=True).start()
        dt = 0.05
        last = time.monotonic()
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                with self.lock:
                    self.physics(min(now - last, 0.2))
                    last = now
                    self.stream(now)
                time.sleep(dt)
        except KeyboardInterrupt:
            pass
        finally:
            self.stop.set()


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--conn", default="udpout:127.0.0.1:14550")
    ap.add_argument("--mirror", default="", help="comma-separated extra UDP ports to mirror the stream to (TUI 14551, logger 14553)")
    ap.add_argument("--control-port", type=int, default=14560)
    ap.add_argument("--custom", action="store_true", default=True, help="accept modes 17 and 20 (custom firmware)")
    ap.add_argument("--no-custom", dest="custom", action="store_false")
    ap.add_argument("--armed", action="store_true")
    ap.add_argument("--mode", type=int, default=4, help="initial mode (4 HOLD, 0 MANUAL, 15 GUIDED)")
    ap.add_argument("--lat", type=float, default=1.2810202, help="start: Course Alpha's centre, Marina Bay (v0.1.33)")
    ap.add_argument("--lon", type=float, default=103.8550097)
    ap.add_argument("--hdg", type=float, default=0.0)
    ap.add_argument("--hull", default="ideal", choices=("ideal", "omni_usv"), help="v0.1.85 (WP41 T0.4): the plant's yaw authority - ideal "
                    "(45 deg/s) or omni_usv (the 11 Nov hull: <= 2 deg/s in place, ~4 deg/s at 0.5 m/s)")
    ap.add_argument("--str-ramp-s", type=float, default=0.0, help="v0.1.85: a first-order lag on the steering output (25 = the 11 Nov FCU ramp)")
    ap.add_argument("--gps-fix", type=int, default=3, help="v0.1.68: the start's GPS fix type; < 3 = no position sent and GUIDED refused "
                                                         "(the bench indoors; the control port's {\"set\": {\"gps_fix\": N}} switches it)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    FakeFcu(parse_args(argv)).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
