#!/usr/bin/env python3
"""usv_ui.py — pool-trial tool v4 = v3 (bursts, save/goto, path, swayto, on-demand logging, MAVLink inspector,
burst summaries, swaycal) + the global_v3 stack around it:

  * ONE FCU link holder: this tool opens the Pixhawk link and SHARES it on udp 14551 (rc_monitor's pattern: every
    FCU byte is re-sent to udpout:127.0.0.1:14551, whatever arrives there goes to the FCU), so the mission manager
    (optiprimex, MAVLink 200/191) runs at the same time through  mavlink.endpoint: udpin:127.0.0.1:14551;
  * MQTT on the boat broker (1884): detections from the AGX (local/usv/perception/#) go through the same PerceptionMap
    as the mission manager ('cv on'), the map is published as telemetry/usv/map (MAP tab here, map.html in a browser),
    and the local/usv/tui/active beat tells the mission manager who has the FCU ('handover' / 'takeover');
  * v0.1.45 (WP10f): the FCU block at 2 Hz on local/usv/ui/fcu (boat broker only) for the boat window (boat-web, 8098), and
    the page's words on local/usv/ui/cmd - only handover takeover stop log note save co cv points (decision 25: no arm / disarm,
    mode, bursts, E-stop, thrusters, cannon); each is answered on local/usv/ui/cmd/result and shown here as "[page] word";
  * MAVLink identity: sysid 200 (= CMAN_SYSID, what mode 20 accepts) compid 190; the mission manager is 200/191, so
    COMMAND_ACKs are addressed to the one that asked. Only one of the two commands at a time (tui/active).

    .venv/bin/python3 usv_ui.py                    # Orin hybrid: internal UART /dev/ttyTHS1 @ 921600 (or rc.sh's share)
    .venv/bin/python3 usv_ui.py usb | eth          # FMU USB jumper | Ethernet udpin:0.0.0.0:14550
    .venv/bin/python3 usv_ui.py --log [LABEL]      # start with logging on (default: off; 'log on|off' while running)
    python3 usv_ui.py udpin:127.0.0.1:14550        # rehearsal against the bench fake FCU (op-fake-fcu --conn udpout:127.0.0.1:14550)
    python3 usv_ui.py ... --mqtt 127.0.0.1:1884 --share udpout:127.0.0.1:14551     # the defaults; --share '' = no share

Who commands the FCU: this tool always owns the serial link and shares it; ACTIVE = usv_ui commands (bursts, goto, modes)
and beats local/<v>/tui/active so the mission manager refuses ENABLE_AUTO; `handover` stops the beat - you stay in usv_ui
(telemetry, map, stop) but the mission manager may now take the FCU from the OCS; `takeover` (or stop / x / disarm) takes it back.

Commands (typed at the bottom line, Enter to send; Ctrl-C = stop + quit while active, plain exit when handed over):
    surge A [T]      burst: M1 = M3 = A                    A in -1..1 (± --max-amp), T seconds (≤ --max-dur)
    sway  A [T]      burst: M2 = F A, M4 = A               +A pushes the hull to STARBOARD; F = sway mix
                     (swaymix on: F = 1.05 for +A, 0.877 for -A, calibrated 2026-10-21; off: firmware 0.957)
    yaw   A [T]      burst: M1 = +A, M3 = -A               +A = bow to starboard (clockwise)
    motor N A [T]    burst on one motor (1 PORT, 2 STERN, 3 STBD, 4 BOW)
    burst m1 m2 m3 m4 [T]   raw four-channel burst
    stop | x         cancel any burst (zeros 0.5 s); if GUIDED/AUTO -> HOLD; takes the FCU back when handed over
    mode 20|cman|hold|guided|manual|<n>      arm | disarm
    save NAME        store current lat/lon (+heading) in points.json       here | points | del NAME
    goto NAME [V]    GUIDED to the point; V m/s optional (DO_CHANGE_SPEED)   speed V
    co [NAME]        set the control origin (local N/E frame of the MAP screen) at the boat or at a point
    path A B [C..] [straight]   plan a path boat -> A -> B (centripetal Catmull-Rom spline, or straight legs); PATH tab
    pathto A B [C..] [V]        plan and follow it: a carrot --lookahead m ahead on the path is sent as the GUIDED
                                position target at 2 Hz (ArduPilot's fast-update input shaping), until the end point
    pathgo [V]       follow the already planned path          set la|step|corridor|endr VALUE
    swayto A B [V]   GUIDED to A, then mode 20: yaw until the STARBOARD side faces B, then sway along the A->B line
                     at V (default --sway-v) with heading held orthogonal to the line and cross-track held by surge
    swayto B [V]     same from the current position (no approach)      set swayv|kpsi|kc|yawsign|swaysign VALUE
    cv on|off|status|clear   AGX detections -> map (MAP tab, telemetry/usv/map); off keeps the map, clear drops it
    fcu fake|real [usb]|status  switch the FCU link while running: fake = the bench plant on udp 14550 (started here if none
                     answers), real = the Pixhawk (uart, or usb / eth / an endpoint); disarmed + ACTIVE + nothing running only
    handover         hand the FCU to the mission manager: the tui/active beat stops, FCU commands here are refused (alias: release)
    takeover         take the FCU back from the mission manager; stop / x / disarm also do (alias: active)
    release | active   the old words for handover | takeover, still accepted
    share            who uses the shared link (bytes relayed, last tool seen)
    param NAME VAL   write one parameter (disarmed only, read back)
    help | quit

Every burst is preceded by 0.3 s of zero frames (mode-20 zero-frame rule) and followed by 0.5 s of zeros.
The firmware itself goes neutral 250 ms after the last frame. Logs: <log-dir>/pool_<utc>.tlog (raw MAVLink),
.csv (5 Hz telemetry), .events (commands and results).
"""
import argparse
import collections
import curses
import locale
import datetime as dt
import glob
import json
import math
import os
import socket
import subprocess
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

MAV = mavutil.mavlink

try:                                   # MQTT is optional: without paho the tool is pool_test3 + the share
    import paho.mqtt.client as mqtt    # noqa: E402
except ImportError:
    mqtt = None

# the map code is the mission manager's (MASTER_SCRIPTS/optiprimex, the tree root above this folder) or on PYTHONPATH
_HERE = os.path.dirname(os.path.abspath(__file__))
for _cand in (os.path.join(_HERE, ".."), _HERE):
    if os.path.isdir(os.path.join(_cand, "optiprimex")) and _cand not in sys.path:
        sys.path.insert(0, _cand)
try:
    from optiprimex import TEAM_NS, __version__ as KIT_VERSION        # noqa: E402
    from optiprimex.companion.config import Config                    # noqa: E402
    from optiprimex.companion.frames import PoseBuffer                # noqa: E402
    from optiprimex.companion.state import LocalPose, Observation     # noqa: E402
    from optiprimex.usv.perception_map import PerceptionMap           # noqa: E402
    try:
        from optiprimex.tools.playbook import (V_MEANING, task_stage_meaning, MISSIONS, MISSION_ORDER,   # noqa: E402
                                               mission_stage_names, how_to_select, flow_stage, flow_rows,
                                               mission_for, code_of, cv_source)   # shared with op-tui (v0.1.19: codes m4t0 ...)
    except Exception:                                                  # paho missing: the meanings are a nicety
        V_MEANING, task_stage_meaning = {}, (lambda a, b: "")
        MISSIONS, MISSION_ORDER, mission_stage_names, how_to_select, flow_stage, flow_rows = {}, [], (lambda t: []), None, None, None
        mission_for, code_of = (lambda t, tier=None: None), (lambda t, tier=None, sim=False: t or "")
        cv_source = (lambda h, a, up=False: ("CV cv_publish up" if up else "CV ?", "dim"))
    OPTIPRIMEX_ERR = None
except ImportError as _e:              # bursts / goto / path still work; 'cv' says what is missing
    TEAM_NS = "team/optiprime/v1"
    KIT_VERSION = "?"
    V_MEANING, task_stage_meaning = {}, (lambda a, b: "")
    MISSIONS, MISSION_ORDER, mission_stage_names, how_to_select, flow_stage, flow_rows = {}, [], (lambda t: []), None, None, None
    mission_for, code_of = (lambda t, tier=None: None), (lambda t, tier=None, sim=False: t or "")
    cv_source = (lambda h, a, up=False: ("CV cv_publish up" if up else "CV ?", "dim"))
    Config = PoseBuffer = LocalPose = Observation = PerceptionMap = None
    OPTIPRIMEX_ERR = str(_e)

OWN_COMPID = 190                       # this tool; the mission manager is 191 (both sysid 200 = CMAN_SYSID)
MM_COMPID = 191

MODE_NAMES = {0: "MANUAL", 1: "ACRO", 3: "STEERING", 4: "HOLD", 5: "LOITER", 6: "FOLLOW", 7: "SIMPLE", 8: "DOCK",
              10: "AUTO", 11: "RTL", 12: "SMART_RTL", 15: "GUIDED", 16: "INITIALISING", 20: "CUSTOM_MANUAL"}
MODE_ALIASES = {"manual": 0, "hold": 4, "guided": 15, "auto": 10, "loiter": 5, "cman": 20, "custom": 20}
MOTOR_NAMES = ["M1 PORT", "M2 STERN", "M3 STBD", "M4 BOW"]
STERN_LAT = 0.957            # OMNIUSV_STERN_LAT in the firmware mixer (sway allocation)
SWAY_F_POS = 1.05            # calibrated stern factor, sway to starboard (+), pool 2026-10-21 (logs/SWAYCAL_RESULTS_2026-10-21.md)
SWAY_F_NEG = 0.877           # calibrated stern factor, sway to port (-)
POS_ONLY_MASK = 0x0DF8       # SET_POSITION_TARGET: ignore vel, accel, yaw, yaw_rate -> position only
STREAMS = {  # message id: interval us
    MAV.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 200000, MAV.MAVLINK_MSG_ID_GPS_RAW_INT: 500000,
    MAV.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: 100000, MAV.MAVLINK_MSG_ID_ATTITUDE: 200000,
    MAV.MAVLINK_MSG_ID_VFR_HUD: 200000, MAV.MAVLINK_MSG_ID_SYS_STATUS: 1000000,
    MAV.MAVLINK_MSG_ID_EKF_STATUS_REPORT: 500000, MAV.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: 500000,
    MAV.MAVLINK_MSG_ID_RC_CHANNELS: 500000,
}


def utc_stamp():
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%d-%H%M%SZ")


def haversine(lat1, lon1, lat2, lon2):
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def bearing(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


class Telemetry(object):
    def __init__(self):
        self.hb_t = None
        self.mode = None
        self.armed = False
        self.sysid = None
        self.lat = self.lon = None
        self.hdg = None            # deg, from GLOBAL_POSITION_INT
        self.vn = self.ve = 0.0
        self.gs = 0.0
        self.fix = 0
        self.sats = 0
        self.hdop = 99.9
        self.servo = [0, 0, 0, 0]
        self.ekf_flags = 0
        self.batt = None
        self.wp_dist = None
        self.target_bearing = None
        self.rc = []
        self.status = []           # (t, severity, text, HH:MM:SS)
        self.compid = None         # v0.1.45: the FCU's component id (HEARTBEAT source)
        self.named = {}            # v0.1.45: NAMED_VALUE_INT name -> (value, t) - KILL / ESTOP when the firmware sends them

    def ekf_pos_ok(self):
        return bool(self.ekf_flags & (1 << 4)) or bool(self.ekf_flags & (1 << 9))   # pos_horiz_abs / pred


class Points(object):
    """points.json. v0.1.29: the OCS writes it too (the mission manager's SAVE_POINT / DELETE_POINT), so `data` re-reads the file
    when its mtime changes and save() replaces it atomically - neither writer drops the other's points."""

    def __init__(self, path):
        self.path = path
        self._data = {}
        self._mtime = None
        self._reload()

    def _reload(self):
        try:
            m = os.path.getmtime(self.path)
        except OSError:
            return
        if m == self._mtime:
            return
        try:
            with open(self.path) as fh:
                d = json.load(fh)
            self._data = d if isinstance(d, dict) else {}
        except Exception:
            return                                   # a half-written file: keep the last good copy, try again next time
        self._mtime = m

    @property
    def data(self):
        self._reload()
        return self._data

    def save(self):
        tmp = "%s.%d.tmp" % (self.path, os.getpid())
        with open(tmp, "w") as fh:
            json.dump(self._data, fh, indent=1)
        os.replace(tmp, self.path)
        try:
            self._mtime = os.path.getmtime(self.path)
        except OSError:
            pass


class Origin(object):
    """Control origin (CO) and lat/lon <-> local NED (x north, y east) projection, as in the August stack:
    local tangent plane about a CO we own (not the EKF origin), WGS84 radii of curvature at the CO latitude."""
    A = 6378137.0
    F = 1.0 / 298.257223563
    E2 = F * (2.0 - F)

    def __init__(self, path):
        self.path = path
        self.rec = None            # {"lat","lon","utc","how"}
        self.rm = self.rn = 0.0
        self.lat0 = self.lon0 = 0.0
        if os.path.exists(path):
            try:
                rec = json.load(open(path))
                self.set(rec["lat"], rec["lon"], rec.get("how", "file"), rec.get("utc", ""), persist=False)
            except Exception:
                self.rec = None

    def set(self, lat, lon, how, utc=None, persist=True):
        self.rec = {"lat": lat, "lon": lon, "utc": utc or utc_stamp(), "how": how}
        self.lat0, self.lon0 = math.radians(lat), math.radians(lon)
        s = math.sin(self.lat0)
        d = 1.0 - self.E2 * s * s
        self.rm = self.A * (1.0 - self.E2) / (d ** 1.5)
        self.rn = self.A / math.sqrt(d)
        if persist:
            json.dump(self.rec, open(self.path, "w"), indent=1)

    @property
    def is_set(self):
        return self.rec is not None

    def to_local(self, lat, lon):
        """(x north, y east) metres from the CO."""
        if self.rec is None:
            return None
        x = (math.radians(lat) - self.lat0) * self.rm
        y = (math.radians(lon) - self.lon0) * self.rn * math.cos(self.lat0)
        return x, y

    def from_local(self, x, y):
        """local (x north, y east) metres -> (lat, lon)."""
        lat = math.degrees(self.lat0 + x / self.rm)
        lon = math.degrees(self.lon0 + y / (self.rn * math.cos(self.lat0)))
        return lat, lon


ARROWS_ASCII = ["^", "/", ">", "\\", "v", "/", "<", "\\"]
ARROWS_UNI = ["↑", "↗", "→", "↘", "↓", "↙", "←", "↖"]


ENT_CHAR = {"red": "R", "green": "G", "white": "W", "yellow": "Y", "black": "K", "blue": "B", "orange": "O"}


def render_map(width, height, points, vehicle, target, trail, unicode_ok, min_span_m=10.0, path=None, anchors=None,
               entities=None, gates=None):
    """Text map of the local frame: x (north) up, y (east) right; columns are half a row in metres
    (terminal cells are ~2:1), so a square course looks square. Auto-fits CO + points + vehicle (+ map entities).
    points: {name: (x, y)}; vehicle: (x, y, psi_deg) or None; target: (x, y) or None; trail: [(x, y)];
    entities: [(x, y, kind, colour, stale)] from the perception map (R/G/.. buoys, D dock; lowercase = stale);
    gates: [(x, y)] gate midpoints drawn as '|'. Returns rows of [(char, style)] and the metres spanned by 10 columns."""
    pts = [(0.0, 0.0)] + list(points.values())
    if vehicle:
        pts.append((vehicle[0], vehicle[1]))
    if target:
        pts.append(target)
    if path:
        pts.extend(path)
    for e in (entities or []):
        pts.append((e[0], e[1]))
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    xc, yc = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    span_x = max(max(xs) - min(xs), min_span_m) * 1.25
    span_y = max(max(ys) - min(ys), min_span_m) * 1.25
    s = max(span_x / max(1, height - 1), span_y / max(1, width - 1) * 2.0, 1e-6)   # metres per row
    r_mid, c_mid = (height - 1) / 2.0, (width - 1) / 2.0

    def to_rc(x, y):
        return int(round(r_mid - (x - xc) / s)), int(round(c_mid + (y - yc) / (s / 2.0)))

    grid = [[(" ", "") for _ in range(width)] for _ in range(height)]

    def put(r, c, ch, style, soft=False):
        if 0 <= r < height and 0 <= c < width and (not soft or grid[r][c][0] == " "):
            grid[r][c] = (ch, style)

    dot = "·" if unicode_ok else "."
    for r in range(0, height, 3):
        for c in range(0, width, 6):
            put(r, c, dot, "dim")
    for (x, y) in (path or []):
        r, c = to_rc(x, y)
        put(r, c, dot, "path", soft=True)
    for (x, y) in trail:
        r, c = to_rc(x, y)
        put(r, c, ".", "trail", soft=True)
    if vehicle and target:          # leg to the goto target
        r0, c0 = to_rc(vehicle[0], vehicle[1])
        r1, c1 = to_rc(target[0], target[1])
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr, sc = (1 if r0 < r1 else -1), (1 if c0 < c1 else -1)
        err, r, c = dc - dr, r0, c0
        for _ in range(dr + dc + 2):
            put(r, c, dot, "leg", soft=True)
            if r == r1 and c == c1:
                break
            e2 = 2 * err
            if e2 > -dr:
                err -= dr
                c += sc
            if e2 < dc:
                err += dc
                r += sr
    for (x, y, kind, colour, stale) in (entities or []):
        r, c = to_rc(x, y)
        ch = "D" if kind == "dock" else ENT_CHAR.get(colour, "o")
        style = "ent_red" if colour == "red" else "ent_green" if colour == "green" else "ent"
        put(r, c, ch.lower() if stale else ch, "ent_stale" if stale else style)
    for (x, y) in (gates or []):
        r, c = to_rc(x, y)
        put(r, c, "|", "gate", soft=True)
    r, c = to_rc(0.0, 0.0)
    put(r, c, "+", "co")
    for name, (x, y) in sorted(points.items()):
        r, c = to_rc(x, y)
        put(r, c, name[0].upper(), "target" if target and abs(target[0] - x) < 1e-6 and abs(target[1] - y) < 1e-6 else "point")
    if target:
        r, c = to_rc(target[0], target[1])
        if 0 <= r < height and 0 <= c < width and grid[r][c][1] not in ("point", "target"):
            put(r, c, "x", "target")
    if vehicle:
        r, c = to_rc(vehicle[0], vehicle[1])
        idx = int(round((vehicle[2] % 360.0) / 45.0)) % 8
        put(r, c, (ARROWS_UNI if unicode_ok else ARROWS_ASCII)[idx], "vehicle")
    return grid, 10 * (s / 2.0)



def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def catmull_rom(ctrl, n_per_seg=16, alpha=0.5):
    """Centripetal Catmull-Rom curve through the control points (local N, E). Ghost end points by
    extrapolation, so the curve starts and ends exactly on the first / last control point."""
    if len(ctrl) < 2:
        return list(ctrl)
    if len(ctrl) == 2:
        return [ctrl[0], ctrl[1]]
    g0 = (2 * ctrl[0][0] - ctrl[1][0], 2 * ctrl[0][1] - ctrl[1][1])
    g1 = (2 * ctrl[-1][0] - ctrl[-2][0], 2 * ctrl[-1][1] - ctrl[-2][1])
    P = [g0] + list(ctrl) + [g1]
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        t0 = 0.0
        t1 = t0 + max(_dist(p0, p1), 1e-3) ** alpha
        t2 = t1 + max(_dist(p1, p2), 1e-3) ** alpha
        t3 = t2 + max(_dist(p2, p3), 1e-3) ** alpha
        for k in range(n_per_seg):
            t = t1 + (t2 - t1) * k / float(n_per_seg)
            a1 = tuple((t1 - t) / (t1 - t0) * p0[j] + (t - t0) / (t1 - t0) * p1[j] for j in range(2))
            a2 = tuple((t2 - t) / (t2 - t1) * p1[j] + (t - t1) / (t2 - t1) * p2[j] for j in range(2))
            a3 = tuple((t3 - t) / (t3 - t2) * p2[j] + (t - t2) / (t3 - t2) * p3[j] for j in range(2))
            b1 = tuple((t2 - t) / (t2 - t0) * a1[j] + (t - t0) / (t2 - t0) * a2[j] for j in range(2))
            b2 = tuple((t3 - t) / (t3 - t1) * a2[j] + (t - t1) / (t3 - t1) * a3[j] for j in range(2))
            c = tuple((t2 - t) / (t2 - t1) * b1[j] + (t - t1) / (t2 - t1) * b2[j] for j in range(2))
            out.append(c)
    out.append(tuple(ctrl[-1]))
    return out


def resample(pts, ds):
    """Points every ds metres along a polyline, first and last kept. Returns (points, cumulative s)."""
    if len(pts) < 2:
        return list(pts), [0.0] * len(pts)
    out, s_out = [tuple(pts[0])], [0.0]
    carry = 0.0
    s_total = 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        seg = _dist(a, b)
        if seg < 1e-9:
            continue
        pos = ds - carry
        while pos <= seg:
            f = pos / seg
            out.append((a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])))
            s_out.append(s_total + pos)
            pos += ds
        carry = seg - (pos - ds)
        s_total += seg
    if _dist(out[-1], pts[-1]) > 1e-6:
        out.append(tuple(pts[-1]))
        s_out.append(s_total)
    return out, s_out


def plan_path(ctrl, ds, smooth=True):
    """Control points (local N, E) -> dense path (points, cumulative s)."""
    dense = catmull_rom(ctrl) if (smooth and len(ctrl) >= 3) else list(ctrl)
    return resample(dense, ds)


def utc_iso():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


class Share(object):
    """rc_monitor's link share: every FCU message is re-sent raw to udpout:127.0.0.1:14551 and whatever a tool sends
    there is written to the FCU. The tool side binds udpin:127.0.0.1:14551 (one tool at a time - the socket answers
    the last sender). The .share file (endpoint + pid) lets pool_test3-style wrappers find it."""

    def __init__(self, endpoint, share_file):
        self.endpoint = endpoint
        self.client = endpoint.replace("udpout:", "udpin:")
        self.c = mavutil.mavlink_connection(endpoint, source_system=255, source_component=192)
        self.file = share_file
        self.n_in = self.n_out = 0
        self.t_in = None
        self.peer = None
        with open(share_file, "w") as f:
            f.write("%s %d\n" % (self.client, os.getpid()))

    def to_fcu(self, m):
        """tools -> FCU: relay raw bytes, the sender keeps its own sysid/compid/seq."""
        for _ in range(50):
            up = self.c.recv_msg()
            if up is None:
                break
            if up.get_type() == "BAD_DATA":
                continue
            m.write(up.get_msgbuf())
            self.n_in += 1
            self.t_in = time.monotonic()
            self.peer = (up.get_srcSystem(), up.get_srcComponent())

    def from_fcu(self, msg):
        try:
            self.c.write(msg.get_msgbuf())
            self.n_out += 1
        except OSError:
            pass

    def close(self):
        try:
            os.unlink(self.file)
        except OSError:
            pass
        try:
            self.c.close()
        except Exception:
            pass


class Mq(object):
    """The boat broker (paho, network thread). Inbound messages are queued; App.service_mqtt drains them."""

    def __init__(self, host, port, vehicle, vehicle_id, node="usv_ui"):
        self.host, self.port, self.vehicle, self.vehicle_id, self.node = host, port, vehicle, vehicle_id, node
        self.inbox = collections.deque()
        self.connected = False
        self.t_conn = None
        self.seq = 0
        self.n_pub = 0
        self.err = None
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="%s-%s" % (node, os.getpid()), protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = self._on_disconnect
        self.c.on_message = self._on_message
        self.c.reconnect_delay_set(1, 4)
        self.c.will_set("presence/%s" % node, json.dumps({"node": node, "up": False, "t": utc_iso()}), qos=0, retain=True)

    def start(self):
        try:
            self.c.connect_async(self.host, self.port, keepalive=15)
            self.c.loop_start()
        except Exception as e:
            self.err = str(e)

    def _on_connect(self, c, u, f, rc, p=None):
        self.connected = True
        self.t_conn = time.monotonic()
        for t in ("local/%s/perception/#" % self.vehicle, "local/%s/mm/state" % self.vehicle,
                  "$SYS/broker/connection/+/state",            # the boat broker's bridge to the hub: 1 / 0 (the OCS link)
                  "ocs/state",                                  # the OCS's own state, bridged in (boat_pool.conf.in, v0.1.18)
                  "presence/#",                                 # the nodes on this broker (mission_manager, cv_publish, logger-boat)
                  "local/%s/ui/cmd" % self.vehicle):            # v0.1.45 (WP10f): the boat window's whitelisted words
            c.subscribe(t, 0)

    def _on_disconnect(self, c, u, f, rc=None, p=None):
        self.connected = False

    def _on_message(self, c, u, msg):
        if msg.topic.startswith("$SYS/"):
            self.inbox.append((time.monotonic(), msg.topic, {"raw": msg.payload.decode(errors="ignore").strip()}))
            return
        if msg.retain and msg.topic.endswith("/ui/cmd"):           # a page word is never replayed from a retained message
            return
        try:
            env = json.loads(msg.payload)
        except Exception:
            return
        self.inbox.append((time.monotonic(), msg.topic, env))

    def envelope(self, kind, body):
        self.seq += 1
        return json.dumps({"schema_version": 1, "msg_id": "ui-%010d" % self.seq, "seq": self.seq, "sent_at": utc_iso(), "run_id": None,
                           "declaration_seq": None, "vehicle_id": self.vehicle_id, "kind": kind, "source": self.node, "body": body})

    def publish(self, topic, kind, body, retain=False, raw=None):
        if not self.connected:
            return False
        self.c.publish(topic, raw if raw is not None else self.envelope(kind, body), qos=0, retain=retain)
        self.n_pub += 1
        return True

    def presence(self, up, extra=None):
        body = {"node": self.node, "broker": "%s:%d" % (self.host, self.port), "host": socket.gethostname(),
                "pid": os.getpid(), "up": up, "t": utc_iso(), "t_unix": round(time.time(), 3)}
        body.update(extra or {})                      # v0.1.18: fcu_link fake|real, fcu_conn, log label, active (op-tui shows them)
        return self.publish("presence/%s" % self.node, "local", None, retain=True, raw=json.dumps(body))

    def close(self):
        try:
            self.presence(False)
            time.sleep(0.2)
            self.c.loop_stop()
            self.c.disconnect()
        except Exception:
            pass


class App(object):
    def __init__(self, args):
        self.args = args
        self.m = mavutil.mavlink_connection(args.conn, baud=args.baud, source_system=args.sysid,
                                            source_component=OWN_COMPID, autoreconnect=True)
        self.base = None             # logging is off until "log on" / --log (see log_start)
        self.csv = None
        self.ev = None
        self.log_files = []          # every base written this run (printed on exit)
        self.mstats = {}             # MAVLink inspector: type -> {"n", "times", "last"}
        self.mav_scroll = 0
        self.cal = None              # swaycal run (see swaycal_cmd)
        self.swaymix = args.swaymix == "on"   # calibrated per-direction stern factor for sway / swayto (see sway_lat)
        self.sway_fpos = args.sway_fpos
        self.sway_fneg = args.sway_fneg
        self.tel = Telemetry()
        self.fake_proc = None        # the fake plant started by 'fcu fake' (None when the launcher or nobody runs one)
        self.fake_port()             # remember the fake plant's port when we start on it
        self.await_hb = None         # set by 'fcu fake|real' until the new link's first heartbeat
        self.points = Points(args.points)
        self.origin = Origin(args.co)
        self.trail = []              # local (x, y) history for the map
        self.path = None             # planned / running path (see plan_path_cmd)
        self.path_la = args.lookahead
        self.path_step = args.path_step
        self.path_corridor = args.corridor
        self.path_end_r = args.end_radius
        self.sway = None             # sway transit state (see swayto_cmd)
        self.sway_v = args.sway_v
        self.sway_kpsi = args.sway_kpsi      # yaw command per 45 deg of heading error
        self.sway_kc = args.sway_kc          # surge command per metre of cross-track
        self.sway_rmax = 0.3
        self.sway_umax = 0.25
        self.sway_corridor = args.sway_corridor
        self.dock = None             # dockto state (see dockto_cmd): sway abeam the bay, then bow in (bench / sim only)
        self.dock_standoff = 4.0     # m off the dock face for the sway along it
        self.dock_stop = 1.5         # m from the face where the bow-in stops (v0.1.34: 1.0 -> 1.5, the real 51 deg camera sees the windows)
        self.dock_v = 0.15           # bow-in surge amplitude
        self.pathgen_min_obs = 5     # pathgen ignores buoys seen fewer times (ghosts)
        self.yaw_sign = 1.0
        self.sway_sign = 1.0
        self.log = []                # (t, text, kind) kind = event | cmd | result; LOG tab shows all, MAIN splits EVENTS / CONSOLE
        self._in_cmd = False         # True while do_command runs: event() lines are command results (CONSOLE)
        self.last_refused = None     # (t, text) of the last refused / bad command (WARNINGS, 10 s)
        self.cond_since = {}         # warning key -> first seen
        self.bridge = None           # the boat broker's bridge to the hub ($SYS/broker/connection/<name>/state): None unknown, True, False
        self.bridge_t = 0.0
        self.ocs = None              # (t_recv, body) of team/.../ocs/state when it is bridged into the boat broker
        self.presence = {}           # node -> (t_recv, body) from presence/# on the boat broker
        self.mission_cursor = 0      # the highlighted mission on the MISSION tab
        self.action = None           # dict: label, controls, phase, t_phase_end, t_end
        self.goto = None             # dict: name, lat, lon, t
        self.pending_goto = None     # (name, lat, lon, speed, deadline) waiting for GUIDED
        self.cmdline = ""
        self.last_send = 0.0
        self.t_last_mav_cmd = -1e9    # when this console last sent a MAVLink command (its acks are events; the mission manager's are not)
        self.last_hb = 0.0
        self.last_csv = 0.0
        self.quit = False
        self.t0 = time.monotonic()
        self.scr = None
        # ---- v4: link share, MQTT, perception map, FCU handover ----
        self.share = None
        self.share_err = None
        if args.share and not args.conn.startswith(args.share.replace("udpout:", "udpin:")):
            try:
                self.share = Share(args.share, os.path.join(_HERE, ".share"))
            except Exception as e:
                self.share_err = str(e)
        self.mq = None
        self.mq_err = None
        if args.mqtt:
            if mqtt is None:
                self.mq_err = "paho-mqtt not installed (pip install paho-mqtt)"
            else:
                host, _, port = args.mqtt.partition(":")
                self.mq = Mq(host or "127.0.0.1", int(port or 1884), args.vehicle, args.vehicle_id)
        self.cvmap = None
        self.poses = None
        if PerceptionMap is not None:
            cfg = Config()
            cfg.frames.cam_forward_m, cfg.frames.cam_right_m, cfg.frames.cam_yaw_deg = args.cam_forward, args.cam_right, args.cam_yaw
            self.cvmap = PerceptionMap(cfg)
            self.poses = PoseBuffer(maxlen=400)
        self.cv_on = False
        self.cv_health = None            # (t_recv, body) of local/<v>/perception/health
        self.cv_n_rx = 0                 # observations received (any kind)
        self.cv_n_used = 0               # items folded into the map
        self.cv_t_rx = None
        self.cv_skew = 0                 # observations whose t_cam was > 1 s off our clock (AGX/NX clock skew)
        self.cv_known = set()            # entity ids already announced in the log
        self.mm = None                   # (t_recv, body) of local/<v>/mm/state (the mission manager, if it runs)
        self.active = not args.handover  # True: usv_ui commands the FCU (tui/active beat); False: handed over to the mission manager
        self.last_beat = 0.0
        self.last_presence = 0.0
        self.last_map = 0.0
        self.last_fcu_pub = 0.0          # v0.1.45: the FCU block on local/<v>/ui/fcu at 2 Hz (boat-web)
        self.last_remote = -1e9          # v0.1.45: when the last page word was accepted (flood guard)
        self.n_remote = 0
        self._cmd_out = None             # the event lines of the command running now (the page's reply)

    # ---------------- logging (off by default; "log on [LABEL]" / "log off" / --log) ----------------
    CSV_HEADER = ("t_mono,utc,mode,armed,lat,lon,hdg,gs,vn,ve,fix,sats,hdop,ekf,s1,s2,s3,s4,"
                  "action,c1,c2,c3,c4,goto,wp_dist,x_n,y_e,path_state,path_s,path_xte,carrot_n,carrot_e,"
                  "sway_state,sway_s,sway_c,sway_e\n")

    def log_start(self, label=None):
        if self.base:
            self.event("logging already on -> %s" % self.base)
            return
        os.makedirs(self.args.log_dir, exist_ok=True)
        name = "pool_%s" % utc_stamp() + ("_" + "".join(c if c.isalnum() or c in "-_" else "-" for c in label) if label else "")
        self.base = os.path.join(self.args.log_dir, name)
        self.m.setup_logfile(self.base + ".tlog")
        self.csv = open(self.base + ".csv", "w")
        self.csv.write(self.CSV_HEADER)
        self.ev = open(self.base + ".events", "w")
        self.log_files.append(self.base)
        self.log_t0 = time.time()                      # v0.1.83.5: the OCS's REC (presence log_t0 / log_bytes)
        a = self.args
        self.event("logging ON -> %s.tlog/.csv/.events" % self.base)
        self.event("settings: conn %s baud %s | la %.1f step %.1f corridor %.1f endr %.1f | max_amp %.2f max_dur %.0f | "
                   "swayv %.2f kpsi %.2f kc %.2f yawsign %+d swaysign %+d | sway mix %s" % (
                       a.conn, a.baud, self.path_la, self.path_step, self.path_corridor, self.path_end_r,
                       a.max_amp, a.max_dur, self.sway_v, self.sway_kpsi, self.sway_kc, self.yaw_sign, self.sway_sign,
                       self.swaymix_text()))

    def log_bytes(self):
        """v0.1.83.5: the run log's size now (.tlog + .csv + .events), None when off - the OCS's REC shows it growing."""
        if not self.base:
            return None
        n = 0
        for ext in (".tlog", ".csv", ".events"):
            try:
                n += os.path.getsize(self.base + ext)
            except OSError:
                pass
        return n

    def log_stop(self):
        if not self.base:
            self.event("logging is already off")
            return
        base = self.base
        self.event("logging OFF (%s)" % base)
        if self.m.logfile:
            self.m.logfile.close()
            self.m.logfile = None
        self.csv.close()
        self.ev.close()
        self.base = self.csv = self.ev = None
        self.log_t0 = None

    def event(self, text):
        t = time.monotonic()
        self.log.append((t, text, "result" if self._in_cmd else "event"))
        self.log = self.log[-400:]
        if self._in_cmd and self._cmd_out is not None:
            self._cmd_out.append(text)
        if text.startswith(("refused", "bad arguments", "unknown command", "warning")):
            self.last_refused = (t, text)
        if self.ev:
            self.ev.write("%s,%.3f,%s\n" % (dt.datetime.now(dt.timezone.utc).isoformat(), t, text))
            self.ev.flush()

    def write_csv(self):
        if not self.csv:
            return
        t = self.tel
        a = self.action
        c = a["controls"] if a and a["phase"] == "burst" else [0, 0, 0, 0]
        loc = self.local() if self.origin.is_set else None
        pp = self.path
        pcols = ("%s,%.1f,%.2f,%s,%s" % (pp["state"], pp["s"][pp["idx"]], pp["xte"],
                                       ("%.1f" % pp["carrot"][0]) if pp["carrot"] else "", ("%.1f" % pp["carrot"][1]) if pp["carrot"] else "")
                 if pp else ",,,,")
        sw = self.sway
        scols = ("%s,%.1f,%.2f,%.0f" % (sw["state"], sw["s"], sw["c"], sw["e"])) if sw else ",,,"
        self.csv.write("%.3f,%s,%s,%d,%s,%s,%s,%.2f,%.2f,%.2f,%d,%d,%.2f,%d,%d,%d,%d,%d,%s,%.3f,%.3f,%.3f,%.3f,%s,%s,%s,%s,%s,%s\n" % (
            time.monotonic(), dt.datetime.now(dt.timezone.utc).isoformat(), MODE_NAMES.get(t.mode, t.mode),
            int(t.armed), "" if t.lat is None else "%.7f" % t.lat, "" if t.lon is None else "%.7f" % t.lon,
            "" if t.hdg is None else "%.1f" % t.hdg, t.gs, t.vn, t.ve, t.fix, t.sats, t.hdop, t.ekf_flags,
            t.servo[0], t.servo[1], t.servo[2], t.servo[3], (a["label"] + ":" + a["phase"]) if a else "",
            c[0], c[1], c[2], c[3], self.goto["name"] if self.goto else "",
            "" if t.wp_dist is None else "%.1f" % t.wp_dist,
            "" if loc is None else "%.2f" % loc[0], "" if loc is None else "%.2f" % loc[1], pcols, scols))

    # ---------------- MAVLink in ----------------
    def request_streams(self):
        for mid, us in STREAMS.items():
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         MAV.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, us, 0, 0, 0, 0, 0)

    def handle(self, msg):
        t = self.tel
        k = msg.get_type()
        st = self.mstats.get(k)
        if st is None:
            st = self.mstats[k] = {"n": 0, "times": collections.deque(maxlen=60), "last": None}
        st["n"] += 1
        st["times"].append(time.monotonic())
        st["last"] = msg
        if k == "HEARTBEAT":
            if msg.type in (MAV.MAV_TYPE_GCS,) or msg.get_srcSystem() != self.m.target_system:
                return
            t.hb_t = time.monotonic()
            t.mode = msg.custom_mode
            t.armed = bool(msg.base_mode & MAV.MAV_MODE_FLAG_SAFETY_ARMED)
            t.sysid = msg.get_srcSystem()
            t.compid = msg.get_srcComponent()
        elif k == "GLOBAL_POSITION_INT":
            t.lat, t.lon = msg.lat / 1e7, msg.lon / 1e7
            t.hdg = msg.hdg / 100.0 if msg.hdg != 65535 else None
            t.vn, t.ve = msg.vx / 100.0, msg.vy / 100.0
            if self.poses is not None and self.origin.is_set and t.hdg is not None:
                x, y = self.origin.to_local(t.lat, t.lon)
                self.poses.push(LocalPose(x_n=x, y_e=y, hdg_deg=t.hdg, co_lat=self.origin.rec["lat"], co_lon=self.origin.rec["lon"],
                                          t=time.monotonic(), sog_mps=t.gs))
        elif k == "GPS_RAW_INT":
            t.fix, t.sats = msg.fix_type, msg.satellites_visible
            t.hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.9
        elif k == "SERVO_OUTPUT_RAW":
            t.servo = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
        elif k == "VFR_HUD":
            t.gs = msg.groundspeed
        elif k == "EKF_STATUS_REPORT":
            t.ekf_flags = msg.flags
        elif k == "SYS_STATUS":
            t.batt = msg.voltage_battery / 1000.0 if msg.voltage_battery not in (0, 65535) else None
        elif k == "NAV_CONTROLLER_OUTPUT":
            t.wp_dist, t.target_bearing = msg.wp_dist, msg.target_bearing
        elif k == "RC_CHANNELS":
            t.rc = [getattr(msg, "chan%d_raw" % i) for i in range(1, 9)]
        elif k == "NAMED_VALUE_INT":
            name = msg.name.rstrip("\x00") if isinstance(msg.name, str) else msg.name.rstrip(b"\0").decode(errors="replace")
            t.named[name] = (msg.value, time.monotonic())
        elif k == "STATUSTEXT":
            text = msg.text.rstrip("\x00") if isinstance(msg.text, str) else msg.text.rstrip(b"\0").decode(errors="replace")
            t.status.append((time.monotonic(), msg.severity, text, time.strftime("%H:%M:%S")))
            t.status = t.status[-200:]
        elif k == "COMMAND_ACK":
            tc = getattr(msg, "target_component", 0)
            ours = tc == OWN_COMPID or (time.monotonic() - self.t_last_mav_cmd) < 3.0
            text = "ack cmd %d result %d%s" % (msg.command, msg.result,
                                               "" if tc in (0, OWN_COMPID) else "  (for comp %d%s)" % (tc, " = mission manager" if tc == MM_COMPID else ""))
            if ours:
                self.event(text)
            else:                                 # the mission manager's traffic on the shared link: LOG tab only, not EVENTS
                self.log.append((time.monotonic(), text + "  (mm)", "mav"))
                self.log = self.log[-400:]

    # ---------------- MAVLink out ----------------
    def send_controls(self, c):
        self.m.mav.set_actuator_control_target_send(int((time.monotonic() - self.t0) * 1e6), 0,
                                                    self.m.target_system, self.m.target_component,
                                                    [float(c[0]), float(c[1]), float(c[2]), float(c[3]), 0, 0, 0, 0])

    def set_mode(self, n):
        self.t_last_mav_cmd = time.monotonic()
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, MAV.MAV_CMD_DO_SET_MODE, 0,
                                     MAV.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, n, 0, 0, 0, 0, 0)
        self.event("mode request %s (%d)" % (MODE_NAMES.get(n, "?"), n))

    def arm(self, on):
        self.t_last_mav_cmd = time.monotonic()
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     MAV.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1.0 if on else 0.0, 0, 0, 0, 0, 0, 0)
        self.event("arm request" if on else "disarm request")

    def change_speed(self, v):
        self.t_last_mav_cmd = time.monotonic()
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component, MAV.MAV_CMD_DO_CHANGE_SPEED, 0,
                                     1, float(v), -1, 0, 0, 0, 0)
        self.event("speed request %.2f m/s" % v)

    def send_goto(self, name, lat, lon, speed=None):
        self.t_last_mav_cmd = time.monotonic()
        self.m.mav.set_position_target_global_int_send(
            int((time.monotonic() - self.t0) * 1000), self.m.target_system, self.m.target_component,
            MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POS_ONLY_MASK, int(round(lat * 1e7)), int(round(lon * 1e7)),
            0.0, 0, 0, 0, 0, 0, 0, 0, 0)
        self.goto = {"name": name, "lat": lat, "lon": lon, "t": time.monotonic()}
        self.event("goto %s %.7f %.7f sent" % (name, lat, lon))
        if speed is not None:
            # DO_CHANGE_SPEED only works in GUIDED's WP sub-mode, i.e. after the target; a GUIDED re-entry
            # resets it to WP_SPEED (set WP_SPEED itself for a standing pool value)
            self.change_speed(speed)

    # ---------------- actions ----------------
    def start_burst(self, label, controls, dur):
        if self.tel.mode != 20:
            self.event("refused: mode 20 (CUSTOM_MANUAL) first, now %s" % MODE_NAMES.get(self.tel.mode, "?"))
            return
        if not self.tel.armed:
            self.event("refused: not armed")
            return
        mx = max(abs(x) for x in controls)
        if mx > self.args.max_amp:
            self.event("refused: |amp| %.2f > max %.2f (--max-amp)" % (mx, self.args.max_amp))
            return
        dur = min(max(dur, 0.1), self.args.max_dur)
        now = time.monotonic()
        self.action = {"label": label, "controls": [float(x) for x in controls], "phase": "zero",
                       "t_phase_end": now + 0.3, "t_burst": dur}
        self.event("%s [%s] %.1f s" % (label, " ".join("%+.2f" % x for x in controls), dur))

    def stop(self, why="stop"):
        now = time.monotonic()
        if self.action and self.action["phase"] != "tail":
            self.action = {"label": "stop", "controls": [0, 0, 0, 0], "phase": "tail", "t_phase_end": now + 0.5,
                           "t_burst": 0}
        elif self.tel.mode == 20 and self.tel.armed:
            self.action = {"label": "stop", "controls": [0, 0, 0, 0], "phase": "tail", "t_phase_end": now + 0.5,
                           "t_burst": 0}
        if self.tel.mode in (15, 10, 5, 11, 12):
            self.set_mode(4)
        self.goto = None
        self.pending_goto = None
        if self.path and self.path["state"] in ("waiting", "running"):
            self.path["state"] = "aborted"
            self.path["t_done"] = now
        if self.sway and self.sway["state"] not in ("done", "aborted"):
            self.sway["state"] = "aborted"
            self.sway["t_state"] = now
        if self.dock and self.dock["state"] not in ("docked", "aborted"):
            self.dock["state"] = "aborted"
            self.dock["t_state"] = now
        if self.cal and self.cal["state"] in ("mode", "arm", "burst"):
            self.cal_end("ABORTED: %s" % why)
        self.event(why)

    def service_action(self):
        a = self.action
        if a is None:
            return
        now = time.monotonic()
        if now >= a["t_phase_end"]:
            if a["phase"] == "zero":
                a["phase"], a["t_phase_end"] = "burst", now + a["t_burst"]
                a["samples"] = []
            elif a["phase"] == "burst":
                a["phase"], a["t_phase_end"] = "tail", now + 0.5
                self.burst_summary(a)
            else:
                self.action = None
                self.event("%s done" % a["label"])
                return
        if now - self.last_send >= 0.1:
            self.send_controls(a["controls"] if a["phase"] == "burst" else [0, 0, 0, 0])
            self.last_send = now
            if a["phase"] == "burst" and self.tel.hdg is not None:
                a.setdefault("samples", []).append((now, self.tel.hdg, self.tel.vn, self.tel.ve))
        if a["phase"] == "burst" and (self.tel.mode != 20 or not self.tel.armed):
            self.action = None
            self.event("burst aborted: mode/arm changed")

    def sway_factor(self, v):
        if not self.swaymix:
            return STERN_LAT
        return self.sway_fpos if v >= 0 else self.sway_fneg

    def sway_lat(self, v, f=None):
        """(M2 stern, M4 bow) for a sway command v: M2 = F*v, M4 = v; F > 1 lowers M4 so neither exceeds |v|."""
        f = self.sway_factor(v) if f is None else f
        return (f * v, v) if f <= 1.0 else (v, v / f)

    def swaymix_text(self):
        return ("CAL +%.3f / -%.3f" % (self.sway_fpos, self.sway_fneg)) if self.swaymix else ("fw %.3f" % STERN_LAT)

    def burst_summary(self, a):
        """Heading drift and body-frame speeds over the burst (for sway / surge calibration)."""
        sm = a.get("samples") or []
        if len(sm) < 3:
            self.event("%s summary: no heading/velocity samples" % a["label"])
            return
        t0 = sm[0][0]
        dt_ = sm[-1][0] - t0
        ts, hu, fwd, stb = [], [], [], []
        off, prev = 0.0, None
        for tt, h, vn, ve in sm:
            if prev is not None:              # unwrap: the boat can turn more than 180 deg in one burst
                if h - prev > 180.0:
                    off -= 360.0
                elif h - prev < -180.0:
                    off += 360.0
            prev = h
            ts.append(tt - t0)
            hu.append(h + off)
            r = math.radians(h)
            fwd.append(vn * math.cos(r) + ve * math.sin(r))
            stb.append(-vn * math.sin(r) + ve * math.cos(r))
        k = [i for i, x in enumerate(ts) if x >= 2.0]      # yaw rate: least-squares fit after the 2 s spin-up
        if len(k) < 3:
            k = list(range(len(ts)))
        mt = sum(ts[i] for i in k) / len(k)
        mh = sum(hu[i] for i in k) / len(k)
        sxx = sum((ts[i] - mt) ** 2 for i in k)
        rate = sum((ts[i] - mt) * (hu[i] - mh) for i in k) / sxx if sxx else 0.0
        self.event("%s summary: %.1f s  hdg turned %+.0f deg  rate %+.2f deg/s (fit after 2 s)  v_fwd %+.2f  v_stbd %+.2f m/s"
                   "  (mean, n %d)" % (a["label"], dt_, hu[-1] - hu[0], rate, sum(fwd) / len(fwd), sum(stb) / len(stb), len(sm)))
        if self.cal and self.cal["state"] == "burst":
            self.cal["summary"] = True

    # ---------------- sway calibration run: mode 20 -> arm -> log on -> sway burst -> log off ----------------
    def swaycal_cmd(self, a, dur, f):
        if self.cal and self.cal["state"] in ("mode", "arm", "burst"):
            self.event("refused: swaycal already running (stop / x cancels it)")
            return
        if self.action:
            self.event("refused: burst running")
            return
        if abs(a) > self.args.max_amp:
            self.event("refused: |A| %.2f > max %.2f (--max-amp)" % (abs(a), self.args.max_amp))
            return
        if dur > self.args.max_dur:
            self.event("refused: T %.0f s > --max-dur %.0f  (restart: usv_ui.py --max-dur %.0f)" % (
                dur, self.args.max_dur, math.ceil(dur)))
            return
        if not 0.5 <= f <= 2.0:
            self.event("refused: stern factor %.3f outside 0.5..2.0" % f)
            return
        m2, m4 = self.sway_lat(a, f)                          # the larger command stays |A|
        label = "swaycal_f%04d_a%s%03d_%ds" % (round(f * 1000), "p" if a >= 0 else "m", round(abs(a) * 100), round(dur))
        self.cal = {"state": "mode", "a": a, "dur": dur, "f": f, "controls": [0.0, m2, 0.0, m4], "label": label,
                    "t_state": time.monotonic(), "sent": False, "own_log": False, "summary": False}
        self.event("swaycal %s: mode 20 -> arm -> log on -> sway -> log off" % label)

    def service_cal(self, now):
        c, t = self.cal, self.tel
        if c is None or c["state"] not in ("mode", "arm", "burst"):
            return
        if c["state"] == "mode":
            if t.mode == 20:
                c["state"], c["t_state"], c["sent"] = "arm", now, False
            elif not c["sent"]:
                self.set_mode(20)
                c["sent"] = True
            elif now - c["t_state"] > 3.0:
                self.cal_end("ABORTED: mode 20 not confirmed within 3 s")
            return
        if c["state"] == "arm":
            if t.armed:
                if self.base:
                    self.event("swaycal: logging already on -> recording into %s" % os.path.basename(self.base))
                else:
                    self.log_start(c["label"])
                    c["own_log"] = True
                self.event("swaycal %s: A %+.2f  T %.0f s  stern factor %.3f  ->  M2 %+.3f  M4 %+.3f" % (
                    c["label"], c["a"], c["dur"], c["f"], c["controls"][1], c["controls"][3]))
                self.start_burst(c["label"], c["controls"], c["dur"])
                if not self.action:
                    self.cal_end("ABORTED: burst refused")
                    return
                c["state"], c["t_state"] = "burst", now
            elif not c["sent"]:
                self.arm(True)
                c["sent"] = True
            elif now - c["t_state"] > 3.0:
                self.cal_end("ABORTED: not armed within 3 s (pre-arm, RC kill / E-stop?)")
            return
        if self.action is None:
            self.cal_end("done" if c["summary"] else "ABORTED: burst did not complete (mode / arm changed?)")

    def cal_end(self, result):
        c = self.cal
        c["state"] = "done" if result == "done" else "aborted"
        self.event("swaycal %s %s%s" % (c["label"], result,
                                         "  - boat stays armed in mode 20, outputs neutral" if self.tel.armed else ""))
        if c["own_log"] and self.base:
            self.log_stop()

    # ---------------- path following (carrot on a planned path, GUIDED position targets) ----------------
    @staticmethod
    def _is_number(s):
        try:
            float(s)
            return True
        except ValueError:
            return False

    def plan_path_cmd(self, names, smooth):
        if not self.origin.is_set:
            self.event("refused: no control origin (co) yet")
            return False
        loc = self.local()
        if loc is None:
            self.event("refused: no position yet")
            return False
        ctrl = [loc]
        for n in names:
            p = self.points.data.get(n)
            if p is None:
                self.event("unknown point %s" % n)
                return False
            ctrl.append(self.origin.to_local(p["lat"], p["lon"]))
        pts, s = plan_path(ctrl, self.path_step, smooth)
        if len(pts) < 2 or s[-1] < 0.5:
            self.event("refused: path too short (%.1f m)" % (s[-1] if s else 0))
            return False
        self.path = {"anchors": list(names), "ctrl": ctrl, "pts": pts, "s": s, "smooth": smooth, "state": "planned",
                     "idx": 0, "carrot": None, "carrot_idx": 0, "xte": 0.0, "max_xte": 0.0, "t_start": None,
                     "t_done": None, "speed": None, "n_sent": 0, "last_send": 0.0, "la": self.path_la}
        self.event("path %s: %.1f m, %d pts, %s (la %.1f step %.1f)" % (
            "-".join(["boat"] + names), s[-1], len(pts), "spline" if smooth else "straight", self.path_la, self.path_step))
        return True

    def start_path(self, speed):
        t = self.tel
        p = self.path
        if p is None:
            self.event("no path")
            return
        if not t.armed:
            self.event("refused: not armed")
            return
        if not (t.fix >= 3 and t.ekf_pos_ok()):
            self.event("refused: fix %d / EKF pos %s" % (t.fix, t.ekf_pos_ok()))
            return
        if self.action:
            self.event("refused: burst running")
            return
        self.goto = None
        self.pending_goto = None
        p.update({"state": "waiting", "idx": 0, "carrot": None, "carrot_idx": 0, "xte": 0.0, "max_xte": 0.0,
                  "t_start": time.monotonic(), "t_done": None, "speed": speed, "n_sent": 0, "last_send": 0.0,
                  "la": self.path_la})
        if t.mode != 15:
            self.set_mode(15)
        self.event("pathto %s started%s" % ("-".join(p["anchors"]), (" at %.2f m/s" % speed) if speed else ""))

    def send_target_quiet(self, lat, lon):
        self.m.mav.set_position_target_global_int_send(
            int((time.monotonic() - self.t0) * 1000), self.m.target_system, self.m.target_component,
            MAV.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, POS_ONLY_MASK, int(round(lat * 1e7)), int(round(lon * 1e7)),
            0.0, 0, 0, 0, 0, 0, 0, 0, 0)

    def service_path(self, now):
        p = self.path
        if p is None or p["state"] not in ("waiting", "running"):
            return
        t = self.tel
        if p["state"] == "waiting":
            if t.mode == 15:
                p["state"] = "running"
                self.event("path running (GUIDED confirmed)")
            elif now - p["t_start"] > 3.0:
                p["state"] = "aborted"
                self.event("path aborted: GUIDED not confirmed")
                return
            else:
                return
        if t.mode != 15 or not t.armed:
            p["state"] = "aborted"
            p["t_done"] = now
            self.event("path aborted: mode %s armed %s" % (MODE_NAMES.get(t.mode, t.mode), t.armed))
            return
        loc = self.local()
        if loc is None:
            return
        pts, s = p["pts"], p["s"]
        n = len(pts)
        # progress: nearest path point in a window ahead of the last progress index (monotonic)
        i0 = p["idx"]
        i1 = min(n - 1, i0 + int(20.0 / self.path_step))
        best, bd = i0, _dist(loc, pts[i0])
        for i in range(i0, i1 + 1):
            d = _dist(loc, pts[i])
            if d < bd:
                best, bd = i, d
        p["idx"] = best
        # signed cross-track (+ = boat to starboard of the path direction)
        j = min(best + 1, n - 1)
        k = max(best - 1, 0) if j == best else best
        dx, dy = pts[j][0] - pts[k][0], pts[j][1] - pts[k][1]
        dl = math.hypot(dx, dy)
        if dl > 1e-6:
            vx, vy = loc[0] - pts[k][0], loc[1] - pts[k][1]
            p["xte"] = (dx * vy - dy * vx) / dl
        else:
            p["xte"] = bd
        p["max_xte"] = max(p["max_xte"], abs(p["xte"]))
        if abs(p["xte"]) > self.path_corridor:
            p["state"] = "aborted"
            p["t_done"] = now
            self.stop("path aborted: xte %.1f m > corridor %.1f m" % (p["xte"], self.path_corridor))
            return
        # carrot: first point at least la ahead along the path; the end point once within reach
        s_boat = s[best]
        ci = best
        while ci < n - 1 and s[ci] < s_boat + p["la"]:
            ci += 1
        p["carrot_idx"] = ci
        p["carrot"] = pts[ci]
        remaining = s[-1] - s_boat
        d_end = _dist(loc, pts[-1])
        if d_end <= self.path_end_r or (best >= n - 1 and d_end <= 2 * self.path_end_r):
            lat, lon = self.origin.from_local(*pts[-1])
            self.send_target_quiet(lat, lon)
            p["state"] = "done"
            p["t_done"] = now
            self.goto = {"name": "path-end", "lat": lat, "lon": lon, "t": now}
            self.event("path done: %.0f s, %d targets, max |xte| %.1f m, end dist %.1f m" % (
                now - p["t_start"], p["n_sent"], p["max_xte"], d_end))
            return
        if now - p["last_send"] >= 0.5:
            lat, lon = self.origin.from_local(*p["carrot"])
            self.send_target_quiet(lat, lon)
            p["last_send"] = now
            p["n_sent"] += 1
            self.goto = {"name": "carrot", "lat": lat, "lon": lon, "t": p["t_start"]}
            if p["n_sent"] == 1 and p["speed"]:
                self.change_speed(p["speed"])
            if p["n_sent"] % 20 == 0:
                self.event("path %.0f%% s %.1f/%.1f m xte %+.1f" % (100.0 * s_boat / s[-1], s_boat, s[-1], p["xte"]))

    # ---------------- sway transit (mode 20 closed loop: heading orthogonal to the A->B line) ----------------
    @staticmethod
    def wrap180(a):
        return (a + 180.0) % 360.0 - 180.0

    def swayto_cmd(self, names, speed):
        t = self.tel
        if not self.origin.is_set or self.local() is None:
            self.event("refused: no position / control origin")
            return
        if not t.armed:
            self.event("refused: not armed")
            return
        if self.action or (self.path and self.path["state"] in ("waiting", "running")):
            self.event("refused: burst or path running")
            return
        if len(names) == 1:
            a_loc, a_name = self.local(), "here"
            b = self.points.data.get(names[0])
            if b is None:
                self.event("unknown point %s" % names[0])
                return
            approach = False
        elif len(names) == 2:
            pa, b = self.points.data.get(names[0]), self.points.data.get(names[1])
            if pa is None or b is None:
                self.event("unknown point %s" % (names[0] if pa is None else names[1]))
                return
            a_loc, a_name = self.origin.to_local(pa["lat"], pa["lon"]), names[0]
            approach = True
        else:
            raise ValueError("swayto B  |  swayto A B [V]")
        b_loc = self.origin.to_local(b["lat"], b["lon"])
        dN, dE = b_loc[0] - a_loc[0], b_loc[1] - a_loc[1]
        length = math.hypot(dN, dE)
        if length < 1.0:
            self.event("refused: A and B are %.1f m apart" % length)
            return
        theta = math.degrees(math.atan2(dE, dN)) % 360.0          # bearing A -> B
        psi_t = (theta - 90.0) % 360.0                              # starboard side faces B
        self.sway = {"a": a_loc, "b": b_loc, "a_name": a_name, "b_name": names[-1], "theta": theta, "psi_t": psi_t,
                     "length": length, "v": speed if speed else self.sway_v, "state": "approach" if approach else "to20",
                     "t_state": time.monotonic(), "t_start": time.monotonic(), "s": 0.0, "c": 0.0, "e": 0.0,
                     "cmd": [0, 0, 0, 0], "max_c": 0.0, "max_e": 0.0, "aligned_since": None, "zero_until": 0.0, "n": 0}
        self.goto = None
        self.pending_goto = None
        self.event("swayto %s->%s: %.1f m, line brg %03.0f, hold hdg %03.0f, v %.2f" % (
            a_name, names[-1], length, theta, psi_t, self.sway["v"]))
        if approach:
            self.pending_goto = (a_name, pa["lat"], pa["lon"], None, time.monotonic() + 3.0)
            if t.mode != 15:
                self.set_mode(15)
            else:
                self.pending_goto = None
                self.send_goto(a_name, pa["lat"], pa["lon"])
        else:
            self.sway_enter_20()

    def sway_enter_20(self):
        sw = self.sway
        sw["state"], sw["t_state"] = "to20", time.monotonic()
        self.goto = None
        if self.tel.mode != 20:
            self.set_mode(20)

    def sway_abort(self, why):
        sw = self.sway
        if sw and sw["state"] not in ("done", "aborted"):
            sw["state"], sw["t_state"] = "aborted", time.monotonic()
        self.stop("sway aborted: " + why)

    def service_sway(self, now):
        sw = self.sway
        if sw is None or sw["state"] in ("done", "aborted"):
            return
        t = self.tel
        loc = self.local()
        if loc is None or t.hdg is None:
            return
        if not t.armed:
            sw["state"] = "aborted"
            self.event("sway aborted: disarmed")
            return
        a, b = sw["a"], sw["b"]
        th = math.radians(sw["theta"])
        dN, dE = math.cos(th), math.sin(th)                    # unit along the line A->B
        dr = sw.get("dir", 1)                                  # +1 = starboard faces B (swayto), -1 = port faces B (dockto)
        bN, bE = dr * math.sin(th), -dr * math.cos(th)         # bow direction for psi_t = theta - 90 * dir
        pN, pE = loc[0] - a[0], loc[1] - a[1]
        sw["s"] = pN * dN + pE * dE                            # along-track from A
        sw["c"] = pN * bN + pE * bE                            # cross-track along the bow axis (+ = ahead of the line)
        sw["e"] = self.wrap180(sw["psi_t"] - t.hdg)            # heading error, + = turn clockwise
        st = sw["state"]
        if st == "approach":
            if self.goto is None and self.pending_goto is None and now - sw["t_state"] > 5.0:
                self.sway_abort("approach: GUIDED target not accepted")
                return
            d_a = _dist(loc, a)
            settled = t.mode == 15 and d_a <= 2.5 and t.gs < 0.3     # GUIDED has stopped / loiters near A
            sw["slow_since"] = (sw.get("slow_since") or now) if settled else None
            if t.mode == 15 and (d_a <= self.path_end_r or (settled and now - sw["slow_since"] > 2.0)):
                self.event("sway: reached %s (%.1f m), switching to mode 20" % (sw["a_name"], d_a))
                self.sway_enter_20()
            elif now - sw["t_state"] > 180.0:
                self.sway_abort("approach timeout")
            return
        if st == "to20":
            if t.mode == 20:
                sw["state"], sw["t_state"] = "align", now
                sw["zero_until"] = now + 0.4
                self.event("sway: aligning to %03.0f (now %03.0f)" % (sw["psi_t"], t.hdg))
            elif now - sw["t_state"] > 3.0:
                self.sway_abort("mode 20 not confirmed")
            return
        if t.mode != 20:
            self.sway_abort("mode left 20")
            return
        if abs(sw["c"]) > self.sway_corridor:
            self.sway_abort("cross-track %.1f m > corridor %.1f m" % (sw["c"], self.sway_corridor))
            return
        # controls (companion allocation; the firmware's SERVOn_REVERSED carries the hardware polarity)
        r = max(-self.sway_rmax, min(self.sway_rmax, self.sway_kpsi * sw["e"] / 45.0)) * self.yaw_sign
        u = max(-self.sway_umax, min(self.sway_umax, -self.sway_kc * sw["c"]))
        v = 0.0
        if st == "align":
            if abs(sw["e"]) < 10.0:
                sw["aligned_since"] = sw["aligned_since"] or now
                if now - sw["aligned_since"] > 1.0:
                    sw["state"], sw["t_state"] = "sway", now
                    self.event("sway: aligned (err %.0f), swaying %.1f m at v %.2f" % (sw["e"], sw["length"] - sw["s"], sw["v"]))
            else:
                sw["aligned_since"] = None
            if now - sw["t_state"] > 60.0:
                self.sway_abort("align timeout, err %.0f" % sw["e"])
                return
        elif st == "sway":
            remaining = sw["length"] - sw["s"]
            if remaining <= self.path_end_r:
                sw["state"], sw["t_state"] = "done", now
                self.send_controls([0, 0, 0, 0])
                self.event("sway done: %.0f s, max |c| %.2f m, max |e| %.0f deg" % (now - sw["t_start"], sw["max_c"], sw["max_e"]))
                self.action = {"label": "sway-end", "controls": [0, 0, 0, 0], "phase": "tail", "t_phase_end": now + 0.5, "t_burst": 0}
                return
            taper = max(0.3, min(1.0, remaining / 2.0))
            v = sw["v"] * taper * self.sway_sign * sw.get("dir", 1)
            if abs(sw["e"]) > 60.0 and now - sw["t_state"] > 5.0:
                self.sway_abort("heading error %.0f deg" % sw["e"])
                return
        sw["max_c"] = max(sw["max_c"], abs(sw["c"]))
        sw["max_e"] = max(sw["max_e"], abs(sw["e"]))
        if now < sw["zero_until"]:
            cmd = [0.0, 0.0, 0.0, 0.0]                              # zero-frame rule after mode entry
        else:
            m2, m4 = self.sway_lat(v)
            cmd = [u + r, m2, u - r, m4]
            mx = max(abs(x) for x in cmd)
            if mx > self.args.max_amp:
                cmd = [x * self.args.max_amp / mx for x in cmd]
        sw["cmd"] = cmd
        if now - self.last_send >= 0.1:
            self.send_controls(cmd)
            self.last_send = now
            sw["n"] += 1
            if sw["n"] % 50 == 0:
                self.event("sway %s: s %.1f/%.1f c %+.2f e %+.0f cmd [%s]" % (
                    st, sw["s"], sw["length"], sw["c"], sw["e"], " ".join("%+.2f" % x for x in cmd)))

    # ---------------- pathgen / dockto (ROADMAP WP1: pool-trial helpers by hand) ----------------
    def pathgen_cmd(self, d=2.0):
        """A path from the cv map's gates: (entry) -> for each gate D m before / the midpoint / D m after -> (exit).
        The gates are chained nearest-first from the entry (or the boat); each through-heading is taken away from the
        previous point. Saved as points EN, G1a G1 G1b, ..., EX and planned like `path` (no motion: `pathgo V`)."""
        if self.cvmap is None or not self.origin.is_set:
            self.event("refused: no cv map / control origin (cv on first)")
            return False
        loc = self.local()
        if loc is None:
            self.event("refused: no position yet")
            return False
        gates = self.cvmap.gates(None, min_obs=self.pathgen_min_obs)
        if not gates:
            self.event("refused: no red/green gate in the cv map (cv on, look at the channel; cv status lists the gates)")
            return False
        mk = self.cvmap.markers()
        ent, ext = mk.get("entry"), mk.get("exit")
        cur = (ent.x_n, ent.y_e) if ent else loc
        left, chain = list(gates), []
        while left:
            g = min(left, key=lambda gt: math.hypot(gt.mid_n - cur[0], gt.mid_e - cur[1]))
            left.remove(g)
            th = math.radians(g.through_hdg_deg)
            if (g.mid_n - cur[0]) * math.cos(th) + (g.mid_e - cur[1]) * math.sin(th) < 0:
                th += math.pi                                  # through away from the previous point (red stays starboard: checked below)
            chain.append((g, th))
            cur = (g.mid_n + d * math.cos(th), g.mid_e + d * math.sin(th))
        pts = []                                               # (name, (x, y))
        if ent:
            g0 = chain[0][0]
            ux, uy = g0.mid_n - ent.x_n, g0.mid_e - ent.y_e
            L = math.hypot(ux, uy) or 1.0
            pts.append(("EN", (ent.x_n + 3.0 * ux / L, ent.y_e + 3.0 * uy / L)))
        wrong = []
        for i, (g, th) in enumerate(chain, 1):
            cn, ce = math.cos(th), math.sin(th)
            # red must be to starboard of the through direction: starboard of heading (cn, ce) in (N, E) is (-ce, cn)
            if ((g.red.x_n - g.mid_n) * (-ce) + (g.red.y_e - g.mid_e) * cn) < 0:
                wrong.append(i)
            pts += [("G%da" % i, (g.mid_n - d * cn, g.mid_e - d * ce)), ("G%d" % i, (g.mid_n, g.mid_e)), ("G%db" % i, (g.mid_n + d * cn, g.mid_e + d * ce))]
        if ext:
            lx, ly = pts[-1][1]
            ux, uy = ext.x_n - lx, ext.y_e - ly
            L = math.hypot(ux, uy) or 1.0
            pts.append(("EX", (ext.x_n - 3.0 * ux / L, ext.y_e - 3.0 * uy / L)))
        for name, (x, y) in pts:
            lat, lon = self.origin.from_local(x, y)
            self.points.data[name] = {"lat": lat, "lon": lon, "hdg": None, "utc": utc_stamp(), "sats": None, "hdop": None, "from": "pathgen"}
        self.points.save()
        names = [n for n, _ in pts]
        self.event("pathgen: %s%d gate%s%s, points %s" % ("entry #%d, " % ent.id if ent else "no entry, ", len(chain), "" if len(chain) == 1 else "s",
                                                       (", exit #%d" % ext.id) if ext else ", no exit", " ".join(names)))
        if wrong:
            self.event("pathgen WARNING: gate%s %s would pass RED to port - check the map before pathgo" % ("" if len(wrong) == 1 else "s", ",".join(map(str, wrong))))
        return self.plan_path_cmd(names, True)

    def dock_faces(self, pose):
        """The dock faces of the cv map, numbered 1..n left to right when facing the dock, with the line's normal."""
        d = self.cvmap.dock(pose, time.monotonic()) if self.cvmap is not None else None
        if not d:
            return None, []
        psi = math.radians(d["psi_n"])                         # normal toward the boat (water side)
        nn, ne = math.cos(psi), math.sin(psi)
        sn, se = ne, -nn                                       # starboard when facing the dock (heading = -normal)
        faces = [e for e in self.cvmap.entities("dock") if e.id in d["faces"]]
        faces.sort(key=lambda e: (e.x_n - d["x_n"]) * sn + (e.y_e - d["y_e"]) * se)
        return (d, (nn, ne)), faces

    def dockto_cmd(self, bay=None, speed=None):
        t = self.tel
        if self.fcu_kind() != "fake":
            self.event("refused: dockto is bench / sim only until the m3t1 task is proven (ROADMAP WP3) - fcu fake")
            return
        if not self.origin.is_set or self.local() is None:
            self.event("refused: no position / control origin")
            return
        if not t.armed:
            self.event("refused: not armed")
            return
        if self.action or (self.path and self.path["state"] in ("waiting", "running")) or (self.sway and self.sway["state"] not in ("done", "aborted")):
            self.event("refused: burst, path or sway running")
            return
        pose = self.poses.latest() if self.poses else None
        dk, faces = self.dock_faces(pose)
        if not dk:
            self.event("refused: no dock line in the cv map (cv on, look at the dock: 2+ faces)")
            return
        (d, (nn, ne)) = dk
        if bay is None:
            greens = [i for i, e in enumerate(faces, 1) if e.colour == "green"]
            if not greens:
                self.event("refused: no GREEN bay indicator in the map - dockto BAY (1..%d left to right)" % len(faces))
                return
            bay = greens[0]
        if not 1 <= bay <= len(faces):
            self.event("refused: bay %d (the map has %d, numbered left to right facing the dock)" % (bay, len(faces)))
            return
        f = faces[bay - 1]
        loc = self.local()
        a = (f.x_n + self.dock_standoff * nn, f.y_e + self.dock_standoff * ne)          # abeam the bay, standoff off the face
        ln, le = -ne, nn                                                                  # along the dock line
        s_boat = (loc[0] - a[0]) * ln + (loc[1] - a[1]) * le
        p = (a[0] + s_boat * ln, a[1] + s_boat * le)                                      # the boat projected on the standoff line
        psi_dock = (math.degrees(math.atan2(-ne, -nn))) % 360.0                           # bow to the dock
        self.dock = {"bay": bay, "face": (f.x_n, f.y_e), "normal": (nn, ne), "a": a, "psi": psi_dock, "state": "sway",
                     "t_state": time.monotonic(), "t_start": time.monotonic(), "v_in": speed if speed else self.dock_v,
                     "d": None, "across": 0.0, "e": 0.0, "n": 0, "zero_until": 0.0}
        self.goto = None
        self.pending_goto = None
        length = math.hypot(a[0] - p[0], a[1] - p[1])
        self.event("dockto bay %d (%s indicator) face N %+.1f E %+.1f: sway %.1f m along the dock at %.1f m, then bow in to %.1f m (hdg %03.0f)" % (
            bay, f.colour, f.x_n, f.y_e, length, self.dock_standoff, self.dock_stop, psi_dock))
        if length < 1.0:
            self.dock_bowin(time.monotonic())
            return
        theta = math.degrees(math.atan2(a[1] - p[1], a[0] - p[0])) % 360.0
        dr = 1 if abs(self.wrap180((theta - 90.0) - psi_dock)) < 90.0 else -1
        approach = math.hypot(loc[0] - p[0], loc[1] - p[1]) > 1.5
        self.sway = {"a": p, "b": a, "a_name": "dock-line", "b_name": "bay%d" % bay, "theta": theta, "psi_t": (theta - 90.0 * dr) % 360.0, "dir": dr,
                     "length": length, "v": self.sway_v, "state": "approach" if approach else "to20",
                     "t_state": time.monotonic(), "t_start": time.monotonic(), "s": 0.0, "c": 0.0, "e": 0.0,
                     "cmd": [0, 0, 0, 0], "max_c": 0.0, "max_e": 0.0, "aligned_since": None, "zero_until": 0.0, "n": 0}
        if approach:
            lat, lon = self.origin.from_local(*p)
            self.pending_goto = ("dock-line", lat, lon, None, time.monotonic() + 3.0)
            if t.mode != 15:
                self.set_mode(15)
            else:
                self.pending_goto = None
                self.send_goto("dock-line", lat, lon)
        else:
            self.sway_enter_20()

    def dock_bowin(self, now):
        dk = self.dock
        dk["state"], dk["t_state"], dk["zero_until"] = "bowin", now, now + 0.4
        if self.tel.mode != 20:
            self.set_mode(20)
        self.event("dockto: bow-in to bay %d (hdg %03.0f, stop %.1f m off the face)" % (dk["bay"], dk["psi"], self.dock_stop))

    def dock_abort(self, why):
        if self.dock and self.dock["state"] not in ("docked", "aborted"):
            self.dock["state"], self.dock["t_state"] = "aborted", time.monotonic()
        self.stop("dockto aborted: " + why)

    def service_dock(self, now):
        dk = self.dock
        if dk is None or dk["state"] in ("docked", "aborted"):
            return
        t = self.tel
        if dk["state"] == "sway":
            sw = self.sway
            if sw is None or sw["state"] == "aborted":
                dk["state"], dk["t_state"] = "aborted", now
                self.event("dockto aborted: the sway along the dock stopped")
            elif sw["state"] == "done" and not self.action:
                self.dock_bowin(now)
            return
        loc = self.local()
        if loc is None or t.hdg is None:
            return
        if not t.armed:
            dk["state"] = "aborted"
            self.event("dockto aborted: disarmed")
            return
        if t.mode != 20:
            if now - dk["t_state"] > 3.0:
                self.dock_abort("mode 20 not confirmed / left")
            return
        nn, ne = dk["normal"]
        fx, fy = dk["face"]
        out = (loc[0] - fx) * nn + (loc[1] - fy) * ne                 # metres out of the face along the normal
        across = (loc[0] - fx) * ne + (loc[1] - fy) * (-nn)           # + = to starboard of the bay centreline (facing the dock)
        e = self.wrap180(dk["psi"] - t.hdg)
        dk["d"], dk["across"], dk["e"] = out - self.dock_stop, across, e
        if abs(across) > 1.2:
            self.dock_abort("off the bay centreline by %.1f m" % across)
            return
        if abs(e) > 30.0:
            self.dock_abort("heading error %.0f deg" % e)
            return
        if now - dk["t_state"] > 90.0:
            self.dock_abort("bow-in timeout")
            return
        if dk["d"] <= 0.15:
            dk["state"], dk["t_state"] = "docked", now
            self.send_controls([0, 0, 0, 0])
            self.action = {"label": "dock-end", "controls": [0, 0, 0, 0], "phase": "tail", "t_phase_end": now + 0.5, "t_burst": 0}
            self.event("dockto: DOCKED in bay %d - %.1f m off the face, %+.2f m across, hdg err %+.0f, %.0f s" % (
                dk["bay"], out, across, e, now - dk["t_start"]))
            return
        r = max(-self.sway_rmax, min(self.sway_rmax, self.sway_kpsi * e / 45.0)) * self.yaw_sign
        u = max(0.0, min(dk["v_in"], 0.1 * dk["d"] + 0.03))
        v = max(-0.2, min(0.2, -0.4 * across)) * self.sway_sign
        if now < dk["zero_until"]:
            cmd = [0.0, 0.0, 0.0, 0.0]
        else:
            m2, m4 = self.sway_lat(v)
            cmd = [u + r, m2, u - r, m4]
            mx = max(abs(x) for x in cmd)
            if mx > self.args.max_amp:
                cmd = [x * self.args.max_amp / mx for x in cmd]
        if now - self.last_send >= 0.1:
            self.send_controls(cmd)
            self.last_send = now
            dk["n"] += 1
            if dk["n"] % 50 == 0:
                self.event("dockto bow-in: %.1f m to go, across %+.2f, e %+.0f" % (dk["d"], across, e))

    # ---------------- v4: link share, MQTT, perception map, FCU handover ----------------
    FCU_CMDS = ("surge", "sway", "yaw", "motor", "burst", "swaycal", "sway_calibr", "swaycalibr", "mode", "arm",
                "goto", "path", "pathto", "pathgo", "swayto", "dockto", "speed", "param")

    def service_share(self):
        if self.share is not None:
            self.share.to_fcu(self.m)

    def share_line(self):
        if self.share is None:
            return "share off" if not self.share_err else "share FAILED: %s" % self.share_err
        s = self.share
        age = ("%.0fs ago" % (time.monotonic() - s.t_in)) if s.t_in else "never"
        peer = ("sys %d comp %d%s" % (s.peer[0], s.peer[1], " = mission manager" if s.peer == (self.args.sysid, MM_COMPID) else "")) if s.peer else "no tool yet"
        return "share %s: out %d in %d, last tool %s (%s)" % (s.client, s.n_out, s.n_in, peer, age)

    def mq_line(self):
        if self.mq is None:
            return "mqtt off" if not self.mq_err else "mqtt FAILED: %s" % self.mq_err
        return "mqtt %s:%d %s" % (self.mq.host, self.mq.port, "ok" if self.mq.connected else "CONNECTING")

    def free_run_banner(self):
        """v0.1.68 (WP17 F6): ' FREE RUN ... ' while the mission manager's mm/state says nav ghost (fresh), else ''."""
        if self.mm is None or time.monotonic() - self.mm[0] > 5.0 or (self.mm[1] or {}).get("nav") != "ghost":
            return ""
        extra = ((self.mm[1] or {}).get("free_run") or {}).get("banner")
        return " FREE RUN: ghost position, mode requests only to the FCU - NEVER at the pool %s" % (("· %s " % extra) if extra else "")

    def mm_line(self):
        """The mission manager as seen on local/<v>/mm/state."""
        if self.mm is None:
            return "mm: not seen"
        t, b = self.mm
        age = time.monotonic() - t
        if age > 3.0:
            return "mm: SILENT %.0f s (state was %s)" % (age, b.get("state"))
        s = "mm: %s" % b.get("state")
        if b.get("task_name"):
            s += " %s/%s" % (b.get("task_name"), b.get("task_state"))
        if b.get("hold_reason"):
            s += " (%s)" % b["hold_reason"]
        loc = b.get("local") or {}
        if loc.get("co_lat") is not None and self.origin.is_set:
            d = haversine(loc["co_lat"], loc["co_lon"], self.origin.rec["lat"], self.origin.rec["lon"])
            s += "  CO %s" % ("same" if d < 1.0 else "DIFFERS %.0f m" % d)
        return s

    def set_active(self, on, why):
        if on == self.active:
            return
        self.active = on
        if on:
            self.last_beat = 0.0               # beat now
            self.event("ACTIVE: usv_ui commands the FCU again (%s); the mission manager drops to MANUAL" % why)
        else:
            if self.mq is not None:
                self.mq.publish("local/%s/tui/active" % self.args.vehicle, "local", {"active": False, "node": "usv_ui", "t": utc_iso()})
            self.event("HANDED OVER: the mission manager may take the FCU (OCS: ocs_cli ready); you stay here - 'takeover' (or stop / x / disarm) takes it back")

    # ---------------- v0.1.45 (WP10f phases 1-2): the boat window's FCU block and its whitelisted words ----------------
    PAGE_WORDS = ("handover", "takeover", "stop", "log", "note", "save", "co", "cv", "points",   # decision 25 ...
                  "mode", "arm", "disarm")    # v0.1.50 (Dmitrii 2026-11-06): + the FCU's mode and ARM / DISARM, sent even while the
                                              # mission manager holds the FCU (as Mission Planner would); bursts, E-stop, thrusters, cannon: no
    PAGE_FCU = ("mode", "arm", "disarm")
    PAGE_GAP_S = 0.5                 # flood guard: one accepted page word per 0.5 s
    PAGE_STALE_S = 10.0              # a word whose t (unix) is older than this is refused (the OCS's stale-command rule)

    def fcu_block(self, now):
        """What this console knows about the FCU, for local/<v>/ui/fcu (2 Hz, boat broker only, never bridged). A value the FCU
        has not sent is None - never a zero or a guess."""
        t, ms = self.tel, self.mstats

        def last(k):
            st = ms.get(k)
            return st["last"] if st else None

        def age(k):
            st = ms.get(k)
            return round(now - st["times"][-1], 2) if st and st["times"] else None

        def rate(k):
            st = ms.get(k)
            if not st or len(st["times"]) < 2 or now - st["times"][-1] > 3.0:
                return None
            ts = st["times"]
            return round((len(ts) - 1) / max(1e-3, ts[-1] - ts[0]), 1)

        hb = t.hb_t is not None
        ss, vfr, nav, att, gps, so = (last(k) for k in ("SYS_STATUS", "VFR_HUD", "NAV_CONTROLLER_OUTPUT", "ATTITUDE", "GPS_RAW_INT",
                                                        "SERVO_OUTPUT_RAW"))
        named = {k: (bool(v), round(now - tv, 2)) for k, (v, tv) in t.named.items()}
        rc = list(t.rc) if t.rc else None
        kinds = ("HEARTBEAT", "SYS_STATUS", "RC_CHANNELS", "SERVO_OUTPUT_RAW", "EKF_STATUS_REPORT", "VFR_HUD", "NAV_CONTROLLER_OUTPUT",
                 "GLOBAL_POSITION_INT", "GPS_RAW_INT", "ATTITUDE", "NAMED_VALUE_INT")
        return {
            "t": utc_iso(), "t_unix": round(time.time(), 3), "node": "usv_ui",
            "fcu_link": self.fcu_kind(), "fcu_conn": self.args.conn,
            "active": self.active, "holder": "usv_ui" if self.active else "mission manager",
            "own": [self.args.sysid, OWN_COMPID],
            "hb_age_s": round(now - t.hb_t, 2) if hb else None,
            "sysid": t.sysid, "compid": t.compid,
            "mode": (MODE_NAMES.get(t.mode, str(t.mode)) if t.mode is not None else None) if hb else None,
            "mode_num": t.mode if hb else None,
            "armed": t.armed if hb else None,
            "kill": named["KILL"][0] if "KILL" in named else None,
            "estop": named["ESTOP"][0] if "ESTOP" in named else None,
            "lamp": int(t.named["LAMP"][0]) if "LAMP" in t.named else None,          # v0.1.55 (D4): read-back hook, null = not sent
            "lamp_age_s": named["LAMP"][1] if "LAMP" in named else None,
            "rc": rc,
            "rc5_estop": (rc[4] > 1750) if rc and len(rc) > 4 and rc[4] else None,
            "rc6_override": (rc[5] > 1750) if rc and len(rc) > 5 and rc[5] else None,
            "servo": [getattr(so, "servo%d_raw" % i, None) for i in range(1, 9)] if so is not None else None,
            "ekf_flags": t.ekf_flags if last("EKF_STATUS_REPORT") is not None else None,
            "ekf_pos_ok": t.ekf_pos_ok() if last("EKF_STATUS_REPORT") is not None else None,
            "voltage_v": t.batt if ss is not None else None,
            "current_a": (ss.current_battery / 100.0) if ss is not None and ss.current_battery not in (-1,) else None,
            "battery_pct": ss.battery_remaining if ss is not None and ss.battery_remaining not in (-1,) else None,
            "heading_deg": vfr.heading if vfr is not None else None,
            "groundspeed_mps": round(vfr.groundspeed, 2) if vfr is not None else None,
            "nav": {"nav_bearing_deg": nav.nav_bearing, "target_bearing_deg": nav.target_bearing,
                    "xtrack_m": round(nav.xtrack_error, 2), "wp_dist_m": nav.wp_dist} if nav is not None else None,
            "lat": t.lat, "lon": t.lon, "hdg_deg": t.hdg,
            "gps": {"fix": gps.fix_type, "sats": gps.satellites_visible, "hdop": None if gps.eph == 65535 else gps.eph / 100.0}
            if gps is not None else None,
            "roll_deg": round(math.degrees(att.roll), 1) if att is not None else None,
            "pitch_deg": round(math.degrees(att.pitch), 1) if att is not None else None,
            "yawspeed_dps": round(math.degrees(att.yawspeed), 2) if att is not None else None,     # v0.1.51: the boat window's ROT scale
            "ages_s": {k: age(k) for k in kinds}, "rates_hz": {k: rate(k) for k in kinds},
            "log": os.path.basename(self.base) if self.base else None, "page_words": self.n_remote,
            "cv_on": self.cv_on,                    # v0.1.52: the boat window's top plate (cv on / off)
            "tail": [[round(now - tl, 1), str(tx)[:160], kd] for tl, tx, kd in self.log[-12:]],    # v0.1.49: the console, read-only (age s, text, kind)
        }

    def remote_cmd(self, t_recv, env):
        """local/<v>/ui/cmd {word, args?, id?, t?} from the boat window: a whitelisted word runs exactly as typed here; every word,
        run or refused, is answered on local/<v>/ui/cmd/result {id, word, ok, text} and shown in the console as "[page] word"."""
        req = env.get("body") if isinstance(env, dict) and isinstance(env.get("body"), dict) else env
        if not isinstance(req, dict):
            return
        word = str(req.get("word") or "").strip().lower()
        args = req.get("args")
        args = " ".join(str(a) for a in args) if isinstance(args, (list, tuple)) else str(args or "")
        args = "".join(ch for ch in args if ch.isprintable())[:200].strip()
        line = (word + " " + args).strip()
        now = time.monotonic()
        ok, text = False, None
        if word not in self.PAGE_WORDS:
            text = ("refused: '%s' is not a page word - the page may send only %s; bursts, the E-stop, thrusters and "
                    "the cannon stay on this console and the transmitter (decision 25, v0.1.50)" % (word or "?", " ".join(self.PAGE_WORDS)))
        elif isinstance(req.get("t"), (int, float)) and time.time() - req["t"] > self.PAGE_STALE_S:
            text = "refused: stale page word (%.0f s old)" % (time.time() - req["t"])
        elif now - self.last_remote < self.PAGE_GAP_S:
            text = "refused: too fast - one page word per %.1f s" % self.PAGE_GAP_S
        if text is not None:
            self.log.append((now, "[page] %s" % (line or "?"), "cmd"))
            self._in_cmd = True
            try:
                self.event(text)
            finally:
                self._in_cmd = False
        else:
            self.last_remote = now
            self.n_remote += 1
            self._cmd_out = []
            try:
                self.do_command(line, src="page")
                out = self._cmd_out
            finally:
                self._cmd_out = None
            text = "; ".join(out[:4]) if out else "done"
            ok = not text.startswith(("refused", "bad arguments", "unknown"))
        if self.mq is not None:
            self.mq.publish("local/%s/ui/cmd/result" % self.args.vehicle, "local",
                            {"id": req.get("id"), "word": word, "ok": ok, "text": text, "active": self.active, "t": utc_iso()})

    def service_mqtt(self, now):
        mq = self.mq
        if mq is None:
            return
        # inbound
        for _ in range(200):
            try:
                t_recv, topic, env = mq.inbox.popleft()
            except IndexError:
                break
            self._on_mqtt(t_recv, topic, env)
        if not mq.connected:
            return
        # the handover beat: the mission manager treats a beat younger than tui_active_s (2 s) as "TUI active"
        if self.active and now - self.last_beat >= 1.0:
            mq.publish("local/%s/tui/active" % self.args.vehicle, "local", {"active": True, "node": "usv_ui", "t": utc_iso()})
            self.last_beat = now
        if now - self.last_presence >= 2.0:
            mq.presence(True, {"fcu_link": self.fcu_kind(), "fcu_conn": self.args.conn, "log": os.path.basename(self.base) if self.base else None,
                               "active": self.active, "cv_on": self.cv_on,         # v0.1.52: cv_on for the boat window
                               "log_t0": getattr(self, "log_t0", None) if self.base else None, "log_bytes": self.log_bytes()})   # v0.1.83.5
            self.last_presence = now
        if now - self.last_map >= 0.5:
            self.publish_map(now)
            self.last_map = now
        if now - self.last_fcu_pub >= 0.5:
            mq.publish("local/%s/ui/fcu" % self.args.vehicle, "local", self.fcu_block(now))
            self.last_fcu_pub = now
        if self.cvmap is not None:
            self.cvmap.age_out(now)

    def _on_mqtt(self, t_recv, topic, env):
        if topic.startswith("$SYS/broker/connection/"):
            up = str((env or {}).get("raw", "")).strip() == "1"
            if up != self.bridge:
                self.bridge, self.bridge_t = up, t_recv
                self.event("OCS LINK %s: the boat broker's bridge to the hub (%s)" % ("UP" if up else "DOWN", topic.split("/")[3])
                           + ("" if up else " - the mission manager continues on its own (%s); the OCS will alert status stale" % self.link_loss_text()))
            return
        if topic.startswith("presence/"):
            node = topic[len("presence/"):]
            if isinstance(env, dict) and node != "usv_ui":
                prev = self.presence.get(node)
                tu = env.get("t_unix")
                if isinstance(tu, (int, float)) and time.time() - tu > 60.0:
                    t_recv = t_recv - (time.time() - tu)                 # a retained beat from an earlier run: already old
                self.presence[node] = (t_recv, env)
                if prev is not None and bool(prev[1].get("up")) != bool(env.get("up")):
                    self.event("%s on the boat broker: %s" % (node, "up" if env.get("up") else "DOWN (clean stop)"))
            return
        if topic == "local/%s/ui/cmd" % self.args.vehicle:
            self.remote_cmd(t_recv, env)
            return
        body = env.get("body") if isinstance(env, dict) else None
        if not isinstance(body, dict):
            return
        if topic == "ocs/state" or topic.endswith("/ocs/state"):
            prev = self.ocs[1] if self.ocs else None
            self.ocs = (t_recv, body)
            if prev is not None and (prev.get("r") != body.get("r") or (prev.get("mission") or {}).get("task") != (body.get("mission") or {}).get("task")):
                mi = body.get("mission") or {}
                self.event("OCS: %s %s  mission %s%s" % (body.get("r"), body.get("r_state"), mi.get("task") or "-", " (auto)" if mi.get("auto") else ""))
            return
        if topic.endswith("/mm/state"):
            prev = self.mm[1] if self.mm else None
            self.mm = (t_recv, body)
            if prev is not None and (prev.get("state") != body.get("state") or prev.get("task_state") != body.get("task_state") or prev.get("task_name") != body.get("task_name")):
                self.event("mm: %s%s -> %s%s" % (prev.get("state"), (" %s/%s" % (prev.get("task_name"), prev.get("task_state"))) if prev.get("task_name") else "",
                                                 body.get("state"), (" %s/%s" % (body.get("task_name"), body.get("task_state"))) if body.get("task_name") else ""))
            return
        if "/perception/" not in topic:
            return
        kind = topic.rsplit("/", 1)[-1]
        if kind == "health":
            self.cv_health = (t_recv, body)
            return
        self.cv_n_rx += 1
        self.cv_t_rx = t_recv
        if not self.cv_on or self.cvmap is None:
            return
        try:
            obs = Observation.from_body(body, kind=kind)
        except Exception as e:
            self.event("cv: bad observation on %s: %s" % (topic, e))
            return
        age = (time.time() - obs.t_cam) if obs.t_cam > 0 else 0.0   # capture -> receipt latency, by the sender's clock
        if age < 0.0 or age > 1.0:                                    # the detector's clock is off: use the latest pose
            self.cv_skew += 1
            if self.cv_skew in (1, 100, 1000):
                self.event("cv: t_cam %+.1f s off our clock (sync the AGX and NX clocks); using the latest pose" % age)
            age = 0.0
        t_mono = t_recv - age                                         # capture time on our monotonic clock
        pose = self.poses.at(t_mono) or self.poses.latest()
        if pose is None:
            return
        before = set(self.cvmap.ents)
        used = self.cvmap.ingest(obs, pose, now=t_recv, source="usv")
        self.cv_n_used += used
        for eid in sorted(set(self.cvmap.ents) - before):
            e = self.cvmap.ents[eid]
            self.cv_known.add(eid)
            self.event("map: new %s #%d %s at N %+.1f E %+.1f (from %d item%s)" % (e.kind, eid, e.colour, e.x_n, e.y_e, len(obs.items), "" if len(obs.items) == 1 else "s"))

    def cv_cmd(self, sub):
        if sub == "on":
            if self.cvmap is None:
                self.event("refused: optiprimex not importable (%s) - put MASTER_SCRIPTS/bench next to this folder or on PYTHONPATH" % OPTIPRIMEX_ERR)
                return
            if self.mq is None:
                self.event("refused: no MQTT (%s)" % (self.mq_err or "--mqtt ''"))
                return
            if not self.origin.is_set:
                self.event("refused: no control origin yet (co / first fix)")
                return
            self.cv_on = True
            self.event("cv ON: ingesting local/%s/perception/# into the map (%s)" % (self.args.vehicle, self.mq_line()))
        elif sub == "off":
            self.cv_on = False
            self.event("cv OFF: map kept (%d entities); 'cv clear' drops it" % (len(self.cvmap.ents) if self.cvmap else 0))
        elif sub == "clear":
            if self.cvmap is not None:
                self.cvmap.clear()
            self.cv_known = set()
            self.cv_n_used = 0
            self.event("cv: map cleared")
        elif sub in ("", "status"):
            self.event("cv %s  %s  %s" % ("ON" if self.cv_on else "off", self.mq_line(), self.share_line()))
            if self.cv_health is None:
                self.event("cv: no perception/health yet (AGX cv_publish.py not running, or not on this broker)")
            else:
                t, h = self.cv_health
                self.event("cv: detector %s ok=%s rate %s Hz pose_age %s s, health %.1f s ago, n_published %s" % (
                    h.get("detector"), h.get("ok"), h.get("rate_hz"), h.get("pose_age_s"), time.monotonic() - t, h.get("n_published")))
            self.event("cv: observations rx %d (%s), items used %d, entities %d" % (
                self.cv_n_rx, ("%.1f s ago" % (time.monotonic() - self.cv_t_rx)) if self.cv_t_rx else "none",
                self.cv_n_used, len(self.cvmap.ents) if self.cvmap else 0))
            if self.cvmap is not None:
                now = time.monotonic()
                mo = getattr(self.cvmap.m, "min_obs", 1)
                cols = {}
                for e in self.cvmap.ents.values():
                    if e.n_obs >= mo:
                        cols[e.colour] = cols.get(e.colour, 0) + 1
                young = sum(1 for e in self.cvmap.ents.values() if e.n_obs < mo)
                self.event("cv: colours %s; young (< %d obs, not in the map) %d" % (", ".join("%s %d" % kv for kv in sorted(cols.items())) or "-", mo, young))
                for e in sorted(self.cvmap.ents.values(), key=lambda e: e.id):
                    iv = getattr(e, "ind_votes", None) or {}             # v0.1.48: a dock face's indicator votes, a window's number / light
                    extra = ("  votes R%d G%d" % (iv.get("red", 0), iv.get("green", 0))) if e.kind == "dock" and iv else \
                        ("  window %s light %s" % (getattr(e, "window_no", None) or "?", e.light) if e.kind == "window" else "")
                    self.event("  #%d %s %s N %+.1f E %+.1f sigma %.1f n %d %s %.0f s%s%s" % (e.id, e.kind, e.colour, e.x_n, e.y_e, e.sigma_m, e.n_obs, e.state, now - e.t_last,
                                                                                        extra, "  (young)" if e.n_obs < mo else ""))
                pose = self.poses.latest() if self.poses else None
                for g in self.cvmap.gates(pose):
                    self.event("  gate #%d/#%d mid N %+.1f E %+.1f through %03.0f width %.1f" % (g.red.id, g.green.id, g.mid_n, g.mid_e, g.through_hdg_deg, g.width_m))
                d = self.cvmap.dock(pose, now)
                if d:
                    self.event("  dock N %+.1f E %+.1f normal %03.0f line %03.0f faces %s" % (d["x_n"], d["y_e"], d["psi_n"], d["line_hdg_deg"], d["faces"]))
        else:
            raise ValueError("cv on | off | status | clear")

    def mm_cmd(self, action):
        """RESET / STEP to the mission manager over the boat broker (the same cmd envelope the OCS sends)."""
        if self.mq is None or not self.mq.connected:
            self.event("refused: no MQTT connection to reach the mission manager (%s)" % self.mq_line())
            return
        if self.mm is None:
            self.event("refused: the mission manager has not been seen on local/%s/mm/state" % self.args.vehicle)
            return
        topic = "%s/cmd/%s/run" % (TEAM_NS, self.args.vehicle)
        self.mq.publish(topic, "cmd", {"action": action, "task": None, "args": {}})
        self.event("%s sent to the mission manager (%s); watch the MISSION MANAGER box - its ack goes to the OCS log%s" % (
            action, topic, "; the OCS keeps its run: `reset` there too (ocs_cli reset) for both ends" if action == "RESET" else ""))

    # ---------------- the FCU link: fake plant <-> the Pixhawk, switched while running ----------------
    FAKE_PREFIX = "udpin:127.0.0.1:"       # a plant on the loopback = the bench's fake FCU (a real one over Ethernet is udpin:0.0.0.0:14550)

    def fcu_kind(self):
        return "fake" if str(self.args.conn).startswith(self.FAKE_PREFIX) else "real"

    def fake_port(self):
        """The loopback port of the fake plant: the current link when it is the fake one, else the last one used (14550 default)."""
        if self.fcu_kind() == "fake":
            self._fake_port = int(self.args.conn[len(self.FAKE_PREFIX):])
        return getattr(self, "_fake_port", 14550)

    def fcu_cmd(self, args):
        what = (args[0].lower() if args else "status")
        if what == "status":
            t = self.tel
            hb = ("HB %.1f s ago" % (time.monotonic() - t.hb_t)) if t.hb_t else "no heartbeat"
            child = ("fake plant started here, pid %d" % self.fake_proc.pid) if self.fake_proc and self.fake_proc.poll() is None else "no plant process of ours"
            self.event("fcu: %s link %s, %s, sys %s mode %s %s; %s" % (self.fcu_kind(), self.args.conn, hb, t.sysid,
                       MODE_NAMES.get(t.mode, t.mode), "ARMED" if t.armed else "disarmed", child))
            return
        if what not in ("fake", "real"):
            self.event("usage: fcu fake | real [usb|eth|<endpoint>] | status")
            return
        if not self.active:
            self.event("refused: handed over - the mission manager commands the FCU; 'takeover' first")
            return
        if self.tel.armed and self.fcu_kind() == "real":
            self.event("refused: the Pixhawk is ARMED - 'disarm' first (it would lose its control stream)")
            return
        if self.action or (self.path and self.path["state"] in ("waiting", "running")) or (self.sway and self.sway["state"] not in ("done", "aborted")) or self.goto:
            self.event("refused: something is running - 'stop' first")
            return
        if what == "fake":
            self._fake_port = int(args[1]) if len(args) > 1 else self.fake_port()
            conn, baud = self.FAKE_PREFIX + str(self._fake_port), 115200
        else:
            try:
                conn, baud = resolve_link(args[1] if len(args) > 1 else "uart", None)
            except SystemExit as e:
                self.event("fcu real: %s" % e)
                return
        if conn == self.args.conn:
            self.event("fcu: already on %s" % conn)
            return
        try:
            m = mavutil.mavlink_connection(conn, baud=baud, source_system=self.args.sysid, source_component=OWN_COMPID, autoreconnect=True)
        except Exception as e:
            self.event("fcu %s: cannot open %s: %s" % (what, conn, e))
            return
        old, self.m = self.m, m
        try:
            old.close()
        except Exception:
            pass
        if what == "real" and self.fake_proc and self.fake_proc.poll() is None:
            self.fake_proc.terminate()
            self.event("fcu: fake plant (pid %d) stopped" % self.fake_proc.pid)
            self.fake_proc = None
        self.args.conn, self.args.baud = conn, baud
        self.tel = Telemetry()
        self.mstats = {}
        self.pending_goto = None
        self.await_hb = {"t0": time.monotonic(), "kind": what, "spawn_tried": False, "warned": False}
        self.event("fcu: switching to %s on %s - waiting for its heartbeat (share + mission manager stay; a gap > 2 s = mm FAULT FCU_LOST -> ocs_cli confirm_safe%s)"
                   % (what, conn, "; a real Pixhawk indoors needs the mission manager started with --lab" if what == "real" else ""))

    def service_fcu_switch(self, now):
        """After `fcu fake|real`: start a fake plant when nothing answers on udp 14550, request streams at the first heartbeat."""
        w = self.await_hb
        if w is None:
            return
        if self.tel.hb_t is not None and self.tel.hb_t >= w["t0"]:
            self.await_hb = None
            self.request_streams()
            self.event("fcu: %s link up - sys %s comp %s, streams requested" % (w["kind"], self.m.target_system, self.m.target_component))
            return
        waited = now - w["t0"]
        if w["kind"] == "fake" and waited > 2.0 and not w["spawn_tried"]:
            w["spawn_tried"] = True
            self.start_fake_plant()
        elif waited > 10.0 and not w["warned"]:
            w["warned"] = True
            self.event("fcu: still no heartbeat on %s after 10 s (%s) - 'fcu real' / 'fcu fake' to switch again" % (
                self.args.conn, "is the Pixhawk powered and the cable right?" if w["kind"] == "real" else "the fake plant did not start: see LOG"))

    def start_fake_plant(self):
        """The bench's kinematic fake FCU (optiprimex.sim.fake_fcu) as our child, sending to udp 14550."""
        bench = next((c for c in sys.path if os.path.isdir(os.path.join(c, "optiprimex"))), None)
        if bench is None:
            self.event("fcu fake: optiprimex not importable (optiprimex/ at the tree root above usv_ui/?) - cannot start the fake plant")
            return
        env = dict(os.environ, PYTHONPATH=bench + (os.pathsep + os.environ["PYTHONPATH"] if os.environ.get("PYTHONPATH") else ""))
        log_path = os.path.join(self.args.log_dir if self.args.log_dir else _HERE, "fake_fcu_%s.log" % time.strftime("%Y%m%d_%H%M%S"))
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            out = open(log_path, "w")
            self.fake_proc = subprocess.Popen([sys.executable, "-m", "optiprimex.sim.fake_fcu", "--conn", "udpout:127.0.0.1:%d" % self.fake_port(),
                                               "--control-port", "14560", "--armed"], env=env, stdout=out, stderr=subprocess.STDOUT)
        except Exception as e:
            self.event("fcu fake: cannot start the fake plant: %s" % e)
            return
        self.event("fcu fake: nothing answered on udp %d - started the bench plant here (pid %d, log %s)" % (self.fake_port(), self.fake_proc.pid, log_path))

    def map_cmd(self):
        """The browser map (MASTER_SCRIPTS/optiprimex/tools/map.html) over this boat broker's websocket: open it here
        when a desktop is available, else print the URL to open on the Mac (host = this machine's address)."""
        html = None
        for cand in (os.path.join(_HERE, "..", "optiprimex", "tools", "map.html"), os.path.join(_HERE, "optiprimex", "tools", "map.html")):
            if os.path.exists(cand):
                html = os.path.abspath(cand)
                break
        try:
            sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sk.connect(("10.255.255.255", 1))
            my_ip = sk.getsockname()[0]
            sk.close()
        except OSError:
            my_ip = "127.0.0.1"
        host = self.mq.host if (self.mq and self.mq.host not in ("127.0.0.1", "localhost")) else my_ip
        url = "file://%s?host=%s&port=%d&source=usv_ui" % (html or "<MASTER_SCRIPTS>/optiprimex/tools/map.html", host, self.args.ws)
        opened = False
        if html and (sys.platform == "darwin" or os.environ.get("DISPLAY")):
            try:
                import webbrowser
                opened = webbrowser.open(url)
            except Exception:
                opened = False
        self.event(("map opened in the browser: %s" if opened else "map (open on the Mac, the boat broker's ws %d): %s") % ((url,) if opened else (self.args.ws, url)))

    def publish_map(self, now):
        """telemetry/<v>/map in the mission manager's shape (+ points, cv) for map.html and the bench TUI."""
        if self.mq is None or not self.origin.is_set:
            return
        loc = self.local()
        t = self.tel
        o = self.origin.rec
        pts = {}
        for name, p in self.points.data.items():
            pts[name] = [round(v, 2) for v in self.origin.to_local(p["lat"], p["lon"])]
        pose = self.poses.latest() if self.poses else None
        dig = self.cvmap.digest(pose, now) if self.cvmap is not None else {"entities": [], "gates": [], "dock": None, "n_ingested": 0, "ingest_age_s": None}
        label = (self.action["label"] if self.action else
                 ("path %s" % "-".join(self.path["anchors"])) if self.path and self.path["state"] in ("waiting", "running") else
                 ("dock %s" % self.dock["state"]) if self.dock and self.dock["state"] not in ("docked", "aborted") else
                 ("sway %s" % self.sway["state"]) if self.sway and self.sway["state"] not in ("done", "aborted") else
                 ("goto %s" % self.goto["name"]) if self.goto else "idle")
        body = {"t": utc_iso(), "co": {"lat": o["lat"], "lon": o["lon"]},
                "boat": None if loc is None else {"x_n": round(loc[0], 2), "y_e": round(loc[1], 2), "hdg_deg": t.hdg,
                                                  "trail": [[round(a, 2), round(b, 2)] for a, b in self.trail[-50:]]},
                "course_boundary_local": [], "path": [[round(a, 2), round(b, 2)] for a, b in (self.path["pts"] if self.path else [])],
                "carrot": None if not (self.path and self.path["carrot"]) else [round(v, 2) for v in self.path["carrot"]],
                "task_state": "ACTIVE" if self.active else "HANDED_OVER", "task": label, "points": pts,
                "cv": {"on": self.cv_on, "n_rx": self.cv_n_rx, "n_used": self.cv_n_used,
                       "health_age_s": None if self.cv_health is None else round(now - self.cv_health[0], 1)},
                "mode": MODE_NAMES.get(t.mode, t.mode), "armed": t.armed}
        body.update(dig)
        self.mq.publish("%s/telemetry/%s/map" % (TEAM_NS, self.args.vehicle), "telemetry", body)

    # ---------------- commands ----------------
    def do_command(self, line, src=None):
        parts = line.strip().split()
        if not parts:
            return
        self.log.append((time.monotonic(), ("[%s] " % src if src else "> ") + line.strip(), "cmd"))
        self._in_cmd = True
        self._src = src
        try:
            self._do_command(parts)
        finally:
            self._in_cmd = False
            self._src = None

    def _do_command(self, parts):
        cmd, args = parts[0].lower(), parts[1:]
        page_fcu = getattr(self, "_src", None) == "page" and cmd in self.PAGE_FCU   # v0.1.50: the page's mode / arm: no takeover
        if not self.active and cmd in self.FCU_CMDS and not page_fcu:
            self.event("refused: handed over to the mission manager - 'takeover' takes the FCU back (stop / x / disarm do it at once)")
            return
        if not self.active and cmd in ("stop", "x", "disarm") and not page_fcu:
            self.set_active(True, cmd)
        try:
            if cmd in ("surge", "sway", "yaw"):
                a = float(args[0]) if args else self.args.amp
                dur = float(args[1]) if len(args) > 1 else self.args.dur
                if cmd == "sway":
                    m2, m4 = self.sway_lat(a)
                    self.start_burst("sway %+.2f %s" % (a, "cal" if self.swaymix else "fw"), [0, m2, 0, m4], dur)
                else:
                    c = {"surge": [a, 0, a, 0], "yaw": [a, 0, -a, 0]}[cmd]
                    self.start_burst("%s %+.2f" % (cmd, a), c, dur)
            elif cmd == "motor":
                n = int(args[0])
                a = float(args[1]) if len(args) > 1 else self.args.amp
                dur = float(args[2]) if len(args) > 2 else self.args.dur
                if not 1 <= n <= 4:
                    raise ValueError("motor 1..4")
                c = [0, 0, 0, 0]
                c[n - 1] = a
                self.start_burst("%s %+.2f" % (MOTOR_NAMES[n - 1], a), c, dur)
            elif cmd == "burst":
                c = [float(x) for x in args[:4]]
                dur = float(args[4]) if len(args) > 4 else self.args.dur
                self.start_burst("burst", c, dur)
            elif cmd in ("swaycal", "sway_calibr", "swaycalibr"):
                a = float(args[0])
                dur = float(args[1]) if len(args) > 1 else self.args.dur
                f = float(args[2]) if len(args) > 2 else self.sway_factor(a)
                self.swaycal_cmd(a, dur, f)
            elif cmd == "swaymix":
                sub = args[0].lower() if args else ""
                if sub in ("on", "cal"):
                    self.swaymix = True
                elif sub in ("off", "fw"):
                    self.swaymix = False
                elif sub:
                    raise ValueError("swaymix on | off")
                self.event("sway mix: %s  (sway bursts, swayto, swaycal default)" % self.swaymix_text())
            elif cmd in ("stop", "x"):
                self.stop()
            elif cmd == "mode":
                s = args[0].lower()
                n = MODE_ALIASES[s] if s in MODE_ALIASES else int(s)
                if n in (15, 10, 5) and self.tel.armed and not (self.tel.fix >= 3 and self.tel.ekf_pos_ok()):
                    self.event("warning: armed and no EKF position -> %s will be refused" % MODE_NAMES.get(n, n))
                self.set_mode(n)
            elif cmd == "arm":
                self.arm(True)
            elif cmd == "disarm":
                if self.active:                      # v0.1.50: a page DISARM while handed over leaves the mission manager's motion alone
                    self.stop("disarm")
                self.arm(False)
            elif cmd == "here":
                t = self.tel
                self.event("here: %s %s hdg %s fix %d sats %d" % (t.lat, t.lon, t.hdg, t.fix, t.sats))
            elif cmd == "save":
                name = args[0]
                t = self.tel
                if t.lat is None or t.fix < 3:
                    self.event("refused: no 3D fix / position")
                    return
                self.points.data[name] = {"lat": t.lat, "lon": t.lon, "hdg": t.hdg, "utc": utc_stamp(),
                                          "sats": t.sats, "hdop": t.hdop}
                self.points.save()
                self.event("saved %s = %.7f %.7f (hdg %s)" % (name, t.lat, t.lon, t.hdg))
            elif cmd == "points":
                if not self.points.data:
                    self.event("no points")
                for k, v in sorted(self.points.data.items()):
                    self.event("%s: %.7f %.7f hdg %s @%s" % (k, v["lat"], v["lon"], v.get("hdg"), v.get("utc")))
            elif cmd == "del":
                self.points.data.pop(args[0], None)
                self.points.save()
                self.event("deleted %s" % args[0])
            elif cmd == "goto":
                name = args[0]
                p = self.points.data.get(name)
                if p is None:
                    self.event("unknown point %s" % name)
                    return
                t = self.tel
                if not t.armed:
                    self.event("refused: not armed")
                    return
                if not (t.fix >= 3 and t.ekf_pos_ok()):
                    self.event("refused: fix %d / EKF pos %s" % (t.fix, t.ekf_pos_ok()))
                    return
                speed = float(args[1]) if len(args) > 1 else None
                if t.mode != 15:
                    self.set_mode(15)
                    self.pending_goto = (name, p["lat"], p["lon"], speed, time.monotonic() + 3.0)
                    self.event("goto %s queued until GUIDED is confirmed" % name)
                else:
                    self.send_goto(name, p["lat"], p["lon"], speed)
            elif cmd in ("path", "pathto"):
                names = [a for a in args if not self._is_number(a)]
                nums = [float(a) for a in args if self._is_number(a)]
                smooth = True
                if names and names[-1].lower() in ("straight", "line"):
                    smooth = False
                    names = names[:-1]
                if not names:
                    raise ValueError("path NAME [NAME ...] [straight] [V]")
                if self.plan_path_cmd(names, smooth):
                    if cmd == "pathto":
                        self.start_path(nums[0] if nums else None)
            elif cmd == "swayto":
                names = [a for a in args if not self._is_number(a)]
                nums = [float(a) for a in args if self._is_number(a)]
                self.swayto_cmd(names, nums[0] if nums else None)
            elif cmd == "pathgen":
                self.pathgen_cmd(float(args[0]) if args else 2.0)
            elif cmd == "dockto":
                bays = [int(a) for a in args if a.isdigit()]
                nums = [float(a) for a in args if self._is_number(a) and not a.isdigit()]
                self.dockto_cmd(bays[0] if bays else None, nums[0] if nums else None)
            elif cmd == "pathgo":
                if self.path is None:
                    self.event("no planned path - path A B first")
                    return
                self.start_path(float(args[0]) if args else None)
            elif cmd == "set":
                key, val = args[0].lower(), float(args[1])
                if key in ("la", "lookahead"):
                    self.path_la = max(0.5, val)
                elif key == "step":
                    self.path_step = max(0.2, val)
                elif key == "corridor":
                    self.path_corridor = max(1.0, val)
                elif key == "endr":
                    self.path_end_r = max(0.3, val)
                elif key in ("fpos", "fneg"):
                    if not 0.5 <= val <= 2.0:
                        raise ValueError("stern factor 0.5..2.0")
                    setattr(self, "sway_" + key, val)
                    self.event("sway mix: %s" % self.swaymix_text())
                    return
                elif key == "swayv":
                    self.sway_v = max(0.05, min(self.args.max_amp, val))
                elif key == "kpsi":
                    self.sway_kpsi = val
                elif key == "kc":
                    self.sway_kc = val
                elif key == "yawsign":
                    self.yaw_sign = 1.0 if val >= 0 else -1.0
                elif key == "swaysign":
                    self.sway_sign = 1.0 if val >= 0 else -1.0
                else:
                    raise ValueError("set la|step|corridor|endr|swayv|kpsi|kc|yawsign|swaysign VALUE")
                self.event("set %s = %g" % (key, val))
            elif cmd == "speed":
                self.change_speed(float(args[0]))
            elif cmd == "co":
                if args:
                    p = self.points.data.get(args[0])
                    if p is None:
                        self.event("unknown point %s" % args[0])
                        return
                    self.origin.set(p["lat"], p["lon"], "point %s" % args[0])
                    self.event("CO set at point %s (%.7f %.7f)" % (args[0], p["lat"], p["lon"]))
                else:
                    t = self.tel
                    if t.lat is None or t.fix < 3:
                        self.event("refused: no 3D fix / position")
                        return
                    self.origin.set(t.lat, t.lon, "boat")
                    self.event("CO set at the boat (%.7f %.7f)" % (t.lat, t.lon))
                self.trail = []
            elif cmd == "param":
                self.write_param(args[0].upper(), float(args[1]))
            elif cmd == "log":
                sub = args[0].lower() if args else ""
                if sub == "on":
                    self.log_start(" ".join(args[1:]) or None)
                elif sub == "off":
                    self.log_stop()
                else:
                    self.event("logging %s" % (("ON -> %s" % self.base) if self.base else "off  (log on [LABEL] | log off)"))
            elif cmd == "note":
                text = " ".join(args)
                self.event("note: %s%s" % (text, "" if self.base else "   (NOT logged: logging is off)"))
            elif cmd == "cv":
                self.cv_cmd(args[0].lower() if args else "status")
            elif cmd in ("handover", "release"):
                if not self.active:
                    self.event("already handed over")
                elif self.mq is None or not self.mq.connected:
                    self.event("refused: no MQTT connection - the mission manager would never see the handover (%s)" % self.mq_line())
                else:
                    if self.action or (self.path and self.path["state"] in ("waiting", "running")) or (self.sway and self.sway["state"] not in ("done", "aborted")):
                        self.stop("release: cancelling what this tool was doing")
                    self.set_active(False, "handover")
            elif cmd in ("takeover", "active", "take"):
                if self.active:
                    self.event("already active")
                else:
                    self.set_active(True, "takeover")
            elif cmd == "fcu":
                self.fcu_cmd(args)
            elif cmd in ("reset", "step", "skip"):         # v0.1.68: skip = free run only (the mission manager refuses otherwise)
                self.mm_cmd(cmd.upper())
            elif cmd == "share":
                self.event(self.share_line())
                self.event(self.mq_line() + "   " + self.mm_line())
            elif cmd == "map":
                self.map_cmd()
            elif cmd == "help":
                self.event("DRIVE mode arm disarm surge sway yaw motor burst stop|x · NAVIGATE save here points del co goto speed path pathto pathgo set · "
                           "SWAY swayto swaymix swaycal · STACK handover takeover reset step cv fcu share param map · RECORD log note help quit  (HELP tab: every modifier)")
            elif cmd in ("quit", "q", "exit"):
                if self.active:
                    self.stop("quit")
                else:
                    self.event("quit: handed over - the FCU stays with the mission manager")
                self.quit = True
            else:
                self.event("unknown command: %s (help)" % cmd)
        except (IndexError, ValueError, KeyError) as e:
            self.event("bad arguments for %s: %s" % (cmd, e))

    def write_param(self, name, value):
        self.t_last_mav_cmd = time.monotonic()
        if self.tel.armed:
            self.event("refused: disarm before writing parameters")
            return
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), value,
                                  MAV.MAV_PARAM_TYPE_REAL32)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            p = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
            if p is None:
                continue
            pid = p.param_id.rstrip("\x00") if isinstance(p.param_id, str) else p.param_id.rstrip(b"\0").decode()
            if pid == name:
                self.event("param %s = %g (wanted %g)" % (name, p.param_value, value))
                return
        self.event("param %s: no PARAM_VALUE echo (unknown name?)" % name)

    # ---------------- UI ----------------
    SCREENS = ["MAIN", "MISSION", "MAP", "PATH", "POINTS", "LOG", "MAVLINK", "HELP"]
    SEVERITY = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERROR", 4: "WARN", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}
    USAGE = [
        ("surge", "surge A [T]      A -1..1, T s         e.g.  surge 0.3 2      (+ ahead, - astern)"),
        ("sway", "sway A [T]                             e.g.  sway 0.3 2       (+ hull to starboard, - to port)"),
        ("yaw", "yaw A [T]                              e.g.  yaw 0.3 2        (+ bow to starboard / clockwise)"),
        ("motor", "motor N A [T]    N 1 PORT 2 STERN 3 STBD 4 BOW   e.g.  motor 3 0.2 1"),
        ("burst", "burst m1 m2 m3 m4 [T]   raw four channels   e.g.  burst 0.2 0 0.2 0 2"),
        ("mode", "mode 20 | cman | hold | guided | manual | <number>      e.g.  mode 20"),
        ("arm", "arm              arm the vehicle (bench board has ARMING_SKIPCHK -1: no pre-arm checks)"),
        ("disarm", "disarm           cancels any burst, then disarms"),
        ("save", "save NAME        store the current position (needs 3D fix; overwrites)   e.g.  save A"),
        ("goto", "goto NAME [V]    GUIDED to a saved point, V m/s optional   e.g.  goto A 0.8"),
        ("speed", "speed V          change the GUIDED speed now   e.g.  speed 0.5"),
        ("path", "path A B [C..] [straight]   plan boat->A->B.. (spline unless 'straight'); see the PATH tab"),
        ("pathto", "pathto A B [C..] [V]   plan and follow the path, V m/s optional   e.g.  pathto A B 0.8"),
        ("pathgo", "pathgo [V]       follow the already planned path"),
        ("swayto", "swayto A B [V]   GUIDED to A, yaw starboard-to-B, sway along A->B (mode 20)   e.g.  swayto A B 0.25"),
        ("pathgen", "pathgen [D]      path from the cv map: entry -> each gate (D m before / mid / after) -> exit; saved as EN G1a G1 G1b .. EX; no motion (pathgo V)"),
        ("dockto", "dockto [BAY] [V]  sway abeam the bay (GREEN one by default; bays 1..3 left to right facing the dock) then bow in (mode 20); FAKE FCU ONLY"),
        ("set", "set la|step|corridor|endr VALUE   lookahead m / path step m / abort corridor m / end radius m"),
        ("co", "co               control origin at the boat   |   co NAME   at a saved point"),
        ("del", "del NAME         remove a saved point"),
        ("points", "points           list saved points (also the POINTS screen)"),
        ("here", "here             print the current position"),
        ("param", "param NAME VALUE write one FCU parameter, disarmed   e.g.  param WP_SPEED 0.8"),
        ("log", "log on [LABEL] | log off | log     start / stop recording .tlog/.csv/.events   e.g.  log on sway03"),
        ("swaycal", "swaycal A T [F]  mode 20 + arm + log on + sway (M2 = F*A, M4 = A; F default = sway mix) + log off   e.g.  swaycal -0.3 20"),
        ("swaymix", "swaymix on | off   calibrated stern factor (+1.05 / -0.877) or firmware 0.957 for sway / swayto   (set fpos|fneg X)"),
        ("sway_calibr", "sway_calibr A T [F]   same as swaycal   e.g.  sway_calibr 0.3 20 0.90"),
        ("note", "note TEXT        put a note in the event log   e.g.  note path la 3 spline 0.8 m/s, calm water"),
        ("cv", "cv on | off | status | clear   AGX detections (local/usv/perception/#) -> map; MAP tab shows R/G/D marks"),
        ("handover", "handover         hand the FCU to the mission manager (tui/active beat stops; FCU commands here refused; you stay here)"),
        ("takeover", "takeover         take the FCU back from the mission manager (stop / x / disarm also do)"),
        ("release", "release          = handover"),
        ("active", "active           = takeover"),
        ("share", "share            link share and MQTT status: relayed counts, last tool, mission manager state"),
        ("map", "map              the browser map over this broker's websocket (opens here on a desktop, else prints the URL for the Mac)"),
        ("fcu", "fcu fake [port] | real [usb|eth|<endpoint>] | status   switch the FCU link live (fake plant on udp 14550 <-> the Pixhawk); disarmed, ACTIVE, idle"),
        ("stop", "stop | x         cancel a burst (0.5 s zeros); GUIDED / AUTO -> HOLD; takes the FCU back when handed over"),
        ("reset", "reset            tell the mission manager to reset: task aborted, auto off, run forgotten -> MANUAL (KILLED / FAULT stay)"),
        ("step", "step             advance the mission manager's m0t0 (TEST_MANUAL) task one stage; m0t1: skip its timer"),
        ("skip", "skip             v0.1.68 free run only: the mission manager ends the current stage's behaviour now (the next stage)"),
        ("help", "help             command list (also the HELP screen)"),
        ("quit", "quit | q         stop, GUIDED -> HOLD, then exit"),
    ]

    EXAMPLES = {
        "surge": "surge 0.2 1", "sway": "sway 0.2 1", "yaw": "yaw 0.2 1", "motor": "motor 1 0.2 1", "burst": "burst 0.2 0 0.2 0 1",
        "mode": "mode 20", "arm": "arm", "disarm": "disarm", "save": "save A", "goto": "goto A 0.8", "speed": "speed 0.5",
        "path": "path A B C", "pathto": "pathto A B 0.6", "pathgo": "pathgo 0.6", "swayto": "swayto A B 0.25", "pathgen": "pathgen", "dockto": "dockto", "set": "set la 3",
        "co": "co A", "del": "del A", "points": "points", "here": "here", "param": "param WP_SPEED 0.8", "log": "log on legA",
        "swaycal": "swaycal 0.3 20", "swaymix": "swaymix on", "note": "note calm water, wind from the east",
        "cv": "cv on", "handover": "handover", "takeover": "takeover", "share": "share", "map": "map", "fcu": "fcu fake",
        "stop": "stop", "reset": "reset", "step": "step", "skip": "skip", "help": "help", "quit": "quit",
    }

    def example_for(self, line):
        word = line.strip().split(" ")[0].lower() if line.strip() else ""
        if not word:
            return None
        if word in self.EXAMPLES:
            return self.EXAMPLES[word]
        pref = [k for k, _ in self.USAGE if k.startswith(word)]
        return self.EXAMPLES.get(pref[0]) if len(pref) == 1 else None

    def usage_for(self, line):
        word = line.strip().split(" ")[0].lower() if line.strip() else ""
        if not word:
            return None
        exact = [u for k, u in self.USAGE if k == word]
        if exact:
            return exact[0]
        pref = [k for k, _ in self.USAGE if k.startswith(word)]
        if len(pref) == 1:
            return [u for k, u in self.USAGE if k == pref[0]][0]
        if pref:
            return "commands: " + "  ".join(pref)
        return "unknown command - see COMMANDS panel or the HELP screen"

    def init_colors(self):
        self.C = {}
        keys = ("ok", "bad", "warn", "title", "dim", "armed", "disarmed", "burst", "bar", "co", "point", "target",
                "vehicle", "trail", "leg", "path", "ent_red", "ent_green", "ent", "ent_stale", "gate", "banner")
        if not curses.has_colors():
            for k in keys:
                self.C[k] = 0
            self.C["armed"] = self.C["burst"] = self.C["banner"] = curses.A_REVERSE
            self.C["vehicle"] = curses.A_BOLD
            return
        curses.start_color()
        try:
            curses.use_default_colors()
            bg = -1
        except curses.error:
            bg = curses.COLOR_BLACK
        pairs = [(1, curses.COLOR_GREEN, bg), (2, curses.COLOR_RED, bg), (3, curses.COLOR_YELLOW, bg),
                 (4, curses.COLOR_CYAN, bg), (5, curses.COLOR_WHITE, curses.COLOR_RED),
                 (6, curses.COLOR_BLACK, curses.COLOR_GREEN), (7, curses.COLOR_MAGENTA, bg),
                 (8, curses.COLOR_BLACK, curses.COLOR_YELLOW), (9, curses.COLOR_BLUE, bg)]
        for n, f, b in pairs:
            curses.init_pair(n, f, b)
        cp = curses.color_pair
        self.C = {"ok": cp(1) | curses.A_BOLD, "bad": cp(2) | curses.A_BOLD, "warn": cp(3) | curses.A_BOLD,
                  "title": cp(4) | curses.A_BOLD, "dim": curses.A_DIM, "armed": cp(5) | curses.A_BOLD,
                  "disarmed": cp(6), "burst": cp(8) | curses.A_BOLD, "bar": cp(7) | curses.A_BOLD,
                  "co": cp(3) | curses.A_BOLD, "point": cp(9) | curses.A_BOLD, "target": cp(4) | curses.A_BOLD,
                  "vehicle": cp(1) | curses.A_BOLD, "trail": cp(1), "leg": cp(4), "path": cp(4) | curses.A_BOLD,
                  "ent_red": cp(2) | curses.A_BOLD, "ent_green": cp(1) | curses.A_BOLD, "ent": curses.A_BOLD,
                  "ent_stale": curses.A_DIM, "gate": cp(7) | curses.A_BOLD, "banner": cp(8) | curses.A_BOLD}

    def put(self, y, x, s, attr=0):
        h, w = self.scr.getmaxyx()
        if 0 <= y < h and x < w - 1:
            try:
                self.scr.addnstr(y, x, s, w - x - 1, attr)
            except curses.error:
                pass

    def box(self, y, x, h, w, title):
        """Bordered panel with a title; clipped to the screen."""
        H, W = self.scr.getmaxyx()
        if y >= H - 1 or x >= W - 1:
            return
        w = min(w, W - x - 1)
        h = min(h, H - y - 1)
        if h < 2 or w < 4:
            return
        s = self.scr
        try:
            s.attron(self.C["dim"])
            s.hline(y, x + 1, curses.ACS_HLINE, w - 2)
            s.hline(y + h - 1, x + 1, curses.ACS_HLINE, w - 2)
            s.vline(y + 1, x, curses.ACS_VLINE, h - 2)
            s.vline(y + 1, x + w - 1, curses.ACS_VLINE, h - 2)
            s.addch(y, x, curses.ACS_ULCORNER)
            s.addch(y, x + w - 1, curses.ACS_URCORNER)
            s.addch(y + h - 1, x, curses.ACS_LLCORNER)
            s.addch(y + h - 1, x + w - 1, curses.ACS_LRCORNER)
            s.attroff(self.C["dim"])
        except curses.error:
            pass
        self.put(y, x + 2, " %s " % title, self.C["title"])

    def kv(self, y, x, key, val, attr=0, kw=6):
        self.put(y, x, ("%-" + str(kw) + "s") % key, self.C["dim"])
        self.put(y, x + kw, val, attr)

    def local(self):
        """Vehicle local (x north, y east) or None."""
        t = self.tel
        if t.lat is None or not self.origin.is_set:
            return None
        return self.origin.to_local(t.lat, t.lon)

    def next_hint(self):
        t, a = self.tel, self.action
        if t.hb_t is None:
            return "waiting for the FCU heartbeat"
        if not self.active:
            mm = self.mm[1] if self.mm else None
            v = (mm or {}).get("state")
            where = {"MANUAL": "mission manager waits in MANUAL: on the Mac ocs_cli ready (enable_auto)",
                     "AUTO_HOLD": "mission manager AUTO_HOLD: on the Mac ocs_cli goto / start_task T4",
                     "AUTO_TASK": "mission manager runs %s %s: watch; ocs_cli hold / abort" % ((mm or {}).get("task_name") or "", (mm or {}).get("task_state") or ""),
                     "AUTO_SAFE_HOLD": "mission manager in SAFE_HOLD: ocs_cli resume / abort"}.get(v, "mission manager %s" % (v or "not seen"))
            return "HANDED OVER - %s - 'takeover' (or stop / x) takes the FCU back here" % where
        if a:
            return "running %s - stop / x cancels" % a["label"]
        if self.path and self.path["state"] in ("waiting", "running"):
            return "path running (%s) - stop -> HOLD; PATH tab shows progress" % "-".join(self.path["anchors"])
        if self.sway and self.sway["state"] not in ("done", "aborted"):
            return "sway %s %s->%s - stop cancels; PATH tab shows c / hdg err" % (self.sway["state"], self.sway["a_name"], self.sway["b_name"])
        if self.goto:
            return "goto %s active - stop -> HOLD" % self.goto["name"]
        if not t.armed:
            return "arm   (mode 20 first for bursts)"
        if t.mode == 20:
            return "ready: surge 0.3 2 | sway 0.3 2 | yaw 0.3 2 | motor 1 0.2 1 | save A"
        if t.mode == 15:
            return "GUIDED: goto NAME [V] | stop"
        return "mode 20 for bursts, goto NAME for GUIDED"

    def draw(self):
        scr = self.scr
        if scr is None:
            return
        scr.erase()
        h, w = scr.getmaxyx()
        now = time.monotonic()
        tabs = "  ".join(("[%s]" if i == self.screen else " %s ") % n for i, n in enumerate(self.SCREENS))
        rec = " REC %s " % os.path.basename(self.base) if self.base else " log off "
        own = " ACTIVE: usv_ui commands the FCU " if self.active else " HANDED OVER: mission manager commands the FCU (takeover / stop take it back) "
        nw = len(self.warnings(now)) if (h >= self.MIN_H and w >= self.MIN_W) else 0
        flag = (" %d WARNING%s " % (nw, "S" if nw != 1 else "")) if nw else ""
        preset = os.environ.get("LAUNCH_PRESET") or "?"           # v0.1.42 (WP9d): the flavour - which preset started this stack
        bar = " usv_ui v%s [%s]  %s %s %s  %s %s Tab: %s " % (KIT_VERSION, preset, self.args.conn, own, time.strftime("%H:%M:%S"), rec, flag, tabs)
        fr = self.free_run_banner()                         # v0.1.68 (WP17 F6): the mission manager runs on the ghost
        if fr:
            bar = fr + bar
        self.put(0, 0, bar.ljust(w - 1), curses.A_REVERSE)
        if fr:
            self.put(0, 0, fr, self.C["bad"] | curses.A_BOLD)
        self.put(0, bar.index(own), own, self.C["ok"] | curses.A_REVERSE if self.active else self.C["bad"] | curses.A_BOLD)
        if self.base:
            self.put(0, bar.index(rec), rec, self.C["bad"] | curses.A_BOLD)
        if flag:
            self.put(0, bar.index(flag), flag, self.C["armed"])
        if h < 30 or w < 90:
            self.put(2, 0, "terminal too small: need 90x30 (compact) or 120x44 (the full MAIN), have %dx%d" % (w, h), self.C["bad"])
            self.draw_prompt()
            scr.refresh()
            return
        [self.draw_main, self.draw_mission_tab, self.draw_map, self.draw_path, self.draw_points, self.draw_log, self.draw_mavlink,
         self.draw_help][self.screen](now)
        self.draw_prompt()
        scr.refresh()

    # ======== MAIN v0.1.18: the shared skeleton (the same rows as op-tui on the Mac) ========
    MIN_W, MIN_H = 120, 44               # below this the compact MAIN (v0.1.17 layout, 90x30) is drawn

    def geometry(self, w, h):
        sp1, sp2, sp3 = int(h >= 46), int(h >= 52), int(h >= 54)
        g = {"y1": 2, "r1h": 9, "r2h": 8, "fh": 7, "r4h": 7}
        g["y2"] = g["y1"] + g["r1h"] + sp1
        g["yf"] = g["y2"] + g["r2h"]
        g["y4"] = g["yf"] + g["fh"] + sp2
        g["y5"] = g["y4"] + g["r4h"] + sp3
        g["bottom"] = h - 4 - g["y5"]
        g["host_w"] = 50
        g["where_w"] = 52 if w >= 140 else 44
        g["a_w"] = int(w * 0.37)
        g["net_w"] = int(w * 0.35)
        g["ev_w"] = int(w * 0.29)
        g["con_w"] = int(w * 0.29)
        return g

    def ocs_body(self):
        """The OCS state as bridged into the boat broker (team/.../ocs/state in 0), or None when not seen for 5 s."""
        if self.ocs is None:
            return None
        t, b = self.ocs
        return b if time.monotonic() - t < 5.0 else None

    def mm_body(self):
        if self.mm is None:
            return None
        t, b = self.mm
        return b if time.monotonic() - t < 3.0 else None

    def flow_state(self):
        """What playbook.flow_stage needs: the OCS body when bridged in, else a stand-in built from mm/state."""
        ocs = self.ocs_body()
        if ocs:
            return ocs
        mm = self.mm_body() or {}
        return {"r": None, "vehicles": {self.args.vehicle_id: {"state": mm.get("state"), "task_state": mm.get("task_state"), "task_name": mm.get("task_name")}}}

    def warnings(self, now):
        """[(age_s, text, attr)] - the active conditions; each clears by itself. Oldest first."""
        t = self.tel
        conds = []
        if t.hb_t is None or now - t.hb_t > 3.0:
            conds.append(("hb", "FCU: no heartbeat" + (" for %.0f s" % (now - t.hb_t) if t.hb_t else "") + " (link %s)" % self.args.conn, "bad", None))
        if self.mq is not None and not self.mq.connected:
            conds.append(("mqtt", "boat broker not connected (%s)" % self.mq_line(), "bad", None))
        if self.bridge is False:
            conds.append(("ocs", "OCS LINK DOWN: the boat broker lost the hub (bridge); the mission manager continues (%s)" % self.link_loss_text(), "bad", None))
        elif self.ocs is not None and now - self.ocs[0] > 5.0 and self.bridge:
            conds.append(("ocsstate", "ocs/state silent %.0f s over the bridge (ocs_app down?)" % (now - self.ocs[0]), "warn", None))
        if self.mm is not None and now - self.mm[0] > 3.0:
            conds.append(("mm", "mission manager SILENT %.0f s (was %s)" % (now - self.mm[0], self.mm[1].get("state")), "bad", None))
        mm = self.mm_body() or {}
        if mm.get("state") in ("KILLED", "FAULT"):
            conds.append(("mmstate", "vehicle %s%s: clear the input, then confirm_safe (OCS)" % (mm.get("state"), (" " + mm["fault"]) if mm.get("fault") else ""), "bad", None))
        rc = t.rc or []
        if rc and len(rc) > 4 and rc[4] > 1750:
            conds.append(("rc5", "RC5 (SF) motor E-stop asserted on the transmitter", "bad", None))
        if rc and len(rc) > 5 and rc[5] > 1750:
            conds.append(("rc6", "RC6 MANUAL override high: the mission manager drops to MANUAL", "warn", None))
        if t.hb_t and t.armed and t.fix < 3:
            conds.append(("fix", "armed without a GPS fix: GUIDED / goto will be refused", "warn", None))
        if self.last_refused and now - self.last_refused[0] < 10.0:
            conds.append(("refused", self.last_refused[1], "warn", self.last_refused[0]))
        keys, out = set(), []
        for key, text, attr, t0 in conds:
            keys.add(key)
            since = t0 if t0 is not None else self.cond_since.setdefault(key, now)
            out.append((max(0.0, now - since), text, attr))
        for k in [k for k in self.cond_since if k not in keys]:
            del self.cond_since[k]
        out.sort(key=lambda it: -it[0])
        return out

    def link_loss_text(self):
        mm = self.mm_body() or {}
        pol = mm.get("link_loss_action")
        return ("policy %s" % pol) if pol else "policy continue"

    def draw_banner(self):
        C = self.C
        h, w = self.scr.getmaxyx()
        txt = " BOAT CONSOLE  ·  usv_ui on the Orin NX  ·  holds the FCU link; bursts, goto, paths, map; handover to the mission manager  ·  the shore side is op-tui "
        if not self.unicode:
            txt = txt.replace("·", "-")
        self.put(1, 0, txt.ljust(w - 1)[: w - 1], C["banner"])

    def draw_main(self, now):
        h, w = self.scr.getmaxyx()
        if h < self.MIN_H or w < self.MIN_W:
            self.draw_main_compact(now)
            return
        g = self.geometry(w, h)
        self.draw_banner()
        y, lw = g["y1"], g["host_w"]
        self.draw_action_links(y, 0, g["r1h"], lw, now)
        self.draw_vehicle_group(y, lw + 2, g["r1h"], w - lw - 3, now)
        y = g["y2"]
        mw = w - g["where_w"] - 2
        self.draw_mission_box(y, 0, g["r2h"], mw, now)
        self.draw_where_box(y, mw + 1, g["r2h"], w - mw - 2, now)
        self.draw_flow_box(g["yf"], 0, g["fh"], w - 1, now)
        y = g["y4"]
        aw, nw = g["a_w"], g["net_w"]
        self.draw_thrusters_rc(y, 0, g["r4h"], aw)
        self.draw_network_box(y, aw + 1, g["r4h"], nw, now)
        self.draw_warnings_box(y, aw + nw + 2, g["r4h"], w - aw - nw - 3, now)
        y = g["y5"]
        self.draw_events_console(y, g["bottom"], g["ev_w"], g["con_w"], now)
        self.draw_commands_box(y, g["ev_w"] + g["con_w"] + 2, g["bottom"], w - g["ev_w"] - g["con_w"] - 3, now)

    def draw_action_links(self, y, x, h, w, now):
        C, t = self.C, self.tel
        self.box(y, x, h, w, "ACTION  /  LINKS  (this console)")
        a = self.action
        if a:
            self.kv(y + 1, x + 2, "action", (" %s " % a["label"]), C["burst"] if a["phase"] == "burst" else C["warn"], 8)
            self.put(y + 1, x + 12 + len(a["label"]), ("%s %.1f s" % (a["phase"].upper(), max(0.0, a["t_phase_end"] - now)))[: w - 16 - len(a["label"])], curses.A_BOLD)
        else:
            own = ("path %s %s" % ("-".join(self.path["anchors"]), self.path["state"]) if self.path and self.path["state"] in ("waiting", "running") else
                   "sway %s %s->%s" % (self.sway["state"], self.sway["a_name"], self.sway["b_name"]) if self.sway and self.sway["state"] not in ("done", "aborted") else
                   "goto %s" % self.goto["name"] if self.goto else ("idle" if self.active else "watching"))
            self.kv(y + 1, x + 2, "action", own[: w - 12], curses.A_BOLD if own not in ("idle", "watching") else C["dim"], 8)
            if own in ("idle", "watching"):
                self.put(y + 1, x + 10 + len(own) + 2, "(bursts need mode 20 + armed)"[: w - 14 - len(own)], C["dim"])
        who = "ACTIVE: this console commands the FCU" if self.active else "HANDED OVER -> mission manager"
        self.kv(y + 2, x + 2, "who", who[: w - 12], C["ok"] if self.active else C["warn"], 8)
        hb = ("HB %.1f s" % (now - t.hb_t)) if t.hb_t else "NO HEARTBEAT"
        self.kv(y + 3, x + 2, "FCU", ("%s plant udp %d" % ("fake", self.fake_port()) if self.fcu_kind() == "fake" else "real Pixhawk %s" % self.args.conn)[: w - 24] + "  " + hb,
                C["warn"] if self.fcu_kind() == "fake" else (C["ok"] if (t.hb_t and now - t.hb_t < 3) else C["bad"]), 8)
        if self.share is not None:
            s = self.share
            age = (now - s.t_in) if s.t_in else None
            self.kv(y + 4, x + 2, "share", ("%d/%d   mission manager %s" % (s.n_out, s.n_in, ("%.0f s ago" % age) if age is not None else "not seen"))[: w - 12],
                    C["ok"] if (age is not None and age < 3) else C["dim"], 8)
        else:
            self.kv(y + 4, x + 2, "share", ("off" if not self.share_err else "FAILED: %s" % self.share_err)[: w - 12], C["bad"] if self.share_err else C["dim"], 8)
        mm_age = (now - self.mm[0]) if self.mm else None
        self.kv(y + 5, x + 2, "boat", ("%s   mm/state %s" % (self.mq_line(), ("%.1f s" % mm_age) if mm_age is not None else "not seen"))[: w - 12],
                C["ok"] if (self.mq is not None and self.mq.connected and mm_age is not None and mm_age < 3) else (C["bad"] if self.mq is not None else C["dim"]), 8)
        self.kv(y + 6, x + 2, "OCS", self.ocs_link_text(now)[: w - 12], self.ocs_link_attr(), 8)
        self.kv(y + 7, x + 2, "rec", ("log %s   cv %s   sway mix %s" % (("ON " + os.path.basename(self.base)) if self.base else "off", "on" if self.cv_on else "off", "CAL" if self.swaymix else "fw"))[: w - 12],
                C["bad"] if self.base else C["dim"], 8)

    def ocs_link_text(self, now):
        if self.mq is None or not self.mq.connected:
            return "unknown (no boat broker)"
        if self.bridge is False:
            return "LINK DOWN %.0f s (bridge)" % (now - self.bridge_t) + (", state %.0f s" % (now - self.ocs[0]) if self.ocs else "")
        if self.bridge is None:
            return "bridge state not seen ($SYS)" + (", ocs/state %.0f s" % (now - self.ocs[0]) if self.ocs else "")
        if self.ocs is None:
            return "bridge up, no ocs/state yet (OCS up? bridged in?)"
        return "link up, ocs/state %.1f s ago  %s" % (now - self.ocs[0], (self.ocs[1].get("r") or ""))

    def ocs_link_attr(self):
        C = self.C
        if self.bridge is False:
            return C["bad"]
        if self.bridge and self.ocs is not None and time.monotonic() - self.ocs[0] < 5:
            return C["ok"]
        return C["dim"] if self.bridge is None else C["warn"]

    def draw_vehicle_group(self, y, gx, h, w, now):
        """VEHICLE: the mission manager's state (ladder, meaning, stage) with this console's FCU view | position / FCU."""
        C, t = self.C, self.tel
        self.box(y, gx, h, w, "VEHICLE  %s" % self.args.vehicle_id)
        split = min(52, max(40, w // 2 + 4))
        s = self.scr
        try:
            s.attron(C["dim"])
            s.hline(y + 1, gx + 1, curses.ACS_HLINE, w - 2)
            s.addch(y + 1, gx, curses.ACS_LTEE)
            s.addch(y + 1, gx + w - 1, curses.ACS_RTEE)
            s.vline(y + 2, gx + split, curses.ACS_VLINE, h - 3)
            s.addch(y + 1, gx + split, curses.ACS_TTEE)
            s.addch(y + h - 1, gx + split, curses.ACS_BTEE)
            s.attroff(C["dim"])
        except curses.error:
            pass
        self.put(y + 1, gx + 2, " state ", C["title"])
        self.put(y + 1, gx + split + 2, " position / FCU ", C["title"])
        x, lwid = gx + 2, split - 3
        mm = self.mm_body() or {}
        v = mm.get("state")
        codes = ["V0", "V1", "V2", "V3", "V4", "V5", "V6"]
        names = ["BOOT", "KILLED", "MANUAL", "AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD", "FAULT"]
        cx = x
        for code, name in zip(codes, names):
            if name == v:
                self.put(y + 2, cx, "[%s]" % code, C["disarmed"] | curses.A_BOLD)
            else:
                self.put(y + 2, cx, " %s " % code, C["dim"])
            cx += len(code) + 3
        mm_age = (now - self.mm[0]) if self.mm else None
        self.kv(y + 3, x, "mm", ("%s %s  (mission manager)" % (codes[names.index(v)] if v in names else "--", v or "not seen"))[: lwid - 16],
                C["bad"] if v in ("KILLED", "FAULT") else (C["warn"] if (v or "").startswith("AUTO") else (C["ok"] if v else C["dim"])), 7)
        self.put(y + 3, x + lwid - 9, ("%.1f s" % mm_age if mm_age is not None else "-")[:9], C["ok"] if (mm_age is not None and mm_age < 3) else C["bad"])
        self.put(y + 4, x, " ARMED " if t.armed else " disarmed ", C["armed"] if t.armed else C["disarmed"])
        self.put(y + 4, x + 11, ("mode %s (%s)" % (MODE_NAMES.get(t.mode, "?"), t.mode))[:18], C["warn"])
        self.put(y + 4, x + 30, ("batt %s" % ("%.1f V" % t.batt if t.batt else "--"))[: lwid - 30], 0)
        ekf = t.ekf_pos_ok()
        rc = t.rc
        self.kv(y + 5, x, "EKF", ("posH %s 0x%03x" % ("OK" if ekf else "NO POS", t.ekf_flags))[:18], C["ok"] if ekf else C["bad"], 7)
        self.put(y + 5, x + 25, ("rc %s  RC5 %d  RC6 %d" % ("ok" if rc else "none", rc[4] if rc else 0, rc[5] if rc else 0))[: lwid - 25],
                 C["bad"] if (rc and (rc[4] > 1750 or rc[5] > 1750)) else (0 if rc else C["dim"]))
        self.kv(y + 6, x, "means", ((V_MEANING.get(v, "") or "").split(" ", 1)[-1] if v else "no mm/state: the mission manager is not up (run_usv.sh starts it)")[: lwid - 7], C["dim"], 7)
        tn, ts = mm.get("task_name"), mm.get("task_state")
        stage = task_stage_meaning(tn, ts) if tn else ""
        self.kv(y + 7, x, "stage", (("%s %s: %s" % (tn, ts, stage)) if stage else (("%s %s" % (tn, ts or "")) if tn else "no task"))[: lwid - 7], C["warn"] if tn else C["dim"], 7)
        px = gx + split + 2
        rwid = gx + w - 1 - px
        self.kv(y + 2, px, "lat", "%.7f" % t.lat if t.lat is not None else "--", curses.A_BOLD, 5)
        self.put(y + 2, px + 18, ("lon %.7f" % t.lon if t.lon is not None else "lon --")[: rwid - 18], curses.A_BOLD)
        loc = self.local()
        self.kv(y + 3, px, "hdg", ("%5.1f deg" % t.hdg) if t.hdg is not None else "--", 0, 5)
        self.put(y + 3, px + 16, "gs %.2f m/s" % t.gs, 0)
        self.put(y + 3, px + 30, (("CO N %+.1f E %+.1f" % loc) if loc else ("CO not set" if not self.origin.is_set else "CO --"))[: rwid - 30], C["co"] if loc else C["dim"])
        fix = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fix"}.get(t.fix, str(t.fix))
        self.kv(y + 4, px, "GPS", fix, C["ok"] if t.fix >= 3 else C["bad"], 5)
        self.put(y + 4, px + 16, "sats %d" % t.sats, C["ok"] if t.sats >= 8 else (C["warn"] if t.sats >= 5 else C["bad"]))
        self.put(y + 4, px + 25, "hdop %.1f" % t.hdop, C["ok"] if t.hdop < 1.5 else C["warn"])
        self.put(y + 4, px + 35, ("v %+.2f %+.2f" % (t.vn, t.ve))[: rwid - 35], 0)
        g = self.goto
        gl = ("goto %s %.1f m / %03.0f" % (g["name"], haversine(t.lat, t.lon, g["lat"], g["lon"]), bearing(t.lat, t.lon, g["lat"], g["lon"]))) if (g and t.lat is not None) else "goto -"
        pl = ("path %s %s" % ("-".join(self.path["anchors"]), self.path["state"])) if self.path else "path -"
        sl = ("sway %s" % self.sway["state"]) if self.sway else "sway -"
        self.put(y + 5, px, ("%s   %s   %s" % (gl, pl, sl))[: rwid], C["warn"] if (g or (self.path and self.path["state"] in ("waiting", "running"))) else C["dim"])
        self.kv(y + 6, px, "FCU", ("FAKE plant udp %d (held here)" % self.fake_port() if self.fcu_kind() == "fake" else "REAL Pixhawk %s (held here)" % self.args.conn)[: rwid - 5],
                C["warn"] if self.fcu_kind() == "fake" else C["ok"], 5)
        prof = mm.get("profile")
        self.kv(y + 7, px, "mm", ("%s%s" % (prof, " - LAB GUARDS" if mm.get("lab") else " (full guards)") if prof else "profile unknown (older mission manager)")[: rwid - 5],
                C["warn"] if mm.get("lab") else C["dim"], 5)

    def draw_mission_box(self, y, x, h, w, now):
        C = self.C
        self.box(y, x, h, w, "MISSION  (as the OCS holds it; stages from the mission manager; MISSION tab for every mission)")
        ocs = self.ocs_body()
        mm = self.mm_body() or {}
        mi = (ocs or {}).get("mission") or {}
        task = mi.get("task")
        tn, ts, v = mm.get("task_name"), mm.get("task_state"), mm.get("state")
        run = (mm.get("run") or {}).get("run_id") or (ocs or {}).get("run_id")
        mw = w - 4
        if tn and v in ("AUTO_TASK", "AUTO_SAFE_HOLD"):
            shown = tn
            state = "RUNNING  run %s  stage %s%s" % (run or "-", ts, "  (paused)" if v == "AUTO_SAFE_HOLD" else "")
            attr = C["ok"]
            hold = str(mm.get("hold_reason") or "")
            if hold.startswith("step: next "):       # v0.1.26 stage stepping
                state, attr = "STEP  run %s  next %s  (ocs_cli resume enters it)" % (run or "-", hold[len("step: next "):]), C["warn"]
        elif task:
            shown = task
            if mi.get("armed_run") and mi.get("armed_run") == (ocs or {}).get("run_id"):
                state, attr = "ARMED by the OCS: starts on the next AUTO_HOLD (run %s)" % run, C["warn"]
            elif mi.get("started_run") and mi.get("started_run") == (ocs or {}).get("run_id"):
                state, attr = "STARTED by the OCS in run %s (task over: vehicle %s)" % (run, v), C["ok"]
            elif mi.get("auto"):
                state, attr = "preselected on the OCS, auto-start on the judge's RunStart", C["warn"]
            else:
                state, attr = "preselected on the OCS: start_task (there) runs it when AUTO_HOLD in R5", C["warn"]
        else:
            shown = ""
            state = ("none selected on the OCS (ocs_cli mission <code>)" if ocs else "OCS view not received (ocs/state over the bridge); mission manager: %s" % ("task %s %s" % (tn, ts) if tn else "no task"))
            attr = C["dim"]
        tier = mi.get("tier") if (shown and mi.get("task") == shown) else None
        code = (mi.get("code") if (shown and mi.get("task") == shown and mi.get("code")) else code_of(shown, tier)) if shown else ""
        self.kv(y + 1, x + 2, "selected", (("%s  (%s)   auto-start on RunStart: %s" % (code, shown, "yes" if mi.get("auto") else "no")) if shown else "none")[: mw - 10], C["warn"] if shown else C["dim"], 10)
        self.kv(y + 2, x + 2, "state", state[: mw - 10], attr, 10)
        m = mission_for(shown, tier) if MISSIONS else None
        if m:
            self.kv(y + 3, x + 2, "what", ("%s: %s" % (m.title, m.what[0]))[: mw - 10], C["dim"], 10)
            self.put(y + 4, x + 12, (m.what[1] if len(m.what) > 1 else "")[: mw - 10], C["dim"])
        else:
            self.kv(y + 3, x + 2, "what", "m4t0 (T4 incident, built) | m0t0 / m0t1 (state-machine tests) | goto (bench) | m1t1 m1t2 m3t1 m0t2 m0t3 planned"[: mw - 10], C["dim"], 10)
            self.put(y + 4, x + 12, "Tab -> MISSION: each mission with its stages and how it is selected (on the OCS); +sim = fake FCU only"[: mw - 10], C["dim"])
        names = mission_stage_names(shown) if shown else []
        self.put(y + 5, x + 2, "stages", C["dim"])
        cx = x + 12
        cur = ts if (tn == shown and v in ("AUTO_TASK", "AUTO_SAFE_HOLD")) else None
        passed = True
        for i, nme in enumerate(names):
            if cx + len(nme) + 6 > x + w - 2:
                break
            if nme == cur:
                self.put(y + 5, cx, " %s " % nme, C["disarmed"] | curses.A_BOLD); passed = False
            else:
                self.put(y + 5, cx, " %s " % nme, C["ok"] if (cur and passed) else C["dim"])
            cx += len(nme) + 2
            if i < len(names) - 1:
                self.put(y + 5, cx, " > ", C["dim"]); cx += 3
        if not names:
            self.put(y + 5, cx, "-", C["dim"])
        here = "step (m0t0) | reset = abort, auto off, MANUAL | takeover / stop take the FCU back" if not self.active else "handover first: then the OCS can enable_auto and start it"
        self.kv(y + 6, x + 2, "from here", here[: mw - 10], C["dim"], 10)

    def draw_where_box(self, y, x, h, w, now):
        C, t = self.C, self.tel
        self.box(y, x, h, w, "WHERE WE ARE  (flavour; help)")
        xx, mw = x + 2, w - 4
        mm = self.mm_body() or {}
        ocs = self.ocs_body()
        v, tn, ts = mm.get("state"), mm.get("task_name"), mm.get("task_state")
        if t.hb_t is None:
            l1, l2 = "no FCU heartbeat yet on %s" % self.args.conn, "fcu fake / fcu real switch the link; check the Pixhawk"
        elif self.active:
            l1 = "this console commands the FCU (ACTIVE)"
            l2 = self.next_hint()
        elif tn and v in ("AUTO_TASK", "AUTO_SAFE_HOLD"):
            l1, l2 = "handed over: the mission manager drives", "%s (%s); you watch" % (tn, ts)
        else:
            l1, l2 = "handed over: mission manager %s" % (v or "not seen"), self.next_hint().replace("HANDED OVER - ", "")
        self.kv(y + 1, xx, "now", l1[: mw - 6], 0, 6)
        self.put(y + 2, xx + 6, l2[: mw - 6], 0)
        self.kv(y + 3, xx, "FCU", "fake plant" if self.fcu_kind() == "fake" else "real Pixhawk", C["warn"] if self.fcu_kind() == "fake" else C["ok"], 6)
        prof = mm.get("profile")
        self.put(y + 3, xx + 18, ("mm %s" % (prof or "?"))[:18], C["warn"] if mm.get("lab") else (C["ok"] if prof else C["dim"]))
        self.put(y + 3, xx + 36, ("judge %s" % ((ocs or {}).get("judge") or "?"))[: mw - 36], C["warn"] if (ocs or {}).get("judge") == "fake" else (C["ok"] if ocs else C["dim"]))
        net = os.environ.get("NET_PROFILE", "")
        self.kv(y + 4, xx, "net", net or "?", C["ok"] if net else C["dim"], 6)
        lk = self.ocs_link_text(now).split(",")[0][:24]
        self.put(y + 4, xx + 12, lk, self.ocs_link_attr())
        agx = self.presence.get("cv_publish")
        agx_up = bool(agx) and bool(agx[1].get("up")) and now - agx[0] < 6
        cvt, cvs = cv_source(self.cv_health[1] if self.cv_health else None, (now - self.cv_health[0]) if self.cv_health else None, agx_up)
        cx = min(38, 12 + len(lk) + 3)                       # right after the link text: room for `CV sim (AGX, world_pool)`
        self.put(y + 4, xx + cx, cvt[: max(0, mw - cx)], C.get(cvs, C["dim"]))
        lb = self.presence.get("logger-boat")
        lb_up = bool(lb) and bool(lb[1].get("up")) and now - lb[0] < 6
        self.kv(y + 5, xx, "log", ("usv_ui %s   SQLite boat %s" % (("ON " + os.path.basename(self.base)) if self.base else "OFF", "ON" if lb_up else "OFF"))[: mw - 6],
                C["warn"] if (self.base or lb_up) else C["dim"], 6)
        you = "`takeover` / `stop` / `x` take the FCU back" if not self.active else "`handover` lets the mission manager take the FCU (you stay here)"
        self.kv(y + 6, xx, "you", you[: mw - 6], C["warn"], 6)

    def draw_flow_box(self, y, x, h, w, now):
        C = self.C
        self.box(y, x, h, w, "FLOW  (where the run is; reverse = now, green = done)")
        if flow_stage is None:
            self.put(y + 1, x + 2, "playbook not importable: no flow (optiprimex/ at the tree root ?)", C["dim"])
            return
        stage = flow_stage(self.flow_state(), self.args.vehicle_id, self.active)
        wide = w >= 140
        rows = flow_rows(stage, wide)
        arrow = " ---> " if wide else " -> "
        styles = {"cur": C["disarmed"] | curses.A_BOLD, "done": C["ok"], "future": C["dim"]}
        s = self.scr

        def draw_row(yy, x0, cells):
            cx = x0
            for i, (label, cap, stt) in enumerate(cells):
                cell = " %s " % label
                self.put(yy, cx, cell, styles[stt])
                self.put(yy + 1, cx + 1, cap, C["dim"])
                cx += len(cell)
                if i < len(cells) - 1:
                    try:
                        s.attron(C["dim"]); s.hline(yy, cx + 1, curses.ACS_HLINE, len(arrow) - 3); s.attroff(C["dim"])
                    except curses.error:
                        pass
                    self.put(yy, cx + len(arrow) - 2, ">", C["dim"])
                    cx += len(arrow)
            return cx
        self.put(y + 1, x + 2, "boat ", C["title"])
        end = draw_row(y + 1, x + 8, rows[0])
        try:
            s.attron(C["dim"])
            s.hline(y + 1, end + 1, curses.ACS_HLINE, 2)
            s.addch(y + 1, end + 3, curses.ACS_URCORNER)
            s.addch(y + 2, end + 3, curses.ACS_VLINE)
            s.addch(y + 3, x + 8, curses.ACS_ULCORNER)
            s.hline(y + 3, x + 9, curses.ACS_HLINE, max(0, end + 3 - (x + 8) - 1))
            s.addch(y + 3, end + 3, curses.ACS_LRCORNER)
            s.attroff(C["dim"])
        except curses.error:
            pass
        self.put(y + 3, x + 2, "OCS  ", C["title"])
        draw_row(y + 4, x + 8, rows[1])
        who = "FCU: usv_ui (this console)" if self.active else "FCU: mission manager"
        self.put(y + 1, x + w - 2 - len(who), who, C["warn"] if self.active else C["ok"])

    def draw_thrusters_rc(self, y, x, h, w):
        C, t = self.C, self.tel
        self.box(y, x, h, w, "THRUSTERS  /  RC INPUT")
        for i in range(4):
            pwm = t.servo[i]
            dev = (pwm - 1500) / 400.0 if pwm else 0.0
            n = int(round(abs(dev) * 5))
            bar = ("%5s|%-5s" % ("#" * n, "")) if dev < 0 else ("%5s|%-5s" % ("", "#" * n))
            self.put(y + 1 + i, x + 2, "%-8s %4d" % (MOTOR_NAMES[i], pwm), curses.A_BOLD if pwm else C["dim"])
            self.put(y + 1 + i, x + 16, "[", C["dim"]); self.put(y + 1 + i, x + 17, bar, C["bar"] if n else C["dim"]); self.put(y + 1 + i, x + 28, "]", C["dim"])
            self.put(y + 1 + i, x + 30, "%+.2f" % dev, C["bar"] if n else 0)
        self.put(y + 5, x + 2, "1500 = stop, +-400 us = +-1", C["dim"])
        rc = t.rc
        rows = [("RC1", 0, "steer"), ("RC3", 2, "surge"), ("RC4", 3, "sway"), ("RC5", 4, "SF kill"), ("RC6", 5, "override")]
        rx = x + 38
        for k, (name, idx, role) in enumerate(rows):
            v = rc[idx] if rc else 0
            hi = v > 1750
            attr = C["bad"] if (hi and idx in (4, 5)) else (0 if v else C["dim"])
            self.put(y + 1 + k, rx, ("%s %4d %s" % (name, v, role))[: w - 40], attr)
        if not rc:
            self.put(y + 1, rx + 16, "no RC yet"[: w - 56], C["dim"])

    def draw_network_box(self, y, x, h, w, now):
        C = self.C
        self.box(y, x, h, w, "NETWORK  (brokers, nodes, links)")
        xx, mw = x + 2, w - 4
        mq_ok = self.mq is not None and self.mq.connected
        self.put(y + 1, xx, ("boat %s  %s" % ("UP" if mq_ok else ("DOWN" if self.mq is not None else "off"), self.mq_line().replace("mqtt ", "")))[: mw - 18],
                 C["ok"] if mq_ok else C["bad"])
        br = "OCS bridge up" if self.bridge else ("OCS bridge DOWN" if self.bridge is False else "bridge ?")
        self.put(y + 1, x + w - 2 - len(br), br, C["ok"] if self.bridge else (C["bad"] if self.bridge is False else C["dim"]))
        ocs_age = (now - self.ocs[0]) if self.ocs else None
        self.put(y + 2, xx, ("hub  via bridge: ocs/state %s" % (("%.1f s" % ocs_age) if ocs_age is not None else "none"))[: mw - 18],
                 C["ok"] if (ocs_age is not None and ocs_age < 5) else C["dim"])
        if self.ocs:
            self.put(y + 2, x + w - 2 - 18, ("(last %s)" % time.strftime("%H:%M:%S", time.localtime(time.time() - ocs_age)))[:18], C["dim"])
        self.put(y + 3, xx, "boat ", C["dim"])
        cx = xx + 5
        nodes = sorted(self.presence.items())
        if not nodes:
            self.put(y + 3, cx, "no presence beats yet (mission_manager, cv_publish, logger-boat)"[: mw - 5], C["dim"])
        for name, (t_recv, body) in nodes:
            if name == "usv_ui":
                continue
            up = bool(body.get("up")) and now - t_recv < 6
            short = name.replace("mission_manager", "mm").replace("cv_publish", "cv_pub").replace("fake_", "f_")
            txt = ("%s %.0fs" % (short, now - t_recv)) if up else ("%s DOWN" % short)
            if cx + len(txt) > x + w - 2:
                break
            self.put(y + 3, cx, txt, C["ok"] if up else C["dim"])
            cx += len(txt) + 2
        agx = self.presence.get("cv_publish")
        agx_up = bool(agx) and bool(agx[1].get("up")) and now - agx[0] < 6
        h_age = (now - self.cv_health[0]) if self.cv_health else None
        self.put(y + 4, xx, ("AGX  cv_publish %s   detector health %s   obs rx %d" % ("up" if agx_up else "none", ("%.0f s" % h_age) if h_age is not None else "none", self.cv_n_rx))[: mw],
                 C["ok"] if agx_up else C["dim"])
        t = self.tel
        hb = ("FCU %.1f s" % (now - t.hb_t)) if t.hb_t else "FCU none"
        sh = ("share mm %.0f s" % (now - self.share.t_in)) if (self.share is not None and self.share.t_in) else "share -"
        self.put(y + 5, xx, "links ", C["dim"])
        cx = xx + 6
        for txt, ok in ((hb, bool(t.hb_t) and now - t.hb_t < 3), (sh, True if (self.share is not None and self.share.t_in and now - self.share.t_in < 3) else None),
                        ("UAV relay none", None)):
            if cx + len(txt) > x + w - 2:
                break
            self.put(y + 5, cx, txt, C["dim"] if ok is None else (C["ok"] if ok else C["bad"]))
            cx += len(txt) + 2

    def draw_warnings_box(self, y, x, h, w, now):
        C = self.C
        items = self.warnings(now)
        self.box(y, x, h, w, "WARNINGS  (active; self-clearing)")
        if items:
            try:
                self.scr.addch(y, x, curses.ACS_ULCORNER, C["bad"])
            except curses.error:
                pass
        else:
            self.put(y + 1, x + 2, "none", C["ok"])
        for i, (age, text, attr) in enumerate(items[: h - 2]):
            self.put(y + 1 + i, x + 2, ("%4.0f s  %s" % (age, text))[: w - 4], C[attr])
        if len(items) > h - 2:
            self.put(y + h - 2, x + w - 8, "+%d" % (len(items) - (h - 2)), C["bad"])

    def draw_events_console(self, y, h, ew, cw, now):
        C = self.C
        self.box(y, 0, h, ew, "EVENTS  (what happened: FCU, mm, links; LOG tab = all)")
        n = max(1, h - 2)
        ev = [e for e in self.log if e[2] == "event"]
        show = ev[-n:] if not self.log_scroll else ev[max(0, len(ev) - n - self.log_scroll): max(1, len(ev) - self.log_scroll)]
        for i, (ts, text, kind) in enumerate(show):
            attr = C["bad"] if text.startswith(("OCS LINK", "FCU", "mm: SILENT", "refused", "bad")) or " DOWN" in text else (C["warn"] if text.startswith(("mm:", "map:", "HANDED", "ACTIVE")) else 0)
            self.put(y + 1 + i, 2, ("%4.0fs %s" % (now - ts, text))[: ew - 4], attr)
        self.box(y, ew + 1, h, cw, "CONSOLE  (your commands -> result, hint)")
        con = [e for e in self.log if e[2] in ("cmd", "result")]
        for i, (ts, text, kind) in enumerate(con[-n:]):
            attr = C["title"] if kind == "cmd" else (C["bad"] if text.startswith(("refused", "bad", "unknown", "warning")) else C["ok"])
            self.put(y + 1 + i, ew + 3, (text if kind == "cmd" else "  " + text)[: cw - 4], attr)

    COMMAND_GROUPS = [
        ("DRIVE", ["mode", "arm", "disarm", "surge", "sway", "yaw", "motor", "burst", "stop|x"]),
        ("NAVIGATE", ["save", "here", "points", "del", "co", "goto", "speed", "path", "pathto", "pathgo", "set"]),
        ("SWAY", ["swayto", "swaymix", "swaycal", "dockto", "pathgen"]),
        ("STACK", ["handover", "takeover", "reset", "step", "skip", "cv", "fcu", "share", "param", "map"]),
        ("RECORD", ["log on|off", "note", "help", "quit"]),
    ]

    def word_style(self, word, now):
        C, t = self.C, self.tel
        mm = self.mm_body() or {}
        fcu = word in ("mode", "arm", "disarm", "surge", "sway", "yaw", "motor", "burst", "goto", "speed", "pathto", "pathgo", "swayto", "dockto", "swaycal", "param", "fcu")
        if fcu and not self.active:
            return C["dim"]
        if word in ("surge", "sway", "yaw", "motor", "burst", "swaycal") and not (t.armed and t.mode == 20):
            return C["dim"]
        if word in ("goto", "pathto", "swayto") and (t.fix < 3 or not t.hb_t):
            return C["dim"]
        if word == "dockto" and (self.fcu_kind() != "fake" or not (self.cvmap and self.cvmap.entities("dock"))):
            return C["dim"]
        if word == "pathgen" and not (self.cvmap and self.cvmap.entities("buoy")):
            return C["dim"]
        if word == "handover":
            return C["ok"] if self.active else C["dim"]
        if word == "takeover":
            return C["dim"] if self.active else C["ok"]
        if word == "skip":                                  # v0.1.68: green when the mission manager says skip is possible now
            return C["ok"] if ((mm or {}).get("free_run") or {}).get("skip", {}).get("ok") else C["dim"]
        if word in ("reset", "step"):
            if not mm:
                return C["dim"]
            if word == "step":
                return C["ok"] if str(mm.get("task_name") or "").startswith("TEST_") and mm.get("state") == "AUTO_TASK" else C["dim"]
            return C["ok"]
        if word == "cv":
            return C["ok"] if (self.mq is not None and self.mq.connected) else C["dim"]
        if word in ("fcu", "param"):
            return C["ok"] if (not t.armed and self.active) else C["dim"]
        if word == "stop|x":
            return C["ok"]
        return C["ok"] if word not in ("help", "quit") else 0

    def draw_commands_box(self, y, x, h, w, now):
        C = self.C
        self.box(y, x, h, w, "COMMANDS  (green = allowed now; type a word for its usage)")
        maxw = w - 13
        row = 0
        for label, words in self.COMMAND_GROUPS:
            line, cur, first = [], 0, True
            for wd in words:
                add = len(wd) + (3 if line else 0)
                if cur + add > maxw and line:
                    if row >= h - 2:
                        return
                    self._emit_words(y + 1 + row, x, label if first else "", line)
                    row += 1
                    first = False
                    line, cur = [], 0
                line.append((wd, self.word_style(wd, now)))
                cur += len(wd) + (3 if len(line) > 1 else 0)
            if line and row < h - 2:
                self._emit_words(y + 1 + row, x, label if first else "", line)
                row += 1

    def _emit_words(self, yy, x, label, line):
        C = self.C
        self.put(yy, x + 2, "%-8s" % label, C["title"])
        cx = x + 11
        for j, (wd, attr) in enumerate(line):
            self.put(yy, cx, wd, attr)
            cx += len(wd)
            if j < len(line) - 1:
                try:
                    self.scr.addch(yy, cx + 1, curses.ACS_VLINE, C["dim"])
                except curses.error:
                    pass
                cx += 3

    # -- MISSION tab (the same reference as op-tui's; selection commands are the OCS's) --
    def draw_mission_tab(self, now):
        C = self.C
        h, w = self.scr.getmaxyx()
        if not MISSIONS:
            self.put(2, 2, "playbook not importable: no mission catalogue (optiprimex/ at the tree root ?)", C["bad"])
            return
        self.draw_banner()
        ocs = self.ocs_body()
        mm = self.mm_body() or {}
        v, tn, ts = mm.get("state"), mm.get("task_name"), mm.get("task_state")
        running = tn if v in ("AUTO_TASK", "AUTO_SAFE_HOLD") else None
        mi = (ocs or {}).get("mission") or {}
        order = MISSION_ORDER
        cur_task = order[self.mission_cursor % len(order)]                  # a mission code (m4t0 ...)
        rcode = code_of(running) if running else None                        # the running wire task as a code
        pcode = ((mi.get("code") or code_of(mi.get("task"), mi.get("tier")) or "").replace("sim", "")) if mi.get("task") else ""
        top = min(30, h - 4 - 2 - 8)
        lw = 48
        self.box(2, 0, top, lw, "MISSIONS  (Up/Down highlight)")
        self.put(3, 2, "code   wire  what                      tier", C["dim"])
        for i, k in enumerate(order):
            ms = MISSIONS[k]
            yy = 4 + i * 2
            if yy + 1 >= 2 + top - 1:
                break
            attr = {"built": C["ok"], "planned": C["dim"], "bench": C["warn"], "test": C["ok"]}[ms.status]
            self.put(yy, 2, ("%-6s %-5s %-25s %s" % (k, ms.task[:5], ms.title[:25], ms.tier[:5]))[: lw - 4], (C["disarmed"] | curses.A_BOLD) if k == cur_task else attr)
            flag = "  * running" if rcode == k else (("  preselected (OCS)" + (" sim" if mi.get("sim") else "")) if pcode == k else "")
            self.put(yy + 1, 15, (ms.status + flag)[: lw - 17], C["warn"] if flag else C["dim"])
        yy = 4 + len(order) * 2 + 1
        self.put(yy, 2, ("live         %s" % (("%s %s" % (rcode, ts)) if running else "no task running"))[: lw - 4], C["warn"] if running else C["dim"])
        self.put(yy + 1, 2, ("preselected  %s%s" % (mi.get("code") or pcode or "-", " --auto" if mi.get("auto") else "") + ("" if ocs else "  (no OCS view)"))[: lw - 4], 0)
        self.put(yy + 2, 2, "m<task>t<tier>: m4t0 = Task 4 core; +sim = fake FCU only", C["dim"])
        self.put(yy + 3, 2, "green built  grey planned  yellow bench", C["dim"])
        self.put(yy + 4, 2, "selection is typed on the OCS (ocs_cli)", C["dim"])
        self.put(yy + 5, 2, "T2 and the UUV: not attempted (TIER_NONE)", C["dim"])
        rx = lw + 1
        rw = w - rx - 1
        ms = MISSIONS[cur_task]
        self.box(2, rx, top, rw, ("%s  -  %s (%s %s)" % (cur_task, ms.title, ms.task, ms.tier)) + (("   [%s]" % ms.status) if ms.status != "built" else "") + (("   * running, %s" % ts) if rcode == cur_task else ""))
        x = rx + 2
        mw = rw - 4
        self.kv(3, x, "what", ms.what[0][: mw - 8], 0, 8)
        self.put(4, x + 8, (ms.what[1] if len(ms.what) > 1 else "")[: mw - 8], 0)
        self.kv(5, x, "starts", ms.starts[: mw - 8], 0, 8)
        self.kv(6, x, "ends", ms.ends[: mw - 8], 0, 8)
        self.kv(7, x, "judge", ("in: %s   out: %s" % (ms.judge_in, ms.judge_out))[: mw - 8], C["dim"], 8)
        self.kv(8, x, "config", ms.config[: mw - 8], C["dim"], 8)
        ty = 10
        self.put(ty, x, ("%-13s %-34s %-11s %-23s judge" % ("stage", "meaning", "behaviour", "leaves on"))[: mw], C["title"])
        cur_stage = ts if rcode == cur_task else None
        passed = cur_stage is not None
        for i, stg in enumerate(ms.stages):
            if ty + 1 + i >= 2 + top - 2:
                break
            if stg.name == cur_stage:
                attr, passed = C["disarmed"] | curses.A_BOLD, False
            else:
                attr = C["ok"] if passed else C["dim"]
            self.put(ty + 1 + i, x, (" %-12s" % stg.name)[:14], attr)
            self.put(ty + 1 + i, x + 14, ("%-34s %-11s %-23s %s" % (stg.meaning[:33], stg.behaviour[:11], stg.leaves_on[:23], stg.judge))[: mw - 14], 0 if stg.name == cur_stage else C["dim"])
        ny = ty + 1 + min(len(ms.stages), 11)
        if rcode == cur_task:
            self.put(ny, x, ("now      %s: %s" % (cur_stage, task_stage_meaning(ms.task, cur_stage)))[: mw], C["warn"])
        else:
            self.put(ny, x, ("now      %s is not running" % cur_task + ((" (%s is, in %s)" % (rcode, ts)) if running else "") + " - the stages light up once it runs")[: mw], C["dim"])
        if ms.note:
            self.put(ny + 1, x, ("note     " + ms.note)[: mw], C["dim"])
        hy = ny + 3
        hh = 2 + top - hy - 1
        if hh >= 4 and how_to_select is not None:
            steps, needs = how_to_select(cur_task, ocs, self.args.vehicle_id, self.active, {"health": {"bridge_up": self.bridge}} if self.bridge is not None else None)
            steps = [(n_, t_, c_, (nt.replace("Enter here = into the prompt", "typed on the OCS (ocs_cli)")) ) for n_, t_, c_, nt in steps]
            state_txt = "%s, vehicle %s%s%s" % ((ocs or {}).get("r") or "--", v or "?", (" (%s)" % running) if running else "", ", usv_ui ACTIVE" if self.active else ", handed over")
            self.box(hy, rx + 1, hh, rw - 2, ("HOW TO SELECT %s  from this state: %s  (typed on the OCS)" % (cur_task, state_txt))[: rw - 6])
            hx = rx + 3
            c1 = min(46, (rw - 6) // 2 + 4)
            for i, (num, text, cmd, note) in enumerate(steps[: hh - 3]):
                self.put(hy + 1 + i, hx, num, C["warn"] if num not in ("boat", "*", "-") else C["title"])
                self.put(hy + 1 + i, hx + 5, text[: (c1 - 6) if (cmd or note) else (rw - 10)], 0)
                if cmd:
                    self.put(hy + 1 + i, hx + c1, cmd[: rw - 8 - c1], C["ok"] if not cmd.startswith("inject") else C["warn"])
                    if note and hx + c1 + len(cmd) + 3 + len(note) < rx + rw - 3:
                        self.put(hy + 1 + i, hx + c1 + len(cmd) + 3, note, C["dim"])
                elif note:
                    self.put(hy + 1 + i, hx + c1, note[: rw - 8 - c1], C["dim"])
            if needs:
                cx = hx
                self.put(hy + hh - 2, cx, "needs", C["title"]); cx += 7
                for label, ok in needs:
                    txt = "%s %s" % (label, "ok" if ok else ("?" if ok is None else "NO"))
                    if cx + len(txt) > rx + rw - 3:
                        break
                    self.put(hy + hh - 2, cx, txt, C["dim"] if ok is None else (C["ok"] if ok else C["bad"]))
                    cx += len(txt) + 3
        y2 = 2 + top
        self.box(y2, 0, h - 4 - y2, w - 1, "ALL MISSIONS AT A GLANCE  (stages in order; grey = not built yet)")
        yy = y2 + 1
        for k in order:
            ms2 = MISSIONS[k]
            if yy + 1 >= h - 5:
                break
            attr = {"built": C["ok"], "planned": C["dim"], "bench": C["warn"], "test": C["ok"]}[ms2.status]
            self.put(yy, 2, "%-6s %-5s" % (k, ms2.task[:5]), attr)
            self.put(yy, 14, " -> ".join(s.name for s in ms2.stages)[: w - 16], 0 if ms2.status != "planned" else C["dim"])
            self.put(yy + 1, 14, (ms2.note or ("%s; %s" % (ms2.starts, ms2.judge_out)))[: w - 16], C["dim"])
            yy += 2

    def draw_prompt(self):
        h, w = self.scr.getmaxyx()
        C = self.C
        last = self.log[-1][1] if self.log else ""
        self.put(h - 4, 0, ("last: " + last)[:w - 1], C["warn"] if last.startswith(("refused", "bad", "unknown")) else 0)
        usage = self.usage_for(self.cmdline)
        if usage:
            self.put(h - 3, 0, ("usage: " + usage)[:w - 1], C["warn"] if usage.startswith("unknown") else C["title"])
            ex = self.example_for(self.cmdline)
            self.put(h - 2, 0, ("example: " + ex) if ex else "", C["dim"])
        else:
            self.put(h - 3, 0, ("next: " + self.next_hint())[:w - 1], C["title"])
            self.put(h - 2, 0, "example: mode 20  |  surge 0.2 1  |  goto A 0.8  |  pathto A B 0.6  |  handover  |  fcu fake", C["dim"])
        self.put(h - 1, 0, "> " + self.cmdline, curses.A_BOLD)

    def draw_state_strip(self, y, now):
        """One-line vehicle summary used by MAP / POINTS / LOG screens."""
        t, C = self.tel, self.C
        hb_ok = t.hb_t is not None and now - t.hb_t < 3
        self.put(y, 1, "HB" if hb_ok else "NO HB", C["ok"] if hb_ok else C["bad"])
        self.put(y, 7, "%s" % MODE_NAMES.get(t.mode, "?"), C["warn"])
        self.put(y, 22, " ARMED " if t.armed else " disarmed ", C["armed"] if t.armed else C["disarmed"])
        self.put(y, 34, "GPS %s sats %d hdop %.1f" % ({3: "3D", 4: "DGPS", 5: "RTKf", 6: "RTK"}.get(t.fix, str(t.fix)), t.sats, t.hdop),
                 C["ok"] if t.fix >= 3 else C["bad"])
        a = self.action
        if a:
            self.put(y, 60, " %s %s %.1fs " % (a["label"], a["phase"], max(0.0, a["t_phase_end"] - now)),
                     C["burst"] if a["phase"] == "burst" else C["warn"])
        elif self.goto:
            self.put(y, 60, "goto %s" % self.goto["name"], C["warn"])

    # -- MAIN, compact (below 120x44: the v0.1.17 layout) --
    def draw_main_compact(self, now):
        t, C = self.tel, self.C
        h, w = self.scr.getmaxyx()
        lw = 48                                   # left column width
        # vehicle state
        self.box(1, 0, 6, lw, "VEHICLE")
        hb_ok = t.hb_t is not None and now - t.hb_t < 3
        self.kv(2, 2, "link", ("HB %.1f s ago" % (now - t.hb_t)) if t.hb_t else "NO HEARTBEAT", C["ok"] if hb_ok else C["bad"])
        self.put(2, 24, "sys %s" % (t.sysid if t.sysid is not None else "-"), C["dim"])
        self.kv(3, 2, "mode", "%s (%s)" % (MODE_NAMES.get(t.mode, "?"), t.mode), C["warn"])
        self.put(4, 2, " ARMED " if t.armed else " disarmed ", C["armed"] if t.armed else C["disarmed"])
        self.put(4, 16, "batt %s" % ("%.1f V" % t.batt if t.batt else "--"))
        ekf = t.ekf_pos_ok()
        self.kv(5, 2, "EKF", "posH %s  flags 0x%03x" % ("OK" if ekf else "NO POS", t.ekf_flags), C["ok"] if ekf else C["bad"])
        # position
        self.box(1, lw + 1, 6, w - lw - 2, "POSITION / GPS")
        x = lw + 3
        self.kv(2, x, "lat", "%.7f" % t.lat if t.lat is not None else "--", curses.A_BOLD, 5)
        self.kv(2, x + 18, "lon", "%.7f" % t.lon if t.lon is not None else "--", curses.A_BOLD, 5)
        self.kv(3, x, "hdg", "%5.1f deg" % t.hdg if t.hdg is not None else "--", 0, 5)
        self.kv(3, x + 18, "gs", "%.2f m/s" % t.gs, 0, 5)
        self.kv(3, x + 33, "v", "%+.2f %+.2f" % (t.vn, t.ve), 0, 2)
        fix = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fix"}.get(t.fix, str(t.fix))
        self.kv(4, x, "GPS", fix, C["ok"] if t.fix >= 3 else C["bad"], 5)
        self.kv(4, x + 18, "sats", "%d" % t.sats, C["ok"] if t.sats >= 8 else (C["warn"] if t.sats >= 5 else C["bad"]), 5)
        self.kv(4, x + 33, "hdop", "%.1f" % t.hdop, C["ok"] if t.hdop < 1.5 else C["warn"], 5)
        loc = self.local()
        self.kv(5, x, "CO", ("N %+.1f E %+.1f" % loc) if loc else ("not set" if not self.origin.is_set else "--"),
                C["co"] if loc else C["dim"], 5)
        g = self.goto
        if g and t.lat is not None:
            self.put(5, x + 22, "goto %s %.1f m / %03.0f" % (g["name"], haversine(t.lat, t.lon, g["lat"], g["lon"]),
                                                            bearing(t.lat, t.lon, g["lat"], g["lon"])), C["warn"])
        # thrusters
        self.box(7, 0, 8, lw, "THRUSTERS  (SERVO_OUTPUT_RAW)")
        for i in range(4):
            pwm = t.servo[i]
            dev = (pwm - 1500) / 400.0 if pwm else 0.0
            n = int(round(abs(dev) * 10))
            bar = ("%10s|%-10s" % ("#" * n, "")) if dev < 0 else ("%10s|%-10s" % ("", "#" * n))
            self.put(8 + i, 2, "%-9s %4d" % (MOTOR_NAMES[i], pwm), curses.A_BOLD if pwm else C["dim"])
            self.put(8 + i, 17, "[", C["dim"])
            self.put(8 + i, 18, bar, C["bar"] if n else C["dim"])
            self.put(8 + i, 39, "]", C["dim"])
            self.put(8 + i, 41, "%+.2f" % dev, C["bar"] if n else 0)
        self.put(13, 2, "1500 = stop, +-400 us = +-1.00", C["dim"])
        # RC
        self.box(7, lw + 1, 8, w - lw - 2, "RC INPUT")
        rc = t.rc
        rows = [("RC1", 0, "steering / yaw"), ("RC3", 2, "throttle / surge"), ("RC4", 3, "lateral / sway"),
                ("RC5", 4, "SF  motor E-stop"), ("RC6", 5, "MANUAL override"), ("RC8", 7, "mode: 20 / HOLD / GUIDED")]
        for k, (name, idx, role) in enumerate(rows):
            v = rc[idx] if rc else 0
            hi = v > 1750
            attr = C["bad"] if (hi and idx in (4, 5)) else (0 if v else C["dim"])
            self.put(8 + k, x, "%s %4d  %s" % (name, v, role), attr)
        if not rc:
            self.put(8, x + 30, "no RC_CHANNELS yet", C["dim"])
        # action + the v4 stack line
        self.box(15, 0, 4, w - 1, "ACTION  /  STACK")
        a = self.action
        if a:
            left = max(0.0, a["t_phase_end"] - now)
            self.put(16, 2, " %s " % a["label"], C["burst"] if a["phase"] == "burst" else C["warn"])
            self.put(16, 4 + len(a["label"]), "  phase %-5s %4.1f s left   controls [%s]" % (
                a["phase"].upper(), left, "  ".join("%+.2f" % c for c in a["controls"])), curses.A_BOLD)
        else:
            self.put(16, 2, "idle   (a burst = 0.3 s zeros, burst, 0.5 s zeros; needs mode 20 + armed)", C["dim"])
            self.put(16, 80, "sway mix: %s" % self.swaymix_text(), C["ok"] if self.swaymix else C["warn"])
        self.draw_stack_line(17, now)
        self.draw_mm_box(19, now)
        # FCU messages (top left) + command log (bottom left) | commands (right)
        y = 25
        cw = 54
        lh = h - 4 - y                            # height of the lower area
        mh = max(3, min(12, lh // 2))             # FCU MESSAGES box height
        self.box(y, 0, mh, w - cw - 1, "FCU MESSAGES (STATUSTEXT, newest first)  %d" % len(t.status))
        if not t.status:
            self.put(y + 1, 2, "none yet (messages sent before this tool connected are not repeated)", C["dim"])
        for i, (ts, sev, text, wall) in enumerate(reversed(t.status[-(mh - 2):])):
            self.put(y + 1 + i, 2, ("%s %-8s %s" % (wall, "[%s]" % self.SEVERITY.get(sev, sev), text))[:w - cw - 4],
                     C["bad"] if sev <= 3 else C["warn"] if sev <= 5 else 0)
        ly = y + mh
        self.box(ly, 0, lh - mh, w - cw - 1, "LOG (commands and results, newest last)")
        n_log = lh - mh - 2
        for i, (ts, text, _k) in enumerate(self.log[-max(1, n_log):]):
            self.put(ly + 1 + i, 2, "%4.0fs %s" % (now - ts, text[:w - cw - 10]))
        self.box(y, w - cw - 1, lh, cw, "COMMANDS  (usage appears above the prompt)")
        cx = w - cw + 1
        cmds = [("mode 20  ->  arm", "then bursts; disarm at the end"),
                ("surge 0.3 2", "ahead 2 s @30%; -0.3 = astern"),
                ("sway 0.3 2", "to stbd; yaw 0.3 2 bow to stbd"),
                ("motor 3 0.2 1", "1 PORT 2 STERN 3 STBD 4 BOW"),
                ("save A / goto A 0.8", "store pos / GUIDED to it"),
                ("stop  or  x", "cancel burst; GUIDED -> HOLD"),
                ("pathto A B 0.8", "path through points (PATH tab)"),
                ("swaycal 0.3 20 [F]", "auto: mode20,arm,log,sway,log off"),
                ("log on LABEL / log off", "record .tlog/.csv/.events"),
                ("cv on / cv status", "AGX detections -> map (MAP tab)"),
                ("handover / takeover", "FCU to mission manager / back"),
                ("reset / step", "mm: reset run | m0t0 step"),
                ("fcu fake | real", "swap the FCU link live"),
                ("note TEXT", "annotate the log")]
        for i, (c_, d_) in enumerate(cmds[:h - 4 - y - 2]):
            self.put(y + 1 + i, cx, "%-20s" % c_, curses.A_BOLD)
            self.put(y + 1 + i, cx + 21, d_, C["dim"])

    def draw_mm_box(self, y, now):
        """MISSION MANAGER: its vehicle state with a one-line meaning, the task and its stage, the run, and what usv_ui itself does."""
        C = self.C
        h, w = self.scr.getmaxyx()
        self.box(y, 0, 6, w - 1, "MISSION MANAGER  (local/usv/mm/state)   V = vehicle state, stage = task sub-state")
        mm = self.mm[1] if self.mm else None
        age = (time.monotonic() - self.mm[0]) if self.mm else None
        x = 2
        if mm is None or age is None or age > 3.0:
            self.put(y + 1, x, "not seen" if mm is None else "SILENT %.0f s (last state %s)" % (age, mm.get("state")), C["bad"])
            self.put(y + 2, x, "the mission manager runs on this host through the share (scripts/run_usv.sh starts it); 'share' shows the link", C["dim"])
        else:
            v = mm.get("state") or "?"
            code = {"BOOT": "V0", "KILLED": "V1", "MANUAL": "V2", "AUTO_HOLD": "V3", "AUTO_TASK": "V4", "AUTO_SAFE_HOLD": "V5", "FAULT": "V6"}.get(v, "")
            attr = C["bad"] if v in ("KILLED", "FAULT") else C["warn"] if v.startswith("AUTO") else C["ok"]
            self.put(y + 1, x, " %s %s " % (code, v), attr | curses.A_BOLD)
            self.put(y + 1, x + 20, (V_MEANING.get(v, "") or "")[:w - 26], C["dim"])
            tn, ts = mm.get("task_name"), mm.get("task_state")
            stage = task_stage_meaning(tn, ts) if tn else ""
            self.put(y + 2, x, ("task  %s %s   %s" % (tn, ts or "", stage) if tn else "task  none (AUTO_HOLD holds position; start_task / goto from the OCS)")[:w - 4],
                     C["warn"] if tn else C["dim"])
            run = mm.get("run") or {}
            hf = (mm.get("health") or {}).get("faults") or []
            self.put(y + 3, x, ("run %s  decl %s   hold %s   fault %s   faults %s   ui_active %s" % (
                run.get("run_id") or "-", run.get("declaration_seq") or "-", mm.get("hold_reason") or "-", mm.get("fault") or "-",
                ",".join(hf) if hf else "-", mm.get("ui_active")))[:w - 4], C["bad"] if mm.get("fault") else 0)
        # usv_ui's own stage, so the two sides read together
        a = self.action
        own = ("burst %s %s" % (a["label"], a["phase"]) if a else
               "path %s %s" % ("-".join(self.path["anchors"]), self.path["state"]) if self.path and self.path["state"] in ("waiting", "running") else
               "sway %s %s->%s" % (self.sway["state"], self.sway["a_name"], self.sway["b_name"]) if self.sway and self.sway["state"] not in ("done", "aborted") else
               "goto %s" % self.goto["name"] if self.goto else "idle")
        who = "usv_ui ACTIVE: this console commands the FCU" if self.active else "HANDED OVER: the mission manager commands the FCU"
        self.put(y + 4, x, ("usv_ui  %s   |   %s   |   cv %s   |   reset / step talk to the mission manager" % (own, who, "on" if self.cv_on else "off"))[:w - 4],
                 C["ok"] if self.active else C["warn"])

    def draw_stack_line(self, y, now):
        """mqtt | share | mission manager | cv, in one line (MAIN) - the v4 health at a glance."""
        C = self.C
        x = 2
        s = self.mq_line()
        self.put(y, x, s, C["ok"] if (self.mq is not None and self.mq.connected) else (C["dim"] if self.mq is None and not self.mq_err else C["bad"]))
        x += len(s) + 3
        if self.share is not None:
            age = (now - self.share.t_in) if self.share.t_in else None
            s = "share %d/%d%s" % (self.share.n_out, self.share.n_in, (" tool msg %.0fs ago" % age) if age is not None else " no tool yet")
            self.put(y, x, s, C["ok"] if (age is not None and age < 3) else C["dim"])
        else:
            s = "share off" if not self.share_err else "share FAILED"
            self.put(y, x, s, C["bad"] if self.share_err else C["dim"])
        x += len(s) + 3
        s = self.mm_line()
        self.put(y, x, s, C["warn"] if ("SILENT" in s or "DIFFERS" in s) else (C["dim"] if "not seen" in s else 0))
        x += len(s) + 3
        if self.cv_on:
            n = len(self.cvmap.ents) if self.cvmap else 0
            h_age = (now - self.cv_health[0]) if self.cv_health else None
            s = "cv ON %d ent, det %s" % (n, ("%.0fs" % h_age) if h_age is not None else "none")
            self.put(y, x, s, C["ok"] if (h_age is not None and h_age < 3) else C["warn"])
        else:
            self.put(y, x, "cv off", C["dim"])

    def map_entities(self):
        """[(x, y, kind, colour, stale)] for render_map, and the gate midpoints."""
        if self.cvmap is None or not self.cvmap.ents:
            return [], []
        ents = [(e.x_n, e.y_e, e.kind, e.colour, e.state != "tracked") for e in self.cvmap.ents.values()]
        pose = self.poses.latest() if self.poses else None
        gates = [(g.mid_n, g.mid_e) for g in self.cvmap.gates(pose)]
        return ents, gates

    # -- MAP --
    def draw_map(self, now):
        t, C = self.tel, self.C
        h, w = self.scr.getmaxyx()
        self.draw_state_strip(1, now)
        iw = 30                                   # info column
        mh = h - 4 - 2                            # map box rows
        mw = w - iw - 1
        self.box(2, 0, mh, mw, "MAP  N up, E right   + CO   A.. points   arrow = boat")
        if not self.origin.is_set:
            self.put(4, 3, "no control origin yet: type  co  (at the boat) or  co NAME  (at a saved point)", C["warn"])
        else:
            pts = {}
            for name, p in self.points.data.items():
                pts[name] = self.origin.to_local(p["lat"], p["lon"])
            loc = self.local()
            veh = (loc[0], loc[1], t.hdg if t.hdg is not None else 0.0) if loc else None
            tgt = self.origin.to_local(self.goto["lat"], self.goto["lon"]) if self.goto else None
            ents, gates = self.map_entities()
            grid, m10 = render_map(mw - 2, mh - 2, pts, veh, tgt, self.trail, self.unicode,
                                   path=(self.path["pts"] if self.path else None), entities=ents, gates=gates)
            for r, row in enumerate(grid):
                c = 0
                while c < len(row):          # run-length draw: consecutive cells with the same style
                    style = row[c][1]
                    j = c
                    s = ""
                    while j < len(row) and row[j][1] == style:
                        s += row[j][0]
                        j += 1
                    self.put(3 + r, 1 + c, s, C.get(style, 0) if style else 0)
                    c = j
            self.put(2 + mh - 1, 3, " |<--------->| %.1f m " % m10, C["dim"])
        # info column
        x = mw + 2
        self.box(2, mw + 1, mh, iw, "LOCAL")
        o = self.origin.rec
        self.put(3, x, "CO", C["co"])
        self.put(4, x, ("%.6f" % o["lat"]) if o else "not set", 0)
        self.put(5, x, ("%.6f  %s" % (o["lon"], o.get("how", ""))) if o else "", C["dim"] if o else 0)
        loc = self.local()
        self.put(7, x, "boat", C["vehicle"])
        self.put(8, x, ("N %+7.1f  E %+7.1f" % loc) if loc else "--", curses.A_BOLD)
        self.put(9, x, "hdg %s  gs %.2f m/s" % ("%03.0f" % t.hdg if t.hdg is not None else "--", t.gs))
        self.put(11, x, "target", C["target"])
        g = self.goto
        if g and loc:
            tl = self.origin.to_local(g["lat"], g["lon"])
            self.put(12, x, "%s  N %+.1f E %+.1f" % (g["name"], tl[0], tl[1]))
            self.put(13, x, "dist %.1f m  brg %03.0f" % (haversine(t.lat, t.lon, g["lat"], g["lon"]), bearing(t.lat, t.lon, g["lat"], g["lon"])), C["warn"])
        else:
            self.put(12, x, "none", C["dim"])
        # CV / map block
        self.put(15, x, "cv map  R G buoys  D dock  | gate", C["ent_red"] if self.cv_on else C["dim"])
        if self.cvmap is None:
            self.put(16, x, "no optiprimex (cv unavailable)", C["dim"])
        else:
            n_tr = sum(1 for e in self.cvmap.ents.values() if e.state == "tracked")
            n_all = len(self.cvmap.ents)
            h_age = (now - self.cv_health[0]) if self.cv_health else None
            self.put(16, x, "%s  %d tracked / %d" % ("ON " if self.cv_on else "off", n_tr, n_all), C["ok"] if self.cv_on else 0)
            self.put(17, x, "detector %s  rx %d" % (("%.0f s ago" % h_age) if h_age is not None else "none", self.cv_n_rx),
                     C["ok"] if (h_age is not None and h_age < 3) else C["dim"])
            pose = self.poses.latest() if self.poses else None
            gs = self.cvmap.gates(pose)
            self.put(18, x, "gates %d%s" % (len(gs), ("  next thr %03d" % (round(gs[0].through_hdg_deg) % 360)) if gs else ""), C["gate"] if gs else C["dim"])
        self.put(20, x, "points  (N, E)", C["point"])
        yc = 2 + mh - 8                            # commands block at the bottom of the column
        for i, (name, p) in enumerate(sorted(self.points.data.items())[:max(0, yc - 22)]):
            pl = self.origin.to_local(p["lat"], p["lon"]) if self.origin.is_set else None
            self.put(21 + i, x, ("%-6s %+7.1f %+7.1f" % (name, pl[0], pl[1])) if pl else ("%-6s --" % name))
        self.put(yc, x, "COMMANDS (map)", C["title"])
        for i, (c_, d_) in enumerate([("co", "CO = boat, now"), ("co A", "CO = point A"), ("cv on", "detections -> map"),
                                      ("goto A 0.8", "GUIDED to A"), ("stop", "-> HOLD"), ("Tab", "next screen")]):
            self.put(yc + 1 + i, x, "%-11s" % c_, curses.A_BOLD)
            self.put(yc + 1 + i, x + 11, d_, C["dim"])

    # -- PATH --
    def draw_path(self, now):
        t, C = self.tel, self.C
        h, w = self.scr.getmaxyx()
        self.draw_state_strip(1, now)
        iw = 34
        mh = h - 4 - 2
        mw = w - iw - 1
        p = self.path
        self.box(2, 0, mh, mw, "PATH  . path   x carrot   A.. anchors   arrow = boat")
        if not self.origin.is_set:
            self.put(4, 3, "no control origin yet: type  co", C["warn"])
        else:
            pts = {}
            for name, pp in self.points.data.items():
                pts[name] = self.origin.to_local(pp["lat"], pp["lon"])
            loc = self.local()
            veh = (loc[0], loc[1], t.hdg if t.hdg is not None else 0.0) if loc else None
            carrot = p["carrot"] if (p and p["state"] in ("running", "done")) else None
            draw_path = p["pts"] if p else None
            if self.sway and self.sway["state"] not in ("done", "aborted") and not draw_path:
                draw_path, _ = resample([self.sway["a"], self.sway["b"]], 0.5)
                carrot = self.sway["b"]
            ents, gates = self.map_entities()
            grid, m10 = render_map(mw - 2, mh - 2, pts, veh, carrot, self.trail, self.unicode,
                                   path=draw_path, anchors=(p["anchors"] if p else None), entities=ents, gates=gates)
            for r, row in enumerate(grid):
                c = 0
                while c < len(row):
                    style = row[c][1]
                    j = c
                    s_ = ""
                    while j < len(row) and row[j][1] == style:
                        s_ += row[j][0]
                        j += 1
                    self.put(3 + r, 1 + c, s_, C.get(style, 0) if style else 0)
                    c = j
            self.put(2 + mh - 1, 3, " |<--------->| %.1f m " % m10, C["dim"])
        x = mw + 2
        self.box(2, mw + 1, mh, iw, "PATH STATUS")
        sway_live = self.sway is not None and self.sway["state"] not in ("done", "aborted")
        if p is None and not sway_live:
            self.put(3, x, "no path planned", C["dim"])
            self.put(4, x, "path A B [C..]   plan only", 0)
            self.put(5, x, "pathto A B [C..] [V]  plan + go", 0)
            self.put(6, x, "swayto A B [V]   sideways A->B", 0)
        elif p is not None:
            st = p["state"]
            attr = {"running": C["burst"], "done": C["ok"], "aborted": C["bad"], "waiting": C["warn"]}.get(st, C["warn"])
            self.put(3, x, " %s " % st.upper(), attr)
            self.put(3, x + 11, "boat-" + "-".join(p["anchors"]), curses.A_BOLD)
            self.put(4, x, "%s, %.1f m, %d pts" % ("spline" if p["smooth"] else "straight", p["s"][-1], len(p["pts"])), C["dim"])
            self.put(5, x, "la %.1f  step %.1f  corr %.1f m" % (p["la"], self.path_step, self.path_corridor), C["dim"])
            self.put(6, x, "speed %s  end r %.1f m" % (("%.2f m/s" % p["speed"]) if p["speed"] else "WP_SPEED", self.path_end_r), C["dim"])
            s_boat = p["s"][p["idx"]]
            self.put(8, x, "progress", C["title"])
            self.put(9, x, "%5.1f / %5.1f m  (%3.0f%%)" % (s_boat, p["s"][-1], 100.0 * s_boat / max(p["s"][-1], 1e-6)), curses.A_BOLD)
            bar_n = int(round(26 * s_boat / max(p["s"][-1], 1e-6)))
            self.put(10, x, "[" + "#" * bar_n + " " * (26 - bar_n) + "]", C["ok"])
            self.put(11, x, "xte %+5.2f m   max %.2f m" % (p["xte"], p["max_xte"]), C["warn"] if abs(p["xte"]) > 1.5 else 0)
            if p["carrot"]:
                self.put(12, x, "carrot N %+6.1f E %+6.1f (#%d)" % (p["carrot"][0], p["carrot"][1], p["carrot_idx"]), C["target"])
            el = ((p["t_done"] or now) - p["t_start"]) if p["t_start"] else 0.0
            self.put(13, x, "targets sent %d   %.0f s" % (p["n_sent"], el), C["dim"])
            self.put(15, x, "anchors (N, E)", C["point"])
            for i, (nm, c_) in enumerate(list(zip(p["anchors"], p["ctrl"][1:]))[:6]):
                self.put(16 + i, x, "%-6s %+7.1f %+7.1f" % (nm, c_[0], c_[1]))
        sw = self.sway
        if sw and sw["state"] not in ("done", "aborted") or (sw and p is None):
            y0 = 3 if p is None else 17
            attr = {"sway": C["burst"], "align": C["warn"], "approach": C["warn"], "to20": C["warn"],
                    "done": C["ok"], "aborted": C["bad"]}.get(sw["state"], 0)
            self.put(y0, x, " %s " % sw["state"].upper(), attr)
            self.put(y0, x + 16, "%s->%s" % (sw["a_name"], sw["b_name"]), curses.A_BOLD)
            self.put(y0 + 1, x, "line %03.0f  hold hdg %03.0f  v %.2f" % (sw["theta"], sw["psi_t"], sw["v"]), C["dim"])
            self.put(y0 + 2, x, "s %5.1f/%5.1f m  c %+5.2f m" % (sw["s"], sw["length"], sw["c"]), curses.A_BOLD)
            self.put(y0 + 3, x, "hdg err %+4.0f  max c %.2f e %.0f" % (sw["e"], sw["max_c"], sw["max_e"]),
                     C["warn"] if abs(sw["e"]) > 20 else 0)
            self.put(y0 + 4, x, "cmd [%s]" % " ".join("%+.2f" % c_ for c_ in sw["cmd"]), C["bar"])
        yc = 2 + mh - 8
        self.put(yc, x, "COMMANDS (path)", C["title"])
        for i, (c_, d_) in enumerate([("path A B C", "plan, show here"), ("pathto A B 0.8", "plan + go"),
                                      ("pathgo [V]", "follow planned"), ("swayto A B [V]", "sideways A->B"),
                                      ("set la 3", "lookahead m"), ("stop", "cancel -> HOLD")]):
            self.put(yc + 1 + i, x, "%-15s" % c_, curses.A_BOLD)
            self.put(yc + 1 + i, x + 15, d_, C["dim"])

    # -- POINTS --
    def draw_points(self, now):
        t, C = self.tel, self.C
        h, w = self.scr.getmaxyx()
        self.draw_state_strip(1, now)
        self.box(2, 0, h - 12, w - 1, "SAVED POINTS")
        self.put(3, 2, "%-8s %-12s %-13s %-5s %-17s %-8s %-8s %-7s %-6s" % ("name", "lat", "lon", "hdg", "saved (UTC)", "local N", "local E", "dist m", "brg"), C["title"])
        rows = sorted(self.points.data.items())
        if not rows:
            self.put(4, 2, "no points yet - drive somewhere and type: save A", C["dim"])
        for i, (name, p) in enumerate(rows[:h - 19]):
            d = b = None
            if t.lat is not None:
                d = haversine(t.lat, t.lon, p["lat"], p["lon"])
                b = bearing(t.lat, t.lon, p["lat"], p["lon"])
            pl = self.origin.to_local(p["lat"], p["lon"]) if self.origin.is_set else None
            active = self.goto and self.goto["name"] == name
            self.put(4 + i, 2, "%-8s %-12.7f %-13.7f %-5s %-17s %-8s %-8s %-7s %-6s" % (
                name, p["lat"], p["lon"], ("%.0f" % p["hdg"]) if p.get("hdg") is not None else "-", p.get("utc", "-"),
                ("%+.1f" % pl[0]) if pl else "-", ("%+.1f" % pl[1]) if pl else "-",
                ("%.1f" % d) if d is not None else "-", ("%03.0f" % b) if b is not None else "-"),
                C["warn"] if active else 0)
        yb = min(h - 14, 6 + len(rows))
        self.put(yb, 2, "current:", C["title"])
        loc = self.local()
        self.put(yb, 11, "lat %s  lon %s  hdg %s  local %s  EKF %s" % (
            "%.7f" % t.lat if t.lat is not None else "--", "%.7f" % t.lon if t.lon is not None else "--",
            "%.0f" % t.hdg if t.hdg is not None else "--", ("N %+.1f E %+.1f" % loc) if loc else "--",
            "OK" if t.ekf_pos_ok() else "NO POS"))
        g = self.goto
        self.put(yb + 1, 2, "goto:", C["title"])
        if g and t.lat is not None:
            self.put(yb + 1, 11, "%s  dist %.1f m  brg %03.0f  fcu wp_dist %s  tgt_brg %s  %.0f s  mode %s" % (
                g["name"], haversine(t.lat, t.lon, g["lat"], g["lon"]), bearing(t.lat, t.lon, g["lat"], g["lon"]),
                "%d" % t.wp_dist if t.wp_dist is not None else "--",
                "%d" % t.target_bearing if t.target_bearing is not None else "--", now - g["t"], MODE_NAMES.get(t.mode, "?")), C["warn"])
        elif self.pending_goto:
            self.put(yb + 1, 11, "queued: %s (waiting for GUIDED)" % self.pending_goto[0], C["warn"])
        else:
            self.put(yb + 1, 11, "none", C["dim"])
        yc = h - 10
        self.box(yc, 0, 7, w - 1, "COMMANDS (points)")
        rows_c = [("save A", "store the current fix as A (overwrites A)", "here", "print the position"),
                  ("goto A", "GUIDED to A at WP_SPEED", "goto A 0.5", "... at 0.5 m/s"),
                  ("speed 0.5", "change speed during the transit", "stop", "-> HOLD, goto cleared"),
                  ("del A", "remove A", "co A", "local-frame origin at A"),
                  ("", "goto needs armed + 3D fix + EKF; the script enters GUIDED, waits, then sends the target", "", "")]
        for i, (c1, d1, c2, d2) in enumerate(rows_c):
            self.put(yc + 1 + i, 2, "%-11s" % c1, curses.A_BOLD)
            self.put(yc + 1 + i, 14, d1, C["dim"] if c1 else C["warn"])
            if c2:
                self.put(yc + 1 + i, 58, "%-11s" % c2, curses.A_BOLD)
                self.put(yc + 1 + i, 70, d2, C["dim"])

    # -- LOG --
    def draw_log(self, now):
        t, C = self.tel, self.C
        h, w = self.scr.getmaxyx()
        self.draw_state_strip(1, now)
        half = (h - 6) // 2
        self.box(2, 0, half, w - 1, "MESSAGES - FCU STATUSTEXT (newest first)")
        for i, (ts, sev, text, wall) in enumerate(reversed(t.status[-(half - 2):])):
            self.put(3 + i, 2, "%s %-8s %s" % (wall, "[%s]" % self.SEVERITY.get(sev, sev), text),
                     C["bad"] if sev <= 3 else C["warn"] if sev <= 5 else 0)
        y = 2 + half
        self.box(y, 0, h - 4 - y, w - 1, "COMMAND LOG (PgUp/PgDn scroll, newest last)  %s" % (
            ("%s.events" % os.path.basename(self.base)) if self.base else "logging off"))
        n = h - 4 - y - 3
        end = len(self.log) - self.log_scroll
        for i, (ts, text, kind) in enumerate(self.log[max(0, end - n):end]):
            self.put(y + 1 + i, 2, "%6.1fs %s" % (now - ts, text), C["title"] if kind == "cmd" else (C["dim"] if kind == "result" else 0))
        self.put(h - 5, 2, ("REC %s.tlog / .csv / .events" % self.base if self.base else "logging OFF - 'log on [LABEL]' to record")
                 + "   points: %s   CO: %s   keys: PgUp/PgDn scroll, Tab next screen" % (self.args.points, self.args.co),
                 C["bad"] if self.base else C["dim"])

    # -- MAVLINK (inspector, like Mission Planner's MAVLink Inspector) --
    def draw_mavlink(self, now):
        C = self.C
        h, w = self.scr.getmaxyx()
        self.draw_state_strip(1, now)
        rows = []
        total = 0.0
        for k in sorted(self.mstats):
            st = self.mstats[k]
            ts = st["times"]
            hz = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 1 and ts[-1] > ts[0] and now - ts[-1] < 3.0 else 0.0
            total += hz
            try:
                d = st["last"].to_dict()
            except Exception:
                d = {}
            d.pop("mavpackettype", None)
            fields = "  ".join("%s %s" % (f, ("%.4g" % v) if isinstance(v, float) else v)
                               for f, v in d.items() if not isinstance(v, (list, bytes, bytearray)) or len(v) <= 8)
            rows.append((k, hz, st["n"], now - ts[-1] if ts else 0.0, fields))
        self.box(2, 0, h - 5, w - 1, "MAVLINK INSPECTOR  %d types  %.0f msg/s  (PgUp/PgDn scroll; latest values)" % (len(rows), total))
        self.put(3, 2, "%-26s %7s %8s %6s  %s" % ("TYPE", "HZ", "COUNT", "AGE s", "LAST"), C["title"])
        n = h - 8
        self.mav_scroll = max(0, min(self.mav_scroll, max(0, len(rows) - n)))
        for i, (k, hz, cnt, age, fields) in enumerate(rows[self.mav_scroll:self.mav_scroll + n]):
            attr = C["dim"] if age > 3.0 else (C["warn"] if k in ("STATUSTEXT", "BAD_DATA") else 0)
            self.put(4 + i, 2, ("%-26s %7.1f %8d %6.1f  %s" % (k[:26], hz, cnt, age, fields))[:w - 4], attr)

    # -- HELP --
    def draw_help(self, now):
        C = self.C
        h, w = self.scr.getmaxyx()
        self.box(1, 0, h - 4, w - 1, "HELP")
        lines = [
            ("Bursts (mode 20 CUSTOM_MANUAL, armed)", C["title"]),
            ("  surge A [T]     M1 = M3 = A              + = ahead     A in -1..1 (|A| <= %.1f), T <= %.0f s" % (self.args.max_amp, self.args.max_dur), 0),
            ("  sway  A [T]     M2 = F*A, M4 = A  + = hull to starboard; F = sway mix (swaymix on: +1.05 / -0.877, off: 0.957)", 0),
            ("  yaw   A [T]     M1 = +A, M3 = -A         + = bow to starboard (clockwise)", 0),
            ("  motor N A [T]   one motor: 1 PORT, 2 STERN, 3 STBD, 4 BOW", 0),
            ("  burst m1 m2 m3 m4 [T]   raw four-channel burst", 0),
            ("  every burst = 0.3 s zeros, burst, 0.5 s zeros; FCU neutral 250 ms after the last frame", C["dim"]),
            ("", 0),
            ("Modes / arming", C["title"]),
            ("  mode 20|cman|hold|guided|manual|<n>    arm    disarm    stop | x  (cancel burst; GUIDED -> HOLD)", 0),
            ("", 0),
            ("Points, control origin, GUIDED", C["title"]),
            ("  here            print current position         save NAME     store it (points.json)", 0),
            ("  points / del NAME                              goto NAME [V] GUIDED to it, V m/s optional", 0),
            ("  co              set the control origin at the boat (co.json)   co NAME   at a saved point", 0),
            ("  speed V         DO_CHANGE_SPEED (GUIDED WP sub-mode only)", 0),
            ("  path A B [C..] [straight] plan boat->A->B (spline)   pathto A B [V] plan + follow   pathgo [V]", 0),
            ("  carrot = path point --lookahead ahead of the boat, sent as GUIDED target at 2 Hz; set la|step|corridor|endr", C["dim"]),
            ("  swayto A B [V]  GUIDED to A -> mode 20 -> yaw so starboard faces B -> sway along A-B, hdg held, xtrack by surge", 0),
            ("  set swayv 0.25 | kpsi 0.6 | kc 0.15 | yawsign -1 | swaysign -1   (signs: flip if the boat turns/moves the wrong way)", C["dim"]),
            ("", 0),
            ("Other", C["title"]),
            ("  param NAME VAL  write one parameter (disarmed)      help      quit", 0),
            ("  log on [LABEL] | log off | log   record .tlog/.csv/.events (OFF by default; --log [LABEL] starts on)", 0),
            ("  note TEXT       annotate the log (carrot / path type / speed / water state ...)", 0),
            ("  swaycal A T [F] (= sway_calibr) mode 20 -> arm -> log on swaycal_f<F>_a<p|m><A>_<T>s -> sway (M2 = F*A, M4 = A,", 0),
            ("                  F default = sway mix; F > 1 lowers M4 instead) -> summary -> log off; ABORTED runs say why; stop / x cancels", C["dim"]),
            ("  swaymix on|off  per-direction stern factor for sway, swayto, swaycal (set fpos 1.05 | set fneg 0.877); MAIN shows it", 0),
            ("  every burst ends with a summary: hdg drift (deg, deg/s) and mean v_fwd / v_stbd (sway calibration)", C["dim"]),
            ("  screens: MAVLINK = inspector (all message types, Hz, latest values); LOG = messages newest first", C["dim"]),
            ("  Tab / Shift-Tab = next / previous screen, Esc = clear line, Ctrl-L = repaint, Ctrl-C = stop + quit", 0),
            ("  MAIN (120x44 or more) = the shared skeleton with op-tui: ACTION/LINKS | VEHICLE, MISSION | WHERE WE ARE, FLOW, THRUSTERS/RC | NETWORK |", C["dim"]),
            ("  WARNINGS (active conditions, self-clearing; the bar counts them), EVENTS (what happened) | CONSOLE (your commands) | COMMANDS (groups)", C["dim"]),
            ("  MISSION tab: every mission with its stages and how it is selected (Up/Down); smaller terminals get the compact MAIN", C["dim"]),
            ("", 0),
            ("Safety: SF (RC5) E-stop and RC6 MANUAL override on the transmitter beat everything here.", C["warn"]),
            ("", 0),
            ("v4: the global_v3 stack around this tool", C["title"]),
            ("  share           this tool holds the FCU link and re-shares it on udp 14551 (.share file); the mission manager uses udpin:127.0.0.1:14551", 0),
            ("  cv on|off|status|clear   detections from the AGX (local/usv/perception/#, cv_publish.py) -> PerceptionMap -> MAP tab + telemetry/usv/map", 0),
            ("  release / active   hand the FCU to the mission manager / take it back (local/usv/tui/active beat); stop, x, disarm always take it back", 0),
            ("  identity: this tool is MAVLink 200/190, the mission manager 200/191 - acks '(for comp 191)' in the log are its, not ours", C["dim"]),
            ("  map             the browser map (optiprimex/tools/map.html) on this broker's websocket; prints the URL to open on the Mac", 0),
            ("", 0),
            ("Typical pool session", C["title"]),
            ("  mode 20 -> arm -> motor 1 0.2 1 ... motor 4 0.2 1 -> surge 0.3 2 -> sway 0.3 2 -> yaw 0.3 2 -> save A", 0),
            ("  (move) -> save B -> goto A 0.8 -> (watch MAP) -> stop -> goto B 0.5 -> disarm -> quit", 0),
            ("  while typing, the line above the prompt shows the usage of the command you are typing", C["dim"]),
        ]
        for i, (s, a) in enumerate(lines[:h - 6]):
            self.put(2 + i, 2, s, a)

    def run(self, scr):
        self.scr = scr
        self.screen = 0
        self.log_scroll = 0
        self.unicode = (not self.args.ascii) and "utf" in (locale.getpreferredencoding(False) or "").lower()
        self.init_colors()
        try:
            curses.curs_set(1)
        except curses.error:
            pass                    # terminal without cursor-visibility control (e.g. TERM=vt100)
        scr.nodelay(True)
        scr.keypad(True)
        scr.idlok(False)            # no insert/delete-line optimisation: full redraws, nothing to desync over ssh
        if self.args.log is not None:
            self.log_start(self.args.log or None)
        if self.origin.is_set:
            self.event("CO loaded from %s (%s, set %s)" % (self.args.co, self.origin.rec.get("how"), self.origin.rec.get("utc")))
        self.start_links()
        self.event("waiting for heartbeat on %s" % self.args.conn)
        hb = None
        last_draw = 0.0
        while hb is None and not self.quit:
            self.service_share()
            hb = self.m.wait_heartbeat(timeout=0.5)
            self.poll_keys()
            self.draw()
        if self.quit:
            return
        self.connected(hb)
        while not self.quit:
            now = self.step()
            self.poll_keys()
            if now - last_draw >= 0.1:
                self.draw()
                last_draw = now
            time.sleep(0.02)
        if self.action and self.active:
            for _ in range(5):
                self.send_controls([0, 0, 0, 0])
                time.sleep(0.1)

    def start_links(self):
        """The share and the MQTT client; called once before the loop (run() and headless users)."""
        self.event("usv_ui kit v%s  (%s, pid %d)" % (KIT_VERSION, os.path.abspath(__file__), os.getpid()))
        if self.share is not None:
            self.event("sharing the FCU link on %s (tools: %s)" % (self.share.endpoint, self.share.client))
        elif self.share_err:
            self.event("share FAILED: %s" % self.share_err)
        if self.mq is not None:
            self.mq.start()
            self.event("mqtt %s:%d: perception + mm/state in, tui/active beat %s, map out" % (
                self.mq.host, self.mq.port, "ON (usv_ui commands the FCU)" if self.active else "off (handed over)"))
        elif self.mq_err:
            self.event("mqtt off: %s" % self.mq_err)
        if self.cvmap is None:
            self.event("optiprimex not importable - 'cv' unavailable (%s)" % OPTIPRIMEX_ERR)

    def wait_heartbeat(self, timeout=None):
        """Headless users: block (with the share pumped) until the FCU heartbeat, then request streams."""
        t_end = None if timeout is None else time.monotonic() + timeout
        while not self.quit:
            self.service_share()
            hb = self.m.wait_heartbeat(timeout=0.5)
            if hb is not None:
                self.connected(hb)
                return True
            if t_end is not None and time.monotonic() > t_end:
                return False
        return False

    def connected(self, hb):
        self.handle(hb)
        if self.share is not None:
            self.share.from_fcu(hb)
        self.request_streams()
        self.event("connected sys %d comp %d; streams requested; this tool is %d/%d" % (
            self.m.target_system, self.m.target_component, self.args.sysid, OWN_COMPID))
        self._last_trail = 0.0

    def step(self):
        """One iteration of the control loop (no UI): drain the FCU, relay the share, heartbeat, services. Returns now."""
        now = time.monotonic()
        self.service_share()                      # tools -> FCU first: their frames are time-critical too
        for _ in range(200):                      # drain the link
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            self.handle(msg)
            if self.share is not None:
                self.share.from_fcu(msg)
        if now - self.last_hb >= 1.0:
            self.m.mav.heartbeat_send(MAV.MAV_TYPE_ONBOARD_CONTROLLER, MAV.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_hb = now
        self.service_fcu_switch(now)
        self.service_action()
        self.service_path(now)
        self.service_sway(now)
        self.service_dock(now)
        self.service_cal(now)
        if self.pending_goto:
            name, lat, lon, speed, deadline = self.pending_goto
            if self.tel.mode == 15:
                self.pending_goto = None
                self.send_goto(name, lat, lon, speed)
            elif now > deadline:
                self.pending_goto = None
                self.event("goto %s dropped: GUIDED not confirmed (armed without EKF position?)" % name)
        if self.tel.lat is not None and self.tel.fix >= 3 and not self.origin.is_set:
            self.origin.set(self.tel.lat, self.tel.lon, "auto (first fix)")
            self.event("CO set automatically at the first fix - type  co  to move it to the boat later")
        if now - getattr(self, "_last_trail", 0.0) >= 0.5:
            loc = self.local()
            if loc:
                self.trail.append(loc)
                self.trail = self.trail[-240:]
            self._last_trail = now
        if now - self.last_csv >= 0.2:
            self.write_csv()
            self.last_csv = now
        self.service_mqtt(now)
        return now

    def close_links(self):
        if self.fake_proc is not None and self.fake_proc.poll() is None:
            self.fake_proc.terminate()
        if self.mq is not None:
            if self.active:
                self.mq.publish("local/%s/tui/active" % self.args.vehicle, "local", {"active": False, "node": "usv_ui", "t": utc_iso()})
            self.mq.close()
        if self.share is not None:
            self.share.close()

    def poll_keys(self):
        scr = self.scr
        if scr is None:
            return
        while True:
            try:
                ch = scr.getch()
            except curses.error:
                return
            if ch == -1:
                return
            if ch in (10, 13, curses.KEY_ENTER):
                if not self.cmdline.strip() and self.SCREENS[self.screen] == "MISSION" and MISSION_ORDER:
                    task = MISSION_ORDER[self.mission_cursor % len(MISSION_ORDER)]
                    self.event("selection is an OCS command: on the Mac  ocs_cli mission %s  (or start_task %s; %ssim = fake FCU only); here: handover / step / reset" % (task, task, task))
                    continue
                line, self.cmdline = self.cmdline, ""
                self.do_command(line)
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                self.cmdline = self.cmdline[:-1]
            elif ch == curses.KEY_UP and self.SCREENS[self.screen] == "MISSION" and MISSION_ORDER:
                self.mission_cursor = (self.mission_cursor - 1) % len(MISSION_ORDER)
            elif ch == curses.KEY_DOWN and self.SCREENS[self.screen] == "MISSION" and MISSION_ORDER:
                self.mission_cursor = (self.mission_cursor + 1) % len(MISSION_ORDER)
            elif ch == 9:           # Tab: next screen
                self.screen = (self.screen + 1) % len(self.SCREENS)
            elif ch == curses.KEY_BTAB:
                self.screen = (self.screen - 1) % len(self.SCREENS)
            elif ch == curses.KEY_PPAGE:
                if self.SCREENS[self.screen] == "MAVLINK":
                    self.mav_scroll = max(0, self.mav_scroll - 10)
                else:
                    self.log_scroll = min(max(0, len(self.log) - 5), self.log_scroll + 5)
            elif ch == curses.KEY_NPAGE:
                if self.SCREENS[self.screen] == "MAVLINK":
                    self.mav_scroll += 10
                else:
                    self.log_scroll = max(0, self.log_scroll - 5)
            elif ch == 27:          # Esc clears the line
                self.cmdline = ""
            elif ch == 12:          # Ctrl-L: full repaint (garbled terminal over ssh)
                scr.clearok(True)
            elif ch == curses.KEY_RESIZE:
                pass
            elif 32 <= ch < 127:
                self.cmdline += chr(ch)


def resolve_link(conn, baud):
    """FCU link on the Orin hybrid board: uart (default) | usb | eth, or any pymavlink connection string.
    With no link given and rc.sh running, use the link it shares instead of opening the UART a second time."""
    if conn is None:
        try:
            ep, pid = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".share")).read().split()
            os.kill(int(pid), 0)
            print("rc.sh (pid %s) holds the FCU link -> using its share %s" % (pid, ep))
            return ep, baud or 115200
        except (OSError, ValueError):
            conn = "uart"
    name = conn.lower()
    if name == "uart":
        return "/dev/ttyTHS1", baud or 921600
    if name == "usb":
        found = sorted(glob.glob("/dev/serial/by-id/*ArduPilot*")) or sorted(glob.glob("/dev/ttyACM*"))
        if not found:
            raise SystemExit("usb: no ArduPilot USB serial port (FMU USB-C -> Orin USB jumper fitted?)")
        return found[0], baud or 115200
    if name in ("eth", "ethernet"):
        return "udpin:0.0.0.0:14550", baud or 115200
    return conn, baud or 57600


def build_parser():
    ap = argparse.ArgumentParser(description="usv_ui - the boat console (mode 20 bursts, GUIDED goto / path / sway, the FCU link share, cv map, "
                                             "handover to the mission manager, live fake/real FCU switch)",
                                 epilog=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("conn", nargs="?", default=None,
                    help="uart (default, /dev/ttyTHS1 @ 921600; rc.sh share if it runs) | usb | eth"
                         " | or /dev/ttyACM0 | udpin:0.0.0.0:14550 | tcp:host:5760 | udpin:127.0.0.1:14550")
    ap.add_argument("--baud", type=int, default=None,
                    help="serial baud; default 921600 uart, 115200 usb, 57600 other serial ports")
    ap.add_argument("--sysid", type=int, default=200, help="this script's MAVLink system id (CMAN_SYSID on the FCU)")
    ap.add_argument("--amp", type=float, default=0.3, help="default burst amplitude")
    ap.add_argument("--dur", type=float, default=2.0, help="default burst duration s")
    ap.add_argument("--max-amp", type=float, default=0.7, help="burst amplitude limit (0.7 = sway calibration)")
    ap.add_argument("--max-dur", type=float, default=12.0, help="burst duration limit, s")
    ap.add_argument("--swaymix", choices=["on", "off"], default="on",
                    help="calibrated per-direction stern factor for sway / swayto (default on; 'swaymix on|off' while running)")
    ap.add_argument("--sway-fpos", type=float, default=SWAY_F_POS, help="stern factor for sway to starboard (+)")
    ap.add_argument("--sway-fneg", type=float, default=SWAY_F_NEG, help="stern factor for sway to port (-)")
    ap.add_argument("--log", nargs="?", const="", default=None, metavar="LABEL",
                    help="start with logging ON (optional file label); default off, toggle with 'log on|off'")
    ap.add_argument("--log-dir", default="logs")
    ap.add_argument("--points", default="points.json")
    ap.add_argument("--co", default="co.json", help="control-origin file (lat/lon of the local frame)")
    ap.add_argument("--ascii", action="store_true", help="ASCII map symbols (no unicode arrows)")
    ap.add_argument("--lookahead", type=float, default=3.0, help="path carrot distance ahead of the boat, m")
    ap.add_argument("--path-step", type=float, default=0.5, help="path point spacing, m")
    ap.add_argument("--corridor", type=float, default=6.0, help="abort the path when |xte| exceeds this, m")
    ap.add_argument("--end-radius", type=float, default=1.0, help="path complete when within this of the end, m")
    ap.add_argument("--sway-v", type=float, default=0.25, help="sway command (0..1) for swayto")
    ap.add_argument("--sway-kpsi", type=float, default=0.6, help="yaw command per 45 deg heading error (cap 0.3)")
    ap.add_argument("--sway-kc", type=float, default=0.15, help="surge command per metre of cross-track (cap 0.25)")
    ap.add_argument("--sway-corridor", type=float, default=3.0, help="abort swayto when |cross-track| exceeds this, m")
    ap.add_argument("--share", default="udpout:127.0.0.1:14551",
                    help="re-share the FCU link here for the mission manager (udpin side); '' = off; off automatically when this tool itself runs on a share")
    ap.add_argument("--mqtt", default="127.0.0.1:1884", help="boat broker host:port for perception, map, tui/active ('' = off)")
    ap.add_argument("--vehicle", default="usv", help="topic token (local/<vehicle>/...)")
    ap.add_argument("--vehicle-id", default="USV1", help="envelope vehicle_id")
    ap.add_argument("--cam-forward", type=float, default=0.0, help="camera offset ahead of the GPS/EKF reference, m (map geolocation)")
    ap.add_argument("--cam-right", type=float, default=0.0, help="camera offset to starboard, m")
    ap.add_argument("--cam-yaw", type=float, default=0.0, help="camera yaw from the bow, deg (+ starboard)")
    ap.add_argument("--handover", "--released", dest="handover", action="store_true",
                    help="start handed over (no tui/active beat): the mission manager has the FCU until 'takeover'")
    ap.add_argument("--ws", type=int, default=int(os.environ.get("BOAT_WS", "9002")), help="the boat broker's websocket port for the 'map' command")
    return ap


def main():
    args = build_parser().parse_args()
    args.conn, args.baud = resolve_link(args.conn, args.baud)
    try:
        locale.setlocale(locale.LC_ALL, "")
    except locale.Error:
        pass
    try:
        app = App(args)
    except Exception as e:
        if "Permission denied" in str(e):
            raise SystemExit("cannot open %s: permission denied - this login is not in the 'dialout' group yet.\n"
                             "log out and back in (or run: newgrp dialout), then start again." % args.conn)
        raise SystemExit("cannot open %s: %s" % (args.conn, e))
    try:
        curses.wrapper(app.run)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if app.active:
                app.stop("exit")
                for _ in range(5):
                    if app.tel.mode == 20:
                        app.send_controls([0, 0, 0, 0])
                    time.sleep(0.1)
            else:
                print("handed over: the FCU stays with the mission manager (not touched on exit)")
        except Exception:
            pass
        app.close_links()
        if app.base:
            app.log_stop()
        for b in app.log_files:
            print("logs: %s.tlog / .csv / .events" % b)
        if not app.log_files:
            print("logging was off - nothing written (use --log or 'log on')")
    return 0


if __name__ == "__main__":
    sys.exit(main())
