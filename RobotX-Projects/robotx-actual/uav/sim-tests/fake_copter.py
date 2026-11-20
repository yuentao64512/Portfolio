"""Minimal fake ArduCopter for testing lawnmower_mission.py. Not a flight sim:
it just speaks enough MAVLink (params, fence/mission protocol, arm, takeoff,
mode changes, mission progress) for the script's logic to run end to end.

usage: fake_copter.py <port> <scenario>   scenario: ok | takeover | rc_loiter | fence_off
"""
import os, sys, time, math
os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil

port, scenario = int(sys.argv[1]), sys.argv[2]
m = mavutil.mavlink
fcu = mavutil.mavlink_connection(f"udpout:127.0.0.1:{port}", source_system=1, source_component=1)
L = fcu.mav

MODES = {v: k for k, v in mavutil.mode_mapping_acm.items()}
params = {"FENCE_ENABLE": 0 if scenario == "fence_off" else 1, "FENCE_TYPE": 2 if scenario == "listener" else 7, "FENCE_ALT_MAX": 16 if scenario == "lowalt" else 20 if "alt20" in scenario else 100, "FENCE_ACTION": 1, "FENCE_RADIUS": 300,
          "FLTMODE_CH": 5, "FLTMODE1": 0, "FLTMODE2": 2, "FLTMODE3": 5, "FLTMODE4": 4, "FLTMODE5": 3, "FLTMODE6": 6}
rc5 = 1300 if scenario in ("rc_loiter", "rc_mismatch") else 1555  # slot 2 (ALT_HOLD) vs slot 4 (GUIDED)
fence = [(1.412671, 103.913356), (1.412380, 103.913572), (1.412481, 103.913768), (1.412709, 103.913545)]
INC, EXC_CIRCLE = m.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION, m.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION
fence_items = [(INC, len(fence), la, lo) for la, lo in fence]
if scenario == "lshape":
    import math
    o = lambda x, y: (1.41 + y / 111320, 103.91 + x / (111320 * math.cos(math.radians(1.41))))
    lpts = [o(*q) for q in [(0, 0), (40, 0), (40, 15), (15, 15), (15, 40), (0, 40)]]
    fence_items = [(INC, len(lpts), la, lo) for la, lo in lpts] + [(EXC_CIRCLE, 1.5, *o(8, 8))]
    home = o(36, 4)
home = (1.412560, 103.913560) if scenario != "lshape" else home
mode = "ALT_HOLD" if scenario == "rc_loiter" else "GUIDED"
armed, alt, target_alt = (True, 5.0, 5.0) if scenario.startswith("hover") else (False, 0.0, 0.0)
mission, upload, cur = [], None, 0
auto_t = 0.0
taken_over = False
log = lambda *a: print("[fcu]", *a, flush=True)
rc_flip_at, fltmode6_reads = None, 0  # rcflip_* scenarios: when the pilot flipped the RC switch to LOITER

def send_rc(now):
    L.rc_channels_send(int(now * 1000) % 2**32, 8, 1500, 1500, 1000, 1500, rc5, 1500, 1500, 1500, *([65535] * 10), 200)

def flip_rc(reason):
    global rc5, rc_flip_at
    if rc_flip_at is None:
        rc5, rc_flip_at = 1425, time.time()  # slot 3 = LOITER
        log(f"RC switch flipped to LOITER ({reason}) - reported mode follows 1.5 s later")
        send_rc(time.time())

REQ = (255, 0)  # who sent the message being handled - ArduPilot addresses its replies to them
OLD_FENCE = [(INC, 6, 1.2809652, 103.8548424), (INC, 6, 1.2808457, 103.8550331), (INC, 6, 1.2810752, 103.8551770),
             (INC, 6, 1.281174598, 103.855239379), (INC, 6, 1.281294119, 103.855048646), (INC, 6, 1.2811947, 103.8549863)]

def item_msg(seq, mtype, items=None, to=None):
    if seq >= len(items or (fence_items if mtype == m.MAV_MISSION_TYPE_FENCE else mission)):
        L.mission_ack_send(*(to or REQ), m.MAV_MISSION_INVALID_SEQUENCE, mtype)  # what ArduPilot does
        return
    if mtype == m.MAV_MISSION_TYPE_FENCE:
        cmd, p1, lat, lon = (items or fence_items)[seq]
        if "badverify" in scenario and not globals().get("mp_wrote"):
            lat += 2e-5  # the read-back doesn't match what was uploaded (verify MISMATCH)
        L.mission_item_int_send(*(to or REQ), seq, m.MAV_FRAME_GLOBAL, cmd, 0, 1,
                                p1, 0, 0, 0, int(lat * 1e7), int(lon * 1e7), 0, mtype)
    else:
        it = mission[seq]
        L.mission_item_int_send(*(to or REQ), seq, it.frame, it.command, 0, 1, it.param1, it.param2, it.param3, it.param4,
                                it.x, it.y, it.z, mtype)

MP_FENCE = [(1.4122, 103.9132), (1.4128, 103.9132), (1.4128, 103.9138), (1.4122, 103.9138)]
MP_FENCE_OUT = [(1.4122, 103.9132), (1.4140, 103.9132), (1.4128, 103.9138), (1.4122, 103.9138)]
last_hb = last_fast = 0
t0 = time.time()
while True:
    now = time.time()
    # mpfence_* scenarios: at t=12 s Mission Planner (sys 255 / comp 190) writes a new fence; the FCU's ACK goes to it
    if scenario.startswith("mpfence") and now - t0 > 14 and not globals().get("mp_done"):
        mp_done = True
        if scenario == "mpfence_armed":
            armed = True
        pts = MP_FENCE_OUT if scenario.startswith("mpfence_out") else MP_FENCE
        fence_items = [(INC, len(pts), la, lo) for la, lo in pts]
        if scenario == "mpfence_two":
            fence_items += [(INC, 3, la, lo) for la, lo in MP_FENCE[:3]]
        L.mission_ack_send(255, 190, m.MAV_MISSION_ACCEPTED, m.MAV_MISSION_TYPE_FENCE)
        log(f"Mission Planner wrote a fence: {len(fence_items)} items")
    if scenario.startswith("mpalt") and now - t0 > 12 and not globals().get("mpalt_done"):
        mpalt_done = True
        params["FENCE_ALT_MAX"] = 25
        L.param_value_send(b"FENCE_ALT_MAX", 25.0, m.MAV_PARAM_TYPE_REAL32, len(params), 0)
        log("Mission Planner set FENCE_ALT_MAX = 25")
    # mpreplot_*: Mission Planner removes the fence (Write of an empty fence) at t=9 s, then writes a new one at t=13 s
    if scenario.startswith("mpreplot") and now - t0 > 9 and not globals().get("mpclear_done"):
        mpclear_done = True
        fence_items = []
        L.mission_ack_send(255, 190, m.MAV_MISSION_ACCEPTED, m.MAV_MISSION_TYPE_FENCE)
        log("Mission Planner removed the fence: 0 items")
        mp_wrote = True
    # guiclear: the drone GUI's 'fence clear' at t=12 s (MISSION_CLEAR_ALL from the GUI; the ACK goes to it)
    if scenario == "guiclear" and now - t0 > 12 and not globals().get("guiclear_done"):
        guiclear_done = True
        fence_items = []
        L.mission_ack_send(255, 0, m.MAV_MISSION_ACCEPTED, m.MAV_MISSION_TYPE_FENCE)
        log("GUI fence clear: 0 items")
    if scenario.startswith("mpreplot") and now - t0 > 13 and not globals().get("mpreplot_done"):
        mpreplot_done = True
        fence_items = [(INC, len(MP_FENCE), la, lo) for la, lo in MP_FENCE]
        L.mission_ack_send(255, 190, m.MAV_MISSION_ACCEPTED, m.MAV_MISSION_TYPE_FENCE)
        log(f"Mission Planner wrote a fence: {len(fence_items)} items")
    # mpaltseq_*: Mission Planner sets Max Alt 60 at t=14 s (too high), then 30 at t=22 s
    for _t, _v in ((14, 60), (22, 30)):
        if scenario.startswith("mpaltseq") and now - t0 > _t and not globals().get(f"mpalt_{_t}"):
            globals()[f"mpalt_{_t}"] = True
            params["FENCE_ALT_MAX"] = _v
            L.param_value_send(b"FENCE_ALT_MAX", float(_v), m.MAV_PARAM_TYPE_REAL32, len(params), 0)
            log(f"Mission Planner set FENCE_ALT_MAX = {_v}")
    # mpfix_*: Mission Planner writes a fence OUTSIDE the course at t=14 s, then a good one at t=20 s
    for _t, _pts in ((14, "out"), (20, "in")):
        if scenario.startswith("mpfix") and now - t0 > _t and not globals().get(f"mpfix_{_t}"):
            globals()[f"mpfix_{_t}"] = True
            _p = MP_FENCE_OUT if _pts == "out" else MP_FENCE
            fence_items = [(INC, len(_p), la, lo) for la, lo in _p]
            L.mission_ack_send(255, 190, m.MAV_MISSION_ACCEPTED, m.MAV_MISSION_TYPE_FENCE)
            log(f"Mission Planner wrote a fence ({_pts}): {len(fence_items)} items")
    # mpsilent_*: Mission Planner on another link (no reply through MAVProxy) writes a good fence at t=20 s;
    # mpsilentfix_* first writes an OUTSIDE fence through MAVProxy at t=14 s (rejected -> cleared)
    if scenario.startswith("mpsilentfix") and now - t0 > 14 and not globals().get("mps_14"):
        mps_14 = True
        fence_items = [(INC, len(MP_FENCE_OUT), la, lo) for la, lo in MP_FENCE_OUT]
        L.mission_ack_send(255, 190, m.MAV_MISSION_ACCEPTED, m.MAV_MISSION_TYPE_FENCE)
        log("Mission Planner wrote a fence (outside, via MAVProxy)")
    if scenario.startswith("mpsilent") and now - t0 > 20 and not globals().get("mps_20"):
        mps_20 = True
        fence_items = [(INC, len(MP_FENCE), la, lo) for la, lo in MP_FENCE]
        log("Mission Planner wrote a fence (inside, over ANOTHER link - no reply through MAVProxy)")
    if scenario == "mpfence_armed" and now - t0 > 20 and armed:
        armed = False
        log("landed + disarmed")
    if now - last_hb > 0.5:
        base = m.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (m.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0)
        L.heartbeat_send(m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_ARDUPILOTMEGA, base, MODES[mode], 4)
        L.gps_raw_int_send(0, 3, int(home[0] * 1e7), int(home[1] * 1e7), int(alt * 1000), 100, 100, 0, 0, 12, alt_ellipsoid=int(alt * 1000) + 8000)
        L.attitude_send(0, 0.05, -0.02, 1.0, 0, 0, 0)
        if scenario == 'hoverland' and now - t0 > 14 and armed:
            armed, alt, target_alt = False, 0.0, 0.0; log('landed + disarmed')
        if scenario == 'hb':
            el = now - t0
            if 6 < el < 12 and mode != 'AUTO': mode, armed, target_alt = 'AUTO', True, 5.0; log('-> AUTO, armed, climbing')
            if el >= 12 and mode != 'LOITER': mode = 'LOITER'; log('-> LOITER (RC takeover)')
        L.mission_current_send(cur)
        last_hb = now
    if now - last_fast > 0.1:
        send_rc(now)
        if scenario == "rcflip_climb" and armed and alt >= 2.0:
            flip_rc("climbing")
        if rc_flip_at is not None and mode != "LOITER" and now - rc_flip_at > 1.5:
            mode = "LOITER"
            log("mode -> LOITER (from the RC switch)")
        # crude physics
        if armed and mode in ("GUIDED", "AUTO") and alt < target_alt:
            alt = min(target_alt, alt + 0.4)
        if mode == "AUTO" and armed and mission:
            if now - auto_t > 0.4:
                auto_t = now
                if mission[cur].command == m.MAV_CMD_NAV_LAND:
                    target_alt = 0.0
                    alt = max(0.0, alt - 1.0)
                    if alt == 0.0:
                        armed = False
                        log("landed + disarmed")
                elif cur < len(mission) - 1:
                    L.mission_item_reached_send(cur)
                    cur += 1
                    if scenario == "rcflip_auto" and cur == 6:
                        flip_rc("flying the legs in AUTO")
                    if scenario == "takeover" and cur == 8 and not taken_over:
                        mode, taken_over = "LOITER", True
                        log("RC flipped to LOITER")
        L.global_position_int_send(int((now - t0) * 1000), int(home[0] * 1e7), int(home[1] * 1e7), int(alt * 1000), int(alt * 1000), 0 if scenario.startswith("hover") else 120, 0 if scenario.startswith("hover") else -40, 0, 9000)
        tb = int((now - t0) * 1000)
        L.local_position_ned_send(tb, 3.0 + (now - t0) * 0.5, -1.5, -alt, 0.5, 0.0, 0.0)
        L.vfr_hud_send(0, 0.5, 90, 35, alt, 0.0)
        L.scaled_imu_send(tb, 12, -8, -1000, 1, 2, 3, 0, 0, 0)
        L.attitude_send(tb, math.radians(5 * math.sin(now - t0)), math.radians(-3), math.radians(90), 0.1, 0.0, 0.0)
        last_fast = now

    try:
        msg = fcu.recv_match(blocking=True, timeout=0.02)
    except ConnectionResetError:
        continue
    if msg is None:
        continue
    t = msg.get_type()
    REQ = (msg.get_srcSystem(), msg.get_srcComponent())
    if rc_flip_at is not None and (t == "MISSION_SET_CURRENT" or (t == "COMMAND_LONG" and msg.command in (
            m.MAV_CMD_COMPONENT_ARM_DISARM, m.MAV_CMD_NAV_TAKEOFF, m.MAV_CMD_DO_SET_MODE))):
        log(f"OVERRIDE: script sent {t} {getattr(msg, 'command', '')} {time.time() - rc_flip_at:.2f} s after the RC takeover")
    if t == "TIMESYNC" and msg.tc1 == 0 and not __import__("os").environ.get("NO_TIMESYNC"):
        time.sleep(0.002)
        L.timesync_send(int((time.time() - t0) * 1e9), msg.ts1)
    if t == "PARAM_REQUEST_READ":
        name = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id.decode().rstrip("\x00")
        if scenario == "rcflip_prearm" and name == "FLTMODE6":
            fltmode6_reads += 1
            if fltmode6_reads == 2:
                flip_rc("after the preflight check, before arming")
        if name in params:
            L.param_value_send(name.encode(), float(params[name]), m.MAV_PARAM_TYPE_REAL32, len(params), 0)
    elif t == "MISSION_REQUEST_LIST":
        if scenario == "otherreader" and msg.mission_type == m.MAV_MISSION_TYPE_FENCE:
            # Mission Planner (255/190) is reading the fence at the same moment - the
            # FCU's replies to it reach every MAVProxy output too
            L.mission_count_send(255, 190, len(OLD_FENCE), msg.mission_type)
        L.mission_count_send(*REQ, len(fence_items) if msg.mission_type == m.MAV_MISSION_TYPE_FENCE else len(mission), msg.mission_type)
    elif t == "MISSION_REQUEST_INT" and upload is None:
        if scenario == "otherreader" and msg.mission_type == m.MAV_MISSION_TYPE_FENCE and msg.seq < len(OLD_FENCE):
            item_msg(msg.seq, msg.mission_type, OLD_FENCE, (255, 190))
        item_msg(msg.seq, msg.mission_type)
    elif t == "MISSION_COUNT":
        upload = [None] * msg.count
        L.mission_request_int_send(*REQ, 0, msg.mission_type)
    elif t == "MISSION_ITEM_INT" and upload is not None:
        upload[msg.seq] = msg
        if msg.seq + 1 < len(upload):
            L.mission_request_int_send(*REQ, msg.seq + 1, msg.mission_type)
        else:
            if msg.mission_type == m.MAV_MISSION_TYPE_FENCE:
                fence_items = [(it.command, it.param1, it.x / 1e7, it.y / 1e7) for it in upload]
                upload = None
                log(f"fence stored: {len(fence_items)} vertices")
            else:
                mission, upload = upload, None
                log(f"mission stored: {len(mission)} items")
            L.mission_ack_send(*REQ, m.MAV_MISSION_ACCEPTED, msg.mission_type)
    elif t == "PARAM_SET":
        name = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) else msg.param_id.decode().rstrip("\x00")
        params[name] = msg.param_value
        log(f"param {name} = {msg.param_value:g}")
        L.param_value_send(name.encode(), float(msg.param_value), m.MAV_PARAM_TYPE_REAL32, len(params), 0)
    elif t == "MISSION_CLEAR_ALL":
        mtype = getattr(msg, "mission_type", 0)
        if scenario == "refuseclear":
            L.mission_ack_send(*REQ, m.MAV_MISSION_DENIED, mtype)
        elif scenario != "noclearack":
            if mtype == m.MAV_MISSION_TYPE_FENCE:
                fence_items = []
            elif mtype == m.MAV_MISSION_TYPE_MISSION:
                mission = []
            log(f"mission clear (type {mtype}) - mission {len(mission)} items, fence {len(fence_items)} vertices")
            L.mission_ack_send(*REQ, m.MAV_MISSION_ACCEPTED, mtype)
    elif t == "MISSION_SET_CURRENT":
        if scenario == "rcflip_preauto":
            flip_rc("just before the switch to AUTO")
        cur = msg.seq
        L.mission_current_send(cur)
    elif t == "COMMAND_LONG":
        result = m.MAV_RESULT_ACCEPTED
        if msg.command == m.MAV_CMD_COMPONENT_ARM_DISARM:
            armed = msg.param1 == 1
            log("armed" if armed else "disarmed")
        elif msg.command == m.MAV_CMD_NAV_TAKEOFF:
            if armed and mode == "GUIDED":
                target_alt = msg.param7
                log(f"takeoff to {target_alt}")
            else:
                result = m.MAV_RESULT_FAILED
        elif msg.command == m.MAV_CMD_DO_SET_MODE:
            mode = {v: k for k, v in MODES.items()}[int(msg.param2)]
            auto_t = time.time()
            log("mode ->", mode)
        L.command_ack_send(msg.command, result)
