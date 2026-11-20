"""fcu_reboot: reboot the Pixhawk (or the fake FCU) through usv_ui's share - one shot, on the NX (v0.1.28, ROADMAP WP8a).

  python3 -m optiprimex.tools.fcu_reboot [--via udpout:127.0.0.1:14551] [--wait 15] [--dry-run]

usv_ui holds the serial port and shares it on udp 14551 (one program per serial port); this connects to the share as sysid 200
comp 192 (usv_ui is 190, the mission manager 191), waits for the autopilot's HEARTBEAT, REFUSES when it is armed, sends
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (param1 = 1, reboot the autopilot), waits for the COMMAND_ACK and then for the heartbeat to come
back. Exit 0 = accepted (and back, unless --wait 0), 1 = refused / denied / no answer, 2 = no autopilot seen. The OCS page's SYSTEM
tab runs it over ssh; by hand on the NX from the tree root. Never arms, never changes a mode.
"""
from __future__ import annotations

import argparse
import sys
import time

COMP_ID = 192


def _autopilot_hb(m, timeout_s: float):
    from pymavlink import mavutil
    t_end = time.monotonic() + timeout_s
    while time.monotonic() < t_end:
        msg = m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.type == mavutil.mavlink.MAV_TYPE_GCS or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            continue                                  # usv_ui / the mission manager / a GCS, not the autopilot
        return msg
    return None


def reboot(via: str, wait_s: float = 15.0, dry_run: bool = False, out=print) -> int:
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(via, source_system=200, source_component=COMP_ID)
    try:
        sent_at = time.monotonic()
        while time.monotonic() - sent_at < 3.0:       # a udpout share needs a first packet before it answers
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            hb = _autopilot_hb(m, 1.0)
            if hb is not None:
                break
        else:
            hb = None
        if hb is None:
            out(f"fcu_reboot: no autopilot heartbeat on {via} (is usv_ui running and sharing?)")
            return 2
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
        out(f"fcu_reboot: autopilot {sysid}/{compid} on {via}, mode {hb.custom_mode}, {'ARMED' if armed else 'disarmed'}")
        if armed:
            out("fcu_reboot: REFUSED - the FCU is armed (disarm from the transmitter / usv_ui first)")
            return 1
        if dry_run:
            out("fcu_reboot: dry run - nothing sent")
            return 0
        m.mav.command_long_send(sysid, compid, mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        t_end = time.monotonic() + 3.0
        res = None
        while time.monotonic() < t_end:
            ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if ack is not None and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
                res = int(ack.result)
                break
        if res != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            name = {None: "no answer"}.get(res, mavutil.mavlink.enums["MAV_RESULT"][res].name if res in mavutil.mavlink.enums["MAV_RESULT"] else str(res))
            out(f"fcu_reboot: the FCU did not accept the reboot ({name})")
            return 1
        out("fcu_reboot: ACCEPTED - the FCU reboots (the stream stops for a few seconds; usv_ui shows the link drop)")
        if wait_s <= 0:
            return 0
        time.sleep(1.5)
        back = _autopilot_hb(m, wait_s)
        out("fcu_reboot: the FCU is back" if back is not None else f"fcu_reboot: no heartbeat {wait_s:.0f} s after the reboot - look at usv_ui")
        return 0 if back is not None else 1
    finally:
        m.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--via", default="udpout:127.0.0.1:14551", help="usv_ui's share (default udpout:127.0.0.1:14551)")
    ap.add_argument("--wait", type=float, default=15.0, help="seconds to wait for the heartbeat to come back (0 = do not wait)")
    ap.add_argument("--dry-run", action="store_true", help="connect and check, send nothing")
    a = ap.parse_args(argv)
    return reboot(a.via, a.wait, a.dry_run, out=lambda s: print(s, flush=True))


if __name__ == "__main__":
    sys.exit(main())
