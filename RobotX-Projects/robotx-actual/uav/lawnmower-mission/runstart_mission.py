#!/usr/bin/env python3
"""
RunStart mission: what the drone flies when RoboCommand sends RunStart (the
drone GUI starts it by itself, see heartbeat_monitor.py).

Arms, takes off to 5 m above home (AGL), hovers 15 s there, then flies
mission2's lawnmower (10 m) at 3 m/s, returns home and lands.

It runs lawnmower_mission.py's own code, so everything else is the same as
the other missions: fence read from the FCU and checked (altitude must be
2 m+ under FENCE_ALT_MAX), RC mode switch in GUIDED, RC takeover stops the
script at once, GUIDED + waypoints cleared after it lands.

CV start signal: from the moment ArduPilot has the mission (AUTO, start of the
hover) this sends one UDP JSON message a second to CV_TOPIC, and active=false
when the script stops watching (landed, RC takeover, 'mission stop'):
    {"mission": "runstart", "active": true, "phase": "hover"}
phase: hover -> lawnmower (from leaving the hover until home) -> landing -> done.
The CV program starts on active=true and stops on active=false, or if nothing
has arrived for 3 s (this script died). See README.md.

Usage (every lawnmower_mission.py option works, and overrides these):
    python3 runstart_mission.py --upload-only   # plan + upload, no flying
    python3 runstart_mission.py                 # fly it (asks to confirm)
"""

import json
import socket
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lawnmower_mission  # noqa: E402

HOVER_ALTITUDE_M = 5.0
HOVER_S = 15.0
ALTITUDE_M = 10.0  # mission2's
SPEED_MS = 3.0

CV_TOPIC = ("127.0.0.1", 14660)
CV_MISSION = "runstart"  # mission2.py sets its own name
CV_PERIOD_S = 1.0

_watch_mission = lawnmower_mission.watch_mission


def watch_mission_with_cv(link, messages, land_seq, say, switch=None):
    """lawnmower_mission.watch_mission, with the CV signal on while it runs.
    Only reads the mission progress - nothing here touches the flight."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop = threading.Event()

    def send(active, phase):
        try:
            sock.sendto(json.dumps({"mission": CV_MISSION, "active": active, "phase": phase}).encode("utf-8"), CV_TOPIC)
        except OSError:
            pass  # a problem with the CV signal must never affect the mission

    def publish():
        while True:
            seq = link.mission_current()
            if seq is None or seq <= lawnmower_mission.HOVER_SEQ:
                send(True, "hover")
            else:
                send(True, "landing" if seq >= land_seq else "lawnmower")
            if stop.wait(CV_PERIOD_S):
                return

    say(f"Mission: CV signal ON (UDP {CV_TOPIC[0]}:{CV_TOPIC[1]})")
    thread = threading.Thread(target=publish, daemon=True)
    thread.start()
    try:
        return _watch_mission(link, messages, land_seq, say, switch)
    finally:
        stop.set()
        thread.join(timeout=2.0)
        for _ in range(3):  # UDP can drop one
            send(False, "done")
        sock.close()
        say("Mission: CV signal OFF")


if __name__ == "__main__":
    lawnmower_mission.watch_mission = watch_mission_with_cv
    lawnmower_mission.main({"hover_altitude": HOVER_ALTITUDE_M, "hover": HOVER_S, "altitude": ALTITUDE_M, "speed": SPEED_MS})
