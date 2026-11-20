#!/usr/bin/env python3
"""
RunStart mission: what the drone flies when RoboCommand sends RunStart (the
drone GUI starts it by itself, see heartbeat_monitor.py).

Arms, takes off to 5 m above home (AGL), hovers 60 s there, then flies
mission2's lawnmower (10 m) at 3 m/s, returns home and lands.

It runs lawnmower_mission.py's own code, so everything else is the same as
the other missions: fence read from the FCU and checked (altitude must be
2 m+ under FENCE_ALT_MAX), RC mode switch in GUIDED, RC takeover stops the
script at once, GUIDED + waypoints cleared after it lands.

Usage (every lawnmower_mission.py option works, and overrides these):
    python3 runstart_mission.py --upload-only   # plan + upload, no flying
    python3 runstart_mission.py                 # fly it (asks to confirm)
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lawnmower_mission  # noqa: E402

HOVER_ALTITUDE_M = 5.0
HOVER_S = 60.0
ALTITUDE_M = 10.0  # mission2's
SPEED_MS = 3.0

if __name__ == "__main__":
    lawnmower_mission.main({"hover_altitude": HOVER_ALTITUDE_M, "hover": HOVER_S, "altitude": ALTITUDE_M, "speed": SPEED_MS})
