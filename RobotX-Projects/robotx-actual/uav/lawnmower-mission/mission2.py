#!/usr/bin/env python3
"""
Mission 2: takes off to 5 m above home (AGL), hovers 15 s there, then flies
the lawnmower at 10 m, 3 m/s, returns home and lands. The same flight as
runstart_mission.py, started by hand, with the same CV start signal
(runstart_mission.py's, with "mission": "mission2" - see README.md).

It runs lawnmower_mission.py's own code, so everything is the same as the
main mission: fence read from the FCU and checked (altitude must be 2 m+
under FENCE_ALT_MAX), RC mode switch in GUIDED, RC takeover stops the script
at once, GUIDED + waypoints cleared after it lands.

Usage (every lawnmower_mission.py option works, and overrides these):
    python3 mission2.py --upload-only   # plan + upload, no flying
    python3 mission2.py                 # fly it (asks to confirm)
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lawnmower_mission  # noqa: E402
import runstart_mission  # noqa: E402

HOVER_ALTITUDE_M = 5.0
HOVER_S = 15.0
ALTITUDE_M = 10.0
SPEED_MS = 3.0

if __name__ == "__main__":
    runstart_mission.CV_MISSION = "mission2"
    lawnmower_mission.watch_mission = runstart_mission.watch_mission_with_cv
    lawnmower_mission.main({"hover_altitude": HOVER_ALTITUDE_M, "hover": HOVER_S, "altitude": ALTITUDE_M, "speed": SPEED_MS})
