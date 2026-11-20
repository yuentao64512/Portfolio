#!/usr/bin/env python3
"""
Mission 2: the lawnmower mission at 10 m above home (AGL), 5 m/s.

It runs lawnmower_mission.py's own code, so everything is the same as the
main mission: fence read from the FCU and checked (altitude must be 2 m+
under FENCE_ALT_MAX), RC mode switch in GUIDED, RC takeover stops the script
at once, 30 s hover, GUIDED + waypoints cleared after it lands. Only the
altitude and speed defaults differ.

Usage (every lawnmower_mission.py option works, and overrides these):
    python3 mission2.py --upload-only   # plan + upload, no flying
    python3 mission2.py                 # fly it (asks to confirm)
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import lawnmower_mission  # noqa: E402

ALTITUDE_M = 10.0
SPEED_MS = 5.0

if __name__ == "__main__":
    lawnmower_mission.main({"altitude": ALTITUDE_M, "speed": SPEED_MS})
