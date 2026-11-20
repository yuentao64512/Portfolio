#!/usr/bin/env python3
"""Verify the boat sees Gazebo images in Core, Advanced, and Disruptive."""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

from robocommand import topics
from robocommand.bus import JsonBus


def main() -> int:
    package = Path(__file__).resolve().parents[1]
    process = subprocess.Popen(
        [sys.executable, "-m", "robocommand.boat_agent", "--config", str(package / "config/task1_world.yaml")],
        cwd=package,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
    )
    bus = JsonBus()
    statuses: list[dict] = []
    bus.subscribe(topics.STATUS_USV, statuses.append)
    try:
        for tier in ("core", "advanced", "disruptive"):
            bus.publish_reliable(topics.TEAM_GO, bus.envelope("sim_cv_smoke", tier=tier, run_id=f"smoke-{tier}"))
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError("Boat agent exited unexpectedly")
                valid = [
                    msg for msg in statuses
                    if msg.get("state") == "AUTO"
                    and msg.get("tier") == tier
                    and msg.get("cv_source") == "gazebo"
                    and int(msg.get("cv_frames", 0)) > 0
                    and (msg.get("stage") == "T1_CORE_SIM_CV" if tier == "core"
                         else msg.get("stage") == "WAIT_UAV_MAP")
                ]
                if valid:
                    msg = valid[-1]
                    print(f"{tier} stage={msg['stage']} sim_cv_frames={msg['cv_frames']} markers={msg['cv_detections']}")
                    break
                time.sleep(0.2)
            else:
                raise RuntimeError(f"No simulated CV status for {tier}: {statuses[-2:]}")
            statuses.clear()
            bus.publish_reliable(topics.CONTROL_STOP, bus.envelope("sim_cv_smoke", command="HOLD"))
            time.sleep(0.5)
        return 0
    finally:
        bus.publish_reliable(topics.CONTROL_STOP, bus.envelope("sim_cv_smoke", command="HOLD"))
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)


if __name__ == "__main__":
    raise SystemExit(main())
