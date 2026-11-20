#!/usr/bin/env python3
"""Check simulated UAV-map to boat-route to Gazebo-thruster handoff."""

from __future__ import annotations

import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

from robocommand import topics
from robocommand.bus import JsonBus
from robocommand.contract import xy_to_latlon


def main() -> int:
    if not os.environ.get("GZ_PARTITION", "").startswith("sim_boat_drive_test_"):
        raise RuntimeError("Refusing to test outside isolated Gazebo partition")
    pkg = Path(__file__).resolve().parents[1]
    cfg = yaml.safe_load((pkg / "config/task1_world.yaml").read_text())
    processes = [
        subprocess.Popen([sys.executable, "-m", "robocommand.boat_agent", "--config", str(pkg / "config/task1_world.yaml")], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT),
        subprocess.Popen([sys.executable, "-m", "robocommand.sim_boat_driver"], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT),
    ]
    bus = JsonBus()
    boat_status: list[dict] = []
    drive_status: list[dict] = []
    bus.subscribe(topics.STATUS_USV, boat_status.append)
    bus.subscribe(topics.USV_DRIVE_STATUS, drive_status.append)
    try:
        time.sleep(2)
        run_id = "handoff-test"
        bus.publish_reliable(topics.TEAM_GO, bus.envelope("test", tier="advanced", run_id=run_id))
        items = []
        for item in cfg["ground_truth"]:
            lat, lon = xy_to_latlon(item["x"], item["y"], cfg["origin"]["lat"], cfg["origin"]["lon"])
            items.append({"id": item["id"], "lat": lat, "lon": lon, "class": item["class"],
                          "colour": item["colour"], "lit": item["lit"], "conf": 0.99, "sigma_m": 0.1})
        report = bus.envelope("test", items=items, scan={"state": "complete", "coverage_pct": 100, "n_items": len(items)})
        deadline = time.monotonic() + 45
        handoff = False
        while time.monotonic() < deadline:
            bus.publish(topics.USV_UAV_REPORT, report)
            if any(m.get("stage") == "ROUTE_FOLLOWING" and m.get("route_points", 0) > 10 for m in boat_status) and any(m.get("state") == "MOVING" for m in drive_status):
                handoff = True
            if handoff and any(m.get("route_index", 0) >= 1 for m in drive_status):
                print("FIRST_WAYPOINT_OK", [m for m in drive_status if m.get("route_index", 0) >= 1][-1])
                return 0
            if any(p.poll() is not None for p in processes):
                raise RuntimeError("Agent exited during handoff")
            time.sleep(0.5)
        raise RuntimeError(f"Waypoint timeout handoff={handoff} boat={boat_status[-2:]} drive={drive_status[-2:]}")
    finally:
        bus.publish_reliable(topics.CONTROL_STOP, bus.envelope("test", command="HOLD"))
        for process in processes:
            process.terminate()
            process.wait(timeout=5)


if __name__ == "__main__":
    raise SystemExit(main())
