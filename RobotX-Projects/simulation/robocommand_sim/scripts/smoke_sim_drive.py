#!/usr/bin/env python3
"""Isolated Gazebo proof: a route moves the boat, STOP sends zero thrust."""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time

from gz.msgs10.double_pb2 import Double
from gz.transport13 import Node

from robocommand import topics
from robocommand.bus import JsonBus


def main() -> int:
    if not os.environ.get("GZ_PARTITION", "").startswith("sim_boat_drive_test_"):
        raise RuntimeError("Refusing to test outside isolated Gazebo partition")
    process = subprocess.Popen([sys.executable, "-m", "robocommand.sim_boat_driver"],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    bus = JsonBus()
    statuses: list[dict] = []
    thrust: list[float] = []
    bus.subscribe(topics.USV_DRIVE_STATUS, statuses.append)
    node = Node()
    node.subscribe(Double, "/model/usv_four_thruster/joint/motor_port_joint/cmd_thrust",
                   lambda msg, *_: thrust.append(float(msg.data)))
    try:
        time.sleep(2)
        bus.publish_reliable(topics.TEAM_GO, bus.envelope("test", tier="advanced", run_id="isolated-test"))
        route = [{"action": "TEST", "x": 3.0, "y": 37.0}]
        bus.publish_reliable(topics.USV_ROUTE, bus.envelope("test", tier="advanced", run_id="isolated-test", route=route))
        start = time.monotonic()
        moving = []
        while time.monotonic() - start < 30:
            if time.monotonic() - start > 0.8:
                bus.publish(topics.USV_ROUTE, bus.envelope("test", tier="advanced", run_id="isolated-test", route=route))
            moving = [m for m in statuses if m.get("run_id") == "isolated-test" and m.get("state") == "MOVING" and m.get("position")]
            if moving and moving[-1]["position"]["x"] > 0.8:
                break
            if process.poll() is not None:
                raise RuntimeError("Driver exited during route")
            time.sleep(0.2)
        if not moving or moving[-1]["position"]["x"] <= 0.8:
            raise RuntimeError(f"Boat did not move: {statuses[-3:]}")
        if not any(abs(value) > 0.1 for value in thrust):
            raise RuntimeError("No nonzero thrust observed")
        print("MOVE_OK", moving[-1]["position"], "nonzero_thrust_count", sum(abs(v) > 0.1 for v in thrust))
        bus.publish_reliable(topics.CONTROL_STOP, bus.envelope("test", command="HOLD"))
        time.sleep(1.0)
        if not thrust or abs(thrust[-1]) > 1e-6:
            raise RuntimeError(f"STOP did not zero thrust: {thrust[-5:]}")
        print("STOP_ZERO_OK", thrust[-1])
        return 0
    finally:
        bus.publish_reliable(topics.CONTROL_STOP, bus.envelope("test", command="HOLD"))
        process.terminate()
        output, _ = process.communicate(timeout=5)
        print(output[-2500:])


if __name__ == "__main__":
    raise SystemExit(main())
