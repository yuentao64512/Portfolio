#!/usr/bin/env python3
"""Disruptive-tier Gazebo route changer (swaps gate-2 red/green markers)."""

from __future__ import annotations

import argparse
import subprocess
import time
from typing import Any

from . import topics
from .bus import JsonBus


class LightController:
    def __init__(self, world: str, dry_run: bool = False) -> None:
        self.world = world
        self.dry_run = dry_run
        self.bus = JsonBus()
        self.bus.subscribe(topics.ROUTE_CHANGE, self._change)
        self.changed = False
        self.last_event: float | None = None

    def set_pose(self, name: str, x: float, y: float, z: float = 0.65) -> None:
        command = [
            "gz", "service", "-s", f"/world/{self.world}/set_pose",
            "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
            "--req", f'name: "{name}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{w: 1}}',
        ]
        if self.dry_run:
            print("DRY-RUN", " ".join(command))
            return
        result = subprocess.run(command, check=False, text=True, capture_output=True)
        if result.returncode != 0 or "true" not in result.stdout.lower():
            raise RuntimeError(f"set_pose failed for {name}: {result.stdout} {result.stderr}")

    def _change(self, msg: dict[str, Any]) -> None:
        if msg.get("action") != "swap_gate_2":
            return
        event = float(msg.get("ts", 0.0))
        if event and event == self.last_event:
            return
        self.last_event = event
        try:
            if not self.changed:
                self.set_pose("green_2", -6.0, -10.0)
                self.set_pose("red_2", -22.0, -8.0)
            else:
                self.set_pose("green_2", -22.0, -8.0)
                self.set_pose("red_2", -6.0, -10.0)
            self.changed = not self.changed
            print(f"Gazebo gate 2 swapped={self.changed}; UAV must rescan", flush=True)
        except Exception as exc:
            print(f"Route change failed safely: {exc}", flush=True)

    def loop(self) -> None:
        print("light_ctl ready; press d in Terminal 2 to swap gate 2")
        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            return


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="usv_four_thruster")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    LightController(args.world, args.dry_run).loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

