#!/usr/bin/env python3
"""Terminal 1: declare a tier, request go, monitor status, and stop safely."""

from __future__ import annotations

import argparse
import select
import sys
import time

from .bus import JsonBus
from .contract import REQUIRED, TIERS
from . import topics


class TeamConsole:
    def __init__(self) -> None:
        self.bus = JsonBus()
        self.approved: dict | None = None
        self.tier: str | None = None
        self.bus.subscribe(topics.RUN_START, self._run_start)
        self.bus.subscribe(topics.STATUS_UAV, lambda m: self._status("UAV1", m))
        self.bus.subscribe(topics.STATUS_USV, lambda m: self._status("USV1", m))

    def _run_start(self, msg: dict) -> None:
        self.approved = msg
        print(f"\nAPPROVED run_id={msg.get('run_id')} tier={msg.get('tier')}", flush=True)

    @staticmethod
    def _status(vehicle: str, msg: dict) -> None:
        extra = ""
        if vehicle == "UAV1":
            extra = f" sim_cam_frames={msg.get('frames', 0)} coverage={msg.get('coverage_pct', 0)}% buoys={msg.get('n_items', 0)}"
        elif vehicle == "USV1":
            extra = f" sim_cv_frames={msg.get('cv_frames', 0)} markers={msg.get('cv_detections', 0)}"
        print(f"\n{vehicle} state={msg.get('state')} stage={msg.get('stage')}{extra}", flush=True)

    def declare(self, tier: str) -> None:
        tier = tier.lower()
        if tier not in TIERS:
            print("Use: declare core|advanced|disruptive")
            return
        self.tier = tier
        self.approved = None
        msg = self.bus.envelope("team_console", tier=tier, vehicles=list(REQUIRED[tier]))
        self.bus.publish_reliable(topics.DECLARATION, msg)
        print(f"DECLARED {tier}; waiting for RunStart")

    def go(self) -> None:
        if not self.approved:
            print("Cannot go: wait for APPROVED")
            return
        msg = self.bus.envelope("team_console", tier=self.tier, run_id=self.approved.get("run_id"))
        self.bus.publish_reliable(topics.TEAM_GO, msg)
        print(f"GO sent tier={self.tier}")

    def stop(self) -> None:
        self.bus.publish_reliable(topics.CONTROL_STOP, self.bus.envelope("team_console", command="HOLD"))
        print("STOP/HOLD sent")

    def loop(self) -> None:
        print("Team console ready. Commands: declare core|advanced|disruptive, go, stop, quit")
        try:
            while True:
                readable, _, _ = select.select([sys.stdin], [], [], 0.2)
                if not readable:
                    continue
                line = sys.stdin.readline().strip()
                words = line.split()
                if not words:
                    continue
                if words[0] == "declare" and len(words) == 2:
                    self.declare(words[1])
                elif words[0] == "go":
                    self.go()
                elif words[0] == "stop":
                    self.stop()
                elif words[0] in {"quit", "exit", "q"}:
                    self.stop()
                    return
                else:
                    print("Unknown command")
        except KeyboardInterrupt:
            self.stop()


def main() -> int:
    argparse.ArgumentParser(description=__doc__).parse_args()
    TeamConsole().loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

