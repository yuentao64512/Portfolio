#!/usr/bin/env python3
"""Terminal 2: mock RoboCommand approval, OCS relay, live tables and scoring."""

from __future__ import annotations

import argparse
import json
import math
import select
import sys
import threading
import time
import uuid
from pathlib import Path
from typing import Any

import yaml

from .bus import JsonBus
from .contract import REQUIRED, heartbeat_ok, latlon_to_xy, validate_declaration, validate_report, xy_to_latlon
from . import topics


class MockRoboCommand:
    def __init__(self, config_path: Path, manual_approve: bool = False, auto_go: bool = False) -> None:
        self.cfg = yaml.safe_load(config_path.read_text())
        self.bus = JsonBus()
        self.manual = manual_approve
        self.auto_go = auto_go
        self.lock = threading.Lock()
        self.heartbeats: dict[str, dict[str, Any]] = {}
        self.declaration: dict[str, Any] | None = None
        self.run_id: str | None = None
        self.approved = False
        self.pending_reason = "waiting for declaration"
        self.latest_report: dict[str, Any] | None = None
        self.last_table = 0.0
        for vehicle in ("USV1", "UAV1"):
            self.bus.subscribe(topics.heartbeat(vehicle), lambda m, v=vehicle: self._heartbeat(v, m))
        self.bus.subscribe(topics.DECLARATION, self._declaration)
        self.bus.subscribe(topics.UAV_REPORT, self._report)
        self.bus.subscribe(topics.STATUS_UAV, lambda m: self._status("UAV1", m))
        self.bus.subscribe(topics.STATUS_USV, lambda m: self._status("USV1", m))

    def _heartbeat(self, vehicle: str, msg: dict[str, Any]) -> None:
        with self.lock:
            self.heartbeats[vehicle] = msg

    def _declaration(self, msg: dict[str, Any]) -> None:
        with self.lock:
            self.declaration = msg
            self.approved = False
            self.run_id = None
        ok, reason = validate_declaration(msg)
        print(f"\nDECLARATION tier={msg.get('tier')} vehicles={msg.get('vehicles')} -> {'WAITING' if ok else 'REFUSED'} {reason}")

    def _status(self, vehicle: str, msg: dict[str, Any]) -> None:
        stage = msg.get("stage", msg.get("state", "?"))
        print(f"\nSTATUS {vehicle}: {stage}", flush=True)

    def _report(self, msg: dict[str, Any]) -> None:
        ok, reason = validate_report(msg)
        if not ok:
            print(f"\nREFUSED buoy report: {reason}", flush=True)
            return
        with self.lock:
            self.latest_report = msg
        self.bus.publish(topics.USV_UAV_REPORT, msg)
        self.print_buoy_table(msg)

    def course_message(self) -> dict[str, Any]:
        origin = self.cfg["origin"]
        polygon = []
        for x, y in self.cfg["survey_polygon_xy"]:
            lat, lon = xy_to_latlon(x, y, origin["lat"], origin["lon"])
            polygon.append({"lat": lat, "lon": lon})
        return self.bus.envelope(
            "mock_robocommand",
            world=self.cfg["world_name"],
            origin=origin,
            survey_polygon=polygon,
            survey_waypoints_xy=self.cfg["survey_waypoints_xy"],
            n_expected=len(self.cfg["ground_truth"]),
        )

    def declaration_state(self) -> tuple[bool, str]:
        with self.lock:
            declaration = dict(self.declaration) if self.declaration else None
            heartbeats = dict(self.heartbeats)
        if declaration is None:
            return False, "waiting for declaration"
        ok, reason = validate_declaration(declaration)
        if not ok:
            return False, reason
        tier = str(declaration["tier"]).lower()
        now = time.time()
        for vehicle in REQUIRED[tier]:
            ok, reason = heartbeat_ok(heartbeats.get(vehicle), now)
            if not ok:
                return False, f"{vehicle}: {reason}"
        return True, "ready"

    def approve(self) -> None:
        ok, reason = self.declaration_state()
        if not ok:
            print(f"\nREFUSED: {reason}", flush=True)
            return
        if self.approved:
            return
        self.run_id = f"sim-{uuid.uuid4().hex[:8]}"
        tier = str(self.declaration["tier"]).lower()  # type: ignore[index]
        msg = self.bus.envelope("mock_robocommand", approved=True, run_id=self.run_id, tier=tier)
        self.bus.publish_reliable(topics.RUN_START, msg)
        self.approved = True
        print(f"\nAPPROVED run_id={self.run_id} tier={tier}", flush=True)
        if self.auto_go:
            self.go()

    def go(self) -> None:
        if not self.approved:
            print("\nCannot go: run is not approved", flush=True)
            return
        tier = str(self.declaration["tier"]).lower()  # type: ignore[index]
        self.bus.publish_reliable(topics.TEAM_GO, self.bus.envelope("mock_robocommand", run_id=self.run_id, tier=tier))
        print(f"\nGO published tier={tier} run_id={self.run_id}", flush=True)

    def print_heartbeats(self) -> None:
        now = time.time()
        with self.lock:
            heartbeats = dict(self.heartbeats)
        print("\nVEHICLE  STATE       MODE  RATE   AGE     POSITION")
        for vehicle in ("USV1", "UAV1"):
            hb = heartbeats.get(vehicle)
            if not hb:
                print(f"{vehicle:<8} LOST        -     -      >3s     -")
                continue
            age = now - float(hb.get("ts", 0))
            state = str(hb.get("state", "?")) if age <= 3 else "LOST"
            pos = hb.get("position", {})
            xyz = f"{pos.get('x', 0):.1f},{pos.get('y', 0):.1f},{pos.get('z', 0):.1f}" if isinstance(pos, dict) else "-"
            print(f"{vehicle:<8} {state:<11} {str(hb.get('mode','?')):<5} {float(hb.get('rate_hz',0)):<6.1f} {age:<7.1f} {xyz}")
        ok, reason = self.declaration_state()
        self.pending_reason = reason
        print(f"CHECK: {'READY' if ok else 'WAIT'} - {reason}")

    def print_buoy_table(self, report: dict[str, Any]) -> None:
        origin = self.cfg["origin"]
        gt = self.cfg["ground_truth"]
        print("\nDRONE BUOY TABLE")
        print("ID             COLOUR  LIT       CONF  SIGMA  ERROR")
        for item in report["items"]:
            x, y = latlon_to_xy(item["lat"], item["lon"], origin["lat"], origin["lon"])
            candidates = [g for g in gt if g["colour"] == item["colour"]]
            nearest = min(candidates or gt, key=lambda g: math.hypot(x - g["x"], y - g["y"]))
            error = math.hypot(x - nearest["x"], y - nearest["y"])
            print(f"{str(item['id']):<14} {item['colour']:<7} {str(item['lit']):<9} {item['conf']:<5.2f} {item['sigma_m']:<6.2f} {error:.2f}m")
        scan = report["scan"]
        print(f"scan={scan.get('state')} coverage={scan.get('coverage_pct')}% items={scan.get('n_items')}")
        self.print_score(report)

    def print_score(self, report: dict[str, Any]) -> None:
        origin = self.cfg["origin"]
        gt = self.cfg["ground_truth"]
        detected = report["items"]
        used: set[int] = set()
        correct_colour = 0
        within_one = 0
        role_correct = 0
        for truth in gt:
            best_i = None
            best_d = 1e9
            for i, item in enumerate(detected):
                if i in used:
                    continue
                x, y = latlon_to_xy(item["lat"], item["lon"], origin["lat"], origin["lon"])
                d = math.hypot(x - truth["x"], y - truth["y"])
                if d < best_d:
                    best_i, best_d = i, d
            if best_i is None or best_d > 2.0:
                continue
            used.add(best_i)
            item = detected[best_i]
            correct_colour += int(item["colour"] == truth["colour"])
            within_one += int(best_d < 1.0)
            role_correct += int(item["class"] == truth["class"])
        passed = len(detected) == len(gt) and correct_colour == len(gt) and within_one == len(gt) and role_correct == len(gt)
        print(f"DRONE SCORE found={len(used)}/{len(gt)} colour={correct_colour}/{len(gt)} role={role_correct}/{len(gt)} under_1m={within_one}/{len(gt)} RESULT={'PASS' if passed else 'FAIL'}")
        print("BOAT SCORE pending track evaluator: entry_cw=? gates=? exit_ccw=?")

    def disruptive_change(self) -> None:
        payload = self.bus.envelope("mock_robocommand", action="swap_gate_2", requested_by="operator")
        self.bus.publish_reliable(topics.ROUTE_CHANGE, payload)
        print("\nDisruptive route-change request published (swap gate 2)", flush=True)

    def loop(self) -> None:
        print("Mock RoboCommand + OCS ready. Keys: a=approve, g=go, d=disrupt, s=stop, q=quit")
        next_course = 0.0
        next_table = 0.0
        try:
            while True:
                now = time.time()
                if now >= next_course:
                    self.bus.publish(topics.COURSE, self.course_message())
                    next_course = now + 1.0
                if now >= next_table:
                    self.print_heartbeats()
                    next_table = now + 2.0
                    if not self.manual:
                        self.approve()
                readable, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not readable:
                    continue
                raw = sys.stdin.readline()
                if raw == "":
                    time.sleep(0.2)
                    continue
                command = raw.strip().lower()
                if command == "a":
                    self.approve()
                elif command == "g":
                    self.go()
                elif command == "d":
                    self.disruptive_change()
                elif command == "s":
                    self.bus.publish_reliable(topics.CONTROL_STOP, self.bus.envelope("mock_robocommand", command="HOLD"))
                    print("HOLD published")
                elif command == "q":
                    return
        except KeyboardInterrupt:
            self.bus.publish_reliable(topics.CONTROL_STOP, self.bus.envelope("mock_robocommand", command="HOLD"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--manual-approve", action="store_true")
    parser.add_argument("--auto-go", action="store_true", help="publish team/go immediately after automatic approval")
    args = parser.parse_args()
    MockRoboCommand(args.config, args.manual_approve, args.auto_go).loop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

