#!/usr/bin/env python3
"""Refuse mapping or docking when calibration and safety fields are incomplete."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("config", type=Path, nargs="?", default=Path("runtime_config.yaml"))
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    errors = []
    mapping = bool(config["mapping"]["enabled"])
    docking = bool(config["docking"]["enabled"])
    required_measurements = (
        ("camera.calibration_file", config["camera"]["calibration_file"]),
        ("camera.latency_seconds", config["camera"]["latency_seconds"]),
        ("camera.height_above_water_m", config["camera"]["height_above_water_m"]),
        ("camera.tilt_down_degrees", config["camera"]["tilt_down_degrees"]),
        ("camera.offset_from_gps_m", config["camera"]["offset_from_gps_m"]),
        ("objects.buoy_height_above_water_m", config["objects"]["buoy_height_above_water_m"]),
        ("objects.bay_height_above_water_m", config["objects"]["bay_height_above_water_m"]),
        ("objects.dock_height_above_water_m", config["objects"]["dock_height_above_water_m"]),
    )
    if mapping or docking:
        for name, value in required_measurements:
            if value is None:
                errors.append(f"missing {name}")
        calibration = config["camera"]["calibration_file"]
        if calibration and not Path(calibration).is_file():
            errors.append(f"calibration file does not exist: {calibration}")
    if docking and not mapping:
        errors.append("docking requires mapping.enabled=true")
    if config["docking"]["target_bay_colour"] not in ("red", "green"):
        errors.append("target_bay_colour must be red or green")
    if config["docking"]["enable_guided_commands"]:
        if not docking:
            errors.append("GUIDED commands require docking.enabled=true")
        if not config["safety"]["require_rc_override"]:
            errors.append("RC override guard must remain enabled")
        if not config["safety"]["require_ardupilot_failsafes"]:
            errors.append("ArduPilot failsafe guard must remain enabled")
    if errors:
        print("CONFIGURATION INVALID")
        for error in errors:
            print(f"- {error}")
        return 1
    print("CONFIGURATION VALID")
    print(f"mapping_enabled={mapping}")
    print(f"docking_enabled={docking}")
    print(f"guided_commands_enabled={bool(config['docking']['enable_guided_commands'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

