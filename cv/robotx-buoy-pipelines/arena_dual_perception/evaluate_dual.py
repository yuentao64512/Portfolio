#!/usr/bin/env python3
"""Evaluate both final checkpoints on independent master-derived test splits."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--derived", type=Path, required=True)
    parser.add_argument("--runs", type=Path, default=Path("runs"))
    args = parser.parse_args()
    cases = {
        "nav": (args.runs / "nav_yolo11n_960" / "weights" / "best.pt", args.derived / "nav.yaml"),
        "dock": (args.runs / "dock_yolo11n_960" / "weights" / "best.pt", args.derived / "dock.yaml"),
    }
    report = {}
    for name, (weight, data) in cases.items():
        if not weight.is_file():
            raise FileNotFoundError(weight)
        result = YOLO(str(weight)).val(data=str(data), split="test", imgsz=960)
        report[name] = {
            "weight": str(weight.resolve()),
            "mAP50": float(result.box.map50),
            "mAP50_95": float(result.box.map),
        }
    (args.runs / "dual_test_metrics.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

