#!/usr/bin/env python3
"""Train the requested nav and dock YOLO11n models after coverage validation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--derived", type=Path, required=True)
    parser.add_argument("--project", type=Path, default=Path("runs"))
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()
    summary = json.loads((args.derived / "dataset_summary.json").read_text())
    if summary["classes_below_minimum"]:
        raise SystemExit("Training blocked; classes below 150 boxes: " + ", ".join(summary["classes_below_minimum"]))

    common = dict(
        epochs=args.epochs, imgsz=960, batch=args.batch, patience=35,
        device=args.device, workers=8, seed=42, deterministic=True,
        hsv_h=0.0, hsv_s=0.45, hsv_v=0.45, fliplr=0.5, flipud=0.0,
        degrees=8.0, project=str(args.project),
    )
    YOLO("yolo11n.pt").train(data=str(args.derived / "nav.yaml"), name="nav_yolo11n_960", **common)
    YOLO("yolo11n.pt").train(data=str(args.derived / "dock.yaml"), name="dock_yolo11n_960", **common)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

