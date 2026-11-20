#!/usr/bin/env python3
"""Run the six-class RobotX detector on a camera, image, or video."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def source_value(value: str):
    return int(value) if value.isdigit() else value


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="weights/best.pt")
    parser.add_argument("--source", required=True, help="Camera index or image/video path")
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--device", default="0")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--track", action="store_true", help="Use ByteTrack IDs")
    parser.add_argument("--project", type=Path, default=Path("runs/predict"))
    parser.add_argument("--name", default="six_class")
    args = parser.parse_args()

    model = YOLO(args.model, task="detect")
    common = dict(
        source=source_value(args.source), imgsz=args.imgsz, conf=args.conf,
        device=args.device, show=args.show, save=True, project=str(args.project),
        name=args.name,
    )
    if args.track:
        model.track(tracker="bytetrack.yaml", persist=True, **common)
    else:
        model.predict(**common)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
