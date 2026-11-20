#!/usr/bin/env python3
"""Train the final six-class RobotX detector."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--model", default="yolo11m.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--batch", type=int, default=2)
    args = parser.parse_args()

    YOLO(args.model).train(
        data=str(args.data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        patience=20,
        amp=True,
        device=0,
        workers=8,
        hsv_h=0.0,
        hsv_s=0.35,
        hsv_v=0.40,
        fliplr=0.5,
        flipud=0.0,
        degrees=6.0,
        project=str(args.project),
        name=args.name,
        seed=42,
        deterministic=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
