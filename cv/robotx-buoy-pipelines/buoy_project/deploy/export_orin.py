#!/usr/bin/env python3
"""Export both trained weights to Orin-specific FP16 TensorRT engines."""

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--buoy", type=Path, default=Path("models/buoys_best.pt"))
    parser.add_argument("--digit", type=Path, default=Path("models/digits_best.pt"))
    args = parser.parse_args()
    YOLO(str(args.buoy), task="detect").export(
        format="engine", half=True, imgsz=960, batch=1, device=0, workspace=4
    )
    YOLO(str(args.digit), task="detect").export(
        format="engine", half=True, imgsz=320, dynamic=True, batch=8, device=0, workspace=4
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
