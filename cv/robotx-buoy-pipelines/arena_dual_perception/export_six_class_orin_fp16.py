#!/usr/bin/env python3
"""Export the final PT checkpoint to FP16 TensorRT on the target Jetson."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weight", type=Path, default=Path("weights/best.pt"))
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--workspace", type=float, default=4.0)
    args = parser.parse_args()
    if not Path("/etc/nv_tegra_release").is_file():
        raise SystemExit("Run this export on the target Jetson; TensorRT engines are device-specific")
    if not args.weight.is_file():
        raise FileNotFoundError(args.weight)
    exported = YOLO(str(args.weight), task="detect").export(
        format="engine", half=True, imgsz=args.imgsz, batch=1,
        device=0, workspace=args.workspace,
    )
    print(exported)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
