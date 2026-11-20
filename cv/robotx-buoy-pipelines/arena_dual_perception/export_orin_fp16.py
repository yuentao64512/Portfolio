#!/usr/bin/env python3
"""Export final nav/dock checkpoints to FP16 TensorRT on the target Orin."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nav", type=Path, default=Path("weights/nav_best.pt"))
    parser.add_argument("--dock", type=Path, default=Path("weights/dock_best.pt"))
    args = parser.parse_args()
    if not Path("/etc/nv_tegra_release").is_file():
        raise SystemExit("Refusing export: TensorRT engines must be built on the target Jetson AGX Orin")
    for weight in (args.nav, args.dock):
        if not weight.is_file():
            raise FileNotFoundError(weight)
        YOLO(str(weight), task="detect").export(
            format="engine", half=True, imgsz=960, batch=1, device=0, workspace=4
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

