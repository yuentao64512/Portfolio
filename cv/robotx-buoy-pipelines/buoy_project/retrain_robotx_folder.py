#!/usr/bin/env python3
"""Fine-tune both best checkpoints with the derived RobotX training data."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--run-name", default="robotx_folder_v2")
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    buoy_yaml = args.dataset / "buoys_retrain.yaml"
    digit_yaml = args.dataset / "digits_retrain.yaml"
    buoy_yaml.write_text(
        f"path: {args.dataset / 'buoy_dataset'}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n  0: buoy\n"
    )
    digit_yaml.write_text(
        f"path: {args.dataset / 'digit_dataset'}\ntrain: images/train\nval: images/val\ntest: images/test\n"
        "names:\n" + "".join(f'  {i}: "{i}"\n' for i in range(10))
    )

    YOLO(str(args.project / "runs" / "buoys_v1" / "weights" / "best.pt")).train(
        data=str(buoy_yaml), epochs=100, imgsz=960, batch=8, patience=25,
        device=args.device, workers=8, seed=42, deterministic=True,
        fliplr=0.5, flipud=0.0, hsv_h=0.0, hsv_v=0.5, hsv_s=0.5,
        project=str(args.project / "runs"), name=f"buoys_{args.run_name}",
    )
    YOLO(str(args.project / "runs" / "digits_v1_legacy_provisional" / "weights" / "best.pt")).train(
        data=str(digit_yaml), epochs=150, imgsz=320, batch=32, patience=30,
        device=args.device, workers=8, seed=42, deterministic=True,
        fliplr=0.0, flipud=0.0, degrees=25.0, shear=10.0,
        perspective=0.0005, mosaic=0.5, hsv_h=0.0, hsv_v=0.5,
        project=str(args.project / "runs"), name=f"digits_{args.run_name}_pseudo",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
