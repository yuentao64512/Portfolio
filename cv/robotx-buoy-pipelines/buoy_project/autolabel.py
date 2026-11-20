#!/usr/bin/env python3
"""Draft YOLO labels for unlabelled full frames; human correction is mandatory."""

import argparse
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--name", default="autolabel_draft")
    parser.add_argument("--conf", type=float, default=0.30)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--save-conf", action="store_true", help="append confidence as a sixth field")
    args = parser.parse_args()
    YOLO(str(args.weights)).predict(
        source=str(args.source),
        imgsz=args.imgsz,
        conf=args.conf,
        save=False,
        save_txt=True,
        save_conf=args.save_conf,
        project=str(ROOT / "runs"),
        name=args.name,
    )
    print("Draft labels require human review before import.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
