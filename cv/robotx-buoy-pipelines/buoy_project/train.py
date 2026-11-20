#!/usr/bin/env python3
"""Train the two-stage YOLO11n models sequentially on the RTX 5060 Ti."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent


def train_buoy(args):
    return YOLO("yolo11n.pt").train(
        data=str(ROOT / "buoys.yaml"),
        epochs=args.epochs_buoy,
        imgsz=960,
        batch=args.batch_buoy,
        patience=30,
        device=args.device,
        workers=8,
        seed=42,
        deterministic=True,
        fliplr=0.5,
        flipud=0.0,
        hsv_h=0.0,
        hsv_v=0.5,
        hsv_s=0.5,
        project=str(ROOT / "runs"),
        name="buoys_v1",
    )


def train_digit(args):
    return YOLO("yolo11n.pt").train(
        data=str(ROOT / "digits.yaml"),
        epochs=args.epochs_digit,
        imgsz=320,
        batch=args.batch_digit,
        patience=40,
        device=args.device,
        workers=8,
        seed=42,
        deterministic=True,
        fliplr=0.0,
        flipud=0.0,
        degrees=25.0,
        shear=10.0,
        perspective=0.0005,
        mosaic=0.5,
        hsv_h=0.0,
        hsv_v=0.5,
        project=str(ROOT / "runs"),
        name="digits_v1_legacy_provisional",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", choices=("both", "buoy", "digit"), default="both")
    parser.add_argument("--epochs-buoy", type=int, default=150)
    parser.add_argument("--epochs-digit", type=int, default=200)
    parser.add_argument("--batch-buoy", type=int, default=8)
    parser.add_argument("--batch-digit", type=int, default=32)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()
    if args.model in ("both", "buoy"):
        train_buoy(args)
    if args.model in ("both", "digit"):
        train_digit(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
