#!/usr/bin/env python3
"""Train and evaluate the one-class air-purifier YOLO11 segmentation model."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


def main() -> int:
    root = Path(__file__).parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default=str(root / "dataset" / "air_purifier.yaml"))
    parser.add_argument("--model", default="yolo11n-seg.pt")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=8)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    model = YOLO(args.model)
    results = model.train(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=8,
        project=str(root / "runs"),
        name="yolo11n_air_purifier_seg",
        patience=20,
        pretrained=True,
        plots=True,
        cache=False,
        seed=42,
        deterministic=True,
    )
    best = Path(results.save_dir) / "weights" / "best.pt"
    print(f"Best model: {best}")
    YOLO(best).val(data=args.data, split="test", device=args.device, plots=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
