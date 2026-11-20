#!/usr/bin/env python3
"""Measure false detections on the project's hard-negative image set."""

from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO

from add_negatives import EXCLUDED


def count_false_positives(model_path: Path, images: list[Path], confidence: float) -> tuple[int, int]:
    model = YOLO(model_path)
    false_images = detections = 0
    for start in range(0, len(images), 32):
        results = model.predict(
            [str(path) for path in images[start:start + 32]],
            conf=confidence,
            imgsz=640,
            device=0,
            verbose=False,
        )
        for result in results:
            count = 0 if result.boxes is None else len(result.boxes)
            false_images += int(count > 0)
            detections += count
    return false_images, detections


def main() -> int:
    root = Path(__file__).parent
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "models", nargs="*", type=Path,
        default=[
            root / "runs/yolo11n_air_purifier_seg/weights/best.pt",
            root / "runs/yolo11n_air_purifier_seg2/weights/best.pt",
        ],
    )
    parser.add_argument("--input", type=Path, default=root / "raw_negative")
    parser.add_argument("--pattern", default="*.jpg", help="Filename glob inside --input")
    parser.add_argument("--confidences", type=float, nargs="+", default=[0.20, 0.35, 0.50])
    args = parser.parse_args()
    images = [path for path in sorted(args.input.glob(args.pattern)) if path.name not in EXCLUDED]
    print(f"Hard-negative images: {len(images)}")
    for model_path in args.models:
        print(f"\n{model_path}")
        for confidence in args.confidences:
            false_images, detections = count_false_positives(model_path, images, confidence)
            rate = 100 * false_images / len(images) if images else 0
            print(
                f"  conf={confidence:.2f}: {false_images}/{len(images)} images "
                f"({rate:.1f}%), {detections} detections"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
