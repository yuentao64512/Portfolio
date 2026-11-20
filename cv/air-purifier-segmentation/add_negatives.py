#!/usr/bin/env python3
"""Add verified non-purifier images as empty-label examples."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

from ultralytics import YOLO


EXCLUDED = {"b38650212a8ebefb.jpg"}  # Multifunction product advertised with purification.


def split_for(name: str) -> str:
    value = int(hashlib.sha256(("negative:" + name).encode()).hexdigest()[:8], 16) % 100
    return "train" if value < 80 else "val" if value < 95 else "test"


def main() -> int:
    root = Path(__file__).parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, default=root / "raw_negative")
    parser.add_argument("--dataset", type=Path, default=root / "dataset")
    parser.add_argument(
        "--model", default=str(root / "runs/yolo11n_air_purifier_seg/weights/best.pt")
    )
    parser.add_argument("--reject-confidence", type=float, default=1.01)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    images = sorted(args.input.glob("*.jpg"))
    model = YOLO(args.model)
    accepted = rejected = 0
    manifest_path = args.dataset / "negative_manifest.jsonl"
    with manifest_path.open("w", encoding="utf-8") as manifest:
        for start in range(0, len(images), 16):
            batch = images[start:start + 16]
            results = ([None] * len(batch) if args.reject_confidence >= 1.0 else model.predict(
                [str(path) for path in batch], conf=args.reject_confidence,
                device=args.device, imgsz=640, verbose=False,
            ))
            for source, result in zip(batch, results):
                if source.name in EXCLUDED:
                    rejected += 1
                    manifest.write(json.dumps({"file": source.name, "accepted": False,
                                               "reason": "manual_exclusion"}) + "\n")
                    continue
                detections = 0 if result is None or result.boxes is None else len(result.boxes)
                if detections:
                    rejected += 1
                    manifest.write(json.dumps({"file": source.name, "accepted": False,
                                               "reason": "purifier_prediction"}) + "\n")
                    continue
                split = split_for(source.name)
                destination = args.dataset / "images" / split / f"negative_{source.name}"
                label = args.dataset / "labels" / split / f"negative_{source.stem}.txt"
                shutil.copy2(source, destination)
                label.write_text("", encoding="utf-8")
                accepted += 1
                manifest.write(json.dumps({"file": source.name, "accepted": True,
                                           "split": split}) + "\n")
    print(f"Negative examples: accepted {accepted}, rejected {rejected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
