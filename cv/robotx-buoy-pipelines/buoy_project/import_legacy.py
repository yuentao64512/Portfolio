#!/usr/bin/env python3
"""Build provisional one-class buoy and digit-only datasets from reviewed legacy data."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEFAULT_A = Path("/home/robotx/buoy_actual/boat_yolo/datasets/model_a_plus_img3922_v1")
DEFAULT_B = Path("/home/robotx/buoy_actual/boat_yolo/datasets/model_b_reviewed_v2")
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def remap_labels(source: Path, target: Path, mode: str, counts: Counter):
    output = []
    for number, line in enumerate(source.read_text().splitlines(), 1):
        parts = line.split()
        if len(parts) != 5:
            raise ValueError(f"{source}:{number}: expected five fields")
        class_id = int(parts[0])
        if mode == "buoy" and class_id in (0, 1, 2):
            output.append("0 " + " ".join(parts[1:]))
            counts[0] += 1
        elif mode == "digit" and 2 <= class_id <= 11:
            output.append(f"{class_id-2} " + " ".join(parts[1:]))
            counts[class_id-2] += 1
    target.write_text("\n".join(output) + ("\n" if output else ""))


def convert_dataset(source: Path, target: Path, mode: str):
    counts = Counter()
    image_counts = Counter()
    for split in ("train", "val", "test"):
        source_images = source / "images" / split
        source_labels = source / "labels" / split
        target_images = target / "images" / split
        target_labels = target / "labels" / split
        target_images.mkdir(parents=True, exist_ok=False)
        target_labels.mkdir(parents=True, exist_ok=False)
        if not source_images.is_dir():
            continue
        for image in sorted(source_images.iterdir()):
            if image.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            label = source_labels / f"{image.stem}.txt"
            if not label.is_file():
                raise FileNotFoundError(label)
            shutil.copy2(image, target_images / image.name)
            remap_labels(label, target_labels / label.name, mode, counts)
            image_counts[split] += 1
    return image_counts, counts


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-a", type=Path, default=DEFAULT_A)
    parser.add_argument("--model-b", type=Path, default=DEFAULT_B)
    parser.add_argument("--buoy-output", type=Path, default=ROOT / "buoy_dataset")
    parser.add_argument("--digit-output", type=Path, default=ROOT / "digit_dataset")
    args = parser.parse_args()
    if args.buoy_output.exists() or args.digit_output.exists():
        raise SystemExit("Refusing to overwrite an existing imported dataset")

    buoy_images, buoy_counts = convert_dataset(args.model_a, args.buoy_output, "buoy")
    digit_images, digit_counts = convert_dataset(args.model_b, args.digit_output, "digit")
    summary = {
        "sources": {"model_a": str(args.model_a), "model_b": str(args.model_b)},
        "buoy_images": dict(buoy_images),
        "buoy_instances": int(buoy_counts[0]),
        "digit_images": dict(digit_images),
        "digit_instances": {str(i): int(digit_counts[i]) for i in range(10)},
        "missing_digit_classes": [i for i in range(10) if digit_counts[i] == 0],
        "provisional": True,
    }
    (ROOT / "legacy_import_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
