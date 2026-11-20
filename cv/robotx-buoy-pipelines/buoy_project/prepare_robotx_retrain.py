#!/usr/bin/env python3
"""Derive and merge pseudo-labelled RobotX training data without leakage."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from collections import Counter
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def copy_dataset(source: Path, target: Path):
    if target.exists():
        raise FileExistsError(target)
    shutil.copytree(source, target)


def append_train(source: Path, target: Path, prefix: str) -> int:
    copied = 0
    for image in sorted((source / "images" / "train").iterdir()):
        if image.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label = source / "labels" / "train" / f"{image.stem}.txt"
        if not label.is_file():
            raise FileNotFoundError(label)
        image_target = target / "images" / "train" / f"{prefix}{image.name}"
        label_target = target / "labels" / "train" / f"{prefix}{image.stem}.txt"
        if image_target.exists() or label_target.exists():
            raise FileExistsError(image_target)
        shutil.copy2(image, image_target)
        shutil.copy2(label, label_target)
        copied += 1
    return copied


def count_labels(dataset: Path):
    counts = {split: Counter() for split in ("train", "val", "test")}
    images = Counter()
    for split in counts:
        image_dir = dataset / "images" / split
        label_dir = dataset / "labels" / split
        if not image_dir.is_dir():
            continue
        for image in image_dir.iterdir():
            if image.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            images[split] += 1
            label = label_dir / f"{image.stem}.txt"
            for line in label.read_text().splitlines():
                parts = line.split()
                if len(parts) == 5:
                    counts[split][int(parts[0])] += 1
    return dict(images), {split: dict(counter) for split, counter in counts.items()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project", type=Path, required=True)
    parser.add_argument("--pseudo-full", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite {args.output}")
    args.output.mkdir(parents=True)

    pseudo_buoy = args.output / "pseudo_buoy"
    pseudo_digit = args.output / "pseudo_digit"
    subprocess.run(
        [str(args.project.parent / ".venv" / "bin" / "python"),
         str(args.project / "build_datasets.py"), "--full", str(args.pseudo_full),
         "--buoy", str(pseudo_buoy), "--digit", str(pseudo_digit)], check=True
    )

    merged_buoy = args.output / "buoy_dataset"
    merged_digit = args.output / "digit_dataset"
    copy_dataset(args.project / "buoy_dataset", merged_buoy)
    copy_dataset(args.project / "digit_dataset", merged_digit)
    added_buoy = append_train(pseudo_buoy, merged_buoy, "new_")
    added_digit = append_train(pseudo_digit, merged_digit, "new_")

    buoy_images, buoy_labels = count_labels(merged_buoy)
    digit_images, digit_labels = count_labels(merged_digit)
    summary = {
        "new_training_images": {"buoy": added_buoy, "digit_crops": added_digit},
        "buoy": {"images": buoy_images, "labels": buoy_labels},
        "digit": {"images": digit_images, "labels": digit_labels},
        "validation_and_test_source": "unchanged reviewed legacy dataset",
        "new_folder_used_for": "training only",
        "provisional_pseudo_labels": True,
    }
    (args.output / "dataset_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
