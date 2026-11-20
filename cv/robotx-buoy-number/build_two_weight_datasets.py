#!/usr/bin/env python3
"""Build reviewed YOLO detection and classification datasets."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
from collections import Counter, defaultdict
from pathlib import Path


DIGITS = {str(value) for value in range(10)}
MARKERS = DIGITS | {"C", "D"}
NUMBER_LABELS = DIGITS | {"N"}
BOUNDARY_LABELS = {"C", "D", "BD"}


def link_or_copy(source: Path, destination: Path):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        return
    try:
        os.link(source, destination)
    except OSError:
        shutil.copy2(source, destination)


def session_split(filename: str) -> str:
    stem = Path(filename).stem
    if stem.startswith("rgb_"):
        return "test"
    if stem.isdigit() and len(stem) == 2:
        return "val"
    return "train"


def marker_split(label: str, filename: str) -> str:
    """Keep recordings grouped, with explicit contiguous blocks for rare classes."""
    stem = Path(filename).stem
    if label == "C" and stem.isdigit():
        value = int(stem)
        if value in {20, 118, 119, 120, 121}:
            return "val"
        if value in {128, 137, 138, 139, 140, 141, 142, 143}:
            return "test"
        return "train"
    if label == "2" and stem.startswith("rgb_"):
        frame = int(stem.rsplit("_", 1)[1])
        if frame <= 36:
            return "train"
        if frame <= 38:
            return "val"
        return "test"
    return session_split(filename)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--crops", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, required=True)
    parser.add_argument("--manual", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-train-per-marker", type=int, default=20)
    args = parser.parse_args()

    with args.candidates.open(newline="", encoding="utf-8") as handle:
        candidates = {row["id"]: row for row in csv.DictReader(handle)}
    with args.manual.open(newline="", encoding="utf-8") as handle:
        manual_rows = list(csv.DictReader(handle))
    manual = {row["id"]: row for row in manual_rows}
    if set(candidates) != set(manual):
        missing = sorted(set(candidates) - set(manual))
        extra = sorted(set(manual) - set(candidates))
        raise SystemExit(f"ID mismatch: missing={missing} extra={extra}")

    detector_root = args.output / "buoy_detector"
    classifier_root = args.output / "marker_classifier"
    labels_by_image: dict[str, list[str]] = defaultdict(list)
    detector_counts = Counter()
    marker_counts = Counter()
    split_marker_counts = defaultdict(Counter)

    for candidate_id, candidate in candidates.items():
        label = manual[candidate_id]["label"].strip()
        if label in NUMBER_LABELS:
            detector_class = 0
        elif label in BOUNDARY_LABELS:
            detector_class = 1
        else:
            detector_class = None

        if detector_class is not None:
            image_path = args.images / candidate["image"]
            import cv2

            image = cv2.imread(str(image_path))
            if image is None:
                raise SystemExit(f"Cannot read {image_path}")
            height, width = image.shape[:2]
            x1, y1 = int(candidate["x1"]), int(candidate["y1"])
            x2, y2 = int(candidate["x2"]), int(candidate["y2"])
            xc = ((x1 + x2) / 2) / width
            yc = ((y1 + y2) / 2) / height
            bw = (x2 - x1) / width
            bh = (y2 - y1) / height
            labels_by_image[candidate["image"]].append(
                f"{detector_class} {xc:.8f} {yc:.8f} {bw:.8f} {bh:.8f}"
            )
            detector_counts[(session_split(candidate["image"]), detector_class)] += 1

        if label in MARKERS:
            split = marker_split(label, candidate["image"])
            source_crop = args.crops / candidate["crop"]
            destination = classifier_root / split / label / candidate["crop"]
            link_or_copy(source_crop, destination)
            marker_counts[label] += 1
            split_marker_counts[split][label] += 1

    balanced_train_counts = Counter()
    train_root = classifier_root / "train"
    for label in sorted(MARKERS):
        class_dir = train_root / label
        files = sorted(p for p in class_dir.glob("*") if p.is_file())
        if not files:
            continue
        needed = max(0, args.min_train_per_marker - len(files))
        for index in range(needed):
            source = files[index % len(files)]
            destination = class_dir / f"{source.stem}__repeat_{index + 1:03d}{source.suffix}"
            link_or_copy(source, destination)
        balanced_train_counts[label] = len(files) + needed

    image_suffixes = {".jpg", ".jpeg", ".png"}
    image_counts = Counter()
    for image_path in sorted(args.images.iterdir()):
        if image_path.suffix.lower() not in image_suffixes:
            continue
        split = session_split(image_path.name)
        image_counts[split] += 1
        destination = detector_root / "images" / split / image_path.name
        link_or_copy(image_path, destination)
        label_path = detector_root / "labels" / split / f"{image_path.stem}.txt"
        label_path.parent.mkdir(parents=True, exist_ok=True)
        label_path.write_text("\n".join(labels_by_image.get(image_path.name, [])) +
                              ("\n" if labels_by_image.get(image_path.name) else ""),
                              encoding="utf-8")

    dataset_yaml = (
        f"path: {detector_root}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "names:\n"
        "  0: number_buoy\n"
        "  1: boundary\n"
    )
    (detector_root / "data.yaml").write_text(dataset_yaml, encoding="utf-8")

    summary = {
        "detector_images": dict(image_counts),
        "detector_boxes": {
            split: {
                "number_buoy": detector_counts[(split, 0)],
                "boundary": detector_counts[(split, 1)],
            }
            for split in ("train", "val", "test")
        },
        "marker_total": dict(sorted(marker_counts.items())),
        "marker_by_split": {
            split: dict(sorted(split_marker_counts[split].items()))
            for split in ("train", "val", "test")
        },
        "marker_balanced_train": dict(sorted(balanced_train_counts.items())),
        "split_policy": {
            "train": "numeric photo sessions with 3-digit names",
            "val": "numeric photo session with 2-digit names",
            "test": "rgb_ recording session",
            "marker_exception": "rare C and 2 use contiguous source-frame blocks so every class can be trained and independently checked",
        },
    }
    (args.output / "dataset_summary.json").parent.mkdir(parents=True, exist_ok=True)
    (args.output / "dataset_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
