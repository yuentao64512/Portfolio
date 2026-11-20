#!/usr/bin/env python3
"""Derive nav and dock YOLO datasets from one immutable 10-class master."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path


MASTER_NAMES = (
    "buoy_red", "buoy_green", "buoy_blue", "buoy", "dock",
    "platform", "obstacle", "bay_red", "bay_green", "bay",
)
NAV_NAMES = MASTER_NAMES[:7]
DOCK_NAMES = ("bay_red", "bay_green", "bay", "obstacle")
NAV_MAP = {index: index for index in range(7)}
DOCK_MAP = {0: 3, 1: 3, 2: 3, 3: 3, 5: 3, 6: 3, 7: 0, 8: 1, 9: 2}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_label(path: Path):
    rows = []
    for line_number, line in enumerate(path.read_text().splitlines(), 1):
        parts = line.split()
        if len(parts) != 5:
            raise ValueError(f"{path}:{line_number}: expected five YOLO fields")
        class_id = int(parts[0])
        values = tuple(map(float, parts[1:]))
        if not 0 <= class_id < len(MASTER_NAMES):
            raise ValueError(f"{path}:{line_number}: class {class_id} is outside 0..9")
        if not all(0.0 <= value <= 1.0 for value in values):
            raise ValueError(f"{path}:{line_number}: coordinates must be normalized")
        if values[2] <= 0 or values[3] <= 0:
            raise ValueError(f"{path}:{line_number}: box width/height must be positive")
        rows.append((class_id, parts[1:]))
    return rows


def write_yaml(path: Path, dataset: Path, names):
    lines = [f"path: {dataset}", "train: images/train", "val: images/val", "test: images/test", "names:"]
    lines.extend(f"  {index}: {name}" for index, name in enumerate(names))
    path.write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--master", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-boxes", type=int, default=150)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite {args.output}")

    nav = args.output / "nav_dataset"
    dock = args.output / "dock_dataset"
    master_counts = Counter()
    derived_counts = {"nav": Counter(), "dock": Counter()}
    image_counts = Counter()
    for split in ("train", "val", "test"):
        for dataset in (nav, dock):
            (dataset / "images" / split).mkdir(parents=True)
            (dataset / "labels" / split).mkdir(parents=True)
        source_images = args.master / "images" / split
        source_labels = args.master / "labels" / split
        if not source_images.is_dir():
            continue
        for image in sorted(source_images.iterdir()):
            if image.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            label = source_labels / f"{image.stem}.txt"
            if not label.is_file():
                raise FileNotFoundError(label)
            rows = parse_label(label)
            image_counts[split] += 1
            for class_id, _ in rows:
                master_counts[class_id] += 1
            for target, mapping, key in ((nav, NAV_MAP, "nav"), (dock, DOCK_MAP, "dock")):
                shutil.copy2(image, target / "images" / split / image.name)
                output_rows = []
                for class_id, values in rows:
                    if class_id not in mapping:
                        continue
                    mapped = mapping[class_id]
                    output_rows.append(f"{mapped} {' '.join(values)}")
                    derived_counts[key][mapped] += 1
                (target / "labels" / split / label.name).write_text(
                    "\n".join(output_rows) + ("\n" if output_rows else "")
                )

    write_yaml(args.output / "nav.yaml", nav.resolve(), NAV_NAMES)
    write_yaml(args.output / "dock.yaml", dock.resolve(), DOCK_NAMES)
    missing = [MASTER_NAMES[index] for index in range(10) if master_counts[index] < args.min_boxes]
    summary = {
        "master": str(args.master.resolve()),
        "images": dict(image_counts),
        "master_counts": {MASTER_NAMES[i]: master_counts[i] for i in range(10)},
        "derived_counts": {
            "nav": {NAV_NAMES[i]: derived_counts["nav"][i] for i in range(len(NAV_NAMES))},
            "dock": {DOCK_NAMES[i]: derived_counts["dock"][i] for i in range(len(DOCK_NAMES))},
        },
        "minimum_boxes_per_master_class": args.min_boxes,
        "classes_below_minimum": missing,
        "source_modified": False,
    }
    (args.output / "dataset_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

