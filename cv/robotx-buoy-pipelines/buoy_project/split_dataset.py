#!/usr/bin/env python3
"""Split paired full-frame labels by source group without leakage."""

from __future__ import annotations

import argparse
import json
import random
import re
import shutil
from collections import defaultdict
from pathlib import Path


ROOT = Path(__file__).resolve().parent
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def group_name(path: Path) -> str:
    match = re.match(r"((?:vid|img)-.+?)__", path.stem)
    return match.group(1) if match else path.stem


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=ROOT / "labelled")
    parser.add_argument("--output", type=Path, default=ROOT / "dataset_full")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")

    groups: dict[str, list[Path]] = defaultdict(list)
    for image in sorted(args.source.iterdir() if args.source.is_dir() else []):
        if image.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        label = image.with_suffix(".txt")
        if not label.is_file():
            print(f"SKIP missing label: {image.name}")
            continue
        groups[group_name(image)].append(image)
    if len(groups) < 3:
        raise SystemExit("At least three independent source groups are required")

    keys = list(groups)
    random.Random(args.seed).shuffle(keys)
    targets = {"train": 0.80, "val": 0.10, "test": 0.10}
    total = sum(len(groups[key]) for key in keys)
    assignments: dict[str, list[str]] = {split: [] for split in targets}
    counts = {split: 0 for split in targets}

    # Give val and test one group first, then greedily approach target counts.
    assignments["val"].append(keys.pop())
    assignments["test"].append(keys.pop())
    counts["val"] = sum(len(groups[k]) for k in assignments["val"])
    counts["test"] = sum(len(groups[k]) for k in assignments["test"])
    for key in keys:
        deficits = {
            split: targets[split] * total - counts[split]
            for split in ("train", "val", "test")
        }
        split = max(deficits, key=deficits.get)
        assignments[split].append(key)
        counts[split] += len(groups[key])

    for split, split_groups in assignments.items():
        image_dir = args.output / "images" / split
        label_dir = args.output / "labels" / split
        image_dir.mkdir(parents=True, exist_ok=False)
        label_dir.mkdir(parents=True, exist_ok=False)
        for key in split_groups:
            for image in groups[key]:
                shutil.copy2(image, image_dir / image.name)
                shutil.copy2(image.with_suffix(".txt"), label_dir / f"{image.stem}.txt")

    summary = {"seed": args.seed, "counts": counts, "groups": assignments}
    (args.output / "split_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
