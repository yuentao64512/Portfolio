#!/usr/bin/env python3
"""Build a balanced continuation dataset from the reviewed v16 dataset.

The source is read-only.  Images are hard-linked when possible.  One complete
legacy recording group is moved from train into validation, obstacle-only and
empty images are sampled by recording group, and minority-class images are
repeated under unique names so normal YOLO augmentation produces different
views without fabricating any boxes.
"""

from __future__ import annotations

import argparse
import json
import os
import random
import shutil
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import yaml


IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
HOLDOUT_PREFIX = "v14_train_supplement_val_new"


@dataclass(frozen=True)
class Sample:
    stem: str
    image: Path
    label: Path
    counts: Counter


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--minority-target", type=int, default=180)
    parser.add_argument("--obstacle-per-group", type=int, default=45)
    parser.add_argument("--empty-per-group", type=int, default=10)
    return parser.parse_args()


def link_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source.resolve(), target)
    except OSError:
        shutil.copy2(source.resolve(), target)


def image_for(source: Path, split: str, stem: str) -> Path | None:
    for suffix in IMAGE_SUFFIXES:
        candidate = source / "images" / split / (stem + suffix)
        if candidate.exists():
            return candidate
    return None


def class_counts(label: Path) -> Counter:
    counts = Counter()
    for line in label.read_text().splitlines():
        parts = line.split()
        if len(parts) >= 5:
            counts[int(parts[0])] += 1
    return counts


def recording_group(stem: str) -> str:
    keys = (
        "v14_train_legacy_train_IMG_3865",
        "v14_train_legacy_train_IMG_3866",
        "v14_train_legacy_train_IMG_3867",
        "v14_train_legacy_train_IMG_7695",
        "v14_train_legacy_train_WhatsApp_Video_2026-11-08_at_4_51_46_",
        "v14_train_legacy_train_WhatsApp_Video_2026-11-08_at_4_51_50_",
        "v14_train_legacy_train_WhatsApp_Video_2026-11-08_at_4_51_51_",
        "v14_train_legacy_train_WhatsApp_Video_2026-11-08_at_4_51_53_",
        "v14_train_legacy_train_still",
        "v14_train_new_20261111_112939",
        "v14_train_new_20261111_115135",
        "v14_train_supplement_train_new",
        "v14_train_supplement_val_new",
        "v14_train_supplement_test_new",
        "v14_train_synth_red",
        "v14_train_synth_green",
        "v14_train_synth_blue",
        "new_20261112_111929",
        "new_20261112_113136",
    )
    return next((key for key in keys if stem.startswith(key)), stem.split("__")[0])


def read_split(source: Path, split: str) -> list[Sample]:
    samples = []
    for label in sorted((source / "labels" / split).glob("*.txt")):
        image = image_for(source, split, label.stem)
        if image is None:
            raise FileNotFoundError(f"No image for {label}")
        samples.append(Sample(label.stem, image, label, class_counts(label)))
    return samples


def add_sample(sample: Sample, output: Path, split: str, stem: str,
               totals: Counter) -> None:
    link_or_copy(sample.image, output / "images" / split / (stem + sample.image.suffix.lower()))
    link_or_copy(sample.label, output / "labels" / split / (stem + ".txt"))
    totals.update(sample.counts)


def stratified(items: list[Sample], cap: int, rng: random.Random) -> list[Sample]:
    groups = defaultdict(list)
    for sample in items:
        groups[recording_group(sample.stem)].append(sample)
    chosen = []
    for group in sorted(groups):
        values = groups[group]
        rng.shuffle(values)
        group_cap = cap * 2 if "IMG_7695" in group else cap
        chosen.extend(values[:group_cap])
    return chosen


def copy_validation(source: Path, output: Path, split: str, prefix: str,
                    totals: Counter) -> None:
    for sample in read_split(source, split):
        add_sample(sample, output, "val_combined", f"{prefix}__{sample.stem}", totals)


def main():
    args = arguments()
    rng = random.Random(20261112)
    if args.output.exists():
        raise FileExistsError(args.output)

    config = yaml.safe_load((args.source / "data.yaml").read_text())
    names = {int(k): v for k, v in config["names"].items()}
    train = read_split(args.source, "train")
    holdout = [s for s in train if s.stem.startswith(HOLDOUT_PREFIX)]
    candidates = [s for s in train if not s.stem.startswith(HOLDOUT_PREFIX)]

    foreground = [s for s in candidates if any(c < 7 for c in s.counts)]
    obstacle_only = [s for s in candidates if s.counts and not any(c < 7 for c in s.counts)]
    empty = [s for s in candidates if not s.counts]
    selected = foreground + stratified(obstacle_only, args.obstacle_per_group, rng)
    selected += stratified(empty, args.empty_per_group, rng)

    # De-duplicate while preserving deterministic order.
    selected_by_stem = {s.stem: s for s in selected}
    selected = [selected_by_stem[k] for k in sorted(selected_by_stem)]
    train_totals = Counter()
    for sample in selected:
        add_sample(sample, args.output, "train", sample.stem, train_totals)

    # Repeating a source image is deliberate: YOLO applies random geometric and
    # photometric augmentation each time. Hue remains disabled by the trainer.
    for class_id in range(7):
        pool = [s for s in selected if s.counts[class_id]]
        clean_pool = [s for s in pool if not s.counts[7]]
        # Prefer objects that do not share the frame with an obstacle label;
        # otherwise oversampling a rare colour also recreates the original
        # obstacle imbalance.
        if clean_pool:
            pool = clean_pool
        if not pool:
            continue
        rng.shuffle(pool)
        index = 0
        while train_totals[class_id] < args.minority_target:
            sample = pool[index % len(pool)]
            stem = f"repeat_c{class_id}_{index:05d}__{sample.stem}"
            add_sample(sample, args.output, "train", stem, train_totals)
            index += 1

    val_totals = Counter()
    for source_split, prefix in (("test_new", "new_depth"), ("val", "legacy_val"),
                                 ("test_old", "legacy_test")):
        copy_validation(args.source, args.output, source_split, prefix, val_totals)
    for sample in holdout:
        add_sample(sample, args.output, "val_combined", f"heldout_session__{sample.stem}", val_totals)

    # Keep the untouched newest session as the formal test split.
    test_totals = Counter()
    for sample in read_split(args.source, "test_new"):
        add_sample(sample, args.output, "test_new", sample.stem, test_totals)

    yaml_text = "\n".join([
        f"path: {args.output}",
        "train: images/train",
        "val: images/val_combined",
        "test: images/test_new",
        "names:",
        *[f"  {key}: {names[key]}" for key in sorted(names)],
        "",
    ])
    (args.output / "data.yaml").write_text(yaml_text)
    manifest = {
        "source": str(args.source),
        "holdout_prefix": HOLDOUT_PREFIX,
        "train_images": len(list((args.output / "images" / "train").glob("*"))),
        "val_images": len(list((args.output / "images" / "val_combined").glob("*"))),
        "test_images": len(list((args.output / "images" / "test_new").glob("*"))),
        "train_boxes": dict(sorted(train_totals.items())),
        "val_boxes": dict(sorted(val_totals.items())),
        "test_boxes": dict(sorted(test_totals.items())),
        "source_train_images": len(train),
        "selected_original_train_images": len(selected),
        "excluded_holdout_images": len(holdout),
        "notes": [
            "No source files or labels were modified.",
            "Validation groups are excluded from balanced train.",
            "Repeated images retain reviewed boxes and rely on runtime augmentation.",
        ],
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
