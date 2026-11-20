#!/usr/bin/env python3
"""Build a reviewed 0-9 YOLO detector dataset from buoy crops.

The existing digit detector supplies box geometry only.  The manually reviewed
buoy label supplies the correct digit class, so every accepted box inside that
buoy is remapped to the reviewed digit.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import random
import shutil
from collections import Counter, defaultdict
from pathlib import Path

import cv2


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def copy_file(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def link_or_copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def nms(boxes, threshold=0.50, limit=3):
    kept = []
    for item in sorted(boxes, key=lambda item: item[0], reverse=True):
        if all(iou(item[1], old[1]) < threshold for old in kept):
            kept.append(item)
        if len(kept) >= limit:
            break
    return kept


def load_yolo(path: Path):
    rows = []
    if not path.exists():
        return rows
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) >= 5:
            rows.append([float(value) for value in parts])
    return rows


def class_counts(label_dir: Path) -> Counter:
    counts = Counter()
    for path in label_dir.glob("*.txt"):
        for line in path.read_text(encoding="utf-8").splitlines():
            if line.strip():
                counts[int(line.split()[0])] += 1
    return counts


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robotx-root", type=Path,
                        default=Path("/home/robotx/Robotx_train"))
    parser.add_argument("--base-dataset", type=Path, default=Path(
        "/home/robotx/buoy_actual/buoy_project/"
        "retrain_robotx_do_not_touch_20261110/merged/digit_dataset"))
    parser.add_argument("--output", type=Path,
                        default=Path("/home/robotx/Robotx_train/number_detector_dataset_v1"))
    parser.add_argument("--min-proposal-conf", type=float, default=0.08)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    root = args.robotx_root
    output = args.output
    if output.exists():
        shutil.rmtree(output)
    for split in ("train", "val", "test"):
        (output / "images" / split).mkdir(parents=True, exist_ok=True)
        (output / "labels" / split).mkdir(parents=True, exist_ok=True)

    # Preserve the older detector data and its established split.
    for split in ("train", "val", "test"):
        src_images = args.base_dataset / "images" / split
        src_labels = args.base_dataset / "labels" / split
        for image_path in src_images.iterdir():
            if image_path.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            stem = f"base_{image_path.stem}"
            copy_file(image_path, output / "images" / split / f"{stem}{image_path.suffix.lower()}")
            label_path = src_labels / f"{image_path.stem}.txt"
            target_label = output / "labels" / split / f"{stem}.txt"
            if label_path.exists():
                copy_file(label_path, target_label)
            else:
                target_label.write_text("", encoding="utf-8")

    split_for_image = {}
    detector_images = root / "two_weight_dataset_v2" / "buoy_detector" / "images"
    for split in ("train", "val", "test"):
        for image_path in (detector_images / split).iterdir():
            split_for_image[image_path.name] = split

    manual = {
        row["id"]: row["label"].strip()
        for row in csv.DictReader((root / "manual_marker_labels.csv").open(encoding="utf-8"))
    }
    candidates = list(csv.DictReader(
        (root / "review_v1" / "candidates.csv").open(encoding="utf-8")
    ))

    proposal_cache = {}
    image_cache = {}
    new_samples = defaultdict(list)
    skipped_no_box = Counter()

    for row in candidates:
        digit = manual.get(row["id"], "")
        if len(digit) != 1 or digit not in "0123456789":
            continue
        image_name = row["image"]
        split = split_for_image.get(image_name)
        if split is None:
            continue
        image_path = root / "images" / image_name
        if image_name not in image_cache:
            image_cache[image_name] = cv2.imread(str(image_path))
        image = image_cache[image_name]
        if image is None:
            continue
        height, width = image.shape[:2]
        x1, y1, x2, y2 = (int(float(row[key])) for key in ("x1", "y1", "x2", "y2"))
        buoy_w, buoy_h = x2 - x1, y2 - y1
        pad_x, pad_y = max(2, int(buoy_w * 0.08)), max(2, int(buoy_h * 0.08))
        cx1, cy1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
        cx2, cy2 = min(width, x2 + pad_x), min(height, y2 + pad_y)

        if image_name not in proposal_cache:
            proposal_cache[image_name] = load_yolo(
                root / "proposals" / "digits_old" / "labels" / f"{Path(image_name).stem}.txt"
            )
        hits = []
        for proposal in proposal_cache[image_name]:
            _, xc, yc, bw, bh = proposal[:5]
            conf = proposal[5] if len(proposal) > 5 else 1.0
            if conf < args.min_proposal_conf:
                continue
            px, py = xc * width, yc * height
            if not (x1 <= px <= x2 and y1 <= py <= y2):
                continue
            bx1 = max(float(cx1), (xc - bw / 2) * width)
            by1 = max(float(cy1), (yc - bh / 2) * height)
            bx2 = min(float(cx2), (xc + bw / 2) * width)
            by2 = min(float(cy2), (yc + bh / 2) * height)
            if bx2 - bx1 >= 3 and by2 - by1 >= 3:
                hits.append((conf, (bx1, by1, bx2, by2)))
        hits = nms(hits)
        if not hits:
            skipped_no_box[int(digit)] += 1
            continue

        crop = image[cy1:cy2, cx1:cx2]
        crop_h, crop_w = crop.shape[:2]
        stem = f"new_{row['id']}__{Path(image_name).stem}"
        image_out = output / "images" / split / f"{stem}.jpg"
        cv2.imwrite(str(image_out), crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
        lines = []
        for _, (bx1, by1, bx2, by2) in hits:
            rx1, ry1 = bx1 - cx1, by1 - cy1
            rx2, ry2 = bx2 - cx1, by2 - cy1
            xc = ((rx1 + rx2) / 2) / crop_w
            yc = ((ry1 + ry2) / 2) / crop_h
            bw = (rx2 - rx1) / crop_w
            bh = (ry2 - ry1) / crop_h
            lines.append(f"{digit} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")
        (output / "labels" / split / f"{stem}.txt").write_text(
            "\n".join(lines) + "\n", encoding="utf-8"
        )
        new_samples[(split, int(digit))].append(stem)

    # Bring sparse training classes up to 40 boxes by hard-linking reviewed crops.
    rng = random.Random(args.seed)
    train_counts = class_counts(output / "labels" / "train")
    for class_id in range(10):
        sources = new_samples.get(("train", class_id), [])
        duplicate_index = 0
        while train_counts[class_id] < 40 and sources:
            source_stem = rng.choice(sources)
            duplicate_index += 1
            target_stem = f"aug_c{class_id}_{duplicate_index:03d}_{source_stem}"
            source_image = output / "images" / "train" / f"{source_stem}.jpg"
            source_label = output / "labels" / "train" / f"{source_stem}.txt"
            link_or_copy(source_image, output / "images" / "train" / f"{target_stem}.jpg")
            link_or_copy(source_label, output / "labels" / "train" / f"{target_stem}.txt")
            for line in source_label.read_text(encoding="utf-8").splitlines():
                if line.strip():
                    train_counts[int(line.split()[0])] += 1

    yaml_text = f"""path: {output}
train: images/train
val: images/val
test: images/test
names:
  0: '0'
  1: '1'
  2: '2'
  3: '3'
  4: '4'
  5: '5'
  6: '6'
  7: '7'
  8: '8'
  9: '9'
"""
    (output / "data.yaml").write_text(yaml_text, encoding="utf-8")

    summary = {
        "new_reviewed_samples": {
            split: {str(class_id): len(new_samples.get((split, class_id), []))
                    for class_id in range(10)}
            for split in ("train", "val", "test")
        },
        "skipped_reviewed_buoys_without_digit_box": dict(sorted(skipped_no_box.items())),
        "final": {},
    }
    for split in ("train", "val", "test"):
        summary["final"][split] = {
            "images": len(list((output / "images" / split).glob("*"))),
            "boxes_by_class": dict(sorted(class_counts(output / "labels" / split).items())),
        }
    (output / "dataset_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
