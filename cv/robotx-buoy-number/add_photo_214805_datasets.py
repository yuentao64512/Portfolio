#!/usr/bin/env python3
"""Add photo_2026-11-12_21-48-05 to all three reviewed datasets."""

from __future__ import annotations

import json
import os
import shutil
from collections import Counter
from pathlib import Path

import cv2


ROOT = Path("/home/robotx/Robotx_train")
PHOTO = ROOT / "photo_2026-11-12_21-48-05.jpg"
OUT = ROOT / "photo_214805_training_v1"

# Whole-object boxes, manually checked against the 1280x960 source.
# Detector class 0=number_buoy, 1=boundary.
BUOYS = {
    "D": (1, (46, 548, 122, 597)),
    "4": (0, (142, 574, 212, 636)),
    "2": (0, (120, 610, 214, 687)),
    "7": (0, (300, 562, 379, 630)),
    "5": (0, (335, 600, 415, 670)),
    "3": (0, (303, 658, 403, 733)),
    "0": (0, (237, 696, 342, 781)),
    "8": (0, (544, 556, 623, 636)),
    "6": (0, (518, 612, 602, 695)),
    "9": (0, (795, 552, 881, 650)),
    "1": (0, (702, 658, 800, 750)),
}

# Visible diamond-marker boxes in full-image pixel coordinates.
DIGITS = {
    "4": [(163, 597, 190, 623)],
    "2": [(144, 645, 164, 668), (165, 644, 189, 671)],
    "7": [(329, 585, 353, 609)],
    "5": [(359, 624, 389, 654)],
    "3": [(331, 681, 363, 714), (363, 684, 383, 713)],
    "0": [(267, 728, 301, 763), (300, 731, 320, 761)],
    "8": [(566, 587, 592, 613)],
    "6": [(544, 648, 575, 680), (574, 650, 595, 680)],
    "9": [(823, 606, 851, 633)],
    "1": [(731, 698, 762, 730), (760, 700, 782, 730)],
}


def clone_tree_hardlinks(source: Path, target: Path) -> None:
    def copy_or_link(src, dst):
        src_path, dst_path = Path(src), Path(dst)
        if src_path.suffix == ".cache":
            return str(dst_path)
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(src_path, dst_path)
        except OSError:
            shutil.copy2(src_path, dst_path)
        return str(dst_path)

    shutil.copytree(source, target, copy_function=copy_or_link,
                    ignore=shutil.ignore_patterns("*.cache"))


def yolo_line(class_id: int, box, width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    xc = ((x1 + x2) / 2) / width
    yc = ((y1 + y2) / 2) / height
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return f"{class_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}"


def hardlink(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        target.unlink()
    os.link(source, target)


def main() -> None:
    if OUT.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {OUT}")
    image = cv2.imread(str(PHOTO))
    if image is None:
        raise SystemExit(f"Cannot read {PHOTO}")
    height, width = image.shape[:2]
    if (width, height) != (1280, 960):
        raise SystemExit(f"Unexpected photo dimensions: {width}x{height}")

    detector_ds = OUT / "buoy_detector_dataset"
    classifier_ds = OUT / "marker_classifier_dataset"
    number_ds = OUT / "number_detector_dataset"
    clone_tree_hardlinks(ROOT / "two_weight_dataset_v2" / "buoy_detector", detector_ds)
    clone_tree_hardlinks(ROOT / "two_weight_dataset_v2" / "marker_classifier", classifier_ds)
    clone_tree_hardlinks(ROOT / "number_detector_dataset_v1", number_ds)

    detector_yaml = f"""path: {detector_ds}
train: images/train
val: images/val
test: images/test
names:
  0: number_buoy
  1: boundary
"""
    (detector_ds / "data.yaml").write_text(detector_yaml, encoding="utf-8")
    number_yaml = f"""path: {number_ds}
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
    (number_ds / "data.yaml").write_text(number_yaml, encoding="utf-8")

    # Repeat the hard example three times so it is not drowned out by the older data.
    detector_label = "\n".join(
        yolo_line(class_id, box, width, height)
        for _, (class_id, box) in BUOYS.items()
    ) + "\n"
    for repeat in range(3):
        stem = f"photo_214805_r{repeat}"
        target_image = detector_ds / "images" / "train" / f"{stem}.jpg"
        hardlink(PHOTO, target_image)
        (detector_ds / "labels" / "train" / f"{stem}.txt").write_text(
            detector_label, encoding="utf-8"
        )

    crop_dir = OUT / "reviewed_crops"
    crop_dir.mkdir(parents=True)
    crop_summary = {}
    preview = image.copy()
    for marker, (_, (x1, y1, x2, y2)) in BUOYS.items():
        crop = image[y1:y2, x1:x2]
        crop_path = crop_dir / f"{marker}.jpg"
        cv2.imwrite(str(crop_path), crop, [cv2.IMWRITE_JPEG_QUALITY, 96])
        crop_summary[marker] = {"buoy_box": [x1, y1, x2, y2]}
        object_colour = (255, 0, 0) if marker == "D" else (0, 0, 255)
        cv2.rectangle(preview, (x1, y1), (x2, y2), object_colour, 2)
        cv2.putText(preview, "boundary D" if marker == "D" else f"number_buoy {marker}",
                    (x1, max(18, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.48, object_colour, 2, cv2.LINE_AA)

        # Classifier sees the whole buoy crop.  Repeat for augmented fine-tuning.
        for repeat in range(4):
            target = classifier_ds / "train" / marker / f"photo_214805_{marker}_r{repeat}.jpg"
            hardlink(crop_path, target)

        if marker == "D":
            continue

        crop_h, crop_w = crop.shape[:2]
        digit_lines = []
        for digit_box in DIGITS[marker]:
            dx1, dy1, dx2, dy2 = digit_box
            cv2.rectangle(preview, (dx1, dy1), (dx2, dy2), (0, 255, 0), 2)
            local_box = (dx1 - x1, dy1 - y1, dx2 - x1, dy2 - y1)
            digit_lines.append(yolo_line(int(marker), local_box, crop_w, crop_h))
        crop_summary[marker]["digit_boxes"] = [list(box) for box in DIGITS[marker]]
        for repeat in range(4):
            stem = f"photo_214805_{marker}_r{repeat}"
            hardlink(crop_path, number_ds / "images" / "train" / f"{stem}.jpg")
            (number_ds / "labels" / "train" / f"{stem}.txt").write_text(
                "\n".join(digit_lines) + "\n", encoding="utf-8"
            )

    cv2.imwrite(str(OUT / "manual_labels_preview.jpg"), preview,
                [cv2.IMWRITE_JPEG_QUALITY, 95])

    summary = {
        "source": str(PHOTO),
        "dimensions": [width, height],
        "whole_buoys": len(BUOYS),
        "number_buoys": sum(1 for value in BUOYS.values() if value[0] == 0),
        "boundary_buoys": sum(1 for value in BUOYS.values() if value[0] == 1),
        "visible_digit_boxes": sum(len(value) for value in DIGITS.values()),
        "classes": dict(Counter(key for key in BUOYS)),
        "crops": crop_summary,
    }
    (OUT / "photo_label_summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
