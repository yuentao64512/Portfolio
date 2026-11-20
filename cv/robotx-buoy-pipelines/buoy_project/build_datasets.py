#!/usr/bin/env python3
"""Build buoy-only and digit-crop datasets without editing full labels."""

from __future__ import annotations

import argparse
import json
import shutil
from collections import Counter
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parent
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def read_labels(path: Path, width: int, height: int):
    rows = []
    if not path.is_file():
        return rows
    for number, line in enumerate(path.read_text().splitlines(), 1):
        parts = line.split()
        if len(parts) != 5:
            raise ValueError(f"{path}:{number}: expected five fields")
        class_id = int(parts[0])
        x, y, w, h = map(float, parts[1:])
        if not 0 <= class_id <= 10 or not all(0 <= value <= 1 for value in (x, y, w, h)):
            raise ValueError(f"{path}:{number}: invalid label")
        rows.append((class_id, (x-w/2)*width, (y-h/2)*height,
                     (x+w/2)*width, (y+h/2)*height))
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--full", type=Path, default=ROOT / "dataset_full")
    parser.add_argument("--buoy", type=Path, default=ROOT / "buoy_dataset")
    parser.add_argument("--digit", type=Path, default=ROOT / "digit_dataset")
    parser.add_argument("--padding", type=float, default=0.10)
    args = parser.parse_args()
    if args.buoy.exists() or args.digit.exists():
        raise SystemExit("Refusing to overwrite an existing derived dataset")

    summary = {"images": Counter(), "buoys": Counter(), "digits": Counter()}
    for split in ("train", "val", "test"):
        for root in (args.buoy, args.digit):
            (root / "images" / split).mkdir(parents=True, exist_ok=False)
            (root / "labels" / split).mkdir(parents=True, exist_ok=False)

        source_images = args.full / "images" / split
        if not source_images.is_dir():
            continue
        for image_path in sorted(source_images.iterdir()):
            if image_path.suffix.lower() not in IMAGE_SUFFIXES:
                continue
            image = cv2.imread(str(image_path))
            if image is None:
                raise RuntimeError(f"Cannot read {image_path}")
            height, width = image.shape[:2]
            rows = read_labels(args.full / "labels" / split / f"{image_path.stem}.txt", width, height)
            buoys = [row for row in rows if row[0] == 0]
            digits = [row for row in rows if row[0] >= 1]

            shutil.copy2(image_path, args.buoy / "images" / split / image_path.name)
            buoy_lines = []
            for _, x1, y1, x2, y2 in buoys:
                buoy_lines.append(
                    f"0 {(x1+x2)/2/width:.6f} {(y1+y2)/2/height:.6f} "
                    f"{(x2-x1)/width:.6f} {(y2-y1)/height:.6f}"
                )
            (args.buoy / "labels" / split / f"{image_path.stem}.txt").write_text(
                "\n".join(buoy_lines) + ("\n" if buoy_lines else "")
            )
            summary["images"][split] += 1
            summary["buoys"][split] += len(buoys)

            for index, (_, x1, y1, x2, y2) in enumerate(buoys):
                pad_w, pad_h = (x2-x1)*args.padding, (y2-y1)*args.padding
                crop_x1, crop_y1 = max(0, int(x1-pad_w)), max(0, int(y1-pad_h))
                crop_x2, crop_y2 = min(width, int(x2+pad_w)), min(height, int(y2+pad_h))
                crop_w, crop_h = crop_x2-crop_x1, crop_y2-crop_y1
                if crop_w < 8 or crop_h < 8:
                    continue
                digit_lines = []
                for class_id, a, b, c, d in digits:
                    center_x, center_y = (a+c)/2, (b+d)/2
                    if not (crop_x1 < center_x < crop_x2 and crop_y1 < center_y < crop_y2):
                        continue
                    a, b = max(a, crop_x1), max(b, crop_y1)
                    c, d = min(c, crop_x2), min(d, crop_y2)
                    digit_lines.append(
                        f"{class_id-1} {((a+c)/2-crop_x1)/crop_w:.6f} "
                        f"{((b+d)/2-crop_y1)/crop_h:.6f} {(c-a)/crop_w:.6f} {(d-b)/crop_h:.6f}"
                    )
                    summary["digits"][str(class_id-1)] += 1
                name = f"{image_path.stem}_b{index:03d}"
                crop = image[crop_y1:crop_y2, crop_x1:crop_x2]
                if not cv2.imwrite(str(args.digit / "images" / split / f"{name}.jpg"), crop):
                    raise RuntimeError(f"Failed to write crop {name}")
                (args.digit / "labels" / split / f"{name}.txt").write_text(
                    "\n".join(digit_lines) + ("\n" if digit_lines else "")
                )

    serializable = {key: dict(value) for key, value in summary.items()}
    (ROOT / "build_summary.json").write_text(json.dumps(serializable, indent=2) + "\n")
    print(json.dumps(serializable, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
