#!/usr/bin/env python3
"""Combine 400 ground and 400 waterfront examples without name collisions."""

import csv
import shutil
from pathlib import Path

import cv2
import numpy as np


PROJECT = Path("/home/robotx/pipe_cones")
OUTPUT = PROJECT / "dataset_combined_800"
SOURCES = (
    ("ground", PROJECT / "dataset_ground_400"),
    ("water", PROJECT / "dataset_recorded_400"),
)


def read_box(label_path, width, height):
    values = label_path.read_text().split()
    if len(values) != 5 or values[0] != "0":
        raise RuntimeError(f"Expected one class-0 box in {label_path}")
    xc, yc, bw, bh = map(float, values[1:])
    return xc * width, yc * height, bw * width, bh * height


def write_box(label_path, box, width, height):
    xc, yc, bw, bh = box
    label_path.write_text(
        f"0 {xc / width:.8f} {yc / height:.8f} {bw / width:.8f} {bh / height:.8f}\n"
    )


def background_suppressed(image, box):
    """Blur scene detail while preserving a softly feathered object region."""
    xc, yc, bw, bh = box
    height, width = image.shape[:2]
    x1 = max(0, int(xc - 0.65 * bw))
    x2 = min(width, int(xc + 0.65 * bw))
    y1 = max(0, int(yc - 0.65 * bh))
    y2 = min(height, int(yc + 0.65 * bh))
    mask = np.zeros((height, width), dtype=np.float32)
    mask[y1:y2, x1:x2] = 1.0
    mask = cv2.GaussianBlur(mask, (0, 0), sigmaX=18, sigmaY=18)[:, :, None]
    blurred = cv2.GaussianBlur(image, (0, 0), sigmaX=18, sigmaY=18)
    return (image * mask + blurred * (1.0 - mask)).astype(np.uint8)


def context_crop(image, box):
    """Retain varied real context but enlarge small cones relative to the frame."""
    xc, yc, bw, bh = box
    height, width = image.shape[:2]
    side = min(width, height, max(5.0 * max(bw, bh), 0.42 * min(width, height)))
    x1 = min(max(0.0, xc - side / 2), width - side)
    y1 = min(max(0.0, yc - side / 2), height - side)
    x2, y2 = x1 + side, y1 + side
    crop = image[int(y1):int(y2), int(x1):int(x2)]
    new_box = (xc - x1, yc - y1, bw, bh)
    return crop, new_box


def main():
    if OUTPUT.exists():
        shutil.rmtree(OUTPUT)
    for split in ("train", "val", "test"):
        (OUTPUT / "images" / split).mkdir(parents=True)
        (OUTPUT / "labels" / split).mkdir(parents=True)

    rows = []
    for domain, source in SOURCES:
        for split in ("train", "val", "test"):
            images = sorted((source / "images" / split).glob("*.jpg"))
            for index, image in enumerate(images):
                label = source / "labels" / split / f"{image.stem}.txt"
                if not label.exists():
                    raise RuntimeError(f"Missing label for {image}")
                stem = f"{domain}_{image.stem}"
                output_image = OUTPUT / "images" / split / f"{stem}.jpg"
                output_label = OUTPUT / "labels" / split / f"{stem}.txt"
                variant = "natural"
                if domain == "ground" and split == "train" and index % 3:
                    frame = cv2.imread(str(image))
                    if frame is None:
                        raise RuntimeError(f"Could not read {image}")
                    height, width = frame.shape[:2]
                    box = read_box(label, width, height)
                    if index % 3 == 1:
                        frame = background_suppressed(frame, box)
                        variant = "background_suppressed"
                    else:
                        frame, box = context_crop(frame, box)
                        height, width = frame.shape[:2]
                        variant = "context_crop"
                    if not cv2.imwrite(str(output_image), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                        raise RuntimeError(f"Could not write {output_image}")
                    write_box(output_label, box, width, height)
                else:
                    shutil.copy2(image, output_image)
                    shutil.copy2(label, output_label)
                rows.append({
                    "domain": domain, "split": split, "variant": variant,
                    "image": f"images/{split}/{stem}.jpg",
                })

    with (OUTPUT / "manifest.csv").open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=("domain", "split", "variant", "image"))
        writer.writeheader()
        writer.writerows(rows)
    counts = {split: sum(row["split"] == split for row in rows) for split in ("train", "val", "test")}
    domains = {domain: sum(row["domain"] == domain for row in rows) for domain, _ in SOURCES}
    if len(rows) != 800 or domains != {"ground": 400, "water": 400}:
        raise RuntimeError(f"Unexpected dataset counts: total={len(rows)}, domains={domains}")
    print(f"Created combined dataset: total={len(rows)}, domains={domains}, splits={counts}")


if __name__ == "__main__":
    main()
