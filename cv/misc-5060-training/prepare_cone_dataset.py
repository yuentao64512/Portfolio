#!/usr/bin/env python3
"""Extract and label orange/green cones from the supplied videos.

The labels are generated from color masks and then checked visually before
training.  It creates ordinary Ultralytics YOLO detection labels:
class 0 = orange_cone, class 1 = green_cone.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
from pathlib import Path

import cv2
import numpy as np


VIDEOS = [
    Path("/home/robotx/IMG_3148.MOV"),
    Path("/home/robotx/IMG_3149.MOV"),
    Path("/home/robotx/video_2026-09-15_16-59-47.mp4"),
    Path("/home/robotx/video_2026-09-15_16-59-52.mp4"),
]


def color_mask(image: np.ndarray, color: str) -> np.ndarray:
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    if color == "orange":
        mask = (((h <= 28) | (h >= 170)) & (s >= 80) & (v >= 55)).astype(np.uint8)
    else:
        # Pool water is also cyan/green, but is much brighter than the dark
        # green cone body. This value cap removes most of the water mass.
        mask = ((h >= 30) & (h <= 100) & (s >= 45) & (v >= 30) & (v <= 190)).astype(np.uint8)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def overlap_x(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    left = max(a[0], b[0])
    right = min(a[2], b[2])
    overlap = max(0, right - left)
    return overlap / max(1, min(a[2] - a[0], b[2] - b[0]))


def merge_boxes(boxes: list[tuple[int, int, int, int]]) -> list[tuple[int, int, int, int]]:
    changed = True
    while changed:
        changed = False
        result = []
        while boxes:
            current = boxes.pop()
            merged = True
            while merged:
                merged = False
                remaining = []
                for other in boxes:
                    vertical_gap = max(current[1], other[1]) - min(current[3], other[3])
                    close = vertical_gap <= max(80, int(0.7 * max(
                        current[3] - current[1], other[3] - other[1]
                    )))
                    if overlap_x(current, other) >= 0.25 and close:
                        current = (
                            min(current[0], other[0]), min(current[1], other[1]),
                            max(current[2], other[2]), max(current[3], other[3]),
                        )
                        merged = True
                        changed = True
                    else:
                        remaining.append(other)
                boxes = remaining
            result.append(current)
        boxes = result
    return boxes


def boxes_for_color(image: np.ndarray, color: str) -> list[tuple[int, int, int, int]]:
    mask = color_mask(image, color)
    h, w = mask.shape
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    min_area = max(70, int(h * w * 0.000025))
    candidates = []
    for i in range(1, count):
        x, y, bw, bh, area = stats[i]
        if area < min_area or bw < 6 or bh < 8:
            continue
        if bw > 0.80 * w or bh > 0.85 * h:
            continue
        candidates.append((int(x), int(y), int(x + bw), int(y + bh)))
    boxes = merge_boxes(candidates)
    scored = []
    for x1, y1, x2, y2 in boxes:
        bw, bh = x2 - x1, y2 - y1
        # The actual cones occupy a meaningful portion of the lower image.
        # Small upper-frame color blobs are usually trees, clothing, or water
        # reflections and should not become training labels.
        if y1 < int(0.18 * h):
            continue
        if bw * bh < 5000 or bw / max(1, bh) > 2.2:
            continue
        pad_x = max(2, int(0.04 * bw))
        pad_y = max(2, int(0.04 * bh))
        box = (max(0, x1 - pad_x), max(0, y1 - pad_y),
               min(w, x2 + pad_x), min(h, y2 + pad_y))
        roi = mask[box[1]:box[3], box[0]:box[2]]
        row_widths = np.count_nonzero(roi, axis=1)
        nonempty = row_widths[row_widths > 0]
        if len(nonempty) < 3:
            continue
        split = max(1, len(nonempty) // 3)
        top_width = float(np.mean(nonempty[:split]))
        bottom_width = float(np.mean(nonempty[-split:]))
        taper = bottom_width / max(1.0, top_width)
        aspect = bh / max(1.0, bw)
        density = float(np.count_nonzero(roi)) / max(1, roi.size)
        # Cones are vertically elongated, taper narrower at the top, and have
        # a relatively dense colored silhouette. Background clothing/equipment
        # generally scores lower on this shape test.
        # Area and verticality dominate; the small shape term breaks ties
        # between similarly sized background components.
        shape_score = (bw * bh) * min(2.5, max(0.5, aspect)) * min(2.5, max(0.5, taper)) * (0.5 + density)
        scored.append((shape_score, box))
    scored.sort(key=lambda item: item[0], reverse=True)
    return [box for _, box in scored[:1]]


def extract_boxes(image: np.ndarray, allowed_colors=None) -> list[tuple[int, int, int, int, int]]:
    labels = []
    colors = ("orange", "green") if allowed_colors is None else allowed_colors
    for color in colors:
        class_id = 0 if color == "orange" else 1
        for x1, y1, x2, y2 in boxes_for_color(image, color):
            labels.append((class_id, x1, y1, x2, y2))
    return labels


def write_yolo_label(path: Path, labels, width: int, height: int) -> None:
    rows = []
    for class_id, x1, y1, x2, y2 in labels:
        xc = ((x1 + x2) / 2) / width
        yc = ((y1 + y2) / 2) / height
        bw = (x2 - x1) / width
        bh = (y2 - y1) / height
        rows.append(f"{class_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}")
    path.write_text("\n".join(rows) + ("\n" if rows else ""), encoding="utf-8")


def far_variant(image, labels, rng: random.Random):
    h, w = image.shape[:2]
    scale = rng.uniform(0.38, 0.72)
    nw, nh = max(8, int(w * scale)), max(8, int(h * scale))
    small = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = cv2.copyMakeBorder(
        small, (h - nh) // 2, h - nh - (h - nh) // 2,
        (w - nw) // 2, w - nw - (w - nw) // 2,
        cv2.BORDER_REPLICATE,
    )
    if rng.random() < 0.8:
        sigma = rng.uniform(0.4, 1.5)
        canvas = cv2.GaussianBlur(canvas, (0, 0), sigma)
    if rng.random() < 0.7:
        haze = np.full_like(canvas, (210, 220, 225))
        alpha = rng.uniform(0.03, 0.15)
        canvas = cv2.addWeighted(canvas, 1 - alpha, haze, alpha, 0)
    transformed = []
    dx, dy = (w - nw) // 2, (h - nh) // 2
    for class_id, x1, y1, x2, y2 in labels:
        transformed.append((class_id, int(x1 * scale + dx), int(y1 * scale + dy),
                            int(x2 * scale + dx), int(y2 * scale + dy)))
    return canvas, transformed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, default=Path("/home/robotx/cone_yolo26"))
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--far-variants", type=int, default=2)
    parser.add_argument("--seed", type=int, default=3148)
    args = parser.parse_args()
    rng = random.Random(args.seed)
    for split in ("train", "val"):
        (args.out / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.out / "labels" / split).mkdir(parents=True, exist_ok=True)

    # Hold out the last video to measure generalization to its far/background views.
    manifest = []
    for video_index, video_path in enumerate(VIDEOS):
        cap = cv2.VideoCapture(str(video_path))
        count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        split = "val" if video_index == len(VIDEOS) - 1 else "train"
        for frame_index in range(0, count, args.stride):
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            ok, image = cap.read()
            if not ok:
                continue
            # The first two clips are single-color close-ups. Restricting their
            # labels prevents lighting/shadows from swapping orange and green.
            allowed = ("orange",) if video_index == 0 else (("green",) if video_index == 1 else None)
            labels = extract_boxes(image, allowed)
            if not labels:
                continue
            stem = f"v{video_index}_{frame_index:06d}"
            image_path = args.out / "images" / split / f"{stem}.jpg"
            label_path = args.out / "labels" / split / f"{stem}.txt"
            cv2.imwrite(str(image_path), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            write_yolo_label(label_path, labels, image.shape[1], image.shape[0])
            manifest.append({"image": str(image_path), "video": str(video_path),
                             "frame": frame_index, "split": split, "labels": labels})
            if split == "train":
                for variant_index in range(args.far_variants):
                    variant, variant_labels = far_variant(image, labels, rng)
                    vstem = f"{stem}_far{variant_index}"
                    cv2.imwrite(str(args.out / "images" / split / f"{vstem}.jpg"), variant,
                                [cv2.IMWRITE_JPEG_QUALITY, 88])
                    write_yolo_label(args.out / "labels" / split / f"{vstem}.txt",
                                     variant_labels, variant.shape[1], variant.shape[0])
                    manifest.append({"image": str(args.out / "images" / split / f"{vstem}.jpg"),
                                     "video": str(video_path), "frame": frame_index,
                                     "split": split, "synthetic_far": True,
                                     "labels": variant_labels})
        cap.release()
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str), encoding="utf-8")
    (args.out / "dataset.yaml").write_text(
        f"path: {args.out}\ntrain: images/train\nval: images/val\nnames:\n  0: orange_cone\n  1: green_cone\n",
        encoding="utf-8",
    )
    counts = {}
    for item in manifest:
        counts[item["split"]] = counts.get(item["split"], 0) + 1
    print(f"Wrote {len(manifest)} images: {counts}")
    print(f"Dataset: {args.out / 'dataset.yaml'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
