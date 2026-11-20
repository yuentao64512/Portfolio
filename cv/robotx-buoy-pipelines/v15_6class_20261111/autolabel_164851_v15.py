#!/usr/bin/env python3
"""Conservatively pre-label session 164851 for the v15 six-class dataset.

Only the dedicated whole-buoy detector is used. A visible, centred, saturated
cap component maps the whole buoy to red/green/blue. Every other detected buoy
maps to obstacle. No dock labels are guessed in this session. The raw session
is read-only; output images are hard-linked or copied into the v15 workspace.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


NAMES = {
    0: "buoy_red",
    1: "buoy_green",
    2: "buoy_blue",
    3: "dock_red",
    4: "dock_green",
    5: "obstacle",
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--buoy-model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--test-start", type=int, default=338,
                        help="source index where the held-out T1 time block starts")
    parser.add_argument("--conf", type=float, default=0.22)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--hash-threshold", type=int, default=4)
    parser.add_argument("--max-gap", type=int, default=4,
                        help="always retain at least one source frame in this many")
    return parser.parse_args()


def link_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source.resolve(), target)
    except OSError:
        shutil.copy2(source.resolve(), target)


def dhash(image: np.ndarray) -> int:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (9, 8), interpolation=cv2.INTER_AREA)
    bits = small[:, 1:] > small[:, :-1]
    value = 0
    for bit in bits.reshape(-1):
        value = (value << 1) | int(bit)
    return value


def hamming(left: int, right: int) -> int:
    return (left ^ right).bit_count()


def crop(image: np.ndarray, xyxy: tuple[float, float, float, float]) -> np.ndarray:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = xyxy
    return image[max(0, int(y1)):min(height, int(np.ceil(y2))),
                 max(0, int(x1)):min(width, int(np.ceil(x2)))]


def looks_like_whole_buoy(patch: np.ndarray) -> bool:
    """Reject tight digit, placard, circle and lamp boxes."""
    if patch.size == 0 or min(patch.shape[:2]) < 10:
        return False
    height, width = patch.shape[:2]
    if not 1.15 <= width / max(height, 1) <= 3.2:
        return False
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    lower = slice(int(0.52 * height), height)
    cyan = ((hue[lower] >= 72) & (hue[lower] <= 112)
            & (saturation[lower] >= 45) & (value[lower] >= 45))
    white = (saturation <= 95) & (value >= 90)
    area = height * width
    binary = cyan.astype(np.uint8) * 255
    count, _, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
    widest = 0
    largest = 0
    for index in range(1, count):
        _, _, component_width, _, component_area = stats[index]
        widest = max(widest, int(component_width))
        largest = max(largest, int(component_area))
    # A valid whole-buoy crop must contain the long cyan flotation base.  The
    # earlier 42% threshold still admitted tight crops of a letter/placard
    # with a few cyan pixels at the bottom.  Requiring a wider, larger cyan
    # component removes those bad boxes while retaining distant whole buoys.
    white_fraction = float(white.mean())
    return (largest >= max(4, int(0.008 * area))
            and widest >= max(7, int(0.62 * width))
            and int(white.sum()) >= max(8, int(0.070 * area))
            # Tight crops of a white letter/placard were almost uniformly
            # low-saturation white.  A whole buoy always includes enough cyan
            # base/water/black marking to stay below this ceiling.
            and white_fraction <= 0.95)


def cap_class(image: np.ndarray, xyxy: tuple[float, float, float, float]) -> int | None:
    """Return visible cap colour, or None when no reliable colour is present."""
    x1, y1, x2, y2 = xyxy
    width = x2 - x1
    height = y2 - y1
    # Cylinder heads are centred at the top. Excluding the outer quarters
    # prevents shore objects and neighbouring buoys from setting the colour.
    patch = crop(image, (x1 + 0.25 * width, y1, x2 - 0.25 * width,
                         y1 + 0.32 * height))
    if patch.size == 0 or min(patch.shape[:2]) < 4:
        return None
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    vivid = (saturation >= 80) & (value >= 85)
    masks = {
        # Real red lamps in this recording appear orange-red.
        0: vivid & ((hue <= 24) | (hue >= 168)),
        1: vivid & (hue >= 35) & (hue <= 90),
        2: vivid & (hue >= 92) & (hue <= 142),
    }
    scores: dict[int, int] = {}
    for class_id, mask in masks.items():
        binary = mask.astype(np.uint8) * 255
        count, _, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
        best = 0
        for index in range(1, count):
            _, y, component_width, component_height, area = stats[index]
            center_x, center_y = centroids[index]
            aspect = component_width / max(component_height, 1)
            if not 0.45 <= aspect <= 3.5:
                continue
            if not 0.22 * patch.shape[1] <= center_x <= 0.78 * patch.shape[1]:
                continue
            if center_y > 0.78 * patch.shape[0] or y > 0.80 * patch.shape[0]:
                continue
            best = max(best, int(area))
        scores[class_id] = best
    winner = max(scores, key=scores.get)
    minimum = max(5, int(patch.shape[0] * patch.shape[1] * 0.010))
    ordered = sorted(scores.values(), reverse=True)
    if scores[winner] < minimum:
        return None
    if len(ordered) > 1 and ordered[1] > 0.65 * ordered[0]:
        return None
    # This session was visually audited: it contains red heads and unlit
    # buoys, but no trustworthy green or blue lamps. Any non-red colour cue is
    # cyan base, water, placard fringing or background and must stay obstacle.
    return winner if winner == 0 else None


def visibility(xyxy: tuple[float, float, float, float], width: int, height: int) -> str:
    x1, y1, x2, y2 = xyxy
    box_height = y2 - y1
    if box_height < 24:
        return "tiny"
    margin = 3
    if x1 <= margin or y1 <= margin or x2 >= width - margin or y2 >= height - margin:
        return "partial_edge"
    return "full"


def write_label(path: Path, rows: list[tuple[int, tuple[float, float, float, float]]],
                width: int, height: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for class_id, (x1, y1, x2, y2) in rows:
        center_x = (x1 + x2) / (2 * width)
        center_y = (y1 + y2) / (2 * height)
        box_width = (x2 - x1) / width
        box_height = (y2 - y1) / height
        lines.append(f"{class_id} {center_x:.6f} {center_y:.6f} {box_width:.6f} {box_height:.6f}")
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def main() -> int:
    args = arguments()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")
    paths = sorted(path for path in args.images.iterdir()
                   if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not paths:
        raise SystemExit("No source images found")
    if not 0 < args.test_start < len(paths):
        raise SystemExit("--test-start must split the session into train and T1")

    model = YOLO(str(args.buoy_model), task="detect")
    counters: dict[str, Counter] = {"train": Counter(), "test_T1": Counter()}
    audit: list[dict] = []
    previous_hash: dict[str, int | None] = {"train": None, "test_T1": None}
    skipped: dict[str, int] = {"train": 0, "test_T1": 0}

    for source_index, source in enumerate(paths):
        split = "test_T1" if source_index >= args.test_start else "train"
        image = cv2.imread(str(source))
        if image is None:
            audit.append({"source_index": source_index, "source": source.name,
                          "split": split, "kept": 0, "reason": "unreadable"})
            continue
        current_hash = dhash(image)
        old_hash = previous_hash[split]
        distance = 64 if old_hash is None else hamming(old_hash, current_hash)
        keep = old_hash is None or distance > args.hash_threshold or skipped[split] >= args.max_gap - 1
        if not keep:
            skipped[split] += 1
            audit.append({"source_index": source_index, "source": source.name,
                          "split": split, "kept": 0, "reason": "near_duplicate",
                          "hash_distance": distance})
            continue
        previous_hash[split] = current_hash
        skipped[split] = 0

        result = model.predict(image, imgsz=args.imgsz, conf=args.conf,
                               iou=0.55, device=0, verbose=False)[0]
        height, width = image.shape[:2]
        rows: list[tuple[int, tuple[float, float, float, float]]] = []
        if result.boxes is not None:
            for detection_index, item in enumerate(result.boxes):
                xyxy = tuple(float(value) for value in item.xyxy[0].tolist())
                if xyxy[3] - xyxy[1] < 8:
                    continue
                if not looks_like_whole_buoy(crop(image, xyxy)):
                    counters[split]["rejected_nonwhole_box"] += 1
                    continue
                colour = cap_class(image, xyxy)
                class_id = 5 if colour is None else colour
                rows.append((class_id, xyxy))
                tag = visibility(xyxy, width, height)
                counters[split][class_id] += 1
                counters[split][f"visibility_{tag}"] += 1
                audit.append({
                    "source_index": source_index,
                    "source": source.name,
                    "split": split,
                    "kept": 1,
                    "reason": "detection",
                    "hash_distance": distance,
                    "detection_index": detection_index,
                    "class_id": class_id,
                    "class_name": NAMES[class_id],
                    "confidence": f"{float(item.conf.item()):.6f}",
                    "visibility": tag,
                    "x1": f"{xyxy[0]:.2f}", "y1": f"{xyxy[1]:.2f}",
                    "x2": f"{xyxy[2]:.2f}", "y2": f"{xyxy[3]:.2f}",
                })

        target_image = args.output / "images" / split / source.name
        target_label = args.output / "labels" / split / source.with_suffix(".txt").name
        link_or_copy(source, target_image)
        write_label(target_label, rows, width, height)
        counters[split]["images"] += 1
        if not rows:
            counters[split]["empty_images"] += 1

    args.output.mkdir(parents=True, exist_ok=True)
    yaml_text = (
        f"path: {args.output}\n"
        "train: images/train\n"
        "val: images/test_T1\n"
        "test: images/test_T1\n"
        "names:\n"
        + "".join(f"  {index}: {name}\n" for index, name in NAMES.items())
    )
    (args.output / "data.yaml").write_text(yaml_text)
    fields = sorted({key for row in audit for key in row})
    with (args.output / "audit.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audit)
    summary = {
        "source": str(args.images),
        "source_images": len(paths),
        "test_start": args.test_start,
        "classes": NAMES,
        "counts": {split: dict(count) for split, count in counters.items()},
        "warning": "Pseudo-labels require visual review before training.",
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
