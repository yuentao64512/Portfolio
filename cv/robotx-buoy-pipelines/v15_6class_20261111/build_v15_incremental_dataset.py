#!/usr/bin/env python3
"""Build a non-destructive v15 dataset from v14 plus session 164851.

The v14 schema used class 3 for unlit boundary buoys and class 4 for a broad
"dock" class.  The v15 schema instead uses colour-specific dock classes and a
single obstacle class.  This converter keeps known coloured buoys, maps old
unlit/obstacle boxes to obstacle, and conservatively resolves old class-4
boxes from their pixels.  Inputs are read-only and the output must not exist.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


NAMES = (
    "buoy_red", "buoy_green", "buoy_blue",
    "dock_red", "dock_green", "obstacle",
)
SUFFIXES = (".jpg", ".jpeg", ".png")


def args():
    p = argparse.ArgumentParser()
    p.add_argument("--v14", type=Path, required=True)
    p.add_argument("--new-session", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    return p.parse_args()


def image_for_label(dataset: Path, split: str, label: Path) -> Path | None:
    for suffix in SUFFIXES:
        candidate = dataset / "images" / split / (label.stem + suffix)
        if candidate.exists():
            return candidate
    return None


def link_or_copy(source: Path, target: Path):
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source.resolve(), target)
    except OSError:
        shutil.copy2(source.resolve(), target)


def yolo_to_xyxy(row, width, height):
    xc, yc, bw, bh = map(float, row[1:5])
    return ((xc - bw / 2) * width, (yc - bh / 2) * height,
            (xc + bw / 2) * width, (yc + bh / 2) * height)


def crop(image, box):
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    return image[max(0, int(y1)):min(h, int(np.ceil(y2))),
                 max(0, int(x1)):min(w, int(np.ceil(x2)))]


def looks_like_buoy(patch):
    if patch.size == 0 or min(patch.shape[:2]) < 8:
        return False
    h, w = patch.shape[:2]
    if not 1.05 <= w / max(h, 1) <= 3.2:
        return False
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    lower = slice(int(.48 * h), h)
    cyan = ((hue[lower] >= 72) & (hue[lower] <= 112) &
            (sat[lower] >= 45) & (val[lower] >= 45))
    white = (sat <= 95) & (val >= 90)
    return (float(cyan.mean()) >= .005 and
            int(np.any(cyan, axis=0).sum()) >= max(3, int(.15 * w)) and
            float(white.mean()) >= .04)


def strongest_component(mask):
    binary = mask.astype(np.uint8) * 255
    count, _, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
    best = (0, None)
    for index in range(1, count):
        area = int(stats[index, cv2.CC_STAT_AREA])
        if area > best[0]:
            best = (area, centroids[index])
    return best


def colour_in_region(patch, top, bottom, centre_only=False):
    """Return red/green/blue ID for one reliable vivid component."""
    if patch.size == 0:
        return None
    h, w = patch.shape[:2]
    y1, y2 = int(top * h), max(int(top * h) + 1, int(bottom * h))
    x1, x2 = (int(.18 * w), int(.82 * w)) if centre_only else (0, w)
    region = patch[y1:y2, x1:x2]
    if region.size == 0:
        return None
    hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    vivid = (sat >= 90) & (val >= 85)
    masks = {
        0: vivid & ((hue <= 24) | (hue >= 168)),
        1: vivid & (hue >= 35) & (hue <= 90),
        2: vivid & (hue >= 92) & (hue <= 142),
    }
    scores = {cls: strongest_component(mask)[0] for cls, mask in masks.items()}
    winner = max(scores, key=scores.get)
    ordered = sorted(scores.values(), reverse=True)
    minimum = max(4, int(region.shape[0] * region.shape[1] * .0025))
    if scores[winner] < minimum:
        return None
    if ordered[1] > .65 * ordered[0]:
        return None
    return winner


def looks_like_dock_station(patch):
    """Reject cranes, boats and shoreline before considering a dock lamp."""
    if patch.size == 0 or min(patch.shape[:2]) < 10:
        return False
    h, w = patch.shape[:2]
    if not .45 <= w / max(h, 1) <= 2.5:
        return False
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    _, sat, val = cv2.split(hsv)
    white = (sat <= 30) & (val >= 105)
    dark = (val <= 105).astype(np.uint8) * 255
    # The station is a large white panel with one or two dark windows.  These
    # simple fractions reject coloured foliage/cranes and people/boats.
    if float(white.mean()) < .45 or float((dark > 0).mean()) < .025:
        return False
    # At least one dark, roughly rectangular window must be present.  This
    # excludes sky/crane and boat crops whose red/green pixels are unrelated
    # to a dock lamp.
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    area = h * w
    windows = 0
    for contour in contours:
        ca = cv2.contourArea(contour)
        if not .008 * area <= ca <= .30 * area:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, .055 * perimeter, True)
        x, y, cw, ch = cv2.boundingRect(contour)
        if (4 <= len(approx) <= 7 and .35 <= cw / max(ch, 1) <= 3.8
                and y < .90 * h):
            windows += 1
    return windows >= 2


def dock_light_colour(patch):
    """Return red/green only for a compact component in the lower station."""
    if not looks_like_dock_station(patch):
        return None
    h, w = patch.shape[:2]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    vivid = (sat >= 90) & (val >= 85)
    masks = {
        0: vivid & ((hue <= 24) | (hue >= 168)),
        1: vivid & (hue >= 35) & (hue <= 90),
    }
    scores = {}
    for cls, mask in masks.items():
        binary = mask.astype(np.uint8) * 255
        count, _, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
        best = 0
        for i in range(1, count):
            x, y, cw, ch, area = stats[i]
            cx, cy = centroids[i]
            if cy < .42 * h or y < .30 * h:
                continue
            if area > .12 * h * w:
                continue
            if not .18 <= cw / max(ch, 1) <= 6.0:
                continue
            margin_x, margin_y = max(3, cw), max(3, ch)
            sx1, sy1 = max(0, x-margin_x), max(0, y-margin_y)
            sx2, sy2 = min(w, x+cw+margin_x), min(h, y+ch+margin_y)
            surround = cv2.cvtColor(patch[sy1:sy2, sx1:sx2], cv2.COLOR_BGR2HSV)
            local_white = (surround[..., 1] <= 30) & (surround[..., 2] >= 105)
            if local_white.size == 0 or float(local_white.mean()) < .20:
                continue
            best = max(best, int(area))
        scores[cls] = best
    winner = max(scores, key=scores.get)
    ordered = sorted(scores.values(), reverse=True)
    minimum = max(4, int(h * w * .0018))
    if scores[winner] < minimum or ordered[1] > .55 * ordered[0]:
        return None
    return winner


def remap_class4(image, box):
    patch = crop(image, box)
    if looks_like_buoy(patch):
        colour = colour_in_region(patch, 0.0, .36, centre_only=True)
        return colour if colour in (0, 1, 2) else 5
    # Dock bay lamps sit in the lower half. Blue is not a dock class, and a
    # mixed/uncertain lamp is deliberately treated as an obstacle.
    colour = dock_light_colour(patch)
    return {0: 3, 1: 4}.get(colour, 5)


def write_rows(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(rows) + ("\n" if rows else ""))


def copy_v14(source, output, source_split, target_split, prefix, counters):
    for label in sorted((source / "labels" / source_split).glob("*.txt")):
        image_path = image_for_label(source, source_split, label)
        if image_path is None:
            counters["missing_image"] += 1
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            counters["unreadable"] += 1
            continue
        height, width = image.shape[:2]
        converted = []
        for line in label.read_text().splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            old = int(parts[0])
            if old in (0, 1, 2):
                new = old
            elif old in (3, 5):
                new = 5
            elif old == 4:
                new = remap_class4(image, yolo_to_xyxy(parts, width, height))
                # IMG_7695 is a shoreline/barrier sequence.  Its red fixtures
                # are hard negatives, not RobotX dock lamps (confirmed in the
                # class contact sheet), so keep them as obstacle.
                if new == 3 and "IMG_7695" in label.name:
                    new = 5
            else:
                continue
            converted.append(" ".join([str(new)] + parts[1:]))
            counters[f"v14_{source_split}_{new}"] += 1
        name = prefix + image_path.name
        link_or_copy(image_path, output / "images" / target_split / name)
        write_rows(output / "labels" / target_split / (Path(name).stem + ".txt"), converted)
        counters[f"v14_{source_split}_images"] += 1


def copy_new_session(source, output, source_split, target_split, counters):
    for label in sorted((source / "labels" / source_split).glob("*.txt")):
        image_path = image_for_label(source, source_split, label)
        if image_path is None:
            continue
        rows = []
        for line in label.read_text().splitlines():
            parts = line.split()
            if len(parts) >= 5 and int(parts[0]) in range(6):
                rows.append(line)
                counters[f"new_{target_split}_{int(parts[0])}"] += 1
        name = "s164851_" + image_path.name
        link_or_copy(image_path, output / "images" / target_split / name)
        write_rows(output / "labels" / target_split / (Path(name).stem + ".txt"), rows)
        counters[f"new_{target_split}_images"] += 1


def main():
    a = args()
    if a.output.exists():
        raise SystemExit(f"Refusing to overwrite {a.output}")
    counters = Counter()
    # Preserve v14's train/validation boundary.  The new session's later time
    # block remains the cleanly separated T1 test and is never used for model
    # selection.
    copy_v14(a.v14, a.output, "train", "train", "v14tr_", counters)
    copy_v14(a.v14, a.output, "val", "val", "v14va_", counters)
    # Keep v14 test as a separate T2 directory; it is never used for training.
    copy_v14(a.v14, a.output, "test", "test_T2", "v14t2_", counters)
    copy_new_session(a.new_session, a.output, "train", "train", counters)
    copy_new_session(a.new_session, a.output, "test_T1", "test_T1", counters)
    yaml = (
        f"path: {a.output}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test_T1\n"
        "names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(NAMES))
    )
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output / "data.yaml").write_text(yaml)
    (a.output / "data_T2.yaml").write_text(yaml.replace(
        "val: images/val\ntest: images/test_T1",
        "val: images/val\ntest: images/test_T2"))
    summary = {
        "classes": dict(enumerate(NAMES)),
        "inputs": {"v14": str(a.v14), "new_session": str(a.new_session)},
        "counts": dict(sorted(counters.items())),
        "notes": [
            "Inputs were read-only; output uses hard links when possible.",
            "T1 is the held-out later block of 164851.",
            "T2 is v14 test data and was not used in this build's training split.",
            "Old class-4 boxes were pixel-remapped; dock class QA is required before training.",
        ],
    }
    (a.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
