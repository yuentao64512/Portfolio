#!/usr/bin/env python3
"""Build an eight-class v16 dataset from the read-only v14 base plus new labels."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from collections import Counter
from pathlib import Path

import cv2
import numpy as np

from autolabel_training12_v16 import NAMES, cap_colour, dock_colour, looks_like_dock


SUFFIXES = (".jpg", ".jpeg", ".png")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v14", type=Path, required=True)
    parser.add_argument("--new", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def link_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source.resolve(), target)
    except OSError:
        shutil.copy2(source.resolve(), target)


def image_for_label(root: Path, split: str, label: Path) -> Path | None:
    for suffix in SUFFIXES:
        candidate = root / "images" / split / (label.stem + suffix)
        if candidate.exists():
            return candidate
    return None


def crop(image: np.ndarray, box: tuple[float, float, float, float]) -> np.ndarray:
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    return image[max(0, int(y1)):min(h, int(np.ceil(y2))),
                 max(0, int(x1)):min(w, int(np.ceil(x2)))]


def yolo_box(parts: list[str], width: int, height: int) -> tuple[float, float, float, float]:
    xc, yc, bw, bh = map(float, parts[1:5])
    return ((xc - bw / 2) * width, (yc - bh / 2) * height,
            (xc + bw / 2) * width, (yc + bh / 2) * height)


def looks_like_buoy(patch: np.ndarray) -> bool:
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
    upper = white[int(.10 * h):max(int(.36 * h), 1)]
    middle = white[int(.36 * h):max(int(.72 * h), 1)]
    upper_width = int(np.any(upper, axis=0).sum()) if upper.size else 0
    middle_width = int(np.any(middle, axis=0).sum()) if middle.size else 0
    return (float(cyan.mean()) >= .005 and
            int(np.any(cyan, axis=0).sum()) >= max(3, int(.15 * w)) and
            .04 <= float(white.mean()) <= .86 and
            middle_width >= max(5, int(.22 * w)) and
            upper_width <= .92 * max(middle_width, 1))


def looks_like_coloured_buoy_loose(patch: np.ndarray) -> bool:
    """Recover blurred/far legacy colour examples after basic shape checks."""
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
    white_fraction = float(white.mean())
    return (float(cyan.mean()) >= .005 and
            int(np.any(cyan, axis=0).sum()) >= max(3, int(.15 * w)) and
            .04 <= white_fraction <= .86)


def has_dock_windows(patch: np.ndarray) -> bool:
    """Require two dark rectangular windows on a lit dock station."""
    if patch.size == 0 or min(patch.shape[:2]) < 12:
        return False
    h, w = patch.shape[:2]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    dark = (hsv[..., 2] <= 110).astype(np.uint8) * 255
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    windows = 0
    total = h * w
    for contour in contours:
        area = cv2.contourArea(contour)
        if not .006 * total <= area <= .30 * total:
            continue
        x, y, cw, ch = cv2.boundingRect(contour)
        if y >= .88 * h or not .35 <= cw / max(ch, 1) <= 4.2:
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, .06 * perimeter, True)
        if 4 <= len(approx) <= 8:
            windows += 1
    return windows >= 2


def has_strong_cyan_buoy_base(patch: np.ndarray) -> bool:
    """Reject a triangular buoy from the legacy dock-light classes.

    The legacy labels occasionally place a dock-light class on a foreground
    buoy.  Dark number diamonds can then look like dock windows.  A real buoy
    crop has a strong cyan flotation band in its lower half; reviewed dock
    crops in this dataset do not.
    """
    if patch.size == 0 or min(patch.shape[:2]) < 8:
        return False
    h = patch.shape[0]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    lower = slice(int(.48 * h), h)
    cyan = ((hue[lower] >= 72) & (hue[lower] <= 112) &
            (sat[lower] >= 45) & (val[lower] >= 45))
    return float(cyan.mean()) >= .12


def convert_v14_class(old: int, image: np.ndarray, parts: list[str], filename: str) -> int | None:
    if old not in (0, 1, 2, 3, 4, 5):
        return None
    if "IMG_7695" in filename:
        return 7
    box = yolo_box(parts, image.shape[1], image.shape[0])
    patch = crop(image, box)
    # Geometry overrides the legacy class. The old dataset contains examples
    # of dock boxes labelled as coloured/no-light buoys and boats labelled as
    # blue buoys; retaining those labels recreates the known confusion.
    # In the old broad dock class, a foreground buoy can overlap dock windows
    # in the background. Whole-buoy geometry must win in that case.
    if old == 4 and (looks_like_buoy(patch) or has_strong_cyan_buoy_base(patch)):
        # Old class 4 is a dock label.  Some of its boxes actually cover
        # foreground buoys or course flags.  Do not recycle those uncertain
        # boxes into a buoy colour class; clean buoy colours already exist in
        # the explicit legacy colour classes.
        return None
    if old in (0, 1, 2) and has_strong_cyan_buoy_base(patch):
        # The cap colour classes are explicit.  A buoy's number diamonds can
        # resemble two dock windows, so preserve strong buoy-base evidence
        # before applying the dock-window test.
        visible = cap_colour(image, box)
        return visible if visible is not None else old
    if looks_like_dock(patch) and has_dock_windows(patch):
        visible = dock_colour(image, box)
        # The old broad dock class contains boats and scenery. Keep only an
        # explicit colour-supported dock-light label; clean generic docks come
        # from the newly reviewed recording.
        return visible
    if old in (0, 1, 2) and looks_like_coloured_buoy_loose(patch):
        visible = cap_colour(image, box)
        return visible if visible is not None else old
    # Legacy class 3 (no-light) is too noisy to retain. The new recordings
    # provide a reviewed no-light class without docks, boats, or scenery.
    if old == 3:
        return None
    # Only the old explicit obstacle class may survive without buoy/dock
    # geometry. Ambiguous boxes from every other class are omitted.
    if old == 5:
        if looks_like_coloured_buoy_loose(patch) or looks_like_dock(patch):
            return None
        return 7
    return None


def write_rows(path: Path, rows: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(rows) + ("\n" if rows else ""))


def copy_v14(root: Path, output: Path, source_split: str, target_split: str,
             counters: Counter) -> None:
    for label in sorted((root / "labels" / source_split).glob("*.txt")):
        image_path = image_for_label(root, source_split, label)
        if image_path is None:
            counters["v14_missing_image"] += 1
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            counters["v14_unreadable"] += 1
            continue
        converted: list[str] = []
        for line in label.read_text().splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            new_class = convert_v14_class(int(parts[0]), image, parts, label.name)
            if new_class is None:
                continue
            converted.append(" ".join([str(new_class)] + parts[1:]))
            counters[f"v14_{target_split}_class_{new_class}"] += 1
        target_name = f"v14_{source_split}_{image_path.name}"
        link_or_copy(image_path, output / "images" / target_split / target_name)
        write_rows(output / "labels" / target_split / Path(target_name).with_suffix(".txt"), converted)
        counters[f"v14_{target_split}_images"] += 1


def copy_new(root: Path, output: Path, source_split: str, target_split: str,
             counters: Counter) -> None:
    for image_path in sorted((root / "images" / source_split).glob("*")):
        if image_path.suffix.lower() not in SUFFIXES:
            continue
        label = root / "labels" / source_split / image_path.with_suffix(".txt").name
        rows: list[str] = []
        if label.exists():
            for line in label.read_text().splitlines():
                parts = line.split()
                if len(parts) >= 5 and int(parts[0]) in NAMES:
                    rows.append(line)
                    counters[f"new_{target_split}_class_{int(parts[0])}"] += 1
        target_name = f"new_{image_path.name}"
        link_or_copy(image_path, output / "images" / target_split / target_name)
        write_rows(output / "labels" / target_split / Path(target_name).with_suffix(".txt"), rows)
        counters[f"new_{target_split}_images"] += 1


def main() -> int:
    args = arguments()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")
    counters = Counter()
    copy_v14(args.v14, args.output, "train", "train", counters)
    copy_v14(args.v14, args.output, "val", "val", counters)
    copy_v14(args.v14, args.output, "test", "test_old", counters)
    copy_new(args.new, args.output, "train", "train", counters)
    copy_new(args.new, args.output, "test_new", "test_new", counters)
    yaml = (
        f"path: {args.output}\n"
        "train: images/train\n"
        "val: images/test_new\n"
        "test: images/test_new\n"
        "names:\n" + "".join(f"  {index}: {name}\n" for index, name in NAMES.items())
    )
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "data.yaml").write_text(yaml)
    (args.output / "data_old_test.yaml").write_text(
        yaml.replace("test: images/test_new", "test: images/test_old"))
    (args.output / "data_legacy_val.yaml").write_text(
        yaml.replace("val: images/test_new", "val: images/val"))
    summary = {
        "classes": NAMES,
        "inputs": {"v14": str(args.v14), "new": str(args.new)},
        "counts": dict(sorted(counters.items())),
        "notes": [
            "Inputs were read-only and output uses hard links where possible.",
            "v14 train/val/test boundaries were preserved.",
            "Two new sessions augment train; the final new session is held out as test_new.",
            "The v15 best checkpoint is used to initialize training, but the detection head expands to eight classes.",
        ],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
