#!/usr/bin/env python3
"""Build the final six-class RobotX dataset without modifying source data.

Reviewed legacy labels are remapped as follows:
red->buoy_red, green->buoy_green, no_led->buoy, dock->dock,
home->platform. Legacy obstacle boxes are intentionally dropped.

New robotx_do_not_touch media is pseudo-labelled with the reviewed buoy and
legacy navigation detectors. Buoy cap colour is estimated in HSV so blue
buoys can be added to the new schema. Pseudo-labels remain provisional and
must be visually reviewed before results are called final.
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
from ultralytics import YOLO


NAMES = ("buoy_red", "buoy_green", "buoy_blue", "buoy", "dock", "platform")
LEGACY_MAP = {0: 0, 1: 1, 2: 3, 3: 4, 4: 5}
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
VIDEO_SUFFIXES = (".mov", ".mp4", ".avi", ".mkv")


def link_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists() or target.is_symlink():
        return
    try:
        os.symlink(source.resolve(), target)
    except OSError:
        shutil.copy2(source, target)


def image_for_label(root: Path, label: Path) -> Path | None:
    rel = label.relative_to(root)
    parts = list(rel.parts)
    parts[parts.index("labels")] = "images"
    stem = root.joinpath(*parts).with_suffix("")
    return next((stem.with_suffix(ext) for ext in IMAGE_SUFFIXES if stem.with_suffix(ext).exists()), None)


def write_label(path: Path, rows: list[tuple[int, float, float, float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = "".join(f"{c} {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n" for c, x, y, w, h in rows)
    path.write_text(text)


def remap_reviewed(source: Path, output: Path, counts: dict[str, Counter]) -> None:
    for split in ("train", "val", "test"):
        label_dir = source / "labels" / split
        if not label_dir.exists():
            continue
        for label in sorted(label_dir.glob("*.txt")):
            image = image_for_label(source, label)
            if image is None:
                continue
            rows = []
            for raw in label.read_text().splitlines():
                fields = raw.split()
                if len(fields) < 5:
                    continue
                old = int(fields[0])
                if old not in LEGACY_MAP:
                    continue
                new = LEGACY_MAP[old]
                rows.append((new, *map(float, fields[1:5])))
                counts[split][new] += 1
            name = f"reviewed_{split}_{image.name}"
            link_or_copy(image, output / "images" / split / name)
            write_label(output / "labels" / split / Path(name).with_suffix(".txt"), rows)


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if inter <= 0:
        return 0.0
    aa = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    bb = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / max(aa + bb - inter, 1e-9)


def cap_class(frame: np.ndarray, box: tuple[float, float, float, float]) -> int:
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    # Coloured lamps are centred near the top of the whole-buoy box.
    ax1 = max(0, int(x1 + 0.18 * bw))
    ax2 = min(width, int(x2 - 0.18 * bw))
    ay1 = max(0, int(y1))
    ay2 = min(height, int(y1 + 0.38 * bh))
    crop = frame[ay1:ay2, ax1:ax2]
    if crop.size == 0:
        return 3
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    vivid = (s >= 85) & (v >= 55)
    masks = {
        0: vivid & ((h <= 12) | (h >= 170)),
        1: vivid & (h >= 35) & (h <= 88),
        2: vivid & (h >= 92) & (h <= 135),
    }
    scores = {class_id: int(mask.sum()) for class_id, mask in masks.items()}
    best = max(scores, key=scores.get)
    minimum = max(5, int(crop.shape[0] * crop.shape[1] * 0.007))
    return best if scores[best] >= minimum else 3


def model_boxes(result, allowed: set[int] | None = None):
    boxes = []
    if result.boxes is None:
        return boxes
    for row in result.boxes:
        class_id = int(row.cls.item())
        if allowed is not None and class_id not in allowed:
            continue
        x1, y1, x2, y2 = map(float, row.xyxy[0].tolist())
        boxes.append((float(row.conf.item()), class_id, (x1, y1, x2, y2)))
    return boxes


def pseudo_rows(frame: np.ndarray, buoy_model: YOLO, nav_model: YOLO) -> list[tuple[int, float, float, float, float]]:
    height, width = frame.shape[:2]
    buoy_result = buoy_model.predict(frame, imgsz=1280, conf=0.25, iou=0.55, verbose=False)[0]
    nav_result = nav_model.predict(frame, imgsz=1280, conf=0.30, iou=0.55, verbose=False)[0]

    candidates = [(conf, box) for conf, _, box in model_boxes(buoy_result)]
    candidates += [(conf * 0.95, box) for conf, _, box in model_boxes(nav_result, {0, 1, 2})]
    kept: list[tuple[float, tuple[float, float, float, float]]] = []
    for conf, box in sorted(candidates, reverse=True):
        if all(iou(box, previous) < 0.50 for _, previous in kept):
            kept.append((conf, box))

    rows: list[tuple[int, float, float, float, float]] = []
    for _, box in kept:
        class_id = cap_class(frame, box)
        x1, y1, x2, y2 = box
        rows.append((class_id, (x1 + x2) / (2 * width), (y1 + y2) / (2 * height),
                     (x2 - x1) / width, (y2 - y1) / height))

    # The legacy nav model's class 3 is whole dock; class 4 (home) is platform.
    for conf, old_class, box in model_boxes(nav_result, {3, 4}):
        del conf
        class_id = 4 if old_class == 3 else 5
        x1, y1, x2, y2 = box
        rows.append((class_id, (x1 + x2) / (2 * width), (y1 + y2) / (2 * height),
                     (x2 - x1) / width, (y2 - y1) / height))
    return rows


def changed_enough(previous: np.ndarray | None, frame: np.ndarray, threshold: float) -> bool:
    small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (160, 90))
    if previous is None:
        return True
    return float(cv2.absdiff(previous, small).mean()) >= threshold


def save_pseudo(frame: np.ndarray, output: Path, split: str, name: str,
                buoy_model: YOLO, nav_model: YOLO, counts: dict[str, Counter]) -> None:
    rows = pseudo_rows(frame, buoy_model, nav_model)
    image_path = output / "images" / split / name
    image_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 94])
    write_label(output / "labels" / split / Path(name).with_suffix(".txt"), rows)
    for row in rows:
        counts[split][row[0]] += 1


def add_new_media(media: Path, output: Path, buoy_model: YOLO, nav_model: YOLO,
                  counts: dict[str, Counter], fps: float, motion_threshold: float) -> None:
    photos = sorted(p for p in media.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    for index, path in enumerate(photos):
        frame = cv2.imread(str(path))
        if frame is not None:
            save_pseudo(frame, output, "train", f"new_photo_{index:04d}.jpg",
                        buoy_model, nav_model, counts)

    videos = sorted(p for p in media.iterdir() if p.suffix.lower() in VIDEO_SUFFIXES)
    split_order = ("train", "val", "test")
    for video_index, path in enumerate(videos):
        split = split_order[min(video_index, len(split_order) - 1)]
        capture = cv2.VideoCapture(str(path))
        native_fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
        stride = max(1, round(native_fps / fps))
        frame_index = kept_index = 0
        previous_small = None
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame_index % stride == 0:
                small = cv2.resize(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), (160, 90))
                if changed_enough(previous_small, frame, motion_threshold):
                    name = f"new_{path.stem}_{kept_index:05d}.jpg"
                    save_pseudo(frame, output, split, name, buoy_model, nav_model, counts)
                    previous_small = small
                    kept_index += 1
            frame_index += 1
        capture.release()
        print(f"{path.name}: kept {kept_index} pseudo-labelled frames in {split}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reviewed", type=Path, required=True)
    parser.add_argument("--media", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--buoy-model", type=Path, required=True)
    parser.add_argument("--nav-model", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--motion-threshold", type=float, default=1.8)
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")
    for split in ("train", "val", "test"):
        (args.output / "images" / split).mkdir(parents=True)
        (args.output / "labels" / split).mkdir(parents=True)

    counts = {split: Counter() for split in ("train", "val", "test")}
    remap_reviewed(args.reviewed, args.output, counts)
    buoy_model = YOLO(str(args.buoy_model))
    nav_model = YOLO(str(args.nav_model))
    add_new_media(args.media, args.output, buoy_model, nav_model, counts,
                  args.fps, args.motion_threshold)

    yaml_text = (
        f"path: {args.output.resolve()}\ntrain: images/train\nval: images/val\ntest: images/test\n"
        "names:\n" + "".join(f"  {i}: {name}\n" for i, name in enumerate(NAMES))
    )
    (args.output / "data.yaml").write_text(yaml_text)
    summary = {
        "classes": list(NAMES),
        "counts": {split: {NAMES[i]: counts[split][i] for i in range(len(NAMES))}
                   for split in counts},
        "warning": "New-media labels are pseudo-labels and require visual review.",
    }
    (args.output / "dataset_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
