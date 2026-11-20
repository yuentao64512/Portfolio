#!/usr/bin/env python3
"""Conservatively auto-label aligned ZED RGB/depth pairs for an 8-class YOLO dataset.

The script reads recording folders without modifying them. It ensembles the legacy
v14 six-class model (which can distinguish unlit buoys and generic docks) with the
v15 colour-specific model. Aligned 16-bit depth is used only for QA metadata; RGB
is the detector input because lamp colour is a required class cue.
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
    3: "buoy_no_light",
    4: "dock",
    5: "dock_red",
    6: "dock_green",
    7: "obstacle",
}

SPECIFIC = {0, 1, 2, 5, 6}
TRAIN_SESSIONS = {"20261112_111929_obs", "20261112_113136_obs"}
TEST_SESSIONS = {"20261112_113936_obs"}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", type=Path, required=True)
    parser.add_argument("--v14-model", type=Path, required=True)
    parser.add_argument("--v15-model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--conf", type=float, default=0.18)
    parser.add_argument("--hash-threshold", type=int, default=2)
    parser.add_argument("--max-gap", type=int, default=3)
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
    value = 0
    for bit in (small[:, 1:] > small[:, :-1]).reshape(-1):
        value = (value << 1) | int(bit)
    return value


def iou(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if inter <= 0:
        return 0.0
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return inter / max(area_a + area_b - inter, 1e-9)


def crop(image: np.ndarray, box: tuple[float, float, float, float]) -> np.ndarray:
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    return image[max(0, int(y1)):min(h, int(np.ceil(y2))),
                 max(0, int(x1)):min(w, int(np.ceil(x2)))]


def looks_like_buoy(patch: np.ndarray) -> bool:
    """Require the long cyan flotation base and white triangular body."""
    if patch.size == 0 or min(patch.shape[:2]) < 10:
        return False
    height, width = patch.shape[:2]
    if not 1.08 <= width / max(height, 1) <= 3.4:
        return False
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    lower = slice(int(.48 * height), height)
    cyan = ((hue[lower] >= 72) & (hue[lower] <= 112) &
            (saturation[lower] >= 40) & (value[lower] >= 45))
    white = (saturation <= 100) & (value >= 85)
    upper = white[int(.10 * height):max(int(.36 * height), 1)]
    middle = white[int(.36 * height):max(int(.72 * height), 1)]
    upper_width = int(np.any(upper, axis=0).sum()) if upper.size else 0
    middle_width = int(np.any(middle, axis=0).sum()) if middle.size else 0
    count, _, stats, _ = cv2.connectedComponentsWithStats(cyan.astype(np.uint8), 8)
    widest = 0
    largest = 0
    for index in range(1, count):
        _, _, component_width, _, area = stats[index]
        widest = max(widest, int(component_width))
        largest = max(largest, int(area))
    return (largest >= max(4, int(.006 * height * width)) and
            widest >= max(7, int(.48 * width)) and
            .045 <= float(white.mean()) <= .86 and
            middle_width >= max(5, int(.22 * width)) and
            upper_width <= .92 * max(middle_width, 1))


def looks_like_dock(patch: np.ndarray) -> bool:
    """Require a broad rectangular white station body over a dark platform."""
    if patch.size == 0 or min(patch.shape[:2]) < 12:
        return False
    height, width = patch.shape[:2]
    if not .75 <= width / max(height, 1) <= 2.8:
        return False
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    white = (saturation <= 65) & (value >= 105)
    cyan = ((hue >= 72) & (hue <= 112) & (saturation >= 50) & (value >= 45))
    dark = value <= 105
    white_fraction = float(white.mean())
    if white_fraction < .24:
        return False
    # A dock face stays broad near both its upper and middle sections. A buoy
    # has a narrow triangular top and a long cyan flotation base.
    upper = white[int(.12 * height):max(int(.36 * height), 1)]
    middle = white[int(.38 * height):max(int(.72 * height), 1)]
    upper_width = int(np.any(upper, axis=0).sum()) if upper.size else 0
    middle_width = int(np.any(middle, axis=0).sum()) if middle.size else 0
    broad_rectangle = (middle_width >= .38 * width and
                       upper_width >= .58 * max(middle_width, 1))
    if not broad_rectangle:
        return False
    lower = slice(int(.58 * height), height)
    dark_lower = float(dark[lower].mean())
    if dark_lower < .06:
        return False
    # Overexposed white dock boxes can reflect cyan water into the lower crop.
    # Their nearly solid rectangular face is stronger evidence than that
    # reflection, while triangular buoys never fill this much of the box.
    if white_fraction >= .86:
        return True
    if float(cyan[lower].mean()) > .12:
        return False
    return True


def cap_colour(image: np.ndarray, box: tuple[float, float, float, float]) -> int | None:
    """Return buoy red/green/blue only for one compact vivid top-centre component."""
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    xa, xb = max(0, int(x1 + .28 * bw)), min(w, int(x2 - .28 * bw))
    ya, yb = max(0, int(y1)), min(h, int(y1 + .27 * bh))
    patch = image[ya:yb, xa:xb]
    if patch.size == 0 or min(patch.shape[:2]) < 4:
        return None
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    vivid = (sat >= 105) & (val >= 85)
    masks = {
        0: vivid & ((hue <= 24) | (hue >= 168)),
        1: vivid & (hue >= 35) & (hue <= 90),
        2: vivid & (hue >= 92) & (hue <= 142),
    }
    scores: dict[int, int] = {}
    for class_id, mask in masks.items():
        count, _, stats, centers = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
        best = 0
        for index in range(1, count):
            _, y, cw, ch, area = stats[index]
            cx, cy = centers[index]
            if not .42 <= cw / max(ch, 1) <= 3.2:
                continue
            if not .16 * patch.shape[1] <= cx <= .84 * patch.shape[1]:
                continue
            if y > .72 * patch.shape[0] or cy > .78 * patch.shape[0]:
                continue
            best = max(best, int(area))
        scores[class_id] = best
    winner = max(scores, key=scores.get)
    ordered = sorted(scores.values(), reverse=True)
    minimum = max(6, int(patch.shape[0] * patch.shape[1] * .012))
    if scores[winner] < minimum or ordered[1] > .55 * ordered[0]:
        return None
    return winner


def dock_colour(image: np.ndarray, box: tuple[float, float, float, float]) -> int | None:
    """Return 5/6 for a reliable compact red/green lamp in the lower dock box."""
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    xa, xb = max(0, int(x1)), min(w, int(np.ceil(x2)))
    ya, yb = max(0, int(y1)), min(h, int(np.ceil(y2)))
    patch = image[ya:yb, xa:xb]
    if patch.size == 0 or min(patch.shape[:2]) < 8:
        return None
    ph, pw = patch.shape[:2]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    vivid = (sat >= 90) & (val >= 80)
    masks = {
        5: vivid & ((hue <= 24) | (hue >= 168)),
        6: vivid & (hue >= 35) & (hue <= 90),
    }
    scores: dict[int, int] = {}
    for class_id, mask in masks.items():
        count, _, stats, centers = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
        best = 0
        for index in range(1, count):
            x, y, cw, ch, area = stats[index]
            cx, cy = centers[index]
            if cy < .38 * ph or y < .27 * ph:
                continue
            if area > .12 * ph * pw or not .18 <= cw / max(ch, 1) <= 6.0:
                continue
            if .10 * pw <= cx <= .90 * pw:
                best = max(best, int(area))
        scores[class_id] = best
    winner = max(scores, key=scores.get)
    ordered = sorted(scores.values(), reverse=True)
    minimum = max(4, int(ph * pw * .0015))
    if scores[winner] < minimum or ordered[1] > .58 * ordered[0]:
        return None
    return winner


def mapped_candidates(result, source: str, image: np.ndarray) -> list[dict]:
    candidates: list[dict] = []
    if result.boxes is None:
        return candidates
    v14_map = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 7}
    v15_map = {0: 0, 1: 1, 2: 2, 3: 5, 4: 6, 5: 7}
    mapping = v14_map if source == "v14" else v15_map
    thresholds = {0: .22, 1: .22, 2: .22, 3: .25, 4: .28, 5: .25, 6: .25, 7: .42}
    for item in result.boxes:
        old = int(item.cls.item())
        if old not in mapping:
            continue
        confidence = float(item.conf.item())
        model_class = mapping[old]
        if confidence < thresholds[model_class]:
            continue
        box = tuple(float(value) for value in item.xyxy[0].tolist())
        if box[2] - box[0] < 7 or box[3] - box[1] < 7:
            continue
        patch = crop(image, box)
        buoy_geometry = looks_like_buoy(patch)
        dock_geometry = looks_like_dock(patch)
        # When both tests fire, prefer the stricter rectangular dock test.
        # Strong water reflections beneath a white dock can otherwise imitate
        # a buoy's cyan flotation base.
        if dock_geometry:
            visible = dock_colour(image, box)
            class_id = visible if visible is not None else 4
        elif buoy_geometry:
            visible = cap_colour(image, box)
            # Colour is a visual class cue. If the cap pixels do not support a
            # colour, label the object as no-light even when a legacy model
            # guesses a colour or dock class.
            class_id = visible if visible is not None else 3
        elif model_class == 7:
            # A coloured-buoy prediction without buoy geometry is not allowed
            # to become colour training data.
            class_id = 7
        else:
            # Partial or geometrically ambiguous buoy/dock detections are
            # omitted from pseudo-labels rather than assigned a wrong class.
            continue
        priority = 3 if class_id in SPECIFIC else (2 if class_id in {3, 4} else 1)
        candidates.append({
            "class_id": class_id,
            "confidence": confidence,
            "box": box,
            "source": source,
            "priority": priority,
        })
    return candidates


def fuse(candidates: list[dict]) -> list[dict]:
    ordered = sorted(candidates, key=lambda row: (row["priority"], row["confidence"]), reverse=True)
    selected: list[dict] = []
    for row in ordered:
        overlap = next((old for old in selected if iou(row["box"], old["box"]) >= .52), None)
        if overlap is None:
            selected.append(row)
            continue
        if row["class_id"] == overlap["class_id"] and row["confidence"] > overlap["confidence"]:
            selected[selected.index(overlap)] = row
    return sorted(selected, key=lambda row: row["box"][0])


def median_depth(depth: np.ndarray | None, box: tuple[float, float, float, float]) -> float | None:
    if depth is None:
        return None
    h, w = depth.shape[:2]
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    xa, xb = max(0, int(x1 + .2 * bw)), min(w, int(x2 - .2 * bw))
    ya, yb = max(0, int(y1 + .25 * bh)), min(h, int(y2 - .1 * bh))
    values = depth[ya:yb, xa:xb]
    if values.size == 0:
        return None
    valid = values[np.isfinite(values) & (values > 0)]
    if not valid.size:
        return None
    value = float(np.median(valid))
    # Recorder PNGs are normally millimetres; keep a defensive metres fallback.
    return value / 1000.0 if value > 100.0 else value


def write_yolo(path: Path, rows: list[dict], width: int, height: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for row in rows:
        x1, y1, x2, y2 = row["box"]
        xc, yc = (x1 + x2) / (2 * width), (y1 + y2) / (2 * height)
        bw, bh = (x2 - x1) / width, (y2 - y1) / height
        lines.append(f'{row["class_id"]} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}')
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def make_contacts(output: Path, audit: list[dict], max_per_class: int = 40) -> None:
    by_class: dict[int, list[dict]] = {i: [] for i in NAMES}
    for row in audit:
        if row.get("class_id") not in by_class:
            continue
        if len(by_class[row["class_id"]]) < max_per_class:
            by_class[row["class_id"]].append(row)
    for class_id, rows in by_class.items():
        tiles = []
        for row in rows:
            image = cv2.imread(row["output_image"])
            if image is None:
                continue
            x1, y1, x2, y2 = (int(float(row[k])) for k in ("x1", "y1", "x2", "y2"))
            pad_x, pad_y = max(10, (x2 - x1) // 3), max(10, (y2 - y1) // 3)
            crop = image[max(0, y1-pad_y):min(image.shape[0], y2+pad_y),
                         max(0, x1-pad_x):min(image.shape[1], x2+pad_x)]
            if crop.size == 0:
                continue
            crop = cv2.resize(crop, (240, 160), interpolation=cv2.INTER_AREA)
            cv2.putText(crop, f'{NAMES[class_id]} {float(row["confidence"]):.2f}',
                        (5, 18), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 255), 1, cv2.LINE_AA)
            tiles.append(crop)
        if not tiles:
            continue
        while len(tiles) % 5:
            tiles.append(np.zeros_like(tiles[0]))
        sheet = np.vstack([np.hstack(tiles[i:i+5]) for i in range(0, len(tiles), 5)])
        cv2.imwrite(str(output / f'class_{class_id}_{NAMES[class_id]}_contact.jpg'), sheet)


def main() -> int:
    args = arguments()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")
    sessions = sorted(path for path in args.recordings.iterdir() if path.is_dir())
    names = {path.name for path in sessions}
    missing = (TRAIN_SESSIONS | TEST_SESSIONS) - names
    if missing:
        raise SystemExit(f"Missing expected sessions: {sorted(missing)}")
    args.output.mkdir(parents=True)
    v14 = YOLO(str(args.v14_model), task="detect")
    v15 = YOLO(str(args.v15_model), task="detect")
    counters = Counter()
    audit: list[dict] = []

    for session in sessions:
        split = "train" if session.name in TRAIN_SESSIONS else "test_new"
        paths = sorted((session / "pairs").glob("rgb_*.jpg"))
        previous_hash: int | None = None
        skipped = 0
        for source in paths:
            image = cv2.imread(str(source))
            if image is None:
                counters["unreadable_rgb"] += 1
                continue
            current_hash = dhash(image)
            distance = 64 if previous_hash is None else (previous_hash ^ current_hash).bit_count()
            keep = previous_hash is None or distance > args.hash_threshold or skipped >= args.max_gap - 1
            if not keep:
                skipped += 1
                counters[f"{split}_near_duplicate"] += 1
                continue
            previous_hash, skipped = current_hash, 0
            result14 = v14.predict(image, imgsz=args.imgsz, conf=args.conf, iou=.55,
                                   device=0, verbose=False)[0]
            result15 = v15.predict(image, imgsz=args.imgsz, conf=args.conf, iou=.55,
                                   device=0, verbose=False)[0]
            rows = fuse(mapped_candidates(result14, "v14", image) +
                        mapped_candidates(result15, "v15", image))
            depth_name = source.name.replace("rgb_", "depth_").replace(".jpg", ".png")
            depth_path = source.with_name(depth_name)
            depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED) if depth_path.exists() else None
            target_name = f"{session.name}_{source.name}"
            target_image = args.output / "images" / split / target_name
            target_label = args.output / "labels" / split / Path(target_name).with_suffix(".txt")
            link_or_copy(source, target_image)
            write_yolo(target_label, rows, image.shape[1], image.shape[0])
            counters[f"{split}_images"] += 1
            if not rows:
                counters[f"{split}_empty"] += 1
            for index, row in enumerate(rows):
                depth_m = median_depth(depth, row["box"])
                counters[f'{split}_class_{row["class_id"]}'] += 1
                x1, y1, x2, y2 = row["box"]
                audit.append({
                    "session": session.name,
                    "split": split,
                    "source": str(source),
                    "output_image": str(target_image),
                    "detection_index": index,
                    "class_id": row["class_id"],
                    "class_name": NAMES[row["class_id"]],
                    "confidence": f'{row["confidence"]:.6f}',
                    "detector": row["source"],
                    "depth_m": "" if depth_m is None else f"{depth_m:.3f}",
                    "x1": f"{x1:.2f}", "y1": f"{y1:.2f}",
                    "x2": f"{x2:.2f}", "y2": f"{y2:.2f}",
                })

    yaml = (
        f"path: {args.output}\n"
        "train: images/train\n"
        "val: images/test_new\n"
        "test: images/test_new\n"
        "names:\n" + "".join(f"  {i}: {name}\n" for i, name in NAMES.items())
    )
    (args.output / "data.yaml").write_text(yaml)
    fields = sorted({key for row in audit for key in row})
    with (args.output / "audit.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(audit)
    summary = {
        "classes": NAMES,
        "recordings": str(args.recordings),
        "v14_model": str(args.v14_model),
        "v15_model": str(args.v15_model),
        "counts": dict(sorted(counters.items())),
        "notes": [
            "Input recordings were read-only.",
            "Two earlier sessions are train; the final session is held out as test_new.",
            "Depth is recorded in audit.csv for QA and is not used as an RGB training channel.",
            "All pseudo-label contact sheets require visual review before training.",
        ],
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    make_contacts(args.output, audit)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
