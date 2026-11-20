#!/usr/bin/env python3
"""Prepare a boundary-aware RobotX buoy dataset without modifying source data.

The existing six-class detector calls every frame with no visible lamp ``buoy``.
This script separates those boxes using the already-trained sign model:

* a diamond or digit means a numbered buoy whose lamp may merely be off; the
  real off-state box is withheld and used as a synthetic-light input;
* otherwise the box remains ``buoy_boundary_no_light`` (the A/B/C/D course
  boundary markers).

Synthetic red/green/blue cap appearances are copied from real lit buoy boxes,
feather-blended onto the apex of off numbered buoys, and used only in train.
The raw recording folders and the source dataset are never modified.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import random
import shutil
import subprocess
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


NAMES = (
    "buoy_red",
    "buoy_green",
    "buoy_blue",
    "buoy_boundary_no_light",
    "dock",
    "obstacle",
)
IMAGE_SUFFIXES = (".jpg", ".jpeg", ".png")
COLOUR_NAMES = {0: "red", 1: "green", 2: "blue"}


@dataclass
class Box:
    cls: int
    xyxy: tuple[float, float, float, float]
    conf: float = 1.0


@dataclass
class Candidate:
    image: Path
    output_image: Path
    box: tuple[float, float, float, float]
    base_rows: list[Box]
    source: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dataset", type=Path, required=True)
    parser.add_argument("--supplement-dataset", type=Path,
                        help="Optional older six-class data; de-duplicated and whole-box filtered")
    parser.add_argument("--sessions", type=Path, nargs="+", required=True,
                        help="Session image directories; these are read-only")
    parser.add_argument("--session-splits", nargs="+",
                        help="One split per session (train, val or test)")
    parser.add_argument("--session-test-start", type=int, nargs="+",
                        help="Optional original-frame cutoff per session; -1 disables")
    parser.add_argument("--buoy-model", type=Path, required=True,
                        help="Dedicated one-class whole-buoy detector")
    parser.add_argument("--nav-model", type=Path, required=True)
    parser.add_argument("--shape-model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--nav-conf", type=float, default=0.25)
    parser.add_argument("--shape-conf", type=float, default=0.18)
    parser.add_argument("--hash-threshold", type=int, default=4)
    parser.add_argument("--max-gap", type=int, default=4,
                        help="Always keep at least one of every N source frames")
    parser.add_argument("--blur-threshold", type=float, default=45.0)
    parser.add_argument("--target-per-colour", type=int, default=300)
    parser.add_argument("--max-synth-ratio", type=float, default=0.50)
    parser.add_argument("--seed", type=int, default=20261111)
    return parser.parse_args()


def link_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    resolved = source.resolve()
    try:
        os.link(resolved, target)
    except OSError:
        shutil.copy2(resolved, target)


def file_digest(path: Path) -> str:
    hasher = hashlib.sha1()
    with path.resolve().open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            hasher.update(chunk)
    return hasher.hexdigest()


def image_for_label(dataset: Path, label: Path) -> Path | None:
    rel = label.relative_to(dataset)
    parts = list(rel.parts)
    parts[parts.index("labels")] = "images"
    stem = dataset.joinpath(*parts).with_suffix("")
    for suffix in IMAGE_SUFFIXES:
        candidate = stem.with_suffix(suffix)
        if candidate.exists():
            return candidate
    return None


def yolo_to_xyxy(row: list[str], width: int, height: int) -> Box:
    cls = int(row[0])
    x, y, w, h = map(float, row[1:5])
    return Box(cls, ((x - w / 2) * width, (y - h / 2) * height,
                     (x + w / 2) * width, (y + h / 2) * height))


def write_yolo(path: Path, boxes: list[Box], width: int, height: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = []
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy
        x = (x1 + x2) / (2 * width)
        y = (y1 + y2) / (2 * height)
        w = (x2 - x1) / width
        h = (y2 - y1) / height
        if w > 0 and h > 0:
            lines.append(f"{box.cls} {x:.6f} {y:.6f} {w:.6f} {h:.6f}")
    path.write_text("\n".join(lines) + ("\n" if lines else ""))


def crop_box(image: np.ndarray, box: tuple[float, float, float, float]) -> np.ndarray:
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    a, b = max(0, int(x1)), max(0, int(y1))
    c, d = min(w, int(np.ceil(x2))), min(h, int(np.ceil(y2)))
    return image[b:d, a:c]


def looks_like_whole_buoy(crop: np.ndarray) -> bool:
    """Reject placard-only boxes by requiring white body plus cyan lower base."""
    if crop.size == 0 or min(crop.shape[:2]) < 10:
        return False
    height, width = crop.shape[:2]
    # A full pyramid buoy plus floating base is wider than tall. Tight placard,
    # letter and lamp boxes in the legacy pseudo-labels are square/portrait.
    if not (1.05 <= width / max(height, 1) <= 3.2):
        return False
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    lower = slice(int(0.52 * height), height)
    cyan = (h[lower] >= 72) & (h[lower] <= 112) & (s[lower] >= 45) & (v[lower] >= 45)
    white = (s <= 95) & (v >= 90)
    area = height * width
    cyan_columns = int(np.any(cyan, axis=0).sum())
    cyan_row = int(cyan.sum(axis=1).max()) if cyan.size else 0
    return (int(cyan.sum()) >= max(3, int(0.004 * area))
            and cyan_columns >= max(4, int(0.28 * width))
            and cyan_row >= max(3, int(0.25 * width))
            and int(white.sum()) >= max(8, int(0.045 * area)))


def ocr_boundary_letter(crop: np.ndarray) -> str | None:
    """Read a prominent A/B/C/D marker with the installed Tesseract binary."""
    if crop.size == 0:
        return None
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    scale = max(2.0, 220.0 / max(gray.shape))
    enlarged = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    enlarged = cv2.GaussianBlur(enlarged, (3, 3), 0)
    _, binary = cv2.threshold(enlarged, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ok, encoded = cv2.imencode(".png", binary)
    if not ok:
        return None
    try:
        result = subprocess.run(
            ["tesseract", "stdin", "stdout", "--psm", "10", "-l", "eng",
             "-c", "tessedit_char_whitelist=ABCD"],
            input=encoded.tobytes(), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            check=False, timeout=3,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    text = "".join(chr for chr in result.stdout.decode(errors="ignore").upper() if chr in "ABCD")
    return text[0] if text else None


def has_classical_diamond(crop: np.ndarray) -> bool:
    """Return True for a filled diamond placard, not a bare A/B/C/D letter."""
    if crop.size == 0:
        return False
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, dark = cv2.threshold(gray, 105, 255, cv2.THRESH_BINARY_INV)
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = crop.shape[0] * crop.shape[1]
    for contour in contours:
        area = cv2.contourArea(contour)
        if not (0.025 * image_area <= area <= 0.48 * image_area):
            continue
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.055 * perimeter, True)
        if len(approx) not in (4, 5) or not cv2.isContourConvex(approx):
            continue
        x, y, w, h = cv2.boundingRect(approx)
        pixel_fill = cv2.countNonZero(dark[y:y + h, x:x + w]) / max(w * h, 1)
        center_x = x + w / 2
        if (0.85 <= w / max(h, 1) <= 1.30
                and y > 0.10 * crop.shape[0]
                and 0.18 * crop.shape[1] <= center_x <= 0.82 * crop.shape[1]
                and pixel_fill >= 0.38):
            return True
    return False


def is_numbered_off_buoy(crop: np.ndarray, shape_model: YOLO,
                         confidence: float) -> tuple[bool, list[str]]:
    if not looks_like_whole_buoy(crop):
        return False, []
    boundary_letter = ocr_boundary_letter(crop)
    result = shape_model.predict(crop, imgsz=640, conf=confidence,
                                 iou=0.55, verbose=False)[0]
    evidence: list[str] = []
    saw_diamond = False
    saw_digit = False
    if result.boxes is not None:
        names = result.names
        for item in result.boxes:
            cls = int(item.cls.item())
            name = str(names.get(cls, cls)) if isinstance(names, dict) else str(names[cls])
            # A/B/C/D boundary markers are plain letters. A detected diamond or
            # digit is evidence that this is a numbered lamp buoy.
            if name == "diamond":
                saw_diamond = True
                evidence.append(f"{name}:{float(item.conf.item()):.3f}")
            elif name.startswith("digit_"):
                saw_digit = True
                evidence.append(f"{name}:{float(item.conf.item()):.3f}")
    classical = has_classical_diamond(crop)
    if classical:
        evidence.append("classical_diamond")
    if classical or saw_diamond or saw_digit:
        return True, evidence
    if boundary_letter:
        return False, [f"boundary_ocr_{boundary_letter}"]
    # Without an explicit A/B/C/D OCR result, a dark-apex buoy is not safe to
    # call a boundary marker. Withhold it as off-lamp/unknown even when the
    # weak shape detector cannot read a distant diamond.
    if not evidence:
        evidence.append("off_unknown")
    return True, evidence


def cap_class(image: np.ndarray, box: tuple[float, float, float, float]) -> int | None:
    """Classify a real visible cap as red/green/blue, else return None."""
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    region = (x1 + 0.25 * bw, y1, x2 - 0.25 * bw, y1 + 0.30 * bh)
    crop = crop_box(image, region)
    if crop.size == 0:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    vivid = (s >= 105) & (v >= 110)
    masks = {
        0: vivid & ((h <= 16) | (h >= 168)),
        1: vivid & (h >= 32) & (h <= 92),
        2: vivid & (h >= 92) & (h <= 142),
    }
    scores = {}
    for class_id, raw_mask in masks.items():
        binary = raw_mask.astype(np.uint8) * 255
        count, _, stats, centroids = cv2.connectedComponentsWithStats(binary, 8)
        best = 0
        for index in range(1, count):
            a, b, component_w, component_h, area = stats[index]
            cx, cy = centroids[index]
            aspect = component_w / max(component_h, 1)
            if not (0.55 <= aspect <= 3.2):
                continue
            if not (0.25 * crop.shape[1] <= cx <= 0.75 * crop.shape[1]):
                continue
            if cy > 0.72 * crop.shape[0]:
                continue
            best = max(best, int(area))
        scores[class_id] = best
    best = max(scores, key=scores.get)
    minimum = max(7, int(crop.shape[0] * crop.shape[1] * 0.015))
    return best if scores[best] >= minimum else None


def dhash(image: np.ndarray) -> int:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    small = cv2.resize(gray, (9, 8), interpolation=cv2.INTER_AREA)
    bits = small[:, 1:] > small[:, :-1]
    value = 0
    for bit in bits.reshape(-1):
        value = (value << 1) | int(bit)
    return value


def hamming(a: int, b: int) -> int:
    return (a ^ b).bit_count()


def model_boxes(result) -> list[Box]:
    output = []
    if result.boxes is None:
        return output
    for item in result.boxes:
        output.append(Box(
            int(item.cls.item()),
            tuple(map(float, item.xyxy[0].tolist())),
            float(item.conf.item()),
        ))
    return output


def colour_mask(patch: np.ndarray, class_id: int) -> np.ndarray:
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    vivid = (s >= 65) & (v >= 45)
    if class_id == 0:
        mask = vivid & ((h <= 16) | (h >= 168))
    elif class_id == 1:
        mask = vivid & (h >= 32) & (h <= 92)
    else:
        mask = vivid & (h >= 90) & (h <= 140)
    result = (mask.astype(np.uint8) * 255)
    kernel = np.ones((3, 3), np.uint8)
    result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel, iterations=2)
    result = cv2.dilate(result, kernel, iterations=2)
    return result


def extract_cap(image: np.ndarray, box: tuple[float, float, float, float],
                class_id: int) -> tuple[np.ndarray, np.ndarray] | None:
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    region = (x1 + 0.20 * bw, y1, x2 - 0.20 * bw, y1 + 0.35 * bh)
    patch = crop_box(image, region)
    if patch.size == 0:
        return None
    mask = colour_mask(patch, class_id)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    choices = []
    for index in range(1, count):
        a, b, w, h, area = stats[index]
        cx, cy = centroids[index]
        aspect = w / max(h, 1)
        if area < 8 or not (0.55 <= aspect <= 3.2):
            continue
        if not (0.22 * patch.shape[1] <= cx <= 0.78 * patch.shape[1]):
            continue
        distance = abs(cx - patch.shape[1] / 2) / max(patch.shape[1], 1)
        choices.append((area / (1.0 + 4.0 * distance), index, a, b, w, h))
    if not choices:
        return None
    _, component, a, b, w, h = max(choices)
    component_mask = np.where(labels == component, 255, 0).astype(np.uint8)
    pad = max(2, int(0.25 * max(w, h)))
    a0, b0 = max(0, a - pad), max(0, b - pad)
    a1, b1 = min(patch.shape[1], a + w + pad), min(patch.shape[0], b + h + pad)
    rgb = patch[b0:b1, a0:a1].copy()
    alpha = component_mask[b0:b1, a0:a1]
    alpha = cv2.GaussianBlur(alpha, (0, 0), max(0.8, 0.06 * max(alpha.shape)))
    if rgb.size == 0 or alpha.max() == 0:
        return None
    return rgb, alpha


def paste_cap(image: np.ndarray, target_box: tuple[float, float, float, float],
              cap: tuple[np.ndarray, np.ndarray], rng: random.Random) -> np.ndarray:
    output = image.copy()
    rgb, alpha = cap
    x1, y1, x2, y2 = target_box
    bw, bh = x2 - x1, y2 - y1
    target_w = max(4, int(bw * rng.uniform(0.24, 0.34)))
    scale = target_w / max(rgb.shape[1], 1)
    target_h = max(3, int(rgb.shape[0] * scale))
    resized = cv2.resize(rgb, (target_w, target_h), interpolation=cv2.INTER_AREA)
    mask = cv2.resize(alpha, (target_w, target_h), interpolation=cv2.INTER_LINEAR)

    cx = int((x1 + x2) / 2 + rng.uniform(-0.025, 0.025) * bw)
    top = int(y1 + rng.uniform(0.00, 0.035) * bh)
    left = cx - target_w // 2
    height, width = output.shape[:2]
    a0, b0 = max(0, left), max(0, top)
    a1, b1 = min(width, left + target_w), min(height, top + target_h)
    if a1 <= a0 or b1 <= b0:
        return output
    pa0, pb0 = a0 - left, b0 - top
    pa1, pb1 = pa0 + (a1 - a0), pb0 + (b1 - b0)
    foreground = resized[pb0:pb1, pa0:pa1].astype(np.float32)
    blend = (mask[pb0:pb1, pa0:pa1].astype(np.float32) / 255.0)[..., None]
    background = output[b0:b1, a0:a1].astype(np.float32)
    output[b0:b1, a0:a1] = np.clip(foreground * blend + background * (1 - blend), 0, 255).astype(np.uint8)
    return output


def add_cap_library(library: dict[int, list[tuple[np.ndarray, np.ndarray]]],
                    image: np.ndarray, boxes: list[Box]) -> None:
    for box in boxes:
        if box.cls in COLOUR_NAMES:
            cap = extract_cap(image, box.xyxy, box.cls)
            if cap is not None:
                library[box.cls].append(cap)


def process_source(args: argparse.Namespace, shape_model: YOLO,
                   output: Path, counters: dict[str, Counter],
                   candidates: list[Candidate], cap_library: dict[int, list],
                   seen: set[str]) -> None:
    for source_split in ("train", "val", "test"):
        label_dir = args.source_dataset / "labels" / source_split
        if not label_dir.exists():
            continue
        for label in sorted(label_dir.glob("*.txt")):
            source_image = image_for_label(args.source_dataset, label)
            if source_image is None:
                continue
            image = cv2.imread(str(source_image.resolve()))
            if image is None:
                continue
            seen.add(file_digest(source_image))
            height, width = image.shape[:2]
            parsed = []
            for raw in label.read_text().splitlines():
                fields = raw.split()
                if len(fields) >= 5:
                    parsed.append(yolo_to_xyxy(fields, width, height))

            kept: list[Box] = []
            pending: list[tuple[Box, list[str]]] = []
            # The reviewed Model A schema is red, green, no_led, dock, home,
            # obstacle. "home" is the low floating platform in this dataset.
            for box in parsed:
                if box.cls in (0, 1, 2) and not looks_like_whole_buoy(crop_box(image, box.xyxy)):
                    counters[source_split]["rejected_nonwhole_buoy_box"] += 1
                    continue
                if box.cls in (0, 1, 2):
                    visible_colour = cap_class(image, box.xyxy)
                    if visible_colour is not None:
                        kept.append(Box(visible_colour, box.xyxy))
                        continue
                    numbered, evidence = is_numbered_off_buoy(
                        crop_box(image, box.xyxy), shape_model, args.shape_conf)
                    if numbered:
                        pending.append((box, evidence))
                    else:
                        kept.append(Box(3, box.xyxy))
                elif box.cls == 3:
                    kept.append(Box(4, box.xyxy))
                elif box.cls == 4:
                    kept.append(Box(5, box.xyxy))

            split = source_split
            if source_split == "val" and source_image.name.startswith("WhatsApp_Video_"):
                split = "test"
            name = f"legacy_{split}_{source_image.name}"
            target_image = output / "images" / split / name
            link_or_copy(source_image, target_image)
            write_yolo(output / "labels" / split / Path(name).with_suffix(".txt"), kept, width, height)
            for box in kept:
                counters[split][box.cls] += 1
            add_cap_library(cap_library, image, kept)
            if split == "train":
                for box, evidence in pending:
                    candidates.append(Candidate(source_image.resolve(), target_image, box.xyxy,
                                                list(kept), "legacy:" + ",".join(evidence)))
            counters[split]["off_numbered_withheld"] += len(pending)


def process_supplement(args: argparse.Namespace, shape_model: YOLO,
                       output: Path, counters: dict[str, Counter],
                       candidates: list[Candidate], cap_library: dict[int, list],
                       seen: set[str]) -> None:
    if args.supplement_dataset is None:
        return
    for source_split in ("train", "val", "test"):
        label_dir = args.supplement_dataset / "labels" / source_split
        if not label_dir.exists():
            continue
        for label in sorted(label_dir.glob("*.txt")):
            source_image = image_for_label(args.supplement_dataset, label)
            if source_image is None:
                continue
            digest = file_digest(source_image)
            if digest in seen:
                counters["train"]["supplement_duplicate_skipped"] += 1
                continue
            image = cv2.imread(str(source_image.resolve()))
            if image is None:
                continue
            height, width = image.shape[:2]
            kept: list[Box] = []
            pending: list[tuple[Box, list[str]]] = []
            for raw in label.read_text().splitlines():
                fields = raw.split()
                if len(fields) < 5:
                    continue
                box = yolo_to_xyxy(fields, width, height)
                if box.cls in (0, 1, 2, 3):
                    crop = crop_box(image, box.xyxy)
                    if not looks_like_whole_buoy(crop):
                        counters["train"]["supplement_nonwhole_skipped"] += 1
                        continue
                if box.cls in (0, 1, 2, 3):
                    visible_colour = cap_class(image, box.xyxy)
                    if visible_colour is not None:
                        kept.append(Box(visible_colour, box.xyxy))
                        continue
                    numbered, evidence = is_numbered_off_buoy(
                        crop_box(image, box.xyxy), shape_model, args.shape_conf)
                    if numbered:
                        pending.append((box, evidence))
                    else:
                        kept.append(Box(3, box.xyxy))
                elif box.cls == 4:
                    kept.append(Box(4, box.xyxy))
                elif box.cls == 5:
                    kept.append(Box(5, box.xyxy))
            if not kept and not pending:
                continue
            seen.add(digest)
            name = f"supplement_{source_split}_{source_image.name}"
            target_image = output / "images" / "train" / name
            link_or_copy(source_image, target_image)
            write_yolo(output / "labels" / "train" / Path(name).with_suffix(".txt"),
                       kept, width, height)
            for box in kept:
                counters["train"][box.cls] += 1
            add_cap_library(cap_library, image, kept)
            for box, evidence in pending:
                candidates.append(Candidate(source_image.resolve(), target_image, box.xyxy,
                                            list(kept), "supplement:" + ",".join(evidence)))
            counters["train"]["off_numbered_withheld"] += len(pending)


def process_sessions(args: argparse.Namespace, buoy_model: YOLO, nav_model: YOLO, shape_model: YOLO,
                     output: Path, counters: dict[str, Counter],
                     candidates: list[Candidate], cap_library: dict[int, list],
                     audit_rows: list[dict], seen: set[str]) -> None:
    splits = args.session_splits or ["train"] * len(args.sessions)
    cutoffs = args.session_test_start or [-1] * len(args.sessions)
    if len(splits) != len(args.sessions) or any(split not in {"train", "val", "test"} for split in splits):
        raise SystemExit("--session-splits must provide train/val/test once per --sessions entry")
    if len(cutoffs) != len(args.sessions):
        raise SystemExit("--session-test-start must provide one integer per session")
    for session_dir, default_split, test_start in zip(args.sessions, splits, cutoffs):
        session = session_dir.parent.name
        images = sorted(p for p in session_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
        previous_hash: int | None = None
        skipped = 0
        for index, source_image in enumerate(images):
            split = "test" if test_start >= 0 and index >= test_start else default_split
            image = cv2.imread(str(source_image))
            if image is None:
                audit_rows.append({"session": session, "image": source_image.name,
                                   "kept": 0, "reason": "unreadable", "blur": ""})
                continue
            current_hash = dhash(image)
            distance = 64 if previous_hash is None else hamming(previous_hash, current_hash)
            keep = previous_hash is None or distance > args.hash_threshold or skipped >= args.max_gap - 1
            blur = float(cv2.Laplacian(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var())
            audit_rows.append({"session": session, "image": source_image.name,
                               "kept": int(keep), "reason": "kept" if keep else "near_duplicate",
                               "hash_distance": distance, "blur": round(blur, 2),
                               "blurry_flag": int(blur < args.blur_threshold)})
            if not keep:
                skipped += 1
                continue
            digest = file_digest(source_image)
            if digest in seen:
                audit_rows[-1]["reason"] = "duplicate_existing"
                audit_rows[-1]["kept"] = 0
                continue
            seen.add(digest)
            previous_hash = current_hash
            skipped = 0

            buoy_result = buoy_model.predict(image, imgsz=1280, conf=args.nav_conf,
                                             iou=0.55, verbose=False)[0]
            nav_result = nav_model.predict(image, imgsz=1280, conf=args.nav_conf,
                                           iou=0.55, verbose=False)[0]
            kept_boxes: list[Box] = []
            pending: list[tuple[Box, list[str]]] = []
            for box in model_boxes(buoy_result):
                if not looks_like_whole_buoy(crop_box(image, box.xyxy)):
                    counters["train"]["rejected_nonwhole_buoy_box"] += 1
                    continue
                visible_colour = cap_class(image, box.xyxy)
                if visible_colour is not None:
                    kept_boxes.append(Box(visible_colour, box.xyxy, box.conf))
                else:
                    numbered, evidence = is_numbered_off_buoy(
                        crop_box(image, box.xyxy), shape_model, args.shape_conf)
                    if numbered:
                        pending.append((box, evidence))
                    else:
                        kept_boxes.append(Box(3, box.xyxy, box.conf))
            for box in model_boxes(nav_result):
                if box.cls == 4:
                    kept_boxes.append(Box(4, box.xyxy, box.conf))
                elif box.cls == 5:
                    kept_boxes.append(Box(5, box.xyxy, box.conf))

            name = f"new_{session}_{index:06d}{source_image.suffix.lower()}"
            target_image = output / "images" / split / name
            link_or_copy(source_image, target_image)
            height, width = image.shape[:2]
            write_yolo(output / "labels" / split / Path(name).with_suffix(".txt"),
                       kept_boxes, width, height)
            for box in kept_boxes:
                counters[split][box.cls] += 1
            if split == "train":
                add_cap_library(cap_library, image, kept_boxes)
                for box, evidence in pending:
                    candidates.append(Candidate(source_image, target_image, box.xyxy,
                                                list(kept_boxes), f"{session}:" + ",".join(evidence)))
            counters[split]["off_numbered_withheld"] += len(pending)


def generate_synthetic(args: argparse.Namespace, output: Path,
                       candidates: list[Candidate], cap_library: dict[int, list],
                       counters: dict[str, Counter], manifest: list[dict]) -> list[Path]:
    rng = random.Random(args.seed)
    eligible: list[Candidate] = []
    for candidate in candidates:
        image = cv2.imread(str(candidate.image.resolve()))
        if image is None:
            continue
        crop = crop_box(image, candidate.box)
        # Distant placard-only detections are useful evidence that the buoy is
        # not A/B/C/D, so their boxes stay withheld from the boundary class.
        # They are too small to support a credible pasted lamp, however.
        if crop.size and min(crop.shape[:2]) >= 40 and looks_like_whole_buoy(crop):
            eligible.append(candidate)
    candidates = eligible
    rng.shuffle(candidates)
    created: list[Path] = []
    for class_id in (0, 1, 2):
        real_count = counters["train"][class_id]
        ratio = args.max_synth_ratio
        max_synth = int(real_count * ratio / max(1e-9, 1.0 - ratio))
        needed = min(max(0, args.target_per_colour - real_count), max_synth)
        if not cap_library[class_id] or needed == 0 or not candidates:
            continue
        selected = [candidates[index % len(candidates)] for index in range(needed)]
        for index, candidate in enumerate(selected):
            image = cv2.imread(str(candidate.image.resolve()))
            if image is None:
                continue
            cap = rng.choice(cap_library[class_id])
            synthetic = paste_cap(image, candidate.box, cap, rng)
            digest = hashlib.sha1(f"{candidate.image}:{candidate.box}:{class_id}:{index}".encode()).hexdigest()[:12]
            name = f"synth_{COLOUR_NAMES[class_id]}_{digest}.jpg"
            image_path = output / "images" / "train" / name
            cv2.imwrite(str(image_path), synthetic, [cv2.IMWRITE_JPEG_QUALITY, 95])
            rows = list(candidate.base_rows) + [Box(class_id, candidate.box)]
            height, width = synthetic.shape[:2]
            write_yolo(output / "labels" / "train" / Path(name).with_suffix(".txt"),
                       rows, width, height)
            counters["train"][class_id] += 1
            counters["train"]["synthetic"] += 1
            manifest.append({
                "file": name,
                "source_image": str(candidate.image),
                "source_kind": candidate.source,
                "generator": "real_cap_feather_blend",
                "target_class": NAMES[class_id],
                "synthetic": 1,
            })
            created.append(image_path)
    return created


def contact_sheet(paths: list[Path], output: Path, limit: int = 100) -> None:
    tiles = []
    for path in paths[:limit]:
        image = cv2.imread(str(path))
        if image is None:
            continue
        tile = cv2.resize(image, (320, 180), interpolation=cv2.INTER_AREA)
        cv2.putText(tile, path.stem[:44], (8, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (0, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
    if not tiles:
        return
    columns = 5
    rows = []
    blank = np.zeros_like(tiles[0])
    for start in range(0, len(tiles), columns):
        row = tiles[start:start + columns]
        row += [blank] * (columns - len(row))
        rows.append(np.hstack(row))
    cv2.imwrite(str(output), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 92])


def main() -> int:
    args = parse_args()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")
    for split in ("train", "val", "test"):
        (args.output / "dataset" / "images" / split).mkdir(parents=True)
        (args.output / "dataset" / "labels" / split).mkdir(parents=True)

    buoy_model = YOLO(str(args.buoy_model), task="detect")
    nav_model = YOLO(str(args.nav_model), task="detect")
    shape_model = YOLO(str(args.shape_model), task="detect")
    dataset = args.output / "dataset"
    counters = {split: Counter() for split in ("train", "val", "test")}
    candidates: list[Candidate] = []
    cap_library: dict[int, list] = defaultdict(list)
    audit_rows: list[dict] = []
    manifest: list[dict] = []
    seen: set[str] = set()

    process_source(args, shape_model, dataset, counters, candidates, cap_library, seen)
    process_supplement(args, shape_model, dataset, counters, candidates, cap_library, seen)
    process_sessions(args, buoy_model, nav_model, shape_model, dataset, counters,
                     candidates, cap_library, audit_rows, seen)
    synthetic_paths = generate_synthetic(args, dataset, candidates, cap_library,
                                         counters, manifest)

    yaml_text = (
        f"path: {dataset.resolve()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n"
        "names:\n" + "".join(f"  {i}: {name}\n" for i, name in enumerate(NAMES))
    )
    (dataset / "data.yaml").write_text(yaml_text)
    with (args.output / "frame_audit.csv").open("w", newline="") as handle:
        fields = ["session", "image", "kept", "reason", "hash_distance", "blur", "blurry_flag"]
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(audit_rows)
    with (args.output / "synthetic_manifest.csv").open("w", newline="") as handle:
        fields = ["file", "source_image", "source_kind", "generator", "target_class", "synthetic"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(manifest)
    contact_sheet(synthetic_paths, args.output / "synthetic_contact_sheet.jpg")

    summary = {
        "classes": list(NAMES),
        "counts": {split: dict(counters[split]) for split in counters},
        "raw_frames": len(audit_rows),
        "kept_new_frames": sum(int(row["kept"]) for row in audit_rows),
        "blurry_kept_frames": sum(int(row.get("blurry_flag", 0)) for row in audit_rows if row["kept"]),
        "off_numbered_candidates": len(candidates),
        "cap_library": {COLOUR_NAMES[i]: len(cap_library[i]) for i in range(3)},
        "synthetic_images": len(synthetic_paths),
        "warning": "New-session boxes and off-vs-boundary separation are pseudo-labels; inspect the contact sheet before treating training as final.",
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
