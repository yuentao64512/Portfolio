#!/usr/bin/env python3
"""Extract IMG_3922 frames and create reviewed-candidate Model A labels.

The target is the single cyan-base, white-body letter buoy. The letter is not a
class; the full object receives Model A class 2 (buoy_no_led).
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--sample-fps", type=float, default=2.0)
    parser.add_argument(
        "--initial-base",
        default="148,448,228,152",
        help="frame-zero cyan-base x,y,width,height",
    )
    return parser.parse_args()


def find_buoy(frame: np.ndarray) -> tuple[int, int, int, int] | None:
    height, width = frame.shape[:2]
    blue, green, red = cv2.split(frame)
    mask = (
        (blue.astype(np.int16) - red.astype(np.int16) > 12)
        & (green.astype(np.int16) - red.astype(np.int16) > 5)
        & (blue > 115)
        & (green > 105)
    ).astype(np.uint8) * 255

    roi = np.zeros_like(mask)
    roi[int(height * 0.18) : int(height * 0.92), int(width * 0.05) : int(width * 0.95)] = 255
    mask &= roi
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[tuple[float, tuple[int, int, int, int]]] = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area < 250 or w < 55 or w / max(h, 1) < 1.15:
            continue
        cx = x + w / 2
        centrality = 1.0 - min(abs(cx - width / 2) / (width / 2), 1.0)
        lower = min((y + h) / height, 1.0)
        score = area * (0.5 + centrality) * (0.5 + lower)
        candidates.append((score, (x, y, w, h)))
    if not candidates:
        return None

    _, (x, y, w, h) = max(candidates, key=lambda item: item[0])
    x1 = max(0, round(x - 0.08 * w))
    x2 = min(width - 1, round(x + w + 0.08 * w))
    y1 = max(0, round(y - 1.02 * w))
    y2 = min(height - 1, round(y + h + 0.05 * w))
    if x2 <= x1 or y2 <= y1:
        return None
    return x1, y1, x2, y2


def yolo_line(box: tuple[int, int, int, int], width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    cx = ((x1 + x2) / 2) / width
    cy = ((y1 + y2) / 2) / height
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return f"2 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n"


def find_base(frame: np.ndarray, previous: np.ndarray) -> tuple[np.ndarray, int]:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (75, 40, 70), (115, 255, 255))
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
        iterations=1,
    )
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pcx = previous[0] + previous[2] / 2
    pcy = previous[1] + previous[3] / 2
    candidates = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area < 350 or w < 45 or w / max(h, 1) < 0.85:
            continue
        distance = float(np.hypot((x + w / 2) - pcx, (y + h / 2) - pcy))
        if distance > max(140.0, previous[2] * 0.8):
            continue
        candidates.append((distance - min(area / 10000.0, 2.0), np.array([x, y, w, h], dtype=np.float32)))
    if not candidates:
        return previous, 0
    return min(candidates, key=lambda item: item[0])[1], len(candidates)


def full_box_from_base(base: np.ndarray, width: int, height: int) -> np.ndarray:
    x, y, w, h = base
    return np.array(
        [
            max(0, x - 0.04 * w),
            max(0, y - 0.52 * w),
            min(width - 1, x + w + 0.04 * w),
            min(height - 1, y + h + 0.03 * w),
        ],
        dtype=np.float32,
    )


def features(gray: np.ndarray, box: np.ndarray) -> np.ndarray | None:
    height, width = gray.shape
    x1, y1, x2, y2 = np.round(box).astype(int)
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(width - 1, x2), min(height - 1, y2)
    mask = np.zeros_like(gray)
    mask[y1:y2, x1:x2] = 255
    return cv2.goodFeaturesToTrack(
        gray,
        maxCorners=250,
        qualityLevel=0.01,
        minDistance=5,
        mask=mask,
        blockSize=7,
    )


def update_box(previous: np.ndarray, current: np.ndarray, box: np.ndarray) -> tuple[np.ndarray, int]:
    points = features(previous, box)
    if points is None or len(points) < 8:
        return box, 0
    moved, status, _ = cv2.calcOpticalFlowPyrLK(
        previous,
        current,
        points,
        None,
        winSize=(31, 31),
        maxLevel=4,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    if moved is None or status is None:
        return box, 0
    old = points[status.ravel() == 1].reshape(-1, 2)
    new = moved[status.ravel() == 1].reshape(-1, 2)
    if len(old) < 8:
        return box, len(old)
    matrix, inliers = cv2.estimateAffinePartial2D(
        old,
        new,
        method=cv2.RANSAC,
        ransacReprojThreshold=2.5,
        maxIters=2000,
        confidence=0.99,
    )
    if matrix is None:
        return box, len(old)
    scale = float(np.hypot(matrix[0, 0], matrix[0, 1]))
    if not 0.94 <= scale <= 1.06:
        return box, len(old)
    x1, y1, x2, y2 = box
    center = np.array([[[(x1 + x2) / 2, (y1 + y2) / 2]]], dtype=np.float32)
    next_center = cv2.transform(center, matrix)[0, 0]
    # Track translation and scale, but do not repeatedly turn the rotated box
    # into a larger axis-aligned box. That would make the label grow each frame.
    box_width = (x2 - x1) * scale
    box_height = (y2 - y1) * scale
    next_box = np.array(
        [
            next_center[0] - box_width / 2,
            next_center[1] - box_height / 2,
            next_center[0] + box_width / 2,
            next_center[1] + box_height / 2,
        ],
        dtype=np.float32,
    )
    inlier_count = int(inliers.sum()) if inliers is not None else len(old)
    return next_box, inlier_count


def main() -> int:
    args = parse_args()
    images = args.output / "images"
    labels = args.output / "labels"
    previews = args.output / "previews"
    for folder in (images, labels, previews):
        folder.mkdir(parents=True, exist_ok=False)

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f"Cannot open {args.video}")
    source_fps = capture.get(cv2.CAP_PROP_FPS)
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    step = max(1, round(source_fps / args.sample_fps))

    initial = [int(value) for value in args.initial_base.split(",")]
    if len(initial) != 4:
        raise SystemExit("--initial-base must be x,y,width,height")
    base_box = np.array(initial, dtype=np.float32)

    records = []
    preview_frames = []
    for frame_index in range(frame_count):
        ok, frame = capture.read()
        if not ok:
            continue
        base_box, candidates = find_base(frame, base_box)
        tracked_box = full_box_from_base(base_box, frame.shape[1], frame.shape[0])
        if frame_index % step:
            continue

        box = tuple(np.round(tracked_box).astype(int).tolist())

        stem = f"IMG_3922_f{frame_index:06d}"
        cv2.imwrite(str(images / f"{stem}.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        (labels / f"{stem}.txt").write_text(yolo_line(box, frame.shape[1], frame.shape[0]))

        preview = frame.copy()
        x1, y1, x2, y2 = box
        cv2.rectangle(preview, (x1, y1), (x2, y2), (0, 255, 0), 4)
        cv2.putText(preview, "buoy_no_led", (x1, max(30, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.imwrite(str(previews / f"{stem}.jpg"), preview, [cv2.IMWRITE_JPEG_QUALITY, 90])
        small = cv2.resize(preview, (360, 640))
        preview_frames.append(small)
        records.append((frame_index, frame_index / source_fps, stem, f"BASE_REVIEW_{candidates}"))

    capture.release()
    with (args.output / "manifest.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("frame_index", "time_seconds", "stem", "status"))
        writer.writerows(records)

    if preview_frames:
        cols = 6
        blank = np.full_like(preview_frames[0], 255)
        while len(preview_frames) % cols:
            preview_frames.append(blank.copy())
        rows = [cv2.hconcat(preview_frames[i : i + cols]) for i in range(0, len(preview_frames), cols)]
        cv2.imwrite(str(args.output / "contact_sheet.jpg"), cv2.vconcat(rows))

    print(f"selected={sum(1 for r in records if r[2])} missing={sum(1 for r in records if not r[2])}")
    print(args.output / "contact_sheet.jpg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
