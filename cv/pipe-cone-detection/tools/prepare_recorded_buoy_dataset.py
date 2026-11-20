#!/usr/bin/env python3
"""Build a 400-image YOLO dataset from the two 2026-10-02 ZED recordings."""

from __future__ import annotations

import csv
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


PROJECT = Path("/home/robotx/pipe_cones")
SOURCE = PROJECT / "retraining_20261002" / "source_videos"
DATASET = PROJECT / "dataset_recorded_400"
TRACK_SCALE = 0.5


@dataclass(frozen=True)
class Segment:
    video: str
    start: int
    end: int
    samples: int
    split: str
    initial_frame: int
    initial_box: tuple[int, int, int, int]


# Splits are separated temporal blocks, rather than adjacent random frames, so
# validation/test scores measure later viewpoints instead of near duplicates.
SEGMENTS = (
    Segment("20261002_035945_rgb_left.mp4", 60, 220, 130, "train", 60, (1150, 870, 390, 372)),
    Segment("20261002_035945_rgb_left.mp4", 225, 245, 15, "val", 60, (1150, 870, 390, 372)),
    Segment("20261002_035945_rgb_left.mp4", 250, 276, 15, "test", 60, (1150, 870, 390, 372)),
    Segment("20261002_040217_rgb_left.mp4", 30, 430, 190, "train", 30, (1360, 710, 380, 330)),
    Segment("20261002_040217_rgb_left.mp4", 445, 500, 25, "val", 30, (1360, 710, 380, 330)),
    Segment("20261002_040217_rgb_left.mp4", 510, 565, 25, "test", 30, (1360, 710, 380, 330)),
)


def clamp_box(box, width, height):
    x, y, w, h = box
    # The tracker follows the visible buoy well but can shave an edge as its
    # scale changes. A small symmetric margin retains the complete object while
    # keeping water, people, buildings, ropes, and equipment outside the label.
    x1 = max(0.0, x - 0.07 * w)
    y1 = max(0.0, y - 0.07 * h)
    x2 = min(float(width), x + 1.07 * w)
    y2 = min(float(height), y + 1.18 * h)
    return x1, y1, x2, y2


def yolo_box(box, width, height):
    x1, y1, x2, y2 = clamp_box(box, width, height)
    return (
        f"0 {(x1 + x2) / (2 * width):.8f} {(y1 + y2) / (2 * height):.8f} "
        f"{(x2 - x1) / width:.8f} {(y2 - y1) / height:.8f}\n"
    )


def track_video(path: Path, first_frame: int, initial_box):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {path}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.set(cv2.CAP_PROP_POS_FRAMES, first_frame)
    ok, frame = cap.read()
    if not ok:
        raise RuntimeError(f"Could not read frame {first_frame} from {path}")
    tracker = cv2.TrackerCSRT_create()
    small = cv2.resize(frame, None, fx=TRACK_SCALE, fy=TRACK_SCALE)
    tracker.init(small, tuple(int(v * TRACK_SCALE) for v in initial_box))
    boxes = {first_frame: tuple(float(v) for v in initial_box)}
    frames = {first_frame: frame}
    box = initial_box
    for index in range(first_frame + 1, total):
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Could not read frame {index} from {path}")
        tracked, updated = tracker.update(
            cv2.resize(frame, None, fx=TRACK_SCALE, fy=TRACK_SCALE)
        )
        if not tracked:
            raise RuntimeError(f"Buoy tracking failed in {path.name} frame {index}")
        box = tuple(value / TRACK_SCALE for value in updated)
        boxes[index] = box
        frames[index] = frame
    cap.release()
    return frames, boxes, fps


def main():
    if DATASET.exists():
        shutil.rmtree(DATASET)
    for split in ("train", "val", "test"):
        (DATASET / "images" / split).mkdir(parents=True)
        (DATASET / "labels" / split).mkdir(parents=True)

    grouped = {}
    for segment in SEGMENTS:
        grouped.setdefault(segment.video, []).append(segment)

    rows = []
    for video, segments in grouped.items():
        path = SOURCE / video
        first = segments[0]
        frames, boxes, fps = track_video(path, first.initial_frame, first.initial_box)
        for segment in segments:
            indices = np.linspace(segment.start, segment.end, segment.samples, dtype=int)
            if len(set(indices.tolist())) != segment.samples:
                raise RuntimeError(f"Duplicate selection indices in {segment}")
            for index in indices:
                frame = frames[int(index)]
                box = boxes[int(index)]
                height, width = frame.shape[:2]
                stem = f"{Path(video).stem}_f{int(index):04d}"
                image_path = DATASET / "images" / segment.split / f"{stem}.jpg"
                label_path = DATASET / "labels" / segment.split / f"{stem}.txt"
                if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    raise RuntimeError(f"Could not write {image_path}")
                label_path.write_text(yolo_box(box, width, height))
                x1, y1, x2, y2 = clamp_box(box, width, height)
                rows.append({
                    "image": image_path.relative_to(PROJECT),
                    "split": segment.split,
                    "source_video": video,
                    "source_frame": int(index),
                    "source_time_s": f"{int(index) / fps:.3f}",
                    "x1": f"{x1:.2f}", "y1": f"{y1:.2f}",
                    "x2": f"{x2:.2f}", "y2": f"{y2:.2f}",
                })

    with (DATASET / "manifest.csv").open("w", newline="") as output:
        writer = csv.DictWriter(output, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    counts = {split: sum(row["split"] == split for row in rows) for split in ("train", "val", "test")}
    if counts != {"train": 320, "val": 40, "test": 40}:
        raise RuntimeError(f"Unexpected split counts: {counts}")
    print(f"Created {len(rows)} tightly labelled images in {DATASET}: {counts}")


if __name__ == "__main__":
    main()
