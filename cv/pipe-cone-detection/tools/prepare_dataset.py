#!/usr/bin/env python3
"""Extract 120 duration-weighted frames and bootstrap pipe-cone boxes by tracking."""

from __future__ import annotations

import csv
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


ROOT = Path("/home/robotx")
PROJECT = ROOT / "pipe_cones"
DATASET = PROJECT / "dataset"
TRACK_SCALE = 0.5
TRACK_STRIDE = 2


@dataclass(frozen=True)
class Source:
    stem: str
    samples: int
    split: str
    # Initial x, y, width, height around the device only (support legs excluded).
    initial_box: tuple[int, int, int, int]


SOURCES = (
    Source("IMG_7816", 14, "train", (286, 825, 528, 438)),
    Source("IMG_7817", 33, "train", (388, 788, 250, 225)),
    Source("IMG_7818", 8, "test", (460, 805, 95, 108)),
    Source("IMG_7819", 27, "val", (422, 750, 318, 258)),
    Source("IMG_7820", 38, "train", (414, 925, 263, 280)),
)


def clamp_box(box: tuple[float, float, float, float], width: int, height: int):
    x, y, w, h = box
    x1 = max(0.0, x)
    y1 = max(0.0, y)
    x2 = min(float(width), x + w)
    y2 = min(float(height), y + h)
    return x1, y1, x2, y2


def yolo_box(box: tuple[float, float, float, float], width: int, height: int) -> str:
    x1, y1, x2, y2 = clamp_box(box, width, height)
    xc = (x1 + x2) / (2 * width)
    yc = (y1 + y2) / (2 * height)
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return f"0 {xc:.8f} {yc:.8f} {bw:.8f} {bh:.8f}\n"


def main() -> None:
    if DATASET.exists():
        shutil.rmtree(DATASET)
    for split in ("train", "val", "test"):
        (DATASET / "images" / split).mkdir(parents=True)
        (DATASET / "labels" / split).mkdir(parents=True)

    manifest_rows: list[dict[str, object]] = []
    for source in SOURCES:
        video_path = ROOT / f"{source.stem}.MOV"
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open {video_path}")
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        selected = set(np.linspace(0, total - 1, source.samples, dtype=int).tolist())

        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Could not read first frame of {video_path}")
        tracker = cv2.TrackerCSRT_create()
        tracking_frame = cv2.resize(
            frame, None, fx=TRACK_SCALE, fy=TRACK_SCALE, interpolation=cv2.INTER_AREA
        )
        tracking_box = tuple(int(round(value * TRACK_SCALE)) for value in source.initial_box)
        tracker.init(tracking_frame, tracking_box)
        current_box: tuple[float, float, float, float] = source.initial_box

        for frame_index in range(total):
            if frame_index > 0:
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f"Could not read frame {frame_index} of {video_path}")
                if frame_index % TRACK_STRIDE == 0 or frame_index in selected:
                    tracking_frame = cv2.resize(
                        frame,
                        None,
                        fx=TRACK_SCALE,
                        fy=TRACK_SCALE,
                        interpolation=cv2.INTER_AREA,
                    )
                    tracked, updated = tracker.update(tracking_frame)
                    if not tracked:
                        raise RuntimeError(
                            f"Tracking failed: {source.stem} frame {frame_index}"
                        )
                    current_box = tuple(value / TRACK_SCALE for value in updated)

            if frame_index not in selected:
                continue
            name = f"{source.stem}_f{frame_index:04d}"
            image_path = DATASET / "images" / source.split / f"{name}.jpg"
            label_path = DATASET / "labels" / source.split / f"{name}.txt"
            height, width = frame.shape[:2]
            if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f"Could not write {image_path}")
            label_path.write_text(yolo_box(current_box, width, height))
            x1, y1, x2, y2 = clamp_box(current_box, width, height)
            manifest_rows.append(
                {
                    "image": image_path.relative_to(PROJECT),
                    "split": source.split,
                    "source_video": video_path.name,
                    "frame": frame_index,
                    "time_seconds": f"{frame_index / fps:.3f}",
                    "x1": f"{x1:.2f}",
                    "y1": f"{y1:.2f}",
                    "x2": f"{x2:.2f}",
                    "y2": f"{y2:.2f}",
                }
            )
        cap.release()

    with (PROJECT / "manifest.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=manifest_rows[0].keys())
        writer.writeheader()
        writer.writerows(manifest_rows)
    print(f"Extracted {len(manifest_rows)} images to {DATASET}")


if __name__ == "__main__":
    main()
