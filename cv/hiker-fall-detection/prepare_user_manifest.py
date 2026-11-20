#!/usr/bin/env python3
"""Create hand-reviewed action segments for the recordings in this workspace.

The recordings contain transitions, so a whole-file label would be misleading.
Times below are approximate, conservative labels used for training; the raw
videos remain the source of truth and should be re-reviewed before publication.
"""
from __future__ import annotations

import csv
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "data" / "user_segments.csv"

# file, label, start, end.  All six recordings are included.
SEGMENTS = [
    ("IMG_7562.MOV", "standing", 0.0, 1.3),
    ("IMG_7562.MOV", "walking", 1.3, 2.8),
    ("IMG_7562.MOV", "sitting", 2.8, 4.7),
    ("IMG_7562.MOV", "standing", 4.7, 6.2),
    ("IMG_7562.MOV", "walking", 6.2, 8.3),
    ("IMG_7563.MOV", "standing", 0.0, 1.7),
    ("IMG_7563.MOV", "falling_down", 1.7, 3.2),
    ("IMG_7563.MOV", "lying_down", 3.2, 4.46),
    ("IMG_7564.MOV", "standing", 0.0, 1.4),
    ("IMG_7564.MOV", "walking", 1.4, 2.7),
    ("IMG_7564.MOV", "sitting", 2.7, 4.0),
    ("IMG_7564.MOV", "lying_down", 4.0, 5.66),
    ("IMG_7565.MOV", "standing", 0.0, 1.5),
    ("IMG_7565.MOV", "falling_down", 1.5, 2.7),
    ("IMG_7565.MOV", "lying_down", 2.7, 5.13),
    ("IMG_7566.MOV", "walking", 0.0, 2.3),
    ("IMG_7566.MOV", "lying_down", 2.3, 4.56),
    ("IMG_7567.MOV", "standing", 0.0, 2.5),
    ("IMG_7567.MOV", "sitting", 2.5, 5.8),
    ("IMG_7567.MOV", "standing", 5.8, 8.7),
    ("IMG_7567.MOV", "walking", 8.7, 12.13),
]


def main() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    fields = ["segment_id", "group", "source", "video_path", "label", "start_sec", "end_sec"]
    with OUT.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for index, (name, label, start, end) in enumerate(SEGMENTS):
            path = ROOT / name
            if not path.exists():
                raise FileNotFoundError(path)
            writer.writerow({
                "segment_id": f"USER_{Path(name).stem}_{index:03d}",
                "group": Path(name).stem,
                "source": "user_recording",
                "video_path": str(path.resolve()),
                "label": label,
                "start_sec": f"{start:.3f}",
                "end_sec": f"{end:.3f}",
            })
    print(f"Wrote {len(SEGMENTS)} segments to {OUT}")
    for label in sorted({row[1] for row in SEGMENTS}):
        print(label, sum(row[1] == label for row in SEGMENTS))


if __name__ == "__main__":
    main()
