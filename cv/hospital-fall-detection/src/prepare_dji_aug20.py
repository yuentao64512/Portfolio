#!/usr/bin/env python3
"""Prepare a labeled manifest from the user's Aug 20 DJI top-down clips."""

from __future__ import annotations

import argparse
import csv
import shutil
from pathlib import Path

ANNOTATIONS = [
    # file name, label, start_sec, end_sec, window_sec, stride_sec
    ("DJI_20260820124247_0033_D.MP4", "walking", 2.0, 24.5, 3.0, 1.5),
    ("DJI_20260820124323_0034_D.MP4", "standing_still", 0.5, 13.5, 3.0, 1.5),
    ("DJI_20260820124345_0035_D.MP4", "running", 0.5, 14.0, 3.0, 1.5),
    ("DJI_20260820124412_0036_D.MP4", "lying_down", 0.5, 18.5, 3.0, 1.5),
    ("DJI_20260820124434_0037_D.MP4", "lying_down", 0.5, 25.5, 3.0, 1.5),
    ("DJI_20260820124528_0038_D.MP4", "standing_still", 0.5, 7.0, 3.0, 1.0),
    ("DJI_20260820124541_0039_D.MP4", "standing_still", 0.0, 1.6, 1.6, 1.0),
    ("DJI_20260820124541_0039_D.MP4", "falling_down", 1.6, 4.8, 1.6, 0.5),
    ("DJI_20260820124541_0039_D.MP4", "lying_down", 4.8, 8.4, 2.0, 0.8),
]


def windows(start: float, end: float, window_sec: float, stride_sec: float) -> list[tuple[float, float]]:
    if end <= start:
        return []
    if end - start <= window_sec:
        return [(start, end)]
    rows = []
    t = start
    while t + window_sec <= end + 1e-6:
        rows.append((t, t + window_sec))
        t += stride_sec
    if rows and rows[-1][1] < end - 0.25:
        rows.append((max(start, end - window_sec), end))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-dir", type=Path, required=True)
    parser.add_argument("--project-raw-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--copy", action="store_true")
    args = parser.parse_args()

    args.project_raw_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, str]] = []
    for file_name, label, start, end, window_sec, stride_sec in ANNOTATIONS:
        src = args.source_dir / file_name
        if not src.exists():
            raise FileNotFoundError(src)
        dst = args.project_raw_dir / file_name
        if args.copy and not dst.exists():
            shutil.copy2(src, dst)
        video_path = dst if dst.exists() else src
        for i, (a, b) in enumerate(windows(start, end, window_sec, stride_sec)):
            rows.append(
                {
                    "segment_id": f"DJI_AUG20_{Path(file_name).stem}_{label}_{i:03d}",
                    "subject": "DJI_AUG20",
                    "source_split": "custom",
                    "video_path": str(video_path.resolve()),
                    "file_name": file_name,
                    "label": label,
                    "start_sec": f"{a:.3f}",
                    "end_sec": f"{b:.3f}",
                    "duration_sec": f"{b-a:.3f}",
                    "segment_source": "manual_dji_aug20",
                    "description": "User DJI top-down clip",
                    "raw_classes": label,
                }
            )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "segment_id",
        "subject",
        "source_split",
        "video_path",
        "file_name",
        "label",
        "start_sec",
        "end_sec",
        "duration_sec",
        "segment_source",
        "description",
        "raw_classes",
    ]
    with args.output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    counts: dict[str, int] = {}
    for row in rows:
        counts[row["label"]] = counts.get(row["label"], 0) + 1
    print(f"Wrote {len(rows)} DJI segments to {args.output}")
    for label, count in sorted(counts.items()):
        print(f"{label}: {count}")


if __name__ == "__main__":
    main()
