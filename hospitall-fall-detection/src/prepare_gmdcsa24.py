#!/usr/bin/env python3
"""Build a segment-level manifest for the GMDCSA24 fall dataset."""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path

CLASS_MAP = {
    "falling": "falling_down",
    "fall": "falling_down",
    "walking": "walking",
    "standing": "standing_still",
    "sitting": "sitting_kneeling",
    "sleeping": "lying_down",
}


def parse_segments(raw: str) -> list[tuple[str, float, float]]:
    segments: list[tuple[str, float, float]] = []
    pattern = re.compile(
        r"([A-Za-z][A-Za-z /_-]*?)(?:\s*\([^)]*\))?\s*\[\s*([0-9.]+)\s*to\s*([0-9.]+)\s*\]"
    )
    for match in pattern.finditer(raw or ""):
        label = " ".join(match.group(1).split())
        start = float(match.group(2))
        end = float(match.group(3))
        mapped = CLASS_MAP.get(label.lower())
        if mapped and end > start:
            segments.append((mapped, start, end))
    return segments


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-duration", type=float, default=0.4)
    parser.add_argument("--fall-transition-sec", type=float, default=1.6)
    parser.add_argument("--post-fall-lying-sec", type=float, default=1.2)
    args = parser.parse_args()

    rows: list[dict[str, str]] = []
    for csv_path in sorted(args.dataset.glob("Subject */*.csv")):
        subject = csv_path.parent.name
        split = csv_path.stem
        video_dir = csv_path.parent / split
        with csv_path.open(newline="", encoding="utf-8-sig") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                file_name = (row.get("File Name") or "").strip()
                classes = row.get(" Classes") or row.get("Classes") or ""
                video_path = video_dir / file_name
                if not video_path.exists():
                    continue
                expanded_segments: list[tuple[str, float, float, str]] = []
                for label, start, end in parse_segments(classes):
                    if label == "falling_down":
                        fall_end = min(end, start + args.fall_transition_sec)
                        expanded_segments.append((label, start, fall_end, "csv_fall_transition"))
                        if end - start >= args.fall_transition_sec + args.post_fall_lying_sec:
                            expanded_segments.append(
                                ("lying_down", max(fall_end, end - args.post_fall_lying_sec), end, "derived_post_fall_lying")
                            )
                    else:
                        expanded_segments.append((label, start, end, "csv"))

                for idx, (label, start, end, source) in enumerate(expanded_segments):
                    duration = end - start
                    if duration < args.min_duration:
                        continue
                    rows.append(
                        {
                            "segment_id": f"{subject}_{split}_{Path(file_name).stem}_{idx:02d}",
                            "subject": subject,
                            "source_split": split,
                            "video_path": str(video_path.resolve()),
                            "file_name": file_name,
                            "label": label,
                            "start_sec": f"{start:.3f}",
                            "end_sec": f"{end:.3f}",
                            "duration_sec": f"{duration:.3f}",
                            "segment_source": source,
                            "description": row.get("Description", "").strip(),
                            "raw_classes": classes.strip(),
                        }
                    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
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
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    counts: dict[str, int] = {}
    for row in rows:
        counts[row["label"]] = counts.get(row["label"], 0) + 1
    print(f"Wrote {len(rows)} segments to {args.output}")
    for label, count in sorted(counts.items()):
        print(f"{label}: {count}")


if __name__ == "__main__":
    main()
