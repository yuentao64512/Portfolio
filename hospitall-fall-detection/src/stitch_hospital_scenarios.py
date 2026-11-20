#!/usr/bin/env python3
"""Build one labelled hospital-fall test video from multiple scenario clips."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


DEFAULT_SEGMENTS = [
    ("Bed to floor transition", "test_videos/synthetic_hospital_fall/synthetic_hospital_bed_to_floor.mp4"),
    ("Walking then fall", "test_videos/generated_hospital/hospital_cctv_walking_then_fall.mp4"),
    ("Walking only control", "test_videos/generated_hospital/hospital_cctv_walking_only_control.mp4"),
    ("Floor only control", "test_videos/generated_hospital/hospital_cctv_floor_only_control.mp4"),
]


def put_text(frame: np.ndarray, text: str, origin: tuple[int, int], scale: float = 0.8) -> None:
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2, cv2.LINE_AA)


def labelled_frame(frame: np.ndarray, label: str, index: int, total: int, width: int, height: int) -> np.ndarray:
    if frame.shape[1] != width or frame.shape[0] != height:
        frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
    output = frame.copy()
    cv2.rectangle(output, (0, 0), (width, 58), (20, 20, 20), -1)
    put_text(output, f"Scenario {index}/{total}: {label}", (18, 39), 0.78)
    return output


def slate(width: int, height: int, label: str, seconds: float) -> np.ndarray:
    frame = np.full((height, width, 3), 22, dtype=np.uint8)
    title = f"NEXT: {label}"
    subtitle = f"{seconds:.0f}s reset interval (longer than the predictor's 10s transition memory)"
    (tw, _), _ = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 1.05, 3)
    (sw, _), _ = cv2.getTextSize(subtitle, cv2.FONT_HERSHEY_SIMPLEX, 0.58, 2)
    put_text(frame, title, ((width - tw) // 2, height // 2 - 12), 1.05)
    put_text(frame, subtitle, ((width - sw) // 2, height // 2 + 38), 0.58)
    return frame


def parse_segment(value: str) -> tuple[str, str]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("Use LABEL=PATH for --segment")
    label, path = value.split("=", 1)
    if not label.strip() or not path.strip():
        raise argparse.ArgumentTypeError("Both LABEL and PATH are required")
    return label.strip(), path.strip()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--segment", action="append", type=parse_segment, help="LABEL=PATH; repeat for each clip")
    parser.add_argument("--output", type=Path, default=Path("test_videos/generated_hospital/hospital_all_scenarios_stitched.mp4"))
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--separator-sec", type=float, default=11.0)
    args = parser.parse_args()

    segments = args.segment or DEFAULT_SEGMENTS
    manifest_path = args.manifest or args.output.with_suffix(".csv")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)

    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), args.fps, (args.width, args.height)
    )
    if not writer.isOpened():
        raise SystemExit(f"Could not create {args.output}")

    rows: list[dict[str, object]] = []
    output_frames = 0
    try:
        for segment_index, (label, path_text) in enumerate(segments, start=1):
            source_path = Path(path_text)
            if not source_path.is_file():
                raise SystemExit(f"Missing input: {source_path}")

            if segment_index > 1:
                reset_frame = slate(args.width, args.height, label, args.separator_sec)
                reset_frames = int(round(args.separator_sec * args.fps))
                for _ in range(reset_frames):
                    writer.write(reset_frame)
                output_frames += reset_frames

            capture = cv2.VideoCapture(str(source_path))
            if not capture.isOpened():
                raise SystemExit(f"Could not open {source_path}")
            source_fps = float(capture.get(cv2.CAP_PROP_FPS) or args.fps)
            source_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
            start_frame = output_frames
            input_index = 0
            segment_written = 0
            last_frame = None
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                last_frame = labelled_frame(
                    frame, label, segment_index, len(segments), args.width, args.height
                )
                input_index += 1
                target_written = int(round(input_index * args.fps / source_fps))
                while segment_written < target_written:
                    writer.write(last_frame)
                    segment_written += 1
                    output_frames += 1
            capture.release()

            if segment_written == 0 or last_frame is None:
                raise SystemExit(f"No frames read from {source_path}")
            expected_frames = int(round(source_frames * args.fps / source_fps)) if source_frames else segment_written
            rows.append(
                {
                    "scenario": segment_index,
                    "label": label,
                    "source": str(source_path),
                    "source_fps": source_fps,
                    "source_frames": source_frames,
                    "stitched_start_sec": start_frame / args.fps,
                    "stitched_end_sec": output_frames / args.fps,
                    "stitched_frames": segment_written,
                    "expected_frames": expected_frames,
                }
            )
    finally:
        writer.release()

    with manifest_path.open("w", newline="", encoding="utf-8") as stream:
        table = csv.DictWriter(stream, fieldnames=list(rows[0].keys()))
        table.writeheader()
        table.writerows(rows)

    print(f"output={args.output}")
    print(f"manifest={manifest_path}")
    print(f"frames={output_frames} fps={args.fps:.3f} duration_sec={output_frames / args.fps:.3f}")


if __name__ == "__main__":
    main()
