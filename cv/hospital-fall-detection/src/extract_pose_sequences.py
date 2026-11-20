#!/usr/bin/env python3
"""Extract fixed-length YOLO pose sequences from video segments."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm
from ultralytics import YOLO


def sample_times(start: float, end: float, fps: float) -> list[float]:
    step = 1.0 / fps
    count = max(1, int((end - start) * fps))
    return [start + (i + 0.5) * step for i in range(count) if start + (i + 0.5) * step < end]


def read_frame_at(capture: cv2.VideoCapture, sec: float):
    capture.set(cv2.CAP_PROP_POS_MSEC, sec * 1000.0)
    ok, frame = capture.read()
    return frame if ok else None


def largest_person_keypoints(result, width: int, height: int) -> np.ndarray:
    # COCO pose has 17 keypoints. Store x, y normalized to 0..1 and confidence.
    empty = np.zeros((17, 3), dtype=np.float32)
    if result.keypoints is None or result.boxes is None or len(result.boxes) == 0:
        return empty
    boxes = result.boxes.xyxy.detach().cpu().numpy()
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    idx = int(np.argmax(areas))
    xy = result.keypoints.xy[idx].detach().cpu().numpy()
    conf = result.keypoints.conf[idx].detach().cpu().numpy()
    out = empty.copy()
    n = min(out.shape[0], xy.shape[0])
    out[:n, 0] = xy[:n, 0] / max(width, 1)
    out[:n, 1] = xy[:n, 1] / max(height, 1)
    out[:n, 2] = conf[:n]
    return out


def resize_sequence(seq: list[np.ndarray], seq_len: int) -> np.ndarray:
    if not seq:
        return np.zeros((seq_len, 17, 3), dtype=np.float32)
    arr = np.stack(seq).astype(np.float32)
    if len(arr) == seq_len:
        return arr
    src_idx = np.linspace(0, len(arr) - 1, num=seq_len)
    lo = np.floor(src_idx).astype(int)
    hi = np.ceil(src_idx).astype(int)
    frac = (src_idx - lo).reshape(-1, 1, 1)
    return arr[lo] * (1.0 - frac) + arr[hi] * frac


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", default="yolo11n-pose.pt")
    parser.add_argument("--sample-fps", type=float, default=5.0)
    parser.add_argument("--seq-len", type=int, default=32)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    with args.manifest.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if args.limit:
        rows = rows[: args.limit]

    model = YOLO(args.model)
    sequences = []
    labels = []
    segment_ids = []
    detection_rates = []

    for row in tqdm(rows, desc="pose segments"):
        video_path = row["video_path"]
        start = float(row["start_sec"])
        end = float(row["end_sec"])
        cap = cv2.VideoCapture(video_path)
        seq: list[np.ndarray] = []
        valid = 0
        for sec in sample_times(start, end, args.sample_fps):
            frame = read_frame_at(cap, sec)
            if frame is None:
                continue
            h, w = frame.shape[:2]
            result = model.predict(frame, imgsz=args.imgsz, device=args.device, verbose=False)[0]
            kp = largest_person_keypoints(result, w, h)
            if float(kp[:, 2].max()) > 0:
                valid += 1
            seq.append(kp)
        cap.release()
        sequences.append(resize_sequence(seq, args.seq_len))
        labels.append(row["label"])
        segment_ids.append(row["segment_id"])
        detection_rates.append(valid / max(len(seq), 1))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.output,
        X=np.stack(sequences).astype(np.float32),
        y=np.array(labels),
        segment_id=np.array(segment_ids),
        detection_rate=np.array(detection_rates, dtype=np.float32),
    )
    print(f"Wrote {len(labels)} pose sequences to {args.output}")
    print(f"Mean detection rate: {float(np.mean(detection_rates)):.3f}")


if __name__ == "__main__":
    main()
