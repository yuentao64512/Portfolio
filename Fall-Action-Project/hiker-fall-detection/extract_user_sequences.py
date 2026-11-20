#!/usr/bin/env python3
"""Extract YOLO11 pose sequences from the reviewed user segments."""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm
from ultralytics import YOLO


def pose(result, width: int, height: int) -> np.ndarray:
    out = np.zeros((17, 3), dtype=np.float32)
    if result.keypoints is None or result.boxes is None or len(result.boxes) == 0:
        return out
    boxes = result.boxes.xyxy.detach().cpu().numpy()
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    i = int(np.argmax(areas))
    xy = result.keypoints.xy[i].detach().cpu().numpy()
    conf = result.keypoints.conf[i].detach().cpu().numpy()
    out[:, :2] = xy[:, :2] / [max(width, 1), max(height, 1)]
    out[:, 2] = conf
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=Path("data/user_segments.csv"))
    parser.add_argument("--output", type=Path, default=Path("data/user_pose_sequences.npz"))
    parser.add_argument("--model", type=Path, default=Path("/home/robotx/fall_action_project/yolo11n-pose.pt"))
    parser.add_argument("--sample-fps", type=float, default=8.0)
    parser.add_argument("--seq-len", type=int, default=12)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    rows = list(csv.DictReader(args.manifest.open(newline="", encoding="utf-8")))
    model = YOLO(str(args.model), task="pose")
    sequences, labels, segment_ids, groups, rates = [], [], [], [], []
    for row in tqdm(rows, desc="user pose segments"):
        cap = cv2.VideoCapture(row["video_path"])
        start, end = float(row["start_sec"]), float(row["end_sec"])
        times = np.linspace(start, max(start, end - 1.0 / args.sample_fps), args.seq_len)
        seq, valid = [], 0
        for sec in times:
            cap.set(cv2.CAP_PROP_POS_MSEC, float(sec) * 1000.0)
            ok, frame = cap.read()
            if not ok:
                seq.append(np.zeros((17, 3), dtype=np.float32))
                continue
            h, w = frame.shape[:2]
            kp = pose(model.predict(frame, imgsz=args.imgsz, device=args.device, verbose=False)[0], w, h)
            valid += int(kp[:, 2].max() > 0)
            seq.append(kp)
        cap.release()
        sequences.append(np.stack(seq))
        labels.append(row["label"])
        segment_ids.append(row["segment_id"])
        groups.append(row["group"])
        rates.append(valid / len(times))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, X=np.stack(sequences), y=np.array(labels),
                        segment_id=np.array(segment_ids), group=np.array(groups),
                        source=np.array(["user_recording"] * len(labels)),
                        detection_rate=np.array(rates, dtype=np.float32))
    print(f"Wrote {len(labels)} sequences to {args.output}; mean detection rate {np.mean(rates):.3f}")


if __name__ == "__main__":
    main()
