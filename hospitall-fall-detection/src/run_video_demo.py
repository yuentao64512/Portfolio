#!/usr/bin/env python3
"""Run the trained pose classifier on one video with fall/lying timer logic."""

from __future__ import annotations

import argparse
import pickle
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def largest_person_keypoints(result, width: int, height: int) -> np.ndarray:
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


def featurize(X: np.ndarray) -> np.ndarray:
    xy = X[..., :2]
    conf = X[..., 2:3]
    velocity = np.diff(xy, axis=1, prepend=xy[:, :1])
    features = [
        xy.mean(axis=1).reshape(len(X), -1),
        xy.std(axis=1).reshape(len(X), -1),
        np.abs(velocity).mean(axis=1).reshape(len(X), -1),
        np.abs(velocity).max(axis=1).reshape(len(X), -1),
        conf.mean(axis=1).reshape(len(X), -1),
    ]
    return np.concatenate(features, axis=1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--classifier", type=Path, default=Path("models/current_pose_rf.pkl"))
    parser.add_argument("--pose-model", default="yolo11n-pose.pt")
    parser.add_argument("--sample-fps", type=float, default=5.0)
    parser.add_argument("--seq-len", type=int, default=32)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--lying-alert-sec", type=float, default=3.0)
    args = parser.parse_args()

    with args.classifier.open("rb") as handle:
        bundle = pickle.load(handle)
    clf = bundle["pipeline"]
    pose = YOLO(args.pose_model)

    cap = cv2.VideoCapture(str(args.video))
    source_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    duration = total_frames / source_fps if total_frames else 0.0
    times = np.arange(0.0, max(duration, 0.01), 1.0 / args.sample_fps)

    buffer: deque[np.ndarray] = deque(maxlen=args.seq_len)
    fall_seen = False
    lying_started_at: float | None = None
    last_reported_second = -1

    for sec in times:
        cap.set(cv2.CAP_PROP_POS_MSEC, sec * 1000.0)
        ok, frame = cap.read()
        if not ok:
            continue
        h, w = frame.shape[:2]
        result = pose.predict(frame, imgsz=args.imgsz, device=args.device, verbose=False)[0]
        buffer.append(largest_person_keypoints(result, w, h))
        if len(buffer) < max(4, args.seq_len // 4):
            continue

        seq = resize_sequence(list(buffer), args.seq_len)[None, ...]
        probs = clf.predict_proba(featurize(seq))[0]
        classes = clf.classes_
        pred = classes[int(np.argmax(probs))]

        if pred == "falling_down":
            fall_seen = True
            lying_started_at = None
        elif fall_seen and pred == "lying_down":
            if lying_started_at is None:
                lying_started_at = sec
        elif pred != "lying_down":
            lying_started_at = None

        alert = fall_seen and lying_started_at is not None and (sec - lying_started_at) >= args.lying_alert_sec
        if int(sec) != last_reported_second:
            last_reported_second = int(sec)
            top = ", ".join(
                f"{name}:{prob:.2f}"
                for name, prob in sorted(zip(classes, probs), key=lambda item: item[1], reverse=True)[:3]
            )
            print(f"{sec:6.2f}s  pred={pred:16s}  top={top}  alert={alert}")
    cap.release()


if __name__ == "__main__":
    main()
