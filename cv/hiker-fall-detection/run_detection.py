#!/usr/bin/env python3
"""Progressive normal-camera multi-person pose and action detector."""
import argparse
import csv
import json
import pickle
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment
from ultralytics import YOLO

from action_core import (
    BODY_EDGES,
    FPS,
    FULL_EDGES,
    FEATURE_VERSION,
    WINDOW,
    LyingTimer,
    detections_from_result,
    featurize,
)

ROOT = Path(__file__).resolve().parent


def draw_text(image, value, x, y, color=(230, 230, 230)):
    cv2.putText(image, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, value, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


class Track:
    def __init__(self, track_id, alert_seconds, gap_tolerance):
        self.id = track_id
        self.timer = LyingTimer(alert_seconds, gap_tolerance)
        self.buffer = deque(maxlen=WINDOW * 2)
        self.last = -1.0
        self.box = None
        self.pose = None
        self.label = "warming_up"
        self.score = 0.0
        self.probabilities = None

    def update(self, detection, now, classifier, threshold):
        if self.last >= 0 and now - self.last > 0.4:
            self.buffer.clear()
            self.probabilities = None
            self.timer.update("uncertain", now, False)
        self.last = now
        self.box = detection["box"]
        self.pose = detection["pose"]
        self.buffer.append((now, self.pose))
        if len(self.buffer) < 4 or now - self.buffer[0][0] < 0.35:
            self.label = "warming_up"
            return
        timestamps = np.asarray([item[0] for item in self.buffer])
        poses = np.stack([item[1] for item in self.buffer])
        grid = now - np.arange(WINDOW - 1, -1, -1) / FPS
        indexes = np.searchsorted(timestamps, grid, side="right") - 1
        sequence = poses[np.clip(indexes, 0, len(poses) - 1)]
        raw = classifier.predict_proba(featurize(sequence))[0]
        self.probabilities = raw if self.probabilities is None else 0.65 * raw + 0.35 * self.probabilities
        best = int(np.argmax(self.probabilities))
        self.score = float(self.probabilities[best])
        label = str(classifier.classes_[best])
        if self.score < threshold:
            label = "uncertain"
        self.label = self.timer.update(label, now, observed=label != "uncertain")


def associate(tracks, detections, now):
    ids = [track_id for track_id, track in tracks.items() if now - track.last <= 0.5]
    if not ids or not detections:
        return [], list(range(len(detections)))
    costs = np.full((len(ids), len(detections)), 100.0, np.float32)
    for row, track_id in enumerate(ids):
        first = tracks[track_id].box
        for column, detection in enumerate(detections):
            second = detection["box"]
            top_left = np.maximum(first[:2], second[:2])
            bottom_right = np.minimum(first[2:], second[2:])
            intersection = float(np.maximum(0, bottom_right - top_left).prod())
            first_area = float((first[2:] - first[:2]).prod())
            second_area = float((second[2:] - second[:2]).prod())
            iou = intersection / max(first_area + second_area - intersection, 1.0)
            distance = np.linalg.norm((first[:2] + first[2:] - second[:2] - second[2:]) / 2)
            distance /= max(np.linalg.norm(first[2:] - first[:2]), 1.0)
            if distance < 0.55:
                costs[row, column] = 1 - iou + 0.4 * distance
    rows, columns = linear_sum_assignment(costs)
    pairs = [(ids[row], int(column)) for row, column in zip(rows, columns) if costs[row, column] < 1.25]
    matched = {column for _, column in pairs}
    return pairs, [index for index in range(len(detections)) if index not in matched]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default="0", help="Camera index or video path")
    parser.add_argument("--pose-model", type=Path, required=True)
    parser.add_argument("--classifier", type=Path, default=ROOT / "models/actions.pkl")
    parser.add_argument("--max-people", type=int, default=1)
    parser.add_argument("--skeleton", choices=["body", "full", "none"], default="body")
    parser.add_argument("--imgsz", type=int, default=384)
    parser.add_argument("--device", default="0")
    parser.add_argument("--lying-seconds", type=float, default=10.0)
    parser.add_argument("--gap-tolerance", type=float, default=0.5)
    parser.add_argument("--confidence", type=float, default=0.45)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--title", default="RSE4207 normal-camera detection")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--log", type=Path)
    parser.add_argument("--stats", type=Path)
    parser.add_argument("--max-seconds", type=float, default=0.0)
    args = parser.parse_args()
    if args.max_people < 1:
        parser.error("--max-people must be at least 1")
    if args.lying_seconds <= 0:
        parser.error("--lying-seconds must be positive")

    with args.classifier.open("rb") as handle:
        bundle = pickle.load(handle)
    if bundle.get("feature_version") != FEATURE_VERSION:
        raise RuntimeError("Classifier feature version does not match this runner")
    classifier = bundle["pipeline"]
    # n_jobs belongs to the final estimator, not to the sklearn Pipeline.
    # Keep inference single-threaded on embedded hardware when supported.
    if hasattr(classifier, "named_steps"):
        for estimator in classifier.named_steps.values():
            if hasattr(estimator, "n_jobs"):
                estimator.set_params(n_jobs=1)
    elif hasattr(classifier, "n_jobs"):
        classifier.set_params(n_jobs=1)
    pose_model = YOLO(str(args.pose_model), task="pose")

    is_camera = args.source.isdigit()
    source = int(args.source) if is_camera else args.source
    capture = cv2.VideoCapture(source, cv2.CAP_V4L2 if is_camera else cv2.CAP_ANY)
    if is_camera:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        capture.set(cv2.CAP_PROP_FPS, 30)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open camera/video {source}")

    source_fps = capture.get(cv2.CAP_PROP_FPS) or 25.0
    tracks = {}
    next_id = 1
    writer = None
    frame_index = 0
    processed = 0
    next_sample = 0.0
    started = time.monotonic()
    processing_times = []
    log_path = args.log or ROOT / "reports" / ("live_" + time.strftime("%Y%m%d_%H%M%S") + ".csv")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_handle = log_path.open("w", newline="")
    log = csv.writer(log_handle)
    log.writerow(["source_seconds", "track_id", "label", "confidence", "lying_seconds", "alert"])
    edges = FULL_EDGES if args.skeleton == "full" else BODY_EDGES
    print(f"RUNNING | max_people={args.max_people} | skeleton={args.skeleton} | Q to quit", flush=True)

    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            now = time.monotonic() - started if is_camera else frame_index / source_fps
            frame_index += 1
            if args.max_seconds and now >= args.max_seconds:
                break
            if now + 1e-6 < next_sample:
                continue
            next_sample = (np.floor(now * FPS) + 1) / FPS
            tick = time.perf_counter()
            height, width = frame.shape[:2]
            result = pose_model.predict(
                frame, imgsz=args.imgsz, device=args.device, verbose=False,
                conf=0.25, max_det=args.max_people,
            )[0]
            detections = detections_from_result(result, width, height)[:args.max_people]
            pairs, remaining = associate(tracks, detections, now)
            seen = set()
            for track_id, detection_index in pairs:
                tracks[track_id].update(detections[detection_index], now, classifier, args.confidence)
                seen.add(track_id)
            for detection_index in remaining:
                track = Track(next_id, args.lying_seconds, args.gap_tolerance)
                tracks[next_id] = track
                track.update(detections[detection_index], now, classifier, args.confidence)
                seen.add(next_id)
                next_id += 1
            for track_id in list(tracks):
                if track_id not in seen:
                    tracks[track_id].timer.update("uncertain", now, False)
                    if now - tracks[track_id].last > 0.5:
                        del tracks[track_id]

            for track_id in sorted(seen):
                track = tracks[track_id]
                color = (40, 40, 255) if track.timer.alert else (40, 225, 80)
                box = track.box.astype(int)
                cv2.rectangle(frame, tuple(box[:2]), tuple(box[2:]), color, 2)
                if args.skeleton != "none":
                    for first, second in edges:
                        if min(track.pose[first, 2], track.pose[second, 2]) > 0.25:
                            a = tuple((track.pose[first, :2] * [width, height]).astype(int))
                            b = tuple((track.pose[second, :2] * [width, height]).astype(int))
                            cv2.line(frame, a, b, color, 2)
                draw_text(frame, f"ID {track_id}: {track.label} {track.score:.2f}", max(5, box[0]), max(55, box[1] - 20), color)
                draw_text(frame, f"Down {track.timer.elapsed:.1f}/{args.lying_seconds:g}s", max(5, box[0]), max(75, box[1]), color)
                log.writerow([round(now, 3), track_id, track.label, round(track.score, 3), round(track.timer.elapsed, 3), int(track.timer.alert)])
            if not seen:
                draw_text(frame, "NO PERSON", 10, 65)

            processing_times.append(time.perf_counter() - tick)
            processed += 1
            speed = 1 / max(np.mean(processing_times[-30:]), 1e-6)
            draw_text(frame, f"{args.title} | {speed:.1f} processing FPS | people {len(seen)}/{args.max_people}", 10, 22)
            draw_text(frame, "falling_down = lying beyond timer; possible fall only", 10, 43)
            if args.output:
                if writer is None:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (width, height))
                    if not writer.isOpened():
                        raise RuntimeError("Cannot open output video")
                writer.write(frame)
            if not args.headless:
                cv2.imshow(args.title, frame)
                if cv2.waitKey(1) & 255 == ord("q"):
                    break
            if processed % 40 == 0:
                print("t", round(now, 1), "people", len(seen), "processing_ms", round(np.mean(processing_times[-30:]) * 1000, 1), flush=True)
    finally:
        capture.release()
        log_handle.close()
        if writer:
            writer.release()
        cv2.destroyAllWindows()

    stats = {
        "processed_frames": processed,
        "wall_seconds": time.monotonic() - started,
        "mean_processing_ms": float(np.mean(processing_times) * 1000) if processing_times else None,
        "pose_model": str(args.pose_model),
        "max_people": args.max_people,
        "skeleton": args.skeleton,
        "action_rate_hz": FPS,
        "source": str(source),
        "log": str(log_path),
        "output": str(args.output) if args.output else None,
    }
    if args.stats:
        args.stats.parent.mkdir(parents=True, exist_ok=True)
        args.stats.write_text(json.dumps(stats, indent=2))
    print(json.dumps(stats), flush=True)
    if processed == 0:
        raise RuntimeError("No frames processed")


if __name__ == "__main__":
    main()

