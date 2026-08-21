#!/usr/bin/env python3
"""Live camera fall/action detection using YOLO pose plus the trained classifier."""

from __future__ import annotations

import argparse
import pickle
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


COCO_EDGES = [
    (5, 7),
    (7, 9),
    (6, 8),
    (8, 10),
    (5, 6),
    (5, 11),
    (6, 12),
    (11, 12),
    (11, 13),
    (13, 15),
    (12, 14),
    (14, 16),
]


@dataclass
class PersonTrack:
    track_id: int
    seq_len: int
    buffer: deque[np.ndarray] = field(init=False)
    last_seen: float = 0.0
    center: tuple[float, float] = (0.0, 0.0)
    box: np.ndarray | None = None
    xy: np.ndarray | None = None
    conf: np.ndarray | None = None
    pred: str = "warming_up"
    top: str = ""
    alert: bool = False
    fall_seen_at: float | None = None
    lying_started_at: float | None = None

    def __post_init__(self) -> None:
        self.buffer = deque(maxlen=self.seq_len)


def detections_from_result(result, width: int, height: int, min_visible_keypoints: int) -> list[dict]:
    if result.keypoints is None or result.boxes is None or len(result.boxes) == 0:
        return []

    boxes = result.boxes.xyxy.detach().cpu().numpy()
    keypoint_xy = result.keypoints.xy.detach().cpu().numpy()
    keypoint_conf = result.keypoints.conf.detach().cpu().numpy()
    detections = []

    for idx in range(min(len(boxes), len(keypoint_xy))):
        xy = keypoint_xy[idx]
        conf = keypoint_conf[idx]
        visible = int((conf > 0.25).sum())
        if visible < min_visible_keypoints:
            continue

        normalized = np.zeros((17, 3), dtype=np.float32)
        n = min(normalized.shape[0], xy.shape[0])
        normalized[:n, 0] = xy[:n, 0] / max(width, 1)
        normalized[:n, 1] = xy[:n, 1] / max(height, 1)
        normalized[:n, 2] = conf[:n]

        box = boxes[idx].astype(np.float32)
        center = (
            float((box[0] + box[2]) / (2.0 * max(width, 1))),
            float((box[1] + box[3]) / (2.0 * max(height, 1))),
        )
        area = float(max(box[2] - box[0], 0.0) * max(box[3] - box[1], 0.0))
        detections.append(
            {
                "keypoints": normalized,
                "xy": xy,
                "conf": conf,
                "box": box,
                "center": center,
                "area": area,
            }
        )

    detections.sort(key=lambda item: item["area"], reverse=True)
    return detections


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


def draw_pose(frame, xy: np.ndarray | None, conf: np.ndarray | None, color=(0, 220, 255)) -> None:
    if xy is None or conf is None:
        return
    for a, b in COCO_EDGES:
        if a < len(conf) and b < len(conf) and conf[a] > 0.25 and conf[b] > 0.25:
            pa = tuple(np.round(xy[a]).astype(int))
            pb = tuple(np.round(xy[b]).astype(int))
            cv2.line(frame, pa, pb, color, 2, cv2.LINE_AA)
    for point, score in zip(xy, conf):
        if score > 0.25:
            cv2.circle(frame, tuple(np.round(point).astype(int)), 3, (0, 255, 80), -1, cv2.LINE_AA)


def put_text(frame, text: str, y: int, color=(255, 255, 255)) -> None:
    cv2.putText(frame, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, (16, y), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)


def label_at(frame, text: str, x: int, y: int, color=(255, 255, 255)) -> None:
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)


def track_priority(track: PersonTrack, now: float) -> tuple[int, float, int]:
    if track.alert:
        return (0, track.last_seen, track.track_id)
    if track.pred == "falling_down":
        return (1, track.last_seen, track.track_id)
    if track.fall_seen_at is not None and track.pred == "lying_down":
        return (2, track.last_seen, track.track_id)
    if track.pred == "lying_down":
        return (3, track.last_seen, track.track_id)
    if track.pred == "uncertain":
        return (4, track.last_seen, track.track_id)
    if now - track.last_seen <= 0.5:
        return (5, track.last_seen, track.track_id)
    return (6, track.last_seen, track.track_id)


def match_and_update_tracks(
    tracks: dict[int, PersonTrack],
    detections: list[dict],
    next_track_id: int,
    now: float,
    seq_len: int,
    max_people: int,
    match_threshold: float,
    track_timeout_sec: float,
) -> tuple[int, list[int]]:
    detections = detections[:max_people]
    active_track_ids = [tid for tid, track in tracks.items() if now - track.last_seen <= track_timeout_sec]
    pairs: list[tuple[float, int, int]] = []
    for det_idx, det in enumerate(detections):
        dcx, dcy = det["center"]
        for tid in active_track_ids:
            tcx, tcy = tracks[tid].center
            distance = ((dcx - tcx) ** 2 + (dcy - tcy) ** 2) ** 0.5
            pairs.append((distance, tid, det_idx))

    matched_tracks: set[int] = set()
    matched_detections: set[int] = set()
    updated_ids: list[int] = []
    for distance, tid, det_idx in sorted(pairs, key=lambda item: item[0]):
        if distance > match_threshold or tid in matched_tracks or det_idx in matched_detections:
            continue
        track = tracks[tid]
        det = detections[det_idx]
        update_track(track, det, now)
        matched_tracks.add(tid)
        matched_detections.add(det_idx)
        updated_ids.append(tid)

    for det_idx, det in enumerate(detections):
        if det_idx in matched_detections:
            continue
        track = PersonTrack(track_id=next_track_id, seq_len=seq_len)
        next_track_id += 1
        update_track(track, det, now)
        tracks[track.track_id] = track
        updated_ids.append(track.track_id)

    return next_track_id, updated_ids


def update_track(track: PersonTrack, detection: dict, now: float) -> None:
    track.last_seen = now
    track.center = detection["center"]
    track.box = detection["box"]
    track.xy = detection["xy"]
    track.conf = detection["conf"]
    track.buffer.append(detection["keypoints"])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="0", help="Camera index such as 0, or a video path/stream URL.")
    parser.add_argument("--backend", default="v4l2", choices=["auto", "v4l2"], help="OpenCV camera backend.")
    parser.add_argument("--classifier", type=Path, default=Path("models/current_pose_rf.pkl"))
    parser.add_argument("--pose-model", default="yolo11n-pose.pt")
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--sample-fps", type=float, default=5.0)
    parser.add_argument("--seq-len", type=int, default=32)
    parser.add_argument("--lying-alert-sec", type=float, default=10.0)
    parser.add_argument("--fall-cooldown-sec", type=float, default=20.0)
    parser.add_argument("--min-detection-rate", type=float, default=0.35)
    parser.add_argument("--min-action-confidence", type=float, default=0.45)
    parser.add_argument("--max-people", type=int, default=8)
    parser.add_argument("--match-threshold", type=float, default=0.18)
    parser.add_argument("--track-timeout-sec", type=float, default=2.0)
    parser.add_argument("--min-visible-keypoints", type=int, default=3)
    parser.add_argument("--headless", action="store_true", help="Print results without opening a preview window.")
    args = parser.parse_args()

    camera_source = int(args.camera) if str(args.camera).isdigit() else args.camera
    backend = cv2.CAP_V4L2 if args.backend == "v4l2" else cv2.CAP_ANY
    cap = cv2.VideoCapture(camera_source, backend)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera/video source: {args.camera}")
    if isinstance(camera_source, int):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_FPS, 30)

    with args.classifier.open("rb") as handle:
        bundle = pickle.load(handle)
    clf = bundle["pipeline"]
    pose = YOLO(args.pose_model)

    tracks: dict[int, PersonTrack] = {}
    next_track_id = 1
    last_sample = 0.0
    last_print = 0.0
    frame_count = 0
    fps_t0 = time.monotonic()

    print("Live detector started. Press q in the preview window to stop.")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed; stopping.")
            break
        frame_count += 1
        now = time.monotonic()
        do_sample = (now - last_sample) >= (1.0 / args.sample_fps)

        if do_sample:
            last_sample = now
            result = pose.predict(frame, imgsz=args.imgsz, device=args.device, verbose=False)[0]
            h, w = frame.shape[:2]
            detections = detections_from_result(result, w, h, args.min_visible_keypoints)
            next_track_id, updated_ids = match_and_update_tracks(
                tracks,
                detections,
                next_track_id,
                now,
                args.seq_len,
                args.max_people,
                args.match_threshold,
                args.track_timeout_sec,
            )

            for track_id in updated_ids:
                track = tracks[track_id]
                if len(track.buffer) >= max(4, args.seq_len // 4):
                    seq = resize_sequence(list(track.buffer), args.seq_len)[None, ...]
                    probs = clf.predict_proba(featurize(seq))[0]
                    classes = clf.classes_
                    best_idx = int(np.argmax(probs))
                    raw_pred = str(classes[best_idx])
                    best_prob = float(probs[best_idx])
                    track.pred = raw_pred if best_prob >= args.min_action_confidence else "uncertain"
                    track.top = ", ".join(
                        f"{name}:{prob:.2f}"
                        for name, prob in sorted(zip(classes, probs), key=lambda item: item[1], reverse=True)[:3]
                    )
                else:
                    track.pred = "warming_up"
                    track.top = f"samples:{len(track.buffer)}/{max(4, args.seq_len // 4)}"

                if track.pred == "falling_down":
                    track.fall_seen_at = now
                    track.lying_started_at = None
                elif track.fall_seen_at is not None and track.pred == "lying_down":
                    if track.lying_started_at is None:
                        track.lying_started_at = now
                elif track.pred not in {"lying_down", "falling_down", "uncertain"}:
                    track.lying_started_at = None

                if track.fall_seen_at is not None and (now - track.fall_seen_at) > args.fall_cooldown_sec:
                    track.fall_seen_at = None
                    track.lying_started_at = None

                track.alert = (
                    track.fall_seen_at is not None
                    and track.lying_started_at is not None
                    and (now - track.lying_started_at) >= args.lying_alert_sec
                )

            for track_id in list(tracks):
                if now - tracks[track_id].last_seen > args.track_timeout_sec:
                    del tracks[track_id]

        active_tracks = [track for track in tracks.values() if now - track.last_seen <= args.track_timeout_sec]
        active_tracks.sort(key=lambda item: track_priority(item, now))
        alert_tracks = [track for track in active_tracks if track.alert]
        fall_watch_tracks = [
            track
            for track in active_tracks
            if not track.alert and (track.pred == "falling_down" or track.fall_seen_at is not None)
        ]

        if not args.headless:
            for track in active_tracks:
                if track.alert:
                    color = (0, 0, 255)
                elif track.pred == "falling_down" or track.fall_seen_at is not None:
                    color = (0, 165, 255)
                else:
                    color = (0, 220, 255)
                draw_pose(frame, track.xy, track.conf, color=color)
                if track.box is not None:
                    x1, y1, x2, y2 = np.round(track.box).astype(int)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label_at(frame, f"ID {track.track_id}: {track.pred}", max(0, x1), max(18, y1 - 8), color)

        elapsed = max(time.monotonic() - fps_t0, 1e-6)
        fps = frame_count / elapsed
        if alert_tracks:
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 78), (0, 0, 180), -1)
            alert_ids = ",".join(str(track.track_id) for track in alert_tracks)
            put_text(frame, f"ALERT: possible fall + person down  IDs={alert_ids}", 34, (255, 255, 255))
            put_text(frame, f"people={len(active_tracks)}  fps={fps:.1f}", 66, (255, 255, 255))
        elif fall_watch_tracks:
            cv2.rectangle(frame, (0, 0), (frame.shape[1], 78), (0, 105, 180), -1)
            watch_ids = ",".join(str(track.track_id) for track in fall_watch_tracks)
            put_text(frame, f"WATCH: possible fall  IDs={watch_ids}", 34, (255, 255, 255))
            put_text(frame, f"people={len(active_tracks)}  fps={fps:.1f}", 66, (255, 255, 255))
        else:
            put_text(frame, f"people={len(active_tracks)}  fps={fps:.1f}", 32, (255, 255, 255))
            summary = " | ".join(f"ID {track.track_id}:{track.pred}" for track in active_tracks[:4])
            put_text(frame, summary or "no_person", 64, (220, 220, 220))

        if now - last_print >= 1.0:
            last_print = now
            if active_tracks:
                status = "; ".join(
                    f"ID {track.track_id} state={track.pred} alert={track.alert} top={track.top}"
                    for track in active_tracks[: args.max_people]
                )
            else:
                status = "no_person"
            print(f"people={len(active_tracks)} fps={fps:.1f} {status}")

        if not args.headless:
            cv2.imshow("Fall Action Live Detector", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
