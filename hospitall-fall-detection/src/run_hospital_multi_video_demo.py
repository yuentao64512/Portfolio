#!/usr/bin/env python3
"""Multi-person hospital fall detection with per-track context and state.

Fall evidence deliberately uses shoulders, hips and body orientation. Wrist or
arm motion is never used to create a fall transition.
"""

from __future__ import annotations

import argparse
import csv
import pickle
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from hospital_context import ContextGeometry, load_context_config
from hospital_state import HospitalStateMachine, classify_patient_location, horizontal_pose
from run_video_demo import featurize, resize_sequence


POSE_EDGES = [
    (5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11),
    (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
]
PALETTE = [(70, 220, 70), (255, 170, 40), (220, 80, 220), (60, 210, 240), (240, 120, 80)]


@dataclass
class TrackState:
    track_id: int
    sequence: deque[np.ndarray]
    body_history: deque[tuple[float, float, float]]
    machine: HospitalStateMachine
    last_seen: float = 0.0
    action: str = "collecting_pose"
    action_confidence: float = 0.0
    last_full_body_descent: float | None = None
    state: str = "UNKNOWN"
    alert: bool = False
    floor_down_seconds: float = 0.0


def draw_text(frame, text: str, origin: tuple[int, int], color, scale=0.50) -> None:
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def normalized_keypoints(xy: np.ndarray, confidence: np.ndarray, width: int, height: int) -> np.ndarray:
    output = np.zeros((17, 3), dtype=np.float32)
    count = min(17, len(xy))
    output[:count, 0] = xy[:count, 0] / max(width, 1)
    output[:count, 1] = xy[:count, 1] / max(height, 1)
    output[:count, 2] = confidence[:count]
    return output


def non_duplicate_indices(boxes: np.ndarray, containment_threshold: float) -> list[int]:
    """Keep the largest box when one person box is substantially inside another."""
    if len(boxes) <= 1:
        return list(range(len(boxes)))
    areas = np.maximum(0.0, boxes[:, 2] - boxes[:, 0]) * np.maximum(0.0, boxes[:, 3] - boxes[:, 1])
    kept: list[int] = []
    for index in np.argsort(-areas).tolist():
        candidate = boxes[index]
        duplicate = False
        for previous in kept:
            accepted = boxes[previous]
            intersection = max(0.0, min(candidate[2], accepted[2]) - max(candidate[0], accepted[0])) * max(
                0.0, min(candidate[3], accepted[3]) - max(candidate[1], accepted[1])
            )
            smaller_area = max(1.0, min(float(areas[index]), float(areas[previous])))
            if intersection / smaller_area >= containment_threshold:
                duplicate = True
                break
        if not duplicate:
            kept.append(index)
    return sorted(kept)


def torso_y(xy: np.ndarray, confidence: np.ndarray, height: int, minimum_confidence: float) -> float | None:
    # Shoulders and hips only. Arms/wrists are intentionally excluded.
    values = [
        float(xy[index, 1]) / max(height, 1)
        for index in (5, 6, 11, 12)
        if index < len(xy) and confidence[index] >= minimum_confidence
    ]
    return float(np.median(values)) if len(values) >= 2 else None


def full_body_descent(
    track: TrackState,
    now: float,
    box: np.ndarray,
    xy: np.ndarray,
    confidence: np.ndarray,
    frame_height: int,
    window_seconds: float,
    threshold: float,
    keypoint_confidence: float,
) -> tuple[bool, float]:
    width = max(1.0, float(box[2] - box[0]))
    height = max(1.0, float(box[3] - box[1]))
    aspect = width / height
    centre_y = torso_y(xy, confidence, frame_height, keypoint_confidence)
    if centre_y is None:
        return False, 0.0
    track.body_history.append((now, centre_y, aspect))
    while track.body_history and now - track.body_history[0][0] > window_seconds:
        track.body_history.popleft()
    if len(track.body_history) < 2:
        return False, 0.0
    baseline = min(item[1] for item in track.body_history)
    baseline_aspect = min(item[2] for item in track.body_history)
    delta = centre_y - baseline
    orientation_change = aspect - baseline_aspect
    evidence = delta >= threshold and (aspect >= 1.0 or orientation_change >= 0.30)
    if evidence:
        track.last_full_body_descent = now
    return evidence, delta


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--roi-config", type=Path, required=True)
    parser.add_argument("--classifier", type=Path, default=Path("models/current_pose_rf.pkl"))
    parser.add_argument("--pose-model", default="yolo11n-pose.pt")
    parser.add_argument("--tracker", default="bytetrack.yaml")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--sample-fps", type=float, default=5.0)
    parser.add_argument("--seq-len", type=int, default=32)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--keypoint-conf", type=float, default=0.25)
    parser.add_argument("--lying-alert-sec", type=float, default=1.0)
    parser.add_argument("--transition-window-sec", type=float, default=10.0)
    parser.add_argument("--descent-window-sec", type=float, default=2.5)
    parser.add_argument("--descent-threshold", type=float, default=0.07)
    parser.add_argument("--descent-latch-sec", type=float, default=10.0)
    parser.add_argument("--max-missing-sec", type=float, default=3.0)
    parser.add_argument(
        "--duplicate-containment",
        type=float,
        default=0.55,
        help="Suppress a smaller person box when this fraction is contained in a larger one.",
    )
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    output_path = args.output or args.video.with_name(args.video.stem + "_multi_prediction.mp4")
    csv_path = args.csv or output_path.with_suffix(".csv")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with args.classifier.open("rb") as handle:
        classifier = pickle.load(handle)["pipeline"]
    pose = YOLO(args.pose_model)
    context = ContextGeometry(load_context_config(args.roi_config))

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f"Could not open video: {args.video}")
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 15.0)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise SystemExit(f"Could not create output: {output_path}")

    sample_interval = max(1, int(round(fps / max(args.sample_fps, 0.1))))
    history_length = max(8, int(round(args.descent_window_sec * fps)) + 2)
    tracks: dict[int, TrackState] = {}
    fallback_next_id = 100000
    frame_index = 0
    fields = [
        "time_sec", "frame", "track_id", "action", "action_confidence",
        "full_body_descent", "torso_descent", "effective_action", "location",
        "bed_score", "floor_score", "horizontal", "state", "alert",
        "floor_down_seconds", "x1", "y1", "x2", "y2",
    ]

    def new_track(track_id: int) -> TrackState:
        return TrackState(
            track_id=track_id,
            sequence=deque(maxlen=args.seq_len),
            body_history=deque(maxlen=history_length),
            machine=HospitalStateMachine(args.lying_alert_sec, args.transition_window_sec),
        )

    with csv_path.open("w", newline="", encoding="utf-8") as csv_stream:
        table = csv.DictWriter(csv_stream, fieldnames=fields)
        table.writeheader()
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                now = frame_index / fps
                result = pose.track(
                    frame,
                    persist=True,
                    tracker=args.tracker,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    device=args.device,
                    verbose=False,
                )[0]
                display = context.draw(frame, alpha=0.10)
                rows = []

                if result.boxes is not None and result.keypoints is not None and len(result.boxes):
                    boxes = result.boxes.xyxy.detach().cpu().numpy()
                    keypoint_xy = result.keypoints.xy.detach().cpu().numpy()
                    keypoint_conf = result.keypoints.conf.detach().cpu().numpy()
                    if result.boxes.id is not None:
                        ids = result.boxes.id.detach().cpu().numpy().astype(int).tolist()
                    else:
                        ids = []
                    while len(ids) < len(boxes):
                        ids.append(fallback_next_id)
                        fallback_next_id += 1

                    kept_indices = non_duplicate_indices(boxes, args.duplicate_containment)
                    for detection_index in kept_indices:
                        track_id = ids[detection_index]
                        box = boxes[detection_index]
                        xy = keypoint_xy[detection_index]
                        confidence = keypoint_conf[detection_index]
                        track = tracks.setdefault(int(track_id), new_track(int(track_id)))
                        track.last_seen = now
                        if frame_index % sample_interval == 0:
                            track.sequence.append(normalized_keypoints(xy, confidence, width, height))
                            if len(track.sequence) >= max(4, args.seq_len // 4):
                                prepared = resize_sequence(list(track.sequence), args.seq_len)[None, ...]
                                probabilities = classifier.predict_proba(featurize(prepared))[0]
                                best = int(np.argmax(probabilities))
                                track.action = str(classifier.classes_[best])
                                track.action_confidence = float(probabilities[best])

                        descent_now, descent_delta = full_body_descent(
                            track, now, box, xy, confidence, height,
                            args.descent_window_sec, args.descent_threshold, args.keypoint_conf,
                        )
                        descent_recent = (
                            track.last_full_body_descent is not None
                            and now - track.last_full_body_descent <= args.descent_latch_sec
                        )
                        effective_action = "falling_down" if descent_recent else track.action

                        overlaps = context.bbox_overlaps(box, width, height)
                        fractions = context.keypoint_region_fractions(xy, confidence, width, height)
                        location = classify_patient_location(overlaps, fractions)
                        is_horizontal = horizontal_pose(box)
                        decision = track.machine.update(now, location.location, effective_action, is_horizontal)
                        track.state = decision.state
                        track.alert = decision.alert
                        track.floor_down_seconds = decision.floor_down_seconds

                        x1, y1, x2, y2 = np.round(box).astype(int)
                        base_color = PALETTE[int(track_id) % len(PALETTE)]
                        color = (0, 0, 255) if track.alert else base_color
                        cv2.rectangle(display, (x1, y1), (x2, y2), color, 3)
                        for first, second in POSE_EDGES:
                            if confidence[first] >= args.keypoint_conf and confidence[second] >= args.keypoint_conf:
                                cv2.line(
                                    display,
                                    tuple(np.round(xy[first]).astype(int)),
                                    tuple(np.round(xy[second]).astype(int)),
                                    color,
                                    2,
                                    cv2.LINE_AA,
                                )
                        label_y = max(70, y1 - 44)
                        draw_text(display, f"ID {track_id} | {location.location} | {track.state}", (max(4, x1), label_y), color)
                        draw_text(
                            display,
                            f"action={track.action} body_drop={descent_delta:.2f} {'FALL' if descent_recent else ''}",
                            (max(4, x1), label_y + 23),
                            color,
                            0.46,
                        )
                        rows.append(
                            {
                                "time_sec": f"{now:.6f}", "frame": frame_index, "track_id": track_id,
                                "action": track.action, "action_confidence": f"{track.action_confidence:.6f}",
                                "full_body_descent": descent_now, "torso_descent": f"{descent_delta:.6f}",
                                "effective_action": effective_action, "location": location.location,
                                "bed_score": f"{location.bed_score:.6f}", "floor_score": f"{location.floor_score:.6f}",
                                "horizontal": is_horizontal, "state": track.state, "alert": track.alert,
                                "floor_down_seconds": f"{track.floor_down_seconds:.6f}",
                                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                            }
                        )

                stale_ids = [
                    track_id for track_id, track in tracks.items()
                    if now - track.last_seen > args.max_missing_sec
                ]
                for track_id in stale_ids:
                    del tracks[track_id]

                expired = [track_id for track_id, track in tracks.items() if now - track.last_seen > args.max_missing_sec]
                for track_id in expired:
                    del tracks[track_id]

                alert_ids = sorted(track.track_id for track in tracks.values() if track.alert)
                banner_color = (0, 0, 210) if alert_ids else (35, 35, 35)
                cv2.rectangle(display, (0, 0), (width, 52), banner_color, -1)
                banner = f"Multi-person hospital fall monitor | alerts: {alert_ids or 'none'}"
                draw_text(display, banner, (16, 34), (255, 255, 255), 0.68)
                writer.write(display)
                table.writerows(rows)
                if frame_index % max(1, int(round(fps))) == 0:
                    print(f"{now:6.2f}s detections={len(rows)} active_tracks={len(tracks)} alerts={alert_ids}")
                if not args.headless:
                    cv2.imshow("Multi-person Hospital Fall Monitor", display)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        break
                frame_index += 1
        finally:
            capture.release()
            writer.release()
            cv2.destroyAllWindows()

    print(f"frames={frame_index} output={output_path} csv={csv_path}")


if __name__ == "__main__":
    main()
