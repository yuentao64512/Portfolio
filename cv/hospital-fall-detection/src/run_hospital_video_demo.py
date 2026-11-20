#!/usr/bin/env python3
"""Run pose, fixed-camera bed/floor context and fall state on one video."""

from __future__ import annotations

import argparse
import csv
import pickle
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from hospital_context import ContextGeometry, load_context_config
from hospital_state import HospitalStateMachine, classify_patient_location, horizontal_pose
from run_video_demo import featurize, largest_person_keypoints, resize_sequence


POSE_EDGES = [
    (5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11),
    (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
]


def draw_text(frame, text: str, origin: tuple[int, int], color: tuple[int, int, int], scale=0.58):
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--roi-config", type=Path, required=True)
    parser.add_argument("--classifier", type=Path, default=Path("models/current_pose_rf.pkl"))
    parser.add_argument("--pose-model", default="yolo11n-pose.pt")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--sample-fps", type=float, default=5.0)
    parser.add_argument("--seq-len", type=int, default=32)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--device", default="0")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--lying-alert-sec", type=float, default=1.0)
    parser.add_argument("--transition-window-sec", type=float, default=10.0)
    parser.add_argument(
        "--reset-after-no-person-sec",
        type=float,
        default=0.0,
        help="Reset pose history and hospital state after this many seconds with no person; 0 disables it.",
    )
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    output_path = args.output or args.video.with_name(args.video.stem + "_hospital_context.mp4")
    csv_path = args.csv or output_path.with_suffix(".csv")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.parent.mkdir(parents=True, exist_ok=True)

    with args.classifier.open("rb") as handle:
        bundle = pickle.load(handle)
    classifier = bundle["pipeline"]
    pose = YOLO(args.pose_model)
    context = ContextGeometry(load_context_config(args.roi_config))
    machine = HospitalStateMachine(args.lying_alert_sec, args.transition_window_sec)

    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f"Could not open video: {args.video}")
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 15.0)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        raise SystemExit(f"Could not create output: {output_path}")

    sample_interval = max(1, int(round(fps / max(args.sample_fps, 0.1))))
    sequence: deque[np.ndarray] = deque(maxlen=args.seq_len)
    current_action = "collecting_pose"
    current_action_confidence = 0.0
    current_location = "UNKNOWN"
    current_state = "UNKNOWN"
    current_bed_score = 0.0
    current_floor_score = 0.0
    current_alert = False
    current_down_seconds = 0.0
    frame_index = 0
    last_reported_second = -1
    no_person_since: float | None = None
    fields = [
        "time_sec", "frame", "action", "action_confidence", "location",
        "bed_score", "floor_score", "state", "alert", "floor_down_seconds",
    ]

    with csv_path.open("w", newline="", encoding="utf-8") as csv_stream:
        table = csv.DictWriter(csv_stream, fieldnames=fields)
        table.writeheader()
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                timestamp = frame_index / fps
                result = pose.predict(
                    frame, imgsz=args.imgsz, conf=args.conf,
                    device=args.device, verbose=False,
                )[0]
                display = context.draw(frame, alpha=0.12)

                if result.boxes is not None and result.keypoints is not None and len(result.boxes):
                    if (
                        no_person_since is not None
                        and args.reset_after_no_person_sec > 0
                        and timestamp - no_person_since >= args.reset_after_no_person_sec
                    ):
                        sequence.clear()
                        machine = HospitalStateMachine(args.lying_alert_sec, args.transition_window_sec)
                        current_action = "collecting_pose"
                        current_action_confidence = 0.0
                        print(
                            f"{timestamp:6.2f}s reset pose/state after "
                            f"{timestamp - no_person_since:.2f}s without a person"
                        )
                    no_person_since = None
                    boxes = result.boxes.xyxy.detach().cpu().numpy()
                    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
                    person_index = int(np.argmax(areas))
                    box = boxes[person_index]
                    xy = result.keypoints.xy[person_index].detach().cpu().numpy()
                    confidence = result.keypoints.conf[person_index].detach().cpu().numpy()

                    if frame_index % sample_interval == 0:
                        sequence.append(largest_person_keypoints(result, width, height))
                        if len(sequence) >= max(4, args.seq_len // 4):
                            prepared = resize_sequence(list(sequence), args.seq_len)[None, ...]
                            probabilities = classifier.predict_proba(featurize(prepared))[0]
                            best = int(np.argmax(probabilities))
                            current_action = str(classifier.classes_[best])
                            current_action_confidence = float(probabilities[best])

                    bbox_overlaps = context.bbox_overlaps(box, width, height)
                    keypoint_fractions = context.keypoint_region_fractions(
                        xy, confidence, width, height
                    )
                    location = classify_patient_location(bbox_overlaps, keypoint_fractions)
                    current_location = location.location
                    current_bed_score = location.bed_score
                    current_floor_score = location.floor_score
                    decision = machine.update(
                        timestamp, current_location, current_action,
                        horizontal_pose(box),
                    )
                    current_state = decision.state
                    current_alert = decision.alert
                    current_down_seconds = decision.floor_down_seconds

                    x1, y1, x2, y2 = np.round(box).astype(int)
                    color = (0, 0, 255) if current_alert else (
                        (0, 180, 255) if current_location == "ON_FLOOR" else (0, 220, 80)
                    )
                    cv2.rectangle(display, (x1, y1), (x2, y2), color, 3)
                    for first, second in POSE_EDGES:
                        if confidence[first] >= args.conf and confidence[second] >= args.conf:
                            cv2.line(
                                display, tuple(np.round(xy[first]).astype(int)),
                                tuple(np.round(xy[second]).astype(int)), color, 2, cv2.LINE_AA,
                            )
                    draw_text(display, f"{current_location} | {current_state}", (x1, max(25, y1 - 48)), color)
                    draw_text(
                        display,
                        f"action={current_action} {current_action_confidence:.2f} "
                        f"bed={current_bed_score:.2f} floor={current_floor_score:.2f}",
                        (x1, max(48, y1 - 22)), color, 0.50,
                    )
                else:
                    if no_person_since is None:
                        no_person_since = timestamp
                    current_location = "NO_PERSON"
                    current_state = "NO_PERSON"
                    current_alert = False

                banner_color = (0, 0, 210) if current_alert else (40, 40, 40)
                cv2.rectangle(display, (0, 0), (width, 48), banner_color, -1)
                draw_text(
                    display,
                    f"Hospital context: {current_state} | floor-down {current_down_seconds:.1f}s",
                    (16, 32), (255, 255, 255), 0.70,
                )
                writer.write(display)
                table.writerow(
                    {
                        "time_sec": timestamp, "frame": frame_index,
                        "action": current_action, "action_confidence": current_action_confidence,
                        "location": current_location, "bed_score": current_bed_score,
                        "floor_score": current_floor_score, "state": current_state,
                        "alert": current_alert, "floor_down_seconds": current_down_seconds,
                    }
                )
                if int(timestamp) != last_reported_second:
                    last_reported_second = int(timestamp)
                    print(
                        f"{timestamp:6.2f}s action={current_action:18s} "
                        f"location={current_location:10s} bed={current_bed_score:.2f} "
                        f"floor={current_floor_score:.2f} state={current_state} alert={current_alert}"
                    )
                if not args.headless:
                    cv2.imshow("Hospital Bed/Floor Context", display)
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
