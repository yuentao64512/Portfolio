#!/usr/bin/env python3
"""Live pose + hospital context preview for ROI validation."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from hospital_context import (
    ContextGeometry,
    FurnitureDetector,
    load_context_config,
    rectangle_overlap_fraction,
)


POSE_EDGES = [
    (5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11),
    (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
]


def parse_source(value: str) -> int | str:
    return int(value) if value.isdigit() else value


def draw_text(frame, text, origin, color):
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0")
    parser.add_argument("--pose-model", default="yolo11n-pose.pt")
    parser.add_argument("--roi-config", type=Path)
    parser.add_argument("--furniture-model", help="Optional COCO detector, e.g. /home/robotx/yolo11n.pt")
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()

    capture = cv2.VideoCapture(parse_source(args.source))
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not capture.isOpened():
        raise SystemExit(f"Could not open source: {args.source}")

    pose = YOLO(args.pose_model)
    context = ContextGeometry(load_context_config(args.roi_config)) if args.roi_config else None
    furniture = FurnitureDetector(args.furniture_model, args.device, args.conf) if args.furniture_model else None
    furniture_detections: list[dict] = []
    frame_index = 0
    start = time.monotonic()

    try:
        while args.max_frames <= 0 or frame_index < args.max_frames:
            ok, frame = capture.read()
            if not ok:
                break
            result = pose.track(
                frame, persist=True, tracker="bytetrack.yaml", imgsz=args.imgsz,
                conf=args.conf, device=args.device, verbose=False,
            )[0]
            if furniture and frame_index % 5 == 0:
                furniture_detections = furniture.detect(frame, args.imgsz)
            display = context.draw(frame) if context else frame.copy()
            height, width = frame.shape[:2]

            for item in furniture_detections:
                x1, y1, x2, y2 = np.round(item["bbox"]).astype(int)
                cv2.rectangle(display, (x1, y1), (x2, y2), (255, 100, 220), 2)
                draw_text(
                    display,
                    f"AUTO {item['region_type']} {item['confidence']:.2f}",
                    (x1, max(20, y1 - 7)), (255, 100, 220),
                )

            if result.boxes is not None and result.keypoints is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                ids = (
                    result.boxes.id.cpu().numpy().astype(int)
                    if result.boxes.id is not None else np.full(len(boxes), -1)
                )
                keypoints = result.keypoints.xy.cpu().numpy()
                keypoint_conf = result.keypoints.conf.cpu().numpy()
                for box, track_id, xy, confidence in zip(boxes, ids, keypoints, keypoint_conf):
                    for first, second in POSE_EDGES:
                        if confidence[first] > 0.25 and confidence[second] > 0.25:
                            cv2.line(
                                display, tuple(np.round(xy[first]).astype(int)),
                                tuple(np.round(xy[second]).astype(int)),
                                (0, 230, 255), 2, cv2.LINE_AA,
                            )
                    x1, y1, x2, y2 = np.round(box).astype(int)
                    cv2.rectangle(display, (x1, y1), (x2, y2), (0, 230, 255), 2)
                    overlaps = context.bbox_overlaps(box, width, height) if context else {}
                    auto_overlaps = {"bed": 0.0, "couch": 0.0, "chair": 0.0}
                    for item in furniture_detections:
                        label = item["region_type"]
                        auto_overlaps[label] = max(
                            auto_overlaps.get(label, 0.0),
                            rectangle_overlap_fraction(box, item["bbox"]),
                        )
                    bed_overlap = max(
                        overlaps.get("bed", 0.0), overlaps.get("stretcher", 0.0),
                        overlaps.get("couch", 0.0), auto_overlaps.get("bed", 0.0),
                        auto_overlaps.get("couch", 0.0),
                    )
                    floor_overlap = overlaps.get("floor", 0.0)
                    label = (
                        f"ID {track_id if track_id >= 0 else '-'} "
                        f"bed/furniture={bed_overlap:.2f} floor={floor_overlap:.2f}"
                    )
                    draw_text(display, label, (x1, max(20, y1 - 7)), (0, 230, 255))

            frame_index += 1
            fps = frame_index / max(time.monotonic() - start, 1e-6)
            draw_text(display, f"pose+context preview | fps={fps:.1f}", (16, height - 18), (255, 255, 255))
            if args.headless:
                if frame_index % 30 == 0:
                    print(f"frame={frame_index} fps={fps:.1f}")
            else:
                cv2.imshow("Hospital Pose + Context Preview", display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        capture.release()
        cv2.destroyAllWindows()
    print(f"frames={frame_index} average_fps={frame_index / max(time.monotonic() - start, 1e-6):.2f}")


if __name__ == "__main__":
    main()
