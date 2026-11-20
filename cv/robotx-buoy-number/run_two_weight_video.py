#!/usr/bin/env python3
"""Run the reviewed buoy detector and marker classifier on a video."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def restricted_prediction(result, allowed_names: set[str]):
    scores = result.probs.data.detach().cpu().numpy()
    names = result.names
    allowed_ids = [index for index, name in names.items() if str(name) in allowed_names]
    if not allowed_ids:
        return "?", 0.0
    best_id = max(allowed_ids, key=lambda index: float(scores[index]))
    return str(names[best_id]), float(scores[best_id])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--detector", type=Path, required=True)
    parser.add_argument("--classifier", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--det-conf", type=float, default=0.15)
    parser.add_argument("--marker-conf", type=float, default=0.25)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    detector = YOLO(str(args.detector))
    classifier = YOLO(str(args.classifier))
    capture = cv2.VideoCapture(str(args.source))
    if not capture.isOpened():
        raise SystemExit(f"Cannot open source: {args.source}")

    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        raise SystemExit(f"Cannot create output: {args.output}")

    colours = {"number_buoy": (0, 210, 255), "boundary": (255, 120, 20)}
    fields = [
        "frame", "time_s", "detector_class", "detector_conf", "marker",
        "marker_conf", "x1", "y1", "x2", "y2",
    ]
    detections = 0
    start = time.time()
    with args.csv.open("w", newline="", encoding="utf-8") as csv_handle:
        csv_writer = csv.DictWriter(csv_handle, fieldnames=fields)
        csv_writer.writeheader()
        frame_index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            prediction = detector.predict(
                frame, imgsz=1280, conf=args.det_conf, iou=0.45,
                device=args.device, verbose=False,
            )[0]
            for box in prediction.boxes:
                class_id = int(box.cls.item())
                detector_name = str(prediction.names[class_id])
                detector_conf = float(box.conf.item())
                x1, y1, x2, y2 = [int(round(value)) for value in box.xyxy[0].tolist()]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(width, x2), min(height, y2)
                bw, bh = x2 - x1, y2 - y1
                marker, marker_conf = "?", 0.0
                if bw >= 16 and bh >= 16:
                    pad_x, pad_y = max(2, int(bw * 0.06)), max(2, int(bh * 0.06))
                    crop = frame[
                        max(0, y1 - pad_y):min(height, y2 + pad_y),
                        max(0, x1 - pad_x):min(width, x2 + pad_x),
                    ]
                    if crop.size:
                        result = classifier.predict(
                            crop, imgsz=320, device=args.device, verbose=False
                        )[0]
                        allowed = set("0123456789") if detector_name == "number_buoy" else {"C", "D"}
                        marker, marker_conf = restricted_prediction(result, allowed)
                        if marker_conf < args.marker_conf:
                            marker = "?"

                colour = colours.get(detector_name, (0, 255, 0))
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
                text = f"{detector_name} {detector_conf:.2f} | {marker} {marker_conf:.2f}"
                text_y = max(22, y1 - 7)
                cv2.putText(frame, text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, colour, 2, cv2.LINE_AA)
                csv_writer.writerow({
                    "frame": frame_index,
                    "time_s": f"{frame_index / fps:.3f}",
                    "detector_class": detector_name,
                    "detector_conf": f"{detector_conf:.6f}",
                    "marker": marker,
                    "marker_conf": f"{marker_conf:.6f}",
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                })
                detections += 1
            writer.write(frame)
            frame_index += 1
            if frame_index % 100 == 0 or frame_index == total:
                elapsed = max(0.001, time.time() - start)
                print(f"frames={frame_index}/{total} speed={frame_index / elapsed:.1f}fps detections={detections}",
                      flush=True)

    capture.release()
    writer.release()
    elapsed = time.time() - start
    print(f"COMPLETE frames={frame_index} detections={detections} elapsed={elapsed:.1f}s")
    print(f"video={args.output}")
    print(f"csv={args.csv}")


if __name__ == "__main__":
    main()
