"""Detect dirty paper with a custom YOLO model at confidence >= 0.90."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path, help="image or video to inspect")
    parser.add_argument("--model", type=Path, default=Path(__file__).with_name("best.pt"))
    parser.add_argument("--confidence", type=float, default=0.90)
    parser.add_argument("--stride", type=int, default=1, help="analyze every Nth video frame")
    parser.add_argument("--output", type=Path, help="optional annotated MP4 output")
    args = parser.parse_args()

    if not 0.90 <= args.confidence <= 1.0:
        raise SystemExit("--confidence must be between 0.90 and 1.0")
    if args.stride < 1:
        raise SystemExit("--stride must be at least 1")

    model = YOLO(str(args.model))
    cap = cv2.VideoCapture(str(args.source))
    if not cap.isOpened():
        raise SystemExit(f"Could not open {args.source}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(args.output),
            cv2.VideoWriter_fourcc(*"mp4v"),
            fps / args.stride,
            (width, height),
        )

    frame_number = analyzed = detected_frames = detections = 0
    confidences: list[float] = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_number % args.stride:
            frame_number += 1
            continue
        result = model.predict(frame, conf=args.confidence, verbose=False)[0]
        frame_confidences = [float(value) for value in result.boxes.conf.cpu().tolist()]
        if frame_confidences:
            detected_frames += 1
            detections += len(frame_confidences)
            confidences.extend(frame_confidences)
        if writer:
            writer.write(result.plot())
        analyzed += 1
        frame_number += 1

    cap.release()
    if writer:
        writer.release()

    summary = {
        "source": str(args.source),
        "threshold": args.confidence,
        "frames_analyzed": analyzed,
        "frames_with_dirt": detected_frames,
        "dirt_frame_rate": round(detected_frames / analyzed, 4) if analyzed else 0.0,
        "detections": detections,
        "mean_confidence": round(sum(confidences) / len(confidences), 4) if confidences else None,
        "maximum_confidence": round(max(confidences), 4) if confidences else None,
        "decision": "DIRT DETECTED" if detections else "NO DIRT DETECTED",
    }
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
