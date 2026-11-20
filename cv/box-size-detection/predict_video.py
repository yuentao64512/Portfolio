"""Run the trained box detector and 12-frame label smoothing on any video.

Example:
    python predict_video.py input.mp4 --weights best.pt --output annotated.mp4

Requires: pip install ultralytics opencv-python
The saved CSV records every tracked detection, including uncertain labels.
"""

from __future__ import annotations

import argparse
import csv
from collections import Counter, defaultdict, deque
from pathlib import Path

import cv2
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=Path("annotated.mp4"))
    parser.add_argument("--csv", type=Path, default=Path("predictions.csv"))
    parser.add_argument("--device", default=None, help="For example, 0 for GPU or cpu")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="Increase for a wide multi-camera montage")
    args = parser.parse_args()
    model = YOLO(str(args.weights))
    capture = cv2.VideoCapture(str(args.video))
    if not capture.isOpened():
        raise SystemExit(f"Cannot open {args.video}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"),
                             fps, (width, height))
    if not writer.isOpened():
        raise SystemExit(f"Cannot write {args.output}")

    history = defaultdict(lambda: deque(maxlen=12))
    fieldnames = ["frame", "time_s", "track_id", "x1", "y1", "x2", "y2",
                  "raw_class", "raw_confidence", "smoothed_class",
                  "mean_confidence", "confirmed"]
    frame_id = 0
    with args.csv.open("w", newline="", encoding="utf-8") as stream:
        table = csv.DictWriter(stream, fieldnames=fieldnames)
        table.writeheader()
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            result = model.track(frame, persist=True, tracker="bytetrack.yaml",
                                 conf=0.5, imgsz=args.imgsz, device=args.device,
                                 verbose=False)[0]
            if result.boxes is not None and len(result.boxes):
                boxes = result.boxes.xyxy.cpu().numpy().astype(int)
                classes = result.boxes.cls.cpu().numpy().astype(int)
                scores = result.boxes.conf.cpu().numpy()
                ids = (result.boxes.id.cpu().numpy().astype(int)
                       if result.boxes.id is not None else [None] * len(boxes))
                for box, class_id, score, track_id in zip(boxes, classes, scores, ids):
                    x1, y1, x2, y2 = map(int, box)
                    if track_id is None:
                        smoothed, mean_score, confirmed = "uncertain", 0.0, False
                    else:
                        window = history[int(track_id)]
                        window.append((int(class_id), float(score)))
                        winner, votes = Counter(cls for cls, _ in window).most_common(1)[0]
                        mean_score = sum(conf for cls, conf in window
                                         if cls == winner) / votes
                        confirmed = votes >= 8 and mean_score >= 0.95
                        smoothed = model.names[winner] if confirmed else "uncertain"
                    table.writerow({"frame": frame_id, "time_s": frame_id / fps,
                                    "track_id": track_id, "x1": x1, "y1": y1,
                                    "x2": x2, "y2": y2,
                                    "raw_class": model.names[int(class_id)],
                                    "raw_confidence": float(score),
                                    "smoothed_class": smoothed,
                                    "mean_confidence": mean_score,
                                    "confirmed": confirmed})
                    color = (0, 180, 0) if confirmed else (0, 180, 255)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                    label = f"ID {track_id} {smoothed} ({score:.2f})"
                    cv2.putText(frame, label, (x1, max(20, y1 - 8)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            writer.write(frame)
            frame_id += 1
    capture.release()
    writer.release()
    print(f"frames={frame_id} video={args.output} csv={args.csv}")


if __name__ == "__main__":
    main()
