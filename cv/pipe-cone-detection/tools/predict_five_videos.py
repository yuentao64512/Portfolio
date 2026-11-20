#!/usr/bin/env python3
"""Run the trained cone detector over the five supplied MOV files."""

import csv
import statistics
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = ROOT / "runs/yolo26l_combined_800_finetune_restart_100e/weights/best.pt"
SOURCES = [Path.home() / f"IMG_{n}.MOV" for n in (7816, 7817, 7818, 7819, 7820)]
OUT = ROOT / "runs/predict_five_mov_trained_015_20261002"
CONF = 0.15
IMGSZ = 960


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(WEIGHTS))
    summaries = []

    for source in SOURCES:
        if not source.exists():
            raise FileNotFoundError(source)

        cap = cv2.VideoCapture(str(source))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open {source}")
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        stem = source.stem
        video_path = OUT / f"{stem}_predicted.mp4"
        csv_path = OUT / f"{stem}_detections.csv"
        writer = cv2.VideoWriter(
            str(video_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
        )
        if not writer.isOpened():
            raise RuntimeError(f"Could not create {video_path}")

        confidences = []
        detected_frames = 0
        frame_idx = 0
        with csv_path.open("w", newline="") as f:
            rows = csv.writer(f)
            rows.writerow(["frame", "time_sec", "confidence", "x1", "y1", "x2", "y2"])
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                result = model.predict(
                    source=frame, conf=CONF, imgsz=IMGSZ, device=0,
                    verbose=False, augment=False,
                )[0]
                boxes = result.boxes
                if boxes is not None and len(boxes):
                    detected_frames += 1
                    xyxy = boxes.xyxy.cpu().numpy()
                    confs = boxes.conf.cpu().numpy()
                    for box, score in zip(xyxy, confs):
                        score = float(score)
                        confidences.append(score)
                        rows.writerow([
                            frame_idx, f"{frame_idx / fps:.3f}", f"{score:.5f}",
                            *(f"{v:.1f}" for v in box),
                        ])
                writer.write(result.plot())
                frame_idx += 1
                if frame_idx % 500 == 0:
                    print(f"{stem}: {frame_idx}/{frame_count or '?'} frames", flush=True)

        cap.release()
        writer.release()
        summaries.append({
            "video": source.name,
            "frames": frame_idx,
            "fps": fps,
            "duration_sec": frame_idx / fps,
            "detected_frames": detected_frames,
            "detection_rate": detected_frames / frame_idx if frame_idx else 0,
            "detections": len(confidences),
            "mean_confidence": statistics.mean(confidences) if confidences else 0,
            "max_confidence": max(confidences, default=0),
            "annotated_video": video_path.name,
            "detections_csv": csv_path.name,
        })
        print(
            f"Finished {stem}: {frame_idx} frames; {len(confidences)} detections "
            f"in {detected_frames} frames", flush=True
        )

    with (OUT / "summary.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)
    print(f"Results: {OUT}", flush=True)


if __name__ == "__main__":
    main()
