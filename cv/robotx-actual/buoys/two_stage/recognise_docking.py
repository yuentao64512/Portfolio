#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
COLOURS = {"dock": (255, 200, 0), "home": (255, 120, 0)}


def parse_source(value: str) -> int | str:
    return int(value) if value.isdigit() else value


def class_name(names: dict[int, str] | list[str], class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    return str(names[class_id]) if 0 <= class_id < len(names) else str(class_id)


def open_source(source: int | str):
    if isinstance(source, str) and Path(source).suffix.lower() in IMAGE_SUFFIXES:
        frame = cv2.imread(source)
        if frame is None:
            raise SystemExit(f"Cannot read image: {source}")
        return None, frame, 1.0, True

    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"Cannot open source: {source}")
    fps = capture.get(cv2.CAP_PROP_FPS)
    if fps <= 0 or fps > 240:
        fps = 30.0
    return capture, None, fps, False


def main() -> None:
    root = Path(__file__).resolve().parent
    default_engine = root / "models" / "model_a_best.engine"
    default_pt = root / "models" / "model_a_best.pt"

    parser = argparse.ArgumentParser(description="Recognise dock/home targets with Model A")
    parser.add_argument("--source", default="0", help="camera index, video, or image")
    parser.add_argument("--model", type=Path, default=default_engine if default_engine.is_file() else default_pt)
    parser.add_argument("--target-classes", nargs="+", default=["dock", "home"])
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.50)
    parser.add_argument("--alignment-tolerance", type=float, default=0.12,
                        help="fraction of half-image width used for ALIGNED")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--log-csv", type=Path)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--no-track", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()

    if not args.model.is_file():
        raise SystemExit(f"Missing model: {args.model}")

    source = parse_source(args.source)
    capture, still_frame, fps, is_image = open_source(source)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = root.parent / "output"
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.output is None:
        args.output = output_dir / (f"dock_{timestamp}.jpg" if is_image else f"dock_{timestamp}.mp4")
    if args.log_csv is None:
        args.log_csv = output_dir / f"dock_{timestamp}.csv"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.log_csv.parent.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(args.model))
    using_tensorrt = args.model.suffix.lower() == ".engine"
    print(f"Model: {args.model}", flush=True)
    print(f"Backend: {'TensorRT' if using_tensorrt else 'PyTorch'}", flush=True)
    print(f"Targets: {', '.join(args.target_classes)}", flush=True)

    csv_file = args.log_csv.open("w", newline="", encoding="utf-8")
    writer = csv.writer(csv_file)
    writer.writerow([
        "frame", "time_seconds", "track_id", "class", "confidence",
        "x1", "y1", "x2", "y2", "center_offset", "alignment", "primary",
    ])

    video_writer = None
    frame_index = 0
    started = time.perf_counter()
    try:
        while True:
            if is_image:
                if frame_index > 0:
                    break
                frame = still_frame.copy()
            else:
                ok, frame = capture.read()
                if not ok:
                    break

            height, width = frame.shape[:2]
            inference = dict(source=frame, imgsz=args.imgsz, conf=args.conf, iou=args.iou,
                             device=args.device, verbose=False)
            if args.no_track:
                result = model.predict(**inference)[0]
            else:
                result = model.track(persist=True, tracker="bytetrack.yaml", **inference)[0]

            detections = []
            if result.boxes is not None and len(result.boxes):
                boxes = result.boxes.xyxy.cpu().tolist()
                class_ids = result.boxes.cls.cpu().tolist()
                confidences = result.boxes.conf.cpu().tolist()
                track_ids = result.boxes.id.int().cpu().tolist() if result.boxes.id is not None else [None] * len(boxes)
                for box, class_id, confidence, track_id in zip(boxes, class_ids, confidences, track_ids):
                    name = class_name(result.names, int(class_id))
                    if name not in args.target_classes:
                        continue
                    x1, y1, x2, y2 = [int(round(v)) for v in box]
                    area = max(0, x2 - x1) * max(0, y2 - y1)
                    center_x = (x1 + x2) / 2.0
                    offset = (center_x - width / 2.0) / max(width / 2.0, 1.0)
                    if abs(offset) <= args.alignment_tolerance:
                        alignment = "ALIGNED"
                    elif offset < 0:
                        alignment = "LEFT"
                    else:
                        alignment = "RIGHT"
                    detections.append({
                        "box": (x1, y1, x2, y2), "name": name, "confidence": float(confidence),
                        "track_id": track_id, "area": area, "offset": offset, "alignment": alignment,
                    })

            primary = max(detections, key=lambda item: item["area"], default=None)
            cv2.line(frame, (width // 2, 0), (width // 2, height), (0, 255, 255), 2)
            tol_px = int(width * args.alignment_tolerance / 2.0)
            cv2.line(frame, (width // 2 - tol_px, 0), (width // 2 - tol_px, height), (80, 80, 80), 1)
            cv2.line(frame, (width // 2 + tol_px, 0), (width // 2 + tol_px, height), (80, 80, 80), 1)

            for detection in detections:
                x1, y1, x2, y2 = detection["box"]
                is_primary = detection is primary
                colour = (0, 255, 255) if is_primary else COLOURS.get(detection["name"], (255, 255, 0))
                thickness = 4 if is_primary else 2
                cv2.rectangle(frame, (x1, y1), (x2, y2), colour, thickness)
                track_text = f" id={detection['track_id']}" if detection["track_id"] is not None else ""
                label = (f"{detection['name']} {detection['confidence']:.2f}{track_text} "
                         f"{detection['alignment']} offset={detection['offset']:+.2f}")
                cv2.putText(frame, label, (x1, max(24, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.65, colour, 2, cv2.LINE_AA)
                writer.writerow([
                    frame_index, frame_index / fps, "" if detection["track_id"] is None else detection["track_id"],
                    detection["name"], f"{detection['confidence']:.5f}", x1, y1, x2, y2,
                    f"{detection['offset']:+.5f}", detection["alignment"], int(is_primary),
                ])

            state = "SEARCHING" if primary is None else f"TARGET {primary['name']} - {primary['alignment']}"
            state_colour = (0, 0, 255) if primary is None else ((0, 255, 0) if primary["alignment"] == "ALIGNED" else (0, 255, 255))
            cv2.rectangle(frame, (0, 0), (width, 48), (0, 0, 0), -1)
            cv2.putText(frame, state, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, state_colour, 2, cv2.LINE_AA)

            if is_image:
                if not cv2.imwrite(str(args.output), frame):
                    raise RuntimeError(f"Failed to write {args.output}")
            else:
                if video_writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    video_writer = cv2.VideoWriter(str(args.output), fourcc, fps, (width, height))
                    if not video_writer.isOpened():
                        raise RuntimeError(f"Failed to open video output: {args.output}")
                video_writer.write(frame)

            if args.show:
                cv2.imshow("Dock recognition", frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break

            frame_index += 1
            if args.max_frames and frame_index >= args.max_frames:
                break
    finally:
        csv_file.close()
        if capture is not None:
            capture.release()
        if video_writer is not None:
            video_writer.release()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = max(time.perf_counter() - started, 1e-9)
    print(f"Finished: frames={frame_index}, average_fps={frame_index / elapsed:.2f}", flush=True)
    print(f"Annotated output: {args.output}", flush=True)
    print(f"Detection log: {args.log_csv}", flush=True)


if __name__ == "__main__":
    main()
