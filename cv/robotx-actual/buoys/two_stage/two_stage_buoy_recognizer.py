#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import time
from collections import Counter, defaultdict, deque
from datetime import datetime
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import torch
from ultralytics import YOLO


MODEL_A_BUOY_CLASSES = {0, 1, 2}
MODEL_A_COLOURS = {
    "buoy_led_red": (0, 0, 255),
    "buoy_led_green": (0, 220, 0),
    "buoy_no_led": (255, 0, 255),
    "dock": (255, 200, 0),
    "home": (255, 120, 0),
    "obstacle": (0, 200, 255),
}


def parse_source(value: str) -> int | str:
    return int(value) if value.isdigit() else value


def class_name(names: dict[int, str] | list[str], class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    return str(names[class_id]) if 0 <= class_id < len(names) else str(class_id)


def stable_vote(history: deque[str] | None, fallback: str) -> str:
    if not history:
        return fallback
    return Counter(history).most_common(1)[0][0]


def choose_model_b_label(result) -> tuple[str | None, int | None, float, float]:
    if result.boxes is None or len(result.boxes) == 0:
        return None, None, 0.0, 0.0
    shape_scores: defaultdict[str, float] = defaultdict(float)
    digit_scores: defaultdict[int, float] = defaultdict(float)
    for class_id, confidence in zip(result.boxes.cls.tolist(), result.boxes.conf.tolist()):
        class_id = int(class_id)
        confidence = float(confidence)
        if class_id in (0, 1):
            name = class_name(result.names, class_id)
            shape_scores[name] = max(shape_scores[name], confidence)
        elif 2 <= class_id <= 11:
            digit = class_id - 2
            digit_scores[digit] = max(digit_scores[digit], confidence)
    shape = max(shape_scores, key=shape_scores.get) if shape_scores else None
    digit = max(digit_scores, key=digit_scores.get) if digit_scores else None
    return (
        shape,
        digit,
        shape_scores.get(shape, 0.0) if shape else 0.0,
        digit_scores.get(digit, 0.0) if digit is not None else 0.0,
    )


def padded_box(box: tuple[int, int, int, int], width: int, height: int, padding: float) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = box
    pad_x = int(round((x2 - x1) * padding))
    pad_y = int(round((y2 - y1) * padding))
    return max(0, x1 - pad_x), max(0, y1 - pad_y), min(width, x2 + pad_x), min(height, y2 + pad_y)


def source_frames(source: int | str) -> tuple[Iterable[tuple[int, np.ndarray, float]], float, bool]:
    if isinstance(source, str):
        path = Path(source)
        if path.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp", ".webp"}:
            image = cv2.imread(str(path))
            if image is None:
                raise RuntimeError(f"Cannot read image: {path}")

            def one_image() -> Iterable[tuple[int, np.ndarray, float]]:
                yield 0, image, 0.0

            return one_image(), 1.0, True

    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise RuntimeError(f"Cannot open source: {source}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not np.isfinite(fps) or fps <= 1.0:
        fps = 20.0

    def video_frames() -> Iterable[tuple[int, np.ndarray, float]]:
        frame_index = 0
        try:
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                timestamp_seconds = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
                if timestamp_seconds <= 0.0:
                    timestamp_seconds = frame_index / fps
                yield frame_index, frame, timestamp_seconds
                frame_index += 1
        finally:
            capture.release()

    return video_frames(), fps, False


def main() -> None:
    script_root = Path(__file__).resolve().parent
    model_a_engine = script_root / "models" / "model_a_best.engine"
    model_b_engine = script_root / "models" / "model_b_best.engine"
    parser = argparse.ArgumentParser(description="Two-stage RobotX buoy recognizer")
    parser.add_argument("--source", default="0", help="camera index, video, or image")
    parser.add_argument(
        "--model-a", type=Path,
        default=model_a_engine if model_a_engine.is_file() else script_root / "models" / "model_a_best.pt",
    )
    parser.add_argument(
        "--model-b", type=Path,
        default=model_b_engine if model_b_engine.is_file() else script_root / "models" / "model_b_best.pt",
    )
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz-a", type=int, default=1280)
    parser.add_argument("--imgsz-b", type=int, default=640)
    parser.add_argument("--conf-a", type=float, default=0.25)
    parser.add_argument("--conf-b", type=float, default=0.20)
    parser.add_argument("--iou-a", type=float, default=0.50)
    parser.add_argument("--padding", type=float, default=0.08)
    parser.add_argument("--vote-window", type=int, default=30)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--log-csv", type=Path)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--no-track", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()

    if not args.model_a.is_file() or not args.model_b.is_file():
        raise SystemExit(f"Missing weights: model_a={args.model_a}, model_b={args.model_b}")
    if not torch.cuda.is_available():
        raise SystemExit("CUDA is not available in this Python environment")

    source = parse_source(args.source)
    frames, fps, is_image = source_frames(source)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    runs_root = script_root / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    output_path = args.output or runs_root / (f"recognised_{stamp}.jpg" if is_image else f"recognised_{stamp}.mp4")
    log_path = args.log_csv or runs_root / f"detections_{stamp}.csv"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.parent.mkdir(parents=True, exist_ok=True)

    model_a = YOLO(str(args.model_a))
    model_b = YOLO(str(args.model_b))
    print(f"CUDA device: {torch.cuda.get_device_name(0)}", flush=True)
    print(f"Model A: {args.model_a}", flush=True)
    print(f"Model B: {args.model_b}", flush=True)
    print(f"Source: {source}", flush=True)

    led_history: defaultdict[int, deque[str]] = defaultdict(lambda: deque(maxlen=args.vote_window))
    shape_history: defaultdict[int, deque[str]] = defaultdict(lambda: deque(maxlen=args.vote_window))
    digit_history: defaultdict[int, deque[str]] = defaultdict(lambda: deque(maxlen=args.vote_window))
    writer: cv2.VideoWriter | None = None
    csv_handle = log_path.open("w", newline="", encoding="utf-8")
    csv_writer = csv.DictWriter(
        csv_handle,
        fieldnames=[
            "frame", "time_seconds", "track_id", "model_a_class", "stable_model_a_class",
            "model_a_confidence", "x1", "y1", "x2", "y2", "shape", "digit",
            "shape_confidence", "digit_confidence",
        ],
    )
    csv_writer.writeheader()
    started = time.perf_counter()
    processed = 0

    try:
        for frame_index, frame, timestamp_seconds in frames:
            if args.max_frames > 0 and processed >= args.max_frames:
                break
            height, width = frame.shape[:2]
            annotated = frame.copy()
            inference_args = dict(
                source=frame,
                imgsz=args.imgsz_a,
                conf=args.conf_a,
                iou=args.iou_a,
                device=args.device,
                verbose=False,
            )
            if args.no_track:
                result_a = model_a.predict(**inference_args)[0]
            else:
                result_a = model_a.track(persist=True, tracker="bytetrack.yaml", **inference_args)[0]

            detections: list[dict[str, object]] = []
            buoy_crops: list[np.ndarray] = []
            buoy_detection_indices: list[int] = []
            if result_a.boxes is not None:
                track_ids = (
                    result_a.boxes.id.int().cpu().tolist()
                    if result_a.boxes.id is not None else [None] * len(result_a.boxes)
                )
                for detection_index, (box_values, class_id_value, confidence_value, track_id) in enumerate(
                    zip(
                        result_a.boxes.xyxy.cpu().tolist(),
                        result_a.boxes.cls.int().cpu().tolist(),
                        result_a.boxes.conf.cpu().tolist(),
                        track_ids,
                    )
                ):
                    x1, y1, x2, y2 = [int(round(value)) for value in box_values]
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(width, x2), min(height, y2)
                    class_id = int(class_id_value)
                    current_name = class_name(result_a.names, class_id)
                    tracking_key = int(track_id) if track_id is not None else -(detection_index + 1)
                    if track_id is not None and class_id in MODEL_A_BUOY_CLASSES:
                        led_history[tracking_key].append(current_name)
                    stable_name = stable_vote(led_history.get(tracking_key), current_name)
                    detection: dict[str, object] = {
                        "box": (x1, y1, x2, y2),
                        "class_id": class_id,
                        "name": current_name,
                        "stable_name": stable_name,
                        "confidence": float(confidence_value),
                        "track_id": track_id,
                        "tracking_key": tracking_key,
                        "shape": None,
                        "digit": None,
                        "shape_confidence": 0.0,
                        "digit_confidence": 0.0,
                    }
                    detections.append(detection)
                    if class_id in MODEL_A_BUOY_CLASSES:
                        crop_box = padded_box((x1, y1, x2, y2), width, height, args.padding)
                        cx1, cy1, cx2, cy2 = crop_box
                        crop = frame[cy1:cy2, cx1:cx2]
                        if crop.size:
                            detection["crop_box"] = crop_box
                            buoy_crops.append(crop)
                            buoy_detection_indices.append(len(detections) - 1)

            if buoy_crops:
                model_b_args = dict(
                    imgsz=args.imgsz_b, conf=args.conf_b, iou=0.45,
                    device=args.device, verbose=False,
                )
                if args.model_b.suffix.lower() == ".engine":
                    # The exported TensorRT engine has fixed batch size 1.
                    results_b = [model_b.predict(source=crop, **model_b_args)[0] for crop in buoy_crops]
                else:
                    results_b = model_b.predict(source=buoy_crops, **model_b_args)
                for result_b, detection_index in zip(results_b, buoy_detection_indices):
                    detection = detections[detection_index]
                    shape, digit, shape_confidence, digit_confidence = choose_model_b_label(result_b)
                    tracking_key = int(detection["tracking_key"])
                    if detection["track_id"] is not None:
                        if shape is not None:
                            shape_history[tracking_key].append(shape)
                        if digit is not None:
                            digit_history[tracking_key].append(str(digit))
                    stable_shape = stable_vote(shape_history.get(tracking_key), shape or "unknown")
                    stable_digit_text = stable_vote(digit_history.get(tracking_key), str(digit) if digit is not None else "unknown")
                    detection["shape"] = None if stable_shape == "unknown" else stable_shape
                    detection["digit"] = None if stable_digit_text == "unknown" else int(stable_digit_text)
                    detection["shape_confidence"] = shape_confidence
                    detection["digit_confidence"] = digit_confidence

                    if result_b.boxes is not None and "crop_box" in detection:
                        crop_x1, crop_y1, _, _ = detection["crop_box"]
                        for inner_box, inner_class, inner_conf in zip(
                            result_b.boxes.xyxy.cpu().tolist(),
                            result_b.boxes.cls.int().cpu().tolist(),
                            result_b.boxes.conf.cpu().tolist(),
                        ):
                            ix1, iy1, ix2, iy2 = [int(round(value)) for value in inner_box]
                            ix1, ix2 = ix1 + crop_x1, ix2 + crop_x1
                            iy1, iy2 = iy1 + crop_y1, iy2 + crop_y1
                            inner_name = class_name(result_b.names, int(inner_class))
                            cv2.rectangle(annotated, (ix1, iy1), (ix2, iy2), (0, 255, 255), 1)
                            cv2.putText(
                                annotated, f"{inner_name} {float(inner_conf):.2f}",
                                (ix1, max(12, iy1 - 3)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.38, (0, 255, 255), 1, cv2.LINE_AA,
                            )

            for detection in detections:
                x1, y1, x2, y2 = detection["box"]
                stable_name = str(detection["stable_name"])
                confidence = float(detection["confidence"])
                colour = MODEL_A_COLOURS.get(stable_name, (255, 255, 255))
                track_text = f" id={detection['track_id']}" if detection["track_id"] is not None else ""
                detail = ""
                if detection["shape"] is not None:
                    detail += f" {detection['shape']}"
                if detection["digit"] is not None:
                    detail += f" digit={detection['digit']}"
                label = f"{stable_name} {confidence:.2f}{track_text}{detail}"
                cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(
                    annotated, label, (x1, max(18, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, colour, 2, cv2.LINE_AA,
                )
                csv_writer.writerow(
                    {
                        "frame": frame_index,
                        "time_seconds": f"{timestamp_seconds:.3f}",
                        "track_id": "" if detection["track_id"] is None else detection["track_id"],
                        "model_a_class": detection["name"],
                        "stable_model_a_class": stable_name,
                        "model_a_confidence": f"{confidence:.5f}",
                        "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                        "shape": "" if detection["shape"] is None else detection["shape"],
                        "digit": "" if detection["digit"] is None else detection["digit"],
                        "shape_confidence": f"{float(detection['shape_confidence']):.5f}",
                        "digit_confidence": f"{float(detection['digit_confidence']):.5f}",
                    }
                )

            if is_image:
                cv2.imwrite(str(output_path), annotated)
            else:
                if writer is None:
                    writer = cv2.VideoWriter(
                        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
                    )
                    if not writer.isOpened():
                        raise RuntimeError(f"Cannot create output video: {output_path}")
                writer.write(annotated)

            if args.show:
                cv2.imshow("Two-stage buoy recognizer", annotated)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            processed += 1
            if processed % 30 == 0:
                elapsed = max(1e-6, time.perf_counter() - started)
                print(f"frames={processed} average_fps={processed / elapsed:.2f}", flush=True)
    finally:
        if writer is not None:
            writer.release()
        csv_handle.close()
        if args.show:
            cv2.destroyAllWindows()

    elapsed = max(1e-6, time.perf_counter() - started)
    print(f"Finished: frames={processed}, average_fps={processed / elapsed:.2f}", flush=True)
    print(f"Annotated output: {output_path}", flush=True)
    print(f"Detection log: {log_path}", flush=True)


if __name__ == "__main__":
    main()
