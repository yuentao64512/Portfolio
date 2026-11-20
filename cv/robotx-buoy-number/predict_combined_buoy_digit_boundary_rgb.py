#!/usr/bin/env python3
"""Combined buoy, digit, boundary-marker and RGB light prediction.

Pipeline:
1. Detect whole number buoys and boundary buoys.
2. Detect 0-9 digit boxes inside number-buoy crops.
3. Classify C/D inside boundary-buoy crops.
4. Detect red, green or blue light caps inside the top of every buoy crop.

OpenCV draws in BGR internally, but all labels and configured target tuples are
reported as RGB: red=(255,0,0), green=(0,255,0), blue=(0,0,255).
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


RGB_TARGETS = {
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
}

# Drawing colours are BGR because OpenCV uses BGR images.
DRAW_BGR = {
    "number_buoy": (0, 0, 255),       # RGB red
    "digit": (0, 255, 0),             # RGB green
    "boundary": (255, 0, 0),          # RGB blue
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "unknown": (180, 180, 180),
}


def restricted_classifier_prediction(result, allowed_names: set[str]):
    scores = result.probs.data.detach().cpu().numpy()
    allowed_ids = [idx for idx, name in result.names.items() if str(name) in allowed_names]
    if not allowed_ids:
        return "?", 0.0
    best_id = max(allowed_ids, key=lambda idx: float(scores[idx]))
    return str(result.names[best_id]), float(scores[best_id])


def largest_component(mask: np.ndarray) -> int:
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return 0
    return int(stats[1:, cv2.CC_STAT_AREA].max())


def detect_rgb_cap(bgr_crop: np.ndarray):
    """Return colour name, canonical RGB tuple and a small area score.

    Only the upper-centre part of the buoy is inspected so the cyan float base
    and water do not get mistaken for a blue light.
    """
    height, width = bgr_crop.shape[:2]
    if height < 8 or width < 8:
        return "unknown", None, 0.0

    x1, x2 = int(width * 0.18), int(width * 0.82)
    y2 = max(1, int(height * 0.45))
    roi = bgr_crop[0:y2, x1:x2]
    if roi.size == 0:
        return "unknown", None, 0.0

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    # Broad camera-safe ranges around the requested RGB primaries.
    red_a = cv2.inRange(hsv, (0, 75, 65), (12, 255, 255))
    red_b = cv2.inRange(hsv, (168, 75, 65), (179, 255, 255))
    masks = {
        "red": cv2.bitwise_or(red_a, red_b),
        "green": cv2.inRange(hsv, (35, 60, 50), (92, 255, 255)),
        "blue": cv2.inRange(hsv, (95, 70, 45), (140, 255, 255)),
    }
    kernel = np.ones((3, 3), np.uint8)
    scores = {}
    for name, mask in masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        scores[name] = largest_component(mask)

    colour = max(scores, key=scores.get)
    area = scores[colour]
    min_area = max(4, int(roi.shape[0] * roi.shape[1] * 0.0015))
    if area < min_area:
        return "unknown", None, area / max(1, roi.shape[0] * roi.shape[1])
    return colour, RGB_TARGETS[colour], area / max(1, roi.shape[0] * roi.shape[1])


def parse_source(value: str):
    return int(value) if value.isdigit() else value


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=(
        "/home/robotx/Robotx_train/two_weight_training_v1/IMG_3969.MOV"
    ))
    parser.add_argument("--buoy-detector", default=(
        "/home/robotx/Robotx_train/two_weight_training_v1/"
        "buoy_detector_yolo11m_1280_20261112_204219/weights/best.pt"
    ))
    parser.add_argument("--number-detector", default=(
        "/home/robotx/Robotx_train/number_detector_training_v1/"
        "number_detector_yolo11n_320_20261112_213348/weights/best.pt"
    ))
    parser.add_argument("--marker-classifier", default=(
        "/home/robotx/Robotx_train/two_weight_training_v1/"
        "marker_classifier_yolo11m_320_20261112_204219/weights/best.pt"
    ))
    parser.add_argument("--output", default=(
        "/home/robotx/Robotx_train/combined_predictions/"
        "IMG_3969_combined_buoy_digit_boundary_rgb.mp4"
    ))
    parser.add_argument("--csv", default=(
        "/home/robotx/Robotx_train/combined_predictions/"
        "IMG_3969_combined_buoy_digit_boundary_rgb.csv"
    ))
    parser.add_argument("--buoy-conf", type=float, default=0.15)
    parser.add_argument("--digit-conf", type=float, default=0.20)
    parser.add_argument("--marker-conf", type=float, default=0.25)
    parser.add_argument("--device", default="0")
    parser.add_argument("--show", action="store_true", help="Show live preview; press Q to stop")
    args = parser.parse_args()

    buoy_model = YOLO(args.buoy_detector)
    number_model = YOLO(args.number_detector)
    marker_model = YOLO(args.marker_classifier)

    source = parse_source(args.source)
    capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise SystemExit(f"Cannot open source: {args.source}")

    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    output_path, csv_path = Path(args.output), Path(args.csv)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        raise SystemExit(f"Cannot create output: {output_path}")

    fields = [
        "frame", "time_s", "buoy_class", "buoy_conf", "marker", "marker_conf",
        "digits", "digit_confidences", "light_colour", "light_rgb", "light_score",
        "x1", "y1", "x2", "y2",
    ]
    start = time.time()
    buoy_count = digit_count = 0
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=fields)
        csv_writer.writeheader()
        frame_index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            result = buoy_model.predict(
                frame, imgsz=1280, conf=args.buoy_conf, iou=0.45,
                device=args.device, verbose=False,
            )[0]
            for buoy_box in result.boxes:
                buoy_class = str(result.names[int(buoy_box.cls.item())])
                buoy_conf = float(buoy_box.conf.item())
                x1, y1, x2, y2 = [int(round(v)) for v in buoy_box.xyxy[0].tolist()]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(width, x2), min(height, y2)
                bw, bh = x2 - x1, y2 - y1
                if bw < 12 or bh < 12:
                    continue

                pad_x, pad_y = max(2, int(bw * 0.08)), max(2, int(bh * 0.08))
                cx1, cy1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
                cx2, cy2 = min(width, x2 + pad_x), min(height, y2 + pad_y)
                crop = frame[cy1:cy2, cx1:cx2]
                raw_buoy_crop = frame[y1:y2, x1:x2]
                if crop.size == 0:
                    continue

                colour_name, colour_rgb, colour_score = detect_rgb_cap(raw_buoy_crop)
                marker, marker_conf = "?", 0.0
                digit_values, digit_confidences = [], []

                if buoy_class == "number_buoy":
                    digit_result = number_model.predict(
                        crop, imgsz=320, conf=args.digit_conf, iou=0.45,
                        device=args.device, verbose=False,
                    )[0]
                    digit_rows = []
                    for digit_box in digit_result.boxes:
                        digit = str(digit_result.names[int(digit_box.cls.item())])
                        confidence = float(digit_box.conf.item())
                        dx1, dy1, dx2, dy2 = [
                            int(round(v)) for v in digit_box.xyxy[0].tolist()
                        ]
                        gx1, gy1 = cx1 + dx1, cy1 + dy1
                        gx2, gy2 = cx1 + dx2, cy1 + dy2
                        digit_rows.append((gx1, digit, confidence, gx1, gy1, gx2, gy2))
                    # Left-to-right order makes multi-face results readable.
                    digit_rows.sort(key=lambda row: row[0])
                    for _, digit, confidence, gx1, gy1, gx2, gy2 in digit_rows:
                        digit_values.append(digit)
                        digit_confidences.append(confidence)
                        cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), DRAW_BGR["digit"], 2)
                        text = f"{digit} {confidence:.2f}"
                        ty = max(18, gy1 - 4)
                        cv2.putText(frame, text, (gx1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.45, (0, 0, 0), 4, cv2.LINE_AA)
                        cv2.putText(frame, text, (gx1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.45, DRAW_BGR["digit"], 2, cv2.LINE_AA)
                        digit_count += 1
                elif buoy_class == "boundary":
                    marker_result = marker_model.predict(
                        crop, imgsz=320, device=args.device, verbose=False
                    )[0]
                    marker, marker_conf = restricted_classifier_prediction(
                        marker_result, {"C", "D"}
                    )
                    if marker_conf < args.marker_conf:
                        marker = "?"

                box_colour = DRAW_BGR.get(buoy_class, (0, 255, 255))
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_colour, 2)
                rgb_text = str(colour_rgb) if colour_rgb else "unknown"
                if buoy_class == "number_buoy":
                    identity = "digits=" + ("/".join(digit_values) if digit_values else "?")
                else:
                    identity = f"marker={marker}"
                label = (
                    f"{buoy_class} {buoy_conf:.2f} | {identity} | "
                    f"{colour_name} RGB{rgb_text}"
                )
                ty = max(22, y1 - 7)
                cv2.putText(frame, label, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                            0.50, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, label, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                            0.50, box_colour, 2, cv2.LINE_AA)

                csv_writer.writerow({
                    "frame": frame_index,
                    "time_s": f"{frame_index / fps:.3f}",
                    "buoy_class": buoy_class,
                    "buoy_conf": f"{buoy_conf:.6f}",
                    "marker": marker,
                    "marker_conf": f"{marker_conf:.6f}",
                    "digits": ";".join(digit_values),
                    "digit_confidences": ";".join(f"{v:.6f}" for v in digit_confidences),
                    "light_colour": colour_name,
                    "light_rgb": rgb_text,
                    "light_score": f"{colour_score:.6f}",
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                })
                buoy_count += 1

            writer.write(frame)
            if args.show:
                cv2.imshow("Combined buoy + digit + boundary + RGB", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            frame_index += 1
            if frame_index % 100 == 0 or frame_index == total:
                elapsed = max(0.001, time.time() - start)
                print(
                    f"frames={frame_index}/{total} speed={frame_index / elapsed:.1f}fps "
                    f"buoys={buoy_count} digits={digit_count}", flush=True,
                )

    capture.release()
    writer.release()
    if args.show:
        cv2.destroyAllWindows()
    elapsed = time.time() - start
    print(f"COMPLETE frames={frame_index} buoys={buoy_count} digits={digit_count} elapsed={elapsed:.1f}s")
    print(f"video={output_path}")
    print(f"csv={csv_path}")


if __name__ == "__main__":
    main()
