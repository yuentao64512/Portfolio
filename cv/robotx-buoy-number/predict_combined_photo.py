#!/usr/bin/env python3
"""Predict buoy/boundary, digits and RGB light colour on one photo."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from predict_combined_buoy_digit_boundary_rgb import (
    DRAW_BGR,
    detect_rgb_cap,
    restricted_classifier_prediction,
)


def agnostic_nms(boxes, scores, iou_threshold=0.45):
    xywh = [[x1, y1, x2 - x1, y2 - y1] for x1, y1, x2, y2 in boxes]
    indices = cv2.dnn.NMSBoxes(xywh, scores, 0.0, iou_threshold)
    return [int(index) for index in indices]


def detect_cap_near_top(image, object_box):
    """Search a tight region above the buoy body for a coloured light cap."""
    height, width = image.shape[:2]
    x1, y1, x2, y2 = object_box
    bw, bh = x2 - x1, y2 - y1
    center_x = (x1 + x2) // 2
    rx1 = max(0, int(center_x - bw * 0.18))
    rx2 = min(width, int(center_x + bw * 0.18))
    ry1 = max(0, int(y1 - bh * 0.55))
    ry2 = min(height, int(y1 + bh * 0.08))
    roi = image[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return "unknown", None, 0.0
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 95, 70), (12, 255, 255)),
        cv2.inRange(hsv, (168, 95, 70), (179, 255, 255)),
    )
    masks = {
        "red": red,
        "green": cv2.inRange(hsv, (38, 100, 60), (85, 255, 255)),
        "blue": cv2.inRange(hsv, (100, 100, 55), (135, 255, 255)),
    }
    areas = {}
    for name, mask in masks.items():
        count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        areas[name] = int(stats[1:, cv2.CC_STAT_AREA].max()) if count > 1 else 0
    colour = max(areas, key=areas.get)
    area = areas[colour]
    if area < 4:
        return "unknown", None, area / max(1, roi.shape[0] * roi.shape[1])
    rgb = {"red": (255, 0, 0), "green": (0, 255, 0), "blue": (0, 0, 255)}[colour]
    return colour, rgb, area / max(1, roi.shape[0] * roi.shape[1])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=(
        "/home/robotx/Robotx_train/photo_2026-11-12_21-48-05.jpg"
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
        "photo_2026-11-12_21-48-05_prediction.jpg"
    ))
    parser.add_argument("--csv", default=(
        "/home/robotx/Robotx_train/combined_predictions/"
        "photo_2026-11-12_21-48-05_prediction.csv"
    ))
    parser.add_argument("--buoy-conf", type=float, default=0.10)
    parser.add_argument("--digit-conf", type=float, default=0.20)
    parser.add_argument("--marker-conf", type=float, default=0.35)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    image = cv2.imread(args.source)
    if image is None:
        raise SystemExit(f"Cannot read image: {args.source}")
    source_image = image.copy()
    height, width = image.shape[:2]
    buoy_model = YOLO(args.buoy_detector)
    number_model = YOLO(args.number_detector)
    marker_model = YOLO(args.marker_classifier)

    first = buoy_model.predict(
        image, imgsz=1280, conf=args.buoy_conf, iou=0.60,
        device=args.device, verbose=False,
    )[0]
    stage_boxes, stage_scores, stage_classes = [], [], []
    for box in first.boxes:
        x1, y1, x2, y2 = [int(round(value)) for value in box.xyxy[0].tolist()]
        stage_boxes.append((max(0, x1), max(0, y1), min(width, x2), min(height, y2)))
        stage_scores.append(float(box.conf.item()))
        stage_classes.append(str(first.names[int(box.cls.item())]))

    rows = []
    for index in agnostic_nms(stage_boxes, stage_scores):
        x1, y1, x2, y2 = stage_boxes[index]
        stage_class, buoy_conf = stage_classes[index], stage_scores[index]
        bw, bh = x2 - x1, y2 - y1
        pad_x, pad_y = max(2, int(bw * 0.08)), max(2, int(bh * 0.08))
        cx1, cy1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
        cx2, cy2 = min(width, x2 + pad_x), min(height, y2 + pad_y)
        crop = source_image[cy1:cy2, cx1:cx2]
        if crop.size == 0:
            continue

        digit_result = number_model.predict(
            crop, imgsz=320, conf=min(args.digit_conf, 0.08), iou=0.45,
            device=args.device, verbose=False,
        )[0]
        raw_digit_hits = []
        for digit_box in digit_result.boxes:
            digit = str(digit_result.names[int(digit_box.cls.item())])
            confidence = float(digit_box.conf.item())
            dx1, dy1, dx2, dy2 = [int(round(v)) for v in digit_box.xyxy[0].tolist()]
            raw_digit_hits.append((dx1, digit, confidence, cx1 + dx1, cy1 + dy1,
                                   cx1 + dx2, cy1 + dy2))
        if raw_digit_hits:
            digit_indices = agnostic_nms(
                [(hit[3], hit[4], hit[5], hit[6]) for hit in raw_digit_hits],
                [hit[2] for hit in raw_digit_hits], 0.40,
            )
            digit_hits = [raw_digit_hits[digit_index] for digit_index in digit_indices]
        else:
            digit_hits = []
        digit_hits.sort(key=lambda item: item[2], reverse=True)

        classified = marker_model.predict(crop, imgsz=320, device=args.device, verbose=False)[0]
        classified_digit, classified_digit_conf = restricted_classifier_prediction(
            classified, set("0123456789")
        )
        confident_digits = [hit for hit in digit_hits if hit[2] >= 0.12]
        classifier_says_number = classified_digit_conf >= 0.80
        final_class = (
            "number_buoy"
            if stage_class == "number_buoy" or confident_digits or classifier_says_number
            else "boundary"
        )
        marker, marker_conf = "?", 0.0
        digits, digit_scores = [], []
        if final_class == "boundary":
            marker, marker_conf = restricted_classifier_prediction(classified, {"C", "D"})
            if marker_conf < args.marker_conf:
                marker = "?"
        else:
            detected_classes = {hit[1] for hit in raw_digit_hits}
            final_digit = "?"
            if confident_digits:
                final_digit = confident_digits[0][1]
                if (
                    classified_digit_conf >= 0.50
                    and classified_digit in detected_classes
                ):
                    final_digit = classified_digit
            elif classifier_says_number:
                final_digit = classified_digit
            marker, marker_conf = final_digit, classified_digit_conf
            if final_digit != "?":
                digits = [final_digit]
            if digit_hits:
                _, _, confidence, gx1, gy1, gx2, gy2 = digit_hits[0]
                digit_scores = [confidence]
                cv2.rectangle(image, (gx1, gy1), (gx2, gy2), DRAW_BGR["digit"], 2)
                cv2.putText(image, f"{final_digit} {confidence:.2f}",
                            (gx1, max(18, gy1 - 4)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.44, DRAW_BGR["digit"], 2, cv2.LINE_AA)

        colour, colour_rgb, colour_score = detect_cap_near_top(
            source_image, (x1, y1, x2, y2)
        )
        box_colour = DRAW_BGR.get(final_class, (0, 255, 255))
        cv2.rectangle(image, (x1, y1), (x2, y2), box_colour, 2)
        if final_class == "number_buoy":
            identity = "digits=" + ("/".join(digits) if digits else marker)
        else:
            identity = f"marker={marker}"
        rgb_text = str(colour_rgb) if colour_rgb else "unknown"
        label = f"{final_class} {buoy_conf:.2f} | {identity} | {colour} RGB{rgb_text}"
        cv2.putText(image, label, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.44, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, label, (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.44, box_colour, 2, cv2.LINE_AA)
        rows.append({
            "class": final_class, "buoy_conf": f"{buoy_conf:.6f}",
            "marker": marker, "marker_conf": f"{marker_conf:.6f}",
            "digits": ";".join(digits),
            "digit_confidences": ";".join(f"{value:.6f}" for value in digit_scores),
            "light_colour": colour, "light_rgb": rgb_text,
            "light_score": f"{colour_score:.6f}",
            "x1": x1, "y1": y1, "x2": x2, "y2": y2,
        })

    output, csv_path = Path(args.output), Path(args.csv)
    output.parent.mkdir(parents=True, exist_ok=True)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
    fields = list(rows[0]) if rows else ["class"]
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"detections={len(rows)}")
    print(f"image={output}")
    print(f"csv={csv_path}")


if __name__ == "__main__":
    main()
