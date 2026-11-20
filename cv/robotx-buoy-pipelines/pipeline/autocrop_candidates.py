from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLOE


PROMPTS = [
    "white triangular navigation buoy with a black letter or number",
    "round orange mooring ball buoy",
    "floating pontoon docking station carrying upright white boxes",
    "long floating pontoon walkway",
    "small single floating rectangular obstacle block",
    "flat floating platform in water",
]
MODEL_A_NAMES = [
    "buoy_led_red",
    "buoy_led_green",
    "buoy_no_led",
    "dock",
    "home",
    "obstacle",
]
BOX_COLOURS = {
    "buoy_led_red": (0, 0, 255),
    "buoy_led_green": (0, 220, 0),
    "buoy_no_led": (255, 0, 255),
    "dock_led_red": (0, 80, 255),
    "dock_led_green": (0, 180, 100),
    "dock_no_led": (255, 200, 0),
    "home": (255, 120, 0),
    "obstacle": (0, 200, 255),
    "platform_water": (180, 180, 0),
}


def padded_crop(image: np.ndarray, box: list[float], padding: float = 0.15) -> tuple[np.ndarray, list[int]]:
    height, width = image.shape[:2]
    x1, y1, x2, y2 = box
    pad_x = (x2 - x1) * padding
    pad_y = (y2 - y1) * padding
    coords = [
        max(0, int(round(x1 - pad_x))),
        max(0, int(round(y1 - pad_y))),
        min(width, int(round(x2 + pad_x))),
        min(height, int(round(y2 + pad_y))),
    ]
    return image[coords[1] : coords[3], coords[0] : coords[2]].copy(), coords


def colour_pixels(crop: np.ndarray, top_only: bool) -> tuple[int, int]:
    if crop.size == 0:
        return 0, 0
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    if top_only:
        hsv = hsv[
            0 : max(1, int(hsv.shape[0] * 0.48)),
            int(hsv.shape[1] * 0.20) : max(1, int(hsv.shape[1] * 0.80)),
        ]
    red = (
        ((hsv[:, :, 0] <= 10) | (hsv[:, :, 0] >= 170))
        & (hsv[:, :, 1] >= 115)
        & (hsv[:, :, 2] >= 105)
    )
    green = (
        (hsv[:, :, 0] >= 35)
        & (hsv[:, :, 0] <= 95)
        & (hsv[:, :, 1] >= 105)
        & (hsv[:, :, 2] >= 85)
    )
    return int(red.sum()), int(green.sum())


def classify_light(crop: np.ndarray, top_only: bool) -> tuple[str, int, int]:
    red, green = colour_pixels(crop, top_only=top_only)
    minimum = max(3, int(crop.shape[0] * crop.shape[1] * 0.0007))
    if red >= minimum and red >= green * 1.8:
        return "red", red, green
    if green >= minimum and green >= red * 1.8:
        return "green", red, green
    return "none_or_off", red, green


def guess_marking(crop: np.ndarray) -> tuple[str, float]:
    if crop.shape[0] < 40 or crop.shape[1] < 30:
        return "unreadable", 0.0
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    roi = gray[int(gray.shape[0] * 0.24) : int(gray.shape[0] * 0.90)]
    mask = cv2.inRange(roi, 0, 95)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_area = float(max(1, roi.shape[0] * roi.shape[1]))
    contours = [contour for contour in contours if cv2.contourArea(contour) / roi_area >= 0.012]
    if not contours:
        return "none_or_unreadable", 0.0
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    perimeter = float(cv2.arcLength(contour, True))
    circularity = 0.0 if perimeter <= 0 else 4.0 * np.pi * area / (perimeter * perimeter)
    approx = cv2.approxPolyDP(contour, 0.035 * perimeter, True)
    if len(approx) == 4 and circularity < 0.72:
        return "diamond_candidate", round(min(1.0, area / (roi_area * 0.12)), 3)
    if len(approx) >= 6 and circularity >= 0.58:
        return "circle_candidate", round(min(1.0, circularity), 3)
    return "letter_digit_or_unknown", round(min(1.0, area / (roi_area * 0.10)), 3)


def yolo_line(class_id: int, box: list[float], width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    return (
        f"{class_id} {((x1+x2)/2)/width:.6f} {((y1+y2)/2)/height:.6f} "
        f"{(x2-x1)/width:.6f} {(y2-y1)/height:.6f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", default="yoloe-11s-seg.pt")
    parser.add_argument("--conf", type=float, default=0.08)
    args = parser.parse_args()

    crops_root = args.output / "crops"
    previews_root = args.output / "previews"
    labels_root = args.output / "labels_A_auto_review"
    model_b_root = args.output / "model_B_unlabeled_buoy_crops"
    for directory in (crops_root, previews_root, labels_root, model_b_root):
        directory.mkdir(parents=True, exist_ok=True)

    model = YOLOE(args.model)
    model.set_classes(PROMPTS)
    rows: list[dict[str, object]] = []
    counts: Counter[str] = Counter()
    image_files = sorted(path for path in args.images.iterdir() if path.suffix.lower() in {".jpg", ".jpeg", ".png"})

    for image_number, image_path in enumerate(image_files, start=1):
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        result = model.predict(
            str(image_path), imgsz=1280, conf=args.conf, iou=0.50,
            device=0, half=True, retina_masks=False, verbose=False,
        )[0]
        label_lines: list[str] = []
        detection_number = 0
        if result.boxes is not None:
            for prompt_id, confidence, xyxy in zip(
                result.boxes.cls.tolist(), result.boxes.conf.tolist(), result.boxes.xyxy.tolist()
            ):
                prompt_id = int(prompt_id)
                box = [float(value) for value in xyxy]
                box_height = box[3] - box[1]
                if box_height < 12:
                    continue
                crop, padded_box = padded_crop(image, box)
                if crop.size == 0:
                    continue

                identified = "rejected_negative"
                model_a_class: int | None = None
                light = "not_applicable"
                red_pixels = 0
                green_pixels = 0
                marking = "not_applicable"
                marking_confidence = 0.0

                if prompt_id == 0:
                    light, red_pixels, green_pixels = classify_light(crop, top_only=True)
                    if light == "red":
                        identified, model_a_class = "buoy_led_red", 0
                    elif light == "green":
                        identified, model_a_class = "buoy_led_green", 1
                    else:
                        identified, model_a_class = "buoy_no_led", 2
                    marking, marking_confidence = guess_marking(crop)
                elif prompt_id == 2:
                    light, red_pixels, green_pixels = classify_light(crop, top_only=False)
                    identified = {
                        "red": "dock_led_red",
                        "green": "dock_led_green",
                        "none_or_off": "dock_no_led",
                    }[light]
                    model_a_class = 3
                elif prompt_id == 3:
                    identified, model_a_class = "home", 4
                elif prompt_id == 4:
                    identified, model_a_class = "obstacle", 5
                elif prompt_id == 5:
                    identified = "platform_water"

                if identified == "rejected_negative":
                    continue
                destination_dir = crops_root / identified
                destination_dir.mkdir(parents=True, exist_ok=True)
                crop_name = f"{image_path.stem}__d{detection_number:03d}.jpg"
                cv2.imwrite(str(destination_dir / crop_name), crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if prompt_id == 0 and box_height >= 40:
                    cv2.imwrite(str(model_b_root / crop_name), crop, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if model_a_class is not None:
                    label_lines.append(yolo_line(model_a_class, box, width, height))

                colour = BOX_COLOURS[identified]
                x1, y1, x2, y2 = [int(round(value)) for value in box]
                cv2.rectangle(image, (x1, y1), (x2, y2), colour, 2)
                cv2.putText(
                    image, f"{identified} {confidence:.2f}", (x1, max(15, y1 - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA,
                )
                rows.append(
                    {
                        "source_image": image_path.name,
                        "crop": str((destination_dir / crop_name).relative_to(args.output)),
                        "prompt": PROMPTS[prompt_id],
                        "prompt_confidence": round(float(confidence), 6),
                        "identified_category": identified,
                        "model_a_class_id": "" if model_a_class is None else model_a_class,
                        "light_suggestion": light,
                        "red_pixels": red_pixels,
                        "green_pixels": green_pixels,
                        "marking_suggestion": marking,
                        "marking_confidence": marking_confidence,
                        "box_xyxy": json.dumps([round(value, 2) for value in box]),
                        "review_required": "yes",
                    }
                )
                counts[identified] += 1
                detection_number += 1

        (labels_root / f"{image_path.stem}.txt").write_text(
            "\n".join(label_lines) + ("\n" if label_lines else ""), encoding="utf-8"
        )
        cv2.imwrite(str(previews_root / image_path.name), image)
        counts["images_processed"] += 1
        print(f"[{image_number:04d}/{len(image_files):04d}] {image_path.name}: {detection_number}", flush=True)

    fieldnames = [
        "source_image", "crop", "prompt", "prompt_confidence", "identified_category",
        "model_a_class_id", "light_suggestion", "red_pixels", "green_pixels",
        "marking_suggestion", "marking_confidence", "box_xyxy", "review_required",
    ]
    with (args.output / "candidate_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(counts)
    summary["status"] = "AUTO_CANDIDATES_REVIEW_REQUIRED"
    summary["model_a_names"] = MODEL_A_NAMES
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
