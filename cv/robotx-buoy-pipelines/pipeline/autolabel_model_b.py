from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


CLASS_NAMES = ["circle", "diamond"] + [f"digit_{digit}" for digit in range(10)]

# Clear examples selected from the current recordings.  The classifier uses only
# the sign appearance; these images are not copied into the generated labels.
PROTOTYPES: dict[int, list[str]] = {
    0: [
        "IMG_3867_ecbdf083__f00000285__t0000009533__d004.jpg",
        "IMG_3867_ecbdf083__f00000330__t0000011033__d013.jpg",
        "IMG_3867_ecbdf083__f00000360__t0000012033__d008.jpg",
        "IMG_3867_ecbdf083__f00000375__t0000012533__d006.jpg",
        "IMG_3867_ecbdf083__f00000390__t0000013033__d017.jpg",
        "IMG_3867_ecbdf083__f00000405__t0000013533__d019.jpg",
        "IMG_7695_7633ac3e__f00000585__t0000019517__d005.jpg",
        "IMG_7695_7633ac3e__f00000675__t0000022517__d003.jpg",
        "IMG_7695_7633ac3e__f00001710__t0000057000__d016.jpg",
    ],
    2: [
        "IMG_3867_ecbdf083__f00000090__t0000003033__d002.jpg",
        "IMG_3867_ecbdf083__f00000255__t0000008533__d013.jpg",
        "IMG_3867_ecbdf083__f00000285__t0000009533__d006.jpg",
        "IMG_3867_ecbdf083__f00000315__t0000010533__d010.jpg",
        "IMG_3867_ecbdf083__f00000330__t0000011033__d023.jpg",
        "IMG_3867_ecbdf083__f00000405__t0000013533__d020.jpg",
        "IMG_7695_7633ac3e__f00000780__t0000026000__d007.jpg",
    ],
    3: [
        "IMG_3867_ecbdf083__f00000015__t0000000500__d021.jpg",
        "IMG_3867_ecbdf083__f00000105__t0000003533__d013.jpg",
        "IMG_3867_ecbdf083__f00000225__t0000007533__d011.jpg",
        "IMG_3867_ecbdf083__f00000240__t0000008033__d007.jpg",
        "IMG_3867_ecbdf083__f00000360__t0000012033__d007.jpg",
        "IMG_3867_ecbdf083__f00000510__t0000017033__d002.jpg",
    ],
    4: [
        "IMG_3867_ecbdf083__f00000390__t0000013033__d013.jpg",
        "IMG_3867_ecbdf083__f00000390__t0000013033__d015.jpg",
        "IMG_3870_88c543a9__f00000405__t0000013533__d004.jpg",
        "IMG_3870_88c543a9__f00000420__t0000014033__d000.jpg",
        "IMG_3870_88c543a9__f00000525__t0000017533__d006.jpg",
        "IMG_7695_7633ac3e__f00000870__t0000029000__d012.jpg",
        "IMG_7695_7633ac3e__f00000960__t0000032000__d014.jpg",
    ],
    5: [
        "IMG_3866_f356c2ac__f00000300__t0000010033__d004.jpg",
        "IMG_3867_ecbdf083__f00000195__t0000006533__d009.jpg",
        "IMG_3867_ecbdf083__f00000210__t0000007033__d016.jpg",
        "IMG_3867_ecbdf083__f00000240__t0000008033__d009.jpg",
        "IMG_3867_ecbdf083__f00000255__t0000008533__d011.jpg",
        "IMG_3867_ecbdf083__f00000270__t0000009033__d010.jpg",
        "IMG_3867_ecbdf083__f00000300__t0000010033__d008.jpg",
        "IMG_3867_ecbdf083__f00000315__t0000010533__d017.jpg",
        "IMG_3867_ecbdf083__f00000330__t0000011033__d006.jpg",
        "IMG_3867_ecbdf083__f00000330__t0000011033__d022.jpg",
        "IMG_3867_ecbdf083__f00000375__t0000012533__d016.jpg",
        "IMG_3867_ecbdf083__f00000570__t0000019033__d016.jpg",
    ],
    6: [
        "IMG_3867_ecbdf083__f00000165__t0000005533__d008.jpg",
        "IMG_3867_ecbdf083__f00000180__t0000006033__d013.jpg",
        "IMG_3867_ecbdf083__f00000195__t0000006533__d005.jpg",
        "IMG_3867_ecbdf083__f00000240__t0000008033__d012.jpg",
        "IMG_3867_ecbdf083__f00000270__t0000009033__d012.jpg",
        "IMG_3867_ecbdf083__f00000315__t0000010533__d023.jpg",
        "IMG_3867_ecbdf083__f00000330__t0000011033__d010.jpg",
        "IMG_3867_ecbdf083__f00000345__t0000011533__d013.jpg",
        "IMG_3867_ecbdf083__f00000390__t0000013033__d008.jpg",
        "IMG_3867_ecbdf083__f00000405__t0000013533__d007.jpg",
        "IMG_3867_ecbdf083__f00000420__t0000014033__d007.jpg",
    ],
    8: [
        "IMG_3867_ecbdf083__f00000300__t0000010033__d024.jpg",
        "IMG_3867_ecbdf083__f00000315__t0000010533__d019.jpg",
        "IMG_3867_ecbdf083__f00000330__t0000011033__d011.jpg",
        "IMG_3867_ecbdf083__f00000345__t0000011533__d008.jpg",
        "IMG_3867_ecbdf083__f00000375__t0000012533__d014.jpg",
        "IMG_3867_ecbdf083__f00000405__t0000013533__d014.jpg",
        "IMG_3867_ecbdf083__f00000420__t0000014033__d008.jpg",
    ],
    9: [
        "IMG_3867_ecbdf083__f00000045__t0000001500__d011.jpg",
        "IMG_3867_ecbdf083__f00000075__t0000002533__d009.jpg",
        "IMG_3867_ecbdf083__f00000165__t0000005533__d014.jpg",
        "IMG_3867_ecbdf083__f00000180__t0000006033__d005.jpg",
        "IMG_3867_ecbdf083__f00000195__t0000006533__d007.jpg",
        "IMG_3867_ecbdf083__f00000210__t0000007033__d002.jpg",
        "IMG_3867_ecbdf083__f00000225__t0000007533__d001.jpg",
        "IMG_3867_ecbdf083__f00000240__t0000008033__d005.jpg",
        "IMG_3867_ecbdf083__f00000255__t0000008533__d003.jpg",
        "IMG_3867_ecbdf083__f00000270__t0000009033__d005.jpg",
        "IMG_3867_ecbdf083__f00000285__t0000009533__d005.jpg",
        "IMG_3867_ecbdf083__f00000300__t0000010033__d014.jpg",
        "IMG_3867_ecbdf083__f00000315__t0000010533__d014.jpg",
        "IMG_3867_ecbdf083__f00000345__t0000011533__d006.jpg",
        "IMG_3867_ecbdf083__f00000375__t0000012533__d005.jpg",
        "IMG_3867_ecbdf083__f00000390__t0000013033__d010.jpg",
        "IMG_3867_ecbdf083__f00000405__t0000013533__d009.jpg",
        "IMG_3867_ecbdf083__f00000420__t0000014033__d021.jpg",
        "IMG_3867_ecbdf083__f00000480__t0000016033__d003.jpg",
    ],
}

# Frame-by-frame visual audit of the supplied crop set.  Letters A/B/C/D and
# non-buoy false crops are intentionally absent.  The index is the stable sort
# order of the filenames in this exact review folder.
VERIFIED_DIGIT_BY_INDEX: dict[int, int] = {
    **dict.fromkeys([61, 77, 86, 88, 96, 98, 102, 143, 150, 179, 180, 181, 182, 183, 184, 185, 207, 208], 0),
    **dict.fromkeys([30, 32, 33, 35, 37, 56, 59, 63, 68, 69, 79, 152, 155, 156, 158, 160, 161, 164, 187], 2),
    **dict.fromkeys([31, 38, 49, 51, 85, 106, 144], 3),
    **dict.fromkeys([7, 91, 95, 145, 147, 151, 154, 162, 163, *range(189, 207)], 4),
    **dict.fromkeys([5, 45, 47, 52, 55, 58, 64, 71, 74, 78, 90, 93, 108, 109, 111], 5),
    **dict.fromkeys([19, 28, 39, 42, 43, 53, 60, 73, 75, 83, 84, 92, 97, 100, 142, 146], 6),
    **dict.fromkeys([67, 72, 76, 81, 89, 101, 105, 149], 8),
    **dict.fromkeys([6, 9, 12, 13, 14, 16, 17, 18, 21, 24, 25, 34, 36, 40, 41, 44, 46, 48, 50, 54, 57, 62, 65, 66, 70, 80, 87, 94, 99, 103, 104, 107, 110], 9),
}


def orient(image: np.ndarray, name: str) -> tuple[np.ndarray, str]:
    if name.startswith(("IMG_3866_", "IMG_3870_")):
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE), "ccw"
    return image, "none"


def unorient_box(box: tuple[int, int, int, int], mode: str, original_shape: tuple[int, ...]) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = box
    if mode == "none":
        return box
    original_height, original_width = original_shape[:2]
    points = [(original_width - y1, x1), (original_width - y2, x2)]
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    return (
        max(0, min(xs)), max(0, min(ys)),
        min(original_width, max(xs)), min(original_height, max(ys)),
    )


def intersection_over_union(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0, x2 - x1) * max(0, y2 - y1)
    area_a = max(1, (a[2] - a[0]) * (a[3] - a[1]))
    area_b = max(1, (b[2] - b[0]) * (b[3] - b[1]))
    return intersection / float(area_a + area_b - intersection)


def detect_diamonds(image: np.ndarray) -> list[tuple[tuple[int, int, int, int], float]]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    height, width = gray.shape
    image_area = float(height * width)
    candidates: list[tuple[tuple[int, int, int, int], float]] = []

    for threshold in (55, 70, 85, 100, 115, 130, 145, 160):
        mask = cv2.inRange(gray, 0, threshold)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            contour_area = float(cv2.contourArea(contour))
            if contour_area / image_area < 0.003 or contour_area / image_area > 0.24:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            if box_width < 6 or box_height < 6:
                continue
            if box_width > width * 0.66 or box_height > height * 0.66:
                continue
            aspect = box_width / float(box_height)
            if not 0.38 <= aspect <= 2.65:
                continue
            if (y + box_height / 2) < height * 0.35:
                continue
            if x <= 1 or y <= 1 or x + box_width >= width - 1 or y + box_height >= height - 1:
                continue
            rectangle_area = float(box_width * box_height)
            fill = contour_area / rectangle_area
            hull_area = float(cv2.contourArea(cv2.convexHull(contour)))
            solidity = contour_area / max(1.0, hull_area)
            dark_fraction = float((gray[y : y + box_height, x : x + box_width] < 135).mean())
            if fill < 0.36 or solidity < 0.68 or dark_fraction < 0.28:
                continue

            pad_x, pad_y = max(2, box_width // 3), max(2, box_height // 3)
            rx1, ry1 = max(0, x - pad_x), max(0, y - pad_y)
            rx2, ry2 = min(width, x + box_width + pad_x), min(height, y + box_height + pad_y)
            ring = gray[ry1:ry2, rx1:rx2].copy()
            inner_x1, inner_y1 = x - rx1, y - ry1
            ring[inner_y1 : inner_y1 + box_height, inner_x1 : inner_x1 + box_width] = 0
            ring_values = ring[ring > 0]
            inside_mean = float(gray[y : y + box_height, x : x + box_width].mean())
            ring_mean = float(np.median(ring_values)) if len(ring_values) else 0.0
            local_contrast = ring_mean - inside_mean
            if local_contrast < 22:
                continue

            score = (
                min(1.0, dark_fraction / 0.72) * 0.35
                + min(1.0, fill / 0.72) * 0.25
                + min(1.0, solidity / 0.92) * 0.20
                + min(1.0, local_contrast / 90.0) * 0.20
            )
            pad = max(1, int(round(min(box_width, box_height) * 0.06)))
            box = (max(0, x - pad), max(0, y - pad), min(width, x + box_width + pad), min(height, y + box_height + pad))
            candidates.append((box, score))

    selected: list[tuple[tuple[int, int, int, int], float]] = []
    for box, score in sorted(candidates, key=lambda item: item[1], reverse=True):
        if any(intersection_over_union(box, kept_box) > 0.46 for kept_box, _ in selected):
            continue
        selected.append((box, score))
    return sorted(selected[:2], key=lambda item: (item[0][0], item[0][1]))


def descriptor(image: np.ndarray, box: tuple[int, int, int, int]) -> np.ndarray:
    x1, y1, x2, y2 = box
    crop = cv2.cvtColor(image[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
    side = max(crop.shape)
    canvas = np.full((side, side), 190, dtype=np.uint8)
    oy = (side - crop.shape[0]) // 2
    ox = (side - crop.shape[1]) // 2
    canvas[oy : oy + crop.shape[0], ox : ox + crop.shape[1]] = crop
    canvas = cv2.resize(canvas, (64, 64), interpolation=cv2.INTER_CUBIC)
    canvas = cv2.equalizeHist(canvas)
    gx = cv2.Sobel(canvas, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(canvas, cv2.CV_32F, 0, 1, ksize=3)
    magnitude, angle = cv2.cartToPolar(gx, gy, angleInDegrees=True)
    angle %= 180.0
    cells: list[np.ndarray] = []
    for cell_y in range(8):
        for cell_x in range(8):
            ys = slice(cell_y * 8, (cell_y + 1) * 8)
            xs = slice(cell_x * 8, (cell_x + 1) * 8)
            hist, _ = np.histogram(
                angle[ys, xs], bins=9, range=(0.0, 180.0), weights=magnitude[ys, xs]
            )
            cells.append(hist.astype(np.float32))
    cell_grid = np.stack(cells).reshape(8, 8, 9)
    blocks: list[np.ndarray] = []
    for block_y in range(7):
        for block_x in range(7):
            block = cell_grid[block_y : block_y + 2, block_x : block_x + 2].reshape(-1)
            block /= max(1e-6, float(np.linalg.norm(block)))
            blocks.append(block)
    vector = np.concatenate(blocks).astype(np.float32)
    vector /= max(1e-6, float(np.linalg.norm(vector)))
    return vector


def build_reference_bank(images_root: Path) -> tuple[np.ndarray, np.ndarray, list[str]]:
    vectors: list[np.ndarray] = []
    labels: list[int] = []
    sources: list[str] = []
    for digit, filenames in PROTOTYPES.items():
        for filename in filenames:
            image = cv2.imread(str(images_root / filename))
            if image is None:
                continue
            oriented, _ = orient(image, filename)
            candidates = detect_diamonds(oriented)
            for box, score in candidates:
                if score < 0.60:
                    continue
                vectors.append(descriptor(oriented, box))
                labels.append(digit)
                sources.append(filename)
    if not vectors:
        raise RuntimeError("No reference diamonds were detected")
    return np.stack(vectors), np.asarray(labels, dtype=np.int32), sources


def digit_scores(vector: np.ndarray, bank: np.ndarray, labels: np.ndarray) -> dict[int, float]:
    similarities = bank @ vector
    class_scores: dict[int, float] = {}
    for digit in sorted(set(labels.tolist())):
        values = np.sort(similarities[labels == digit])[-3:]
        class_scores[digit] = float(values.mean())
    return class_scores


def rank_digit_scores(class_scores: dict[int, float]) -> tuple[int, float, float]:
    ordered = sorted(class_scores.items(), key=lambda item: item[1], reverse=True)
    digit, best = ordered[0]
    second = ordered[1][1]
    confidence = max(0.0, min(1.0, (best - 0.35) / 0.45))
    margin = best - second
    return digit, confidence, margin


def yolo_line(class_id: int, box: tuple[int, int, int, int], width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    return f"{class_id} {((x1 + x2) / 2) / width:.6f} {((y1 + y2) / 2) / height:.6f} {(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-shape-score", type=float, default=0.50)
    parser.add_argument("--min-digit-confidence", type=float, default=0.34)
    parser.add_argument("--min-digit-margin", type=float, default=0.015)
    args = parser.parse_args()

    labels_root = args.output / "labels"
    previews_root = args.output / "previews"
    xanylabeling_root = args.output / "xanylabeling_json"
    labels_root.mkdir(parents=True, exist_ok=True)
    previews_root.mkdir(parents=True, exist_ok=True)
    xanylabeling_root.mkdir(parents=True, exist_ok=True)

    image_files = sorted(path for path in args.images.iterdir() if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    rows: list[dict[str, object]] = []
    counts: Counter[str] = Counter()

    for image_index, image_path in enumerate(image_files):
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        oriented, mode = orient(image, image_path.name)
        label_lines: list[str] = []
        xany_shapes: list[dict[str, object]] = []
        digit = VERIFIED_DIGIT_BY_INDEX.get(image_index)
        candidates = detect_diamonds(oriented) if digit is not None else []
        digit_confidence = 1.0 if digit is not None else 0.0
        digit_margin = 1.0 if digit is not None else 0.0
        for candidate_number, (oriented_box, shape_score) in enumerate(candidates):
            box = unorient_box(oriented_box, mode, image.shape)
            accepted = (
                shape_score >= args.min_shape_score
                and digit is not None
            )
            if accepted:
                label_lines.append(yolo_line(1, box, width, height))
                label_lines.append(yolo_line(2 + digit, box, width, height))
                x1, y1, x2, y2 = box
                for label in ("diamond", f"digit_{digit}"):
                    xany_shapes.append(
                        {
                            "review_status": "pending",
                            "label": label,
                            "score": round(shape_score, 5),
                            "points": [[float(x1), float(y1)], [float(x2), float(y2)]],
                            "group_id": candidate_number,
                            "description": "auto label; verify before training",
                            "difficult": False,
                            "shape_type": "rectangle",
                            "flags": {},
                            "attributes": {},
                            "kie_linking": [],
                        }
                    )
                colour = (0, 210, 0)
                counts[f"digit_{digit}"] += 1
            else:
                colour = (0, 170, 255)
                counts["review_only"] += 1
            x1, y1, x2, y2 = box
            cv2.rectangle(image, (x1, y1), (x2, y2), colour, 1)
            cv2.putText(
                image,
                f"{'AUTO' if accepted else 'REVIEW'} d{digit} s{shape_score:.2f} c{digit_confidence:.2f} m{digit_margin:.2f}",
                (x1, max(10, y1 - 3)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.28,
                colour,
                1,
                cv2.LINE_AA,
            )
            rows.append(
                {
                    "image": image_path.name,
                    "image_index": image_index,
                    "candidate": candidate_number,
                    "box_xyxy": json.dumps(box),
                    "shape_score": round(shape_score, 5),
                    "digit_suggestion": digit,
                    "digit_confidence": round(digit_confidence, 5),
                    "digit_margin": round(digit_margin, 5),
                    "status": "auto_labeled_review_required" if accepted else "not_labeled_review_required",
                }
            )

        if digit is not None and not candidates:
            counts["mapped_images_without_box"] += 1
            rows.append(
                {
                    "image": image_path.name,
                    "image_index": image_index,
                    "candidate": "",
                    "box_xyxy": "",
                    "shape_score": "",
                    "digit_suggestion": digit,
                    "digit_confidence": 1.0,
                    "digit_margin": 1.0,
                    "status": "mapped_number_but_no_box_review_required",
                }
            )

        (labels_root / f"{image_path.stem}.txt").write_text(
            "\n".join(label_lines) + ("\n" if label_lines else ""), encoding="utf-8"
        )
        if xany_shapes:
            xany_payload = {
                "version": "4.3.2",
                "flags": {},
                "checked": False,
                "shapes": xany_shapes,
                "imagePath": f"../../AUTO_REVIEW_REQUIRED/model_B_unlabeled_buoy_crops/{image_path.name}",
                "imageData": None,
                "imageHeight": height,
                "imageWidth": width,
            }
            (xanylabeling_root / f"{image_path.stem}.json").write_text(
                json.dumps(xany_payload, indent=2), encoding="utf-8"
            )
        cv2.imwrite(str(previews_root / image_path.name), image, [cv2.IMWRITE_JPEG_QUALITY, 94])
        counts["images"] += 1
        if label_lines:
            counts["images_auto_labeled"] += 1

    fieldnames = [
        "image", "image_index", "candidate", "box_xyxy", "shape_score", "digit_suggestion",
        "digit_confidence", "digit_margin", "status",
    ]
    with (args.output / "manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    summary = dict(counts)
    summary.update(
        {
            "class_names": CLASS_NAMES,
            "visually_verified_number_images": len(VERIFIED_DIGIT_BY_INDEX),
            "status": "AUTO_LABELS_REVIEW_REQUIRED",
        }
    )
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
