#!/usr/bin/env python3
"""Create reviewable full-frame YOLO drafts from an unlabelled RobotX folder.

Class 0 is the whole buoy. Classes 1..10 are diamond boxes containing digits
0..9. Source files are read only; output is written to a new directory.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import subprocess
import tempfile
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
VIDEO_SUFFIXES = {".mov", ".mp4", ".avi", ".mkv", ".m4v"}


def safe_name(path: Path) -> str:
    return "_".join(path.stem.replace("__", "_").split())


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def iou(a, b) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, x2-x1) * max(0.0, y2-y1)
    union = max(1e-9, (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - intersection)
    return intersection / union


def classical_diamonds(image: np.ndarray):
    """Find dark, roughly diamond-shaped placards inside a buoy crop."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    short = max(1, min(gray.shape[:2]))
    kernel_size = max(3, int(round(short * 0.018)))
    if kernel_size % 2 == 0:
        kernel_size += 1
    mask = cv2.inRange(gray, 0, 165)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((kernel_size, kernel_size), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = image.shape[0] * image.shape[1]
    boxes = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if not 0.0025 * image_area <= area <= 0.25 * image_area:
            continue
        x, y, width, height = cv2.boundingRect(contour)
        if width < 10 or height < 10 or not 0.45 <= width / height <= 2.20:
            continue
        extent = area / max(1, width * height)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = area / max(1.0, hull_area)
        if extent < 0.28 or solidity < 0.55:
            continue
        perimeter = cv2.arcLength(contour, True)
        vertices = len(cv2.approxPolyDP(contour, 0.04 * perimeter, True))
        if not 4 <= vertices <= 10:
            continue
        boxes.append((float(x), float(y), float(x+width), float(y+height)))
    return boxes


def tesseract_digit(image: np.ndarray) -> tuple[int | None, float]:
    """Run three preprocess variants and return the strongest single digit."""
    if image.size == 0:
        return None, -1.0
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    gray = cv2.resize(gray, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)
    _, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # Isolate the light digit inside the largest dark diamond. White buoy body
    # outside the diamond otherwise confuses single-character OCR.
    dark = cv2.inRange(blur, 0, 175)
    dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    isolated = np.full_like(gray, 255)
    if contours:
        contour = max(contours, key=cv2.contourArea)
        mask = np.zeros_like(gray)
        cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
        mask = cv2.erode(mask, np.ones((5, 5), np.uint8))
        threshold = max(145, int(np.percentile(gray[mask > 0], 65))) if np.any(mask > 0) else 175
        digit_pixels = (gray >= threshold) & (mask > 0)
        isolated[digit_pixels] = 0
    variants = (isolated, gray, binary, cv2.bitwise_not(binary))
    best_digit, best_conf = None, -1.0
    for variant in variants:
        ok, encoded = cv2.imencode(".png", variant)
        if not ok:
            continue
        completed = subprocess.run(
            ["tesseract", "stdin", "stdout", "--psm", "10", "-c",
             "tessedit_char_whitelist=0123456789", "tsv"],
            input=encoded.tobytes(), stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL, check=False, timeout=8,
        )
        if completed.returncode:
            continue
        text = completed.stdout.decode("utf-8", errors="replace")
        for row in csv.DictReader(text.splitlines(), delimiter="\t"):
            value = (row.get("text") or "").strip()
            if len(value) != 1 or not value.isdigit():
                continue
            try:
                confidence = float(row.get("conf", -1))
            except ValueError:
                continue
            if confidence > best_conf:
                best_digit, best_conf = int(value), confidence
    return best_digit, best_conf


def write_yolo(path: Path, labels, width: int, height: int):
    rows = []
    for class_id, (x1, y1, x2, y2) in labels:
        x1, y1 = max(0.0, x1), max(0.0, y1)
        x2, y2 = min(float(width), x2), min(float(height), y2)
        if x2 <= x1 or y2 <= y1:
            continue
        rows.append(
            f"{class_id} {(x1+x2)/2/width:.6f} {(y1+y2)/2/height:.6f} "
            f"{(x2-x1)/width:.6f} {(y2-y1)/height:.6f}"
        )
    path.write_text("\n".join(rows) + ("\n" if rows else ""))


def extract_inputs(source: Path, output: Path, fps: float, limit: int | None):
    images_dir = output / "images" / "train"
    images_dir.mkdir(parents=True)
    manifest = []
    source_hashes = {}
    sources = sorted(path for path in source.iterdir() if path.is_file())
    for source_path in sources:
        suffix = source_path.suffix.lower()
        if suffix in IMAGE_SUFFIXES:
            name = f"img-{safe_name(source_path)}__00000000.jpg"
            image = cv2.imread(str(source_path))
            if image is None:
                continue
            cv2.imwrite(str(images_dir / name), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            if source_path not in source_hashes:
                source_hashes[source_path] = sha256(source_path)
            manifest.append({"source": str(source_path), "output": name, "sha256": source_hashes[source_path]})
        elif suffix in VIDEO_SUFFIXES:
            capture = cv2.VideoCapture(str(source_path))
            source_fps = capture.get(cv2.CAP_PROP_FPS)
            if not capture.isOpened() or source_fps <= 0:
                capture.release()
                continue
            step = max(1, round(source_fps / fps))
            frame_index = 0
            while True:
                ok, frame = capture.read()
                if not ok:
                    break
                if frame_index % step == 0:
                    name = f"vid-{safe_name(source_path)}__{frame_index:08d}.jpg"
                    cv2.imwrite(str(images_dir / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    if source_path not in source_hashes:
                        source_hashes[source_path] = sha256(source_path)
                    manifest.append({"source": str(source_path), "output": name, "sha256": source_hashes[source_path]})
                frame_index += 1
            capture.release()
        if limit is not None and len(manifest) >= limit:
            break
    if limit is not None and len(manifest) > limit:
        for entry in manifest[limit:]:
            (images_dir / entry["output"]).unlink(missing_ok=True)
        manifest = manifest[:limit]
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--buoy-weights", type=Path, required=True)
    parser.add_argument("--shape-weights", type=Path, required=True)
    parser.add_argument("--digit-weights", type=Path, help="optional 0-9 detector used separately from shape model")
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--buoy-conf", type=float, default=0.25)
    parser.add_argument("--diamond-conf", type=float, default=0.20)
    parser.add_argument("--digit-conf", type=float, default=0.20)
    parser.add_argument("--ocr-conf", type=float, default=35.0)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--debug-dir", type=Path, help="optional directory for detected diamond crops")
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite {args.output}")
    labels_dir = args.output / "labels" / "train"
    labels_dir.mkdir(parents=True)
    if args.debug_dir:
        args.debug_dir.mkdir(parents=True, exist_ok=True)
    manifest = extract_inputs(args.source, args.output, args.fps, args.limit)

    buoy_model = YOLO(str(args.buoy_weights), task="detect")
    shape_model = YOLO(str(args.shape_weights), task="detect")
    digit_model = YOLO(str(args.digit_weights), task="detect") if args.digit_weights else None
    counts = Counter()
    methods = Counter()
    image_paths = sorted((args.output / "images" / "train").glob("*.jpg"))
    for image_path in image_paths:
        image = cv2.imread(str(image_path))
        height, width = image.shape[:2]
        full_labels = []
        buoy_result = buoy_model.predict(image, imgsz=960, conf=args.buoy_conf, verbose=False)[0]
        buoy_boxes = [] if buoy_result.boxes is None else [tuple(map(float, box.xyxy[0].tolist())) for box in buoy_result.boxes]
        counts["images"] += 1
        counts["buoys"] += len(buoy_boxes)
        for buoy_box in buoy_boxes:
            x1, y1, x2, y2 = buoy_box
            full_labels.append((0, buoy_box))
            px, py = 0.10*(x2-x1), 0.10*(y2-y1)
            ox, oy = max(0, int(x1-px)), max(0, int(y1-py))
            ex, ey = min(width, int(x2+px)), min(height, int(y2+py))
            crop = image[oy:ey, ox:ex]
            if crop.size == 0:
                continue
            result = shape_model.predict(crop, imgsz=1280, conf=min(args.diamond_conf, args.digit_conf), verbose=False)[0]
            detections = []
            if result.boxes is not None:
                for box in result.boxes:
                    detections.append((int(box.cls.item()), float(box.conf.item()), tuple(map(float, box.xyxy[0].tolist()))))
            diamonds = [(conf, box) for cls, conf, box in detections if cls == 1 and conf >= args.diamond_conf]
            for candidate in classical_diamonds(crop):
                if all(iou(candidate, existing) < 0.35 for _, existing in diamonds):
                    diamonds.append((0.0, candidate))
                    counts["classical_diamonds"] += 1
            if digit_model is not None:
                digit_result = digit_model.predict(crop, imgsz=320, conf=args.digit_conf, verbose=False)[0]
                direct = [] if digit_result.boxes is None else [
                    (int(box.cls.item()), float(box.conf.item()), tuple(map(float, box.xyxy[0].tolist())))
                    for box in digit_result.boxes
                ]
            else:
                direct = [(cls-2, conf, box) for cls, conf, box in detections if 2 <= cls <= 11 and conf >= args.digit_conf]
            counts["direct_candidates"] += len(direct)
            for direct_digit, direct_confidence, _ in direct:
                counts[f"direct_candidate_{direct_digit}"] += 1
                for threshold in (0.10, 0.20, 0.30, 0.40, 0.50):
                    if direct_confidence >= threshold:
                        counts[f"direct_ge_{threshold:.1f}"] += 1
            used_direct = set()
            pending_ocr = []
            for diamond_index, (diamond_conf, diamond) in enumerate(diamonds):
                candidates = []
                for index, (direct_digit, direct_conf, direct_box) in enumerate(direct):
                    center_x = (direct_box[0] + direct_box[2]) / 2
                    center_y = (direct_box[1] + direct_box[3]) / 2
                    overlap = iou(diamond, direct_box)
                    if diamond[0] <= center_x <= diamond[2] and diamond[1] <= center_y <= diamond[3]:
                        overlap = max(overlap, 0.50)
                    candidates.append((overlap, direct_digit, direct_conf, direct_box, index))
                candidates = [candidate for candidate in candidates if candidate[0] > 0.20]
                model_choice = max(candidates, default=None)
                a, b, c, d = map(int, diamond)
                pad = max(2, int(0.08 * max(c-a, d-b)))
                diamond_crop = crop[max(0, b-pad):min(crop.shape[0], d+pad), max(0, a-pad):min(crop.shape[1], c+pad)]
                ocr_digit, ocr_conf = tesseract_digit(diamond_crop)
                if args.debug_dir:
                    debug_name = f"{image_path.stem}_b{len(full_labels):03d}_d{diamond_index:02d}_ocr{ocr_digit}_c{ocr_conf:.0f}.jpg"
                    cv2.imwrite(str(args.debug_dir / debug_name), diamond_crop)
                digit = None
                if model_choice is not None and model_choice[2] >= 0.55:
                    digit = model_choice[1]
                    used_direct.add(model_choice[4])
                    methods["model_high"] += 1
                elif model_choice is not None and ocr_digit == model_choice[1] and ocr_conf >= args.ocr_conf:
                    digit = ocr_digit
                    used_direct.add(model_choice[4])
                    methods["agreement"] += 1
                elif ocr_digit is not None and ocr_conf >= max(55.0, args.ocr_conf):
                    digit = ocr_digit
                    methods["ocr_high"] += 1
                if digit is None:
                    counts["uncertain_diamonds"] += 1
                    if ocr_digit is not None and ocr_conf >= 0:
                        pending_ocr.append((ocr_digit, ocr_conf, diamond))
                    continue
                global_box = (ox+diamond[0], oy+diamond[1], ox+diamond[2], oy+diamond[3])
                full_labels.append((digit+1, global_box))
                counts[f"digit_{digit}"] += 1
            if pending_ocr:
                ocr_votes = Counter(record[0] for record in pending_ocr)
                consensus_digit, consensus_count = ocr_votes.most_common(1)[0]
                matching = [record for record in pending_ocr if record[0] == consensus_digit]
                if consensus_count >= 2 and max(record[1] for record in matching) >= 10:
                    for _, _, diamond in matching:
                        global_box = (ox+diamond[0], oy+diamond[1], ox+diamond[2], oy+diamond[3])
                        full_labels.append((consensus_digit+1, global_box))
                        counts[f"digit_{consensus_digit}"] += 1
                        counts["uncertain_diamonds"] -= 1
                        methods["ocr_two_face_consensus"] += 1
            for index, (digit, conf, box) in enumerate(direct):
                if index in used_direct or conf < 0.70:
                    continue
                global_box = (ox+box[0], oy+box[1], ox+box[2], oy+box[3])
                full_labels.append((digit+1, global_box))
                counts[f"digit_{digit}"] += 1
                methods["direct_without_diamond"] += 1
        write_yolo(labels_dir / f"{image_path.stem}.txt", full_labels, width, height)

    report = {
        "source": str(args.source), "output": str(args.output),
        "counts": dict(counts), "digit_methods": dict(methods),
        "provisional_pseudo_labels": True,
        "warning": "Human review is required. Do not use these labels as test ground truth.",
        "manifest": manifest,
    }
    (args.output / "pseudolabel_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({key: value for key, value in report.items() if key != "manifest"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
