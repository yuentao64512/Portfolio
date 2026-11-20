#!/usr/bin/env python3
"""Create single-class YOLO segmentation labels with open-vocabulary YOLOE."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLOE


PROMPTS = [
    "air purifier", "air cleaner", "HEPA air purifier", "air cleaning appliance",
    "cylindrical appliance", "tower air purifier", "rectangular air purifier",
    "desktop air purifier", "wall-mounted air purifier",
]


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    root = Path(__file__).parent
    parser.add_argument("--input", type=Path, default=root / "raw")
    parser.add_argument("--output", type=Path, default=root / "dataset")
    parser.add_argument("--model", default=str(Path.home() / "yoloe-11s-seg.pt"))
    parser.add_argument("--confidence", type=float, default=0.025)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=4)
    parser.add_argument("--device", default="0")
    return parser.parse_args()


def split_for(name: str) -> str:
    value = int(hashlib.sha256(name.encode()).hexdigest()[:8], 16) % 100
    return "train" if value < 80 else "val" if value < 95 else "test"


def polygon_line(points: np.ndarray) -> str | None:
    if len(points) < 3:
        return None
    values = np.clip(points.reshape(-1), 0.0, 1.0)
    return "0 " + " ".join(f"{value:.6f}" for value in values)


def predictions(model: YOLOE, images: list[Path], args: argparse.Namespace):
    for start in range(0, len(images), args.batch):
        batch = images[start:start + args.batch]
        results = model.predict(
            source=[str(path) for path in batch], device=args.device,
            conf=args.confidence, iou=0.45, imgsz=args.imgsz, verbose=False,
        )
        yield from zip(batch, results)


def box_iou(first: np.ndarray, second: np.ndarray) -> float:
    left = max(float(first[0]), float(second[0]))
    top = max(float(first[1]), float(second[1]))
    right = min(float(first[2]), float(second[2]))
    bottom = min(float(first[3]), float(second[3]))
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, float(first[2] - first[0])) * max(0.0, float(first[3] - first[1]))
    second_area = max(0.0, float(second[2] - second[0])) * max(0.0, float(second[3] - second[1]))
    return intersection / max(first_area + second_area - intersection, 1e-9)


def main() -> int:
    args = arguments()
    images = sorted(args.input.glob("*.jpg"))
    if not images:
        raise SystemExit(f"No JPG images found in {args.input}")
    for split in ("train", "val", "test"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=True)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=True)
    review = args.output / "review"
    review.mkdir(parents=True, exist_ok=True)

    model = YOLOE(args.model)
    model.set_classes(PROMPTS, model.get_text_pe(PROMPTS))
    accepted = rejected = 0
    manifest = (args.output / "annotation_manifest.jsonl").open("w", encoding="utf-8")
    try:
        for source, result in predictions(model, images, args):
            record = {"source": source.name, "accepted": False}
            if result.boxes is None or result.masks is None or len(result.boxes) == 0:
                rejected += 1
                record["reason"] = "no_mask"
                manifest.write(json.dumps(record) + "\n")
                continue

            areas = result.boxes.xywhn[:, 2] * result.boxes.xywhn[:, 3]
            candidates = [
                index for index, area in enumerate(areas.tolist())
                if 0.015 <= area <= 0.97 and len(result.masks.xyn[index]) >= 3
            ]
            if not candidates:
                rejected += 1
                record["reason"] = "invalid_mask_area"
                manifest.write(json.dumps(record) + "\n")
                continue

            scores = {
                index: float(result.boxes.conf[index]) * float(areas[index]) ** 0.25
                for index in candidates
            }
            ordered = sorted(candidates, key=scores.get, reverse=True)
            minimum_score = scores[ordered[0]] * 0.20
            selected: list[int] = []
            for index in ordered:
                if scores[index] < minimum_score:
                    continue
                box = result.boxes.xyxyn[index].cpu().numpy()
                if all(box_iou(box, result.boxes.xyxyn[old].cpu().numpy()) < 0.45 for old in selected):
                    selected.append(index)

            lines = [polygon_line(result.masks.xyn[index]) for index in selected]
            lines = [line for line in lines if line is not None]
            if not lines:
                rejected += 1
                record["reason"] = "short_polygon"
                manifest.write(json.dumps(record) + "\n")
                continue

            split = split_for(source.name)
            target_image = args.output / "images" / split / source.name
            target_label = args.output / "labels" / split / f"{source.stem}.txt"
            shutil.copy2(source, target_image)
            target_label.write_text("\n".join(lines) + "\n", encoding="utf-8")

            canvas = cv2.imread(str(source))
            overlay = canvas.copy()
            polygons = [
                (result.masks.xyn[index] * [canvas.shape[1], canvas.shape[0]]).astype(np.int32)
                for index in selected
            ]
            cv2.fillPoly(overlay, polygons, (0, 200, 0))
            canvas = cv2.addWeighted(overlay, 0.35, canvas, 0.65, 0)
            cv2.polylines(canvas, polygons, True, (0, 255, 0), 2)
            cv2.imwrite(str(review / source.name), canvas)

            record.update({
                "accepted": True, "split": split,
                "instances": len(selected),
                "confidence": [float(result.boxes.conf[index]) for index in selected],
                "prompt": [PROMPTS[int(result.boxes.cls[index])] for index in selected],
                "box_area": [float(areas[index]) for index in selected],
            })
            manifest.write(json.dumps(record) + "\n")
            accepted += 1
            if (accepted + rejected) % 25 == 0:
                print(f"Processed {accepted + rejected}/{len(images)}: accepted {accepted}")
    finally:
        manifest.close()

    yaml = (
        f"path: {args.output.resolve()}\n"
        "train: images/train\nval: images/val\ntest: images/test\n"
        "names:\n  0: air_purifier\n"
    )
    (args.output / "air_purifier.yaml").write_text(yaml, encoding="utf-8")
    print(f"Annotation complete: accepted {accepted}, rejected {rejected}")
    print(f"Review overlays before training: {review}")
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
