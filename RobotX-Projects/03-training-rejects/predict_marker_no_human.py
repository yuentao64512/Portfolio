from __future__ import annotations

import argparse
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
VIDEO_EXTENSIONS = {".mp4", ".mov", ".avi", ".mkv", ".m4v", ".wmv"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the good marker model, run a human/person model, then remove marker "
            "detections that overlap humans. This combines the models at inference time."
        )
    )
    parser.add_argument(
        "--source",
        default=str(ROOT / "predict_source"),
        help="Image/video/folder to predict on.",
    )
    parser.add_argument(
        "--marker-model",
        default=str(ROOT / "runs" / "buoy_hard_negative_v1" / "weights" / "best.pt"),
        help="Your good 3-class marker detector.",
    )
    parser.add_argument(
        "--human-model",
        default=str(ROOT / "yolo11l.pt"),
        help="Human/person detector weights. Can also be yolo11m.pt if you want COCO person detection.",
    )
    parser.add_argument(
        "--output",
        default=str(ROOT / "predictions_marker_human_reject_coco"),
        help="Folder for filtered prediction images.",
    )
    parser.add_argument("--marker-conf", type=float, default=0.15)
    parser.add_argument("--human-conf", type=float, default=0.10)
    parser.add_argument("--iou-threshold", type=float, default=0.10)
    parser.add_argument(
        "--human-class",
        type=int,
        default=0,
        help="Human class id. Leave blank for all classes from the human model; use 0 for COCO person.",
    )
    return parser.parse_args()


def iou(a: list[float], b: list[float]) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = area_a + area_b - inter
    return inter / union if union else 0.0


def center_inside(inner: list[float], outer: list[float]) -> bool:
    x1, y1, x2, y2 = inner
    ox1, oy1, ox2, oy2 = outer
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    margin_x = (ox2 - ox1) * 0.15
    margin_y = (oy2 - oy1) * 0.15
    return ox1 - margin_x <= cx <= ox2 + margin_x and oy1 - margin_y <= cy <= oy2 + margin_y


def result_boxes(result, human_class: int | None = None) -> list[dict]:
    boxes: list[dict] = []
    if result.boxes is None:
        return boxes

    for box in result.boxes:
        cls = int(box.cls.item())
        if human_class is not None and cls != human_class:
            continue
        boxes.append(
            {
                "xyxy": [float(v) for v in box.xyxy[0].tolist()],
                "conf": float(box.conf.item()),
                "cls": cls,
            }
        )
    return boxes


def draw_box(image, xyxy: list[float], label: str, color: tuple[int, int, int]) -> None:
    x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        image,
        label,
        (x1, max(20, y1 - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        color,
        2,
        cv2.LINE_AA,
    )


def main() -> int:
    args = parse_args()
    source = Path(args.source)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)

    marker_model = YOLO(args.marker_model)
    human_model = YOLO(args.human_model)
    marker_names = marker_model.names

    marker_results = marker_model.predict(
        source=str(source),
        conf=args.marker_conf,
        imgsz=960,
        stream=True,
        verbose=False,
    )

    kept_total = 0
    rejected_total = 0
    human_total = 0

    for frame_index, marker_result in enumerate(marker_results):
        image_path = Path(marker_result.path)
        human_result = human_model.predict(
            source=marker_result.orig_img,
            conf=args.human_conf,
            imgsz=1280,
            verbose=False,
        )[0]

        marker_boxes = result_boxes(marker_result)
        human_boxes = result_boxes(human_result, args.human_class)
        human_total += len(human_boxes)
        image = marker_result.orig_img.copy()

        for marker in marker_boxes:
            marker_xyxy = marker["xyxy"]
            reject = any(
                iou(marker_xyxy, human["xyxy"]) >= args.iou_threshold
                or center_inside(marker_xyxy, human["xyxy"])
                for human in human_boxes
            )

            if reject:
                rejected_total += 1
                continue

            kept_total += 1
            name = marker_names.get(marker["cls"], str(marker["cls"]))
            draw_box(image, marker_xyxy, f"{name} {marker['conf']:.2f}", (255, 80, 40))

        for human in human_boxes:
            draw_box(image, human["xyxy"], f"human {human['conf']:.2f}", (80, 80, 255))

        if image_path.suffix.lower() in VIDEO_EXTENSIONS:
            output_name = f"{image_path.stem}_frame_{frame_index:06d}.jpg"
        else:
            output_name = image_path.name
        cv2.imwrite(str(output / output_name), image)

        if frame_index % 25 == 0:
            print(
                f"Processed {frame_index + 1} frame(s), "
                f"kept {kept_total}, rejected {rejected_total}, humans {human_total}",
                flush=True,
            )

    print(f"Kept marker boxes: {kept_total}")
    print(f"Rejected marker boxes on humans: {rejected_total}")
    print(f"Output folder: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
