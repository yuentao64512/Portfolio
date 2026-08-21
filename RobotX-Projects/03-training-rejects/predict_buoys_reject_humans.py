from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
VIDEO_EXTENSIONS = {".mp4", ".mov", ".avi", ".mkv", ".m4v", ".wmv"}
COLORS = {
    0: (255, 170, 30),  # blue_marker
    1: (40, 40, 255),   # red_marker
    2: (40, 220, 40),   # green_marker
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Keep buoy detections and reject marker boxes that lie on a detected human."
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--source", required=True, help="Image, video, or folder to process.")
    parser.add_argument(
        "--marker-model",
        default=str(ROOT / "runs" / "detect" / "train-19" / "weights" / "best.pt"),
    )
    parser.add_argument(
        "--human-model",
        default=str(ROOT / "runs" / "human_reject_yolo11l_fullframe" / "weights" / "best.pt"),
    )
    parser.add_argument(
        "--output",
        default=str(ROOT / "predictions_buoys_no_humans"),
    )
    parser.add_argument("--marker-conf", type=float, default=0.15)
    parser.add_argument("--human-conf", type=float, default=0.20)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument(
        "--marker-overlap",
        type=float,
        default=0.20,
        help="Reject when this fraction of a marker box is covered by a human box.",
    )
    parser.add_argument(
        "--human-padding",
        type=float,
        default=0.08,
        help="Expand human boxes by this fraction on each side before rejection.",
    )
    parser.add_argument("--debug", action="store_true", help="Draw humans and rejected marker boxes.")
    return parser.parse_args()


def xyxy(box) -> list[float]:
    return [float(value) for value in box.xyxy[0].tolist()]


def expand_box(box: list[float], width: int, height: int, fraction: float) -> list[float]:
    x1, y1, x2, y2 = box
    pad_x = (x2 - x1) * fraction
    pad_y = (y2 - y1) * fraction
    return [
        max(0.0, x1 - pad_x),
        max(0.0, y1 - pad_y),
        min(float(width - 1), x2 + pad_x),
        min(float(height - 1), y2 + pad_y),
    ]


def covered_fraction(marker: list[float], human: list[float]) -> float:
    mx1, my1, mx2, my2 = marker
    hx1, hy1, hx2, hy2 = human
    intersection_width = max(0.0, min(mx2, hx2) - max(mx1, hx1))
    intersection_height = max(0.0, min(my2, hy2) - max(my1, hy1))
    intersection = intersection_width * intersection_height
    marker_area = max(0.0, mx2 - mx1) * max(0.0, my2 - my1)
    return intersection / marker_area if marker_area else 0.0


def center_inside(marker: list[float], human: list[float]) -> bool:
    x1, y1, x2, y2 = marker
    hx1, hy1, hx2, hy2 = human
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    return hx1 <= center_x <= hx2 and hy1 <= center_y <= hy2


def draw_box(image, box: list[float], text: str, color: tuple[int, int, int], thickness: int = 2) -> None:
    x1, y1, x2, y2 = (int(round(value)) for value in box)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
    cv2.putText(
        image,
        text,
        (x1, max(24, y1 - 7)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        color,
        2,
        cv2.LINE_AA,
    )


def process_frame(
    frame,
    marker_model: YOLO,
    human_model: YOLO,
    args: argparse.Namespace,
) -> tuple[object, list[tuple], int, int]:
    height, width = frame.shape[:2]
    marker_result = marker_model.predict(
        frame,
        conf=args.marker_conf,
        imgsz=args.imgsz,
        verbose=False,
    )[0]
    human_result = human_model.predict(
        frame,
        classes=[0],
        conf=args.human_conf,
        imgsz=args.imgsz,
        verbose=False,
    )[0]

    human_boxes = []
    if human_result.boxes is not None:
        human_boxes = [
            (
                expand_box(xyxy(box), width, height, args.human_padding),
                float(box.conf.item()),
            )
            for box in human_result.boxes
        ]

    output = frame.copy()
    rows: list[tuple] = []
    kept = 0
    rejected = 0

    if marker_result.boxes is not None:
        for box in marker_result.boxes:
            class_id = int(box.cls.item())
            if class_id not in (0, 1, 2):
                continue
            marker_box = xyxy(box)
            marker_conf = float(box.conf.item())
            reject = any(
                center_inside(marker_box, human_box)
                or covered_fraction(marker_box, human_box) >= args.marker_overlap
                for human_box, _ in human_boxes
            )
            class_name = marker_model.names.get(class_id, str(class_id))
            rows.append((class_id, class_name, marker_conf, *marker_box, int(reject)))

            if reject:
                rejected += 1
                if args.debug:
                    draw_box(output, marker_box, f"REJECT {class_name}", (0, 0, 255))
            else:
                kept += 1
                draw_box(
                    output,
                    marker_box,
                    f"{class_name} {marker_conf:.2f}",
                    COLORS.get(class_id, (255, 255, 0)),
                )

    if args.debug:
        for human_box, confidence in human_boxes:
            draw_box(output, human_box, f"human {confidence:.2f}", (255, 0, 255), 1)

    return output, rows, kept, rejected


def media_paths(source: Path) -> list[Path]:
    if source.is_file():
        return [source]
    if source.is_dir():
        allowed = IMAGE_EXTENSIONS | VIDEO_EXTENSIONS
        return sorted(path for path in source.rglob("*") if path.is_file() and path.suffix.lower() in allowed)
    raise FileNotFoundError(f"Prediction source does not exist: {source}")


def process_video(
    source: Path,
    output_dir: Path,
    marker_model: YOLO,
    human_model: YOLO,
    args: argparse.Namespace,
    csv_writer: csv.writer,
) -> tuple[int, int, int]:
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"Could not open video: {source}")

    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    output_path = output_dir / f"{source.stem}_buoys_no_humans.mp4"
    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (width, height),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not create output video: {output_path}")

    frame_index = 0
    kept_total = 0
    rejected_total = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        rendered, rows, kept, rejected = process_frame(frame, marker_model, human_model, args)
        writer.write(rendered)
        for row in rows:
            csv_writer.writerow((source.name, frame_index, *row))
        kept_total += kept
        rejected_total += rejected
        frame_index += 1
        if frame_index % 25 == 0:
            print(
                f"{source.name}: {frame_index} frames, "
                f"{kept_total} buoys kept, {rejected_total} human false positives rejected",
                flush=True,
            )

    capture.release()
    writer.release()
    print(f"Saved: {output_path}")
    return frame_index, kept_total, rejected_total


def process_image(
    source: Path,
    output_dir: Path,
    marker_model: YOLO,
    human_model: YOLO,
    args: argparse.Namespace,
    csv_writer: csv.writer,
) -> tuple[int, int, int]:
    image = cv2.imread(str(source))
    if image is None:
        raise RuntimeError(f"Could not read image: {source}")
    rendered, rows, kept, rejected = process_frame(image, marker_model, human_model, args)
    output_path = output_dir / source.name
    cv2.imwrite(str(output_path), rendered)
    for row in rows:
        csv_writer.writerow((source.name, 0, *row))
    return 1, kept, rejected


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    source = Path(args.source)
    if not source.is_absolute():
        source = root / source
    output_dir = Path(args.output)
    if not output_dir.is_absolute():
        output_dir = root / output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    marker_path = Path(args.marker_model)
    human_path = Path(args.human_model)
    if not marker_path.is_absolute():
        marker_path = root / marker_path
    if not human_path.is_absolute():
        human_path = root / human_path
    if not marker_path.is_file():
        raise FileNotFoundError(f"Marker model not found: {marker_path}")
    if not human_path.is_file():
        raise FileNotFoundError(f"Human reject model not found: {human_path}")

    print(f"Marker model: {marker_path}")
    print(f"Human model: {human_path}")
    marker_model = YOLO(str(marker_path))
    human_model = YOLO(str(human_path))
    totals = [0, 0, 0]

    csv_path = output_dir / "detections.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        csv_writer = csv.writer(handle)
        csv_writer.writerow(
            (
                "source",
                "frame",
                "class_id",
                "class_name",
                "confidence",
                "x1",
                "y1",
                "x2",
                "y2",
                "rejected_as_human",
            )
        )
        for path in media_paths(source):
            if path.suffix.lower() in VIDEO_EXTENSIONS:
                counts = process_video(path, output_dir, marker_model, human_model, args, csv_writer)
            else:
                counts = process_image(path, output_dir, marker_model, human_model, args, csv_writer)
            totals = [left + right for left, right in zip(totals, counts)]

    print("")
    print(f"Processed frames/images: {totals[0]}")
    print(f"Buoy boxes kept: {totals[1]}")
    print(f"Marker false positives rejected on humans: {totals[2]}")
    print(f"Output folder: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
