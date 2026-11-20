from __future__ import annotations

import argparse
import random
import shutil
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO


CLASSES = ["upright", "down"]


@dataclass(frozen=True)
class ClipSpec:
    filename: str
    class_id: int


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--model", default="yolo11n.pt")
    parser.add_argument("--sample-fps", type=float, default=4.0)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.15)
    parser.add_argument("--val-ratio", type=float, default=0.2)
    parser.add_argument("--seed", type=int, default=7)
    return parser.parse_args()


def xyxy_to_yolo(box: np.ndarray, width: int, height: int) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = box.astype(float)
    x1 = max(0.0, min(float(width - 1), x1))
    x2 = max(0.0, min(float(width - 1), x2))
    y1 = max(0.0, min(float(height - 1), y1))
    y2 = max(0.0, min(float(height - 1), y2))
    bw = max(1.0, x2 - x1)
    bh = max(1.0, y2 - y1)
    cx = x1 + bw / 2.0
    cy = y1 + bh / 2.0
    return cx / width, cy / height, bw / width, bh / height


def write_label(label_path: Path, class_id: int, box: np.ndarray, width: int, height: int) -> None:
    cx, cy, bw, bh = xyxy_to_yolo(box, width, height)
    label_path.write_text(f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}\n", encoding="utf-8")


def extract_and_autolabel(
    model: YOLO,
    clip: ClipSpec,
    raw_dir: Path,
    frame_dir: Path,
    label_dir: Path,
    sample_fps: float,
    imgsz: int,
    conf: float,
) -> list[Path]:
    video_path = raw_dir / clip.filename
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {video_path}")
    src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    stride = max(1, round(src_fps / sample_fps))

    saved: list[Path] = []
    frame_index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_index % stride != 0:
            frame_index += 1
            continue

        height, width = frame.shape[:2]
        results = model.predict(frame, imgsz=imgsz, conf=conf, classes=[0], verbose=False)
        boxes = results[0].boxes
        if boxes is None or len(boxes) == 0:
            frame_index += 1
            continue

        confs = boxes.conf.detach().cpu().numpy()
        xyxy = boxes.xyxy.detach().cpu().numpy()
        best_idx = int(np.argmax(confs))
        best_box = xyxy[best_idx]

        stem = f"{Path(clip.filename).stem}_f{frame_index:06d}"
        image_path = frame_dir / f"{stem}.jpg"
        label_path = label_dir / f"{stem}.txt"
        cv2.imwrite(str(image_path), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        write_label(label_path, clip.class_id, best_box, width, height)
        saved.append(image_path)
        frame_index += 1

    cap.release()
    return saved


def split_dataset(image_paths: list[Path], out_dir: Path, val_ratio: float, seed: int) -> None:
    rng = random.Random(seed)
    by_class: dict[int, list[Path]] = {0: [], 1: []}
    label_staging = out_dir / "_labels_staging"
    for image_path in image_paths:
        label_path = label_staging / f"{image_path.stem}.txt"
        class_id = int(label_path.read_text(encoding="utf-8").split()[0])
        by_class[class_id].append(image_path)

    for split in ("train", "val"):
        (out_dir / "images" / split).mkdir(parents=True, exist_ok=True)
        (out_dir / "labels" / split).mkdir(parents=True, exist_ok=True)

    for class_id, paths in by_class.items():
        rng.shuffle(paths)
        val_count = max(1, int(round(len(paths) * val_ratio))) if len(paths) > 1 else 0
        val_set = set(paths[:val_count])
        for image_path in paths:
            split = "val" if image_path in val_set else "train"
            shutil.copy2(image_path, out_dir / "images" / split / image_path.name)
            shutil.copy2(label_staging / f"{image_path.stem}.txt", out_dir / "labels" / split / f"{image_path.stem}.txt")


def make_preview(out_dir: Path, max_images: int = 24) -> None:
    images = sorted((out_dir / "images" / "train").glob("*.jpg"))[:max_images]
    thumbs = []
    for image_path in images:
        label_path = out_dir / "labels" / "train" / f"{image_path.stem}.txt"
        image = cv2.imread(str(image_path))
        if image is None or not label_path.exists():
            continue
        h, w = image.shape[:2]
        parts = label_path.read_text(encoding="utf-8").split()
        class_id = int(parts[0])
        cx, cy, bw, bh = map(float, parts[1:])
        x1 = int((cx - bw / 2) * w)
        y1 = int((cy - bh / 2) * h)
        x2 = int((cx + bw / 2) * w)
        y2 = int((cy + bh / 2) * h)
        color = (255, 255, 255) if class_id == 0 else (0, 255, 255)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 3)
        cv2.putText(image, CLASSES[class_id], (max(0, x1), max(24, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        thumbs.append(cv2.resize(image, (240, 135)))
    if not thumbs:
        return
    rows = []
    for i in range(0, len(thumbs), 4):
        row = thumbs[i : i + 4]
        if len(row) < 4:
            row.extend([np.zeros_like(row[0])] * (4 - len(row)))
        rows.append(np.hstack(row))
    cv2.imwrite(str(out_dir / "preview_autolabels.jpg"), np.vstack(rows))


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    staging = args.out_dir / "_frames_staging"
    label_staging = args.out_dir / "_labels_staging"
    for folder in (staging, label_staging):
        if folder.exists():
            shutil.rmtree(folder)
        folder.mkdir(parents=True, exist_ok=True)

    for folder in ("images", "labels"):
        target = args.out_dir / folder
        if target.exists():
            shutil.rmtree(target)

    clips = [
        ClipSpec("DJI_20260824101545_0025_D.MP4", 0),
        ClipSpec("DJI_20260824101609_0026_D.MP4", 1),
    ]
    model = YOLO(args.model)
    all_images: list[Path] = []
    for clip in clips:
        all_images.extend(
            extract_and_autolabel(
                model=model,
                clip=clip,
                raw_dir=args.raw_dir,
                frame_dir=staging,
                label_dir=label_staging,
                sample_fps=args.sample_fps,
                imgsz=args.imgsz,
                conf=args.conf,
            )
        )

    split_dataset(all_images, args.out_dir, args.val_ratio, args.seed)
    data = {
        "path": str(args.out_dir.resolve()),
        "train": "images/train",
        "val": "images/val",
        "names": {idx: name for idx, name in enumerate(CLASSES)},
    }
    (args.out_dir / "data.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    make_preview(args.out_dir)

    counts = {}
    for split in ("train", "val"):
        split_counts = {name: 0 for name in CLASSES}
        for label_path in (args.out_dir / "labels" / split).glob("*.txt"):
            class_id = int(label_path.read_text(encoding="utf-8").split()[0])
            split_counts[CLASSES[class_id]] += 1
        counts[split] = split_counts
    print({"total_images": len(all_images), "counts": counts, "data": str(args.out_dir / "data.yaml")})


if __name__ == "__main__":
    main()
