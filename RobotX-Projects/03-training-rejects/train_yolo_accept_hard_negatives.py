from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
MARKER_NAMES = ("blue_marker", "red_marker", "green_marker")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
VIDEO_EXTENSIONS = {".mp4", ".mov", ".avi", ".mkv", ".m4v"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Add hard negatives to a 3-class marker dataset, then train YOLO. "
            "Hard negatives are images/video frames with matching empty label files."
        )
    )
    parser.add_argument(
        "--dataset",
        default=str(ROOT / "yolo_train"),
        help="YOLO dataset root. Must contain images/train, images/val, labels/train, labels/val.",
    )
    parser.add_argument(
        "--negative-source",
        default=str(ROOT / "hard_negatives"),
        help="Folder/file containing false-positive examples. Images are copied; videos are sampled.",
    )
    parser.add_argument("--val-fraction", type=float, default=0.2)
    parser.add_argument("--video-fps", type=float, default=1.0)
    parser.add_argument("--epochs", type=int, default=120)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--batch", type=int, default=1)
    parser.add_argument("--run-name", default="dji_marker_hard_negative_marker_only")
    parser.add_argument(
        "--start-from",
        default="auto",
        choices=("auto", "base", "clean-best"),
        help="auto uses the clean marker best.pt when present, otherwise yolo11m.pt.",
    )
    parser.add_argument("--skip-train", action="store_true")
    return parser.parse_args()


def ensure_dataset(dataset: Path) -> None:
    for rel in ("images/train", "images/val", "labels/train", "labels/val"):
        (dataset / rel).mkdir(parents=True, exist_ok=True)

    (dataset / "classes.txt").write_text("\n".join(MARKER_NAMES) + "\n", encoding="utf-8")
    yaml_text = "\n".join(
        (
            f"path: {dataset.as_posix()}",
            "train: images/train",
            "val: images/val",
            "nc: 3",
            "names:",
            "  0: blue_marker",
            "  1: red_marker",
            "  2: green_marker",
            "",
        )
    )
    (dataset / "data.yaml").write_text(yaml_text, encoding="utf-8")


def validate_marker_labels(dataset: Path) -> None:
    bad: list[str] = []
    for label in sorted((dataset / "labels").rglob("*.txt")):
        for line_no, line in enumerate(label.read_text(encoding="utf-8").splitlines(), start=1):
            stripped = line.strip()
            if not stripped:
                continue
            first = stripped.split(maxsplit=1)[0]
            try:
                class_id = int(float(first))
            except ValueError:
                bad.append(f"{label}:{line_no} invalid class id {first!r}")
                continue
            if class_id >= len(MARKER_NAMES):
                bad.append(f"{label}:{line_no} has class {class_id}; expected only 0, 1, 2")

    if bad:
        details = "\n".join(bad[:20])
        raise RuntimeError(
            "Found labels outside the 3 marker classes. "
            "Remove/convert those labels before training.\n" + details
        )


def choose_split(key: str, val_fraction: float) -> str:
    val_fraction = max(0.0, min(1.0, val_fraction))
    bucket = int(hashlib.sha1(key.encode("utf-8")).hexdigest()[:8], 16) / 0xFFFFFFFF
    return "val" if bucket < val_fraction else "train"


def unique_path(folder: Path, stem: str, suffix: str) -> Path:
    candidate = folder / f"{stem}{suffix}"
    counter = 1
    while candidate.exists():
        candidate = folder / f"{stem}_{counter}{suffix}"
        counter += 1
    return candidate


def copy_hard_negative_image(dataset: Path, source: Path, val_fraction: float) -> Path:
    digest = hashlib.sha1(source.read_bytes()).hexdigest()[:12]
    split = choose_split(str(source.resolve()) + digest, val_fraction)
    image_dir = dataset / "images" / split
    label_dir = dataset / "labels" / split
    image_path = unique_path(image_dir, f"hardneg_{source.stem}_{digest}", source.suffix.lower())
    shutil.copy2(source, image_path)
    (label_dir / f"{image_path.stem}.txt").write_text("", encoding="utf-8")
    return image_path


def add_video_hard_negatives(dataset: Path, source: Path, val_fraction: float, fps: float) -> list[Path]:
    import cv2

    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {source}")

    source_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, round(source_fps / max(fps, 0.01)))
    added: list[Path] = []
    frame_no = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame_no % step == 0:
            key = f"{source.resolve()}:{frame_no}"
            split = choose_split(key, val_fraction)
            image_dir = dataset / "images" / split
            label_dir = dataset / "labels" / split
            stem = f"hardneg_{source.stem}_frame_{frame_no:06d}"
            image_path = unique_path(image_dir, stem, ".jpg")
            cv2.imwrite(str(image_path), frame)
            (label_dir / f"{image_path.stem}.txt").write_text("", encoding="utf-8")
            added.append(image_path)
        frame_no += 1

    cap.release()
    return added


def collect_sources(path: Path) -> list[Path]:
    if path.is_file():
        return [path]
    if not path.exists():
        path.mkdir(parents=True, exist_ok=True)
        return []
    return sorted(
        item
        for item in path.rglob("*")
        if item.is_file() and item.suffix.lower() in IMAGE_EXTENSIONS | VIDEO_EXTENSIONS
    )


def pick_model(start_from: str) -> Path | str:
    clean_best = ROOT / "runs" / "dji_marker_clean_float_retrain" / "weights" / "best.pt"
    base = ROOT / "yolo11m.pt"
    if start_from in ("auto", "clean-best") and clean_best.exists():
        return clean_best
    if base.exists():
        return base
    return "yolo11m.pt"


def main() -> int:
    args = parse_args()
    dataset = Path(args.dataset)
    negative_source = Path(args.negative_source)

    ensure_dataset(dataset)
    validate_marker_labels(dataset)

    sources = collect_sources(negative_source)
    if not sources:
        print(f"No hard negatives found in {negative_source}")
        print("Put false-positive images/videos there, then run this script again.")
        return 2

    added: list[Path] = []
    for source in sources:
        suffix = source.suffix.lower()
        if suffix in IMAGE_EXTENSIONS:
            added.append(copy_hard_negative_image(dataset, source, args.val_fraction))
        elif suffix in VIDEO_EXTENSIONS:
            added.extend(add_video_hard_negatives(dataset, source, args.val_fraction, args.video_fps))

    print(f"Added {len(added)} hard-negative image(s) with empty labels.")
    print("Dataset remains marker-only: blue_marker, red_marker, green_marker.")

    validate_marker_labels(dataset)
    if args.skip_train:
        return 0

    model = YOLO(str(pick_model(args.start_from)))
    model.train(
        data=str(dataset / "data.yaml"),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=0,
        workers=0,
        patience=30,
        cache=False,
        amp=False,
        project=str(ROOT / "runs"),
        name=args.run_name,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
