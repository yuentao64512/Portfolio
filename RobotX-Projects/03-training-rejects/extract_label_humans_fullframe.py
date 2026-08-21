from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
DEFAULT_VIDEOS = ("water_test.mp4", "water_test2.mp4")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Extract full frames from water_test videos and create YOLO labels for "
            "the single class 'human' using the COCO person class from YOLO11L."
        )
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument(
        "--video",
        action="append",
        default=None,
        help="Video name/path. Repeat for multiple videos. Defaults to water_test.mp4 and water_test2.mp4.",
    )
    parser.add_argument(
        "--dataset",
        type=Path,
        default=ROOT / "human_reject_dataset_fullframe",
    )
    parser.add_argument("--model", default=str(ROOT / "yolo11l.pt"))
    parser.add_argument("--sample-fps", type=float, default=1.0)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--conf", type=float, default=0.15)
    parser.add_argument("--val-fraction", type=float, default=0.20)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace the existing generated dataset without keeping a backup.",
    )
    return parser.parse_args()


def resolve_videos(root: Path, values: list[str] | None) -> list[Path]:
    requested = values or list(DEFAULT_VIDEOS)
    videos: list[Path] = []
    seen: set[str] = set()

    for value in requested:
        path = Path(value)
        if not path.is_absolute():
            path = root / path
        if not path.exists():
            # Windows paths are case-insensitive, but this also handles renamed extensions.
            matches = [p for p in path.parent.glob("*") if p.name.lower() == path.name.lower()]
            if matches:
                path = matches[0]
        if not path.is_file():
            raise FileNotFoundError(f"Video not found: {path}")
        key = str(path.resolve()).lower()
        if key not in seen:
            seen.add(key)
            videos.append(path)

    if len(videos) != 2 and values is None:
        raise RuntimeError(
            "Expected both water_test.mp4 and water_test2.mp4. "
            f"Resolved {len(videos)} video(s): {videos}"
        )
    return videos


def reset_dataset(dataset: Path, overwrite: bool) -> None:
    if dataset.exists():
        if overwrite:
            import shutil

            shutil.rmtree(dataset)
        else:
            suffix = 1
            backup = dataset.with_name(f"{dataset.name}_old")
            while backup.exists():
                backup = dataset.with_name(f"{dataset.name}_old_{suffix}")
                suffix += 1
            dataset.rename(backup)
            print(f"Previous dataset moved to: {backup}")

    for relative in ("images/train", "images/val", "labels/train", "labels/val"):
        (dataset / relative).mkdir(parents=True, exist_ok=True)

    (dataset / "classes.txt").write_text("human\n", encoding="utf-8")
    (dataset / "data.yaml").write_text(
        "\n".join(
            (
                f"path: {dataset.as_posix()}",
                "train: images/train",
                "val: images/val",
                "nc: 1",
                "names:",
                "  0: human",
                "",
            )
        ),
        encoding="utf-8",
    )


def human_label_lines(result) -> list[str]:
    lines: list[str] = []
    if result.boxes is None:
        return lines

    for box in result.boxes:
        # Class 0 in the pretrained COCO model is person.
        if int(box.cls.item()) != 0:
            continue
        x, y, width, height = (float(value) for value in box.xywhn[0].tolist())
        lines.append(f"0 {x:.6f} {y:.6f} {width:.6f} {height:.6f}")
    return lines


def split_for_frame(frame_number: int, total_frames: int, val_fraction: float) -> str:
    val_fraction = min(max(val_fraction, 0.05), 0.45)
    cutoff = int(total_frames * (1.0 - val_fraction))
    return "train" if frame_number < cutoff else "val"


def process_video(
    video: Path,
    dataset: Path,
    model: YOLO,
    args: argparse.Namespace,
    manifest: csv.writer,
) -> tuple[int, int, int]:
    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise RuntimeError(f"OpenCV could not open: {video}")

    source_fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = max(1, int(capture.get(cv2.CAP_PROP_FRAME_COUNT)))
    step = max(1, round(source_fps / max(args.sample_fps, 0.01)))
    sampled = 0
    positive = 0
    human_boxes = 0
    frame_number = 0

    print(
        f"{video.name}: {total_frames} frames at {source_fps:.2f} FPS; "
        f"sampling every {step} frame(s).",
        flush=True,
    )

    while True:
        ok, frame = capture.read()
        if not ok:
            break

        if frame_number % step == 0:
            result = model.predict(
                source=frame,
                classes=[0],
                conf=args.conf,
                imgsz=args.imgsz,
                verbose=False,
            )[0]
            labels = human_label_lines(result)
            split = split_for_frame(frame_number, total_frames, args.val_fraction)
            stem = f"{video.stem}_frame_{frame_number:08d}"
            image_path = dataset / "images" / split / f"{stem}.jpg"
            label_path = dataset / "labels" / split / f"{stem}.txt"

            if not cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError(f"Failed to save frame: {image_path}")
            label_path.write_text(
                ("\n".join(labels) + "\n") if labels else "",
                encoding="utf-8",
            )

            timestamp_seconds = frame_number / source_fps
            manifest.writerow(
                (
                    image_path.name,
                    video.name,
                    frame_number,
                    f"{timestamp_seconds:.3f}",
                    split,
                    len(labels),
                )
            )
            sampled += 1
            positive += int(bool(labels))
            human_boxes += len(labels)
            if sampled % 25 == 0:
                print(
                    f"  saved {sampled} full frames; "
                    f"{positive} contain human labels; {human_boxes} human boxes",
                    flush=True,
                )

        frame_number += 1

    capture.release()
    return sampled, positive, human_boxes


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    dataset = args.dataset
    if not dataset.is_absolute():
        dataset = root / dataset
    dataset = dataset.resolve()
    videos = resolve_videos(root, args.video)
    reset_dataset(dataset, args.overwrite)

    model_path = Path(args.model)
    if not model_path.is_absolute():
        model_path = root / model_path
    if not model_path.exists():
        raise FileNotFoundError(
            f"YOLO11L weights not found: {model_path}\n"
            "Place yolo11l.pt in C:\\Users\\OptiPrime-UAV\\yolov11_train."
        )

    print(f"Human pseudo-label model: {model_path}")
    print(f"Dataset output: {dataset}")
    model = YOLO(str(model_path))
    totals = [0, 0, 0]

    manifest_path = dataset / "manifest.csv"
    with manifest_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("image", "source_video", "source_frame", "time_seconds", "split", "human_boxes"))
        for video in videos:
            counts = process_video(video, dataset, model, args, writer)
            totals = [left + right for left, right in zip(totals, counts)]

    train_images = len(list((dataset / "images" / "train").glob("*.jpg")))
    val_images = len(list((dataset / "images" / "val").glob("*.jpg")))
    print("")
    print("Dataset complete.")
    print(f"  Full frames: {totals[0]} ({train_images} train, {val_images} val)")
    print(f"  Frames with humans: {totals[1]}")
    print(f"  Human boxes: {totals[2]}")
    print(f"  Empty-label negative frames: {totals[0] - totals[1]}")
    print(f"  YAML: {dataset / 'data.yaml'}")

    if train_images == 0 or val_images == 0 or totals[1] == 0:
        raise RuntimeError("Dataset is not trainable: check the video contents and extraction settings.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
