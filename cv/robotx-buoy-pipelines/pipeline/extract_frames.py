from __future__ import annotations

import argparse
import csv
import hashlib
import re
import shutil
from pathlib import Path

import cv2
import numpy as np


VIDEO_EXTENSIONS = {".mp4", ".mov", ".avi", ".mkv", ".m4v"}
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def slug(text: str) -> str:
    result = re.sub(r"[^A-Za-z0-9_-]+", "_", text).strip("_")
    return result or "media"


def source_id(path: Path) -> str:
    digest = hashlib.sha1(path.name.encode("utf-8")).hexdigest()[:8]
    return f"{slug(path.stem)}_{digest}"


def sharpness(frame: np.ndarray) -> float:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def small_gray(frame: np.ndarray) -> np.ndarray:
    height, width = frame.shape[:2]
    scale = min(1.0, 320.0 / max(height, width))
    resized = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)


def assign_split(source_name: str) -> str:
    bucket = int(hashlib.sha1(source_name.encode("utf-8")).hexdigest()[:8], 16) % 100
    if bucket < 75:
        return "train"
    if bucket < 95:
        return "val"
    return "test"


def extract_video(
    video_path: Path,
    output_dir: Path,
    fps_out: float,
    motion_threshold: float,
) -> list[dict[str, object]]:
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        return [{"source": video_path.name, "status": "unreadable_video"}]

    native_fps = float(capture.get(cv2.CAP_PROP_FPS) or 30.0)
    sample_step = max(1, int(round(native_fps / fps_out)))
    total_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT) or 0)
    identity = source_id(video_path)
    split = assign_split(video_path.name)
    rows: list[dict[str, object]] = []
    previous_kept: np.ndarray | None = None
    frame_index = 0
    kept_index = 0

    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if frame_index % sample_step != 0:
            frame_index += 1
            continue

        preview = small_gray(frame)
        motion = 999.0 if previous_kept is None else float(cv2.absdiff(preview, previous_kept).mean())
        keep = previous_kept is None or motion >= motion_threshold
        if keep:
            time_ms = float(capture.get(cv2.CAP_PROP_POS_MSEC))
            filename = f"{identity}__f{frame_index:08d}__t{int(round(time_ms)):010d}.jpg"
            destination = output_dir / filename
            cv2.imwrite(str(destination), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
            previous_kept = preview
            kept_index += 1
            rows.append(
                {
                    "image": filename,
                    "source": video_path.name,
                    "source_id": identity,
                    "split": split,
                    "frame_index": frame_index,
                    "time_ms": round(time_ms, 3),
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                    "motion_score": round(motion, 4),
                    "sharpness": round(sharpness(frame), 3),
                    "status": "kept",
                }
            )
        frame_index += 1

    capture.release()
    if kept_index == 0:
        rows.append(
            {
                "source": video_path.name,
                "source_id": identity,
                "split": split,
                "frame_index": total_frames,
                "status": "no_frames_kept",
            }
        )
    return rows


def copy_still(image_path: Path, output_dir: Path) -> dict[str, object]:
    frame = cv2.imread(str(image_path))
    identity = source_id(image_path)
    if frame is None:
        return {"source": image_path.name, "source_id": identity, "status": "unreadable_image"}
    filename = f"still__{identity}{image_path.suffix.lower()}"
    shutil.copy2(image_path, output_dir / filename)
    return {
        "image": filename,
        "source": image_path.name,
        "source_id": identity,
        "split": "train",
        "frame_index": 0,
        "time_ms": 0,
        "width": frame.shape[1],
        "height": frame.shape[0],
        "motion_score": 999.0,
        "sharpness": round(sharpness(frame), 3),
        "status": "kept",
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--media", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=2.0)
    parser.add_argument("--motion-threshold", type=float, default=1.8)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    media_files = sorted(path for path in args.media.iterdir() if path.is_file())
    for path in media_files:
        suffix = path.suffix.lower()
        if suffix in VIDEO_EXTENSIONS:
            print(f"extracting {path.name}", flush=True)
            rows.extend(extract_video(path, args.output, args.fps, args.motion_threshold))
        elif suffix in IMAGE_EXTENSIONS:
            rows.append(copy_still(path, args.output))

    fieldnames = [
        "image",
        "source",
        "source_id",
        "split",
        "frame_index",
        "time_ms",
        "width",
        "height",
        "motion_score",
        "sharpness",
        "status",
    ]
    manifest = args.output.parent / "frames_manifest.csv"
    with manifest.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})

    kept = sum(row.get("status") == "kept" for row in rows)
    print(f"kept_frames={kept}", flush=True)
    print(f"manifest={manifest}", flush=True)


if __name__ == "__main__":
    main()
