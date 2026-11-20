#!/usr/bin/env python3
"""Extract original-resolution frames while preserving source-group names."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parent
VIDEO_SUFFIXES = {".mov", ".mp4", ".avi", ".mkv", ".m4v"}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def safe_name(path: Path) -> str:
    return "_".join(path.stem.replace("__", "_").split())


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--videos", type=Path, default=ROOT / "raw" / "videos")
    parser.add_argument("--images", type=Path, default=ROOT / "raw" / "images")
    parser.add_argument("--output", type=Path, default=ROOT / "frames")
    parser.add_argument("--fps", type=float, default=2.0, help="saved frames per second")
    args = parser.parse_args()
    if args.fps <= 0:
        raise SystemExit("--fps must be positive")
    args.output.mkdir(parents=True, exist_ok=True)

    for video in sorted(args.videos.glob("*")):
        if video.suffix.lower() not in VIDEO_SUFFIXES:
            continue
        capture = cv2.VideoCapture(str(video))
        if not capture.isOpened():
            print(f"SKIP cannot open {video}")
            continue
        source_fps = capture.get(cv2.CAP_PROP_FPS)
        if source_fps <= 0:
            print(f"SKIP invalid FPS {video}")
            capture.release()
            continue
        step = max(1, round(source_fps / args.fps))
        index = saved = 0
        prefix = f"vid-{safe_name(video)}"
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if index % step == 0:
                target = args.output / f"{prefix}__{index:08d}.jpg"
                if target.exists():
                    raise SystemExit(f"Refusing to overwrite {target}")
                if not cv2.imwrite(str(target), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    raise RuntimeError(f"Failed to write {target}")
                saved += 1
            index += 1
        capture.release()
        print(f"{video.name}: source_fps={source_fps:.3f} saved={saved}")

    for image_path in sorted(args.images.glob("*")):
        if image_path.suffix.lower() not in IMAGE_SUFFIXES:
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"SKIP cannot read {image_path}")
            continue
        target = args.output / f"img-{safe_name(image_path)}__00000000.jpg"
        if target.exists():
            raise SystemExit(f"Refusing to overwrite {target}")
        if not cv2.imwrite(str(target), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
            raise RuntimeError(f"Failed to write {target}")
        print(f"{image_path.name}: saved=1")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
