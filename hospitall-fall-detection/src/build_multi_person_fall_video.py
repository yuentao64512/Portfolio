#!/usr/bin/env python3
"""Build a deterministic hospital CCTV stress-test video from supplied frames."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def fit_16_9(image: np.ndarray, width: int, height: int) -> np.ndarray:
    """Centre-crop without stretching, then resize to the requested frame size."""
    source_h, source_w = image.shape[:2]
    target_ratio = width / height
    source_ratio = source_w / source_h
    if source_ratio > target_ratio:
        crop_w = int(round(source_h * target_ratio))
        left = max(0, (source_w - crop_w) // 2)
        image = image[:, left : left + crop_w]
    else:
        crop_h = int(round(source_w / target_ratio))
        top = max(0, (source_h - crop_h) // 2)
        image = image[top : top + crop_h, :]
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


def label(frame: np.ndarray, text: str) -> np.ndarray:
    output = frame.copy()
    cv2.rectangle(output, (0, 0), (output.shape[1], 54), (25, 25, 25), -1)
    cv2.putText(
        output,
        text,
        (18, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.78,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    # Hard cuts are deliberate: cross-fading two people can invent ghost bodies
    # and is unsuitable for an occlusion/tracking test.
    timeline = [
        ("01_walk.png", 3.0, "Patient walking normally"),
        ("02_buckle.png", 0.8, "Fall begins: knees and hips buckle"),
        ("03_descend.png", 0.8, "Full-body descent: torso and hips move down"),
        ("04_floor.png", 4.0, "Patient remains fully down on floor"),
        ("05_nurse_enter.png", 1.6, "Second person enters; patient remains tracked"),
        ("06_nurse_overlap_new.png", 2.0, "Occlusion stress test: nurse crosses patient"),
        ("08_nurse_exit.png", 1.6, "Second person exits; patient remains on floor"),
        ("04_floor.png", 4.0, "Patient still down after occlusion"),
    ]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        args.fps,
        (args.width, args.height),
    )
    if not writer.isOpened():
        raise SystemExit(f"Could not create {args.output}")

    total = 0
    try:
        for name, seconds, caption in timeline:
            path = args.frames / name
            image = cv2.imread(str(path))
            if image is None:
                raise SystemExit(f"Could not read {path}")
            image = label(fit_16_9(image, args.width, args.height), caption)
            count = max(1, int(round(seconds * args.fps)))
            for _ in range(count):
                writer.write(image)
            total += count
    finally:
        writer.release()

    print(f"frames={total} fps={args.fps:.3f} duration={total / args.fps:.2f}s output={args.output}")


if __name__ == "__main__":
    main()
