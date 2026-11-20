#!/usr/bin/env python3
"""Calibrate a monocular camera from checkerboard photos at runtime resolution."""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", required=True, help="glob for checkerboard images")
    parser.add_argument("--columns", type=int, required=True, help="inner checkerboard corners across")
    parser.add_argument("--rows", type=int, required=True, help="inner checkerboard corners down")
    parser.add_argument("--square-size-m", type=float, required=True)
    parser.add_argument("--output", type=Path, default=Path("camera_calibration.json"))
    parser.add_argument("--max-error", type=float, default=0.5)
    args = parser.parse_args()

    paths = [Path(path) for path in sorted(glob.glob(args.images))]
    if len(paths) < 15:
        raise SystemExit("At least 15 checkerboard images are required; about 25 is recommended")
    object_template = np.zeros((args.rows * args.columns, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:args.columns, 0:args.rows].T.reshape(-1, 2)
    object_template *= args.square_size_m
    object_points, image_points = [], []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    used = []
    for path in paths:
        image = cv2.imread(str(path))
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]
        elif image_size != gray.shape[::-1]:
            raise SystemExit("All calibration images must have the same runtime resolution")
        found, corners = cv2.findChessboardCorners(gray, (args.columns, args.rows))
        if not found:
            continue
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(object_template.copy())
        image_points.append(corners)
        used.append(str(path))
    if len(used) < 12:
        raise SystemExit(f"Only {len(used)} usable checkerboard images; capture more angles")
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    total_error = 0.0
    total_points = 0
    for obj, observed, rotation, translation in zip(object_points, image_points, rotations, translations):
        projected, _ = cv2.projectPoints(obj, rotation, translation, matrix, distortion)
        total_error += cv2.norm(observed, projected, cv2.NORM_L2) ** 2
        total_points += len(projected)
    reprojection_error = float(np.sqrt(total_error / total_points))
    report = {
        "image_size": list(image_size), "camera_matrix": matrix.tolist(),
        "distortion_coefficients": distortion.tolist(), "rms": float(rms),
        "reprojection_error_px": reprojection_error, "images_used": used,
        "passes_0_5px_target": reprojection_error <= args.max_error,
    }
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if reprojection_error > args.max_error:
        raise SystemExit(f"Calibration error {reprojection_error:.3f}px exceeds {args.max_error:.3f}px")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

