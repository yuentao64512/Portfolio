#!/usr/bin/env python3
"""
Camera calibration + altitude/bank-angle limit estimator for buoy detection.

Three separate things happen here, in order:

1. CAMERA CALIBRATION (standard, well-established): detects a checkerboard in
   your sample photos and runs OpenCV's cv2.calibrateCamera() to get the
   camera's intrinsic matrix (focal length in pixels, optical center) and
   lens distortion coefficients. This needs photos of the board at VARIED
   ANGLES as well as varied distances - straight-on shots at a few distances
   alone are a known-degenerate case for calibration (it can't separate focal
   length from distance if every photo is frontal). Tilt the board in
   different photos.

2. OPTIONAL CROSS-CHECK: for any photo whose filename encodes a known
   distance (e.g. IMG_50cm_01.jpg, board_1.0m.jpg, shot_2m.jpg), this
   independently estimates focal length from that single photo using the
   simple pinhole formula, as a sanity check against the full calibration in
   step 1. Large disagreement between the two means something's off (bad
   square-size value, wrong distance label, blur, etc).

3. ALTITUDE AND BANK-ANGLE ESTIMATES: built on top of step 1's focal length,
   plus a target size and a few parameters YOU must supply (not guessed):
   the minimum pixel size your detector needs, and the camera's mounting
   depression angle. These come with their modeling assumptions printed
   alongside the numbers - read them. This is an engineering estimate from a
   simplified model, not a certified flight envelope; validate before relying
   on it operationally.

Usage:
    python3 altitude_bank_calibration.py \\
        --calibration-dir photos/ \\
        --checkerboard-size 9x6 --square-size-mm 25 \\
        --target-diameter-m 0.61 \\
        --min-detect-px 20 \\
        --camera-depression-deg 15
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

IMAGE_EXTENSIONS = (".jpg", ".jpeg", ".png", ".bmp")


class CalibrationError(RuntimeError):
    pass


@dataclass
class CalibrationResult:
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    image_size: Tuple[int, int]  # (width, height)
    reprojection_error_px: float
    used_images: List[Path]
    skipped_images: List[Tuple[Path, str]]

    @property
    def focal_length_px(self) -> Tuple[float, float]:
        return float(self.camera_matrix[0, 0]), float(self.camera_matrix[1, 1])

    @property
    def principal_point_px(self) -> Tuple[float, float]:
        return float(self.camera_matrix[0, 2]), float(self.camera_matrix[1, 2])

    @property
    def fov_deg(self) -> Tuple[float, float]:
        """(horizontal, vertical) field of view in degrees."""
        fx, fy = self.focal_length_px
        width, height = self.image_size
        fov_h = 2 * math.degrees(math.atan((width / 2) / fx))
        fov_v = 2 * math.degrees(math.atan((height / 2) / fy))
        return fov_h, fov_v

    def to_json_dict(self) -> dict:
        return {
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "image_size": list(self.image_size),
            "reprojection_error_px": self.reprojection_error_px,
            "focal_length_px": list(self.focal_length_px),
            "fov_deg": list(self.fov_deg),
        }


def find_calibration_images(directory: Path) -> List[Path]:
    paths = [p for p in sorted(directory.iterdir()) if p.suffix.lower() in IMAGE_EXTENSIONS]
    if not paths:
        raise CalibrationError(f"no images ({', '.join(IMAGE_EXTENSIONS)}) found in {directory}")
    return paths


def calibrate_camera(
    image_paths: List[Path], pattern_size: Tuple[int, int], square_size_m: float
) -> CalibrationResult:
    """Standard OpenCV checkerboard calibration.

    pattern_size: (cols, rows) of INTERNAL corners - for a board with an
    10x7 grid of squares, that's (9, 6) internal corners.
    """
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : pattern_size[0], 0 : pattern_size[1]].T.reshape(-1, 2) * square_size_m

    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    image_size: Optional[Tuple[int, int]] = None
    used: List[Path] = []
    skipped: List[Tuple[Path, str]] = []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for path in image_paths:
        img = cv2.imread(str(path))
        if img is None:
            skipped.append((path, "could not read image"))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])
        elif (gray.shape[1], gray.shape[0]) != image_size:
            skipped.append((path, f"image size {gray.shape[1]}x{gray.shape[0]} doesn't match the others"))
            continue

        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not found:
            skipped.append((path, "checkerboard not detected"))
            continue

        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners)
        used.append(path)

    if len(objpoints) < 3:
        detail = "; ".join(f"{p.name}: {reason}" for p, reason in skipped)
        raise CalibrationError(
            f"only {len(objpoints)} usable image(s) - need at least 3 (10-15+ at varied "
            f"angles/distances is much better). Skipped: {detail or 'none'}"
        )

    assert image_size is not None
    _, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )

    total_error = 0.0
    for i in range(len(objpoints)):
        projected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        detected = imgpoints[i].reshape(-1, 2).astype(np.float64)
        projected = projected.reshape(-1, 2).astype(np.float64)
        total_error += float(np.linalg.norm(detected - projected, axis=1).mean())
    mean_error_px = total_error / len(objpoints)

    return CalibrationResult(
        camera_matrix=camera_matrix,
        dist_coeffs=dist_coeffs,
        image_size=image_size,
        reprojection_error_px=mean_error_px,
        used_images=used,
        skipped_images=skipped,
    )


DISTANCE_FILENAME_RE = re.compile(r"(\d+(?:\.\d+)?)\s*(cm|m)\b", re.IGNORECASE)


def parse_distance_from_filename(path: Path) -> Optional[float]:
    """Distance in meters if the filename encodes one (e.g. '50cm', '1.5m'), else None."""
    match = DISTANCE_FILENAME_RE.search(path.stem)
    if not match:
        return None
    value, unit = match.groups()
    value = float(value)
    return value / 100.0 if unit.lower() == "cm" else value


@dataclass
class CrossCheckSample:
    path: Path
    distance_m: float
    apparent_square_px: float
    focal_length_estimate_px: float


def cross_check_focal_length(
    image_paths: List[Path], pattern_size: Tuple[int, int], square_size_m: float
) -> List[CrossCheckSample]:
    """For images whose filename encodes a known board distance, independently
    estimate focal length via the simple pinhole formula - a sanity check
    against the full calibration, not a replacement for it."""
    cols, rows = pattern_size
    results: List[CrossCheckSample] = []

    for path in image_paths:
        distance_m = parse_distance_from_filename(path)
        if distance_m is None:
            continue
        img = cv2.imread(str(path))
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if not found:
            continue

        pts = corners.reshape(-1, 2)
        horiz_gaps = []
        for r in range(rows):
            row_pts = pts[r * cols : (r + 1) * cols]
            horiz_gaps.extend(float(np.linalg.norm(row_pts[i + 1] - row_pts[i])) for i in range(cols - 1))
        apparent_square_px = float(np.mean(horiz_gaps))
        focal_length_px = apparent_square_px * distance_m / square_size_m

        results.append(CrossCheckSample(path, distance_m, apparent_square_px, focal_length_px))

    return sorted(results, key=lambda s: s.distance_m)


def max_altitude_m(focal_length_px: float, target_diameter_m: float, min_detect_px: float) -> float:
    """Max distance to the target at which it still spans min_detect_px pixels.

    Pinhole model: apparent_px = focal_length_px * real_size_m / distance_m
    """
    return (focal_length_px * target_diameter_m) / min_detect_px


def _boresight_direction(depression_deg: float) -> np.ndarray:
    """Body-frame unit vector (x=forward, y=right, z=down) for a camera
    pointed `depression_deg` below horizontal, mounted on the aircraft's
    centerline (no left/right offset)."""
    phi = math.radians(depression_deg)
    return np.array([math.cos(phi), 0.0, math.sin(phi)])


def _rolled_direction(depression_deg: float, bank_deg: float) -> np.ndarray:
    """Where that boresight vector points after rolling the aircraft (and the
    rigidly-mounted camera with it) by `bank_deg` about the forward axis,
    with pitch and heading held fixed."""
    phi = math.radians(depression_deg)
    beta = math.radians(bank_deg)
    return np.array(
        [
            math.cos(phi),
            -math.sin(phi) * math.sin(beta),
            math.sin(phi) * math.cos(beta),
        ]
    )


def angular_deviation_deg(depression_deg: float, bank_deg: float) -> float:
    """Angle between the boresight before and after the roll - i.e. how far a
    target that was dead-center in frame before the roll has swept across
    the image."""
    before = _boresight_direction(depression_deg)
    after = _rolled_direction(depression_deg, bank_deg)
    cos_delta = float(np.clip(np.dot(before, after), -1.0, 1.0))
    return math.degrees(math.acos(cos_delta))


def max_bank_angle_deg(depression_deg: float, fov_h_deg: float, fov_v_deg: float) -> float:
    """Largest roll (bank) angle - pitch and heading held fixed - before a
    target that was dead-center in frame before the roll exits the field of
    view. Uses the smaller of the half horizontal/vertical FOV as the exit
    threshold (the conservative, not the generous, choice).

    MODEL ASSUMPTIONS (read these before trusting the number):
      - Target starts exactly at the image center (dead-center in frame)
        the instant before the roll begins.
      - The roll is isolated: pitch and heading (yaw) are held fixed - no
        coordinated turn, no aircraft translation during the roll.
      - Camera is rigidly mounted on the aircraft's longitudinal centerline
        (no lateral offset), at a fixed depression angle below horizontal.
      - Flat water, target at water level.
    This is a simplified engineering estimate, not a certified flight
    envelope. A real coordinated turn, a target that isn't dead-center to
    start with, or gimbal/mount flex will all change the real number.
    """
    half_fov_limit = min(fov_h_deg, fov_v_deg) / 2.0

    lo, hi = 0.0, 180.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if angular_deviation_deg(depression_deg, mid) < half_fov_limit:
            lo = mid
        else:
            hi = mid
    return lo


def parse_pattern_size(text: str) -> Tuple[int, int]:
    match = re.fullmatch(r"(\d+)x(\d+)", text.strip().lower())
    if not match:
        raise argparse.ArgumentTypeError(f"expected format like '9x6' (internal corners), got '{text}'")
    return int(match.group(1)), int(match.group(2))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--calibration-dir",
        required=True,
        type=Path,
        help="Folder of checkerboard photos (varied angles AND distances - "
        "straight-on shots alone can't fully separate focal length from distance)",
    )
    parser.add_argument(
        "--checkerboard-size",
        default="9x6",
        type=parse_pattern_size,
        help="Internal corner count of the checkerboard, cols x rows (default: %(default)s, "
        "i.e. a 10x7 grid of squares)",
    )
    parser.add_argument(
        "--square-size-mm",
        required=True,
        type=float,
        help="Real-world size of one checkerboard square, in mm (measure your printed board - "
        "a wrong value here scales every downstream number)",
    )
    parser.add_argument(
        "--target-diameter-m",
        default=0.61,
        type=float,
        help="Real-world diameter of the target buoy, in meters (default: %(default)s - "
        "derived from the official RobotX RoboBuoy BOM: two 38in pool noodles forming "
        "the float ring, circumference/pi. Not an official published diameter - override "
        "this if you have a direct measurement)",
    )
    parser.add_argument(
        "--min-detect-px",
        required=True,
        type=float,
        help="Minimum size (in pixels) the target needs to appear as for your detector to "
        "find it reliably. No sensible default exists - this depends entirely on your "
        "detection method",
    )
    parser.add_argument(
        "--camera-depression-deg",
        required=True,
        type=float,
        help="Angle between the camera's boresight and horizontal, measured downward: "
        "0 = camera aimed at the horizon, 90 = camera aimed straight down (nadir). "
        "No default - this depends on your physical mount; get it right or the altitude/"
        "bank numbers below will be wrong",
    )
    parser.add_argument(
        "--save-calibration",
        type=Path,
        default=None,
        help="Optional: write the calibration result (camera matrix, distortion, FOV) to this JSON file",
    )
    parser.add_argument(
        "--safety-margin-pct",
        type=float,
        default=30.0,
        help="How far below the hard altitude limit to recommend flying, as a percent "
        "(default: %(default)s%%, i.e. recommend 70%% of the limit). Margin exists because "
        "a detector tends to get less reliable right at its exact pixel threshold, your "
        "autopilot's altitude-hold has its own tolerance, and real flight has motion blur/"
        "vibration an idealized photo doesn't. Set to 0 to just see the hard limit.",
    )
    args = parser.parse_args()

    image_paths = find_calibration_images(args.calibration_dir)
    print(f"Found {len(image_paths)} image(s) in {args.calibration_dir}\n")

    try:
        result = calibrate_camera(image_paths, args.checkerboard_size, args.square_size_mm / 1000.0)
    except CalibrationError as e:
        print(f"Calibration failed: {e}", file=sys.stderr)
        sys.exit(1)

    print("=== CALIBRATION ===")
    print(f"Used {len(result.used_images)}/{len(image_paths)} images")
    if result.skipped_images:
        print(f"Skipped {len(result.skipped_images)}:")
        for path, reason in result.skipped_images:
            print(f"  {path.name}: {reason}")
    fx, fy = result.focal_length_px
    cx, cy = result.principal_point_px
    fov_h, fov_v = result.fov_deg
    print(f"Image size: {result.image_size[0]}x{result.image_size[1]}")
    print(f"Focal length: fx={fx:.1f}px  fy={fy:.1f}px")
    print(f"Principal point: ({cx:.1f}, {cy:.1f})")
    print(f"Field of view: {fov_h:.1f}deg horizontal, {fov_v:.1f}deg vertical")
    print(f"Distortion coefficients (k1,k2,p1,p2,k3): {result.dist_coeffs.flatten()}")
    print(f"Mean reprojection error: {result.reprojection_error_px:.3f}px", end="")
    if result.reprojection_error_px < 0.5:
        print("  (good)")
    elif result.reprojection_error_px < 1.0:
        print("  (acceptable)")
    else:
        print("  (high - consider recapturing with more varied angles/distances)")

    cross_check = cross_check_focal_length(image_paths, args.checkerboard_size, args.square_size_mm / 1000.0)
    if cross_check:
        print("\n=== CROSS-CHECK (from filenames with a known distance) ===")
        for sample in cross_check:
            diff_pct = 100.0 * (sample.focal_length_estimate_px - fx) / fx
            print(
                f"  {sample.path.name}: distance={sample.distance_m:.2f}m  "
                f"apparent_square={sample.apparent_square_px:.1f}px  "
                f"focal_length_estimate={sample.focal_length_estimate_px:.1f}px  "
                f"({diff_pct:+.1f}% vs calibration)"
            )

    if args.save_calibration:
        args.save_calibration.write_text(json.dumps(result.to_json_dict(), indent=2))
        print(f"\nSaved calibration to {args.save_calibration}")

    print("\n=== ALTITUDE LIMIT ===")
    print(f"Target: {args.target_diameter_m:.2f}m diameter buoy")
    print(f"Minimum detectable size: {args.min_detect_px:.0f}px")
    alt_x = max_altitude_m(fx, args.target_diameter_m, args.min_detect_px)
    alt_y = max_altitude_m(fy, args.target_diameter_m, args.min_detect_px)
    altitude_limit = min(alt_x, alt_y)
    print(f"Max distance to target before it drops below {args.min_detect_px:.0f}px: {altitude_limit:.1f}m")
    print("(assumes the target is roughly centered in frame and this is a straight-line")
    print(" distance to it, not necessarily pure vertical altitude if the camera is angled)")

    recommended = altitude_limit * (1 - args.safety_margin_pct / 100.0)
    print(f"\nRecommended operating altitude ({args.safety_margin_pct:.0f}% margin below the hard limit): "
          f"{recommended:.1f}m")
    print(f"  at this height, the target spans ~{args.min_detect_px / (1 - args.safety_margin_pct / 100.0):.0f}px "
          f"instead of exactly {args.min_detect_px:.0f}px - comfortably above your minimum, not right at the edge")

    print("\n=== BANK ANGLE LIMIT ===")
    print(f"Camera depression angle: {args.camera_depression_deg:.1f}deg below horizontal")
    print(f"Field of view used: {fov_h:.1f}deg x {fov_v:.1f}deg")
    bank_limit = max_bank_angle_deg(args.camera_depression_deg, fov_h, fov_v)
    print(f"Max bank (roll) angle before a dead-center target exits frame: {bank_limit:.1f}deg")
    if bank_limit > 90.0:
        print(
            "NOTE: this exceeds any physically meaningful bank angle (>90deg isn't a real "
            "flight attitude) - it means that at this shallow a depression angle, roll alone "
            "essentially never pushes a dead-center target out of frame within a normal "
            "flight envelope. Something else (a coordinated turn's yaw, the target not "
            "being dead-center to start with, etc.) would be the actual binding constraint, "
            "not bank angle by itself."
        )
    print("ASSUMPTIONS - read before relying on this number:")
    print("  - target starts dead-center in frame the instant before the roll")
    print("  - pure roll: pitch and heading held fixed (not a coordinated turn)")
    print("  - camera mounted on the aircraft's centerline, no lateral offset")
    print("  - flat water, target at water level")
    print("This is a simplified engineering estimate, not a certified flight envelope.")


if __name__ == "__main__":
    main()
