#!/usr/bin/env python3
"""Fixed-camera ZED2i box cuboids, wall gap, and stack relationships.

This replaces the incorrect assumption that every rectangular image contour is
a separate box.  A one-time empty-scene calibration supplies the background,
support plane, and wall plane.  Foreground components then contain all visible
faces of a physical box and are fitted in support-aligned 3D coordinates.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

from zed_box_geometry import (
    SupportPlane,
    bbox_iou,
    component_boxes,
    fit_support_plane,
    foreground_mask,
    heights_above_support,
    points_for_component,
    support_plane_points,
)


ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class Plane:
    normal: np.ndarray
    offset: float
    inlier_ratio: float


@dataclass(frozen=True)
class Cuboid:
    length_m: float
    breadth_m: float
    height_m: float
    bottom_height_m: float
    top_height_m: float
    centre_depth_m: float
    geometry_ratio: float
    reliable_3d: bool
    point_count: int


@dataclass
class Detection:
    bbox: tuple[int, int, int, int]
    cuboid: Cuboid
    wall_gap_m: float


@dataclass
class Track:
    track_id: int
    bbox: tuple[int, int, int, int]
    last_frame: int
    values: deque[np.ndarray] = field(default_factory=lambda: deque(maxlen=20))


@dataclass(frozen=True)
class Measurement:
    track_id: int
    bbox: tuple[int, int, int, int]
    length_m: float
    breadth_m: float
    height_m: float
    bottom_height_m: float
    top_height_m: float
    centre_depth_m: float
    wall_gap_m: float
    geometry_ratio: float
    reliable_3d: bool
    point_count: int


def fit_wall_plane(
    points_xyz: np.ndarray,
    support_normal: np.ndarray,
    threshold_m: float = 0.012,
    iterations: int = 160,
    minimum_points: int = 300,
    maximum_support_alignment: float = 0.35,
    rng_seed: int = 31,
) -> Plane | None:
    """Fit the dominant plane approximately perpendicular to the support."""
    points = np.asarray(points_xyz, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < minimum_points:
        return None
    if len(points) > 16000:
        rng = np.random.default_rng(rng_seed)
        points = points[rng.choice(len(points), 16000, replace=False)]
    rng = np.random.default_rng(rng_seed)
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal /= norm
        if abs(float(normal @ support_normal)) > maximum_support_alignment:
            continue
        offset = -float(normal @ sample[0])
        mask = np.abs(points @ normal + offset) <= threshold_m
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < minimum_points:
        return None
    inliers = points[best_mask]
    centre = np.mean(inliers, axis=0)
    _, _, axes = np.linalg.svd(inliers - centre, full_matrices=False)
    normal = axes[-1]
    offset = -float(normal @ centre)
    if offset < 0.0:  # camera origin must be on the positive side
        normal = -normal
        offset = -offset
    ratio = float(np.mean(np.abs(points @ normal + offset) <= threshold_m))
    return Plane(normal, offset, ratio)


def overhead_support_plane_points(
    xyz_map: np.ndarray,
    border_px: int = 45,
    stride: int = 4,
    max_points: int = 24000,
) -> np.ndarray:
    """Sample the central scene for a downward camera's dominant floor plane."""
    height, width = xyz_map.shape[:2]
    y1, y2 = border_px, max(border_px + 1, height - border_px)
    x1, x2 = border_px, max(border_px + 1, width - border_px)
    points = np.asarray(
        xyz_map[y1:y2:stride, x1:x2:stride, :3], dtype=np.float64
    ).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 2] >= 0.15) & (points[:, 2] <= 3.0)]
    if len(points) > max_points:
        rng = np.random.default_rng(47)
        points = points[rng.choice(len(points), max_points, replace=False)]
    return points


def fit_overhead_support_plane(
    points_xyz: np.ndarray,
    expected_distance_m: float,
    threshold_m: float = 0.008,
    distance_tolerance_m: float = 0.12,
    iterations: int = 220,
    minimum_points: int = 500,
    rng_seed: int = 43,
) -> SupportPlane | None:
    """Fit a floor plane without assuming its normal lies on camera Y.

    For a straight-down camera the floor normal is near camera Z.  The known
    perpendicular camera height selects the correct plane instead of nearby
    rails or box faces.
    """
    points = np.asarray(points_xyz, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < minimum_points or expected_distance_m <= 0.0:
        return None
    rng = np.random.default_rng(rng_seed)
    best_mask = None
    best_score = -math.inf
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -float(normal @ sample[0])
        camera_distance = abs(offset)
        distance_error = abs(camera_distance - expected_distance_m)
        if distance_error > distance_tolerance_m:
            continue
        mask = np.abs(points @ normal + offset) <= threshold_m
        count = int(mask.sum())
        score = count - distance_error / max(distance_tolerance_m, 1e-9) * minimum_points
        if count >= minimum_points and score > best_score:
            best_score = score
            best_mask = mask
    if best_mask is None:
        return None
    inliers = points[best_mask]
    centre = np.mean(inliers, axis=0)
    _, _, axes = np.linalg.svd(inliers - centre, full_matrices=False)
    normal = axes[-1]
    offset = -float(normal @ centre)
    if offset < 0.0:
        normal = -normal
        offset = -offset
    camera_height = float(offset)
    if abs(camera_height - expected_distance_m) > distance_tolerance_m:
        return None
    ratio = float(np.mean(np.abs(points @ normal + offset) <= threshold_m))
    return SupportPlane(normal, offset, ratio, camera_height)


def wall_plane_points(
    xyz_map: np.ndarray,
    support: SupportPlane,
    end_row_fraction: float = 0.72,
    stride: int = 4,
) -> np.ndarray:
    """Sample upper-background points, excluding the support plane."""
    height = xyz_map.shape[0]
    points = np.asarray(
        xyz_map[: int(height * end_row_fraction):stride, ::stride, :3], dtype=np.float64
    ).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 2] >= 0.20) & (points[:, 2] <= 12.0)]
    if not len(points):
        return points
    support_distance = np.abs(points @ support.normal + support.offset)
    return points[support_distance > 0.035]


def support_basis(normal: np.ndarray, points_xyz: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return PCA horizontal axes orthogonal to the support normal."""
    points = np.asarray(points_xyz, dtype=np.float64)
    horizontal = points - np.outer(points @ normal, normal)
    centred = horizontal - np.median(horizontal, axis=0)
    _, _, axes = np.linalg.svd(centred, full_matrices=False)
    first = axes[0] - float(axes[0] @ normal) * normal
    first /= max(float(np.linalg.norm(first)), 1e-12)
    second = np.cross(normal, first)
    second /= max(float(np.linalg.norm(second)), 1e-12)
    return first, second


def fit_support_aligned_cuboid(
    points_xyz: np.ndarray,
    support: SupportPlane,
    metric_scale: float = 1.0,
    trim_percent: float = 1.0,
    minimum_points: int = 250,
    height_bounds_m: tuple[float, float] | None = None,
) -> Cuboid | None:
    points = np.asarray(points_xyz, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < minimum_points:
        return None
    first, second = support_basis(support.normal, points)
    coordinates = np.column_stack(
        (points @ first, points @ second, points @ support.normal + support.offset)
    )
    # PCA alone is biased when the camera sees unequal numbers of points on the
    # top/front/side faces.  Use PCA only to establish a 2-D support-plane
    # coordinate system, then fit the footprint's minimum-area rectangle.
    # This fit is invariant to the initial basis rotation and follows the box
    # edges instead of the point-density distribution.
    footprint = coordinates[:, :2].astype(np.float32)
    if trim_percent > 0.0:
        low_2d, high_2d = np.percentile(
            footprint, [trim_percent * 0.25, 100.0 - trim_percent * 0.25], axis=0
        )
        keep = np.all((footprint >= low_2d) & (footprint <= high_2d), axis=1)
        if int(np.count_nonzero(keep)) >= minimum_points:
            footprint = footprint[keep]
    _, rectangle_size, _ = cv2.minAreaRect(footprint)
    horizontal_extents = np.asarray(rectangle_size, dtype=np.float64) * metric_scale
    length_m, breadth_m = sorted(map(float, horizontal_extents), reverse=True)
    if height_bounds_m is None:
        lower, upper = np.percentile(
            coordinates[:, 2], [trim_percent, 100.0 - trim_percent]
        )
        bottom_height_m = max(0.0, float(lower) * metric_scale)
        top_height_m = max(0.0, float(upper) * metric_scale)
    else:
        bottom_height_m = max(0.0, float(height_bounds_m[0]))
        top_height_m = max(bottom_height_m, float(height_bounds_m[1]))
    height_m = max(0.0, top_height_m - bottom_height_m)
    centred = coordinates - np.median(coordinates, axis=0)
    singular_values = np.linalg.svd(centred, compute_uv=False)
    geometry_ratio = float(singular_values[-1] / max(singular_values[-2], 1e-12))
    reliable = (
        length_m >= 0.025 and breadth_m >= 0.025 and height_m >= 0.025
        and (height_bounds_m is not None or geometry_ratio >= 0.07)
    )
    return Cuboid(
        length_m=length_m,
        breadth_m=breadth_m,
        height_m=height_m,
        bottom_height_m=bottom_height_m,
        top_height_m=top_height_m,
        centre_depth_m=float(np.median(points[:, 2])) * metric_scale,
        geometry_ratio=geometry_ratio,
        reliable_3d=reliable,
        point_count=len(points),
    )


def observed_wall_gap(points_xyz: np.ndarray, wall: Plane | None, metric_scale: float) -> float:
    """Shortest robust distance between observed box points and the wall plane."""
    if wall is None or len(points_xyz) < 30:
        return math.nan
    distances = (np.asarray(points_xyz) @ wall.normal + wall.offset) * metric_scale
    distances = distances[np.isfinite(distances) & (distances >= -0.03)]
    return max(0.0, float(np.percentile(distances, 2.0))) if len(distances) else math.nan


def mask_bbox(mask: np.ndarray) -> tuple[int, int, int, int] | None:
    ys, xs = np.where(mask)
    if not len(xs):
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()) + 1, int(ys.max()) + 1


def physical_height_span(mask: np.ndarray, xyz_map: np.ndarray, support: SupportPlane) -> float:
    points = points_for_component(xyz_map, mask, stride=3, max_points=8000)
    bottom, top = heights_above_support(points, support)
    return top - bottom if math.isfinite(bottom) and math.isfinite(top) else 0.0


def overhead_height_tiers(
    component: np.ndarray,
    xyz_map: np.ndarray,
    support: SupportPlane,
    metric_scale: float,
    minimum_area_px: int,
    bin_width_m: float = 0.008,
    minimum_level_gap_m: float = 0.035,
) -> list[tuple[np.ndarray, tuple[float, float]]]:
    """Return nested footprint masks and bottom/top heights for visible tiers.

    With a downward camera, visible box tops form peaks in height above the
    calibrated floor.  A lower tier includes the pixels of tiers above it so
    that an offset upper box does not punch a hole in the lower footprint.
    Fully coincident boxes with no exposed lower top remain unobservable from
    one overhead view and correctly collapse to one total-height tier.
    """
    xyz = np.asarray(xyz_map[..., :3], dtype=np.float64)
    heights = (xyz @ support.normal + support.offset) * metric_scale
    valid = component & np.isfinite(heights) & (heights >= 0.025) & (heights <= 3.0)
    values = heights[valid]
    if len(values) < max(250, minimum_area_px // 8):
        return []

    upper = max(bin_width_m * 3.0, float(np.percentile(values, 99.5)) + bin_width_m)
    edges = np.arange(0.0, upper + bin_width_m, bin_width_m)
    histogram, _ = np.histogram(values, bins=edges)
    if len(histogram) < 3:
        levels = [float(np.median(values))]
    else:
        smooth = np.convolve(histogram.astype(np.float64), np.array([1, 2, 3, 2, 1]) / 9.0, mode="same")
        minimum_peak = max(45.0, len(values) * 0.015)
        candidates = [
            index for index in range(1, len(smooth) - 1)
            if smooth[index] >= smooth[index - 1]
            and smooth[index] >= smooth[index + 1]
            and smooth[index] >= minimum_peak
        ]
        candidates.sort(key=lambda index: smooth[index], reverse=True)
        selected: list[int] = []
        for index in candidates:
            level = float((edges[index] + edges[index + 1]) * 0.5)
            if all(abs(level - float((edges[other] + edges[other + 1]) * 0.5)) >= minimum_level_gap_m for other in selected):
                selected.append(index)
        levels = sorted(float((edges[index] + edges[index + 1]) * 0.5) for index in selected)
        if not levels:
            levels = [float(np.median(values))]

    tiers: list[tuple[np.ndarray, tuple[float, float]]] = []
    previous_top = 0.0
    for level in levels:
        if level - previous_top < 0.025:
            continue
        tier_mask = valid & (heights >= level - max(0.015, bin_width_m * 2.0))
        output = tier_mask.astype(np.uint8) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel, iterations=2)
        tier_mask = output > 0
        if int(tier_mask.sum()) < max(400, minimum_area_px // 4):
            continue
        tiers.append((tier_mask, (previous_top, level)))
        previous_top = level
    return tiers


def split_component_at_stack_seams(
    component: np.ndarray,
    frame: np.ndarray,
    xyz_map: np.ndarray,
    support: SupportPlane,
    minimum_area_px: int,
    minimum_box_height_m: float = 0.045,
    depth: int = 0,
) -> list[np.ndarray]:
    """Recursively split a merged stack at long horizontal image seams."""
    bbox = mask_bbox(component)
    if bbox is None or depth >= 4:
        return [component]
    x1, y1, x2, y2 = bbox
    width, height = x2 - x1, y2 - y1
    if width < 80 or height < 100:
        return [component]
    crop = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(cv2.GaussianBlur(crop, (5, 5), 0), 35, 110)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180.0, threshold=max(35, width // 7),
        minLineLength=max(50, int(width * 0.45)), maxLineGap=max(15, width // 15),
    )
    if lines is None:
        return [component]
    seam_rows = []
    for lx1, ly1, lx2, ly2 in np.asarray(lines).reshape(-1, 4):
        angle = abs(float(np.degrees(np.arctan2(ly2 - ly1, lx2 - lx1))))
        angle = min(angle, 180.0 - angle)
        row = int(round((ly1 + ly2) / 2.0))
        if angle <= 14.0 and height * 0.18 <= row <= height * 0.82:
            seam_rows.append(row)
    if not seam_rows:
        return [component]
    seam_rows.sort()
    clustered = []
    for row in seam_rows:
        if not clustered or row - clustered[-1][-1] > 12:
            clustered.append([row])
        else:
            clustered[-1].append(row)
    candidates = [int(round(np.median(cluster))) for cluster in clustered]
    candidates.sort(key=lambda row: abs(row - height / 2.0))

    for relative_row in candidates:
        seam = y1 + relative_row
        upper = component.copy()
        upper[seam:, :] = False
        lower = component.copy()
        lower[:seam, :] = False
        if int(upper.sum()) < minimum_area_px or int(lower.sum()) < minimum_area_px:
            continue
        upper_height = physical_height_span(upper, xyz_map, support)
        lower_height = physical_height_span(lower, xyz_map, support)
        if upper_height < minimum_box_height_m or lower_height < minimum_box_height_m:
            continue  # likely a top/front face edge of one box, not two boxes
        return split_component_at_stack_seams(
            upper, frame, xyz_map, support, minimum_area_px, minimum_box_height_m, depth + 1
        ) + split_component_at_stack_seams(
            lower, frame, xyz_map, support, minimum_area_px, minimum_box_height_m, depth + 1
        )
    return [component]


def assign_tracks(
    detections: list[Detection], tracks: dict[int, Track], next_id: int, frame_index: int
) -> tuple[list[tuple[int, Detection]], int]:
    available = {track_id for track_id, track in tracks.items() if frame_index - track.last_frame <= 10}
    assigned = []
    for detection in sorted(detections, key=lambda item: item.bbox[0]):
        candidates = [
            (bbox_iou(detection.bbox, tracks[track_id].bbox), track_id)
            for track_id in available
        ]
        best_iou, track_id = max(candidates, default=(0.0, -1))
        if best_iou < 0.20:
            track_id = next_id
            next_id += 1
            tracks[track_id] = Track(track_id, detection.bbox, frame_index)
        else:
            available.remove(track_id)
        tracks[track_id].bbox = detection.bbox
        tracks[track_id].last_frame = frame_index
        assigned.append((track_id, detection))
    return assigned, next_id


def smooth_measurement(track: Track, detection: Detection) -> Measurement:
    cuboid = detection.cuboid
    track.values.append(
        np.asarray(
            [
                cuboid.length_m, cuboid.breadth_m, cuboid.height_m,
                cuboid.bottom_height_m, cuboid.top_height_m, cuboid.centre_depth_m,
                detection.wall_gap_m,
            ], dtype=np.float64,
        )
    )
    values = np.asarray(track.values)
    median = np.nanmedian(values, axis=0)
    return Measurement(
        track.track_id, detection.bbox,
        *map(float, median),
        cuboid.geometry_ratio, cuboid.reliable_3d, cuboid.point_count,
    )


def horizontal_overlap(first: tuple[int, int, int, int], second: tuple[int, int, int, int]) -> float:
    overlap = max(0, min(first[2], second[2]) - max(first[0], second[0]))
    return overlap / max(1, min(first[2] - first[0], second[2] - second[0]))


def infer_stacks(
    measurements: list[Measurement],
    maximum_height_gap_m: float = 0.045,
    minimum_overlap: float = 0.35,
) -> dict[int, int]:
    """Return upper ID -> supporting lower ID."""
    supports = {}
    for upper in measurements:
        best = None
        for lower in measurements:
            if upper.track_id == lower.track_id:
                continue
            if upper.bottom_height_m <= lower.bottom_height_m:
                continue
            overlap = horizontal_overlap(upper.bbox, lower.bbox)
            height_gap = abs(upper.bottom_height_m - lower.top_height_m)
            if overlap < minimum_overlap or height_gap > maximum_height_gap_m:
                continue
            score = overlap - height_gap / maximum_height_gap_m * 0.25
            if best is None or score > best[0]:
                best = (score, lower.track_id)
        if best is not None:
            supports[upper.track_id] = best[1]
    return supports


def stack_roots(measurements: list[Measurement], supports: dict[int, int]) -> dict[int, int]:
    result = {}
    for measurement in measurements:
        current = measurement.track_id
        visited = set()
        while current in supports and current not in visited:
            visited.add(current)
            current = supports[current]
        result[measurement.track_id] = current
    return result


def visible_stack_counts(roots: dict[int, int]) -> dict[int, int]:
    """Count visible, independently measured box tiers in each stack."""
    counts: dict[int, int] = {}
    for root in roots.values():
        counts[root] = counts.get(root, 0) + 1
    return counts


def save_calibration(
    path: Path, background_depth: np.ndarray, support: SupportPlane, wall: Plane | None,
    resolution: tuple[int, int], fx: float, fy: float,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        path,
        background_depth=background_depth.astype(np.float32),
        support_normal=support.normal,
        support_offset=support.offset,
        support_inlier_ratio=support.inlier_ratio,
        wall_normal=wall.normal if wall else np.full(3, np.nan),
        wall_offset=wall.offset if wall else np.nan,
        wall_inlier_ratio=wall.inlier_ratio if wall else 0.0,
        width=resolution[0], height=resolution[1], fx=fx, fy=fy,
    )


def load_calibration(path: Path) -> tuple[np.ndarray, SupportPlane, Plane | None, dict]:
    data = np.load(path)
    support = SupportPlane(
        np.asarray(data["support_normal"], dtype=np.float64),
        float(data["support_offset"]), float(data["support_inlier_ratio"]),
        float(data["support_offset"]),
    )
    wall_normal = np.asarray(data["wall_normal"], dtype=np.float64)
    wall = None
    if np.isfinite(wall_normal).all() and math.isfinite(float(data["wall_offset"])):
        wall = Plane(wall_normal, float(data["wall_offset"]), float(data["wall_inlier_ratio"]))
    metadata = {name: float(data[name]) for name in ("width", "height", "fx", "fy")}
    return np.asarray(data["background_depth"]), support, wall, metadata


def import_pyzed():
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise SystemExit(f"PyZED failed to load: {type(exc).__name__}: {exc}") from exc
    return sl


def draw_text(frame, text: str, origin: tuple[int, int], color: tuple[int, int, int]):
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.56, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.56, color, 2, cv2.LINE_AA)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", type=Path, default=ROOT / "outputs/box_scene_calibration.npz")
    parser.add_argument("--resolution", choices=("HD720", "HD1080", "HD2K"), default="HD2K")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--depth-mode", choices=("NEURAL_PLUS", "NEURAL", "ULTRA"), default="NEURAL_PLUS")
    parser.add_argument(
        "--serial", type=int, default=37406048,
        help="ZED serial to open; 0 accepts any connected ZED",
    )
    parser.add_argument("--camera-height-m", type=float,
                        help="Known perpendicular camera height above the same support plane")
    parser.add_argument("--foreground-threshold", type=float, default=0.045)
    parser.add_argument("--min-area", type=int, default=4500)
    parser.add_argument(
        "--border-margin", type=int, default=45,
        help="Ignore foreground components touching this many image-edge pixels",
    )
    parser.add_argument("--background-frames", type=int, default=8)
    parser.add_argument("--capture-background", action="store_true")
    parser.add_argument("--max-boxes", type=int, default=12)
    parser.add_argument(
        "--overhead", action="store_true",
        help="Straight-down camera: measure top elevation over the floor and visible stack tiers",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    sl = import_pyzed()
    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_fps = args.fps
    init.depth_mode = getattr(sl.DEPTH_MODE, args.depth_mode)
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = 0.20
    init.depth_maximum_distance = 12.0
    if args.serial:
        init.set_from_serial_number(args.serial)
    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"Cannot open ZED: {status}")

    info = camera.get_camera_information()
    connected_serial = int(info.serial_number)
    if args.serial and connected_serial != args.serial:
        camera.close()
        raise SystemExit(
            f"Opened ZED serial {connected_serial}, expected {args.serial}; refusing mismatched calibration"
        )
    print(f"Opened ZED serial {connected_serial}; SDK calibration SN{connected_serial}.conf")
    calibration_info = info.camera_configuration.calibration_parameters.left_cam
    resolution = info.camera_configuration.resolution
    fx, fy = float(calibration_info.fx), float(calibration_info.fy)
    image, depth, point_cloud = sl.Mat(), sl.Mat(), sl.Mat()
    runtime = sl.RuntimeParameters()
    runtime.enable_fill_mode = True

    background_depth = None
    support = None
    wall = None
    if args.calibration.is_file():
        background_depth, support, wall, metadata = load_calibration(args.calibration)
        if background_depth.shape != (resolution.height, resolution.width):
            raise SystemExit(
                f"Calibration is {background_depth.shape[::-1]}, camera is "
                f"{(resolution.width, resolution.height)}. Re-capture calibration."
            )
        print(
            f"Loaded calibration: support_height={support.camera_height_m:.3f}m "
            f"wall={'yes' if wall else 'no'}"
        )

    capture_remaining = args.background_frames if args.capture_background else 0
    capture_depths: list[np.ndarray] = []
    last_xyz = None
    metric_scale = 1.0
    tracks: dict[int, Track] = {}
    next_track_id = 1
    frame_index = 0
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = args.csv or ROOT / "outputs" / f"box_cuboids_{timestamp}.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = [
        "timestamp_utc", "frame", "box_id", "x1", "y1", "x2", "y2",
        "length_cm", "breadth_cm", "height_cm", "centre_depth_m", "wall_gap_cm",
        "bottom_height_cm", "top_height_cm", "reliable_3d", "geometry_ratio",
        "point_count", "support_box_id", "stack_id", "visible_box_count", "metric_scale",
    ]

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as stream:
            table = csv.DictWriter(stream, fieldnames=fieldnames)
            table.writeheader()
            while args.max_frames <= 0 or frame_index < args.max_frames:
                grab_status = camera.grab(runtime)
                if grab_status != sl.ERROR_CODE.SUCCESS:
                    continue
                camera.retrieve_image(image, sl.VIEW.LEFT)
                camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
                camera.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                frame = cv2.cvtColor(np.asarray(image.get_data()), cv2.COLOR_BGRA2BGR)
                depth_map = np.asarray(depth.get_data()).copy()
                xyz_map = np.asarray(point_cloud.get_data())

                if capture_remaining > 0:
                    capture_depths.append(depth_map)
                    last_xyz = xyz_map.copy()
                    capture_remaining -= 1
                    draw_text(
                        frame, f"EMPTY SCENE calibration: {capture_remaining} frames left",
                        (20, 35), (0, 255, 255),
                    )
                    if capture_remaining == 0:
                        with np.errstate(all="ignore"):
                            background_depth = np.nanmedian(
                                np.stack(capture_depths), axis=0
                            ).astype(np.float32)
                        if args.overhead:
                            if not args.camera_height_m:
                                raise SystemExit(
                                    "Overhead calibration requires --camera-height-m"
                                )
                            support_samples = overhead_support_plane_points(
                                last_xyz, border_px=args.border_margin
                            )
                            support = fit_overhead_support_plane(
                                support_samples, args.camera_height_m
                            )
                        else:
                            support_samples = support_plane_points(last_xyz, [])
                            support = fit_support_plane(support_samples, threshold_m=0.010)
                        if support is None:
                            raise SystemExit("Could not fit the empty support plane")
                        if args.overhead:
                            wall = None
                        else:
                            wall_samples = wall_plane_points(last_xyz, support)
                            wall = fit_wall_plane(wall_samples, support.normal)
                        save_calibration(
                            args.calibration, background_depth, support, wall,
                            (resolution.width, resolution.height), fx, fy,
                        )
                        print(
                            f"Saved calibration {args.calibration}: "
                            f"support_height={support.camera_height_m:.3f}m "
                            f"wall={'yes' if wall else 'no'}"
                        )
                        capture_depths.clear()

                measurements: list[Measurement] = []
                if background_depth is None or support is None:
                    draw_text(
                        frame,
                        (
                            "REMOVE ALL BOXES, then press B to calibrate the floor"
                            if args.overhead
                            else "REMOVE ALL BOXES, then press B to calibrate floor/table + wall"
                        ),
                        (20, 35), (0, 255, 255),
                    )
                elif capture_remaining == 0:
                    metric_scale = (
                        args.camera_height_m / support.camera_height_m
                        if args.camera_height_m else 1.0
                    )
                    mask = foreground_mask(
                        background_depth, depth_map, args.foreground_threshold,
                        0.20, 12.0, kernel_size=5,
                    )
                    raw_components = component_boxes(mask, args.min_area)
                    component_items: list[tuple[np.ndarray, tuple[float, float] | None]] = []
                    for _bbox, component, _area in raw_components:
                        if args.overhead:
                            tiers = overhead_height_tiers(
                                component, xyz_map, support, metric_scale, args.min_area
                            )
                            component_items.extend(tiers)
                        else:
                            component_items.extend(
                                (item, None) for item in split_component_at_stack_seams(
                                    component, frame, xyz_map, support, args.min_area
                                )
                            )
                    detections: list[Detection] = []
                    for component, height_bounds in component_items[: args.max_boxes]:
                        bbox = mask_bbox(component)
                        if bbox is None:
                            continue
                        x1, y1, x2, y2 = bbox
                        if (
                            x1 <= args.border_margin or y1 <= args.border_margin
                            or x2 >= resolution.width - args.border_margin
                            or y2 >= resolution.height - args.border_margin
                        ):
                            continue
                        points = points_for_component(xyz_map, component, stride=2, max_points=30000)
                        cuboid = fit_support_aligned_cuboid(
                            points, support, metric_scale, height_bounds_m=height_bounds
                        )
                        if cuboid is None:
                            continue
                        if cuboid.length_m > 4.0 or cuboid.height_m > 3.0:
                            continue
                        detections.append(
                            Detection(bbox, cuboid, observed_wall_gap(points, wall, metric_scale))
                        )
                    assigned, next_track_id = assign_tracks(
                        detections, tracks, next_track_id, frame_index
                    )
                    measurements = [smooth_measurement(tracks[track_id], detection) for track_id, detection in assigned]
                    supports = infer_stacks(measurements)
                    roots = stack_roots(measurements, supports)
                    stack_counts = visible_stack_counts(roots)
                    for measurement in measurements:
                        support_id = supports.get(measurement.track_id)
                        stack_root = roots[measurement.track_id]
                        visible_count = stack_counts[stack_root]
                        color = (0, 220, 80) if measurement.reliable_3d else (0, 180, 255)
                        x1, y1, x2, y2 = measurement.bbox
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        relation = (
                            f"ON Box {support_id} | Stack {stack_root} | Visible count {visible_count}"
                            if support_id is not None
                            else f"Base | Stack {stack_root} | Visible count {visible_count}"
                        )
                        draw_text(frame, f"Box {measurement.track_id} | {relation}", (x1, max(22, y1 - 48)), color)
                        if measurement.reliable_3d:
                            dimensions = (
                                f"LxBxH {measurement.length_m * 100:.1f} x "
                                f"{measurement.breadth_m * 100:.1f} x "
                                f"{measurement.height_m * 100:.1f} cm"
                            )
                        else:
                            dimensions = "LxBxH unreliable: not enough visible 3D faces"
                        draw_text(frame, dimensions, (x1, max(44, y1 - 25)), color)
                        if args.overhead:
                            gap_text = f"top {measurement.top_height_m * 100:.1f} cm above floor"
                        else:
                            gap_text = (
                                f"wall gap {measurement.wall_gap_m * 100:.1f} cm"
                                if math.isfinite(measurement.wall_gap_m) else "wall gap unavailable"
                            )
                        draw_text(frame, gap_text, (x1, max(66, y1 - 2)), color)
                        table.writerow(
                            {
                                "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                                "frame": frame_index, "box_id": measurement.track_id,
                                "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                                "length_cm": measurement.length_m * 100,
                                "breadth_cm": measurement.breadth_m * 100,
                                "height_cm": measurement.height_m * 100,
                                "centre_depth_m": measurement.centre_depth_m,
                                "wall_gap_cm": measurement.wall_gap_m * 100,
                                "bottom_height_cm": measurement.bottom_height_m * 100,
                                "top_height_cm": measurement.top_height_m * 100,
                                "reliable_3d": measurement.reliable_3d,
                                "geometry_ratio": measurement.geometry_ratio,
                                "point_count": measurement.point_count,
                                "support_box_id": support_id if support_id is not None else "",
                                "stack_id": stack_root,
                                "visible_box_count": visible_count,
                                "metric_scale": metric_scale,
                            }
                        )
                    draw_text(
                        frame,
                        f"support {support.camera_height_m:.3f}m | "
                        f"{'overhead floor mode' if args.overhead else ('wall ok' if wall else 'wall missing')} "
                        f"| visible boxes {len(measurements)} | stacks {len(stack_counts)}",
                        (20, 35), (255, 255, 255),
                    )

                frame_index += 1
                if frame_index % 30 == 0:
                    stream.flush()
                for track_id in [
                    track_id for track_id, track in tracks.items()
                    if frame_index - track.last_frame > 90
                ]:
                    del tracks[track_id]

                if not args.headless:
                    draw_text(frame, "B: recapture empty scene | Q: quit", (20, frame.shape[0] - 20), (255, 255, 255))
                    cv2.imshow("ZED2i Box Cuboids + Wall + Stacks", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break
                    if key == ord("b"):
                        capture_depths.clear()
                        capture_remaining = args.background_frames
                        background_depth = None
                        support = None
                        wall = None
                        print("Remove all boxes: capturing empty scene")
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()
        cv2.destroyAllWindows()
    print(f"frames={frame_index} csv={csv_path} calibration={args.calibration}")


if __name__ == "__main__":
    main()
