#!/usr/bin/env python3
"""Model-free ZED2i box detection and L x B x D estimation.

Boxes are detected as connected foreground regions against an empty-scene
depth reference.  Their dimensions are estimated from the ZED XYZ point cloud;
no trained detector or reference object is used.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import defaultdict, deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent


@dataclass(frozen=True)
class CuboidEstimate:
    length_m: float
    breadth_m: float
    depth_m: float
    third_dimension_reliable: bool
    geometry_ratio: float
    plane_inlier_fraction: float
    point_count: int


@dataclass
class Track:
    bbox: tuple[int, int, int, int]
    last_frame: int
    dimensions: deque[tuple[float, float, float]]
    heights: deque[tuple[float, float]]
    depths: deque[float]


@dataclass(frozen=True)
class SupportPlane:
    normal: np.ndarray
    offset: float
    inlier_ratio: float
    camera_height_m: float


@dataclass(frozen=True)
class BoxObservation:
    track_id: int
    bbox: tuple[int, int, int, int]
    length_m: float
    breadth_m: float
    depth_m: float
    median_depth_m: float
    valid_depth_fraction: float
    bottom_height_m: float
    top_height_m: float
    estimate: CuboidEstimate


def valid_depth_mask(depth: np.ndarray, min_depth_m: float, max_depth_m: float) -> np.ndarray:
    return np.isfinite(depth) & (depth >= min_depth_m) & (depth <= max_depth_m)


def foreground_mask(
    background_depth: np.ndarray,
    current_depth: np.ndarray,
    threshold_m: float,
    min_depth_m: float,
    max_depth_m: float,
    kernel_size: int = 5,
) -> np.ndarray:
    """Return pixels sufficiently nearer than the empty-scene background."""
    background_valid = valid_depth_mask(background_depth, min_depth_m, max_depth_m)
    current_valid = valid_depth_mask(current_depth, min_depth_m, max_depth_m)
    mask = background_valid & current_valid & ((background_depth - current_depth) >= threshold_m)
    output = mask.astype(np.uint8) * 255
    kernel_size = max(3, kernel_size | 1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
    output = cv2.morphologyEx(output, cv2.MORPH_OPEN, kernel)
    output = cv2.morphologyEx(output, cv2.MORPH_CLOSE, kernel, iterations=2)
    return output


def component_boxes(mask: np.ndarray, min_area_px: int, max_area_fraction: float = 0.85):
    """Yield (bbox, component_mask, area) for plausible foreground objects."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    image_area = mask.shape[0] * mask.shape[1]
    components = []
    for label in range(1, count):
        x, y, width, height, area = (int(value) for value in stats[label])
        if area < min_area_px or area > image_area * max_area_fraction:
            continue
        if width < 12 or height < 12:
            continue
        components.append(((x, y, x + width, y + height), labels == label, area))
    components.sort(key=lambda item: item[0][0])
    return components


def points_for_component(
    xyz_map: np.ndarray,
    component: np.ndarray,
    stride: int = 2,
    max_points: int = 20000,
) -> np.ndarray:
    sampled_xyz = np.asarray(xyz_map[::stride, ::stride, :3], dtype=np.float64)
    sampled_mask = component[::stride, ::stride]
    points = sampled_xyz[sampled_mask]
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) > max_points:
        rng = np.random.default_rng(11)
        points = points[rng.choice(len(points), max_points, replace=False)]
    return points


def support_plane_points(
    xyz_map: np.ndarray,
    excluded_boxes: list[tuple[int, int, int, int]],
    start_row_fraction: float = 0.48,
    stride: int = 4,
    max_points: int = 12000,
) -> np.ndarray:
    """Sample the lower scene while excluding detected boxes."""
    height, width = xyz_map.shape[:2]
    start_row = int(round(height * start_row_fraction))
    include = np.zeros((height, width), dtype=bool)
    include[start_row:, :] = True
    for x1, y1, x2, y2 in excluded_boxes:
        margin_x = max(5, int((x2 - x1) * 0.05))
        margin_y = max(5, int((y2 - y1) * 0.05))
        include[
            max(0, y1 - margin_y):min(height, y2 + margin_y),
            max(0, x1 - margin_x):min(width, x2 + margin_x),
        ] = False
    points = np.asarray(xyz_map[::stride, ::stride, :3], dtype=np.float64)[include[::stride, ::stride]]
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 2] >= 0.20) & (points[:, 2] <= 8.0)]
    if len(points) > max_points:
        rng = np.random.default_rng(17)
        points = points[rng.choice(len(points), max_points, replace=False)]
    return points


def fit_support_plane(
    points_xyz: np.ndarray,
    threshold_m: float = 0.010,
    iterations: int = 120,
    minimum_points: int = 300,
    minimum_abs_camera_y: float = 0.25,
    rng_seed: int = 19,
) -> SupportPlane | None:
    """Fit the dominant floor/table plane using constrained RANSAC."""
    points = np.asarray(points_xyz, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < minimum_points:
        return None
    rng = np.random.default_rng(rng_seed)
    best_mask = None
    best_score = -1.0
    for _ in range(iterations):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal /= norm
        if abs(float(normal[1])) < minimum_abs_camera_y:
            continue
        offset = -float(normal @ sample[0])
        distances = np.abs(points @ normal + offset)
        mask = distances <= threshold_m
        score = float(mask.sum()) * (0.60 + 0.40 * abs(float(normal[1])))
        if score > best_score:
            best_score = score
            best_mask = mask
    if best_mask is None or int(best_mask.sum()) < minimum_points:
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
    if not 0.05 <= camera_height <= 5.0:
        return None
    residuals = np.abs(points @ normal + offset)
    inlier_ratio = float(np.mean(residuals <= threshold_m))
    return SupportPlane(normal, offset, inlier_ratio, camera_height)


def heights_above_support(
    points_xyz: np.ndarray,
    plane: SupportPlane | None,
    metric_scale: float = 1.0,
) -> tuple[float, float]:
    if plane is None or len(points_xyz) < 30:
        return math.nan, math.nan
    heights = (np.asarray(points_xyz) @ plane.normal + plane.offset) * metric_scale
    heights = heights[np.isfinite(heights)]
    heights = heights[(heights >= -0.05) & (heights <= 5.0)]
    if len(heights) < 30:
        return math.nan, math.nan
    bottom, top = np.percentile(heights, [1.0, 99.0])
    return max(0.0, float(bottom)), max(0.0, float(top))


def clip_bbox(box: tuple[int, int, int, int], width: int, height: int) -> tuple[int, int, int, int]:
    x, y, box_width, box_height = (int(value) for value in box)
    x1 = max(0, min(x, width - 1))
    y1 = max(0, min(y, height - 1))
    x2 = max(x1 + 1, min(x + box_width, width))
    y2 = max(y1 + 1, min(y + box_height, height))
    return x1, y1, x2, y2


def median_depth_in_bbox(
    depth_map: np.ndarray,
    bbox: tuple[int, int, int, int],
    min_depth_m: float,
    max_depth_m: float,
    centre_fraction: float = 0.35,
) -> tuple[float, float]:
    """Median depth near the bbox centre, avoiding background around its edges."""
    x1, y1, x2, y2 = bbox
    width, height = x2 - x1, y2 - y1
    roi_width = max(5, int(round(width * centre_fraction)))
    roi_height = max(5, int(round(height * centre_fraction)))
    centre_x, centre_y = (x1 + x2) // 2, (y1 + y2) // 2
    rx1 = max(x1, centre_x - roi_width // 2)
    ry1 = max(y1, centre_y - roi_height // 2)
    rx2 = min(x2, rx1 + roi_width)
    ry2 = min(y2, ry1 + roi_height)
    values = np.asarray(depth_map[ry1:ry2, rx1:rx2], dtype=np.float64).reshape(-1)
    valid = np.isfinite(values) & (values >= min_depth_m) & (values <= max_depth_m)
    if not valid.any():
        return math.nan, 0.0
    return float(np.median(values[valid])), float(valid.mean())


def visible_face_dimensions(
    bbox: tuple[int, int, int, int], depth_m: float, fx_px: float, fy_px: float
) -> tuple[float, float]:
    """Calibrated bbox width/height in metres at the median face depth."""
    if not math.isfinite(depth_m) or depth_m <= 0.0:
        return math.nan, math.nan
    x1, y1, x2, y2 = bbox
    return (x2 - x1) * depth_m / fx_px, (y2 - y1) * depth_m / fy_px


def points_for_bbox(
    xyz_map: np.ndarray,
    bbox: tuple[int, int, int, int],
    centre_depth_m: float,
    depth_band_m: float,
    inset_fraction: float = 0.025,
    stride: int = 2,
    max_points: int = 20000,
) -> np.ndarray:
    """Return depth-gated XYZ samples from a tight image bbox."""
    if not math.isfinite(centre_depth_m):
        return np.empty((0, 3), dtype=np.float64)
    x1, y1, x2, y2 = bbox
    inset_x = max(1, int(round((x2 - x1) * inset_fraction)))
    inset_y = max(1, int(round((y2 - y1) * inset_fraction)))
    x1, x2 = x1 + inset_x, x2 - inset_x
    y1, y2 = y1 + inset_y, y2 - inset_y
    if x2 <= x1 or y2 <= y1:
        return np.empty((0, 3), dtype=np.float64)
    points = np.asarray(xyz_map[y1:y2:stride, x1:x2:stride, :3], dtype=np.float64).reshape(-1, 3)
    valid = np.isfinite(points).all(axis=1)
    if math.isfinite(centre_depth_m):
        valid &= np.abs(points[:, 2] - centre_depth_m) <= depth_band_m
    points = points[valid]
    if len(points) > max_points:
        rng = np.random.default_rng(13)
        points = points[rng.choice(len(points), max_points, replace=False)]
    return points


def longest_true_run(values: np.ndarray) -> tuple[int, int]:
    """Return [start, end) of the longest contiguous True run."""
    best_start = best_end = current_start = 0
    for index, present in enumerate(np.append(values.astype(bool), False)):
        if present:
            continue
        if index - current_start > best_end - best_start:
            best_start, best_end = current_start, index
        current_start = index + 1
    return best_start, best_end


def refine_box_by_colour_occupancy(
    frame: np.ndarray,
    coarse_bbox: tuple[int, int, int, int],
    colour_threshold: float = 15.0,
) -> tuple[int, int, int, int]:
    """Remove adjacent clutter from a coarse rectangle using its centre colour."""
    x1, y1, x2, y2 = coarse_bbox
    centre_x, centre_y = (x1 + x2) // 2, (y1 + y2) // 2
    patch_radius = max(8, min(x2 - x1, y2 - y1) // 10)
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
    patch = lab[
        max(y1, centre_y - patch_radius):min(y2, centre_y + patch_radius),
        max(x1, centre_x - patch_radius):min(x2, centre_x + patch_radius),
    ]
    if patch.size == 0:
        return coarse_bbox
    reference = np.median(patch.reshape(-1, 3), axis=0)
    weights = np.array([0.55, 1.0, 1.0], dtype=np.float32)
    crop = lab[y1:y2, x1:x2]
    distance = np.linalg.norm((crop - reference) * weights, axis=2)
    colour_mask = distance <= colour_threshold

    column_present = colour_mask.sum(axis=0) >= colour_mask.shape[0] * 0.58
    relative_x1, relative_x2 = longest_true_run(column_present)
    if relative_x2 - relative_x1 < 30:
        return coarse_bbox
    selected_columns = colour_mask[:, relative_x1:relative_x2]
    row_present = selected_columns.sum(axis=1) >= selected_columns.shape[1] * 0.15
    relative_y1, relative_y2 = longest_true_run(row_present)
    if relative_y2 - relative_y1 < 30:
        return coarse_bbox
    return x1 + relative_x1, y1 + relative_y1, x1 + relative_x2, y1 + relative_y2


def detect_rectangular_boxes(
    frame: np.ndarray,
    min_area_px: int = 8000,
    max_boxes: int = 1,
) -> list[tuple[int, int, int, int]]:
    """Automatically detect and refine the strongest rectangular box faces."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 7, 35, 35)
    edges = cv2.Canny(gray, 30, 95)
    edges = cv2.morphologyEx(
        edges,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)),
        iterations=2,
    )
    contours, _ = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    image_height, image_width = gray.shape
    candidates: list[tuple[float, tuple[int, int, int, int]]] = []
    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        approximation = cv2.approxPolyDP(contour, 0.018 * perimeter, True)
        x, y, width, height = cv2.boundingRect(contour)
        bbox_area = width * height
        if bbox_area < min_area_px or bbox_area > image_width * image_height * 0.65:
            continue
        if width < 60 or height < 45:
            continue
        if x <= 4 or y <= 4 or x + width >= image_width - 4 or y + height >= image_height - 4:
            continue
        rotated_width, rotated_height = cv2.minAreaRect(contour)[1]
        rotated_area = rotated_width * rotated_height
        if rotated_area <= 0:
            continue
        rectangularity = abs(cv2.contourArea(contour)) / rotated_area
        aspect = max(rotated_width, rotated_height) / max(1.0, min(rotated_width, rotated_height))
        if rectangularity < 0.55 or aspect > 8.0 or len(approximation) > 10:
            continue
        centre_bonus = 1.0 - min(
            1.0, abs((x + width / 2.0) - image_width / 2.0) / (image_width / 2.0)
        ) * 0.15
        score = rotated_area * rectangularity * centre_bonus
        candidates.append((score, (x, y, x + width, y + height)))

    candidates.sort(reverse=True, key=lambda item: item[0])
    selected: list[tuple[int, int, int, int]] = []
    for _, coarse_bbox in candidates:
        refined_bbox = refine_box_by_colour_occupancy(frame, coarse_bbox)
        if any(bbox_iou(refined_bbox, existing) > 0.55 for existing in selected):
            continue
        selected.append(refined_bbox)
        if len(selected) >= max_boxes:
            break
    return selected


def robust_oriented_cuboid(
    points_xyz: np.ndarray,
    trim_percent: float = 0.5,
    minimum_points: int = 150,
    reliable_ratio: float = 0.18,
    minimum_third_dimension_m: float = 0.025,
    plane_threshold_m: float = 0.012,
    maximum_plane_fraction: float = 0.85,
) -> CuboidEstimate:
    """Estimate sorted cuboid extents with a robust PCA-oriented bounding box.

    The singular-value ratio reports whether the visible points span three
    dimensions.  A nearly planar front face cannot reveal the hidden thickness.
    """
    points = np.asarray(points_xyz, dtype=np.float64)
    points = points[np.isfinite(points).all(axis=1)]
    if len(points) < minimum_points:
        return CuboidEstimate(math.nan, math.nan, math.nan, False, 0.0, 1.0, len(points))

    centre = np.median(points, axis=0)
    centred = points - centre
    _, singular_values, axes = np.linalg.svd(centred, full_matrices=False)
    projected = centred @ axes.T
    lower, upper = np.percentile(projected, [trim_percent, 100.0 - trim_percent], axis=0)
    extents = np.abs(upper - lower)
    length_m, breadth_m, depth_m = sorted((float(value) for value in extents), reverse=True)
    geometry_ratio = float(singular_values[-1] / max(singular_values[-2], 1e-12))
    plane_distances = np.abs(projected[:, 2] - np.median(projected[:, 2]))
    plane_inlier_fraction = float(np.mean(plane_distances <= plane_threshold_m))
    reliable = (
        geometry_ratio >= reliable_ratio
        and depth_m >= minimum_third_dimension_m
        and plane_inlier_fraction <= maximum_plane_fraction
    )
    if not reliable:
        depth_m = math.nan
    return CuboidEstimate(
        length_m, breadth_m, depth_m, reliable, geometry_ratio,
        plane_inlier_fraction, len(points)
    )


def bbox_iou(first: tuple[int, int, int, int], second: tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = first
    bx1, by1, bx2, by2 = second
    intersection = max(0, min(ax2, bx2) - max(ax1, bx1)) * max(0, min(ay2, by2) - max(ay1, by1))
    first_area = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    second_area = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


def assign_track(
    bbox: tuple[int, int, int, int],
    tracks: dict[int, Track],
    frame_index: int,
    next_track_id: int,
    used_track_ids: set[int] | None = None,
) -> tuple[int, int]:
    used_track_ids = used_track_ids or set()
    candidates = [
        (bbox_iou(bbox, track.bbox), track_id)
        for track_id, track in tracks.items()
        if frame_index - track.last_frame <= 8 and track_id not in used_track_ids
    ]
    best_iou, best_id = max(candidates, default=(0.0, -1))
    if best_iou < 0.25:
        best_id = next_track_id
        next_track_id += 1
        tracks[best_id] = Track(
            bbox, frame_index, deque(maxlen=20), deque(maxlen=20), deque(maxlen=20)
        )
    else:
        tracks[best_id].bbox = bbox
        tracks[best_id].last_frame = frame_index
    return best_id, next_track_id


def horizontal_overlap_ratio(
    first: tuple[int, int, int, int], second: tuple[int, int, int, int]
) -> float:
    overlap = max(0, min(first[2], second[2]) - max(first[0], second[0]))
    return overlap / max(1, min(first[2] - first[0], second[2] - second[0]))


def infer_stack_supports(
    observations: list[BoxObservation],
    maximum_height_gap_m: float = 0.045,
    maximum_depth_difference_m: float = 0.12,
    minimum_horizontal_overlap: float = 0.35,
    maximum_image_gap_fraction: float = 0.18,
) -> dict[int, int]:
    """Map upper box ID to the lower box ID supporting it."""
    supports: dict[int, int] = {}
    for upper in observations:
        upper_x1, upper_y1, upper_x2, upper_y2 = upper.bbox
        upper_height_px = upper_y2 - upper_y1
        best: tuple[float, int] | None = None
        for lower in observations:
            if lower.track_id == upper.track_id:
                continue
            lower_x1, lower_y1, lower_x2, lower_y2 = lower.bbox
            lower_height_px = lower_y2 - lower_y1
            if (upper_y1 + upper_y2) / 2.0 >= (lower_y1 + lower_y2) / 2.0:
                continue
            overlap = horizontal_overlap_ratio(upper.bbox, lower.bbox)
            if overlap < minimum_horizontal_overlap:
                continue
            image_gap = abs(upper_y2 - lower_y1)
            maximum_image_gap = max(
                12.0, min(upper_height_px, lower_height_px) * maximum_image_gap_fraction
            )
            if image_gap > maximum_image_gap:
                continue
            if (
                math.isfinite(upper.median_depth_m)
                and math.isfinite(lower.median_depth_m)
                and abs(upper.median_depth_m - lower.median_depth_m) > maximum_depth_difference_m
            ):
                continue
            height_gap = math.nan
            if math.isfinite(upper.bottom_height_m) and math.isfinite(lower.top_height_m):
                height_gap = abs(upper.bottom_height_m - lower.top_height_m)
                if height_gap > maximum_height_gap_m:
                    continue
            score = overlap - image_gap / max(1.0, maximum_image_gap) * 0.15
            if math.isfinite(height_gap):
                score -= height_gap / maximum_height_gap_m * 0.25
            if best is None or score > best[0]:
                best = (score, lower.track_id)
        if best is not None:
            supports[upper.track_id] = best[1]
    return supports


def assign_stack_ids(observations: list[BoxObservation], supports: dict[int, int]) -> dict[int, int]:
    """Assign each connected support chain the ID of its bottom box."""
    result: dict[int, int] = {}
    valid_ids = {observation.track_id for observation in observations}
    for observation in observations:
        current = observation.track_id
        visited = set()
        while current in supports and current not in visited:
            visited.add(current)
            current = supports[current]
        result[observation.track_id] = current if current in valid_ids else observation.track_id
    return result


def import_pyzed():
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise SystemExit(
            "PyZED could not load. Run ./setup_zed_box_env.sh so the pyzed wheel "
            "matches the installed ZED SDK.\n"
            f"Original error: {type(exc).__name__}: {exc}"
        ) from exc
    return sl


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--svo", type=Path, help="Use a recorded SVO instead of a live ZED2i")
    parser.add_argument("--resolution", choices=("HD720", "HD1080", "HD2K"), default="HD2K")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument(
        "--depth-mode",
        choices=("NEURAL_PLUS", "NEURAL", "ULTRA", "QUALITY", "PERFORMANCE"),
        default="NEURAL_PLUS",
    )
    parser.add_argument("--mode", choices=("auto", "background"), default="auto",
                        help="auto: find the strongest rectangle; background: detect multiple foreground components")
    parser.add_argument("--auto-min-area", type=int, default=8000)
    parser.add_argument("--max-boxes", type=int, default=4)
    parser.add_argument("--camera-height-m", type=float,
                        help="Camera height above the detected support plane for metric scale correction")
    parser.add_argument("--support-plane-refresh", type=int, default=30,
                        help="Refit the fixed floor/table plane every N frames")
    parser.add_argument("--support-plane-threshold", type=float, default=0.010)
    parser.add_argument("--stack-height-gap", type=float, default=0.045)
    parser.add_argument("--stack-depth-gap", type=float, default=0.12)
    parser.add_argument("--roi-depth-band", type=float, default=0.45,
                        help="Keep bbox points within this many metres of centre depth")
    parser.add_argument("--min-valid-depth", type=float, default=0.35,
                        help="Suppress measurements when less than this fraction of centre depth is valid")
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=8.0)
    parser.add_argument("--foreground-threshold", type=float, default=0.06,
                        help="Object must be this many metres nearer than the empty scene")
    parser.add_argument("--min-area", type=int, default=1200, help="Minimum foreground component area in pixels")
    parser.add_argument("--background-frames", type=int, default=15)
    parser.add_argument("--background", type=Path, help="Load/save empty-scene depth as a .npy file")
    parser.add_argument("--auto-background", action="store_true",
                        help="Capture background immediately; scene must initially be empty")
    parser.add_argument("--reliable-ratio", type=float, default=0.18)
    parser.add_argument("--min-third-dimension", type=float, default=0.025)
    parser.add_argument("--csv", type=Path, help="Output CSV; defaults under outputs/")
    parser.add_argument("--record", type=Path, help="Optional annotated MP4")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--check-only", action="store_true")
    return parser


def open_camera(sl, args: argparse.Namespace):
    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_fps = args.fps
    init.depth_mode = getattr(sl.DEPTH_MODE, args.depth_mode)
    init.coordinate_units = sl.UNIT.METER
    init.depth_minimum_distance = args.min_depth
    init.depth_maximum_distance = args.max_depth
    if args.svo:
        if not args.svo.is_file():
            raise SystemExit(f"SVO file not found: {args.svo}")
        init.set_from_svo_file(str(args.svo))
    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(
            f"Cannot open ZED camera ({status}). Check the ZED2i USB 3/power connection "
            "and close other applications using the camera."
        )
    return camera


def draw_label(frame: np.ndarray, bbox: tuple[int, int, int, int], lines: list[str], reliable: bool):
    x1, y1, x2, y2 = bbox
    color = (60, 220, 60) if reliable else (0, 185, 255)
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    start_y = max(24, y1 - 8 - (len(lines) - 1) * 23)
    for index, line in enumerate(lines):
        y = start_y + index * 23
        cv2.putText(frame, line, (x1, y), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(frame, line, (x1, y), cv2.FONT_HERSHEY_SIMPLEX, 0.60, color, 2, cv2.LINE_AA)


def parse_roi(value: str) -> tuple[int, int, int, int]:
    try:
        parts = tuple(int(part.strip()) for part in value.split(","))
    except ValueError as exc:
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height") from exc
    if len(parts) != 4 or parts[2] <= 0 or parts[3] <= 0:
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height with positive width and height")
    return parts


def main() -> None:
    args = build_parser().parse_args()
    if args.headless and args.mode == "background" and not (args.background or args.auto_background):
        raise SystemExit("Headless background mode needs --background FILE or --auto-background")
    sl = import_pyzed()
    camera = open_camera(sl, args)
    try:
        info = camera.get_camera_information()
        calibration = info.camera_configuration.calibration_parameters.left_cam
        resolution = info.camera_configuration.resolution
        fx_px, fy_px = float(calibration.fx), float(calibration.fy)
        print(
            f"ZED opened: model={info.camera_model} serial={info.serial_number} "
            f"resolution={resolution.width}x{resolution.height} "
            f"fx={fx_px:.3f} fy={fy_px:.3f} mode={args.mode}"
        )
        if args.check_only:
            return

        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()
        runtime = sl.RuntimeParameters()
        runtime.enable_fill_mode = True
        background_depth = None
        if args.background and args.background.is_file():
            background_depth = np.load(args.background)
            print(f"Loaded background: {args.background}")

        capture_remaining = (
            args.background_frames
            if args.mode == "background" and args.auto_background and background_depth is None
            else 0
        )
        background_samples: list[np.ndarray] = []
        support_plane: SupportPlane | None = None
        metric_scale = 1.0
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = args.csv or ROOT / "outputs" / f"zed_geometry_boxes_{timestamp}.csv"
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        if args.record:
            args.record.parent.mkdir(parents=True, exist_ok=True)

        tracks: dict[int, Track] = {}
        next_track_id = 1
        writer = None
        frame_index = 0
        start_time = time.monotonic()
        fieldnames = [
            "timestamp_utc", "frame", "elapsed_s", "box_id", "x1", "y1", "x2", "y2",
            "length_cm", "breadth_cm", "depth_cm", "third_dimension_reliable",
            "geometry_ratio", "plane_inlier_fraction", "point_count", "foreground_area_px",
            "median_depth_m", "valid_depth_fraction", "measurement_mode",
            "bottom_height_cm", "top_height_cm", "support_box_id", "stack_id",
            "support_plane_camera_height_m", "metric_scale",
        ]
        with csv_path.open("w", newline="", encoding="utf-8") as csv_stream:
            table = csv.DictWriter(csv_stream, fieldnames=fieldnames)
            table.writeheader()
            try:
                while args.max_frames <= 0 or frame_index < args.max_frames:
                    grab_status = camera.grab(runtime)
                    if grab_status != sl.ERROR_CODE.SUCCESS:
                        if args.svo and grab_status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                            break
                        print(f"ZED grab skipped: {grab_status}", file=sys.stderr)
                        continue
                    camera.retrieve_image(image, sl.VIEW.LEFT)
                    camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
                    camera.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
                    frame = cv2.cvtColor(np.asarray(image.get_data()), cv2.COLOR_BGRA2BGR)
                    depth_map = np.asarray(depth.get_data()).copy()
                    xyz_map = np.asarray(point_cloud.get_data())
                    selection_frame = frame.copy()

                    if args.mode == "background" and capture_remaining > 0:
                        background_samples.append(depth_map)
                        capture_remaining -= 1
                        cv2.putText(
                            frame,
                            f"Capturing empty background: {capture_remaining} frames left",
                            (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA,
                        )
                        if capture_remaining == 0:
                            with np.errstate(all="ignore"):
                                background_depth = np.nanmedian(np.stack(background_samples), axis=0).astype(np.float32)
                            background_samples.clear()
                            if args.background:
                                args.background.parent.mkdir(parents=True, exist_ok=True)
                                np.save(args.background, background_depth)
                                print(f"Saved background: {args.background}")
                            print("Background captured. Place boxes in view.")

                    if args.mode == "auto":
                        detected_boxes = detect_rectangular_boxes(
                            selection_frame, min_area_px=args.auto_min_area,
                            max_boxes=max(1, args.max_boxes),
                        )
                        if not detected_boxes:
                            cv2.putText(
                                frame,
                                "Searching for rectangular boxes...",
                                (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                                (0, 255, 255), 2, cv2.LINE_AA,
                            )
                        else:
                            if (
                                support_plane is None
                                or frame_index % max(1, args.support_plane_refresh) == 0
                            ):
                                plane_points = support_plane_points(xyz_map, detected_boxes)
                                fitted_plane = fit_support_plane(
                                    plane_points, threshold_m=args.support_plane_threshold
                                )
                                if fitted_plane is not None:
                                    support_plane = fitted_plane
                                    metric_scale = (
                                        args.camera_height_m / support_plane.camera_height_m
                                        if args.camera_height_m else 1.0
                                    )

                            used_track_ids: set[int] = set()
                            observations: list[BoxObservation] = []
                            for bbox in detected_boxes:
                                track_id, next_track_id = assign_track(
                                    bbox, tracks, frame_index, next_track_id, used_track_ids
                                )
                                used_track_ids.add(track_id)
                                median_depth_m, valid_depth_fraction = median_depth_in_bbox(
                                    depth_map, bbox, args.min_depth, args.max_depth
                                )
                                if valid_depth_fraction < args.min_valid_depth:
                                    median_depth_m = math.nan
                                face_width_m, face_height_m = visible_face_dimensions(
                                    bbox, median_depth_m, fx_px, fy_px
                                )
                                points = points_for_bbox(
                                    xyz_map, bbox, median_depth_m, args.roi_depth_band
                                )
                                estimate = robust_oriented_cuboid(
                                    points,
                                    reliable_ratio=args.reliable_ratio,
                                    minimum_third_dimension_m=args.min_third_dimension,
                                )
                                length_m = max(face_width_m, face_height_m) * metric_scale
                                breadth_m = min(face_width_m, face_height_m) * metric_scale
                                depth_m = (
                                    estimate.depth_m * metric_scale
                                    if estimate.third_dimension_reliable else math.nan
                                )
                                bottom_height_m, top_height_m = heights_above_support(
                                    points, support_plane, metric_scale
                                )
                                track = tracks[track_id]
                                if math.isfinite(length_m) and math.isfinite(breadth_m):
                                    track.dimensions.append((length_m, breadth_m, depth_m))
                                if math.isfinite(bottom_height_m) and math.isfinite(top_height_m):
                                    track.heights.append((bottom_height_m, top_height_m))
                                if math.isfinite(median_depth_m):
                                    track.depths.append(median_depth_m * metric_scale)

                                if track.dimensions:
                                    values = np.asarray(track.dimensions)
                                    length_m = float(np.nanmedian(values[:, 0]))
                                    breadth_m = float(np.nanmedian(values[:, 1]))
                                    depth_m = (
                                        float(np.nanmedian(values[:, 2]))
                                        if np.isfinite(values[:, 2]).any() else math.nan
                                    )
                                if track.heights:
                                    height_values = np.asarray(track.heights)
                                    bottom_height_m = float(np.nanmedian(height_values[:, 0]))
                                    top_height_m = float(np.nanmedian(height_values[:, 1]))
                                if track.depths:
                                    median_depth_m = float(np.nanmedian(track.depths))

                                observations.append(
                                    BoxObservation(
                                        track_id, bbox, length_m, breadth_m, depth_m,
                                        median_depth_m, valid_depth_fraction,
                                        bottom_height_m, top_height_m, estimate,
                                    )
                                )

                            supports = infer_stack_supports(
                                observations,
                                maximum_height_gap_m=args.stack_height_gap,
                                maximum_depth_difference_m=args.stack_depth_gap,
                            )
                            stack_ids = assign_stack_ids(observations, supports)
                            for observation in observations:
                                reliable = (
                                    observation.estimate.third_dimension_reliable
                                    and math.isfinite(observation.depth_m)
                                )
                                supported_by = supports.get(observation.track_id)
                                relationship = (
                                    f"ON Box {supported_by} | Stack {stack_ids[observation.track_id]}"
                                    if supported_by is not None
                                    else f"Base | Stack {stack_ids[observation.track_id]}"
                                )
                                lines = [f"Box {observation.track_id} | {relationship}"]
                                if math.isfinite(observation.length_m) and math.isfinite(observation.breadth_m):
                                    if reliable:
                                        lines.append(
                                            f"LxBxD {observation.length_m * 100:.1f} x "
                                            f"{observation.breadth_m * 100:.1f} x "
                                            f"{observation.depth_m * 100:.1f} cm"
                                        )
                                    else:
                                        lines.append(
                                            f"LxB {observation.length_m * 100:.1f} x "
                                            f"{observation.breadth_m * 100:.1f} cm"
                                        )
                                if math.isfinite(observation.bottom_height_m):
                                    lines.append(
                                        f"height {observation.bottom_height_m * 100:.1f}-"
                                        f"{observation.top_height_m * 100:.1f} cm"
                                    )
                                else:
                                    lines.append("support plane unavailable")
                                draw_label(frame, observation.bbox, lines, reliable)
                                x1, y1, x2, y2 = observation.bbox
                                table.writerow(
                                    {
                                        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                                        "frame": frame_index,
                                        "elapsed_s": time.monotonic() - start_time,
                                        "box_id": observation.track_id,
                                        "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                                        "length_cm": observation.length_m * 100,
                                        "breadth_cm": observation.breadth_m * 100,
                                        "depth_cm": observation.depth_m * 100 if reliable else math.nan,
                                        "third_dimension_reliable": reliable,
                                        "geometry_ratio": observation.estimate.geometry_ratio,
                                        "plane_inlier_fraction": observation.estimate.plane_inlier_fraction,
                                        "point_count": observation.estimate.point_count,
                                        "foreground_area_px": (x2 - x1) * (y2 - y1),
                                        "median_depth_m": observation.median_depth_m,
                                        "valid_depth_fraction": observation.valid_depth_fraction,
                                        "measurement_mode": "auto_multi_stack",
                                        "bottom_height_cm": observation.bottom_height_m * 100,
                                        "top_height_cm": observation.top_height_m * 100,
                                        "support_box_id": supported_by if supported_by is not None else "",
                                        "stack_id": stack_ids[observation.track_id],
                                        "support_plane_camera_height_m": (
                                            support_plane.camera_height_m if support_plane else math.nan
                                        ),
                                        "metric_scale": metric_scale,
                                    }
                                )

                            if support_plane is not None:
                                cv2.putText(
                                    frame,
                                    f"support plane: camera {support_plane.camera_height_m:.3f} m | "
                                    f"inliers {support_plane.inlier_ratio * 100:.0f}% | scale {metric_scale:.3f}",
                                    (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.58,
                                    (255, 255, 255), 2, cv2.LINE_AA,
                                )
                    elif args.mode == "background" and background_depth is None and capture_remaining == 0:
                        cv2.putText(
                            frame,
                            "REMOVE ALL BOXES, then press B to capture the empty scene",
                            (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (0, 255, 255), 2, cv2.LINE_AA,
                        )
                    elif args.mode == "background" and background_depth is not None and capture_remaining == 0:
                        if background_depth.shape != depth_map.shape:
                            raise SystemExit(
                                f"Background shape {background_depth.shape} does not match camera {depth_map.shape}; recapture it."
                            )
                        mask = foreground_mask(
                            background_depth, depth_map, args.foreground_threshold,
                            args.min_depth, args.max_depth,
                        )
                        for bbox, component, area in component_boxes(mask, args.min_area):
                            track_id, next_track_id = assign_track(
                                bbox, tracks, frame_index, next_track_id
                            )
                            points = points_for_component(xyz_map, component)
                            estimate = robust_oriented_cuboid(
                                points,
                                reliable_ratio=args.reliable_ratio,
                                minimum_third_dimension_m=args.min_third_dimension,
                            )
                            track = tracks[track_id]
                            if math.isfinite(estimate.length_m) and math.isfinite(estimate.breadth_m):
                                track.dimensions.append(
                                    (estimate.length_m, estimate.breadth_m, estimate.depth_m)
                                )
                            if track.dimensions:
                                values = np.asarray(track.dimensions)
                                length_m = float(np.nanmedian(values[:, 0]))
                                breadth_m = float(np.nanmedian(values[:, 1]))
                                depth_values = values[:, 2]
                                depth_m = (
                                    float(np.nanmedian(depth_values))
                                    if np.isfinite(depth_values).any() else math.nan
                                )
                            else:
                                length_m, breadth_m, depth_m = (
                                    estimate.length_m, estimate.breadth_m, estimate.depth_m
                                )
                            reliable = estimate.third_dimension_reliable and math.isfinite(depth_m)
                            if reliable:
                                lines = [
                                    f"Box {track_id}",
                                    f"L x B x D: {length_m * 100:.1f} x {breadth_m * 100:.1f} x {depth_m * 100:.1f} cm",
                                ]
                            elif math.isfinite(length_m) and math.isfinite(breadth_m):
                                lines = [
                                    f"Box {track_id}",
                                    f"Visible face: {length_m * 100:.1f} x {breadth_m * 100:.1f} cm | D not observable",
                                ]
                            else:
                                lines = [f"Box {track_id}", "Not enough valid 3D points"]
                            draw_label(frame, bbox, lines, reliable)
                            x1, y1, x2, y2 = bbox
                            table.writerow(
                                {
                                    "timestamp_utc": datetime.now(timezone.utc).isoformat(),
                                    "frame": frame_index,
                                    "elapsed_s": time.monotonic() - start_time,
                                    "box_id": track_id,
                                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                                    "length_cm": length_m * 100 if math.isfinite(length_m) else math.nan,
                                    "breadth_cm": breadth_m * 100 if math.isfinite(breadth_m) else math.nan,
                                    "depth_cm": depth_m * 100 if reliable else math.nan,
                                    "third_dimension_reliable": reliable,
                                    "geometry_ratio": estimate.geometry_ratio,
                                    "plane_inlier_fraction": estimate.plane_inlier_fraction,
                                    "point_count": estimate.point_count,
                                    "foreground_area_px": area,
                                    "median_depth_m": math.nan,
                                    "valid_depth_fraction": math.nan,
                                    "measurement_mode": "background_components",
                                }
                            )

                    help_text = (
                        "Automatic rectangle + ZED depth | Q: quit"
                        if args.mode == "auto"
                        else "Background boxes | B: recapture empty scene | Q: quit"
                    )
                    cv2.putText(
                        frame,
                        help_text,
                        (18, frame.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.60,
                        (255, 255, 255), 2, cv2.LINE_AA,
                    )
                    if args.record:
                        if writer is None:
                            writer = cv2.VideoWriter(
                                str(args.record), cv2.VideoWriter_fourcc(*"mp4v"),
                                float(args.fps), (frame.shape[1], frame.shape[0]),
                            )
                            if not writer.isOpened():
                                raise SystemExit(f"Cannot open video writer: {args.record}")
                        writer.write(frame)
                    if not args.headless:
                        cv2.imshow("ZED2i Geometry Box Measurement", frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key in (ord("q"), 27):
                            break
                        if args.mode == "background" and key == ord("b"):
                            background_samples.clear()
                            background_depth = None
                            capture_remaining = args.background_frames
                            print("Capturing empty-scene background; keep all boxes out of view.")
                    frame_index += 1
                    if frame_index % 30 == 0:
                        csv_stream.flush()
                    for old_id in [key for key, track in tracks.items() if frame_index - track.last_frame > 60]:
                        del tracks[old_id]
            except KeyboardInterrupt:
                pass
            finally:
                if writer is not None:
                    writer.release()
                cv2.destroyAllWindows()
        print(f"frames={frame_index} csv={csv_path}")
        if args.record:
            print(f"video={args.record}")
    finally:
        camera.close()


if __name__ == "__main__":
    main()
