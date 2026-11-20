#!/usr/bin/env python3
"""Reusable hospital scene-context geometry for fall detection."""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np


VALID_REGION_TYPES = {"bed", "floor", "couch", "chair", "stretcher", "ignore"}
REGION_COLORS = {
    "bed": (255, 80, 80),
    "floor": (80, 180, 80),
    "couch": (255, 120, 220),
    "chair": (0, 180, 255),
    "stretcher": (220, 180, 60),
    "ignore": (140, 140, 140),
}


@dataclass(frozen=True)
class PolygonRegion:
    name: str
    region_type: str
    points: list[list[float]]

    def validate(self) -> None:
        if self.region_type not in VALID_REGION_TYPES:
            raise ValueError(f"Unsupported region type {self.region_type!r}")
        if len(self.points) < 3:
            raise ValueError(f"Region {self.name!r} needs at least three points")
        for point in self.points:
            if len(point) != 2 or not all(0.0 <= float(value) <= 1.0 for value in point):
                raise ValueError(f"Region {self.name!r} points must be normalized [x, y] pairs")


@dataclass(frozen=True)
class ContextConfig:
    schema_version: int
    camera_id: str
    reference_width: int
    reference_height: int
    regions: list[PolygonRegion]

    def validate(self) -> None:
        if self.schema_version != 1:
            raise ValueError(f"Unsupported context schema version: {self.schema_version}")
        if self.reference_width <= 0 or self.reference_height <= 0:
            raise ValueError("Reference width and height must be positive")
        names = set()
        for region in self.regions:
            region.validate()
            if region.name in names:
                raise ValueError(f"Duplicate region name: {region.name}")
            names.add(region.name)


def save_context_config(config: ContextConfig, path: Path) -> None:
    config.validate()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(asdict(config), indent=2) + "\n", encoding="utf-8")


def load_context_config(path: Path) -> ContextConfig:
    raw = json.loads(path.read_text(encoding="utf-8"))
    regions = [PolygonRegion(**region) for region in raw.get("regions", [])]
    config = ContextConfig(
        schema_version=int(raw.get("schema_version", 1)),
        camera_id=str(raw["camera_id"]),
        reference_width=int(raw["reference_width"]),
        reference_height=int(raw["reference_height"]),
        regions=regions,
    )
    config.validate()
    return config


def normalized_points(points_px: list[tuple[int, int]], width: int, height: int) -> list[list[float]]:
    return [
        [float(np.clip(x / max(width, 1), 0.0, 1.0)), float(np.clip(y / max(height, 1), 0.0, 1.0))]
        for x, y in points_px
    ]


def scaled_polygon(region: PolygonRegion, width: int, height: int) -> np.ndarray:
    return np.asarray(
        [[round(x * width), round(y * height)] for x, y in region.points], dtype=np.int32
    )


class ContextGeometry:
    """Cached polygon masks and per-person overlap calculations."""

    def __init__(self, config: ContextConfig):
        config.validate()
        self.config = config
        self._cache: dict[tuple[int, int], dict[str, np.ndarray]] = {}

    def masks(self, width: int, height: int) -> dict[str, np.ndarray]:
        cache_key = (width, height)
        if cache_key in self._cache:
            return self._cache[cache_key]
        masks: dict[str, np.ndarray] = {
            region_type: np.zeros((height, width), dtype=np.uint8)
            for region_type in VALID_REGION_TYPES
        }
        for region in self.config.regions:
            cv2.fillPoly(masks[region.region_type], [scaled_polygon(region, width, height)], 1)
        self._cache[cache_key] = masks
        return masks

    def bbox_overlaps(
        self, bbox_xyxy: tuple[float, float, float, float] | np.ndarray, width: int, height: int
    ) -> dict[str, float]:
        x1, y1, x2, y2 = (int(round(float(value))) for value in bbox_xyxy)
        x1, x2 = max(0, min(x1, width)), max(0, min(x2, width))
        y1, y2 = max(0, min(y1, height)), max(0, min(y2, height))
        area = max(0, x2 - x1) * max(0, y2 - y1)
        if area == 0:
            return {region_type: 0.0 for region_type in VALID_REGION_TYPES}
        return {
            region_type: float(mask[y1:y2, x1:x2].sum()) / area
            for region_type, mask in self.masks(width, height).items()
        }

    def point_regions(self, point_xy: tuple[float, float], width: int, height: int) -> set[str]:
        x = int(np.clip(round(point_xy[0]), 0, width - 1))
        y = int(np.clip(round(point_xy[1]), 0, height - 1))
        return {
            region_type
            for region_type, mask in self.masks(width, height).items()
            if bool(mask[y, x])
        }

    def keypoint_region_fractions(
        self,
        points_xy: np.ndarray,
        confidence: np.ndarray | None,
        width: int,
        height: int,
        minimum_confidence: float = 0.25,
        indices: tuple[int, ...] = (5, 6, 11, 12, 13, 14, 15, 16),
    ) -> dict[str, float]:
        """Return confidence-weighted body-keypoint occupancy for each region.

        Shoulders, hips, knees and ankles are used by default.  Face and wrist
        keypoints are deliberately excluded because they frequently extend off
        a bed while the patient's body is still supported by it.
        """
        points = np.asarray(points_xy, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ValueError("points_xy must have shape [N, 2]")
        if confidence is None:
            scores = np.ones(len(points), dtype=np.float64)
        else:
            scores = np.asarray(confidence, dtype=np.float64).reshape(-1)
            if len(scores) != len(points):
                raise ValueError("confidence length must match points_xy")
        selected = [
            index for index in indices
            if index < len(points)
            and np.isfinite(points[index]).all()
            and np.isfinite(scores[index])
            and scores[index] >= minimum_confidence
        ]
        total = float(sum(scores[index] for index in selected))
        fractions = {region_type: 0.0 for region_type in VALID_REGION_TYPES}
        if total <= 0.0:
            return fractions
        masks = self.masks(width, height)
        for index in selected:
            x = int(np.clip(round(points[index, 0]), 0, width - 1))
            y = int(np.clip(round(points[index, 1]), 0, height - 1))
            for region_type, mask in masks.items():
                if bool(mask[y, x]):
                    fractions[region_type] += float(scores[index]) / total
        return fractions

    def draw(self, frame: np.ndarray, alpha: float = 0.25) -> np.ndarray:
        output = frame.copy()
        overlay = frame.copy()
        height, width = frame.shape[:2]
        for region in self.config.regions:
            polygon = scaled_polygon(region, width, height)
            color = REGION_COLORS[region.region_type]
            cv2.fillPoly(overlay, [polygon], color)
            cv2.polylines(output, [polygon], True, color, 2, cv2.LINE_AA)
            anchor = tuple(polygon[0])
            cv2.putText(
                output, f"{region.name} ({region.region_type})", anchor,
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 4, cv2.LINE_AA,
            )
            cv2.putText(
                output, f"{region.name} ({region.region_type})", anchor,
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA,
            )
        cv2.addWeighted(overlay, alpha, output, 1.0 - alpha, 0.0, output)
        return output


def rectangle_overlap_fraction(
    first_xyxy: tuple[float, float, float, float] | np.ndarray,
    second_xyxy: tuple[float, float, float, float] | np.ndarray,
) -> float:
    ax1, ay1, ax2, ay2 = map(float, first_xyxy)
    bx1, by1, bx2, by2 = map(float, second_xyxy)
    intersection = max(0.0, min(ax2, bx2) - max(ax1, bx1)) * max(
        0.0, min(ay2, by2) - max(ay1, by1)
    )
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    return intersection / first_area if first_area > 0.0 else 0.0


class FurnitureDetector:
    """Optional COCO furniture fallback; loaded only when explicitly enabled."""

    TARGET_CLASSES = {"bed", "couch", "chair"}

    def __init__(self, model_path: str, device: str = "0", confidence: float = 0.25):
        from ultralytics import YOLO

        self.model = YOLO(model_path)
        self.device = device
        self.confidence = confidence
        self.class_ids = [
            int(class_id)
            for class_id, name in self.model.names.items()
            if str(name).lower() in self.TARGET_CLASSES
        ]
        if not self.class_ids:
            raise ValueError("The supplied model has no bed, couch, or chair classes")

    def detect(self, frame: np.ndarray, imgsz: int = 640) -> list[dict]:
        result = self.model.predict(
            frame, imgsz=imgsz, conf=self.confidence, classes=self.class_ids,
            device=self.device, verbose=False,
        )[0]
        if result.boxes is None:
            return []
        detections = []
        for box, class_id, confidence in zip(
            result.boxes.xyxy.cpu().numpy(),
            result.boxes.cls.cpu().numpy().astype(int),
            result.boxes.conf.cpu().numpy(),
        ):
            detections.append(
                {
                    "bbox": box.astype(np.float32),
                    "region_type": str(self.model.names[class_id]).lower(),
                    "confidence": float(confidence),
                }
            )
        return detections
