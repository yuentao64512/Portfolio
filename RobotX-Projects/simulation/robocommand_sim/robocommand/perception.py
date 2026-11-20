"""OpenCV beacon detector, world projection, and multi-frame track merger."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Iterable

import cv2
import numpy as np


def gazebo_image_to_bgr(msg: object) -> np.ndarray:
    """Decode a Gazebo RGB image. Reject unexpected formats or buffer sizes."""
    width, height = int(msg.width), int(msg.height)
    if width <= 0 or height <= 0:
        raise ValueError("invalid Gazebo image size")
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    fmt = int(msg.pixel_format_type)
    channels = {3: 3, 8: 3, 4: 4, 5: 4, 1: 1}.get(fmt)
    if channels is None or raw.size != width * height * channels:
        raise ValueError(f"unsupported Gazebo image format/size {fmt}")
    if fmt == 3:
        return cv2.cvtColor(raw.reshape(height, width, 3), cv2.COLOR_RGB2BGR)
    if fmt == 8:
        return raw.reshape(height, width, 3).copy()
    if fmt == 4:
        return cv2.cvtColor(raw.reshape(height, width, 4), cv2.COLOR_RGBA2BGR)
    if fmt == 5:
        return cv2.cvtColor(raw.reshape(height, width, 4), cv2.COLOR_BGRA2BGR)
    return cv2.cvtColor(raw.reshape(height, width), cv2.COLOR_GRAY2BGR)


def detect_usv_markers(bgr: np.ndarray) -> list["Detection"]:
    """Find visible coloured marker blobs in a simulated forward USV image."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    masks = {
        "red": cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 145, 55]), np.array([11, 255, 255])),
            cv2.inRange(hsv, np.array([171, 145, 55]), np.array([179, 255, 255])),
        ),
        "green": cv2.inRange(hsv, np.array([42, 130, 55]), np.array([88, 255, 255])),
        "blue": cv2.inRange(hsv, np.array([94, 150, 70]), np.array([110, 255, 255])),
    }
    result: list[Detection] = []
    height, width = bgr.shape[:2]
    image_area = height * width
    kernel = np.ones((3, 3), np.uint8)
    # In this waterfront world the marker bodies sit just around the horizon.
    # Reflections extend into the lower half and otherwise double the count.
    roi_top, roi_bottom = int(height * 0.32), int(height * 0.58)
    for colour, mask in masks.items():
        mask[:roi_top] = 0
        mask[roi_bottom:] = 0
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < 8.0 or area > image_area * 0.08:
                continue
            x, y, box_w, box_h = cv2.boundingRect(contour)
            if box_w < 3 or box_h < 3:
                continue
            moment = cv2.moments(contour)
            if moment["m00"] == 0:
                continue
            u, v = moment["m10"] / moment["m00"], moment["m01"] / moment["m00"]
            intensity = float(cv2.mean(hsv[:, :, 2], mask=cv2.drawContours(np.zeros(mask.shape, np.uint8), [contour], -1, 255, -1))[0])
            result.append(Detection(colour, u, v, area, min(0.95, 0.5 + area / 300.0), intensity))
    return sorted(result, key=lambda item: item.area, reverse=True)


@dataclass
class Detection:
    colour: str
    u: float
    v: float
    area: float
    confidence: float
    intensity: float


@dataclass
class Track:
    track_id: int
    colour_votes: list[str] = field(default_factory=list)
    points: list[tuple[float, float]] = field(default_factory=list)
    times: list[float] = field(default_factory=list)
    areas: list[float] = field(default_factory=list)
    intensities: list[float] = field(default_factory=list)

    @property
    def colour(self) -> str:
        return max(set(self.colour_votes), key=self.colour_votes.count)

    @property
    def x(self) -> float:
        return float(np.mean([p[0] for p in self.points]))

    @property
    def y(self) -> float:
        return float(np.mean([p[1] for p in self.points]))

    @property
    def sigma_m(self) -> float:
        if len(self.points) < 2:
            return 1.5
        values = [math.hypot(x - self.x, y - self.y) for x, y in self.points]
        return max(0.05, float(np.std(values)))

    @property
    def blink_score(self) -> float:
        if self.colour != "blue" or len(self.times) < 4:
            return 0.0
        gaps = np.diff(sorted(self.times))
        typical = float(np.median(gaps)) if len(gaps) else 0.1
        blink_gaps = gaps[(gaps > max(0.30, typical * 2.5)) & (gaps < 2.5)]
        brightness_range = max(self.intensities, default=0.0) - min(self.intensities, default=0.0)
        return float(len(blink_gaps)) + min(1.0, brightness_range / 35.0)

    @property
    def blinking(self) -> bool:
        return self.blink_score >= 1.0


def detect_beacons(bgr: np.ndarray) -> list[Detection]:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    masks = {
        "red": cv2.bitwise_or(
            cv2.inRange(hsv, np.array([0, 180, 25]), np.array([10, 255, 255])),
            cv2.inRange(hsv, np.array([172, 180, 25]), np.array([179, 255, 255])),
        ),
        "green": cv2.inRange(hsv, np.array([48, 180, 25]), np.array([85, 255, 255])),
        "blue": cv2.inRange(hsv, np.array([94, 180, 80]), np.array([108, 255, 255])),
        "black": cv2.inRange(hsv, np.array([0, 0, 0]), np.array([179, 80, 45])),
    }
    detections: list[Detection] = []
    kernel = np.ones((3, 3), np.uint8)
    image_area = bgr.shape[0] * bgr.shape[1]
    for colour, mask in masks.items():
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = float(cv2.contourArea(contour))
            min_area = 10.0 if colour != "black" else 20.0
            if area < min_area or area > image_area * 0.025:
                continue
            perimeter = cv2.arcLength(contour, True)
            circularity = 4.0 * math.pi * area / (perimeter * perimeter) if perimeter else 0.0
            if circularity < (0.28 if colour != "black" else 0.45):
                continue
            moment = cv2.moments(contour)
            if not moment["m00"]:
                continue
            u = moment["m10"] / moment["m00"]
            v = moment["m01"] / moment["m00"]
            confidence = min(0.99, 0.55 + circularity * 0.3 + min(area / 200.0, 0.14))
            filled = np.zeros(mask.shape, np.uint8)
            cv2.drawContours(filled, [contour], -1, 255, -1)
            intensity = float(cv2.mean(hsv[:, :, 2], mask=filled)[0])
            detections.append(Detection(colour, u, v, area, confidence, intensity))
    return detections


def pixel_to_world(
    u: float,
    v: float,
    width: int,
    height: int,
    altitude: float,
    yaw: float,
    hfov: float,
    yaw_offset: float = 0.0,
) -> tuple[float, float]:
    """Project a downward-looking pixel to a horizontal offset in world x/y."""
    fx = width / (2.0 * math.tan(hfov / 2.0))
    vfov = 2.0 * math.atan((height / width) * math.tan(hfov / 2.0))
    fy = height / (2.0 * math.tan(vfov / 2.0))
    # The Iris gimbal's +90 deg downward pose mirrors image-right relative to
    # the body/world convention: +u maps to body-left.
    right = -(u - width / 2.0) * altitude / fx
    forward = -(v - height / 2.0) * altitude / fy
    heading = yaw + yaw_offset
    dx = math.cos(heading) * forward - math.sin(heading) * right
    dy = math.sin(heading) * forward + math.cos(heading) * right
    return dx, dy


class TrackStore:
    def __init__(self, merge_radius_m: float = 1.5) -> None:
        self.merge_radius = merge_radius_m
        self.tracks: list[Track] = []
        self._next_id = 1

    def add(self, colour: str, x: float, y: float, area: float, timestamp: float | None = None, intensity: float = 0.0) -> Track:
        timestamp = time.time() if timestamp is None else timestamp
        candidates = [
            track
            for track in self.tracks
            if track.colour == colour and math.hypot(track.x - x, track.y - y) <= self.merge_radius
        ]
        if candidates:
            track = min(candidates, key=lambda t: math.hypot(t.x - x, t.y - y))
        else:
            track = Track(self._next_id)
            self._next_id += 1
            self.tracks.append(track)
        track.colour_votes.append(colour)
        track.points.append((x, y))
        track.times.append(timestamp)
        track.areas.append(area)
        track.intensities.append(intensity)
        if len(track.points) > 200:
            track.colour_votes = track.colour_votes[-200:]
            track.points = track.points[-200:]
            track.times = track.times[-200:]
            track.areas = track.areas[-200:]
            track.intensities = track.intensities[-200:]
        return track

    def confirmed(self, minimum_sightings: int = 3) -> Iterable[Track]:
        return (track for track in self.tracks if len(track.points) >= minimum_sightings)

    def consolidated(self, minimum_sightings: int = 5, radius_m: float = 3.5) -> list[Track]:
        """Merge only confirmed same-colour output tracks; raw association stays at 1.5 m."""
        groups: list[Track] = []
        for track in sorted(self.confirmed(minimum_sightings), key=lambda t: len(t.points), reverse=True):
            matches = [g for g in groups if g.colour == track.colour and math.hypot(g.x - track.x, g.y - track.y) <= radius_m]
            if matches:
                group = min(matches, key=lambda g: math.hypot(g.x - track.x, g.y - track.y))
                group.colour_votes.extend(track.colour_votes)
                group.points.extend(track.points)
                group.times.extend(track.times)
                group.areas.extend(track.areas)
                group.intensities.extend(track.intensities)
            else:
                groups.append(Track(
                    track.track_id,
                    list(track.colour_votes), list(track.points), list(track.times),
                    list(track.areas), list(track.intensities),
                ))
        changed = True
        while changed:
            changed = False
            for i in range(len(groups)):
                for j in range(i + 1, len(groups)):
                    if groups[i].colour != groups[j].colour:
                        continue
                    if math.hypot(groups[i].x - groups[j].x, groups[i].y - groups[j].y) > radius_m:
                        continue
                    groups[i].colour_votes.extend(groups[j].colour_votes)
                    groups[i].points.extend(groups[j].points)
                    groups[i].times.extend(groups[j].times)
                    groups[i].areas.extend(groups[j].areas)
                    groups[i].intensities.extend(groups[j].intensities)
                    del groups[j]
                    changed = True
                    break
                if changed:
                    break
        return groups

