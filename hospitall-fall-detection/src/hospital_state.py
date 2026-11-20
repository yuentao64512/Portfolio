#!/usr/bin/env python3
"""Context-aware patient location and fall state for one fixed CCTV view."""

from __future__ import annotations

from dataclasses import dataclass


BED_REGION_TYPES = ("bed", "stretcher", "couch")


@dataclass(frozen=True)
class LocationDecision:
    location: str
    bed_score: float
    floor_score: float


@dataclass(frozen=True)
class HospitalDecision:
    state: str
    alert: bool
    floor_down_seconds: float


def classify_patient_location(
    bbox_overlaps: dict[str, float],
    keypoint_fractions: dict[str, float],
    minimum_score: float = 0.20,
    decision_margin: float = 0.08,
) -> LocationDecision:
    """Fuse coarse bbox overlap with body-keypoint occupancy."""
    bbox_bed = max((bbox_overlaps.get(name, 0.0) for name in BED_REGION_TYPES), default=0.0)
    keypoint_bed = max((keypoint_fractions.get(name, 0.0) for name in BED_REGION_TYPES), default=0.0)
    bed_score = 0.35 * bbox_bed + 0.65 * keypoint_bed
    floor_score = 0.35 * bbox_overlaps.get("floor", 0.0) + 0.65 * keypoint_fractions.get("floor", 0.0)
    if bed_score >= minimum_score and bed_score >= floor_score + decision_margin:
        location = "ON_BED"
    elif floor_score >= minimum_score and floor_score >= bed_score + decision_margin:
        location = "ON_FLOOR"
    elif max(bed_score, floor_score) >= minimum_score:
        location = "AMBIGUOUS"
    else:
        location = "UNKNOWN"
    return LocationDecision(location, float(bed_score), float(floor_score))


def horizontal_pose(bbox_xyxy, minimum_aspect_ratio: float = 1.12) -> bool:
    x1, y1, x2, y2 = map(float, bbox_xyxy)
    width = max(0.0, x2 - x1)
    height = max(1.0, y2 - y1)
    return width / height >= minimum_aspect_ratio


class HospitalStateMachine:
    """Distinguish normal bed occupancy, floor occupancy and a fall alert."""

    def __init__(self, lying_alert_seconds: float = 2.0, transition_window_seconds: float = 10.0):
        self.lying_alert_seconds = float(lying_alert_seconds)
        self.transition_window_seconds = float(transition_window_seconds)
        self.last_bed_time: float | None = None
        self.last_fall_time: float | None = None
        self.floor_down_since: float | None = None

    def update(
        self,
        timestamp: float,
        location: str,
        action: str,
        down_pose: bool,
    ) -> HospitalDecision:
        timestamp = float(timestamp)
        if action == "falling_down":
            self.last_fall_time = timestamp

        if location == "ON_BED":
            self.last_bed_time = timestamp
            self.floor_down_since = None
            return HospitalDecision("PATIENT_ON_BED", False, 0.0)

        if location == "ON_FLOOR":
            down_evidence = down_pose or action in {"lying_down", "falling_down"}
            if down_evidence:
                if self.floor_down_since is None:
                    self.floor_down_since = timestamp
            else:
                self.floor_down_since = None
            down_seconds = (
                max(0.0, timestamp - self.floor_down_since)
                if self.floor_down_since is not None else 0.0
            )
            recent_bed_exit = (
                self.last_bed_time is not None
                and timestamp - self.last_bed_time <= self.transition_window_seconds
            )
            recent_fall = (
                self.last_fall_time is not None
                and timestamp - self.last_fall_time <= self.transition_window_seconds
            )
            alert = (
                self.floor_down_since is not None
                and down_seconds >= self.lying_alert_seconds
                and (recent_bed_exit or recent_fall)
            )
            if alert:
                return HospitalDecision("FALL_ALERT_ON_FLOOR", True, down_seconds)
            if self.floor_down_since is not None:
                return HospitalDecision("PATIENT_DOWN_ON_FLOOR", False, down_seconds)
            return HospitalDecision("PATIENT_ON_FLOOR", False, 0.0)

        self.floor_down_since = None
        return HospitalDecision(location, False, 0.0)
