"""Shared normal-camera action features and the continuous-lying alert timer."""
from dataclasses import dataclass

import numpy as np

FPS = 8.0
WINDOW = 12
FEATURE_VERSION = "body12_centered_timed_v1"
BODY_EDGES = [
    (5, 6), (5, 7), (7, 9), (6, 8), (8, 10), (5, 11),
    (6, 12), (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
]
FULL_EDGES = BODY_EDGES + [(0, 1), (0, 2), (1, 3), (2, 4), (0, 5), (0, 6)]


def featurize(sequence):
    values = np.asarray(sequence, dtype=np.float32)
    if values.ndim == 3:
        values = values[None]
    if values.ndim != 4 or values.shape[1:] != (WINDOW, 17, 3):
        raise ValueError(f"Expected (*, {WINDOW}, 17, 3), got {values.shape}")

    xy = values[:, :, 5:, :2]
    confidence = values[:, :, 5:, 2]
    visible = confidence > 0.25
    hips = (xy[:, :, 6] + xy[:, :, 7]) / 2
    shoulders = (xy[:, :, 0] + xy[:, :, 1]) / 2
    torso = np.linalg.norm(hips - shoulders, axis=-1)
    torso_ok = visible[:, :, [0, 1, 6, 7]].all(-1)
    scales = []
    for index in range(len(values)):
        valid = torso[index][torso_ok[index]]
        scales.append(max(float(np.median(valid)) if len(valid) else 0.15, 0.03))
    scales = np.asarray(scales, dtype=np.float32)
    scale = scales[:, None, None, None]
    relative = np.clip((xy - hips[:, :, None, :]) / scale, -5, 5)
    relative *= visible[:, :, :, None]
    velocity = np.diff(relative, axis=1) * FPS
    velocity *= (visible[:, 1:] & visible[:, :-1])[:, :, :, None]
    parts = [
        relative.mean(1), relative.std(1), relative[:, -1],
        np.abs(velocity).mean(1), np.abs(velocity).max(1),
        confidence.mean(1), visible.mean(1),
    ]
    motion = np.diff(hips, axis=1) * FPS / scales[:, None, None]
    motion *= (torso_ok[:, 1:] & torso_ok[:, :-1])[:, :, None]
    parts += [motion.mean(1), np.abs(motion).mean(1), motion.std(1)]
    body = hips - shoulders
    horizontal = np.abs(body[:, :, 0]) / (np.linalg.norm(body, axis=-1) + 1e-6)
    parts += [np.stack([horizontal.mean(1), horizontal[:, -1], torso_ok.mean(1)], 1)]
    features = np.concatenate([part.reshape(len(values), -1) for part in parts], 1)
    return np.nan_to_num(features.astype(np.float32))


def detections_from_result(result, width, height):
    if result.keypoints is None or result.keypoints.conf is None:
        return []
    boxes = result.boxes.xyxy.cpu().numpy()
    points = result.keypoints.xy.cpu().numpy()
    confidence = result.keypoints.conf.cpu().numpy()
    detections = []
    for box, keypoints, scores in zip(boxes, points, confidence):
        if (scores[5:] > 0.25).sum() < 5:
            continue
        pose = np.zeros((17, 3), np.float32)
        pose[:, :2] = keypoints / [width, height]
        pose[:, 2] = scores
        area = float(max(0, box[2] - box[0]) * max(0, box[3] - box[1]))
        detections.append({"box": box, "pose": pose, "area": area})
    return sorted(detections, key=lambda item: item["area"], reverse=True)


@dataclass
class LyingTimer:
    threshold: float = 10.0
    grace: float = 0.5
    since: float | None = None
    last: float | None = None
    elapsed: float = 0.0
    alert: bool = False
    gap_since: float | None = None
    previous_lying: bool = False

    def update(self, label, now, observed=True):
        delta = 0.0 if self.last is None else max(0.0, now - self.last)
        if delta > 0.6:
            self.since = None
            self.elapsed = 0.0
            self.alert = False
            self.previous_lying = False
            self.gap_since = None
        if not observed or label == "uncertain":
            if self.gap_since is None:
                self.gap_since = now
            if now - self.gap_since > self.grace:
                self.since = None
                self.elapsed = 0.0
                self.alert = False
            self.previous_lying = False
            self.last = now
            return label
        if label != "lying_down":
            self.since = None
            self.elapsed = 0.0
            self.alert = False
            self.previous_lying = False
            self.gap_since = None
        else:
            if self.gap_since is not None and now - self.gap_since > self.grace:
                self.since = None
                self.elapsed = 0.0
                self.alert = False
            if self.since is None:
                self.since = now
            elif self.previous_lying:
                self.elapsed += delta
            self.gap_since = None
            self.previous_lying = True
            self.alert = self.elapsed >= self.threshold
        self.last = now
        return "falling_down" if self.alert else label

