#!/usr/bin/env python3
from pathlib import Path

import cv2
import numpy as np


source = Path("/home/robotx/box_size_training/outputs/current_zed_frame.png")
output = Path("/home/robotx/box_size_training/outputs/box_candidates_debug.png")
frame = cv2.imread(str(source))
if frame is None:
    raise SystemExit(f"Cannot read {source}")

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
lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=65, minLineLength=65, maxLineGap=24)
horizontal = []
vertical = []
if lines is not None:
    for raw in np.asarray(lines).reshape(-1, 4):
        x1, y1, x2, y2 = (int(value) for value in raw)
        dx, dy = x2 - x1, y2 - y1
        length = float(np.hypot(dx, dy))
        angle = abs(float(np.degrees(np.arctan2(dy, dx))))
        angle = min(angle, 180.0 - angle)
        if angle <= 8.0:
            horizontal.append((length, x1, y1, x2, y2))
        elif angle >= 82.0:
            vertical.append((length, x1, y1, x2, y2))
print("HORIZONTAL")
for item in sorted(horizontal, reverse=True)[:20]:
    print(item)
print("VERTICAL")
for item in sorted(vertical, reverse=True)[:20]:
    print(item)
candidates = []
height, width = gray.shape
for contour in contours:
    perimeter = cv2.arcLength(contour, True)
    if perimeter <= 0:
        continue
    approx = cv2.approxPolyDP(contour, 0.018 * perimeter, True)
    x, y, w, h = cv2.boundingRect(contour)
    bbox_area = w * h
    contour_area = abs(cv2.contourArea(contour))
    if bbox_area < 5000 or bbox_area > width * height * 0.65:
        continue
    if w < 60 or h < 45 or x <= 4 or y <= 4 or x + w >= width - 4 or y + h >= height - 4:
        continue
    rect = cv2.minAreaRect(contour)
    rw, rh = rect[1]
    rotated_area = rw * rh
    if rotated_area <= 0:
        continue
    rectangularity = contour_area / rotated_area
    aspect = max(rw, rh) / max(1.0, min(rw, rh))
    if rectangularity < 0.55 or aspect > 8.0 or len(approx) > 10:
        continue
    centre_bonus = 1.0 - min(1.0, abs((x + w / 2) - width / 2) / (width / 2)) * 0.15
    score = rotated_area * rectangularity * centre_bonus
    candidates.append((score, (x, y, x + w, y + h), rectangularity, len(approx), aspect, contour))

candidates.sort(reverse=True, key=lambda item: item[0])
debug = frame.copy()
for rank, (score, bbox, rectangularity, vertices, aspect, contour) in enumerate(candidates[:12], start=1):
    x1, y1, x2, y2 = bbox
    color = (0, 255, 0) if rank == 1 else (0, 165, 255)
    cv2.rectangle(debug, (x1, y1), (x2, y2), color, 2)
    cv2.putText(
        debug,
        f"{rank} r={rectangularity:.2f} v={vertices} a={aspect:.1f}",
        (x1, max(20, y1 - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        color,
        2,
        cv2.LINE_AA,
    )
    print(rank, bbox, "score", round(score), "rect", round(rectangularity, 3), "vertices", vertices, "aspect", round(aspect, 2))

cv2.imwrite(str(output), debug)
cv2.imwrite(str(output.with_name("box_edges_debug.png")), edges)

if candidates:
    _, coarse, *_ = candidates[0]
    x1, y1, x2, y2 = coarse
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB).astype(np.float32)
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    reference = np.median(lab[cy - 20:cy + 20, cx - 20:cx + 20].reshape(-1, 3), axis=0)
    distance = np.linalg.norm((lab - reference) * np.array([0.55, 1.0, 1.0], dtype=np.float32), axis=2)
    for threshold in (10.0, 15.0, 20.0, 25.0):
        colour_mask = ((distance <= threshold).astype(np.uint8) * 255)
        confined = np.zeros_like(colour_mask)
        confined[y1:y2, x1:x2] = colour_mask[y1:y2, x1:x2]
        confined = cv2.morphologyEx(
            confined, cv2.MORPH_CLOSE,
            cv2.getStructuringElement(cv2.MORPH_RECT, (13, 13)), iterations=2,
        )
        count, labels, stats, _ = cv2.connectedComponentsWithStats(confined, 8)
        label = int(labels[cy, cx])
        colour_debug = frame.copy()
        if label > 0:
            x, y, w, h, area = (int(value) for value in stats[label])
            cv2.rectangle(colour_debug, (x, y), (x + w, y + h), (255, 0, 255), 3)
            print("COLOUR", threshold, (x, y, x + w, y + h), "area", area, "reference", reference)
        cv2.imwrite(str(output.with_name(f"box_colour_{int(threshold)}.png")), colour_debug)

    colour_mask = distance <= 15.0
    crop = colour_mask[y1:y2, x1:x2]

    def longest_run(values):
        best_start = best_end = current_start = 0
        for index, present in enumerate(list(values) + [False]):
            if present:
                continue
            if index - current_start > best_end - best_start:
                best_start, best_end = current_start, index
            current_start = index + 1
        return best_start, best_end

    column_present = crop.sum(axis=0) >= crop.shape[0] * 0.58
    rel_x1, rel_x2 = longest_run(column_present)
    selected_columns = crop[:, rel_x1:rel_x2]
    row_present = selected_columns.sum(axis=1) >= max(1, selected_columns.shape[1]) * 0.15
    rel_y1, rel_y2 = longest_run(row_present)
    refined = (x1 + rel_x1, y1 + rel_y1, x1 + rel_x2, y1 + rel_y2)
    print("REFINED", refined)
    rx1, ry1, rx2, ry2 = refined
    refined_debug = frame.copy()
    cv2.rectangle(refined_debug, (rx1, ry1), (rx2, ry2), (0, 0, 255), 3)
    cv2.imwrite(str(output.with_name("box_refined.png")), refined_debug)
print(f"saved={output}")
