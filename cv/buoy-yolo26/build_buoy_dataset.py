from pathlib import Path
import csv
import json
import shutil

import cv2
import numpy as np
from ultralytics import YOLOE

ROOT = Path("/home/robotx/buoy_yolo26")
SOURCE = ROOT / "source_frames"
DATASET = ROOT / "dataset"
PROMPTS = [
    "white floating buoy",
    "floating pool buoy marker",
    "white trapezoid-shaped buoy",
]


def split_for(index: int) -> str:
    block = index // 20
    if block in {2, 5}:
        return "val"
    if block == 8:
        return "test"
    return "train"


def box_iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1e-9)


for split in ("train", "val", "test"):
    (DATASET / "images" / split).mkdir(parents=True, exist_ok=True)
    (DATASET / "labels" / split).mkdir(parents=True, exist_ok=True)

images = sorted(SOURCE.glob("*.jpg"))
model = YOLOE(str(ROOT / "yoloe-26l-seg.pt"))
model.set_classes(PROMPTS)
rows = []
overlays = []
for index, path in enumerate(images):
    result = model.predict(str(path), imgsz=960, conf=0.18, iou=0.45,
                           device=0, half=True, verbose=False)[0]
    split = split_for(index)
    destination_image = DATASET / "images" / split / path.name
    destination_label = DATASET / "labels" / split / f"{path.stem}.txt"
    shutil.copy2(path, destination_image)

    image = cv2.imread(str(path))
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    accepted = []
    if result.boxes is not None and result.masks is not None:
        boxes = result.boxes.xyxy.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        polygons = result.masks.xyn
        candidates = sorted(zip(boxes, confidences, polygons), key=lambda x: -float(x[1]))
        for box, confidence, polygon in candidates:
            box_width = float(box[2] - box[0])
            box_height = float(box[3] - box[1])
            aspect = box_width / max(box_height, 1.0)
            area_fraction = box_width * box_height / (width * height)
            if box_width < 20 or box_height < 18:
                continue
            if not 0.60 <= aspect <= 3.20:
                continue
            if not 0.00015 <= area_fraction <= 0.20:
                continue
            if len(polygon) < 3:
                continue
            pixel_points = np.round(np.clip(np.asarray(polygon), 0.0, 1.0) * [width, height]).astype(np.int32)
            candidate_mask = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(candidate_mask, [pixel_points], 255)
            selected = candidate_mask > 0
            white_fraction = float(np.mean((hsv[:, :, 1][selected] < 90) &
                                           (hsv[:, :, 2][selected] > 135))) if selected.any() else 0.0
            if white_fraction < 0.30:
                continue
            if any(box_iou(box, kept[0]) > 0.45 for kept in accepted):
                continue
            accepted.append((box, float(confidence), polygon))

    lines = []
    overlay = image.copy()
    for box, confidence, polygon in accepted:
        points = np.clip(np.asarray(polygon, dtype=float), 0.0, 1.0)
        coords = " ".join(f"{value:.6f}" for value in points.reshape(-1))
        lines.append(f"0 {coords}")
        pixel_points = np.round(points * [width, height]).astype(np.int32)
        cv2.polylines(overlay, [pixel_points], True, (0, 255, 0), 4, cv2.LINE_AA)
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 180, 255), 3)
        cv2.putText(overlay, f"buoy {confidence:.2f}", (x1, max(25, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, f"buoy {confidence:.2f}", (x1, max(25, y1 - 7)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1, cv2.LINE_AA)
    destination_label.write_text("\n".join(lines) + ("\n" if lines else ""))

    thumb = cv2.resize(overlay, (552, 311))
    cv2.putText(thumb, f"{index:03d} {split} n={len(accepted)}", (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(thumb, f"{index:03d} {split} n={len(accepted)}", (8, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
    overlays.append(thumb)
    rows.append({"index": index, "file": path.name, "split": split,
                 "instances": len(accepted),
                 "min_conf": min((x[1] for x in accepted), default="")})

with (ROOT / "annotation_report.csv").open("w", newline="") as handle:
    writer = csv.DictWriter(handle, fieldnames=rows[0].keys())
    writer.writeheader()
    writer.writerows(rows)

(DATASET / "data.yaml").write_text(
    f"path: {DATASET}\ntrain: images/train\nval: images/val\ntest: images/test\n"
    "names:\n  0: buoy\n"
)

for start in range(0, len(overlays), 40):
    group = overlays[start:start + 40]
    cols = 4
    rows_count = int(np.ceil(len(group) / cols))
    sheet = np.full((rows_count * 311, cols * 552, 3), 245, dtype=np.uint8)
    for i, thumb in enumerate(group):
        row, col = divmod(i, cols)
        sheet[row * 311:(row + 1) * 311, col * 552:(col + 1) * 552] = thumb
    cv2.imwrite(str(ROOT / f"audit_{start:03d}_{start + len(group) - 1:03d}.jpg"), sheet,
                [cv2.IMWRITE_JPEG_QUALITY, 94])

summary = {
    "images": len(images),
    "instances": sum(row["instances"] for row in rows),
    "negative_images": sum(row["instances"] == 0 for row in rows),
    "by_split": {
        split: {
            "images": sum(row["split"] == split for row in rows),
            "instances": sum(row["instances"] for row in rows if row["split"] == split),
        }
        for split in ("train", "val", "test")
    },
}
(ROOT / "dataset_summary.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))
