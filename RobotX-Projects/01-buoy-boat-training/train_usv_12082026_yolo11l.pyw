from __future__ import annotations

from datetime import datetime
from pathlib import Path
import csv
import traceback
import sys

import cv2
from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
DESKTOP = Path.home() / "Desktop"
JOB = ROOT / "usv_12082026_yolo11l"
DATASET = JOB / "dataset"
LOG_FILE = JOB / "training.log"
# ULTRALYTICS_LOG = open(JOB / "ultraly18tics_console.log", "a", buffering=1, encoding="utf-8")
# sys.stdout = ULTRALYTICS_LOG
# sys.stderr = ULTRALYTICS_LOG
VIDEO_EXTENSIONS = {".mp4", ".mov", ".avi", ".mkv", ".m4v", ".wmv"}
SAMPLE_FPS = 1000.0
IMAGE_SIZE = 1280
DEVICE = 0
BUOY_EPOCHS = 100
HUMAN_EPOCHS = 50


def log(message: str) -> None:
    JOB.mkdir(parents=True, exist_ok=True)
    line = f"[{datetime.now():%Y-%m-%d %H:%M:%S}] {message}"
    with LOG_FILE.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")


def intersection_over_marker(marker: tuple[float, float, float, float],
                             human: tuple[float, float, float, float]) -> float:
    mx1, my1, mx2, my2 = marker
    hx1, hy1, hx2, hy2 = human
    iw = max(0.0, min(mx2, hx2) - max(mx1, hx1))
    ih = max(0.0, min(my2, hy2) - max(my1, hy1))
    marker_area = max(1.0, (mx2 - mx1) * (my2 - my1))
    return (iw * ih) / marker_area


def yolo_line(class_id: int, box: tuple[float, float, float, float],
              width: int, height: int) -> str:
    x1, y1, x2, y2 = box
    x1 = max(0.0, min(float(width), x1))
    y1 = max(0.0, min(float(height), y1))
    x2 = max(0.0, min(float(width), x2))
    y2 = max(0.0, min(float(height), y2))
    cx = ((x1 + x2) / 2.0) / width
    cy = ((y1 + y2) / 2.0) / height
    bw = (x2 - x1) / width
    bh = (y2 - y1) / height
    return f"{class_id} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}"


def find_videos() -> list[Path]:
    videos = sorted(
        path for path in DESKTOP.iterdir()
        if path.is_file()
        and path.suffix.lower() in VIDEO_EXTENSIONS
        and path.name.upper().startswith("DJI_")
    )
    if len(videos) < 3:
        videos = sorted(
            path for path in DESKTOP.rglob("*")
            if path.is_file()
            and path.suffix.lower() in VIDEO_EXTENSIONS
            and path.name.upper().startswith("DJI_")
        )
    if len(videos) != 3:
        raise RuntimeError(f"Expected exactly 3 DJI videos on Desktop, found {len(videos)}: {videos}")
    return videos


def marker_weight_path() -> Path:
    candidates = [
        ROOT / "runs" / "detect" / "train-19" / "weights" / "best.pt",
        ROOT / "runs" / "detect" / "train-15" / "weights" / "best.pt",
    ]
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("No existing buoy marker weight was found in train-19 or train-15.")


def prepare_dataset() -> Path:
    for split in ("train", "val"):
        (DATASET / "images" / split).mkdir(parents=True, exist_ok=True)
        (DATASET / "labels" / split).mkdir(parents=True, exist_ok=True)
    review_dir = JOB / "review_samples"
    review_dir.mkdir(parents=True, exist_ok=True)

    videos = find_videos()
    marker_weights = marker_weight_path()
    log("Videos: " + ", ".join(str(path) for path in videos))
    log(f"Buoy teacher weights: {marker_weights}")
    log("Loading teacher models.")
    marker_model = YOLO(str(marker_weights))
    coco_model = YOLO("yolo11l.pt")
    marker_names = marker_model.names

    csv_path = JOB / "autolabel_review.csv"
    saved = 0
    rows: list[list[object]] = []

    for video_index, video_path in enumerate(videos):
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open {video_path}")
        fps = max(1.0, float(cap.get(cv2.CAP_PROP_FPS)))
        frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        step = max(1, int(round(fps / SAMPLE_FPS)))
        log(f"Extracting {video_path.name}: fps={fps:.2f}, frames={frame_count}, step={step}")

        frame_index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame_index % step != 0:
                frame_index += 1
                continue

            height, width = frame.shape[:2]
            marker_result = marker_model.predict(
                source=frame,
                conf=0.12,
                iou=0.50,
                imgsz=IMAGE_SIZE,
                device=DEVICE,
                verbose=False,
            )[0]
            coco_result = coco_model.predict(
                source=frame,
                conf=0.25,
                iou=0.50,
                classes=[0, 8],
                imgsz=IMAGE_SIZE,
                device=DEVICE,
                verbose=False,
            )[0]

            humans: list[tuple[float, float, float, float]] = []
            boats: list[tuple[float, float, float, float]] = []
            if coco_result.boxes is not None:
                for box, class_value in zip(
                    coco_result.boxes.xyxy.cpu().tolist(),
                    coco_result.boxes.cls.cpu().tolist(),
                ):
                    coords = tuple(float(value) for value in box)
                    if int(class_value) == 0:
                        humans.append(coords)
                    elif int(class_value) == 8:
                        boats.append(coords)

            labels: list[tuple[int, tuple[float, float, float, float], str]] = []
            if marker_result.boxes is not None:
                for box, class_value in zip(
                    marker_result.boxes.xyxy.cpu().tolist(),
                    marker_result.boxes.cls.cpu().tolist(),
                ):
                    coords = tuple(float(value) for value in box)
                    source_name = str(marker_names[int(class_value)]).lower()
                    if "green" in source_name:
                        target_id, target_name = 0, "green_buoy"
                    elif "red" in source_name or "orange" in source_name:
                        target_id, target_name = 1, "orange_red_buoy"
                    else:
                        continue
                    if any(intersection_over_marker(coords, human) >= 0.20 for human in humans):
                        continue
                    labels.append((target_id, coords, target_name))

            for boat in boats:
                labels.append((2, boat, "boat"))

            if True:
                minute_block = int(frame_index / fps / 60.0)
                split = "val" if (video_index + minute_block) % 5 == 0 else "train"
                stem = f"{video_path.stem}_{frame_index:08d}"
                image_path = DATASET / "images" / split / f"{stem}.jpg"
                label_path = DATASET / "labels" / split / f"{stem}.txt"
                cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
                label_path.write_text(
                    "\n".join(yolo_line(class_id, box, width, height)
                              for class_id, box, _ in labels) + "\n",
                    encoding="ascii",
                )

                counts = {
                    "green_buoy": sum(name == "green_buoy" for _, _, name in labels),
                    "orange_red_buoy": sum(name == "orange_red_buoy" for _, _, name in labels),
                    "boat": sum(name == "boat" for _, _, name in labels),
                }
                rows.append([
                    video_path.name,
                    frame_index,
                    split,
                    image_path.name,
                    counts["green_buoy"],
                    counts["orange_red_buoy"],
                    counts["boat"],
                    len(humans),
                ])

                if saved % 25 == 0:
                    preview = frame.copy()
                    colors = {0: (0, 255, 0), 1: (0, 100, 255), 2: (255, 180, 0)}
                    for class_id, (x1, y1, x2, y2), name in labels:
                        cv2.rectangle(
                            preview,
                            (int(x1), int(y1)),
                            (int(x2), int(y2)),
                            colors[class_id],
                            3,
                        )
                        cv2.putText(
                            preview,
                            name,
                            (int(x1), max(25, int(y1) - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7,
                            colors[class_id],
                            2,
                        )
                    cv2.imwrite(str(review_dir / f"{stem}.jpg"), preview)

                saved += 1
                if saved % 50 == 0:
                    log(f"Saved {saved} labeled frames.")

            frame_index += 1

        cap.release()

    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "video", "frame", "split", "image",
            "green_buoys", "orange_red_buoys", "boats", "humans",
        ])
        writer.writerows(rows)

    if saved < 50:
        raise RuntimeError(f"Only {saved} labeled frames were produced; stopping before training.")

    data_yaml = JOB / "data.yaml"
    data_yaml.write_text(
        f"path: {DATASET.as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "names:\n"
        "  0: green_buoy\n"
        "  1: orange_red_buoy\n"
        "  2: boat\n",
        encoding="ascii",
    )
    log(f"Dataset ready with {saved} labeled frames. YAML: {data_yaml}")
    return data_yaml


def train_buoy_boat(data_yaml: Path) -> None:
    log("Starting YOLO11L buoy and boat training.")
    model = YOLO(str(ROOT / "runs" / "usv_buoys_boat_yolo11l_12082026-3" / "weights" / "last.pt"))
    model.train(
        resume=True,
        data=str(data_yaml),
        epochs=BUOY_EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=1,
        device=DEVICE,
        workers=4,
        patience=20,
        pretrained=True,
        cache=False,
        project=str(ROOT / "runs"),
        name="usv_buoys_boat_yolo11l_12082026",
    )
    log("Buoy and boat training completed.")


def find_human_yaml() -> Path | None:
    human_root = ROOT / "human_reject_dataset_fullframe"
    for name in ("data.yaml", "dataset.yaml"):
        candidate = human_root / name
        if candidate.exists():
            return candidate
    matches = sorted(human_root.rglob("*.yaml"))
    return matches[0] if matches else None


def train_human_rejector() -> None:
    human_yaml = find_human_yaml()
    if human_yaml is None:
        log("Human dataset YAML not found; skipped separate human model training.")
        return
    log(f"Starting YOLO11L human detector training from {human_yaml}.")
    model = YOLO("yolo11l.pt")
    model.train(
        data=str(human_yaml),
        epochs=HUMAN_EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=1,
        device=DEVICE,
        workers=0,
        patience=15,
        pretrained=True,
        cache=False,
        project=str(ROOT / "runs"),
        name="human_reject_yolo11l_12082026",
    )
    log("Human detector training completed.")


def main() -> None:
    log("Remote-only USV training job started.")
    data_yaml = JOB / "data.yaml"
    train_buoy_boat(data_yaml)
    train_human_rejector()
    log("All requested training completed successfully.")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        log("FAILED:\n" + traceback.format_exc())
        raise
