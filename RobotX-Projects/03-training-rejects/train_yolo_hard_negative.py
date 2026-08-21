from pathlib import Path

from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
DATA_YAML = ROOT / "yolo_train" / "data.yaml"
PREVIOUS_BEST = ROOT / "runs" / "dji_marker_yolo_train" / "weights" / "best.pt"


model_path = PREVIOUS_BEST if PREVIOUS_BEST.exists() else ROOT / "yolo11m.pt"
model = YOLO(str(model_path))

model.train(
    data=str(DATA_YAML),
    epochs=80,
    imgsz=960,
    batch=1,
    device=0,
    workers=0,
    patience=20,
    cache=False,
    amp=False,
    project=str(ROOT / "runs"),
    name="dji_marker_hard_negative_retrain",
)
