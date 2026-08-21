from pathlib import Path

from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
DATA_YAML = ROOT / "yolo_train" / "data.yaml"
PREVIOUS_BEST = ROOT / "runs" / "dji_marker_yolo_train" / "weights" / "best.pt"
HARD_NEGATIVE_BEST = ROOT / "runs" / "dji_marker_hard_negative_retrain" / "weights" / "best.pt"
BASE_MODEL = ROOT / "yolo11m.pt"


for candidate in (HARD_NEGATIVE_BEST, PREVIOUS_BEST, BASE_MODEL):
    if candidate.exists():
        model_path = candidate
        break
else:
    model_path = "yolo11m.pt"

model = YOLO(str(model_path))

model.train(
    data=str(DATA_YAML),
    epochs=100,
    imgsz=960,
    batch=1,
    device=0,
    workers=0,
    patience=25,
    cache=False,
    amp=False,
    project=str(ROOT / "runs"),
    name="dji_marker_human_reject_retrain",
)
