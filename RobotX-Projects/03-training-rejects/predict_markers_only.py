from pathlib import Path

from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
MODEL = ROOT / "runs" / "dji_marker_human_reject_retrain" / "weights" / "best.pt"


model = YOLO(str(MODEL))
model.predict(
    source=str(ROOT / "predict_source"),
    conf=0.65,
    imgsz=960,
    classes=[0, 1, 2],
    save=True,
    project=str(ROOT / "runs"),
    name="markers_only_predict",
)
