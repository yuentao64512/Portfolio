from pathlib import Path
import json
import shutil

from ultralytics import YOLO

ROOT = Path("/home/robotx/buoy_yolo26")
BEST = ROOT / "runs/buoy_yolo26l_seg_100e/weights/best.pt"
FINAL = ROOT / "buoy_yolo26l_seg_best.pt"
shutil.copy2(BEST, FINAL)

model = YOLO(str(FINAL))
metrics = model.val(
    data=str(ROOT / "dataset/data.yaml"),
    split="test",
    imgsz=768,
    batch=1,
    device=0,
    plots=True,
    project=str(ROOT / "runs"),
    name="test_evaluation",
    exist_ok=True,
)

summary = {
    "box_precision": float(metrics.box.mp),
    "box_recall": float(metrics.box.mr),
    "box_map50": float(metrics.box.map50),
    "box_map50_95": float(metrics.box.map),
    "mask_precision": float(metrics.seg.mp),
    "mask_recall": float(metrics.seg.mr),
    "mask_map50": float(metrics.seg.map50),
    "mask_map50_95": float(metrics.seg.map),
}
(ROOT / "test_metrics.json").write_text(json.dumps(summary, indent=2))
print(json.dumps(summary, indent=2))

model.predict(
    source="/home/robotx/rgb_left.mp4",
    imgsz=768,
    conf=0.25,
    device=0,
    save=True,
    project=str(ROOT / "runs"),
    name="full_video_preview",
    exist_ok=True,
    verbose=False,
)
