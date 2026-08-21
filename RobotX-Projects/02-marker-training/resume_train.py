from pathlib import Path
from ultralytics import YOLO

checkpoint = Path(
    r"C:\Users\OptiPrime-UAV\yolov11_train\runs\detect\train-19\weights\last.pt"
)

if not checkpoint.exists():
    raise FileNotFoundError(f"Checkpoint not found: {checkpoint}")

model = YOLO(str(checkpoint))

results = model.train(resume=True)