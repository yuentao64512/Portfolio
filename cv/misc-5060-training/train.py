from pathlib import Path
from ultralytics import YOLO

data_yaml = str(Path("~/my_yolo_dataset/data.yaml").expanduser())
model_weights = "yolo11n.pt"   # or your own pretrained .pt path

model = YOLO(model_weights)
results = model.train(data=data_yaml, epochs=100, imgsz=640)
