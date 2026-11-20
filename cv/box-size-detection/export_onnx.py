"""Export the selected YOLO11n checkpoint to ONNX at 640-pixel input size."""

from pathlib import Path
from ultralytics import YOLO

weights = Path('/home/robotx/box_size_training/runs/yolo11n_dataset_v2/weights/best.pt')
print(YOLO(str(weights)).export(format='onnx', imgsz=640, device='cpu', opset=17))
