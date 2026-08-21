from ultralytics import YOLO


data_yaml = r"C:\Users\OptiPrime-UAV\yolov11_train\yolo_train\data.yaml"

model = YOLO("yolo11m.pt")

results = model.train(
    data=data_yaml,
    epochs=150,
    imgsz=960,
    batch=1,
    device=0,
    workers=0,
    patience=30,
    cache=False,
    amp=False,
    project=r"C:\Users\OptiPrime-UAV\yolov11_train\runs",
    name="dji_marker_yolo_train",
)
