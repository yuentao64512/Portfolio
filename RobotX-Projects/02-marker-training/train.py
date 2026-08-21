from ultralytics import YOLO

data_yaml = r"C:\Users\OptiPrime-UAV\yolov11_train\water_test\data.yaml"

model = YOLO("yolo11l.pt")

results = model.train(
    data=data_yaml,
    epochs=150,
    imgsz=1280,
    batch=1,
    device=0,
    workers=0,
    patience=30,
    cache=False,
    amp=False
)