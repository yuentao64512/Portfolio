import argparse
from ultralytics import YOLO


parser = argparse.ArgumentParser(description="Run the trained single-class RobotX buoy segmentation model.")
parser.add_argument("source", help="Image, video, camera index, stream URL, or directory")
parser.add_argument("--conf", type=float, default=0.25)
parser.add_argument("--imgsz", type=int, default=768)
parser.add_argument("--show", action="store_true")
parser.add_argument("--save", action="store_true")
args = parser.parse_args()

model = YOLO("/home/robotx/buoy_yolo26/buoy_yolo26l_seg_best.pt")
for result in model.predict(source=args.source, conf=args.conf, imgsz=args.imgsz, device=0,
                            show=args.show, save=args.save, stream=True):
    count = 0 if result.boxes is None else len(result.boxes)
    print(f"{result.path}: {count} buoy(s)")
