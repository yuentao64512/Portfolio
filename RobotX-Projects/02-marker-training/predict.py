from ultralytics import YOLO
from pathlib import Path
import csv

# ===========================
# CHANGE THESE PATHS
# ===========================
MODEL_PATH = r"C:\Users\OptiPrime-UAV\yolov11_train\runs\dji_marker_human_reject_retrain-2\weights\best.pt"
IMAGE_FOLDER = r"C:\Users\OptiPrime-UAV\yolov11_train\water_test\images\train"

model = YOLO(MODEL_PATH)

image_paths = list(Path(IMAGE_FOLDER).glob("*.*"))

results_list = []

for img_path in image_paths:

    results = model.predict(
        source=str(img_path),
        imgsz=1280,
        conf=0.25,
        save=True
    )

    result = results[0]
    boxes = result.boxes

    hit_count = len(boxes)

    print("=" * 60)
    print(f"Image: {img_path.name}")
    print(f"Hits : {hit_count}")

    confidences = []

    for i, box in enumerate(boxes):
        confidence = float(box.conf[0])
        cls = int(box.cls[0])
        class_name = model.names[cls]

        confidences.append(round(confidence, 4))

        print(
            f"Detection {i+1}: "
            f"{class_name} | "
            f"Confidence = {confidence:.4f}"
        )

    results_list.append([
        img_path.name,
        hit_count,
        "; ".join([f"{c:.4f}" for c in confidences])
    ])

# Save CSV
with open("prediction_results.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "Image",
        "Hits",
        "Confidences"
    ])
    writer.writerows(results_list)

print("\nFinished!")
print("Results saved to prediction_results.csv")