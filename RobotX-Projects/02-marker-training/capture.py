import cv2
import os

CAMERA_ID = 1

TRAIN_DIR = "dataset/images_1/train"
VAL_DIR = "dataset/images_1/val"

os.makedirs(TRAIN_DIR, exist_ok=True)
os.makedirs(VAL_DIR, exist_ok=True)

cap = cv2.VideoCapture(CAMERA_ID)

if not cap.isOpened():
    print("Failed to open camera")
    exit()

image_count = 0

print("Press SPACE to capture")
print("Press Q to quit")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to read frame")
        break

    cv2.imshow("YOLO Dataset Capture", frame)

    key = cv2.waitKey(1) & 0xFF

    # SPACE = save image
    if key == 32:
        image_count += 1

        filename = f"img_{image_count:05d}.jpg"

        # 80/20 split
        if image_count % 5 == 0:
            save_path = os.path.join(VAL_DIR, filename)
            split = "VAL"
        else:
            save_path = os.path.join(TRAIN_DIR, filename)
            split = "TRAIN"

        cv2.imwrite(save_path, frame)

        print(f"[{split}] Saved: {filename}")

    # Q = quit
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()