from ultralytics import YOLO
import cv2

model = YOLO(r"C:\Users\OptiPrime-UAV\yolov11_train\runs\detect\train-9\weights\best.pt")

cap = cv2.VideoCapture(1)  # 0 = default webcam

if not cap.isOpened():
    print("Camera cannot open. Try changing 0 to 1 or 2.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to read camera frame")
        break

    results = model.predict(
        source=frame,
        imgsz=640,
        conf=0.25,
        device=0,
        verbose=False
    )

    annotated_frame = results[0].plot()

    cv2.imshow("YOLO Live Camera Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()