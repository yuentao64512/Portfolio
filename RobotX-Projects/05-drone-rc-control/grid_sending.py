from ultralytics import YOLO
import cv2
import serial
import json
import time

MODEL_PATH = r"C:\Users\OptiPrime-UAV\yolov11_train\runs\detect\train-17\weights\best.pt"

RADIO_PORT = "COM9"
BAUD = 57600

CAMERA_INDEX = 1

GRID_ROWS = 100
GRID_COLS = 100

CONF_THRESHOLD = 0.25
IMG_SIZE = 640

SEND_EVERY_N_FRAMES = 5

model = YOLO(MODEL_PATH)

ser = serial.Serial(RADIO_PORT, BAUD, timeout=0.1)
ser.reset_input_buffer()
ser.reset_output_buffer()

cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    print("Camera cannot open. Try CAMERA_INDEX = 0, 1, or 2.")
    exit()

frame_id = 0

print("YOLO grid sender started.")
print("Press Q to quit.")

def pixel_to_grid(x, y, frame_w, frame_h):
    col = int((x / frame_w) * GRID_COLS)
    row = int((y / frame_h) * GRID_ROWS)

    col = max(0, min(GRID_COLS - 1, col))
    row = max(0, min(GRID_ROWS - 1, row))

    return row, col

def draw_grid(frame):
    h, w = frame.shape[:2]

    for c in range(0, GRID_COLS + 1, 10):
        x = int(c * w / GRID_COLS)
        cv2.line(frame, (x, 0), (x, h), (80, 80, 80), 1)
        cv2.putText(frame, str(c), (x + 2, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

    for r in range(0, GRID_ROWS + 1, 10):
        y = int(r * h / GRID_ROWS)
        cv2.line(frame, (0, y), (w, y), (80, 80, 80), 1)
        cv2.putText(frame, str(r), (2, y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

while True:
    ret, frame = cap.read()

    if not ret:
        print("Failed to read camera frame.")
        break

    frame_id += 1
    h, w = frame.shape[:2]

    results = model.predict(
        source=frame,
        imgsz=IMG_SIZE,
        conf=CONF_THRESHOLD,
        device=0,
        verbose=False
    )

    detections = []

    result = results[0]
    names = result.names

    if result.boxes is not None:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2

            row, col = pixel_to_grid(cx, cy, w, h)
            class_name = names[cls_id]

            detections.append({
                "class": class_name,
                "row": row,
                "col": col,
                "conf": round(conf, 2)
            })

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 0), 2)

            label = f"{class_name} {conf:.2f} R{row} C{col}"
            cv2.putText(frame, label, (int(x1), max(20, int(y1) - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.circle(frame, (int(cx), int(cy)), 4, (0, 0, 255), -1)

    draw_grid(frame)

    if detections and frame_id % SEND_EVERY_N_FRAMES == 0:
        msg = {
            "type": "yolo_grid",
            "frame_id": frame_id,
            "detections": detections,
            "time": time.time()
        }

        line = json.dumps(msg, separators=(",", ":")) + "\n"

        ser.write(line.encode("utf-8"))
        ser.flush()

        print("Sent:", line.strip())

        start_wait = time.time()
        while time.time() - start_wait < 0.5:
            reply_line = ser.readline().decode("utf-8", errors="ignore").strip()

            if reply_line:
                print("Reply from other laptop:", reply_line)
                break

    cv2.imshow("YOLO Grid Sender", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
ser.close()
cv2.destroyAllWindows()