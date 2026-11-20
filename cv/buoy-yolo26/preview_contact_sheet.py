from pathlib import Path
import cv2
import numpy as np

video = Path("/home/robotx/buoy_yolo26/runs/full_video_preview/rgb_left.avi")
output = Path("/home/robotx/buoy_yolo26/final_prediction_contact_sheet.jpg")
capture = cv2.VideoCapture(str(video))
count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
frames = []
for index in np.linspace(0, count - 1, 24).round().astype(int):
    capture.set(cv2.CAP_PROP_POS_FRAMES, int(index))
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError(index)
    frames.append(cv2.resize(frame, (552, 311)))
capture.release()
sheet = np.full((6 * 311, 4 * 552, 3), 245, dtype=np.uint8)
for i, frame in enumerate(frames):
    row, col = divmod(i, 4)
    sheet[row * 311:(row + 1) * 311, col * 552:(col + 1) * 552] = frame
cv2.imwrite(str(output), sheet, [cv2.IMWRITE_JPEG_QUALITY, 94])
print(output)
