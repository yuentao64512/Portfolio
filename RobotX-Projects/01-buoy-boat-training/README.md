# Aerial Buoy and Boat Training

This project prepares and trains a YOLO11L aerial detector for three maritime classes: `green_buoy`, `orange_red_buoy`, and `boat`. Images and video are captured from a drone looking down at the water. It also includes a general prediction script that can select the newest `best.pt` checkpoint.

## Detection cases

- Detect navigation buoys in aerial drone footage.
- Distinguish the two course-side buoy colours from above.
- Detect and locate boats during aerial maritime observation.
- Run predictions on a drone camera feed, recorded flight video, image, or folder after training.

## Files

- `train_usv_12082026_yolo11l.pyw`: dataset preparation and training workflow from the latest aerial experiment. The historical filename is retained to avoid breaking references.
- `predict_delta.py`: inference and annotated-output utility.
- `data.example.yaml`: expected YOLO class and dataset layout.

## Dataset layout

```text
dataset/
  images/train/
  images/val/
  labels/train/
  labels/val/
```

Copy `data.example.yaml` to `data.yaml`, replace its absolute `path` with your local dataset path, and check the configurable constants at the top of both scripts.

## Run

```powershell
python train_usv_12082026_yolo11l.pyw
python predict_delta.py
```

The latest trained checkpoint was about 146 MB and was intentionally excluded because it exceeds GitHub's normal 100 MB per-file limit. Publish it with Git LFS or a GitHub Release.
