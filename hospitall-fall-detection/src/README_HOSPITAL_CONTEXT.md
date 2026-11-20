# Hospital fall pipeline — steps 1 and 2

This extension preserves the existing hiker/drone classifiers. It adds portable pose deployment and per-camera hospital scene context without yet changing the action classifier or alarm logic.

## Current data check

The project contains 169 MP4 files and 466 extracted JPG frames. It already has temporal GMDCSA24 pose sequences, so it is not limited to still-image training. Hospital hard-negative video—especially ordinary lying in beds, nurses bending over beds, deliberate floor sitting and blanket/rail occlusion—is still required before a hospital alarm can be evaluated responsibly.

## Step 1 — pose model export

The existing pose source is `yolo11n-pose.pt`. A portable 640-pixel ONNX and metadata are generated with:

```bash
cd /home/robotx/fall_action_project
source .venv/bin/activate
python src/export_pose_models.py \
  --weights yolo11n-pose.pt \
  --output-dir models/pose_exports \
  --imgsz 640
```

TensorRT engines are tied to the target GPU, TensorRT and JetPack versions. Build FP16 on the Jetson Orin Nano itself:

```bash
cd /path/to/fall_action_project
chmod +x src/build_jetson_pose_engine.sh
src/build_jetson_pose_engine.sh yolo11n-pose.pt 640 2
```

Do not copy an RTX 5060 TensorRT engine to the Jetson.

## Step 2 — fixed-camera hospital ROIs

Create a config for each physical camera:

```bash
python src/draw_context_rois.py \
  --source 0 \
  --camera-id ward_camera_1 \
  --output configs/ward_camera_1_context.json
```

Editor controls:

- Left click: add polygon point
- Right click or `u`: undo point
- Enter: finish the current polygon
- `1`: bed
- `2`: floor
- `3`: couch
- `4`: chair
- `5`: stretcher
- `6`: ignore/privacy region
- `d`: delete the most recently completed region
- `s`: save
- `q`: save completed regions and quit

Coordinates are normalized, so the same config scales with the frame resolution. Redraw the ROIs whenever a camera is physically moved.

Validate pose, ByteTrack IDs and context overlaps:

```bash
python src/run_hospital_context_preview.py \
  --source 0 \
  --pose-model yolo11n-pose.pt \
  --roi-config configs/ward_camera_1_context.json
```

Optional automatic COCO furniture fallback:

```bash
python src/run_hospital_context_preview.py \
  --source 0 \
  --pose-model yolo11n-pose.pt \
  --roi-config configs/ward_camera_1_context.json \
  --furniture-model /home/robotx/yolo11n.pt
```

The fallback supports COCO `bed`, `couch` and `chair`. A fixed polygon is preferable for a fixed hospital camera; generic COCO detections may miss hospital-specific beds and stretchers.

## Verification

```bash
PYTHONPATH=src python -m unittest -v src/test_hospital_context.py
```

The current preview computes bed/furniture and floor bbox-overlap values for every ByteTrack person ID. The temporal hospital features, classifier and alarm state machine belong to steps 3–6 and should use these tested context values.

## Context-aware recorded-video demo

The hospital video demo fuses the pose action with a fixed camera's bed and
floor polygons.  It reports `PATIENT_ON_BED`, `PATIENT_ON_FLOOR`,
`PATIENT_DOWN_ON_FLOOR`, or `FALL_ALERT_ON_FLOOR` and saves an annotated MP4
plus a frame-level CSV:

```bash
python src/run_hospital_video_demo.py \
  --video test_videos/synthetic_hospital_fall/synthetic_hospital_bed_to_floor.mp4 \
  --roi-config configs/synthetic_ward_context.json \
  --classifier models/current_pose_rf.pkl \
  --output outputs/synthetic_hospital_context.mp4 \
  --headless
```

The alert requires both a floor-level down pose and either a recent bed exit
or a recent `falling_down` action. A person who begins the clip lying on the
floor is identified as down on the floor but is not automatically labelled as
a witnessed fall. Each physical camera requires its own ROI config, and the
polygons must be redrawn if that camera moves.
