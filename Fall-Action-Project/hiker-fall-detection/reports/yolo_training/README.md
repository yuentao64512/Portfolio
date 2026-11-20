# YOLO detector training artifacts

This folder contains the outputs from the completed Ultralytics YOLO run:

- `results.csv` — metrics and losses for all 80 epochs
- `results.png` — training/validation curves
- `labels.jpg` — class and box-position distribution
- `confusion_matrix.png` and `confusion_matrix_normalized.png`
- `BoxP_curve.png`, `BoxR_curve.png`, `BoxF1_curve.png`, `BoxPR_curve.png`
- `train_batch*.jpg` and `val_batch*.jpg` — training/validation examples
- `best.pt` and `last.pt` — YOLO weights
- `data.yaml` — dataset configuration
- `dataset_manifest.csv` — one row per labeled image
- `dataset_summary.json` — dataset counts and class balance

Dataset source (not duplicated into this workspace because it is about 635 MB):

`/home/robotx/fall_action_project/data/AERIAL_UPRIGHT_DOWN/yolo_autolabel_v1`

The dataset contains 232 images and 232 single-person bounding boxes: 186
training images and 46 validation images. Classes are `upright` and `down`.
