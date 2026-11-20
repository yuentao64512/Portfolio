# Two-stage RobotX buoy and digit recognition

This project trains two YOLO11n detectors:

1. Model A detects one class, `buoy`, on full frames.
2. Model B detects digits `0` through `9` on crops from Model A.

The original labels are never edited. `build_datasets.py` derives both datasets
from one full-frame 11-class export. `import_legacy.py` creates a provisional
baseline from the previously reviewed Model A and Model B datasets.

## Full-frame label order

| ID | Class |
|---:|---|
| 0 | buoy |
| 1-10 | digits 0-9 |

Box the complete buoy from top marker to cyan base. Box every readable diamond
digit, including both visible faces. Do not label reflections or unreadable
digits.

## Workflow

```bash
./.venv/bin/python buoy_project/extract_frames.py
# Label frames and place paired image/txt files in buoy_project/labelled
./.venv/bin/python buoy_project/split_dataset.py
./.venv/bin/python buoy_project/build_datasets.py
./.venv/bin/python buoy_project/train.py --model both
./.venv/bin/python buoy_project/evaluate.py
```

Existing reviewed data can be imported without touching its source:

```bash
./.venv/bin/python buoy_project/import_legacy.py
```

The legacy digit dataset has no reviewed examples of digits 1 or 7. A model
trained from it is provisional and cannot be expected to recognize all digits.

## Background training

```bash
buoy_project/start_training.sh
buoy_project/status_training.sh
```

## Runtime

Export both finished weights on the AGX Orin with `export_orin.py`. Run
`predict_orin.py --source zed` for the ZED camera or pass a video path.
Runtime code performs vision only and does not control servos, pumps, or vehicle
motion.
