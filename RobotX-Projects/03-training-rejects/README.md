# Training Rejects and Hard Negatives

This project reduces false positives in aerial buoy and marker detection. It supports two approaches: training with empty-label hard-negative drone images and running a separate human detector to suppress marker boxes that overlap people.

## Detection cases

- Reject a person seen from the air whose bright clothing resembles a buoy.
- Reject reflections, shoreline objects, roofs, or background colours that repeatedly trigger the aerial marker model.
- Keep genuine blue, red, and green marker detections while filtering human-overlapping boxes.
- Build a dedicated one-class human detector from full aerial video frames.

## Workflows

### Hard-negative fine-tuning

Place false-positive images or videos in a `hard_negatives` folder, then run:

```powershell
python train_yolo_accept_hard_negatives.py --dataset PATH_TO_MARKER_DATASET --negative-source PATH_TO_REJECT_CASES
```

Hard-negative images receive empty YOLO label files, teaching the detector that these examples contain no marker.

### Dedicated human reject model

```powershell
python extract_label_humans_fullframe.py --video WATER_VIDEO.mp4 --overwrite
python train_human_reject_yolo11l.py
python predict_buoys_reject_humans.py --source TEST_VIDEO.mp4
```

`predict_marker_no_human.py` provides a similar two-model inference path using a pretrained COCO person detector.

## Important

The source scripts contain defaults pointing to the original workspace. Use their command-line arguments where available or update `ROOT`, dataset, and checkpoint paths. Large trained weights and datasets are excluded from this GitHub-ready copy.
