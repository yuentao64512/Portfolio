# Aerial Coloured Marker Training

Utilities for collecting, converting, training, validating, and detecting blue, red, and green course markers in aerial drone imagery with YOLO.

## Detection cases

- Detect coloured competition markers from above.
- Test a trained marker model on recorded drone footage.
- Run live detection from a drone video feed or development camera.
- Convert Pascal VOC XML annotations to YOLO labels.
- Tile high-resolution images so small distant markers occupy more pixels.

## Main workflow

1. Capture or supply labelled images.
2. Arrange them in YOLO `images/train`, `images/val`, `labels/train`, and `labels/val` folders.
3. Copy `data.example.yaml` to `data.yaml` and make its dataset path portable.
4. Train with `train.py` or `train_yolo_train.py`.
5. Test with `predict.py`, `vid_predict.py`, or `vision_test.py`.

```powershell
python train.py
python vid_predict.py
```

## Supporting utilities

- `capture.py`: webcam image capture.
- `jpg_to_yolo.py`: VOC XML to YOLO conversion and split generation.
- `tiles.py`: overlapping image/label tiling.
- `test.py` and `export.py`: FiftyOne review and export.
- `resume_train.py`: resume an interrupted Ultralytics run.

These scripts came from several iterations and retain editable paths near their tops. Point them to this project's dataset and model before use.
