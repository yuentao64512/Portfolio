# Pipe-cone YOLO26l detector

This project contains 120 frames extracted from `IMG_7816.MOV` through
`IMG_7820.MOV`, one `pipe_cone` bounding box per image, video-aware dataset
splits, and a 100-epoch Ultralytics YOLO26l training script.

The bounding box covers the pipe/cap and white trapezoidal device body. It
intentionally excludes the temporary black support legs. Other objects,
including the yellow caution cone, are background and are not labeled.

## Dataset split

| Split | Source videos | Images |
|---|---|---:|
| train | IMG_7816, IMG_7817, IMG_7820 | 85 |
| val | IMG_7819 | 27 |
| test | IMG_7818 | 8 |

Keeping entire videos in one split prevents neighboring video frames from
leaking into both training and evaluation. `IMG_7818` is the challenging
long-range test clip. The footage contains no distance telemetry, so its exact
physical range cannot be verified as 30 m; the smallest visible instances are
used as the long-range examples.

## Reproduce training

```bash
cd /home/robotx/pipe_cones
.venv/bin/python tools/validate_dataset.py
.venv/bin/python train_yolo26l.py
```

The final checkpoint is written to:

```text
/home/robotx/pipe_cones/runs/yolo26l_100e/weights/best.pt
```

Run test-set evaluation with:

```bash
.venv/bin/yolo detect val \
  model=runs/yolo26l_100e/weights/best.pt \
  data=dataset.yaml split=test imgsz=960 device=0
```
