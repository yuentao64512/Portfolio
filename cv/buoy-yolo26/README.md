# RobotX buoy YOLO26L segmentation model

## Result

- Model: Ultralytics YOLO26L segmentation (`yolo26l-seg.pt`)
- Classes: one class, `buoy`
- Training: 100 epochs at 768 px on an RTX 5060 Ti
- Source: `/home/robotx/rgb_left.mp4` (81.87 s, 1,228 frames, 15 FPS, 2208 x 1242)
- Dataset: 200 evenly spaced frames, 324 buoy masks, 56 negative frames
- Split: 140 train, 40 validation, 20 test using held-out time blocks
- Weight SHA-256: `351a554489e9afac598520c519aa83541829ef7224be6a094fbc7f69474ebc4d`

The letters and symbols (`A`, `B`, `C`, `D`, diamond, etc.) are not separate classes. Each label follows the full white buoy body as one `buoy` object.

## Untouched test-block metrics

| Metric | Box | Mask |
|---|---:|---:|
| Precision | 0.981 | 0.981 |
| Recall | 0.963 | 0.963 |
| mAP50 | 0.993 | 0.993 |
| mAP50-95 | 0.874 | 0.686 |

These results use a temporally held-out section of the same pool video. They measure same-session performance, not performance at new locations.

## Files on the RobotX PC

```text
/home/robotx/buoy_yolo26/buoy_yolo26l_seg_best.pt
/home/robotx/buoy_yolo26/run_buoy_inference.py
/home/robotx/buoy_yolo26/dataset/data.yaml
/home/robotx/buoy_yolo26/test_metrics.json
/home/robotx/buoy_yolo26/runs/full_video_preview/rgb_left.avi
/home/robotx/buoy_yolo26/runs/buoy_yolo26l_seg_100e/
```

## Run on a video

```bash
cd /home/robotx/buoy_yolo26
.venv/bin/python run_buoy_inference.py /path/to/video.mp4 --save
```

## Run on a camera

For camera index 0:

```bash
cd /home/robotx/buoy_yolo26
.venv/bin/python run_buoy_inference.py 0 --show
```

## Important generalisation limit

Training masks and aggressive colour, geometry, mosaic, mixup and copy-paste augmentation reduce reliance on the pool background, but one recording cannot establish location-independent recognition. Before deployment, collect and manually verify new footage containing the same buoy shapes at different pools or waterways, lighting conditions, distances, camera angles and backgrounds. Keep that footage out of training for a true external test.
