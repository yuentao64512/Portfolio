# Buoy two-stage recognition on AGX Orin

This directory contains the important trained artifacts and an AGX-native runtime.

- Model A: buoy LED colour, no-LED buoy, dock, home and obstacle.
- Model B: circle/diamond shape and digits 0-9 on Model A buoy crops.
- Runtime: ByteTrack plus a 30-frame vote for stable labels.
- Acceleration: TensorRT FP16 engines built locally on this AGX Orin.

Check export status:

```bash
/home/optiprime-agxorin/buoy_robotx/runtime/status_agx.sh
```

Run on a video:

```bash
/home/optiprime-agxorin/buoy_robotx/runtime/run_buoy_two_stage.sh \
  --source /path/to/video.mp4 \
  --output /home/optiprime-agxorin/buoy_robotx/output/annotated.mp4 \
  --log-csv /home/optiprime-agxorin/buoy_robotx/output/detections.csv
```

Run a camera (change `0` if required):

```bash
/home/optiprime-agxorin/buoy_robotx/runtime/run_buoy_two_stage.sh --source 0 --show
```

The recognizer automatically prefers `models/model_a_best.engine` and
`models/model_b_best.engine`. If engines are absent, it falls back to `.pt`.

## Dock recognition only

Recognise `dock` and `home` targets from a camera:

```bash
/home/optiprime-agxorin/buoy_robotx/runtime/run_dock_recognition.sh \
  --source 0 \
  --output /home/optiprime-agxorin/buoy_robotx/output/dock_camera.mp4 \
  --log-csv /home/optiprime-agxorin/buoy_robotx/output/dock_camera.csv
```

For a recorded video, replace `--source 0` with the video path. Add `--show`
only when running from the AGX desktop. The program reports `LEFT`, `RIGHT`, or
`ALIGNED`; it does not send movement commands to the vessel.
