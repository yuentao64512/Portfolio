# Pipe-cone detection, Pixhawk geolocation, and 2D mapping

This deployment uses the ZED 2i and the new YOLO26l `pipe_cone` checkpoint.
It follows the three-process design in `cone_detector_20260915` while using
separate files, logs, and UDP ports.

## Start the complete graphical system

Connect the Pixhawk and ZED 2i, then run this from the Orin desktop:

```bash
/home/jetson/pipe_cones/run_pipe_cone_mapping.sh
```

Two windows open: the annotated ZED camera and a live North/East map. The map
shows the vehicle, its heading and recent path, tracked pipe cones, GPS status,
and calculated cone latitude/longitude.

For an explicitly selected Pixhawk device:

```bash
/home/jetson/pipe_cones/run_pipe_cone_mapping.sh \
  --mavlink /dev/serial/by-id/YOUR_PIXHAWK_DEVICE
```

Headless operation:

```bash
/home/jetson/pipe_cones/run_pipe_cone_mapping.sh --headless
```

## Camera mount calibration

The default assumes the ZED points exactly forward and is colocated with the
vehicle navigation origin. Supply measured offsets when that is not true:

```bash
/home/jetson/pipe_cones/run_pipe_cone_mapping.sh \
  --camera-yaw-deg 0 \
  --camera-forward-m 0.25 \
  --camera-right-m 0.00
```

Positive yaw is clockwise looking down, positive forward is toward the bow,
and positive right is starboard. These values materially affect the reported
cone coordinates.

## Logs

Each component creates a UTC timestamped run directory:

- `logs/detector/*/pipe_cone_observations.csv`: confidence, distance, ZED XYZ,
  depth quality, and bounding box.
- `logs/map/*/pipe_cone_geolocation.csv`: vehicle pose/GPS and calculated cone
  North/East plus latitude/longitude.
- `logs/map/*/vehicle_state.csv`: Pixhawk pose, GPS fix, satellites, and rates.
- `logs/map/*/pipe_cone_map.json`: current tracked 2D map and geographic data.
- `logs/launcher/*/*.log`: stdout/stderr from all three processes.

Geographic output requires a Pixhawk 3D GPS fix and fresh EKF attitude. Its
accuracy is limited by GPS accuracy, compass heading, camera mounting
calibration, ZED depth, and detector box quality.

## Record synchronized RGB and depth

Stop the mapping launcher first because one process must have exclusive use of
the ZED camera. Then record until Ctrl+C (or Q when preview is enabled):

```bash
/home/jetson/pipe_cones/run_zed_rgbd_recorder.sh --preview
```

For a fixed five-minute HD2K collection:

```bash
/home/jetson/pipe_cones/run_zed_rgbd_recorder.sh --duration 300
```

Recordings are stored under `recordings/<UTC timestamp>/`. Each session has a
rectified `rgb_left.mp4`, one lossless 16-bit `depth_mm/*.png` per video frame,
a `depth_preview.mp4`, `frames.csv`, and `metadata.json`. Depth values are
millimetres; zero means that no reliable depth was available. `frames.csv`
contains the exact ZED timestamp pairing between video frame N and depth image
N. HD2K supports 15 FPS; use `--resolution hd1080 --fps 30` if motion detail is
more important than maximum range detail.
