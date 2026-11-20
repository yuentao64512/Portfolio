# Actual buoy scripts

Files on the AGX Orin: `/home/optiprime-agxorin/actual_buoy`

## Installed environment

Use the project interpreter for every script:

```bash
cd /home/optiprime-agxorin/actual_buoy
source .venv/bin/activate
```

Installed and verified on this AGX Orin:

- JetPack 6.2.3 / CUDA 12.6 / cuDNN 9.3 / TensorRT 10.3
- ZED SDK 5.4.1 with `pyzed`
- PyTorch 2.8.0 with CUDA, torchvision 0.23.0
- Ultralytics 8.4.124, OpenCV 4.11.0, NumPy 1.26.4
- pymavlink 2.4.49

Quick software-only check:

```bash
.venv/bin/python verify_agx_setup.py
```

## 1. Read-only observer

Connect the ZED and Pixhawk, then run:

```bash
cd /home/optiprime-agxorin/actual_buoy
.venv/bin/python actual_buoy_observer.py \
  --headless --resolution hd2k --fps 15
```

`hd2k` at 15 FPS is the installed default for the observer, approach, and
docking scripts. The first neural-depth run performs a one-time GPU engine
optimization and can take several minutes.

The default Pixhawk link listens for MAVLink UDP at
`udpin:0.0.0.0:14550`. The Orin NX at `192.168.1.218` receives the Pixhawk's
Ethernet MAVLink feed on `10.41.10.1:14550` and forwards telemetry to this AGX
at `192.168.1.99:14550`. Override it for one run with
`--mavlink ENDPOINT`, or set the `ACTUAL_BUOY_MAVLINK` environment variable.
Pass `--mavlink auto` to search `/dev/serial/by-id` and then mutable
`/dev/ttyACM*` paths for a locally connected USB/serial Pixhawk. Logs contain
the vehicle GPS/yaw, buoy depth/bearing, and projected buoy latitude/longitude.
The projection assumes the camera faces vehicle-forward; set
`--camera-yaw-deg`, `--camera-forward-m`, and `--camera-right-m` to the measured
installation offsets.

The installed Orin NX bridge is deliberately telemetry-only. Dry-run and
observer operation work through it, but live approach/docking commands are not
forwarded to the Pixhawk. Do not replace it with a bidirectional bridge until
the vehicle is disarmed and the command path is tested while physically
restrained.

For a camera-only bench test without GPS:

```bash
.venv/bin/python actual_buoy_observer.py \
  --allow-no-pixhawk --headless --frames 60
```

## 2. Guarded approach

Always begin with dry-run. It requires a Pixhawk heartbeat but sends no motion:

```bash
cd /home/optiprime-agxorin/actual_buoy
.venv/bin/python actual_buoy_approach.py \
  --headless --max-runtime-s 30
```

Live mode is intentionally gated. It only supports an ArduPilot
`MAV_TYPE_SURFACE_BOAT` already armed in `GUIDED`; the script never arms or
changes mode. A separate safety operator must continuously refresh the dead-man
file, for example from a second SSH terminal:

```bash
while true; do touch /tmp/actual_buoy_deadman; sleep 0.2; done
```

Then, with the boat restrained for the first test and an operator ready to
disarm:

```bash
.venv/bin/python actual_buoy_approach.py \
  --live --confirm I_ACCEPT_LIVE_MOTION --headless \
  --stop-distance-m 2.5 --max-speed-mps 0.30 --max-runtime-s 15
```

Stop the dead-man loop or delete its file to force zero velocity. The controller
also stops on detection loss, invalid/unstable depth, target-range jumps, stale
heartbeat/GPS/attitude, GPS fix below 3D, mode change, disarm, travel limit,
timeout, stop distance, or any exception.

Do not run live until the ZED, Pixhawk serial path, autopilot type/mode, GPS fix,
camera mounting yaw, propulsion mapping, and a clear restrained test area have
all been verified.
