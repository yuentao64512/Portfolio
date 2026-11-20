# Camera Capture GUI

Tools for a USB camera on the Orin NX — a terminal status tool and a
graphical viewer with photo/video capture, both built on the same
`camera_link.py` backend.

This is also used as a library by the drone heartbeat GUI
(`../drone-heartbeat-gui/heartbeat_monitor.py`), which imports
`camera_link.py` directly to add a `camera`/`photo` command there. Keep this
folder as a sibling of `drone-heartbeat-gui/` (both directly under the same
parent directory) so that import keeps working.

## Files

- `camera_link.py` — the camera side: opens a USB camera with OpenCV on a
  background thread, tracks live status, saves JPEG snapshots, and records
  MP4 video. No GUI code in here at all.
- `camera_monitor.py` — **terminal** GUI (Textual): live status text
  (resolution, FPS, frame count) and a `photo` command. No video preview —
  not practical over a text terminal/SSH session. Run this over plain SSH.
- `camera_view.py` — **graphical** GUI (Tkinter): an actual live video
  preview window with "Take Photo" and "Start/Stop Recording" buttons. This
  needs a real display to show up on — a monitor plugged into the Orin, or
  a remote desktop/VNC session (e.g. over Tailscale). It will not work over
  a plain SSH-only connection.
- `altitude_bank_calibration.py` — standalone **offline analysis** script
  (not a GUI, no camera device needed at run time): calibrates the camera's
  lens from checkerboard photos, then estimates how high you can fly and how
  hard you can bank before a buoy becomes undetectable. See below.
- `attitude_camera_calibration.py` — calibrates the **down-facing** camera
  together with the FCU's attitude/altitude: lens, mounting offset, real
  ground distances from pixels, and a FOV footprint document. See below.

## One-time setup on the Orin NX

```bash
pip3 install -r requirements.txt
```

If `python3-tk` isn't already installed (needed for `camera_view.py`):
```bash
sudo apt install python3-tk
```

## Running

**Terminal status tool** (works over plain SSH):
```bash
python3 camera_monitor.py --device 0 --output-dir captures
```

**Graphical viewer** (needs a display — monitor or remote desktop):
```bash
python3 camera_view.py --device 0 --output-dir captures
```

Both take the same options:
- `--device` — an index (e.g. `0` for `/dev/video0`) or a device path
  (e.g. `/dev/video2`). Default `0`.
- `--output-dir` — where photos/videos get saved (default: a `captures`
  folder next to the script).
- `--rotate` — rotate every frame this many degrees clockwise, to correct a
  physically mis-mounted camera (e.g. `--rotate 155` if it's mounted at a
  155° angle). Default `0` (no rotation). Applies to the live preview, saved
  photos, and recorded video alike — rotated once in `camera_link.py`, not
  redone per-display. Any angle works, not just 90/180/270; the frame canvas
  is expanded so nothing gets cropped off the corners (they fill with black
  instead).

## `camera_monitor.py` commands

| Command | Effect |
|---|---|
| `help` | show the command list |
| `photo` | save a photo, auto-named by timestamp (`photo_20260101_120000.jpg`) |
| `photo <name>` | save a photo under a specific filename |
| `reconnect` | drop and reopen the camera device |
| `quit` | exit |

## `camera_view.py` window

A live preview fills the window, with the current resolution/FPS/frame
count and the last action's result shown in the status bar underneath.

- **Take Photo** — saves the current frame as a timestamped JPEG.
- **Start Recording** / **Stop Recording** — toggles recording the live
  feed to a timestamped MP4. The status bar shows `● RECORDING` while
  active; closing the window while recording stops and finalizes the file
  cleanly rather than leaving it corrupted.

## `altitude_bank_calibration.py` — lens calibration + altitude/bank limits

An offline analysis script — run it on a folder of photos, not against a
live camera. It does three things:

1. **Calibrates the lens** from checkerboard photos (standard OpenCV
   `cv2.calibrateCamera()`), giving focal length (pixels), the optical
   center, and distortion coefficients.
2. **Cross-checks** that calibration using any photos whose filename encodes
   a known distance to the board (e.g. `board_50cm.jpg`, `shot_1.0m.jpg`),
   via the simple pinhole formula — a sanity check, not a replacement for #1.
3. **Estimates an altitude limit and a bank-angle limit** for detecting a
   buoy, from the calibrated focal length plus parameters you supply.

### Capturing calibration photos

Print a checkerboard pattern (e.g. a standard 10×7-square, 9×6-internal-corner
board — [generate one here](https://markhedleyjones.com/projects/calibration-checkerboard-collection)
if you don't have one) and measure one square's real size in mm.

Take **10-20 photos**, varying **both angle and distance** — straight-on
shots at a few distances alone is a known-degenerate case for calibration
(it can't separate focal length from distance if every photo is frontal).
Tilt the board differently in each shot; let it fill different parts of the
frame. If you want the distance cross-check (step 2 above), include the
distance in a few filenames, e.g. `board_50cm.jpg`, `board_1.0m.jpg`,
`board_2.0m.jpg` — those don't need to be a special subset, just regular
calibration photos with a labeled filename.

### Running it

```bash
python3 altitude_bank_calibration.py \
    --calibration-dir photos/ \
    --checkerboard-size 9x6 \
    --square-size-mm 25 \
    --target-diameter-m 0.61 \
    --min-detect-px 20 \
    --camera-depression-deg 15
```

| Option | Meaning |
|---|---|
| `--calibration-dir` | folder of checkerboard photos (required) |
| `--checkerboard-size` | internal corner count, `colsxrows` (default `9x6`) |
| `--square-size-mm` | real size of one checkerboard square — measure your printed board (required) |
| `--target-diameter-m` | buoy diameter in meters (default `0.61` — derived from the official RobotX RoboBuoy BOM: two 38in pool noodles forming the float ring, circumference/π; not an official published spec, override if you have a real measurement) |
| `--min-detect-px` | minimum pixel size your detector needs to find the buoy reliably (required — no sensible default exists, this is entirely about your detection method) |
| `--camera-depression-deg` | angle between the camera's boresight and horizontal, measured downward: `0` = aimed at the horizon, `90` = straight down (required — get this right from your actual mount or the altitude/bank numbers are meaningless) |
| `--save-calibration` | optional path to save the calibration (matrix, distortion, FOV) as JSON for reuse |
| `--safety-margin-pct` | how far below the hard altitude limit to recommend actually flying (default `30`, i.e. recommend 70% of the limit) — margin for detector reliability near the exact threshold, autopilot altitude-hold tolerance, and real-flight motion blur/vibration. `0` to just see the hard limit |

### Reading the output

- **Reprojection error** (pixels) is the standard calibration quality
  metric — under ~0.5px is good, over ~1px means recapture with more varied
  angles/distances.
- **Altitude limit** is a straight-line distance to the target, from
  `apparent_px = focal_length_px × real_size_m / distance_m` — not
  necessarily pure vertical altitude if the camera is angled. This is the
  hard edge where detection fails entirely, not a flying height.
- **Recommended operating altitude** is the number to actually fly at — the
  hard limit minus `--safety-margin-pct`. Flying right at the hard limit
  means the target is only *just* barely at your detector's minimum size;
  this backs off so there's margin for the things an idealized calculation
  doesn't capture (detector reliability right at the threshold, the
  autopilot's own altitude-hold tolerance, motion blur/vibration in real
  flight).
- **Bank-angle limit** comes with its modeling assumptions printed
  alongside it (target starts dead-center, pure roll with pitch/heading
  held fixed, camera on the aircraft centerline, flat water). It's a
  simplified engineering estimate, not a certified flight envelope — a real
  coordinated turn, an off-center target, or mount flex will all change the
  real number. A result over 90° means roll alone essentially can't push the
  target out of frame at that mount angle — something else would be the
  actual constraint.

## `flight_recorder.py` — video + matching flight data

Started from the drone GUI with `record` (`record stop` to finish), or on
its own: `python3 flight_recorder.py --device 0` (Ctrl+C to finish). Writes,
with one name stem:

| File | Contents |
|---|---|
| `flight_<date>_<time>.mp4` | the video |
| `flight_<date>_<time>_frames.csv` | **one row per video frame**: frame number, its time in the video, and the FCU's latest reading of every degree of freedom - x/y/z (north/east/down, m) and velocities, roll/pitch/yaw (deg) and their rates (deg/s), heading (deg, 0-360), lat/lon, altitude (MSL and above home), groundspeed, climb, throttle, accelerations, mode, armed |
| `flight_<date>_<time>_fcu_raw.jsonl` | every FCU message used, exactly as received, at full rate |

True FCU readings only: values come straight from the FCU's ATTITUDE,
LOCAL_POSITION_NED, GLOBAL_POSITION_INT, VFR_HUD and SCALED_IMU messages
(only unit conversions applied), never interpolated. Each group carries the
FCU's own timestamp (`*_fcu_ms`, ms since FCU boot) and how old it was at
that frame (`*_age_ms`); anything the FCU didn't send stays blank. x/y/z
need the EKF to have a position (GPS lock) - indoors they stay blank.

The video runs on the real clock: frames go into the file at a fixed rate
(the camera's, normally 30 fps), repeating a frame when the camera is slower
and skipping one when it's faster - so 1 s of video is 1 s of flight, and
`repeat=1` in the CSV marks a repeated frame (`camera_frame` says which real
camera frame it is).

FCU data comes through the drone GUI's heartbeat code on its own MAVProxy
output, `udp:127.0.0.1:14553` (the GUI's auto-started MAVProxy has it), so
recording works during a lawnmower mission. While recording it raises the
FCU's attitude/position stream rates (30/15/10 Hz), and puts the GUI's
rates back afterwards. The camera can only be used by one program: `camera`
in the GUI and `record` refuse to run at the same time.

## `attitude_camera_calibration.py` — down-facing camera + FCU attitude

A separate script (nothing else uses it) that calibrates the down-facing
camera together with the FCU's attitude and altitude, works out the camera's
**mounting offset**, turns pixels into **real ground distances**, and writes a
document of the **ground footprint** against altitude and attitude.

Boards (squares counted): **A** 10 × 7 squares of 4.2 × 4.2 cm, **B** 7 × 5
squares of 6.0 × 5.6 cm (`lens` checks which side of B is the 6.0 cm one).
Other boards: `--board-def NAME=COLSxROWS:WxH` (cm).

| Step | Command | What it gives |
|---|---|---|
| 1. Lens photos | `python3 attitude_camera_calibration.py capture --out calibration/sessions/lens1 --board A --no-fcu --auto 1` | 20-30 photos, board held at many angles/distances, covering the corners of the picture too |
| 2. Lens | `python3 attitude_camera_calibration.py lens calibration/sessions/lens1` | focal length, centre, distortion, FOV → `calibration/camera_calibration.json` |
| 3. Ground photos | `python3 attitude_camera_calibration.py capture --out calibration/sessions/ground1 --board B --ground --auto 1` | board **flat on the ground**; carry or hover the drone 1-4 m above it, rolling and pitching both ways up to ~20° and turning; it only saves when steady (< 5°/s) |
| 4. Mounting + altitude | `python3 attitude_camera_calibration.py attitude calibration/sessions/ground1` | **mounting offset** (lens tilt toward nose / right wing, picture rotation, ± 95%), FCU altitude vs real camera height (baro and rangefinder), size vs altitude/attitude, distance errors → saved into the calibration |
| 5. Distances | `python3 attitude_camera_calibration.py measure --capture calibration/sessions/ground1/img_0003.png --point 120,200 --point 400,260` | each pixel's spot on the ground (forward/right of the drone, north/east) and the real distance between them; or `--roll --pitch --yaw --alt` instead of `--capture` |
| 6. Document | `python3 attitude_camera_calibration.py fov-report [--frames-csv recordings/flight_..._frames.csv]` | `calibration/FOV_footprint_log.md` (+ `.csv`): footprint, reach and GSD vs height and lean angle, the photos measured, a summary of a recorded flight, and a field-test log to fill in |

Photos can also come from a `record` flight: `from-recording recordings/flight_<...>.mp4 --out calibration/sessions/ground2 --board B --ground`.

Notes:
- `capture` reads the FCU through the drone GUI's spare MAVProxy output
  `udp:127.0.0.1:14552` - don't run the lawnmower mission or `param_set.py` at
  the same time. It opens the camera itself, so close `camera`/`record` in the
  GUI first. Use the same `--rotate` as the GUI's `--camera-rotate`.
- Do the FCU's accelerometer (level) calibration first - the mounting offset
  is measured against the FCU's own level.
- A downward rangefinder (DISTANCE_SENSOR / RANGEFINDER) is used automatically
  when present and is far better than the barometer at low height.
- Boards are only detectable up to a few metres; the footprint tables at
  greater heights come from the calibrated geometry, so check a couple of them
  in the field (the document's field-test log).
- Other scripts can use the projection directly:
  `Projector.from_calib(load_calib()).ground(pixels, roll, pitch, yaw, height)`.

## Troubleshooting

- **"NOT OPEN" / error "could not open camera device"**: check the device
  exists — `ls /dev/video*` on the Orin NX — and that nothing else
  (MAVProxy, another script, a stale process, the *other* camera tool here)
  already has it open; a USB camera can only be opened by one process at a
  time.
- **Opens but FPS stays 0 / no frames**: some UVC cameras need a specific
  resolution/format OpenCV doesn't default to well. Try a different
  `--device` index first to rule out picking the wrong `/dev/videoN` (many
  UVC cameras expose more than one node, e.g. `/dev/video0` and
  `/dev/video1` for the same physical camera).
- **`camera_view.py` window never appears / errors about no display**:
  there's no display server reachable from wherever this is running — plug
  a monitor into the Orin, or connect via your remote desktop/VNC session
  (not a plain SSH terminal) before launching it.
- **`ModuleNotFoundError: No module named 'tkinter'`**: install it with
  `sudo apt install python3-tk` (it's a system package, not a pip one).
