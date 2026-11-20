# RoboCommand Task-1 simulation (5060)

This package is installed at `/home/robotx/usv_sim_workspace/robocommand_sim` on `optiprime-5060`.
It uses Gazebo Harmonic Transport (`gz.transport13` / `gz.msgs10`) and does not require ROS.

## What is implemented

- Core, Advanced and Disruptive declarations with tier/vehicle checks.
- USV1 and UAV1 2 Hz heartbeats; heartbeats older than 3 seconds show `LOST`.
- Automatic approval or `--manual-approve`, `RunStart`, `team/go`, and STOP/HOLD.
- Retained-style 1 Hz course publication from `config/task1_world.yaml`.
- Core and Advanced world variants. Advanced makes coloured bodies neutral, removes side-visible point lights and replaces top beacons with thin glowing discs.
- ArduCopter SITL GUIDED takeoff, 15 m survey, 3 s stop-and-look at each waypoint and LOITER on stop.
- Real Gazebo camera subscription, OpenCV HSV bright-beacon detection, black-blob detection, pixel-to-water projection, simulated GPS conversion, 1.5 m merging, and at least 3 sightings.
- A separate forward Gazebo camera is mounted on the simulation-only USV model. Core, Advanced and Disruptive all run OpenCV colour detection on its rendered frames and publish `/usv/perception/sim_camera`. The console shows `sim_cv_frames` and marker counts. No real ZED or webcam is opened by this package.
- Blinking/solid blue classification from observation timing.
- `uav/perception/buoys` output and OCS relay to `local/usv/uav/perception/buoys`.
- Boat `WAIT_UAV_MAP`, safe-passage route planning, green-to-port gate centres, entry CW and exit CCW route points.
- Disruptive gate-2 swap via Gazebo `set_pose`, route invalidation and UAV re-scan.
- Ground-truth error table and drone pass/fail score.

The simulation-only boat driver follows Advanced and Disruptive routes with Gazebo thruster topics. It never opens a Pixhawk connection. `stop` zeros all four Gazebo thrust commands; `SIM_BOAT_DRIVE=0 ./scripts/run_agents.sh` disables the driver. Core reports simulated camera observations but does not yet plan an own-camera route. The OCS scoring table still lists boat-track fields as pending until its track evaluator is connected.

## Run (four terminals)

First SSH to the 5060:

```bash
ssh robotx@100.115.142.123
cd /home/robotx/usv_sim_workspace/robocommand_sim
```

Terminal 0 - simulation:

```bash
./scripts/run_simulation.sh advanced
```

Over SSH without a graphical display, the launcher automatically uses Gazebo server mode. `SIM_HEADLESS=1` forces this mode even when a display exists. Both simulated camera topics continue rendering in server mode.

Terminal 1 - team console:

```bash
./scripts/run_terminal_1.sh
```

Terminal 2 - mock RoboCommand + OCS:

```bash
./scripts/run_terminal_2.sh
```

Terminal 3 - vehicle agents:

```bash
./scripts/run_agents.sh --show
```

Then type in Terminal 1:

```text
declare advanced
go
```

Wait for `APPROVED` before `go`. For manual approval, start Terminal 2 with `--manual-approve`, type `a` there, then type `go` in Terminal 1. For Disruptive, declare `disruptive`; after the first map, type `d` in Terminal 2.

## Stop

Type `stop` in Terminal 1 first. It publishes HOLD. Then use Ctrl+C in the agent and simulation terminals. Check state with:

```bash
./scripts/status.sh
```

## Verification

```bash
cd /home/robotx/usv_sim_workspace/robocommand_sim
PYTHONPATH=. python3 scripts/check_sim_cameras.py
PYTHONPATH=. python3 scripts/smoke_sim_cv.py
PYTHONPATH=. python3 -m pytest -q tests/test_core.py
PYTHONPATH=. python3 tests/bus_smoke.py
python3 scripts/build_world_variants.py \
  --source ../worlds/usv_four_thruster.world --output-dir worlds
SDF_PATH=../models:/home/robotx/ardupilot_gazebo/models gz sdf -k worlds/task1_core.world
SDF_PATH=../models:/home/robotx/ardupilot_gazebo/models gz sdf -k worlds/task1_advanced.world
```

## Calibration still required before claiming the pass criteria

The camera mount pixel-axis/yaw sign and the simulated ArduPilot local-NED-to-Gazebo axis mapping must be checked from one known waypoint. The package exposes `camera.mount_yaw_offset_deg` in the YAML for that correction. Do not claim `<1 m` or 100% colour accuracy until the live Gazebo ground-truth table reports PASS.

