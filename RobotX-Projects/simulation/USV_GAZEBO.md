# USV buoyancy and propulsion checks

This directory contains two independent Gazebo Sim Harmonic simulations using
the same MBS waterfront map:

| Launch | Model | Propulsion |
| --- | --- | --- |
| `./run_usv_sim.sh four` | `usv_four_thruster` | port + starboard stern surge/reverse/yaw and bow + stern lateral crab |
| `./run_usv_sim.sh two` | `usv_two_thruster` | port + starboard stern surge/reverse/yaw only |

Both worlds are copies of the same waterfront map, with only the included boat
changed. The wavefield and the local `gz-waves1-hydrodynamics-system` plugin are
loaded in each world.

The current wave setting is intentionally gentle (`wind_speed=1.0 m/s`,
`steepness=0.1`) to minimize surface disturbance during buoyancy and motion
checks.

The animated `WavesVisual` renderer is intentionally disabled in these two
launch worlds. The wavefield remains active for buoyancy and hydrodynamics,
while the water surface is rendered as a static mesh; this avoids an OGRE2
material-cleanup assertion that otherwise aborts Gazebo GUI during shutdown.

## Run

After cloning, initialize and build the wave and beacon plugins:

```bash
git clone --recurse-submodules https://github.com/OPTIPRIME-SIT/UAV-sim.git
cd UAV-sim
./scripts/setup_simulation.sh
```

Then check the local dependencies:

```bash
./scripts/check_dependencies.sh
```

Run a GUI simulation with an interactive controller:

```bash
./run_usv_sim.sh four
./run_usv_sim.sh two
```

Run the unattended motion sequence instead:

```bash
./run_usv_sim.sh four demo
./run_usv_sim.sh two demo
```

For a passive float-only test, start Gazebo without the controller or any
thruster commands:

```bash
./run_usv_float.sh four
./run_usv_float.sh two
```

Use `./run_usv_float.sh four --server-only` if you want to inspect the boat
with `gz topic` without opening the GUI.

To move the four-thruster boat left while holding the propulsion pair at zero,
run:

```bash
./run_usv_left.sh
```

The default is 8 seconds at lateral thrust magnitude 7. The bow and stern
lateral thrusters receive equal command signs because their SDF orientations
match; the port/starboard propulsion pair remains at zero. A small default rear
trim of `1.15` compensates the measured force mismatch in this model. The
script leaves Gazebo open after the boat stops. Customize the duration, thrust,
and trim, for example `./run_usv_left.sh 5 4 1.15`.

For Pixhawk state-machine data, use the receive-only logger. It records mode,
arming, attitude, position, status, RC, servo outputs, and status text as JSONL:

```bash
python3 scripts/log_fcu_state.py --duration 120 --output fcu_state.jsonl
```

For the crab run's state transitions, convert its JSONL log to Mermaid:

```bash
python3 scripts/fcu_state_to_mermaid.py fcu_crab_state.jsonl \
  --output fcu_crab_state.mmd
```

The controller flow and safety guards are also provided in
`fcu_crab_state_machine.mmd`.

For Pixhawk hardware-in-the-loop Gazebo testing with the native OmniPlus
mixer, start the world first and then run the OmniPlus bridge. The bridge
routes Pixhawk `SERVO_OUTPUT_RAW` feedback into the four Gazebo thruster
topics:

```bash
./run_mavlink_gazebo.sh
./run_omniplus_gazebo_move.sh 5 -0.20 1.4
```

The second command requests MANUAL mode, arms for the simulation test, sends
native RC steering/throttle/lateral input, and stops/disarms at the end. The
negative lateral value is left for this SDF. The current FCU uses
`MAV_GCS_SYSID=255`, so the bridge defaults to MAVLink source system 255; use
`--source-system` if that FCU parameter is changed. Its log is
`mavlink_omniplus_gazebo.jsonl`.

For this ArduRover OmniPlus setup, the native lateral input is the Rover yaw
channel (`RCMAP_YAW`, currently RC4); `RCMAP_LATERAL` is not used by the
manual OmniPlus mixer. The bridge also holds `MODE_CH` at the configured
MANUAL slot so an old HOLD override cannot suppress the motor outputs. The
current Pixhawk bench profile has `SCR_ENABLE=0`, `FS_THR_ENABLE=0`, and
`FS_ACTION=0` because there is no physical RC/kill input in this Gazebo-only
test. Re-enable the safety Lua script and normal failsafes before connecting
propulsion.

Native Guided crab control is available in the locally built ArduPilot Rover
SITL binary after applying the small Guided extension in the local ArduPilot
checkout (a sibling `../ardupilot` directory by default).
It accepts a standard `SET_POSITION_TARGET_LOCAL_NED` message in `BODY_NED`:
`vx` is forward speed, `vy` is right speed, and ignored yaw holds the current
heading. The bridge exposes that path with:

```bash
bash run_omniplus_gazebo_guided.sh 5 0.0 -0.30 1.4 \
  --log mavlink_guided_omniplus_gazebo.jsonl
```

Negative `--lateral-speed` is left for this SDF. This command must only be used
with the matching patched firmware; the currently connected Pixhawk firmware is
stock ArduRover 4.7.0, while the local SITL build is based on a 4.6 development
checkout. Do not flash the local binary to the Pixhawk. Build and test it in
SITL first, then port the patch to the exact firmware tag before a hardware
upgrade. Keep props disconnected during all bench tests.

## Boat plus separate drone

The local `iris_with_gimbal` ArduPilot Gazebo model can be inserted into the
same waterfront scene as the boat. The combined launcher starts the four-
thruster boat, places the drone on a physical landing pad at `x=0, y=76`, and
creates three boat docking stations on the water side of the seawall at `y=45`:

| Dock | Position | Beacon color |
| --- | --- | --- |
| `boat_dock_red` | `x=-14, y=45` | red |
| `boat_dock_green` | `x=0, y=45` | green |
| `boat_dock_blue` | `x=14, y=45` | blue |

Each dock has an open centre bay sized for the USV to enter from the water,
side-pier collision boundaries, a rear stop, and colored beacon lights. The
dock surfaces remain neutral brown; the colors identify the beacons only.
Behind the docking area is a neutral collision wall at `x=0, y=50`, spanning
`38 m` across the red-to-blue docks and standing `10 m` high. The wall is
landward of the dock rear stops and does not close the water-side entrances.
The launcher also starts a separate ArduCopter SITL endpoint on TCP
`127.0.0.1:5760`:

```bash
bash run_boat_drone_gazebo.sh
```

Use another terminal to control only the drone:

```bash
bash run_drone_guided.sh takeoff 5
bash run_drone_guided.sh move 1.0 0.0 0.0 5
bash run_drone_guided.sh hold 3
bash run_drone_guided.sh land
```

The `move` arguments are `forward_mps right_mps down_mps duration_s`. The
boat remains on its separate Pixhawk connection and can be controlled with its
OmniPlus bridge at the same time. Stop any existing Gazebo instance before
starting the combined launcher.

The guarded MAVLink crab script is dry-run by default and requires the bow and
stern output channels to be supplied after checking `SERVOx_FUNCTION`:

```bash
python3 scripts/mavlink_crab.py --front-channel N --rear-channel M
python3 scripts/mavlink_crab.py --front-channel N --rear-channel M --send
```

It uses a rear trim of `1.4` by default, logs command and telemetry state to
`fcu_crab_state.jsonl`, never arms the FCU, and refuses selected outputs whose
`SERVOx_FUNCTION` is not `0`. Test with propellers disconnected and the hull
secured; the Pixhawk must be connected before either script can receive a
heartbeat.

To find the two output numbers before using the crab sender:

```bash
python3 scripts/read_fcu_servo_map.py
```

The controller accepts `surge sway yaw` values. For example, in the four
thruster case, `8 0 0` drives forward, `-8 0 0` reverses, and `0 7 0` crab
moves sideways. The two-thruster case ignores sway because its lateral
thrusters are intentionally absent.

For a server-only run, useful for CI or inspection with `gz topic`, add the
third argument `--server-only`:

```bash
./run_usv_sim.sh four interactive --server-only
```

## Buoyancy basis

The hull collision mesh is watertight and encloses approximately `0.0682 m^3`.
The four-thruster boat is approximately `30.9 kg`, so its full-weight
displacement in seawater is `30.9 / 1025 = 0.0301 m^3`, below the available hull
volume. The two-thruster variant is lighter and also has positive reserve
buoyancy. The hydrodynamics plugin uses the collision mesh directly for
submerged-triangle buoyancy; it does not use the old `volume=1.5` comment.

The model is started at `z=0.12 m` to give it time to settle onto the wavefield.
The `z` value reported by odometry should remain finite and above the bottom;
the demo also reports final `x`, `y`, and `z` so that floating and motion can be
checked without a GUI.

The standalone variants intentionally omit `ArduPilotPlugin`, so no ArduPilot
SITL or `ardupilot_gazebo` build is required for these checks. The setup script
builds `libgz-waves1-hydrodynamics-system.so`, its wavefield dependencies, and
the navigation beacon plugin locally.
