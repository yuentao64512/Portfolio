# UAV and USV Gazebo simulation

This repository contains a shared Gazebo waterfront simulation for an Iris UAV
and two USV configurations: a four-thruster OmniPlus boat with lateral crab
control, and a conventional two-thruster boat. Both boat variants use the same
map and gentle wave settings.

## Set up

```bash
git clone --recurse-submodules https://github.com/OPTIPRIME-SIT/UAV-sim.git
cd UAV-sim
./scripts/setup_simulation.sh
```

Run the standalone boat simulation with `./run_usv_sim.sh four` or
`./run_usv_sim.sh two`. Run the passive buoyancy check with
`./run_usv_float.sh four`.

The combined UAV/USV launcher additionally expects sibling ArduPilot and
ArduPilot Gazebo checkouts (or the `ARDUPILOT_DIR` and
`ARDUPILOT_GAZEBO_DIR` environment variables):

```bash
./run_boat_drone_gazebo.sh
```

See [USV_GAZEBO.md](USV_GAZEBO.md) for control, dock, beacon, Pixhawk, and
safety details.
