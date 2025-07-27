A custom ROS1 package of Python scripts developed for my robot to handle slope detection, waypoint recording, and interactive waypoint navigation. These scripts were created specifically to integrate with my robot's onboard sensors and navigation stack.

## Features

* **Slope Detection** via `slope_detector.py` (integrated into the launch file)
* **Waypoint Recording** with `record_waypoint.py`
* **Interactive Waypoint Navigation** with `waypoint.py`
* Simple, script-based workflow in the `scripts/` directory

## Prerequisites

* ROS1 Melodic or Noetic
* OpenCV and `cv_bridge`
* A catkin workspace (assumed: `~/catkin_ws`)

## Installation

1. **Install dependencies**:

   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```
2. **Build and source**:

   ```bash
   catkin_make
   source devel/setup.bash
   ```

## Directory Layout

```
slope_waypoint_tools/
├── launch/
│   └── limo_navigation_rtabmap.launch   # Launches navigation and slope detector
├── scripts/
│   ├── slope_detector.py    # Used by the launch file
│   ├── record_waypoint.py   # Record GPS or pose waypoints
│   └── waypoint.py          # Send waypoints interactively
└── README.md                # This file
```

## Setting Up Scripts

All helper scripts live in `scripts/`. Before running:

1. **Switch to scripts directory**:

   ```bash
   cd ~/agilex_ws/src/limo_ros/limo_bringup/scripts
   ```
2. **Make all Python files executable**:

   ```bash
   chmod +x *.py
   ```

## Running the Package

### 1. Launch Slope Detection

This starts the `slope_detector.py` node as defined in the launch file:

```bash
roslaunch limo_bringup limo_navigation_rtabmap.launch
```

### 2. Record Waypoints

In a new terminal (after sourcing your workspace):

```bash
rosrun limo_bringup record_waypoint.py
```

Follow console prompts or inputs to save waypoints to file.

### 3. Interactive Waypoint Navigation

In another terminal:

```bash
rosrun limo_bringup waypoint.py
```

Enter waypoint indices or filenames as prompted to send goals to the robot.

## Launch File Details

  * roslaunch all the files in limo that is needed such as dabai_u3 camera, limo_start, limo_rtabmap_orbbec, limo_navigation_rtabmap, rviz.
  * Runs `scripts/record_waypoint.py`
  * to remap set localisation to false for orbbec. Once done set localisation back to true. map will be auto saved into rtabmap.db
  * 
## Troubleshooting

* **Permission errors**: Ensure `chmod +x *.py` was run in `scripts/`.
* **Module not found**: Check `cv_bridge` is installed and sourced with your workspace.
* **ROS node won’t start**: Verify the launch file name and package name match.

## Contributing

1. Fork the repo
2. Create a feature branch
3. Add or update scripts/tests
4. Submit a pull request

