# Gazebo Simulation with ROS Navigation

## Overview

This repository sets up a Gazebo simulation integrated with the ROS Melodic navigation stack, enabling mobile robot path planning, localization, and control in a simulated environment.

## Prerequisites

1. **Ubuntu 18.04 (Bionic)**

2. **ROS Melodic Morenia**

   * Configure package sources:

     ```bash
     sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
     ```
   * Set up keys and update:

     ```bash
     sudo apt install curl
     curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
     sudo apt update
     ```
   * Install ROS Melodic Desktop-Full:

     ```bash
     sudo apt install ros-melodic-desktop-full
     ```
   * Initialize `rosdep`:

     ```bash
     sudo apt install python-rosdep
     sudo rosdep init
     rosdep update
     ```

   ([wiki.ros.org](https://wiki.ros.org/melodic/Installation/Ubuntu?utm_source=chatgpt.com))

3. **Catkin Workspace**

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

4. **Install Navigation Packages**

   ```bash
   sudo apt install ros-melodic-navigation ros-melodic-map-server ros-melodic-amcl
   ```

   * `navigation` stack provides 2D path planning and control ([wiki.ros.org](https://wiki.ros.org/navigation))

## Repository Structure

```
gazebo-sim-nav/
├── worlds/
│   └── office.world           # Custom Gazebo world for navigation
├── models/
│   └── my_robot/              # URDF, meshes, textures
├── maps/
│   └── office_map.yaml        # 2D occupancy grid map
├── launch/
│   ├── gazebo.launch          # Launch Gazebo with `office.world`
│   ├── spawn_robot.launch     # Spawn robot model
│   ├── nav.launch             # Start `map_server`, `amcl`, `move_base`
│   └── full_sim.launch        # Combines all above components
├── config/
│   ├── costmap_common.yaml    # Global costmap parameters
│   ├── costmap_local.yaml     # Local costmap parameters
│   ├── base_local_planner.yaml# Local planner parameters
│   └── global_planner.yaml    # Global planner parameters
├── scripts/
│   └── spawn.py               # Utility to spawn models via ROS service
├── README.md                  # This file
└── LICENSE
```

## Usage

### 1. Launch Gazebo

```bash
roslaunch gazebo-sim-nav gazebo.launch world:=office.world
```

### 2. Spawn Robot

```bash
roslaunch gazebo-sim-nav spawn_robot.launch model:=my_robot
```

### 3. Run Navigation Stack

```bash
roslaunch gazebo-sim-nav nav.launch map:=maps/office_map.yaml
```

This will start:

* `map_server` (load map)
* `amcl` (adaptive Monte Carlo localization)
* `move_base` (path planning & control)

See [Navigation Wiki Tutorials](https://wiki.ros.org/navigation/Tutorials) for detailed examples ([wiki.ros.org](https://wiki.ros.org/navigation))

### 4. Full Simulation

```bash
roslaunch gazebo-sim-nav full_sim.launch
```

Combines world, robot spawn, and navigation in one command.

## Configuration

Adjust parameters in `config/` according to your robot's footprint, sensor ranges, and environment. Key files:

* **costmap\_common.yaml**: Robot footprint, inflation radius
* **costmap\_local.yaml**: Local costmap resolution
* **base\_local\_planner.yaml**: `DWAPlannerROS` settings
* **global\_planner.yaml**: `NavfnROS` or `GlobalPlanner` settings

For more parameter explanations, refer to the [navigation wiki](https://wiki.ros.org/navigation) ([wiki.ros.org](https://wiki.ros.org/navigation))

## References

* ROS Melodic Installation: [http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu) ([wiki.ros.org](https://wiki.ros.org/melodic/Installation/Ubuntu?utm_source=chatgpt.com))
* Navigation Stack Overview: [https://wiki.ros.org/navigation](https://wiki.ros.org/navigation) ([wiki.ros.org](https://wiki.ros.org/navigation))

