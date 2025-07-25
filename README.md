# AgileX TurtleBot ROS1 Melodic Portfolio

Hi, I’m **Ng Yuen tao** from **SIT Robotics Systems Engineering**. In this portfolio, I showcase what I have learned about SEP 1: Limo robot, Turtlesim, TurtleBot, Gazebo simulations, and Darknet\_ROS integration on the Limo platform.

## 📂 Quick Overview

* **Turtlesim Tutorials**: Learn ROS fundamentals (publishers, subscribers, services, parameters) using the Turtlesim simulator. [Jump »](#turtlesim-tutorials)
* **TurtleBot Applications**: Explore SLAM mapping and autonomous navigation demos on TurtleBot3 Waffle & Waffle Pi. [Jump »](#turtlebot-applications)
* **AgileX Integration**: Conduct software experiments on the AgileX Limo platform, focusing on simulation configurations. [Jump »](#agilex-integration)
* **Darknet\_ROS (YOLO Object Detection)**: Integrate real-time YOLOv3 and YOLOv4 object detection into ROS workflows. [Jump »](#darknet_ros-yolo-object-detection)

## 📚 Topics Covered

### AgileX Integration

* Leveraged the robot’s pre-installed ROS packages for the AgileX TurtleBot Limo and focused on simulation configurations and software experiments.

### Darknet\_ROS (YOLO Object Detection)

* Integrated `darknet_ros` package for real-time object detection (YOLOv3/YOLOv4).
* Tuned detection parameters and visualized bounding boxes in RViz.
* Logged detection results to ROS topics for downstream processing.

### Turtlesim Tutorials

* Created step-by-step Turtlesim demos to explore ROS topics, services, parameters, and node lifecycles.
* Illustrated `roscpp` and `rospy` examples for publishing, subscribing, and service calls.

### TurtleBot Applications

* Developed navigation demos using TurtleBot3 Waffle and Waffle Pi configurations.
* Explored SLAM (`slam_gmapping`) and autonomous patrol routines with `move_base` and costmap tuning.

### Gazebo Simulations

* Designed custom Gazebo 9 worlds with AgileX TurtleBot URDFs and environment models.
* Automated world spawning and camera sensor plugins for perception testing.
* Visualized simulated sensor data streams (LIDAR, depth) in RViz.

### Waffle Platform

* Examined differences between TurtleBot3 Waffle and Waffle Pi.
* Benchmarked performance for navigation and mapping tasks under constrained resources.

### ARM Cross-Compilation

* Configured cross-compile toolchain for AgileX’s ARM-based onboard computer.
* Wrote CMake toolchain files and tested builds on hardware via SSH.

## 🔍 Repository Structure

```
.devcontainer            - Container configuration files
src/                     - ROS workspace with packages
├── agilex_bringup        - Drivers and bringup for AgileX platforms
├── darknet_ros_demo      - YOLO detection integration and launch files
├── turtlesim_tutorials   - Interactive Turtlesim examples
├── turtlebot_apps        - SLAM & navigation demos for TurtleBot3 Waffle
├── gazebo_worlds         - Custom Gazebo worlds and sensor plugins
├── waffle_benchmarks     - Performance scripts and logs for Waffle series
└── cross_compile_tooling - ARM toolchain setup
```

## 🎯 Highlights

* Demo launch files under `src/*/launch` for quick exploration.
* Code examples in each folder demonstrate how to run the demos and interpret the results.
* Parameter files for SLAM, detection, and costmaps in `src/turtlebot_apps/config`.

## 🚶‍♂️ How to Explore

* **Browse** each package to see code structure and launch examples.
* **Inspect** the `.devcontainer` folder for environment setup details.
* **Review** code examples in each folder to see how to run the demos and interpret the results.
* **Compare** Waffle vs. Waffle Pi performance in `waffle_benchmarks`.

---

> This portfolio documents my learning process and key insights across AgileX, object detection, simulation, and navigation—focused on experimentation rather than turnkey usage.
