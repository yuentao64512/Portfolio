# AgileX TurtleBot ROS1 Melodic Portfolio

Welcome to my GitHub portfolio showcasing my journey and learnings while developing ROS1 (Melodic) TurtleBot applications on AgileX platforms. This repository captures experiments with containerized development, simulation, SLAM, navigation, object detection, and ARM cross-compilation.

## 📚 Topics Covered

### AgileX Integration

* Connected AgileX TurtleBot Limo and Scout hardware.
* Configured udev rules and ros\_serial drivers for onboard sensors and actuators.

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


## 🎯 Highlights

* Demo launch files under `src/*/launch` for quick exploration.
* Video recordings of object detection and navigation in `videos/`.
* Parameter files for SLAM, detection, and costmaps in `src/turtlebot_apps/config`.

## 🚶‍♂️ How to Explore

* **Browse** each package to see code structure and launch examples.
* **Inspect** the `.devcontainer` folder for environment setup details.
* **Watch** `videos/` to see live demos of detection and mapping.
* **Compare** Waffle vs. Waffle Pi performance in `waffle_benchmarks`.

---

> This portfolio documents my learning process and key insights across AgileX, object detection, simulation, and navigation—focused.
