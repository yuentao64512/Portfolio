# Ng Yuen Tao - Robotics & Autonomous Systems Portfolio

[![Portfolio](https://img.shields.io/badge/Portfolio-Live%20Site-5bd0ff)](https://yuentao64512.github.io/Portfolio)
[![Focus](https://img.shields.io/badge/Focus-Autonomous%20Systems-blue)](#featured-projects)
[![Robotics](https://img.shields.io/badge/Robotics-ROS%20%7C%20ROS%202-orange)](#technical-focus)

Welcome to my robotics and autonomous systems portfolio. I am a Robotics Systems undergraduate focused on UAV autonomy, mobile robots, computer vision, embedded AI, simulation validation, and hardware-software integration.

This repository hosts my portfolio website and selected project artifacts.

## Live Site

View the portfolio here:

[https://yuentao64512.github.io/Portfolio](https://yuentao64512.github.io/Portfolio)

## Featured Projects

### RobotX - UAV Autonomy & Multi-Robot Systems

**Status:** Ongoing<br>
**Role:** Lead - Unmanned Aerial Vehicle (UAV)

Developing UAV autonomy for a multi-domain robotics platform involving aerial, marine, and underwater systems. The work includes Gazebo simulation, ROS 2/MAVROS flight control, ArduPilot/MAVLink integration, YOLOv11 perception on Jetson Orin, and communication pipelines between UAV, ASV, and ground station.

### Autonomous 3-Axis Robotic Crane System

**Role:** Robotics Software / Autonomy Developer<br>
**Stack:** ROS 2, YOLOv11, Jetson Orin Nano, G-code

Designed a vision-guided robotic crane workflow for logistics side pallet-loading. The system connects perception, AI inference, motion planning, and robotic control into an autonomous pick-and-place pipeline.

### SAFMC - Drone Swarm Flocking

**Status:** Completed<br>
**Role:** Autonomous Drone Programmer

Built and tested autonomous drone flight functions using Python, ROS 2, MAVROS, and MAVLink. Integrated UWB indoor positioning with the flight controller to support non-GPS navigation experiments.

### SEP1 - Limo Autonomous Robot

**Stack:** ROS, LIDAR, SLAM, computer vision

Integrated ROS-based navigation, SLAM, and computer vision workflows on the AgileX Limo platform, including mapping, waypoint navigation, and object-detection experiments.

### FWISH - Water Takeoff Flying Craft

Prototype concept for a craft designed to take off from water and transition into flight. Current work includes scaled and full-size prototype models for design validation and presentation.

## Technical Focus

- **Autonomy:** ROS 2, MAVROS, MAVLink, ArduPilot, waypoint navigation, multi-robot communication
- **Robotics:** ROS, SLAM, Gazebo, LIDAR workflows, robotic control, system integration
- **AI & Perception:** YOLOv11, computer vision, edge AI deployment, perception-to-action pipelines
- **Embedded & Hardware:** Jetson Orin, Jetson Orin Nano, flight-controller integration, sensor integration
- **Validation:** simulation testing, field troubleshooting, debugging, technical documentation

## Repository Structure

```txt
Portfolio/
+-- index.html          # Main portfolio website
+-- styles.css          # Legacy/simple stylesheet
+-- images/             # Portfolio project images
+-- darknet_ros/        # ROS object-detection experiments
+-- gazebo/             # Gazebo simulation files
+-- launch_files/       # ROS launch configurations
+-- python_files/       # Python scripts and experiments
+-- turtlesim/          # ROS fundamentals and examples
```

## Run Locally

This is a static site, so it can be served with any simple web server.

```bash
python -m http.server 4177
```

Then open:

```txt
http://127.0.0.1:4177
```

## Contact

- **Portfolio:** [yuentao64512.github.io/Portfolio](https://yuentao64512.github.io/Portfolio)
- **LinkedIn:** [linkedin.com/in/yuen-tao-rse](https://www.linkedin.com/in/yuen-tao-rse/)
- **GitHub:** [github.com/yuentao64512](https://github.com/yuentao64512)
- **Email:** [ng.yuentao21@gmail.com](mailto:ng.yuentao21@gmail.com)
