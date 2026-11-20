# Ng Yuen Tao - Robotics & Autonomous Systems Portfolio

[![Portfolio](https://img.shields.io/badge/Portfolio-Live%20Site-5bd0ff)](https://yuentao64512.github.io/Portfolio)
[![Focus](https://img.shields.io/badge/Focus-Autonomous%20Systems-blue)](#featured-projects)
[![Robotics](https://img.shields.io/badge/Robotics-ROS%20%7C%20ROS%202-orange)](#technical-focus)

Welcome to my robotics and autonomous systems portfolio. I am a Robotics Systems undergraduate focused on UAV autonomy, mobile robots, computer vision, embedded AI, simulation validation, and hardware-software integration.

This repository hosts my portfolio website and selected project artifacts.

## Live Site

View the portfolio here:

[https://yuentao64512.github.io/Portfolio](https://yuentao64512.github.io/Portfolio)

## Current Project Status

| Project | Status | Current focus |
| --- | --- | --- |
| RobotX actual USV/UAV/OCS stack | Finished | Physical-system source integration across the Orin NX, AGX Orin, OCS, UI and robocommand |
| RobotX Gazebo simulation | Finished | Boat/drone mission rehearsal, MAVLink control and OCS workflow validation |
| SAFMC 2027 high-speed flocking | Ongoing | Developing the next high-speed multi-drone flocking iteration |
| Hospital fall detection | Research prototype | Multi-person bed/floor context, tracking and fall-to-lying alert evaluation |
| Hiker fall/action detection | Research prototype | Pose-sequence training, action classification and recorded-video validation |
| CV training and deployment collection | Maintained | Reusable training, dataset-preparation and deployed RobotX perception source |

## Featured Projects

### RobotX - UAV Autonomy & Multi-Robot Systems

**Status:** Finished<br>
**Role:** Lead - Unmanned Aerial Vehicle (UAV)<br>
**Stack:** Python, YOLOv11, OpenCV, ROS 2, MAVROS, MAVLink, ArduPilot, Jetson Orin, serial radio

Developed the aerial component of a multi-domain robotics platform involving aerial, marine, and underwater systems. The work includes Gazebo simulation, ROS 2/MAVROS flight control, ArduPilot/MAVLink integration, YOLOv11 perception on Jetson Orin, and communication pipelines between the UAV, ASV, and ground station.

The organized [RobotX projects](RobotX-Projects/) now include the [actual physical-system source](RobotX-Projects/robotx-actual/), [Gazebo simulation stack](RobotX-Projects/simulation/), and the earlier aerial perception and control cases:

- Detecting green and orange/red buoys and boats from drone imagery.
- Detecting blue, red, and green course markers from above.
- Reducing false detections using human rejection and hard-negative training.
- Inspecting walls, roofs, panels, and structures for clean or dirty regions.
- Converting camera detections to grid coordinates and transmitting mission data over a serial radio link.

The collection separates [aerial buoy and boat training](RobotX-Projects/01-buoy-boat-training/), [marker training](RobotX-Projects/02-marker-training/), [training rejects](RobotX-Projects/03-training-rejects/), [drone surface inspection](RobotX-Projects/04-clean-dirty-inspection/), and [drone RC control](RobotX-Projects/05-drone-rc-control/) into documented, reusable project folders.

The centralized [CV collection](cv/) contains GitHub-safe training and deployment scripts from the RTX 5060 and the actual RobotX AGX Orin perception stack. Runtime logs, credentials, private recordings, environments and generated model files are intentionally excluded.

### SAFMC 2027 - High-Speed Drone Flocking

**Status:** Ongoing<br>
**Focus:** High-speed multi-drone flocking

Developing the 2027 high-speed flocking iteration, building on the completed three-drone autonomy, ROS 2, MAVROS, MAVLink and UWB positioning foundation. Implementation and validation evidence will be added as the ongoing system matures.

### Drone Fall and Action Detection

**Status:** Research prototype<br>
**Stack:** Python, YOLO11 Pose, OpenCV, scikit-learn, Jetson Orin Nano, TensorRT

Built an aerial safety-monitoring pipeline that extracts human pose sequences, classifies actions with a Random Forest model, tracks multiple people, and raises a configurable alert when a detected fall is followed by the person remaining down. The work combines the GMDCSA24 dataset with custom top-down DJI recordings and includes live-camera, video-demo, training, evaluation, and Jetson deployment workflows.

The current subject-grouped baseline reports 84.1% accuracy, 85.2% falling-down F1, and 89.6% lying-down F1 on 138 held-out sequences. These are development results; broader aerial field validation is still required.

[View the Fall Action Project, evidence, limitations, and setup instructions](Fall-Action-Project/).

### Hospital Fall Detection

**Status:** Research prototype<br>
**Stack:** Python, YOLO11 Pose, OpenCV, scikit-learn, multi-person tracking

Extended the action-recognition work for fixed hospital CCTV scenes with bed/floor context regions, patient-location state, multi-person tracking, stitched synthetic scenarios, prediction evidence and unit-tested transition logic. This is development evidence rather than proof of clinical performance.

[View the hospital source, reports, test media, models and limitations](hospitall-fall-detection/).

### Autonomous 3-Axis Robotic Crane System

**Role:** Robotics Software / Autonomy Developer<br>
**Stack:** ROS 2, YOLOv11, Jetson Orin Nano, G-code

Designed a vision-guided robotic crane workflow for logistics side pallet-loading. The system connects perception, AI inference, motion planning, and robotic control into an autonomous pick-and-place pipeline.

### SAFMC - Three-Drone Flocking Foundation

**Status:** Completed<br>
**Role:** Autonomous Drone Programmer

Built and tested autonomous drone flight functions using Python, ROS 2, MAVROS, and MAVLink. Integrated UWB indoor positioning with the flight controller to support non-GPS navigation experiments. This completed work is the foundation for the ongoing SAFMC 2027 high-speed flocking project.

### SEP1 - Limo Autonomous Robot

**Stack:** ROS, LIDAR, SLAM, computer vision

Integrated ROS-based navigation, SLAM, and computer vision workflows on the AgileX Limo platform, including mapping, waypoint navigation, and object-detection experiments.

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
+-- RobotX-Projects/    # Aerial vision, detection training, and drone RC experiments
+   +-- robotx-actual/  # Actual NX UAV/USV/OCS/UI/robocommand source
+   +-- simulation/     # Gazebo boat/drone and OCS simulation stack
+-- cv/                 # CV training scripts and actual AGX RobotX perception source
+-- Fall-Action-Project/ # Drone and hiker pose-based fall/action projects
+-- hospitall-fall-detection/ # Hospital CCTV fall-detection research prototype
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
