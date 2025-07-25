# darknet\_ros

A ROS1 wrapper for Darknet YOLO real-time object detection.

## Features

* Real-time detection with YOLO (v3, v4, v5, v7)
* Publishes bounding boxes, annotated images, and detection flags
* Configurable via ROS parameters and YAML files
* Supports GPU acceleration with CUDA
* Compatible with ROS1 Melodic & Noetic

## Table of Contents

* [Installation](#installation)
* [Usage](#usage)
* [Configuration](#configuration)
* [Topics](#topics)
* [Launch Files](#launch-files)
* [Examples](#examples)
* [Troubleshooting](#troubleshooting)
* [Contributing](#contributing)
* [License](#license)

## Installation

### Prerequisites

* ROS1 Melodic or Noetic
* CUDA Toolkit (optional for GPU)
* OpenCV
* CMake
* A catkin workspace (`~/catkin_ws` assumed)

### Build

```bash
cd ~/catkin_ws/src
git clone https://github.com/leggedrobotics/darknet_ros.git
rosdep install --from-paths . --ignore-src -r -y
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Download Weights and Config

Place your YOLO config files, weights, and label files in the `yolo_network_config/weights` directory. For example:

```bash
cd ~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights
# Download YOLOv3 pre-trained weights
wget https://pjreddie.com/media/files/yolov3.weights

# Download YOLOv3-tiny .conv.15 pre-trained convolutional layers (for partial training)
wget https://raw.githubusercontent.com/hamzaMahdi/darknet/master/yolov3-tiny.conv.15
```

Place your `.cfg` and `.names` files in `yolo_network_config`, or point to custom paths via launch parameters.

### Run Detection

```bash
roslaunch darknet_ros darknet_ros.launch
```

Override parameters at launch:

```bash
roslaunch darknet_ros darknet_ros.launch image:=/camera/image_raw threshold:=0.6 gpu_id:=0
```

## Configuration

Parameters can be set via YAML or launch arguments:

| Parameter       | Type   | Default                 | Description                    |
| --------------- | ------ | ----------------------- | ------------------------------ |
| `model_cfg`     | string | `*.cfg`                 | Path to Darknet config file    |
| `model_weights` | string | `*.weights`             | Path to Darknet weights file   |
| `class_labels`  | string | `*.names`               | Path to class labels file      |
| `gpu_id`        | int    | `-1`                    | GPU device ID (`-1` for CPU)   |
| `threshold`     | double | `0.5`                   | Detection confidence threshold |
| `image_topic`   | string | `/camera/rgb/image_raw` | Input image topic              |

## Topics

| Topic                          | Type                             | Description                 |
| ------------------------------ | -------------------------------- | --------------------------- |
| `/darknet_ros/bounding_boxes`  | `darknet_ros_msgs/BoundingBoxes` | Detected boxes & classes    |
| `/darknet_ros/detection_image` | `sensor_msgs/Image`              | Annotated image with boxes  |
| `/darknet_ros/found_object`    | `std_msgs/Bool`                  | True if any object detected |
| `image_topic` (configurable)   | `sensor_msgs/Image`              | Raw image feed              |

## Launch Files

* **darknet\_ros.launch**: Main detection node
* **visualization.launch**: RViz visualization

## Examples

**Visualize in RViz**:

```bash
roslaunch darknet_ros visualization.launch
```

**Run on rosbag**:

```bash
rosbag play sample.bag
roslaunch darknet_ros darknet_ros.launch
```

## Troubleshooting

* **No detections**: Verify `image_topic` and active camera feed.
* **GPU not used**: Check CUDA installation and set `gpu_id` appropriately.
* **Build errors**: Ensure OpenCV and CUDA paths are correctly configured in `CMakeLists.txt`.

## Contributing

Contributions welcome! Please:

1. Fork the repository
2. Create a branch for your feature or bugfix
3. Implement changes with tests
4. Submit a pull request with description and issue reference

