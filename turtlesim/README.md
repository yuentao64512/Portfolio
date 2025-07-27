# Turtlesim Demo Package

A ROS1 package demonstrating the use of the built-in `turtlesim` simulator. Quickly get started with ROS concepts such as nodes, topics, services, and parameters by controlling and interacting with a virtual turtle.

## Features

* Launch the `turtlesim_node` simulator
* Spawn additional turtles at runtime
* Control turtle movement via topics and services
* Teleoperate the turtle with keyboard
* Draw shapes by publishing velocity commands
* Reset and clear the simulator canvas

## Prerequisites

* ROS1 Melodic
* A catkin workspace (assumed: `~/catkin_ws`)

## Installation

1. **Create or navigate** to your catkin workspace:

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   ```
2. **Clone this demo package**:

   ```bash
   git clone https://github.com/yourusername/turtlesim_demo.git
   ```
3. **Build and source**:

   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage
### 1. Launch the Simulator

Start the ROS core and the turtlesim node:

```bash
roscore
#new terminal
Rosrun turtlesim turlesim_node
```

This will open the `turtlesim` window with a single turtle at the center.
```bash
#additional to change turtlenode name
rosrun turtlesim turtlesim_node __name:=YOUR_NODE
#to verify new terminal
rosnode list
```

### 2. Teleoperate with Keyboard

In a new terminal (source your workspace first):

```bash
rosrun turtlesim turtle_teleop_key.py
```

Use `W/A/S/D` or arrow keys to move the turtle around.

### 3. Spawn Additional Turtles

Call the spawn service to add a new turtle:

```bash
rosservice call /spawn 2.0 2.0 0.0 "turtle2"
```

Now you can teleoperate or programmatically control `turtle2` by targeting its topics.

### 4. Draw a Shape

Run the shape-drawing script:

```bash
rosrun turtlesim draw_circle.py
```

This node publishes velocity commands to `/turtle1/cmd_vel` to draw a circle.

### 5. Clear the Canvas

Reset the background to white:

```bash
rosservice call /clear
```

### 6. Reset Simulator

To reset turtle position and clear drawings:

```bash
rosservice call /reset
```

## Package Contents

```
turtlesim_demo/
├── launch/
│   └── demo.launch           # Launches turtlesim_node and teleop
├── scripts/
│   ├── turtle_teleop_key.py  # Keyboard teleop node
├── README.md                 # This file
└── package.xml
```

## Scripts Overview

* **turtle\_teleop\_key.py**: Subscribes to keyboard input and publishes geometry\_msgs/Twist messages.

## Troubleshooting

* **`command not found`**: Ensure scripts are executable: `chmod +x scripts/*.py`.
* **`[Err] No module named rospy`**: Make sure you sourced your workspace: `source ~/catkin_ws/devel/setup.bash` and have ROS installed.
* **Simulator window not opening**: Confirm `roscore` is running and `turtlesim_node` is launched.

## Contributing

1. Fork this repository
2. Add or modify scripts
3. Test your changes
4. Submit a pull request
