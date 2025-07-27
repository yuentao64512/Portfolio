# AgileX TurtleBot ROS1 Melodic Portfolio

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE) [![ROS1](https://img.shields.io/badge/ROS-Melodic-orange.svg)](http://wiki.ros.org/melodic)

Welcome to my **AgileX TurtleBot ROS1 Melodic Portfolio**! I’m **Ng Yuen Tao**, a Robotics Systems Engineering student at SIT, showcasing hands‑on work with the AgileX Limo robot, Turtlesim, TurtleBot, Gazebo simulations, and Darknet\_ROS (YOLO) on the Limo platform.

---

## 🌐 Portfolio Website

Explore my full portfolio, background, and additional projects:
🔗 [yuentao64512.github.io/Portfolio](https://yuentao64512.github.io/Portfolio/#about)

---

## 📂 Repository Structure

```txt
gazebo-turtlebot-portfolio/
├── turtlesim/           # Turtlesim tutorials and examples
├── launch_files/        # AgileX Limo and TurtleBot launch configurations
├── gazebo/              # Gazebo simulation worlds and models
├── darknet_ros/         # YOLOv3 & YOLOv4 integration on Limo
├── docs/                # Additional documentation and diagrams
├── LICENSE              # MIT License
└── README.md            # This file
```

---

## 🚀 Quick Links

| Project Area                      | Description                                                   | Link            |
| --------------------------------- | ------------------------------------------------------------- | --------------- |
| **Turtlesim Tutorials**           | Learn ROS fundamentals (publish/subscribe, services, params)  | `/turtlesim`    |
| **AgileX Integration**            | Software experiments on AgileX Limo platform                  | `/launch_files` |
| **Gazebo Simulations**            | Simulate Limo and TurtleBot in Gazebo environments            | `/gazebo`       |
| **Darknet\_ROS (YOLO Detection)** | Real‑time object detection (YOLOv3 & YOLOv4) in ROS workflows | `/darknet_ros`  |

---

## 🔧 Getting Started

1. **Clone the repo**

   ```bash
   git clone https://github.com/<your-github-username>/gazebo-turtlebot-portfolio.git
   ```
2. **Install dependencies**
   Ensure you have ROS Melodic installed along with common packages:

   ```bash
   sudo apt update
   sudo apt install ros-melodic-desktop-full ros-melodic-turtlesim ros-melodic-gazebo-ros-pkgs ros-melodic-darknet-ros
   ```
3. **Build workspace**

   ```bash
   cd ~/catkin_ws/src
   ln -s $(pwd)/gazebo-turtlebot-portfolio .
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```
4. **Launch Examples**

   * **Turtlesim**:  `roslaunch turtlesim turtlesim_demo.launch`
   * **AgileX Limo**:  `roslaunch launch_files limo_sim.launch`
   * **Gazebo**:  `roslaunch gazebo gazebo_sim.launch`
   * **YOLO Detection**:  `roslaunch darknet_ros yolo_run.launch`

---

## 🤝 Contributing

Contributions, feedback, and pull requests are welcome! Please:

1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/YourFeature`).
3. Commit your changes (`git commit -m 'Add feature'`).
4. Push and open a PR.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

*Created with ❤️ by Ng Yuen Tao*
