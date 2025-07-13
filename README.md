<!--
<meta name="keywords" content="AgileX, TurtleBot, Turtlesim, Gazebo, ROS1, Codespaces, ROS Noetic, robotics, navigation, SLAM, catkin"/>
-->

# AgileX TurtleBot ROS1 Codespace

[![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)](#) [![AgileX](https://img.shields.io/badge/Platform-AgileX-green)](#) [![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-orange)](#) [![Turtlesim](https://img.shields.io/badge/Simulator-Turtlesim-lightgrey)](#)

> A containerized **GitHub Codespace** for developing **ROS Noetic** (ROS1) TurtleBot applications on **AgileX** platforms, with Gazebo simulation, Turtlesim demos, SLAM, navigation, and cross-compilation toolchains.

---

## 🚀 Features

- **AgileX–Ready**: Out-of-the-box support for AgileX TurtleBot hardware series (Scout, Limo).  
- **TurtleBot & Turtlesim**: Launch real-world TurtleBot demos or lightweight Turtlesim tutorials.  
- **Gazebo & RViz**: Full simulation stack—Gazebo worlds, custom URDFs, RViz visualization.  
- **ROS Noetic (Ubuntu 20.04)**: Pre-installed `ros-noetic-desktop-full`, `navigation`, `slam_gmapping`, and more.  
- **Catkin Workspace**: Auto-build on container start; seamless `catkin build` workflow.  
- **ARM Cross-Compile**: Build for AgileX’s ARM-based onboard computer via toolchains and udev rules.

---

## 📋 Prerequisites

- GitHub account **with Codespaces enabled**  
- AgileX TurtleBot hardware (e.g., Scout, Limo)  
- Optional: SSH or serial console access to your AgileX robot

---

## 🏷️ Keywords

`AgileX` • `TurtleBot` • `Turtlesim` • `Gazebo` • `ROS1` • `ROS Noetic` • `GitHub Codespaces` • `robotics` • `SLAM` • `navigation`

---

## ⚙️ Getting Started in Codespaces

1. **Clone** this repo to GitHub.  
2. Click **Code ▶ Open with Codespaces ▶ New codespace**.  
3. Wait ~3–5 min for the dev container (with ROS Noetic, Gazebo, Turtlesim, etc.) to spin up.  
4. Terminal lands in `/workspaces/<repo>/src`—you’re ready to build.

---

## 🐳 Development Container

See `.devcontainer/devcontainer.json`:

- **Base:** Ubuntu 20.04  
- **ROS:** `ros-noetic-desktop-full` + `ros-noetic-navigation`, `slam-gmapping`  
- **Simulators:** Gazebo 11, Turtlesim  
- **Build Tools:** `catkin-tools`, `colcon`  
- **Cross-Compile:** ARM toolchain for AgileX  
- **VS Code Exts:** Remote-Containers, ROS, C/C++

> To tweak, edit `.devcontainer/Dockerfile` and restart Codespace.

---

## 🗂 Workspace Layout

