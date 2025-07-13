<!--
<meta name="keywords" content="AgileX, TurtleBot, Turtlesim, Gazebo, ROS1, ROS Melodic, Codespaces, robotics, SLAM, navigation"/>
-->

# AgileX TurtleBot ROS1 Melodic Codespace

[![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-blue)](#) [![AgileX](https://img.shields.io/badge/Platform-AgileX-green)](#) [![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-orange)](#) [![Turtlesim](https://img.shields.io/badge/Simulator-Turtlesim-lightgrey)](#)

> A containerized **GitHub Codespace** for developing **ROS Melodic** (ROS1) TurtleBot applications on **AgileX** platforms, with Gazebo simulation, Turtlesim demos, SLAM, navigation, and ARM cross-compilation toolchains.

---

## 🚀 Features

- **AgileX–Ready**: Out-of-the-box support for AgileX TurtleBot hardware series (Scout, Limo) on Ubuntu 18.04.  
- **TurtleBot & Turtlesim**: Launch real TurtleBot demos or lightweight Turtlesim tutorials under Melodic.  
- **Gazebo & RViz**: Full simulation stack—Gazebo 9 worlds, custom URDFs, RViz visualization.  
- **ROS Melodic (Ubuntu 18.04)**: Pre-installed `ros-melodic-desktop-full`, `ros-melodic-navigation`, `ros-melodic-slam-gmapping`.  
- **Catkin Workspace**: Auto-build on container start; seamless `catkin build` workflow.  
- **ARM Cross-Compile**: Build for AgileX’s ARM-based onboard computer via toolchains and udev rules.

---

## 📋 Prerequisites

- GitHub account **with Codespaces enabled**  
- AgileX TurtleBot hardware (e.g., Scout, Limo)  
- Optional: SSH or serial console access to your AgileX robot

---

## 🏷️ Keywords

`AgileX` • `TurtleBot` • `Turtlesim` • `Gazebo` • `ROS1` • `ROS Melodic` • `GitHub Codespaces` • `robotics` • `SLAM` • `navigation`

---

## ⚙️ Getting Started in Codespaces

1. **Clone** this repo to GitHub.  
2. Click **Code ▶ Open with Codespaces ▶ New codespace**.  
3. Wait ~3–5 min for the dev container (Ubuntu 18.04 + ROS Melodic + Gazebo 9 + Turtlesim) to spin up.  
4. Terminal lands in `/workspaces/<repo>/src`—you’re ready to build.

---

## 🐳 Development Container

See `.devcontainer/devcontainer.json`:

- **Base:** Ubuntu 18.04  
- **ROS:** `ros-melodic-desktop-full` + `ros-melodic-navigation`, `ros-melodic-slam-gmapping`  
- **Simulators:** Gazebo 9, Turtlesim  
- **Build Tools:** `catkin-tools`, `colcon`  
- **Cross-Compile:** ARM toolchain for AgileX  
- **VS Code Exts:** Remote-Containers, ROS, C/C++

> To tweak, edit `.devcontainer/Dockerfile` and restart Codespace.

---

## 🗂 Workspace Layout

