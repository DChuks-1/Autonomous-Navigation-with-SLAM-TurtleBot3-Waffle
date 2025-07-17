# TurtleBot3 Autonomous Exploration — Team 24 (2025)

This ROS2 package allows a TurtleBot3 Waffle to autonomously explore a structured 4×4 meter arena using reactive obstacle avoidance, LiDAR-based sensing, a finite state machine, and optional SLAM via `slam_toolbox` or `gmapping`.

---

## 🎥 Demo

![SLAM & Movement Demo](media/exploration_demo.gif)

> The above shows real-time SLAM mapping in Rviz and reactive pathfinding using LiDAR.

---

## 📦 Package Overview

This package was developed for the [ACS6121 - Mobile Robotics](https://tom-howard.github.io/acs6121/) module. It integrates key concepts from ROS2 simulation, robot control, sensing and planning.

### Key Nodes

| Node Name     | Purpose                                           |
|---------------|---------------------------------------------------|
| `lidar.py`    | Processes `/scan` and publishes obstacle data     |
| `decision.py` | Applies FSM logic to decide motion state          |
| `move.py`     | Converts FSM decisions to `Twist` velocity cmds   |
| SLAM (optional) | Uses `gmapping` or `slam_toolbox` with Rviz     |

### Message Types

| Custom Message         | Description                                                  |
|------------------------|--------------------------------------------------------------|
| `ObstacleDistance.msg` | Contains processed min distances (front, left, right, back)  |
| `DecisionState.msg`    | Indicates motion decision (e.g. forward, turn)               |

---

## 📁 Folder Structure

- 📂 `launch/` – Launch files to start all nodes
- 📂 `msg/` – Custom message definitions
- 📂 `scripts/` – Python ROS2 node scripts
- 📂 `maps/` – SLAM map outputs
- 📂 `acs6121_team24_2025_modules/` – Utility or helper modules
- 📄 `CMakeLists.txt` – Build configuration
- 📄 `package.xml` – ROS2 package metadata
- 📄 `README.md` – This file

---

## 🚀 How to Use

### ✅ Prerequisites

- ROS2 Humble or Foxy
- TurtleBot3 Waffle model
- Packages: `turtlebot3_gazebo`, `turtlebot3_msgs`, `slam_toolbox` or `gmapping`
- Rviz2 installed

### 🔧 Build the Package

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/turtlebot3-autonomous-exploration.git
cd ~/ros2_ws
colcon build
source install/setup.bash

---

### Run the System

1. Set the robot model (in the same terminal):
2. Launch the simulation world:
3. In a new terminal, source your workspace:
4. Launch the exploration system:
5. (Optional) Launch SLAM in a new terminal:

---

### Node Interaction Summary

- `/scan` → processed by `lidar.py` → publishes `/obs_detection`
- `/obs_detection` → used by `decision.py` → publishes `/decision_state`
- `/decision_state` and `/odom` → used by `move.py` → publishes `/cmd_vel`
- Rviz uses: `/map`, `/odom`, `/scan`, `/tf`

---

## Features

- Zone-based LiDAR filtering (front, left, right, back)
- Finite State Machine (FSM) logic
- Optional: Artificial Potential Fields in `move.py`
- SLAM support via Rviz
- Modular structure for testing and debugging

---

## Useful Links

- [Part 1 – ROS packages and nodes](https://tom-howard.github.io/acs6121/course/sim/part1/)
- [Part 2 – Subscribers and sensor input](https://tom-howard.github.io/acs6121/course/sim/part2/)
- [Part 3 – FSM and ROS graph](https://tom-howard.github.io/acs6121/course/sim/part3/)
- [Robot Task Overview](https://tom-howard.github.io/acs6121/course/robot/task/)

---

## Notes

- Filters out bad LiDAR data (NaN, inf, zero)
- FSM prevents jitter by checking directions
- Uses `/odom` for speed feedback
- SLAM maps saved in `maps/` folder

---

## Team

ACS6121 Team 24 – 2025  
Robotics Engineering Students

---

## License

MIT License