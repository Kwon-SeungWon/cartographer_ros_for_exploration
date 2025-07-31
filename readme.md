# Redbot ROS2 Project

## Overview

This repository is the main workspace for the Redbot Robot ROS2 Project. It integrates multiple ROS2 packages and dependencies, managed as git subtrees for modular development and easy updates. The project aims to provide a robust software stack for autonomous Redbot robots, including sensor drivers, navigation, web interfaces, and visualization tools.

---

## Project Structure

```
Redbot_Robot_ROS2_Project/
├── dependency_sensors/
│   ├── realsense-ros/             # Camera driver (subtree)
│   └── ydlidar_ros2_driver/       # LiDAR driver (subtree)
├── dependency_pkgs/
│   ├── cartographer_ros_for_exploration/  # SLAM system (subtree)
│   ├── Fields2Cover/              # Coverage planning library (subtree)
│   └── opennav_coverage/          # Open navigation coverage (subtree)
├── webserver_interface_ros2/      # Web server interface (subtree)
├── MOBILE_ROBOT_BASIC/            # Main robot logic and launch files (subtree)
├── san_msgs/                      # Custom ROS2 message definitions (subtree)
├── caselab_rviz2_plugin/          # Custom RViz2 plugins (subtree)
├── waypoint_node_manager/         # Waypoint management (subtree)
├── ros2_frontier_based_explore/   # Exploration (subtree)
├── laser_filters/                 # Laser filter (subtree)
├── subtree_tools.sh               # Git subtree automation script
└── README.md                      # This file
```

---

## Submodules/Packages

### dependency_sensors/realsense-ros

- **Purpose:** ROS2 driver for Realsense Camera sensors
- **Source:** [realsense-ros (GitHub)](https://github.com/Kwon-SeungWon/realsense-ros.git)
- **Branch:** ros2-master

### dependency_sensors/ydlidar_ros2_driver

- **Purpose:** ROS2 driver for Ydlidar sensors
- **Source:** [ydlidar_ros2_driver (GitHub)](https://github.com/Kwon-SeungWon/ydlidar_ros2_driver.git)
- **Branch:** master

### dependency_pkgs/cartographer_ros_for_exploration

- **Purpose:** SLAM system for exploration and mapping
- **Source:** [cartographer_ros_for_exploration (GitHub)](https://github.com/Kwon-SeungWon/cartographer_ros_for_exploration.git)
- **Branch:** master

### dependency_pkgs/Fields2Cover

- **Purpose:** Coverage planning library for agricultural robotics
- **Source:** [Fields2Cover (GitHub)](https://github.com/Kwon-SeungWon/Fields2Cover.git)
- **Tag:** v1.2.1

### dependency_pkgs/opennav_coverage

- **Purpose:** Open navigation coverage planning
- **Source:** [opennav_coverage (GitHub)](https://github.com/Kwon-SeungWon/opennav_coverage.git)
- **Branch:** humble

### webserver_interface_ros2

- **Purpose:** Web server interface for robot control and monitoring
- **Source:** [webserver_interface_ros2 (GitHub)](https://github.com/cnr-lab/webserver_interface_ros2.git)
- **Branch:** devel

### MOBILE_ROBOT_BASIC

- **Purpose:** Core logic, launch files, and robot configuration
- **Source:** [MOBILE_ROBOT_BASIC (GitHub)](https://github.com/raisewise0211/MOBILE_ROBOT_BASIC.git)
- **Branch:** redbot

### san_msgs

- **Purpose:** Custom ROS2 message, service, and action definitions
- **Source:** [san_msgs (GitHub)](https://github.com/Kwon-SeungWon/san_msgs.git)
- **Branch:** master

### caselab_rviz2_plugin

- **Purpose:** Custom RViz2 plugins for enhanced visualization
- **Source:** [caselab_rviz2_plugin (GitHub)](https://github.com/cnr-lab/caselab_rviz2_plugin.git)
- **Branch:** master

### waypoint_node_manager

- **Purpose:** Manages navigation waypoints for the robot
- **Source:** [waypoint_node_manager (GitHub)](https://github.com/Kwon-SeungWon/waypoint_node_manager.git)
- **Branch:** master

### ros2_frontier_based_explore

- **Purpose:** Explore the frontier unknown space
- **Source:** [ros2_frontier_based_explore (GitHub)](https://github.com/Kwon-SeungWon/ros2_frontier_based_explore.git)
- **Branch:** master

### laser_filters

- **Purpose:** Laser filtering for remove obstacle inside robot footprint
- **Source:** [laser_filters (GitHub)](https://github.com/ros-perception/laser_filters.git)
- **Branch:** ros2

---

## Git Subtree Management

All external packages are managed as [git subtrees](https://www.atlassian.com/git/tutorials/git-subtree) for easy update and contribution.

### Automation Script: `subtree_tools.sh`

- **add**: Add all subtrees (use only for initial setup)
- **pull**: Update all subtrees to the latest remote commit
- **push**: Push local changes in subtrees back to their respective upstream repositories

#### Usage

```bash
# Add all subtrees (only if not already present)
./subtree_tools.sh add

# Pull latest changes from all upstream repos
./subtree_tools.sh pull

# Push your local changes to upstream repos
./subtree_tools.sh push
```

---

## Getting Started

1. **Clone this repository**
2. **Install ROS2 (Humble or compatible)**
3. **Install dependencies** (see each package's README for details)
4. **Build the workspace**
   ```bash
   cd ~/catkin_ws
   colcon build --symlink-install
   ```
5. **Source the workspace**
   ```bash
   source install/setup.bash
   ```
6. **Launch your desired nodes** (see launch files in each package)

---

## Contribution & License

- Please refer to each subpackage's README and LICENSE for contribution guidelines and licensing.
- Main repository: MIT License (see LICENSE file if present)

---

## Contact

- Maintainer: Kwon-Seungwon/Team (CASELAB)
- For issues, please use the respective GitHub repository issue trackers.

