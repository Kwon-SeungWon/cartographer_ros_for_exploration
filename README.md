# Wheelchair Robot ROS2 Project

## Overview

This repository is the main workspace for the Wheelchair Robot ROS2 Project. It integrates multiple ROS2 packages and dependencies, managed as git subtrees for modular development and easy updates. The project aims to provide a robust software stack for autonomous wheelchair robots, including sensor drivers, navigation, web interfaces, and visualization tools.

---

## Project Structure

```
Wheelchair_Robot_ROS2_Project/
├── dependency_sensors/
│   └── Lakibeam_ROS2_Driver/      # LiDAR driver (subtree)
├── ros2_laser_merger/             # Laser scan merger (subtree)
├── waypoint_node_manager/         # Waypoint management (subtree)
├── caselab_rviz2_plugin/          # Custom RViz2 plugins (subtree)
├── san_msgs/                      # Custom ROS2 message definitions (subtree)
├── MOBILE_ROBOT_BASIC/            # Main robot logic and launch files (subtree)
├── webserver_interface_ros2/      # Web server interface (subtree)
├── subtree_tools.sh               # Git subtree automation script
└── README.md                      # This file
```

---

## Submodules/Packages

### dependency_sensors/Lakibeam_ROS2_Driver
- **Purpose:** ROS2 driver for Lakibeam LiDAR sensors.
- **Source:** [Lakibeam_ROS2_Driver (GitHub)](https://github.com/Kwon-SeungWon/Lakibeam_ROS2_Driver.git)
- **Branch:** master

### ros2_laser_merger
- **Purpose:** Merges multiple laser scan topics into one.
- **Source:** [ros2_laser_merger (GitHub)](https://github.com/Kwon-SeungWon/ros2_laser_merger.git)
- **Branch:** wheelchair

### waypoint_node_manager
- **Purpose:** Manages navigation waypoints for the robot.
- **Source:** [waypoint_node_manager (GitHub)](https://github.com/Kwon-SeungWon/waypoint_node_manager.git)
- **Branch:** master

### caselab_rviz2_plugin
- **Purpose:** Custom RViz2 plugins for enhanced visualization.
- **Source:** [caselab_rviz2_plugin (GitHub)](https://github.com/cnr-lab/caselab_rviz2_plugin.git)
- **Branch:** master

### san_msgs
- **Purpose:** Custom ROS2 message, service, and action definitions.
- **Source:** [san_msgs (GitHub)](https://github.com/Kwon-SeungWon/san_msgs.git)
- **Branch:** master

### MOBILE_ROBOT_BASIC
- **Purpose:** Core logic, launch files, and robot configuration.
- **Source:** [MOBILE_ROBOT_BASIC (GitHub)](https://github.com/raisewise0211/MOBILE_ROBOT_BASIC.git)
- **Branch:** wheelchair

### webserver_interface_ros2
- **Purpose:** Web server interface for robot control and monitoring.
- **Source:** [webserver_interface_ros2 (GitHub)](https://github.com/cnr-lab/webserver_interface_ros2.git)
- **Branch:** devel

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
   cd /path/to/Wheelchair_Robot_ROS2_Project
   colcon build
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
- Maintainer: [Your Name/Team]
- For issues, please use the respective GitHub repository issue trackers. 