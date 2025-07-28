# Wheelchair Robot ROS2 Project

## Overview

This repository is the main workspace for the Wheelchair Robot ROS2 Project. It integrates multiple ROS2 packages and dependencies, managed as git subtrees for modular development and easy updates. The project aims to provide a robust software stack for autonomous wheelchair robots, including sensor drivers, navigation, web interfaces, and visualization tools.

---

## Project Structure

```
Wheelchair_Robot_ROS2_Project/
├── dependency_sensors/
│   ├── Lakibeam_ROS2_Driver/      # LiDAR driver (subtree)
│   └── usb_cam/                   # USB camera driver (subtree)
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
- **Purpose:** ROS2 driver for Lakibeam LiDAR sensors with dual LiDAR support
- **Source:** [Lakibeam_ROS2_Driver (GitHub)](https://github.com/Kwon-SeungWon/Lakibeam_ROS2_Driver.git)
- **Branch:** master
- **Features:**
  - Dual LiDAR support (left/right)
  - Real-time scan data processing
  - Configurable scan ranges and frequencies
  - TF frame integration (lidar_l_link, lidar_r_link)

### dependency_sensors/usb_cam
- **Purpose:** USB camera driver with dual camera support
- **Source:** [usb_cam (ROS2)](https://github.com/ros-perception/usb_cam)
- **Features:**
  - Dual camera support (camera_r_optical_link, camera_l_optical_link)
  - Configurable resolution and frame rates
  - Real-time image processing
  - TF frame integration

### ros2_laser_merger
- **Purpose:** Merges multiple laser scan topics into one unified scan
- **Source:** [ros2_laser_merger (GitHub)](https://github.com/Kwon-SeungWon/ros2_laser_merger.git)
- **Branch:** wheelchair
- **Features:**
  - Multi-LiDAR scan merging
  - Configurable scan filtering
  - Real-time scan processing

### waypoint_node_manager
- **Purpose:** Manages navigation waypoints for the robot
- **Source:** [waypoint_node_manager (GitHub)](https://github.com/Kwon-SeungWon/waypoint_node_manager.git)
- **Branch:** master
- **Features:**
  - Waypoint creation and management
  - Navigation sequence planning
  - Goal pose management

### caselab_rviz2_plugin
- **Purpose:** Custom RViz2 plugins for enhanced visualization
- **Source:** [caselab_rviz2_plugin (GitHub)](https://github.com/cnr-lab/caselab_rviz2_plugin.git)
- **Branch:** master
- **Features:**
  - Custom visualization tools
  - Robot state display
  - Sensor data visualization

### san_msgs
- **Purpose:** Custom ROS2 message, service, and action definitions
- **Source:** [san_msgs (GitHub)](https://github.com/Kwon-SeungWon/san_msgs.git)
- **Branch:** master
- **Features:**
  - Custom message types
  - Service definitions
  - Action definitions

### MOBILE_ROBOT_BASIC
- **Purpose:** Core logic, launch files, and robot configuration
- **Source:** [MOBILE_ROBOT_BASIC (GitHub)](https://github.com/raisewise0211/MOBILE_ROBOT_BASIC.git)
- **Branch:** wheelchair
- **Features:**
  - Robot control system (amr_core)
  - Navigation system (amr_navigation)
  - SLAM system (amr_cartographer)
  - Odometry system (amr_odometry)
  - Serial communication (amr_serial)
  - Robot description (amr_description)
  - Interface management (amr_interface)

### webserver_interface_ros2
- **Purpose:** Web server interface for robot control and monitoring
- **Source:** [webserver_interface_ros2 (GitHub)](https://github.com/cnr-lab/webserver_interface_ros2.git)
- **Branch:** devel
- **Features:**
  - Web-based robot control
  - Real-time monitoring
  - Status visualization

---

## System Requirements

### Hardware Requirements
- **Sensors:**
  - 2x Lakibeam LiDAR sensors
  - 2x USB cameras
  - Ultrasonic sensors (8 channels)
  - Collision sensors
  - Battery monitoring system

### Software Requirements
- **OS:** Ubuntu 22.04 LTS
- **ROS2:** Humble (recommended)
- **Dependencies:**
  - Eigen3
  - Boost
  - tf2
  - nav2
  - cartographer
  - can_msgs
  - pyserial

### Permission Setup
```bash
# Serial port permissions
sudo usermod -aG dialout $USER

# Video device permissions (for cameras)
sudo usermod -aG video $USER

# Logout and login again for permissions to take effect
```

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

### 1. Clone and Setup
```bash
# Clone the repository
git clone <repository-url>
cd Wheelchair_Robot_ROS2_Project

# Install dependencies
sudo apt update
sudo apt install -y \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    libeigen3-dev \
    libboost-all-dev \
    python3-pip

# Install Python dependencies
pip3 install pyserial
```

### 2. Build the Workspace
```bash
cd ~/catkin_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. Source the Workspace
```bash
source install/setup.bash

# can settings
can
canup
```

### 4. Launch Systems

#### Basic Robot Control (Included Core, Lidar, Merge, Camera)
```bash
ros2 launch amr_core amr_bringup.launch.py
```

#### (Optional) Dual Camera System
```bash
ros2 launch usb_cam dual_camera.launch.py
```

#### (Optional) Dual LiDAR System
```bash
ros2 launch lakibeam1 lakibeam1_scan_dual_lidar.launch.py
```

#### Navigation System
```bash
ros2 launch amr_navigation navigation_wheelchair.launch.py map:=/home/caselab/catkin_ws/src/Wheelchair_Robot_ROS2_Project/data/map_data/maps/caselab_two_room.yaml
```

#### SLAM System
```bash
ros2 launch amr_cartographer amr_cartographer.launch.py
```

#### Web Interface
```bash
ros2 launch webserver_interface_ros2 webserver_interface.launch.py
```

#### Rviz
```bash
ros2 launch caselab_rviz_plugin test.launch.py 
```

### 5. Monitor System
```bash
# List all topics
ros2 topic list

# Monitor specific topics
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /scan
ros2 topic echo /camera1/image_raw
ros2 topic echo /camera2/image_raw

# View TF tree
ros2 run tf2_tools view_frames
```

---

## System Architecture

### Sensor Integration
- **LiDAR System:** Dual LiDAR for 360° coverage
- **Camera System:** Dual USB cameras for visual perception
- **Safety Sensors:** Ultrasonic and collision sensors
- **Positioning:** Wheel encoders and IMU

### Control System
- **Core Control:** amr_core handles robot state machine
- **Navigation:** Nav2-based path planning and execution
- **SLAM:** Cartographer for mapping and localization
- **Interface:** Web-based control and monitoring

### Communication
- **CAN Bus:** Motor control and sensor data
- **Serial:** External interface (joystick, switches)
- **ROS2 Topics:** Inter-node communication
- **Services:** Synchronous operations

---

## Troubleshooting

### Build Issues
```bash
# Create missing include directories
mkdir -p amr_serial/include

# Install dependencies
rosdep install --from-paths src --ignore-src -y
```

### Permission Issues
```bash
# Set device permissions
sudo chmod 666 /dev/video*
sudo chmod 666 /dev/ttyUSB*

# Add user to groups
sudo usermod -aG dialout $USER
sudo usermod -aG video $USER
```

### Runtime Issues
- **Camera not detected:** Check video device permissions
- **LiDAR not working:** Verify network configuration
- **Serial communication failed:** Check port permissions and device names

---

## Development Guidelines

### Code Style
- Follow ROS2 coding standards
- Use meaningful variable and function names
- Add comprehensive comments
- Include proper error handling

### Testing
- Test individual components before integration
- Verify sensor data accuracy
- Check system performance under load
- Validate safety systems

### Documentation
- Update README files for changes
- Document parameter configurations
- Include usage examples
- Maintain troubleshooting guides

---

## Contribution & License
- Please refer to each subpackage's README and LICENSE for contribution guidelines and licensing.
- Main repository: MIT License (see LICENSE file if present)

---

## Contact
- **Maintainer:** Kwon-Seungwon/Team (CASELAB)
- **Email:** ksw7384@hanyang.ac.kr
- **For issues:** Please use the respective GitHub repository issue trackers 
