#!/bin/bash

# Git subtree automation script for ROS2 packages
# Usage:
#   ./subtree_tools.sh add   # Add all subtrees
#   ./subtree_tools.sh pull  # Pull (update) all subtrees
#   ./subtree_tools.sh push  # Push all subtrees

set -e

# Define package info: <local_path> <repo_url> <branch> <prefix>
# Organized according to readme.md folder structure
PACKAGES=(
  # dependency_sensors/ - Sensor drivers
  "dependency_sensors/realsense-ros https://github.com/Kwon-SeungWon/realsense-ros.git ros2-master realsense-ros"
  "dependency_sensors/ydlidar_ros2_driver https://github.com/Kwon-SeungWon/ydlidar_ros2_driver.git master ydlidar_ros2_driver"
  
  # dependency_pkgs/ - Core dependencies
  "dependency_pkgs/cartographer_ros_for_exploration https://github.com/Kwon-SeungWon/cartographer_ros_for_exploration.git master cartographer_ros_for_exploration"
  "dependency_pkgs/Fields2Cover https://github.com/Kwon-SeungWon/Fields2Cover.git v1.2.1 Fields2Cover"
  "dependency_pkgs/opennav_coverage https://github.com/Kwon-SeungWon/opennav_coverage.git humble opennav_coverage"

  # dependency_pkgs/ - Core dependencies
  # Root level packages
  "webserver_interface_ros2 https://github.com/cnr-lab/webserver_interface_ros2.git devel webserver_interface_ros2"
  "MOBILE_ROBOT_BASIC https://github.com/raisewise0211/MOBILE_ROBOT_BASIC.git redbot MOBILE_ROBOT_BASIC"
  "san_msgs https://github.com/Kwon-SeungWon/san_msgs.git master san_msgs"
  "caselab_rviz2_plugin https://github.com/cnr-lab/caselab_rviz2_plugin.git master caselab_rviz2_plugin"
  "waypoint_node_manager https://github.com/Kwon-SeungWon/waypoint_node_manager.git master waypoint_node_manager"
  "ros2_frontier_based_explore https://github.com/Kwon-SeungWon/ros2_frontier_based_explore.git master ros2_frontier_based_explore"
  "laser_filters https://github.com/ros-perception/laser_filters.git ros2 laser_filters"
)

add_subtrees() {
  for entry in "${PACKAGES[@]}"; do
    set -- $entry
    local_path=$1
    repo_url=$2
    branch=$3
    prefix=$4
    echo "Adding subtree: $local_path ($repo_url:$branch)"
    git subtree add --prefix=$local_path $repo_url $branch --squash || true
  done
}

pull_subtrees() {
  for entry in "${PACKAGES[@]}"; do
    set -- $entry
    local_path=$1
    repo_url=$2
    branch=$3
    prefix=$4
    echo "Pulling subtree: $local_path ($repo_url:$branch)"
    git subtree pull --prefix=$local_path $repo_url $branch --squash || true
  done
}

push_subtrees() {
  for entry in "${PACKAGES[@]}"; do
    set -- $entry
    local_path=$1
    repo_url=$2
    branch=$3
    prefix=$4
    echo "Pushing subtree: $local_path ($repo_url:$branch)"
    git subtree push --prefix=$local_path $repo_url $branch || true
  done
}

case "$1" in
  add)
    add_subtrees
    ;;
  pull)
    pull_subtrees
    ;;
  push)
    push_subtrees
    ;;
  *)
    echo "Usage: $0 {add|pull|push}"
    exit 1
    ;;
esac 