#!/bin/bash

# Git subtree automation script for ROS2 packages
# Usage:
#   ./subtree_tools.sh add   # Add all subtrees
#   ./subtree_tools.sh pull  # Pull (update) all subtrees
#   ./subtree_tools.sh push  # Push all subtrees

set -e

# Define package info: <local_path> <repo_url> <branch> <prefix>
PACKAGES=(
  "webserver_interface_ros2 https://github.com/cnr-lab/webserver_interface_ros2.git devel webserver_interface_ros2"
  "MOBILE_ROBOT_BASIC https://github.com/raisewise0211/MOBILE_ROBOT_BASIC.git wheelchair MOBILE_ROBOT_BASIC"
  "san_msgs https://github.com/Kwon-SeungWon/san_msgs.git master san_msgs"
  "caselab_rviz2_plugin https://github.com/cnr-lab/caselab_rviz2_plugin.git master caselab_rviz2_plugin"
  "waypoint_node_manager https://github.com/Kwon-SeungWon/waypoint_node_manager.git master waypoint_node_manager"
  "ros2_laser_merger https://github.com/Kwon-SeungWon/ros2_laser_merger.git wheelchair ros2_laser_merger"
  "dependency_sensors/Lakibeam_ROS2_Driver https://github.com/Kwon-SeungWon/Lakibeam_ROS2_Driver.git master Lakibeam_ROS2_Driver"
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