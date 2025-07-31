#!/bin/bash

# Git subtree automation script for ROS2 packages
# Usage:
#   ./subtree_tools.sh add   # Add all subtrees
#   ./subtree_tools.sh pull  # Pull (update) all subtrees
#   ./subtree_tools.sh push  # Push all subtrees

set -e

# Define package info: <local_path> <repo_url> <branch> <prefix> <can_push>
# Organized according to readme.md folder structure
# can_push: true if we have write access, false for read-only repos
PACKAGES=(
  # dependency_sensors/ - Sensor drivers
  "dependency_sensors/realsense-ros https://github.com/Kwon-SeungWon/realsense-ros.git ros2-master realsense-ros true"
  "dependency_sensors/ydlidar_ros2_driver https://github.com/Kwon-SeungWon/ydlidar_ros2_driver.git humble ydlidar_ros2_driver true"
  
  # dependency_pkgs/ - Core dependencies
  "dependency_pkgs/cartographer_ros_for_exploration https://github.com/Kwon-SeungWon/cartographer_ros_for_exploration.git master cartographer_ros_for_exploration true"
  "dependency_pkgs/Fields2Cover https://github.com/Kwon-SeungWon/Fields2Cover.git v1.2.1 Fields2Cover true"
  "dependency_pkgs/opennav_coverage https://github.com/Kwon-SeungWon/opennav_coverage.git humble opennav_coverage true"

  # Root level packages
  "webserver_interface_ros2 https://github.com/cnr-lab/webserver_interface_ros2.git devel webserver_interface_ros2 true"
  "MOBILE_ROBOT_BASIC https://github.com/raisewise0211/MOBILE_ROBOT_BASIC.git redbot MOBILE_ROBOT_BASIC true"
  "san_msgs https://github.com/Kwon-SeungWon/san_msgs.git master san_msgs true"
  "caselab_rviz2_plugin https://github.com/cnr-lab/caselab_rviz2_plugin.git master caselab_rviz2_plugin true"
  "waypoint_node_manager https://github.com/Kwon-SeungWon/waypoint_node_manager.git master waypoint_node_manager true"
  "ros2_frontier_based_explore https://github.com/Kwon-SeungWon/ros2_frontier_based_explore.git master ros2_frontier_based_explore true"
  "laser_filters https://github.com/ros-perception/laser_filters.git ros2 laser_filters false"
)

add_subtrees() {
  for entry in "${PACKAGES[@]}"; do
    set -- $entry
    local_path=$1
    repo_url=$2
    branch=$3
    prefix=$4
    can_push=$5
    echo "Adding subtree: $local_path ($repo_url:$branch)"
    git subtree add --prefix=$local_path $repo_url $branch --squash || echo "Warning: Failed to add subtree $local_path"
  done
}

pull_subtrees() {
  for entry in "${PACKAGES[@]}"; do
    set -- $entry
    local_path=$1
    repo_url=$2
    branch=$3
    prefix=$4
    can_push=$5
    echo "Pulling subtree: $local_path ($repo_url:$branch)"
    git subtree pull --prefix=$local_path $repo_url $branch --squash || echo "Warning: Failed to pull subtree $local_path"
  done
}

push_subtrees() {
  for entry in "${PACKAGES[@]}"; do
    set -- $entry
    local_path=$1
    repo_url=$2
    branch=$3
    prefix=$4
    can_push=$5
    
    if [ "$can_push" = "false" ]; then
      echo "Skipping push for read-only repository: $local_path ($repo_url)"
      continue
    fi
    
    echo "Pushing subtree: $local_path ($repo_url:$branch)"
    
    # Try normal push first
    if git subtree push --prefix=$local_path $repo_url $branch; then
      echo "Successfully pushed $local_path"
    else
      echo "Push failed for $local_path, attempting to resolve conflicts..."
      
      # Try to resolve non-fast-forward issues
      cd $local_path
      if git pull origin $branch --rebase; then
        cd - > /dev/null
        if git subtree push --prefix=$local_path $repo_url $branch; then
          echo "Successfully pushed $local_path after resolving conflicts"
        else
          echo "Warning: Failed to push $local_path even after conflict resolution"
        fi
      else
        cd - > /dev/null
        echo "Warning: Failed to resolve conflicts for $local_path"
      fi
    fi
  done
}

# Function to check repository status
check_subtree_status() {
  echo "Checking subtree status..."
  for entry in "${PACKAGES[@]}"; do
    set -- $entry
    local_path=$1
    repo_url=$2
    branch=$3
    prefix=$4
    can_push=$5
    
    if [ -d "$local_path" ]; then
      echo "✓ $local_path exists"
      if [ "$can_push" = "false" ]; then
        echo "  (read-only repository)"
      fi
    else
      echo "✗ $local_path missing"
    fi
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
  status)
    check_subtree_status
    ;;
  *)
    echo "Usage: $0 {add|pull|push|status}"
    echo ""
    echo "Commands:"
    echo "  add     - Add all subtrees"
    echo "  pull    - Pull (update) all subtrees"
    echo "  push    - Push all subtrees (skips read-only repos)"
    echo "  status  - Check status of all subtrees"
    exit 1
    ;;
esac 