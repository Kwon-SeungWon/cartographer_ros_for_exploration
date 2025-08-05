#!/bin/bash

# Migration script for moving subtrees to dependency_pkgs folder
# This script will remove old subtrees and add them with new prefixes

set -e

echo "Starting subtree migration..."

# List of packages to migrate
PACKAGES=(
  "webserver_interface_ros2 https://github.com/cnr-lab/webserver_interface_ros2.git devel"
  "MOBILE_ROBOT_BASIC https://github.com/raisewise0211/MOBILE_ROBOT_BASIC.git wheelchair"
  "san_msgs https://github.com/Kwon-SeungWon/san_msgs.git master"
  "caselab_rviz2_plugin https://github.com/cnr-lab/caselab_rviz2_plugin.git master"
  "waypoint_node_manager https://github.com/Kwon-SeungWon/waypoint_node_manager.git master"
  "ros2_laser_merger https://github.com/Kwon-SeungWon/ros2_laser_merger.git wheelchair"
)

# Step 1: Remove old subtrees
echo "Step 1: Removing old subtrees..."
for entry in "${PACKAGES[@]}"; do
  set -- $entry
  old_path=$1
  repo_url=$2
  branch=$3
  
  echo "Removing old subtree: $old_path"
  
  # Check if the old path exists
  if [ -d "$old_path" ]; then
    # Remove the old subtree
    git rm -rf "$old_path" || true
    git commit -m "Remove old subtree: $old_path" || true
    echo "Removed: $old_path"
  else
    echo "Path does not exist: $old_path (skipping)"
  fi
done

# Step 2: Add new subtrees with correct prefixes
echo "Step 2: Adding new subtrees with correct prefixes..."
for entry in "${PACKAGES[@]}"; do
  set -- $entry
  package_name=$1
  repo_url=$2
  branch=$3
  new_path="dependency_pkgs/$package_name"
  
  echo "Adding new subtree: $new_path ($repo_url:$branch)"
  
  # Check if the new path already exists
  if [ -d "$new_path" ]; then
    echo "Path already exists: $new_path (skipping)"
  else
    # Add the new subtree
    git subtree add --prefix="$new_path" "$repo_url" "$branch" --squash || true
    echo "Added: $new_path"
  fi
done

echo "Migration completed!"
echo "Note: Some packages may already exist in dependency_pkgs/"
echo "You can now use ./subtree_tools.sh for future updates" 