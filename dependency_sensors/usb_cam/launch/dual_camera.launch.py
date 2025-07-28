#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import sys

# Hack to get relative import of .camera_config file working
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)

from camera_config import CameraConfig, USB_CAM_DIR

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node


# Configure two cameras
CAMERAS = []
CAMERAS.append(
    CameraConfig(
        name='camera_right',
        param_path=Path(USB_CAM_DIR, 'config', 'camera_right.yaml')
    )
)
CAMERAS.append(
    CameraConfig(
        name='camera_left', 
        param_path=Path(USB_CAM_DIR, 'config', 'camera_left.yaml')
    )
)


def generate_launch_description():
    ld = LaunchDescription()

    parser = argparse.ArgumentParser(description='Dual USB Camera Launch')
    parser.add_argument('-n', '--node-name', dest='node_name', type=str,
                        help='name for device', default='usb_cam')

    # Create camera nodes
    camera_nodes = [
        Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            output='screen',
            name=camera.name,
            namespace=camera.namespace,
            parameters=[camera.param_path],
            remappings=camera.remappings
        )
        for camera in CAMERAS
    ]

    # Group all camera actions
    camera_group = GroupAction(camera_nodes)

    ld.add_action(camera_group)
    return ld 