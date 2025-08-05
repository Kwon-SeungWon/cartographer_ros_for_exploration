#!/usr/bin/env python3

import os
import cv2
import json
import signal
import subprocess
import psutil
import shutil
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import MapMetaData, OccupancyGrid
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from sensor_msgs.msg import CompressedImage, JointState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray, PoseStamped, Pose
from san_msgs.srv import BuildMap, TaskCommand, ManualManipulatorControl, PauseTask
from san_msgs.action import TaskCommandAction
from san_msgs.msg import BatteryStatus
from action_msgs.msg import GoalStatus

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import numpy as np
from PIL import Image

from .vda5050_manager import VDA5050Manager
from .node_manager import NodeManager
import yaml

class RosHandler(Node):
    def __init__(self, pkg_share_directory):
        super().__init__("ros_handler")
        self.get_logger().info("start_ros_handler")
        self.vda5050_manager = VDA5050Manager(pkg_share_directory)
        # NodeManager 코드는 수정하지 않고 그대로 사용 (노드 데이터는 내부에 로드된 json/yaml 파일 기반임)
        

        self.bridge = CvBridge()

        self.simulation_mode = False
        self.build_map_mode = False
        self.build_map_name = "None"
        self.build_map_subscriber = None
        
        self.amr_state_ = "None"
        self.manipulator_status = "None"
        self.driving_state = False

        self.cmd_vel_ = Twist()
        self.amcl_pose_ = PoseStamped()
        self.slam_pose_ = PoseStamped()

        # Semantic place 관련 변수 추가
        self.semantic_places = []
        self.current_semantic_place = None

        self.init_param(pkg_share_directory)
        self.load_semantic_places()
        self.set_subscriber()
        self.set_publisher()
        self.set_service_client()
        self.set_action_client()

        self.task_state = {"task_id" : "None",
                           "task_type" : "None",
                           "task_status" : "INITIALIZING"}

        
        self.test_timer = self.create_timer(0.1, self.publish_vda5050_topics)

    def init_param(self,pkg_share_directory):
        ##config_file_path####################################################################
        robot_spec_file_name = "robot_spec.yaml"
        topic_list_file_name = "topic_list.yaml"
        robot_param_file_name = "robot_param.yaml"
        data_dir_fime_name = "data_dir.yaml"

        # Save package share directory for later use
        self.pkg_share_directory = pkg_share_directory

        robot_spec_path = os.path.join(pkg_share_directory, "config", robot_spec_file_name)
        topic_list_path = os.path.join(pkg_share_directory, "config", topic_list_file_name)
        self.mapnode_path = os.path.join(pkg_share_directory, "config", robot_param_file_name)
        self.data_dir_path = os.path.join(pkg_share_directory, "config", data_dir_fime_name)
        ######################################################################################

        with open(self.data_dir_path, 'r') as file:
            data_dir_data = yaml.safe_load(file)

        self.map_data_dir = data_dir_data["map_data"]
        self.node_data_dir = data_dir_data["node_data"]
        self.camera_data_dir = data_dir_data["camera_data"]
        self.buildmap_data_dir = data_dir_data["buildmap_data"]

        robot_spec_data = self.load_yaml_file(robot_spec_path)
        robot_spec = {"version": robot_spec_data["version"],
                      "manufacturer": robot_spec_data["manufacturer"],
                      "serialNumber": robot_spec_data["serialNumber"]}
        for topic in self.vda5050_manager.json_data.keys():
            self.vda5050_manager.update_json_data_subkeys(topic, robot_spec)


        # with open(self.mapnode_path, 'r') as file:
        #     robot_param = yaml.safe_load(file)
        robot_param = self.load_yaml_file(self.mapnode_path)
        self.simulation_mode = robot_param['simulationMode']
        current_mapID = robot_param['currentMapNode']['currentMapID']
        current_nodeID = robot_param['currentMapNode']['currentNodeID']
        current_map = robot_param['currentMapNode']['currentMap']
        current_node = robot_param['currentMapNode']['currentNode']


        state_data = self.vda5050_manager.get_json_data("state")
        state_data["maps"][0]["mapId"] = current_mapID
        state_data["maps"][0]["mapVersion"] = "1.0.0"
        state_data["maps"][0]["mapStatus"] = "ENABLED"
        state_data["zoneSetId"] = current_nodeID
        self.vda5050_manager.update_json_data("state",state_data)
        self.node_manager = NodeManager(self.node_data_dir, current_node)

        connection_data = self.vda5050_manager.get_json_data("connection")
        connection_data["connectionState"] = "ONLINE"
        self.vda5050_manager.update_json_data("connection",connection_data)

        self.topic_names = self.load_yaml_file(topic_list_path)


    def set_subscriber(self):
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic_names["pose"],
            self.amcl_pose_callback,
            1
        )

        self.create_subscription(
            PoseStamped,
            "slam_pose",
            self.slam_pose_callback,
            1
        )

        self.create_subscription(
            Twist,
            self.topic_names["cmd_vel"],
            self.cmd_vel_callback,
            1
        )
        self.create_subscription(
            String,
            "vda5050/instantactions",
            self.instant_action_callback,
            1
        )
        self.create_subscription(
            CompressedImage,
            self.topic_names["camera_front"],
            self.camera_img_callback,
            1
        )
        self.create_subscription(
            BatteryStatus,
            self.topic_names["battery_status"],
            self.battery_status_callback,
            1
        )
        self.create_subscription(
            JointState,
            "manipulator_joint_state",
            self.manipulator_joint_state_callback,
            1
        )
        self.create_subscription(
            Pose,
            "manipulator_pose",
            self.manipulator_pose_callback,
            1
        )
        self.create_subscription(
            String,
            "state_machine",
            self.state_machine_callback,
            1
        )

        self.create_subscription(
            Bool,
            "driviing_state",
            self.driving_state_callback,
            1  
        )
        

    def set_publisher(self):
        self.connection_publisher_ = self.create_publisher(String, "vda5050/connection", 1)
        self.factsheet_publisher_ = self.create_publisher(String, "vda5050/factsheet", 1)
        self.state_publisher_ = self.create_publisher(String, "vda5050/state", 1)
        self.visualization_publisher_ = self.create_publisher(String, "vda5050/visualization", 1)
        self.instantactions_publisher_ = self.create_publisher(String, "vda5050/instantactions/response", 1)
        self.manipulator_state_publisher_ = self.create_publisher(String, "vda5050/manipulator_state", 1)
        self.amr_state_publisher_ = self.create_publisher(String, "vda5050/amr_state", 1)
        self.semantic_place_publisher_ = self.create_publisher(String, "vda5050/semantic_place", 1)

        self.init_pose_pulisher_ = self.create_publisher(PoseWithCovarianceStamped, self.topic_names["initial_pose"], 1)
        self.manual_vel_publisher_ = self.create_publisher(Twist, self.topic_names["manual_vel"], 1)
        self.waypoint_list_publisher_ = self.create_publisher(PoseArray, "waypoint_list", 1)
        self.goal_pose_publisher_ = self.create_publisher(PoseStamped, "goal_pose", 1)
        self.set_current_map_publisher_ = self.create_publisher(Bool, "set_current_map", 1)
        self.set_current_node_publisher_ = self.create_publisher(Bool, "set_current_node", 1)
        self.emergency_quickstop_publisher_ = self.create_publisher(Bool, "emergency_quick_stop", 1)
        self.manipulator_status_publisher_ = self.create_publisher(String, "manipulator_state_machine",1)


    def set_service_client(self):
        self.build_map_client = self.create_client(BuildMap, 'build_map')
        self.manual_manipulator_control_client = self.create_client(ManualManipulatorControl, 'manual_manipulator_control')
        self.manipulator_task_complete_client = self.create_client(Trigger, "manipulator_mission_complete")
        self.pause_task_client = self.create_client(PauseTask, "pause_task")


        while not self.build_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for build_map service...')

        while not self.pause_task_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for pause_task service...')

        # while not self.manual_manipulator_control_client.wait_for_service(timeout_sec=2.0):
        #     self.get_logger().info('Waiting for manual_manipulator_control service...')

        while not self.manipulator_task_complete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for manual_manipulator_control service...')

        if self.simulation_mode == True:
            self.lifecycle_localization_client = self.create_client(ManageLifecycleNodes, 'lifecycle_manager_localization/manage_nodes')
            while not self.lifecycle_localization_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('Waiting for lifecycle manager services...')

    def set_action_client(self):
        self.task_command_action_client = ActionClient(self, TaskCommandAction, 'task_command_action')
        self.task_command_goal_handle = None
        while not self.task_command_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().info('Waiting for task_command_action server...')


    def amcl_pose_callback(self, msg):
        self.amcl_pose_.pose = msg.pose.pose


    def slam_pose_callback(self, msg):
        self.slam_pose_ = msg

    def cmd_vel_callback(self, msg):
        self.cmd_vel_ = msg
        
    def camera_img_callback(self, msg):
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')

        tmp_path = self.camera_data_dir + "/cam_image_ori.jpg"
        final_path = self.camera_data_dir + "/camera.jpg"
        tmp_copy = self.camera_data_dir + "/cam_image_copy.tmp.jpg"

        cv2.imwrite(tmp_path, cv_image)
        os.chmod(tmp_path, 0o644)
        shutil.copyfile(tmp_path, tmp_copy)
        os.chmod(tmp_copy, 0o644)
        os.replace(tmp_copy, final_path)

        # cv2.imwrite(tmp_path, cv_image)

        # # Set permissions before replace
        # os.chmod(tmp_path, 0o644)

        # # Atomic replace (safe from race conditions)
        # os.replace(tmp_path, final_path)

    def battery_status_callback(self,msg):
        battery_soc = msg.soc
        battery_volatage = msg.voltage
        battery_soh = msg.soh
        battery_current = msg.current
        battery_charging_status = msg.charge_relay
        
        state_data = self.vda5050_manager.get_json_data("state")
        state_data["batteryState"]["batteryCharge"] = battery_soc
        state_data["batteryState"]["batteryVoltage"] = battery_volatage
        state_data["batteryState"]["batteryHealth"] = battery_soh
        state_data["batteryState"]["charging"] = battery_charging_status
        self.vda5050_manager.update_json_data("state", state_data)

    def map_callback(self, msg):
        map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
    
        image = np.zeros((msg.info.height, msg.info.width), dtype=np.uint8)
        image[map_data == 0] = 254
        image[map_data == 100] = 0
        image[map_data == -1] = 205

        image = cv2.flip(image, 0)

        map_name = "map"
        image_path = os.path.join(self.buildmap_data_dir, f"{map_name}.jpg")
        yaml_path = os.path.join(self.buildmap_data_dir, f"{map_name}.yaml")

        Image.fromarray(image).save(image_path)
        os.chmod(image_path, 0o644)

        q = msg.info.origin.orientation
        quat = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = euler_from_quaternion(quat)

        map_metadata = {
            'image': f"{map_name}.pgm",
            'resolution': msg.info.resolution,
            'origin': [msg.info.origin.position.x,
                    msg.info.origin.position.y,
                    yaw],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }

        with open(yaml_path, 'w') as f:
            yaml.dump(map_metadata, f, default_flow_style=False)
        os.chmod(yaml_path, 0o644)

    def manipulator_joint_state_callback(self, msg):
        manipulator_state = self.vda5050_manager.get_json_data("manipulatorstate")
        
        manipulator_state["manipulatorState"]["jointNames"] = list(msg.name)
        manipulator_state["manipulatorState"]["jointPositions"] = list(msg.position)
        manipulator_state["manipulatorState"]["jointVelocities"] = list(msg.velocity)

        self.vda5050_manager.update_json_data("manipulatorstate", manipulator_state)
        

    def manipulator_pose_callback(self, msg):
        pass

    def state_machine_callback(self, msg):
        self.amr_state_ = msg.data
        

    def driving_state_callback(self, msg):
        self.driving_state = msg.data
        

    def execute_init_position(self, action):
        print(f"Executing initPosition logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]

        quaternion = self.euler_to_quaternion(float(action_data["value"][2]))
        covariance = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.06853891945200942
        ]
        covariance = [float(x) for x in covariance] 

        msg_ = PoseWithCovarianceStamped()
        msg_.header.stamp = self.get_clock().now().to_msg()
        msg_.header.frame_id = "map"

        msg_.pose.pose.position.x = float(action_data["value"][0])
        msg_.pose.pose.position.y = float(action_data["value"][1])
        
        msg_.pose.pose.orientation.x = quaternion[0]
        msg_.pose.pose.orientation.y = quaternion[1]
        msg_.pose.pose.orientation.z = quaternion[2]
        msg_.pose.pose.orientation.w = quaternion[3]
        msg_.pose.covariance = covariance

        self.init_pose_pulisher_.publish(msg_)
        
    def execute_enable_map(self, action):
        print(f"Executing enableMap logic... Data: {action['actionParameters']}")

    def execute_download_map(self, action):
        print(f"Executing downloadMap logic... Data: {action['actionParameters']}")

    def execute_delete_map(self, action):
        print(f"Executing deleteMap logic... Data: {action['actionParameters']}")

    def execute_cancel_task(self, action):
        print(f"Executing cancelTask logic... Data: {action['actionParameters']}")
        cancel_future = self.task_command_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(lambda fut: self.cancel_done_callback(fut))

    def execute_pause_task(self, action):
        print(f"Executing pauseTask logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        val = action_data["value"][0]
        
        try:
            pause_status = val.lower()
        except Exception as e:
            self.get_logger().error(f"pause_status action data value error: {e}")

        request = PauseTask.Request()

        if pause_status == 'true':
            request.pause_task = True
            future = self.pause_task_client.call_async(request)
            future.add_done_callback(lambda fut: self.pause_task_response_callback(fut))
        elif pause_status == 'false':
            request = PauseTask.Request()
            request.pause_task = False
            future = self.pause_task_client.call_async(request)
            future.add_done_callback(lambda fut: self.pause_task_response_callback(fut))

    def pause_task_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response received: success={response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def execute_state_request(self, action):
        print(f"Executing stateRequest logic... Data: {action['actionParameters']}")
        self.publish_message(self.state_publisher_, "state")
        
    def execute_factsheet_request(self, action):
        print(f"Executing factsheetRequest logic... Data: {action['actionParameters']}")
        self.publish_message(self.factsheet_publisher_, "factsheet")

    def execute_camera_request(self, action):
        print(f"Executing cameraRequest logic... Data: {action['actionParameters']}")

    def execute_manual_control(self, action):
        print(f"Executing ManualControl logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        msg_ = Twist()
        msg_.linear.x = float(action_data["value"][0])
        msg_.linear.y = float(action_data["value"][1])
        msg_.linear.z = float(action_data["value"][2])
        msg_.angular.x = float(action_data["value"][3])
        msg_.angular.y = float(action_data["value"][4])
        msg_.angular.z = float(action_data["value"][5])

        self.manual_vel_publisher_.publish(msg_)

    def execute_goal_position(self, action):
        print(f"Executing setGoalPosition logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        quaternion = self.euler_to_quaternion(float(action_data["value"][2]))

        msg_ = PoseStamped()
        msg_.header.frame_id = "map"
        msg_.header.stamp = self.get_clock().now().to_msg()

        msg_.pose.position.x = float(action_data["value"][0])
        msg_.pose.position.y = float(action_data["value"][1])
        
        msg_.pose.orientation.x = quaternion[0]
        msg_.pose.orientation.y = quaternion[1]
        msg_.pose.orientation.z = quaternion[2]
        msg_.pose.orientation.w = quaternion[3]
        self.goal_pose_publisher_.publish(msg_)
        
    def execute_set_task(self, action):
        print(f"Executing setTask logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]

        task_id_ = action_data["value"][0]
        mission_ = action_data["value"][1]
        start_node_val = action_data["value"][2]
        goal_node_val = action_data["value"][3]
        
        if mission_ == "cancel":
            # cancel_future = self.task_command_goal_handle.cancel_goal_async()
            # cancel_future.add_done_callback(lambda fut: self.cancel_done_callback(fut))
            return
        
        else:
            self.task_state["task_id"] = task_id_
            self.task_state["task_type"] = mission_
            self.task_state["task_status"] = "WAITING"

            if goal_node_val is None:
                self.get_logger().error("goal_node is None! Cannot proceed.")
                return
            else:
                goal_node_ = int(goal_node_val)

            request = TaskCommandAction.Goal()
            request.mission = mission_
            # request.start_node = start_node_
            request.goal_node = goal_node_

        if hasattr(self, 'task_command_goal_handle') and self.task_command_goal_handle is not None:
            if self.task_command_goal_handle.status == GoalStatus.STATUS_ACCEPTED or \
            self.task_command_goal_handle.status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().info("Cancelling current goal before sending new one.")
                cancel_future = self.task_command_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda fut: self.send_goal_after_cancel(fut, request))
        else:
            self.send_goal_directly(request)

    def send_goal_after_cancel(self, future, request):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info("Previous goal canceled successfully.")
            else:
                self.get_logger().warn("No goal was canceled or already finished.")
        except Exception as e:
            self.get_logger().error(f"Goal cancel failed: {e}")
        self.send_goal_directly(request)

    def send_goal_directly(self, request):
        self.get_logger().info("Sending new goal...")
        future = self.task_command_action_client.send_goal_async(request, feedback_callback=self.task_command_feedback_callback)
        future.add_done_callback(self.task_command_response_callback)

    def task_command_feedback_callback(self, feedback_msg):
        self.task_state["task_status"] = "RUNNING"

    def task_command_response_callback(self, future):
        try:
            goal_handle = future.result()
            self.task_command_goal_handle = goal_handle
            if not goal_handle.accepted:
                self.get_logger().warn("Goal was rejected by the action server.")
                self.task_state["task_status"] = "FAILED"
                return

            self.get_logger().info("Goal accepted. Waiting for result...")
            goal_handle.get_result_async().add_done_callback(self.task_command_result_callback)

        except Exception as e:
            self.get_logger().error(f"Action call failed: {e}")

    def task_command_result_callback(self, future):
        try:
            get_result_response = future.result()
            result = get_result_response.result
            self.get_logger().info(f"Action result: success={result.success}, message={result.message}")
            self.task_state["task_status"] = "FINISHED"

        except Exception as e:
            self.get_logger().error(f"Error getting action result: {e}")
            self.task_state["task_status"] = "FAILED"
        self.task_command_goal_handle = None

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.task_state["task_status"] = "CANCELED"
        else:
            self.get_logger().info('Goal failed to cancel')
        self.task_command_goal_handle = None

    def build_map_task(self, action):
        print(f"Executing buildMap logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        command_ = int(action_data["value"][0])
        map_name_ = action_data["value"][1]
        map_id_ = action_data["value"][2]

        request = BuildMap.Request()

        if command_ == 1: #BUILD_MAP_START
            if self.build_map_mode == False:
                self.build_map_mode = True

                request.command = command_
                request.map_name = map_id_
                self.build_map_name = map_id_
                self.build_map_subscriber = self.create_subscription(
                                            OccupancyGrid,
                                            "map",
                                            self.map_callback,
                                            1)
                if self.simulation_mode == True:
                    self.send_lifecycle_command(self.lifecycle_localization_client, "Localization", 1, command_)
    
        elif command_ == 2: #BUILD_MAP_CANCEL 
            if self.build_map_mode == True:
                self.build_map_mode = False

                request.command = command_
                request.map_name = map_id_
                self.build_map_name = "None"
                self.destroy_subscription(self.build_map_subscriber)

                if self.simulation_mode == True:
                    self.stop_slam_toolbox()
                    self.send_lifecycle_command(self.lifecycle_localization_client, "Localization", 2, command_)

        elif command_ == 3: #BUILD_MAP_SAVE
            ##save map request
            if self.build_map_mode == True:
                self.build_map_mode = False

                request.command = command_
                request.map_name = map_name_
                map_file_path = self.map_data_dir+ "/" + self.build_map_name
                
                map_save_result = self.save_map_command(map_file_path)
                self.build_map_name = "None"

                if map_save_result == True:
                    action_parameter_value = [str(command_),str(map_name_),str(map_id_),str(1)]
                    response_msg = self.vda5050_manager.get_json_data("buildMap")
                    response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
                    self.vda5050_manager.update_json_data("buildMap",response_msg)
                    self.publish_message(self.instantactions_publisher_, "buildMap")

                elif map_save_result == False:
                    action_parameter_value = [str(command_),str(map_name_),str(map_id_),str(0)]
                    response_msg = self.vda5050_manager.get_json_data("buildMap")
                    response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
                    self.vda5050_manager.update_json_data("buildMap",response_msg)
                    self.publish_message(self.instantactions_publisher_, "buildMap")

                if self.simulation_mode == True:
                    self.stop_slam_toolbox()
                    self.send_lifecycle_command(self.lifecycle_localization_client, "Localization", 2, command_)
                self.destroy_subscription(self.build_map_subscriber)
        else:
            print("error command type undefined")

        ##for_reponse_test##########################################################################
        # action_parameter_value = [str(command_),str(map_name_),str(map_id_),str(1)]

        # response_msg = self.vda5050_manager.get_json_data("buildMap")
        # response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
        # self.vda5050_manager.update_json_data("buildMap",response_msg)
        # self.publish_message(self.instantactions_publisher_, "buildMap")
        ##############################################################################################
        
        future = self.build_map_client.call_async(request)
        future.add_done_callback(lambda fut: self.build_map_response_callback(fut, map_name_, map_id_))

    def build_map_response_callback(self, future, map_name, map_id):
        try:
            response = future.result()
            command = response.message
            self.get_logger().info(f"Response received: success={response.success}")
            if response.success == True and command != 3:
                action_parameter_value = [str(command),str(map_name),str(map_id),str(1)]
                response_msg = self.vda5050_manager.get_json_data("buildMap")
                response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
                self.vda5050_manager.update_json_data("buildMap",response_msg)
                self.publish_message(self.instantactions_publisher_, "buildMap")

            elif response.success == False and command != 3:
                action_parameter_value = [str(command),str(map_name),str(map_id),str(0)]
                response_msg = self.vda5050_manager.get_json_data("buildMap")
                response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
                self.vda5050_manager.update_json_data("buildMap",response_msg)
                self.publish_message(self.instantactions_publisher_, "buildMap")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def save_map_command(self, map_filename):
        subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f' , map_filename
        ])

        timeout = 5
        pgm_path = f"{map_filename}.pgm"
        jpg_path = f"{map_filename}.jpg"
        waited = 0

        while not os.path.exists(pgm_path):
            time.sleep(0.1)
            waited += 0.1
            if waited > timeout:
                self.get_logger().error(f"Timeout: map image {pgm_path} not created.")
                return False

        image = Image.open(pgm_path)
        image_np = np.array(image)

        image_np[image_np == 205] = 255

        image_out = Image.fromarray(image_np, mode='L')
        image_out.save(jpg_path, format='JPEG')
        os.chmod(jpg_path, 0o644)
        
        self.get_logger().info(f"save map finish: {jpg_path}")

        return True


    ##for_simulation##############################################################################################
    def send_lifecycle_command(self, client, service_name, command_id, ia_command_):
        request = ManageLifecycleNodes.Request()
        request.command = command_id
        future = client.call_async(request)
        future.add_done_callback(lambda future: self.response_lifecycle_callback(future, service_name, ia_command_))

    def response_lifecycle_callback(self, future, service_name, ia_command_):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully executed command on {service_name}: {response.success}.")
                if ia_command_ == 1:
                    self.start_slam_toolbox()
            else:
                self.get_logger().error(f"Failed to manage lifecycle nodes on {service_name}.")
        except Exception as e:
            self.get_logger().error(f"Service call failed on {service_name}: {e}")

    def start_slam_toolbox(self):
        self.get_logger().info("Starting SLAM Toolbox...")
        self.slam_toolbox_process = subprocess.Popen(
            ['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=true'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        self.get_logger().info("SLAM Toolbox started successfully.")

    def stop_slam_toolbox(self):
        if hasattr(self, 'slam_toolbox_process') and self.slam_toolbox_process is not None:
            self.get_logger().info(f"Stopping SLAM Toolbox (PID: {self.slam_toolbox_process.pid})...")
            parent_pid = self.slam_toolbox_process.pid 
            parent_process = psutil.Process(parent_pid)
            children = parent_process.children(recursive=True)
            for child in children:
                self.get_logger().info(f"Terminating child process PID: {child.pid}")
                child.terminate()
            gone, still_alive = psutil.wait_procs(children, timeout=5)
            for child in still_alive:
                self.get_logger().warn(f"Force killing child process PID: {child.pid}")
                child.kill()
            self.slam_toolbox_process.terminate()
            try:
                self.slam_toolbox_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("SLAM Toolbox did not exit in time. Sending SIGKILL to parent...")
                os.kill(parent_pid, signal.SIGKILL)
            self.slam_toolbox_process = None
            self.get_logger().info("SLAM Toolbox successfully stopped.")

    def control_map_task(self, action):
        print(f"Executing controlMap logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        command_ = action_data["value"][0]
        map_name_ = action_data["value"][1]
        map_id_ = action_data["value"][2]
        # map_downlink_ = action_data["value"][3]

        if command_ == str(1):
            print(f"map add process")

        elif command_ == str(2):
            print(f"map delete process")

        elif command_ == str(3):
            print(f"map set current process")
            data = Bool()
            data.data = True
            self.set_current_map_publisher_.publish(data)
        else:
            print("error command type undefined")

        ##for_reponse_test##########################################################################
        action_parameter_value = [str(command_),str(map_name_),str(map_id_),str(1)]

        response_msg = self.vda5050_manager.get_json_data("controlMap")
        response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
        self.vda5050_manager.update_json_data("controlMap",response_msg)
        self.publish_message(self.instantactions_publisher_, "controlMap")
        ##############################################################################################

    def control_nodelist_task(self, action):
        print(f"Executing controlNodelist logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        command_ = action_data["value"][0]
        nodelist_name_ = action_data["value"][1]
        nodelist_id_ = action_data["value"][2]
        # nodelist_downlink_ = action_data["value"][3]

        if command_ == str(1):
            print(f"nodelist add process")
        elif command_ == str(2):
            print(f"nodelist delete process")
        elif command_ == str(3):
            print(f"nodelist set current process")
            data = Bool()
            data.data = True
            self.set_current_node_publisher_.publish(data)
        elif command_ == 4:
            print("nodelist edit process")
        else:
            print(f"error command type undefined")

        ##for_reponse_test##########################################################################
        action_parameter_value = [str(command_),str(nodelist_name_),str(nodelist_id_),str(1)]

        response_msg = self.vda5050_manager.get_json_data("controlNodelist")
        response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
        self.vda5050_manager.update_json_data("controlNodelist",response_msg)
        self.publish_message(self.instantactions_publisher_, "controlNodelist")
        ############################################################################################

    def control_manipulator_task(self, action):
        print(f"Executing controlManipulator logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        command_ = action_data["value"][0]
        joint1_angle = float(action_data["value"][1])
        joint2_angle = float(action_data["value"][2])
        joint3_angle = float(action_data["value"][3])
        joint4_angle = float(action_data["value"][4])
        joint5_angle = float(action_data["value"][5])
        joint6_angle = float(action_data["value"][6])

        request = ManualManipulatorControl.Request()

        if command_ == str(1):
            request.command = command_
            request.position = [joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle]
            
            future = self.manual_manipulator_control_client.call_async(request)
            future.add_done_callback(lambda fut: self.manual_manipulator_control_response_callback(fut, command_))
        elif command_ == str(2):
            request.command = command_
            
            future = self.manual_manipulator_control_client.call_async(request)
            future.add_done_callback(lambda fut: self.manual_manipulator_control_response_callback(fut, command_))
        else:
            print("error command type undefined")

        ##for_reponse_test##########################################################################
        # action_parameter_value = [str(command_),str(1)]

        # response_msg = self.vda5050_manager.get_json_data("controlManipulator")
        # response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
        # self.vda5050_manager.update_json_data("controlManipulator",response_msg)
        # self.publish_message(self.instantactions_publisher_, "controlManipulator")
        ############################################################################################
    
        # future = self.manual_manipulator_control_client.call_async(request)
        # future.add_done_callback(self.manual_manipulator_control_response_callback)

    def manual_manipulator_control_response_callback(self, future, command_):
        try:
            response = future.result()
            self.get_logger().info(f"Response received: success={response.success}")
            if response.success == True:
                action_parameter_value = [str(command_), str(1)]
                response_msg = self.vda5050_manager.get_json_data("controlManipulator")
                response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
                self.vda5050_manager.update_json_data("controlManipulator",response_msg)
                self.publish_message(self.instantactions_publisher_, "controlManipulator")

            elif response.success == False:
                action_parameter_value = [str(command_), str(0)]
                response_msg = self.vda5050_manager.get_json_data("controlManipulator")
                response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
                self.vda5050_manager.update_json_data("controlManipulator",response_msg)
                self.publish_message(self.instantactions_publisher_, "controlManipulator")

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def set_mapnode_task(self, action):
        print(f"Executing setMapNode logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        map_id = action_data["value"][0]
        node_id = action_data["value"][1]

        ##set_mapnode seqaunce #####################################################
        mapnode_data = self.load_yaml_file(self.mapnode_path)
        new_map_id = map_id
        new_node_id = node_id
        new_current_map = map_id
        new_current_node = map_id + "_" + node_id + "_" + "node"
        
        mapnode_data["currentMapNode"]["currentMapID"] = new_map_id
        mapnode_data["currentMapNode"]["currentNodeID"] = new_node_id
        mapnode_data["currentMapNode"]["currentMap"] = new_current_map
        mapnode_data["currentMapNode"]["currentNode"] = new_current_node
        
        with open(self.mapnode_path, 'w') as outfile:
            yaml.dump(mapnode_data, outfile, default_flow_style=False, allow_unicode=True, sort_keys=False)

        state_data = self.vda5050_manager.get_json_data("state")
        state_data["maps"][0]["mapId"] = map_id
        state_data["maps"][0]["mapVersion"] = "1.0.0"
        state_data["maps"][0]["mapStatus"] = "ENABLED"
        state_data["zoneSetId"] = node_id
        self.vda5050_manager.update_json_data("state",state_data)
        self.node_manager.update_current_node(self.node_data_dir, new_current_node)
        ############################################################################################
        data = Bool()
        data.data = True
        self.set_current_map_publisher_.publish(data)
        ##for_reponse_test##########################################################################
        action_parameter_value = [str(1)]
        response_msg = self.vda5050_manager.get_json_data("setMapNode")
        # response_msg["actions"][0]["actionParameters"][0]["value"] = []
        response_msg["actions"][0]["actionParameters"][0]["value"] = action_parameter_value
        self.vda5050_manager.update_json_data("setMapNode",response_msg)
        self.publish_message(self.instantactions_publisher_, "setMapNode")
        ############################################################################################

    def set_emergencystop_task(self, action):
        print(f"Executing emergencyStop logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        val = action_data["value"][0]
        if isinstance(val, str):
            emergency_stop_data = val.lower() == "true"
        else:
            emergency_stop_data = bool(val)
        msg = Bool()
        msg.data = emergency_stop_data
        self.get_logger().fatal(f"emergencyStop data!! : " + str(msg.data))
        self.emergency_quickstop_publisher_.publish(msg)

    def send_manipulator_state_task(self, action):
        self.get_logger().info(f"Executing sendManipulatorState logic... Data: {action['actionParameters']}")
        action_data = action["actionParameters"][0]
        val = action_data["value"][0]
        
        self.manipulator_status = val

        if val == '1':
            requsest = Trigger.Request()
            future = self.manipulator_task_complete_client.call_async(requsest)
            future.add_done_callback(lambda fut: self.manipulator_task_complete_response_callback(fut))
        else:
            print("error command type undefined")

    def manipulator_task_complete_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Response received: success={response.success}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        

    def instant_action_callback(self, msg):
        try:
            action_msg = json.loads(msg.data)
            action_map = {
                "initPosition": self.execute_init_position,#
                "enableMap": self.execute_enable_map,### 검토
                "downloadMap": self.execute_download_map,### 검토
                "deleteMap": self.execute_delete_map,### 검토
                "cancelTask": self.execute_cancel_task,
                "pauseTask": self.execute_pause_task,
                "stateRequest": self.execute_state_request,
                "factsheetRequest": self.execute_factsheet_request,
                "cameraRequest": self.execute_camera_request,#미사용
                "manualControl": self.execute_manual_control,
                "setGoalPosition": self.execute_goal_position,
                "setTask": self.execute_set_task,
                "buildMap": self.build_map_task,
                "controlMap": self.control_map_task,
                "controlNodelist": self.control_nodelist_task,
                "controlManipulator": self.control_manipulator_task,
                "setMapNode": self.set_mapnode_task,
                "emergencyStop": self.set_emergencystop_task,
                "sendManipulatorState": self.send_manipulator_state_task,
            }
            for action in action_msg["actions"]:
                action_id = action["actionId"]
                func = action_map.get(action_id)
                if func:
                    func(action)
                else:
                    print(f"Unknown actionId: {action_id}")
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
        except KeyError as e:
            print(f"JSON key error: {e}")
        except Exception as e:
            print(f"JSON error: {e}")


    def publish_message(self, publisher, key):
        json_data = self.vda5050_manager.get_json_data(key)
        if key != "factsheet":
            self.vda5050_manager.update_json_header_time(key)
        json_string = json.dumps(json_data)
        msg = String()
        msg.data = json_string
        publisher.publish(msg)

    def create_semantic_place_data(self, detected_place=None):
        """Semantic place 데이터를 생성하는 함수"""
        if detected_place:
            semantic_place_data = {
                "headerId": 0,
                "timestamp": self.vda5050_manager.get_current_timestamp(),
                "version": "0.0.0",
                "manufacturer": "CASELAB",
                "serialNumber": "SERIALNUMBER",
                "semanticPlace": {
                    "id": detected_place["id"],
                    "name": detected_place["name"],
                    "color": detected_place["color"],
                    "pose": detected_place["pose"]
                }
            }
        else:
            semantic_place_data = {
                "headerId": 0,
                "timestamp": self.vda5050_manager.get_current_timestamp(),
                "version": "0.0.0",
                "manufacturer": "CASELAB",
                "serialNumber": "SERIALNUMBER",
                "semanticPlace": {
                    "id": -1,
                    "name": "none",
                    "color": "none",
                    "pose": []
                }
            }
        return semantic_place_data

    def publish_vda5050_topics(self):
        amr_position = PoseStamped()
        if self.amr_state_ == "SLAM":
            amr_position = self.slam_pose_
        else:
            amr_position = self.amcl_pose_

        _, _, yaw = self.quaternion_to_euler(amr_position.pose.orientation)
        
        state_msg = self.vda5050_manager.get_json_data("state")
        state_msg["actionStates"][0]["actionId"] = self.task_state["task_id"]
        state_msg["actionStates"][0]["actionType"] = self.task_state["task_type"]
        state_msg["actionStates"][0]["actionStatus"] = self.task_state["task_status"]
        state_msg["driving"] = self.driving_state
        state_msg["velocity"]["vx"] = self.cmd_vel_.linear.x
        state_msg["velocity"]["vy"] = self.cmd_vel_.linear.y
        state_msg["velocity"]["omega"] = self.cmd_vel_.angular.z
        state_msg["agvPosition"]["x"] = amr_position.pose.position.x
        state_msg["agvPosition"]["y"] = amr_position.pose.position.y
        state_msg["agvPosition"]["theta"] = yaw
        self.vda5050_manager.update_json_data("state",state_msg)

        visualization_msg = self.vda5050_manager.get_json_data("visualization")
        visualization_msg["velocity"]["vx"] = self.cmd_vel_.linear.x
        visualization_msg["velocity"]["vy"] = self.cmd_vel_.linear.y
        visualization_msg["velocity"]["omega"] = self.cmd_vel_.angular.z

        visualization_msg["agvPosition"]["x"] = amr_position.pose.position.x
        visualization_msg["agvPosition"]["y"] = amr_position.pose.position.y
        visualization_msg["agvPosition"]["theta"] = yaw
        self.vda5050_manager.update_json_data("visualization", visualization_msg)

        amr_state = self.vda5050_manager.get_json_data("amrstate")
        amr_state["amrState"]["stateMachine"] = self.amr_state_
        self.vda5050_manager.update_json_data("amrstate", amr_state)

        # Semantic place 감지 및 publish
        current_x = amr_position.pose.position.x
        current_y = amr_position.pose.position.y
        detected_place = self.detect_semantic_place(current_x, current_y)
        
        self.get_logger().debug(f"Detected place: {detected_place['name'] if detected_place else 'None'}")
        
        if detected_place:
            semantic_place_data = self.create_semantic_place_data(detected_place)
            self.vda5050_manager.update_json_data("semanticPlace", semantic_place_data)
            self.publish_message(self.semantic_place_publisher_, "semanticPlace")
        else:   # None
            semantic_place_data = self.create_semantic_place_data()
            self.vda5050_manager.update_json_data("semanticPlace", semantic_place_data)
            self.publish_message(self.semantic_place_publisher_, "semanticPlace")

        self.publish_message(self.visualization_publisher_, "visualization")
        self.publish_message(self.connection_publisher_, "connection")
        self.publish_message(self.factsheet_publisher_, "factsheet")
        self.publish_message(self.state_publisher_, "state")
        self.publish_message(self.manipulator_state_publisher_, "manipulatorstate")
        self.publish_message(self.amr_state_publisher_, "amrstate")

        # self.publish_message(self.instantactions_publisher_, self.vda5050_manager.get_json_data("instantactions"))
        # self.publish_message(self.order_publisher_, self.vda5050_manager.get_json_data("order"))

    def quaternion_to_euler(self, orientation):
        quat_tuple = (orientation.x, orientation.y, orientation.z, orientation.w)
        euler = euler_from_quaternion(quat_tuple)
        return euler

    def euler_to_quaternion(self, euler_z_data):
        euler_x = 0.0
        euler_y = 0.0
        euler_z = euler_z_data
        quaternion = quaternion_from_euler(euler_x, euler_y, euler_z)
        return quaternion

    def load_yaml_file(self, yaml_file_path):
        try:
            with open(yaml_file_path, "r", encoding="utf-8") as file:
                yaml_data = yaml.safe_load(file)
                return yaml_data
        except FileNotFoundError:
            print(f"Error: yaml file not found: {yaml_file_path}")
            return {}
        except yaml.YAMLError as e:
            print(f"Error: YAML read error: {e}")
            return {}

    def get_nearest_node(self, current_x, current_y):
        node_data = self.node_manager.node_data 
        min_distance = float('inf')
        nearest_node = None
        if "node" in node_data:
            for node in node_data["node"]:
                x = node["position"]["x"]
                y = node["position"]["y"]
                distance = ((current_x - x)**2 + (current_y - y)**2) ** 0.5
                if distance < min_distance:
                    min_distance = distance
                    nearest_node = node["index"]
        return nearest_node

    def load_semantic_places(self):
        """Load semantic places from JSON file"""
        json_path = os.path.join(self.pkg_share_directory, "json_schemas", "semanticPlace.json")
        
        try:
            with open(json_path, 'r') as file:
                data = json.load(file)
                # If semanticPlaces array exists, use it, otherwise use an empty array
                if "semanticPlaces" in data:
                    self.semantic_places = data["semanticPlaces"]
                else:
                    self.semantic_places = []
            
            self.get_logger().info(f'Semantic places loaded: {len(self.semantic_places)} places')
        except Exception as e:
            self.get_logger().error(f'Failed to load semantic places: {e}')
            self.semantic_places = []

    def detect_semantic_place(self, current_x, current_y):
        """Current position is detected in which semantic place"""
        if not self.semantic_places:
            self.get_logger().warning("No semantic places loaded. Cannot detect place.")
            return None

        for place in self.semantic_places:
            if self.is_point_in_polygon(current_x, current_y, place["pose"]):
                if self.current_semantic_place != place:
                    self.current_semantic_place = place
                    self.get_logger().info(f"Entered semantic place: {place['name']} (ID: {place['id']})")
                return place

        # If the current position is not in any area, return None
        if self.current_semantic_place:
            self.get_logger().info(f"Left semantic place: {self.current_semantic_place['name']}")
            self.current_semantic_place = None
        
        return None

    def is_point_in_polygon(self, x, y, polygon_points):

        n = len(polygon_points)
        inside = False
        
        p1x, p1y = polygon_points[0]["x"], polygon_points[0]["y"]
        for i in range(n + 1):
            p2x, p2y = polygon_points[i % n]["x"], polygon_points[i % n]["y"]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside

def main(args=None):
    rclpy.init(args=args)
    package_name = "webserver_interface_ros2"
    package_share_directory = get_package_share_directory(package_name)
    node = RosHandler(package_share_directory)

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("shutting down ros_handler node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()