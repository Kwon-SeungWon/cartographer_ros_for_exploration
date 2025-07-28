#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionServer
from rclpy.action import CancelResponse, GoalResponse
from std_srvs.srv import Trigger
from san_msgs.srv import BuildMap, ManualManipulatorControl
from san_msgs.action import TaskCommandAction
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import quaternion_from_euler

class ServiceTest(Node):
    def __init__(self):
        super().__init__('build_map_server')
        print("start_service_test_node")
        # ✅ 서비스 서버 생성
        self.srv = self.create_service(BuildMap, 'build_map', self.build_map_callback)
        # self.task_command_srv = self.create_service(TaskCommand, 'task_command', self.task_command_callback)
        self.mani_control_srv = self.create_service(ManualManipulatorControl, 'manual_manipulator_control', self.mani_control_callback)
        self.mani_task_srv = self.create_service(Trigger, 'manipulator_mission_complete', self.mani_task_callback)


        # ✅ 액션 서버 생성
        self._action_server = ActionServer(
            self,
            TaskCommandAction,
            'task_command_action',
            execute_callback=self.execute_task_action_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

    def goal_callback(self, goal_request, _):
        self.get_logger().info(f"Received TaskCommandAction Goal: {goal_request.goal.mission}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel request received for TaskCommandAction")
        return CancelResponse.ACCEPT
    
    async def execute_task_action_callback(self, goal_handle):
        self.get_logger().info(f"Executing TaskCommandAction: mission={goal_handle.request.mission}, start={goal_handle.request.start_node}, goal={goal_handle.request.goal_node}")

        feedback_msg = TaskCommandAction.Feedback()
        for i in range(5):
            feedback_msg.progress = f"{(i + 1) * 20}% done"
            self.get_logger().info(f"Feedback: {feedback_msg.progress}")
            goal_handle.publish_feedback(feedback_msg)
            await rclpy.sleep(Duration(seconds=1))

        result = TaskCommandAction.Result()
        result.success = True
        self.get_logger().info("✅ TaskCommandAction completed successfully")
        goal_handle.succeed()
        return result

    def build_map_callback(self, request, response):
        """ `build_map` 서비스 요청을 처리하는 콜백 함수 """
        self.get_logger().info(f"Received request: command={request.command}, map_name={request.map_name}")
        self.get_logger().info(f"Command Type: {type(request.command)}")  # ✅ 디버깅 코드

        # ✅ 요청된 command 값을 정수로 변환 후 비교
        command_value = int(request.command)

        # ✅ 응답 설정
        if command_value in [1, 2, 3]:
            response.success = True
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))
            self.get_logger().info(f"Executing build_map task: {command_value}")
        else:
            response.success = False
            self.get_logger().warn("Invalid command received")

        return response
    
    def task_command_callback(self, request, response):
        """ `task_command` 서비스 요청을 처리하는 콜백 함수 """
        self.get_logger().info(f"Received taskCommand: mission={request.mission}, start={request.start_node}, goal={request.goal_node}")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))
        response.success = True
        self.get_logger().info(f"✅ Task Command Processed Successfully")
        return response
    
    def mani_control_callback(self, request, response):
        """ `mani_control_callback` 서비스 요청을 처리하는 콜백 함수 """
        self.get_logger().info(f"Received taskCommand: mission={request.command}")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))
        response.success = True
        self.get_logger().info(f"✅ Task Command Processed Successfully")
        return response
    
    def mani_task_callback(self, request, response):
        """ `mani_task_callback` 서비스 요청을 처리하는 콜백 함수 """
        self.get_logger().info(f"Received taskCommand: mission={request.command}")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=2))
        response.success = True
        self.get_logger().info(f"✅ Task Command Processed Successfully")
        return response
    

def main(args=None):
    rclpy.init(args=args)
    node = ServiceTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()