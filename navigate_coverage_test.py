#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from opennav_coverage_msgs.action import NavigateCompleteCoverage
from opennav_coverage_msgs.msg import Coordinates, Coordinate
from geometry_msgs.msg import Polygon, Point32
import time

class NavigateCoverageTest(Node):
    def __init__(self):
        super().__init__('navigate_coverage_test')
        self._action_client = ActionClient(self, NavigateCompleteCoverage, 'navigate_complete_coverage')
        self.get_logger().info('Navigate Coverage Test started')

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = NavigateCompleteCoverage.Goal()
        
        # Create polygon coordinates
        polygon = Polygon()
        
        # Add polygon points (clockwise order) - CLOSED polygon (first = last)
        # Polygon around robot's current position (-12.26, -3.72)
        # Create a 4x4 meter square centered on robot position
        robot_x = -12.26
        robot_y = -3.72
        size = 2.0  # 2 meter radius from center
        
        points = [
            [robot_x - size, robot_y - size],  # Bottom-left
            [robot_x + size, robot_y - size],  # Bottom-right
            [robot_x + size, robot_y + size],  # Top-right
            [robot_x - size, robot_y + size],  # Top-left
            [robot_x - size, robot_y - size],  # Back to start (closed polygon)
        ]
        
        for x, y in points:
            point = Point32()
            point.x = x
            point.y = y
            point.z = 0.0
            polygon.points.append(point)
        
        goal_msg.polygons = [polygon]
        goal_msg.frame_id = "map"
        
        self.get_logger().info('Sending coverage navigation goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result received!')
        self.get_logger().info(f'Error code: {result.error_code}')
        
        if result.error_code == 0:
            self.get_logger().info('SUCCESS! Coverage navigation completed.')
        else:
            self.get_logger().info(f'FAILED! Error code: {result.error_code}')
        
        self.get_logger().info('Test completed')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateCoverageTest()
    action_client.send_goal()
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()