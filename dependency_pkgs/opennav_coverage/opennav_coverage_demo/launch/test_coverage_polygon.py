#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from opennav_coverage_msgs.action import ComputeCoveragePath
from geometry_msgs.msg import Polygon, Point32
import numpy as np

class CoveragePolygonClient(Node):
    def __init__(self):
        super().__init__('coverage_polygon_client')
        
        # Create action client
        self._action_client = ActionClient(
            self, 
            ComputeCoveragePath, 
            'compute_coverage_path'
        )
        
        self.get_logger().info('Coverage Polygon Client started')

    def send_polygon(self, polygon_points, frame_id='map'):
        """
        Send polygon to coverage navigation
        
        Args:
            polygon_points: List of [x, y] coordinates defining the polygon
            frame_id: Frame ID for the polygon
        """
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        
        # Create polygon message
        polygon = Polygon()
        for point in polygon_points:
            p = Point32()
            p.x = float(point[0])
            p.y = float(point[1])
            p.z = 0.0
            polygon.points.append(p)
        
        # Create goal message
        goal_msg = ComputeCoveragePath.Goal()
        goal_msg.polygons = [polygon]
        goal_msg.frame_id = frame_id
        
        # Set coverage parameters
        goal_msg.generate_headland = True
        goal_msg.generate_route = True
        goal_msg.generate_path = True
        
        # Send goal
        self.get_logger().info('Sending polygon to coverage navigation...')
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
        self.get_logger().info(f'Coverage planning completed!')
        self.get_logger().info(f'Error code: {result.error_code}')
        self.get_logger().info(f'Planning time: {result.planning_time}')
        
        if result.nav_path:
            self.get_logger().info(f'Path has {len(result.nav_path.poses)} poses')
        
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    client = CoveragePolygonClient()
    
    # Example polygon (rectangle)
    # You can modify these coordinates to match your area of interest
    polygon_points = [
        [-10.0, -10.0],  # Bottom-left
        [10.0, -10.0],   # Bottom-right
        [10.0, 10.0],    # Top-right
        [-10.0, 10.0],   # Top-left
    ]
    
    client.send_polygon(polygon_points, frame_id='map')
    
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 