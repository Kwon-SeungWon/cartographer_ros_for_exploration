#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import networkx as nx

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseArray
from std_srvs.srv import Trigger
from example_interfaces.srv import AddTwoInts
from san_msgs.srv import GetWaypoints

class NodeManager(Node):
    def __init__(self, node_file_dir=None, current_node=None):
        super().__init__("waypoint_node_manager")
        self.pkg_path = get_package_share_directory("waypoint_node_manager")
        self.current_node_name = None
        self.node_list_path = None

        # 기본 테스트 파일 설정
        if node_file_dir is None:
            node_file_dir = self.pkg_path
        if current_node is None:
            current_node = "test_nodes"

        self.update_current_node_path(node_file_dir, current_node)

        self.node_data = {}

        self.node_marker_pub_ = self.create_publisher(MarkerArray, "/web_marker_array", 1)
        self.marker_array = MarkerArray()
        self.visual_idx = 0

        self.node_graph = nx.DiGraph()
        # Only update graph if node_list_path is provided
        if self.node_list_path:
            self.update_graph(self.node_list_path)

        # 서비스 생성
        self.path_planning_srv_ = self.create_service(
            AddTwoInts, 
            'path_planning', 
            self.path_planning_callback
        )
        
        # 새로운 서비스: 실제 waypoint 좌표를 반환
        self.get_waypoints_srv_ = self.create_service(
            GetWaypoints,
            'get_waypoints',
            self.get_waypoints_callback
        )
        
        self.load_graph_srv_ = self.create_service(
            Trigger,
            'load_graph',
            self.load_graph_callback
        )

        self.get_logger().info("NodeManager initialized with services")

    def path_planning_callback(self, request, response):
        """경로 계획 서비스 콜백"""
        start_node = request.a
        target_node = request.b
        
        try:
            waypoints = self.make_dijkstra_path(start_node, target_node)
            if waypoints:
                response.sum = len(waypoints)  # 경로의 노드 수를 반환
                self.get_logger().info(f"Path planning successful: {start_node} -> {target_node}, {len(waypoints)} waypoints")
            else:
                response.sum = -1  # 경로를 찾을 수 없음
                self.get_logger().warn(f"No path found: {start_node} -> {target_node}")
        except Exception as e:
            response.sum = -1
            self.get_logger().error(f"Path planning failed: {str(e)}")
        
        return response

    def get_waypoints_callback(self, request, response):
        """waypoint 좌표를 반환하는 서비스 콜백"""
        start_node = request.start_node
        goal_node = request.goal_node
        
        try:
            waypoints = self.make_dijkstra_path(start_node, goal_node)
            if waypoints:
                # PoseArray로 변환
                pose_array = PoseArray()
                pose_array.header.frame_id = "map"
                pose_array.header.stamp = self.get_clock().now().to_msg()
                
                for wp in waypoints:
                    pose = Pose()
                    pose.position.x = float(wp['x'])
                    pose.position.y = float(wp['y'])
                    pose.position.z = 0.0
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = 0.0
                    pose.orientation.w = 1.0
                    pose_array.poses.append(pose)
                
                response.waypoints = pose_array
                response.success = True
                response.message = f"Generated {len(waypoints)} waypoints from node {start_node} to {goal_node}"
                self.get_logger().info(f"Waypoints generated successfully: {start_node} -> {goal_node}, {len(waypoints)} waypoints")
            else:
                response.success = False
                response.message = f"No path found from node {start_node} to {goal_node}"
                self.get_logger().warn(f"No path found: {start_node} -> {goal_node}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to generate waypoints: {str(e)}"
            self.get_logger().error(f"Waypoint generation failed: {str(e)}")
        
        return response

    def load_graph_callback(self, request, response):
        """그래프 로드 서비스 콜백"""
        try:
            if self.node_list_path and os.path.exists(self.node_list_path):
                self.update_graph(self.node_list_path)
                response.success = True
                response.message = f"Graph loaded from {self.node_list_path}"
                self.get_logger().info("Graph loaded successfully")
            else:
                response.success = False
                response.message = f"Graph file not found: {self.node_list_path}"
                self.get_logger().warn("Graph file not found")
        except Exception as e:
            response.success = False
            response.message = f"Failed to load graph: {str(e)}"
            self.get_logger().error(f"Failed to load graph: {str(e)}")
        
        return response

    def load_node_list(self, node_list_path):
        with open(node_list_path, 'r') as yaml_file:
            self.node_data = yaml.safe_load(yaml_file)

    def make_graph(self):
        self.node_graph.clear()

        for data in self.node_data["node"]:
            self.node_graph.add_node(data['index'], **data)

            if data['connection']:
                for item in data['connection']:
                    self.node_graph.add_edge(data['index'], item)

            if data['type'] == 1:
                cnt = 0
                temp = 0
                for item in data['child']:
                    if cnt == 0:
                        self.node_graph.add_edge(data['index'], item)
                    else:
                        self.node_graph.add_edge(temp, item)
                    temp = item
                    cnt += 1
                self.node_graph.add_edge(temp, data['index'])

    def update_current_node(self, path , current_node_name):
        self.update_current_node_path(path, current_node_name)
        self.update_graph(self.node_list_path)

    def update_current_node_path(self, path, current_node_name):
        if path and current_node_name:
            self.current_node_name = current_node_name + ".yaml"
            self.node_list_path = path + "/" + self.current_node_name
        else:
            self.current_node_name = None
            self.node_list_path = None


    def update_graph(self, path):
        if path and os.path.exists(path):
            self.load_node_list(path)
            self.make_graph()
        else:
            self.get_logger().warn(f"Node list file not found: {path}")

    def make_dijkstra_path(self, start, target):
        try:
            path = nx.dijkstra_path(self.node_graph, start, target)
            waypoint_list = self.get_waypoints_nodexy(path)
            self.get_logger().info(f"Dijkstra Path: {path}")
            self.get_logger().info(f"Dijkstra waypoint_list: {waypoint_list}")
            return waypoint_list
            
        except nx.NetworkXNoPath:
            self.get_logger().error("No path found")

    
    def get_waypoints_nodexy(self, shortest_path):
        waypoints = []
        
        for index in shortest_path:
            if index in self.node_graph.nodes:
                position = self.node_graph.nodes[index].get('position', None)
                if position:
                    waypoints.append(position)
            else:
                self.get_logger().warn(f"Index {index} not found in graph!")

        return waypoints
    
    def make_ros_marker(self):
        self.set_link()
        self.set_node()
        self.get_logger().info("Marker Publishing Complete")

    def publish_ros_marker(self):
        self.node_marker_pub_.publish(self.marker_array)
        # self.get_logger().info(" Markers Published")

    def set_link(self):
        edge_list = list(self.node_graph.edges)

        for (start, end) in edge_list:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = self.visual_idx
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.color.a = 1.0
            marker.scale.x = 0.02
            marker.scale.y = 0.05
            marker.lifetime.sec = 0

            start_point = Point()
            end_point = Point()

            start_point.x = float(self.node_graph.nodes[start]['position']['x'])
            start_point.y = float(self.node_graph.nodes[start]['position']['y'])
            end_point.x = float(self.node_graph.nodes[end]['position']['x'])
            end_point.y = float(self.node_graph.nodes[end]['position']['y'])

            marker.points.append(start_point)
            marker.points.append(end_point)

            self.marker_array.markers.append(marker)
            self.visual_idx += 1

    def set_node(self):
        for idx in self.node_graph.nodes:
            node_marker = Marker()
            node_marker.header.frame_id = "map"
            node_marker.header.stamp = self.get_clock().now().to_msg()
            node_marker.id = self.visual_idx
            node_marker.type = Marker.SPHERE
            node_marker.action = Marker.ADD
            node_marker.pose.orientation.w = 1.0
            node_marker.pose.position.x = float(self.node_graph.nodes[idx]['position']['x'])
            node_marker.pose.position.y = float(self.node_graph.nodes[idx]['position']['y'])
            node_marker.scale.x = 0.1
            node_marker.scale.y = 0.1
            node_marker.scale.z = 0.1
            node_marker.color.r = 0.5
            node_marker.color.g = 0.5
            node_marker.color.b = 0.5
            node_marker.color.a = 0.8
            node_marker.lifetime.sec = 0

            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.id = self.visual_idx + 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.text = str(idx)
            text_marker.action = Marker.ADD
            text_marker.pose.orientation.w = 1.0
            text_marker.pose.position.x = float(self.node_graph.nodes[idx]['position']['x'])
            text_marker.pose.position.y = float(self.node_graph.nodes[idx]['position']['y'])
            text_marker.pose.position.z = 0.1
            text_marker.scale.z = 0.3
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            self.marker_array.markers.append(text_marker)
            self.marker_array.markers.append(node_marker)
            self.visual_idx += 2

    


def main():
    rclpy.init()
    waypoint_node_manager = NodeManager()
    rclpy.spin(waypoint_node_manager)

    waypoint_node_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Node Stopped") 