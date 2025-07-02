#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import networkx as nx
import math

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseArray
from std_srvs.srv import Trigger
from example_interfaces.srv import AddTwoInts
from san_msgs.srv import GetWaypoints
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

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
        self.path_marker_pub_ = self.create_publisher(MarkerArray, "/path_marker_array", 1)
        self.robot_marker_pub_ = self.create_publisher(MarkerArray, "/robot_marker_array", 1)
        self.marker_array = MarkerArray()
        self.path_marker_array = MarkerArray()
        self.robot_marker_array = MarkerArray()
        self.visual_idx = 0
        self.path_visual_idx = 0
        self.robot_visual_idx = 0

        # TF 리스너 초기화
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 로봇 위치 추적 변수
        self.robot_position = None
        self.current_node = None
        self.robot_update_timer = None

        self.node_graph = nx.DiGraph()
        # Only update graph if node_list_path is provided
        if self.node_list_path:
            self.update_graph(self.node_list_path)

        # 마커 발행 활성화
        self.make_ros_marker()
        self.create_timer(1.0, self.publish_ros_marker)

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
        
        # 로봇 위치 추적 타이머 시작
        self.robot_update_timer = self.create_timer(0.5, self.update_robot_position)

    def path_planning_callback(self, request, response):
        """경로 계획 서비스 콜백"""
        start_node = request.a
        target_node = request.b
        
        try:
            waypoints = self.make_dijkstra_path(start_node, target_node)
            if waypoints:
                response.sum = len(waypoints)  # 경로의 노드 수를 반환
                self.get_logger().info(f"Path planning successful: {start_node} -> {target_node}, {len(waypoints)} waypoints")
                
                # 경로 시각화 (빨간색)
                self.visualize_path(waypoints, path_color=(1.0, 0.0, 1.0, 0.5))
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
                response.message = f"Generated {len(waypoints)} waypoints from node {start_node} to {goal_node}, {len(waypoints)} waypoints"
                self.get_logger().info(f"Waypoints generated successfully: {start_node} -> {goal_node}, {len(waypoints)} waypoints")
                
                # 경로 시각화 (파란색)
                self.visualize_path(waypoints, path_color=(0.0, 0.0, 1.0, 1.0))
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
        self.path_marker_pub_.publish(self.path_marker_array)
        self.robot_marker_pub_.publish(self.robot_marker_array)
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

    def clear_path_markers(self):
        """경로 마커를 초기화"""
        self.path_marker_array = MarkerArray()
        self.path_visual_idx = 0

    def visualize_path(self, waypoints, path_color=(1.0, 1.0, 0.0, 1.0)):
        """경로를 시각화하는 함수"""
        self.clear_path_markers()
        
        if not waypoints or len(waypoints) < 2:
            return
        
        # 경로 라인 마커 생성
        path_marker = Marker()
        path_marker.header.frame_id = "map"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.id = self.path_visual_idx
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.pose.orientation.w = 1.0
        path_marker.scale.x = 0.03  # 선 두께
        path_marker.color.r = path_color[0]
        path_marker.color.g = path_color[1]
        path_marker.color.b = path_color[2]
        path_marker.color.a = path_color[3]
        path_marker.lifetime.sec = 0
        
        # 경로 포인트 추가
        for wp in waypoints:
            point = Point()
            point.x = float(wp['x'])
            point.y = float(wp['y'])
            point.z = 0.0
            path_marker.points.append(point)
        
        self.path_marker_array.markers.append(path_marker)
        self.path_visual_idx += 1
        
        # 경로의 각 waypoint에 화살표 마커 추가
        for i, wp in enumerate(waypoints):
            if i < len(waypoints) - 1:  # 마지막 waypoint 제외
                next_wp = waypoints[i + 1]
                
                # 화살표 마커 생성
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "map"
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.id = self.path_visual_idx
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.pose.orientation.w = 1.0
                arrow_marker.scale.x = 0.05  # 화살표 길이
                arrow_marker.scale.y = 0.1   # 화살표 두께
                arrow_marker.color.r = path_color[0]
                arrow_marker.color.g = path_color[1]
                arrow_marker.color.b = path_color[2]
                arrow_marker.color.a = path_color[3]
                arrow_marker.lifetime.sec = 0
                
                # 화살표 시작점과 끝점
                start_point = Point()
                start_point.x = float(wp['x'])
                start_point.y = float(wp['y'])
                start_point.z = 0.0
                
                end_point = Point()
                end_point.x = float(next_wp['x'])
                end_point.y = float(next_wp['y'])
                end_point.z = 0.0
                
                arrow_marker.points.append(start_point)
                arrow_marker.points.append(end_point)
                
                self.path_marker_array.markers.append(arrow_marker)
                self.path_visual_idx += 1
        
        # 시작 노드 마커 (초록색 원)
        start_wp = waypoints[0]
        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.id = self.path_visual_idx
        start_marker.type = Marker.CYLINDER
        start_marker.action = Marker.ADD
        start_marker.pose.orientation.w = 1.0
        start_marker.pose.position.x = float(start_wp['x'])
        start_marker.pose.position.y = float(start_wp['y'])
        start_marker.pose.position.z = 0.0
        start_marker.scale.x = 0.3
        start_marker.scale.y = 0.3
        start_marker.scale.z = 0.1
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        start_marker.color.a = 0.8
        start_marker.lifetime.sec = 0
        
        self.path_marker_array.markers.append(start_marker)
        self.path_visual_idx += 1
        
        # 시작 노드 텍스트 라벨
        start_text = Marker()
        start_text.header.frame_id = "map"
        start_text.header.stamp = self.get_clock().now().to_msg()
        start_text.id = self.path_visual_idx
        start_text.type = Marker.TEXT_VIEW_FACING
        start_text.text = "START"
        start_text.action = Marker.ADD
        start_text.pose.orientation.w = 1.0
        start_text.pose.position.x = float(start_wp['x'])
        start_text.pose.position.y = float(start_wp['y'])
        start_text.pose.position.z = 0.3
        start_text.scale.z = 0.4
        start_text.color.r = 0.0
        start_text.color.g = 1.0
        start_text.color.b = 0.0
        start_text.color.a = 1.0
        start_text.lifetime.sec = 0
        
        self.path_marker_array.markers.append(start_text)
        self.path_visual_idx += 1
        
        # 끝 노드 마커 (빨간색 원)
        end_wp = waypoints[-1]
        end_marker = Marker()
        end_marker.header.frame_id = "map"
        end_marker.header.stamp = self.get_clock().now().to_msg()
        end_marker.id = self.path_visual_idx
        end_marker.type = Marker.CYLINDER
        end_marker.action = Marker.ADD
        end_marker.pose.orientation.w = 1.0
        end_marker.pose.position.x = float(end_wp['x'])
        end_marker.pose.position.y = float(end_wp['y'])
        end_marker.pose.position.z = 0.0
        end_marker.scale.x = 0.3
        end_marker.scale.y = 0.3
        end_marker.scale.z = 0.1
        end_marker.color.r = 1.0
        end_marker.color.g = 0.0
        end_marker.color.b = 0.0
        end_marker.color.a = 0.8
        end_marker.lifetime.sec = 0
        
        self.path_marker_array.markers.append(end_marker)
        self.path_visual_idx += 1
        
        # 끝 노드 텍스트 라벨
        end_text = Marker()
        end_text.header.frame_id = "map"
        end_text.header.stamp = self.get_clock().now().to_msg()
        end_text.id = self.path_visual_idx
        end_text.type = Marker.TEXT_VIEW_FACING
        end_text.text = "GOAL"
        end_text.action = Marker.ADD
        end_text.pose.orientation.w = 1.0
        end_text.pose.position.x = float(end_wp['x'])
        end_text.pose.position.y = float(end_wp['y'])
        end_text.pose.position.z = 0.3
        end_text.scale.z = 0.4
        end_text.color.r = 1.0
        end_text.color.g = 0.0
        end_text.color.b = 0.0
        end_text.color.a = 1.0
        end_text.lifetime.sec = 0
        
        self.path_marker_array.markers.append(end_text)
        self.path_visual_idx += 1

    def update_robot_position(self):
        """로봇의 현재 위치를 업데이트하고 가장 가까운 노드를 찾습니다"""
        try:
            # map에서 base_footprint로의 변환 가져오기
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            
            # 로봇 위치 업데이트
            self.robot_position = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y
            }
            
            # 가장 가까운 노드 찾기
            nearest_node = self.find_nearest_node(self.robot_position)
            if nearest_node != self.current_node:
                self.current_node = nearest_node
                self.get_logger().info(f"Robot moved to node: {nearest_node}")
            
            # 로봇 마커 업데이트
            self.update_robot_marker()
            
        except Exception as e:
            # TF 변환을 찾을 수 없는 경우 (로봇이 아직 시작되지 않았거나)
            pass

    def find_nearest_node(self, position, max_distance=1.0):
        """주어진 위치에서 가장 가까운 노드를 찾습니다"""
        if not self.node_graph.nodes:
            return None
            
        min_distance = float('inf')
        nearest_node = None
        
        for node_id in self.node_graph.nodes:
            node_pos = self.node_graph.nodes[node_id].get('position', None)
            if node_pos:
                distance = math.sqrt(
                    (position['x'] - node_pos['x'])**2 + 
                    (position['y'] - node_pos['y'])**2
                )
                if distance < min_distance and distance <= max_distance:
                    min_distance = distance
                    nearest_node = node_id
        
        return nearest_node

    def update_robot_marker(self):
        """로봇의 현재 위치를 시각화하는 마커를 업데이트합니다"""
        self.robot_marker_array = MarkerArray()
        self.robot_visual_idx = 0
        
        if self.robot_position:
            # 로봇 위치 마커 (파란색 원)
            robot_marker = Marker()
            robot_marker.header.frame_id = "map"
            robot_marker.header.stamp = self.get_clock().now().to_msg()
            robot_marker.id = self.robot_visual_idx
            robot_marker.type = Marker.CYLINDER
            robot_marker.action = Marker.ADD
            robot_marker.pose.orientation.w = 1.0
            robot_marker.pose.position.x = self.robot_position['x']
            robot_marker.pose.position.y = self.robot_position['y']
            robot_marker.pose.position.z = 0.0
            robot_marker.scale.x = 0.4
            robot_marker.scale.y = 0.4
            robot_marker.scale.z = 0.1
            robot_marker.color.r = 0.0
            robot_marker.color.g = 0.0
            robot_marker.color.b = 1.0
            robot_marker.color.a = 0.8
            robot_marker.lifetime.sec = 0
            
            self.robot_marker_array.markers.append(robot_marker)
            self.robot_visual_idx += 1
            
            # 로봇 라벨
            robot_text = Marker()
            robot_text.header.frame_id = "map"
            robot_text.header.stamp = self.get_clock().now().to_msg()
            robot_text.id = self.robot_visual_idx
            robot_text.type = Marker.TEXT_VIEW_FACING
            robot_text.text = "ROBOT"
            robot_text.action = Marker.ADD
            robot_text.pose.orientation.w = 1.0
            robot_text.pose.position.x = self.robot_position['x']
            robot_text.pose.position.y = self.robot_position['y']
            robot_text.pose.position.z = 0.4
            robot_text.scale.z = 0.3
            robot_text.color.r = 0.0
            robot_text.color.g = 0.0
            robot_text.color.b = 1.0
            robot_text.color.a = 1.0
            robot_text.lifetime.sec = 0
            
            self.robot_marker_array.markers.append(robot_text)
            self.robot_visual_idx += 1
            
            # 현재 노드가 있으면 강조 표시
            if self.current_node is not None:
                self.highlight_current_node()

    def highlight_current_node(self):
        """현재 로봇이 위치한 노드를 강조 표시합니다"""
        if self.current_node and self.current_node in self.node_graph.nodes:
            node_pos = self.node_graph.nodes[self.current_node]['position']
            
            # 현재 노드 강조 마커 (노란색 원)
            current_node_marker = Marker()
            current_node_marker.header.frame_id = "map"
            current_node_marker.header.stamp = self.get_clock().now().to_msg()
            current_node_marker.id = self.robot_visual_idx
            current_node_marker.type = Marker.CYLINDER
            current_node_marker.action = Marker.ADD
            current_node_marker.pose.orientation.w = 1.0
            current_node_marker.pose.position.x = float(node_pos['x'])
            current_node_marker.pose.position.y = float(node_pos['y'])
            current_node_marker.pose.position.z = 0.0
            current_node_marker.scale.x = 0.5
            current_node_marker.scale.y = 0.5
            current_node_marker.scale.z = 0.1
            current_node_marker.color.r = 1.0
            current_node_marker.color.g = 1.0
            current_node_marker.color.b = 0.0
            current_node_marker.color.a = 0.6
            current_node_marker.lifetime.sec = 0
            
            self.robot_marker_array.markers.append(current_node_marker)
            self.robot_visual_idx += 1
            
            # 현재 노드 텍스트
            current_node_text = Marker()
            current_node_text.header.frame_id = "map"
            current_node_text.header.stamp = self.get_clock().now().to_msg()
            current_node_text.id = self.robot_visual_idx
            current_node_text.type = Marker.TEXT_VIEW_FACING
            current_node_text.text = f"CURRENT: {self.current_node}"
            current_node_text.action = Marker.ADD
            current_node_text.pose.orientation.w = 1.0
            current_node_text.pose.position.x = float(node_pos['x'])
            current_node_text.pose.position.y = float(node_pos['y'])
            current_node_text.pose.position.z = 0.5
            current_node_text.scale.z = 0.4
            current_node_text.color.r = 1.0
            current_node_text.color.g = 1.0
            current_node_text.color.b = 0.0
            current_node_text.color.a = 1.0
            current_node_text.lifetime.sec = 0
            
            self.robot_marker_array.markers.append(current_node_text)
            self.robot_visual_idx += 1

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