#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import networkx as nx

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class NodeManager(Node):
    def __init__(self, node_file_dir=None, current_node=None):
        super().__init__("node_manager")
        self.pkg_path = get_package_share_directory("webserver_interface_ros2")
        self.current_node_name = None
        self.node_list_path = None

        self.update_current_node_path(node_file_dir, current_node)

        self.node_data = {}

        self.node_marker_pub_ = self.create_publisher(MarkerArray, "/web_marker_array", 1)
        self.marker_array = MarkerArray()
        self.visual_idx = 0

        self.node_graph = nx.DiGraph()
        self.update_graph(self.node_list_path)

        # self.make_ros_marker()
        # self.create_timer(1.0, self.publish_ros_marker)
        # self.make_dijkstra_path(777, 3)

    # def load_node_list(self, node_list_path):
    #     with open(node_list_path, 'r') as yaml_file:
    #         self.node_data = yaml.safe_load(yaml_file)

    def load_node_list(self, node_list_path):
        try:
            with open(node_list_path, 'r') as yaml_file:
                self.node_data = yaml.safe_load(yaml_file)
                return self.node_data
        except FileNotFoundError:
            print(f"Error: yaml file not found: {node_list_path}")
            return {}
        except yaml.YAMLError as e:
            print(f"Error: YAML read error: {e}")
            return {}

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
        self.current_node_name = current_node_name + ".yaml"
        self.node_list_path = path + "/" + self.current_node_name


    def update_graph(self, path):
        if self.load_node_list(path)=={}:
            return

        self.make_graph()

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
    node_manager = NodeManager()
    rclpy.spin(node_manager)

    node_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Node Stopped")