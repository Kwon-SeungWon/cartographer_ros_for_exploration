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
        # Store logger as member variable and initialize only once
        self._logger = None
        self.pkg_path = get_package_share_directory("waypoint_node_manager")
        self.current_node_name = None
        self.node_list_path = None
        self.node_data = {}

        # Default test file settings
        if node_file_dir is None:
            node_file_dir = self.pkg_path
        if current_node is None:
            current_node = "test_nodes"

        self.update_current_node_path(node_file_dir, current_node)

        self.node_marker_pub_ = self.create_publisher(MarkerArray, "/web_marker_array", 1)
        self.path_marker_pub_ = self.create_publisher(MarkerArray, "/path_marker_array", 1)
        self.robot_marker_pub_ = self.create_publisher(MarkerArray, "/robot_marker_array", 1)
        self.marker_array = MarkerArray()
        self.path_marker_array = MarkerArray()
        self.robot_marker_array = MarkerArray()
        self.visual_idx = 0
        self.path_visual_idx = 0
        self.robot_visual_idx = 0

        # Initialize TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Robot position tracking variables
        self.robot_position = None
        self.current_node = None
        self.robot_update_timer = None

        self.node_graph = nx.DiGraph()
        # Only update graph if node_list_path is provided
        if self.node_list_path:
            self.update_graph(self.node_list_path)

        # Enable marker publishing
        self.make_ros_marker()
        self.create_timer(0.1, self.publish_ros_marker)

        # Create services
        self.path_planning_srv_ = self.create_service(
            AddTwoInts, 
            'path_planning', 
            self.path_planning_callback
        )
        
        # New service: return actual waypoint coordinates
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

        try:
            print("NodeManager initialized with services")
        except:
            pass
        
        # Start robot position tracking timer
        self.robot_update_timer = self.create_timer(0.5, self.update_robot_position)

    def path_planning_callback(self, request, response):
        """Path planning service callback"""
        start_node = request.a
        target_node = request.b
        
        try:
            waypoints = self.make_dijkstra_path(start_node, target_node)
            if waypoints:
                response.sum = len(waypoints)  # Return number of nodes in path
                try:
                    print(f"Path planning successful: {start_node} -> {target_node}, {len(waypoints)} waypoints")
                except:
                    pass
                
                # Visualize path (red)
                self.visualize_path(waypoints, path_color=(1.0, 0.0, 1.0, 0.5))
            else:
                response.sum = -1  # No path found
                try:
                    print(f"No path found: {start_node} -> {target_node}")
                except:
                    pass
        except Exception as e:
            response.sum = -1
            try:
                print(f"Path planning failed: {str(e)}")
            except:
                pass
        
        return response

    def get_waypoints_callback(self, request, response):
        """Service callback to return waypoint coordinates and node index path"""
        # Only update node file if request.node_file is provided
        if hasattr(request, 'node_file') and request.node_file:
            node_file_base = request.node_file.replace('.yaml', '')
            # Use the current path or fallback to package path
            path = self.pkg_path
            self.update_current_node_path(path, node_file_base)
            self.update_graph(self.node_list_path)
        
        # If start_node is empty, find the nearest node from the robot's current position
        if not request.start_node and self.robot_position:
            start_node = self.find_nearest_node(self.robot_position)
            if start_node is not None:
                try:
                    print(f"Auto-detected start node: {start_node} from robot position")
                except:
                    pass
            else:
                response.success = False
                response.message = "Could not find nearest node to robot position"
                response.node_path = []
                return response
        else:
            start_node = request.start_node
            
        goal_node = request.goal_node
        try:
            node_path = nx.dijkstra_path(self.node_graph, start_node, goal_node)
            waypoints = self.get_waypoints_nodexy(node_path)
            if waypoints:
                # Convert to PoseArray
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
                response.node_path = [int(idx) for idx in node_path]
                response.success = True
                response.message = f"Generated {len(waypoints)} waypoints from node {start_node} to {goal_node}, {len(waypoints)} waypoints"
                try:
                    print(f"Waypoints generated successfully: {start_node} -> {goal_node}, {len(waypoints)} waypoints")
                except:
                    pass
                self.visualize_path(waypoints, path_color=(0.0, 0.0, 1.0, 1.0))
            else:
                response.success = False
                response.message = f"No path found from node {start_node} to {goal_node}"
                response.node_path = []
                try:
                    print(f"No path found: {start_node} -> {goal_node}")
                except:
                    pass
        except Exception as e:
            response.success = False
            response.message = f"Failed to generate waypoints: {str(e)}"
            response.node_path = []
            try:
                print(f"Waypoint generation failed: {str(e)}")
            except:
                pass
        return response

    def load_graph_callback(self, request, response):
        """Graph load service callback"""
        try:
            if self.node_list_path and os.path.exists(self.node_list_path):
                self.update_graph(self.node_list_path)
                response.success = True
                response.message = f"Graph loaded from {self.node_list_path}"
                try:
                    print("Graph loaded successfully")
                except:
                    pass
            else:
                response.success = False
                response.message = f"Graph file not found: {self.node_list_path}"
                try:
                    print("Graph file not found")
                except:
                    pass
        except Exception as e:
            response.success = False
            response.message = f"Failed to load graph: {str(e)}"
            try:
                print(f"Failed to load graph: {str(e)}")
            except:
                pass
        
        return response

    def load_node_list(self, node_list_path):
        try:
            with open(node_list_path, 'r') as yaml_file:
                self.node_data = yaml.safe_load(yaml_file)
                if self.node_data is None:
                    self.node_data = {}
        except Exception as e:
            print(f"Node list file could not be loaded: {e}")
            self.node_data = {}

    def make_graph(self):
        self.node_graph.clear()

        for data in self.node_data.get("node", []):
            self.node_graph.add_node(data['index'], **data)

            if data.get('connection'):
                for item in data['connection']:
                    self.node_graph.add_edge(data['index'], item)

            if data.get('type') == 1 and data.get('child'):
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
            # If absolute path, keep it, otherwise base it on pkg_path
            if os.path.isabs(path):
                self.node_list_path = os.path.join(path, self.current_node_name)
            else:
                self.node_list_path = os.path.join(self.pkg_path, path, self.current_node_name)
        else:
            self.current_node_name = None
            self.node_list_path = None


    def update_graph(self, path):
        if path and os.path.exists(path):
            self.load_node_list(path)
            self.make_graph()
        else:
            print(f"Node list file not found: {path} (but continuing without it)")
            self.node_data = {}
            self.node_graph.clear()

    def make_dijkstra_path(self, start, target):
        try:
            path = nx.dijkstra_path(self.node_graph, start, target)
            waypoint_list = self.get_waypoints_nodexy(path)
            try:
                print(f"Dijkstra Path: {path}")
                print(f"Dijkstra waypoint_list: {waypoint_list}")
            except:
                pass
            return waypoint_list
            
        except nx.NetworkXNoPath:
            try:
                print("No path found")
            except:
                pass

    
    def get_waypoints_nodexy(self, shortest_path):
        waypoints = []
        
        for index in shortest_path:
            if index in self.node_graph.nodes:
                position = self.node_graph.nodes[index].get('position', None)
                if position:
                    waypoints.append(position)
            else:
                try:
                    print(f"Index {index} not found in graph!")
                except:
                    pass

        return waypoints
    
    def make_ros_marker(self):
        self.set_link()
        self.set_node()
        try:
            print("Marker Publishing Complete")
        except:
            pass

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
        """Initialize path markers"""
        self.path_marker_array = MarkerArray()
        self.path_visual_idx = 0

    def visualize_path(self, waypoints, path_color=(1.0, 1.0, 0.0, 1.0)):
        """Function to visualize path"""
        if not waypoints or len(waypoints) < 2:
            return
        try:
            self.clear_path_markers()
            # Create path line marker
            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.id = self.path_visual_idx
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.pose.orientation.w = 1.0
            path_marker.scale.x = 0.03  # Line thickness
            path_marker.color.r = path_color[0]
            path_marker.color.g = path_color[1]
            path_marker.color.b = path_color[2]
            path_marker.color.a = path_color[3]
            path_marker.lifetime.sec = 0

            # Add path points
            for wp in waypoints:
                point = Point()
                point.x = float(wp['x'])
                point.y = float(wp['y'])
                point.z = 0.0
                path_marker.points.append(point)

            self.path_marker_array.markers.append(path_marker)
            self.path_visual_idx += 1

            # Add arrow markers for each waypoint in the path
            for i, wp in enumerate(waypoints):
                if i < len(waypoints) - 1:  # Exclude last waypoint
                    next_wp = waypoints[i + 1]
                    arrow_marker = Marker()
                    arrow_marker.header.frame_id = "map"
                    arrow_marker.header.stamp = self.get_clock().now().to_msg()
                    arrow_marker.id = self.path_visual_idx
                    arrow_marker.type = Marker.ARROW
                    arrow_marker.action = Marker.ADD
                    arrow_marker.pose.orientation.w = 1.0
                    arrow_marker.scale.x = 0.05
                    arrow_marker.scale.y = 0.1
                    arrow_marker.color.r = path_color[0]
                    arrow_marker.color.g = path_color[1]
                    arrow_marker.color.b = path_color[2]
                    arrow_marker.color.a = path_color[3]
                    arrow_marker.lifetime.sec = 0

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

            # Start node marker (green cylinder)
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

            # Start node text label
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

            # End node marker (red cylinder)
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

            # End node text label
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
        except Exception as e:
            try:
                print(f"Error in visualize_path: {str(e)}")
            except:
                pass
            self.clear_path_markers()

    def update_robot_position(self):
        """Update robot's current position and find the nearest node"""
        try:
            # Get transform from map to base_footprint
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time()
            )
            
            # Update robot position
            self.robot_position = {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y
            }
            
            # Find the nearest node
            nearest_node = self.find_nearest_node(self.robot_position)
            if nearest_node != self.current_node:
                self.current_node = nearest_node
                try:
                    print(f"Robot moved to node: {nearest_node}")
                except:
                    pass
            
            # Update robot marker
            self.update_robot_marker()
            
        except Exception as e:
            # If transform cannot be found (robot hasn't started yet or)
            pass

    def find_nearest_node(self, position, max_distance=100.0):
        """Find the nearest node to a given position"""
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
        """Update the marker to visualize the robot's current position"""
        self.robot_marker_array = MarkerArray()
        self.robot_visual_idx = 0
        
        if self.robot_position:
            # Robot position marker (blue cylinder)
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
            
            # Robot label
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
            
            # If current node exists, highlight it
            if self.current_node is not None:
                self.highlight_current_node()

    def highlight_current_node(self):
        """Highlight the node where the robot is currently located"""
        if self.current_node and self.current_node in self.node_graph.nodes:
            node_pos = self.node_graph.nodes[self.current_node]['position']
            
            # Current node highlight marker (yellow cylinder)
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
            
            # Current node text
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