#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
import yaml
import json
import os
import numpy as np
from PIL import Image

class SemanticPlaceVisualizer(Node):
    def __init__(self):
        super().__init__('semantic_place_visualizer')
        
        # 마커 배열 퍼블리셔 생성
        self.marker_pub = self.create_publisher(MarkerArray, 'semantic_places', 10)
        
        # Semantic place subscriber 생성
        self.semantic_place_sub = self.create_subscription(
            String,
            'vda5050/semantic_place',
            self.semantic_place_callback,
            10
        )
        
        # 색상 매핑 정의
        self.color_map = {
            "red": ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7),
            "green": ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7),
            "blue": ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7),
            "yellow": ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.7),
            "magenta": ColorRGBA(r=1.0, g=0.0, b=1.0, a=0.7),
            "cyan": ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.7),
            "orange": ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.7),
            "purple": ColorRGBA(r=0.5, g=0.0, b=0.5, a=0.7),
            "pink": ColorRGBA(r=1.0, g=0.75, b=0.8, a=0.7),
            "brown": ColorRGBA(r=0.6, g=0.4, b=0.2, a=0.7),
            "gray": ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.7),
            "white": ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.7),
            "black": ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.7)
        }
        
        # 현재 semantic place 저장
        self.current_semantic_place = None
        
        self.get_logger().info('Semantic Place Visualizer started')
    
    def semantic_place_callback(self, msg):
        """Semantic place 메시지를 받아서 처리"""
        try:
            data = json.loads(msg.data)
            if "semanticPlace" in data:
                self.current_semantic_place = data["semanticPlace"]
                self.get_logger().info(f'Received semantic place: {self.current_semantic_place["name"]}')
                self.publish_current_semantic_place()
            else:
                self.get_logger().warn('No semanticPlace in message')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing semantic place: {e}')
    
    def get_color_from_string(self, color_string):
        """색상 문자열을 ColorRGBA로 변환"""
        color_string = color_string.lower()
        if color_string in self.color_map:
            return self.color_map[color_string]
        else:
            # 기본 색상 (빨강)
            return ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.7)
    
    def create_polygon_marker(self, place, marker_id):
        """Polygon 마커 생성 (선)"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_semantic_place"
        marker.id = marker_id * 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # 색상 설정 (JSON에서 읽어온 색상 사용)
        color_string = place.get("color", "red")
        marker.color = self.get_color_from_string(color_string)
        
        # 선 두께 설정
        marker.scale.x = 0.2  # 선 두께를 더 굵게
        
        # 포인트 설정
        points = []
        for point_data in place["pose"]:
            point = Point()
            point.x = float(point_data["x"])
            point.y = float(point_data["y"])
            point.z = 0.0
            points.append(point)
        
        # 폐곡선을 만들기 위해 첫 번째 점을 마지막에 추가
        if len(points) > 0:
            points.append(points[0])
        
        marker.points = points
        
        return marker
    
    def create_filled_polygon_marker(self, place, marker_id):
        """채워진 Polygon 마커 생성 (내부)"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_semantic_place_filled"
        marker.id = marker_id * 2 + 1
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # 색상 설정 (JSON에서 읽어온 색상 사용, 반투명)
        color_string = place.get("color", "red")
        color = self.get_color_from_string(color_string)
        color.a = 0.4  # 더 투명하게
        marker.color = color
        
        # 크기 설정
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.01
        
        # 포인트 설정 - 폴리곤 내부를 격자로 채움
        points = []
        x_coords = [float(p["x"]) for p in place["pose"]]
        y_coords = [float(p["y"]) for p in place["pose"]]
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # 격자 간격
        step = 0.1
        
        # 폴리곤 내부를 격자로 채움
        for x in np.arange(min_x, max_x, step):
            for y in np.arange(min_y, max_y, step):
                # 점이 폴리곤 내부에 있는지 확인
                if self.is_point_in_polygon(x, y, place["pose"]):
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = 0.01
                    points.append(point)
        
        marker.points = points
        
        return marker
    
    def is_point_in_polygon(self, x, y, polygon_points):
        """점이 폴리곤 내부에 있는지 확인 (ray casting algorithm)"""
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
    
    def create_text_marker(self, place, marker_id):
        """텍스트 마커 생성 (place 이름)"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "current_semantic_place_name"
        marker.id = marker_id * 2 + 2
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # 텍스트 설정
        marker.text = place["name"]
        
        # 색상 설정 (검은색으로 고정)
        marker.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0)
        
        # 크기 설정
        marker.scale.z = 0.5  # 텍스트 크기를 더 크게
        
        # 위치 설정 (polygon의 중심점 계산)
        if len(place["pose"]) > 0:
            x_coords = [float(p["x"]) for p in place["pose"]]
            y_coords = [float(p["y"]) for p in place["pose"]]
            
            center_x = sum(x_coords) / len(x_coords)
            center_y = sum(y_coords) / len(y_coords)
            
            marker.pose.position.x = center_x
            marker.pose.position.y = center_y
            marker.pose.position.z = 0.2  # 지도 위에 더 올림
        
        return marker
    
    def publish_current_semantic_place(self):
        """현재 semantic place를 마커로 발행"""
        if not self.current_semantic_place:
            return
        
        marker_array = MarkerArray()
        
        # 채워진 Polygon 마커 생성 (내부)
        filled_marker = self.create_filled_polygon_marker(self.current_semantic_place, 0)
        marker_array.markers.append(filled_marker)
        
        # 선 Polygon 마커 생성
        line_marker = self.create_polygon_marker(self.current_semantic_place, 0)
        marker_array.markers.append(line_marker)
        
        # 텍스트 마커 생성
        text_marker = self.create_text_marker(self.current_semantic_place, 0)
        marker_array.markers.append(text_marker)
        
        # 마커 배열 발행
        self.marker_pub.publish(marker_array)
        
        self.get_logger().debug(f'Published current semantic place: {self.current_semantic_place["name"]}')

def main(args=None):
    rclpy.init(args=args)
    
    visualizer = SemanticPlaceVisualizer()
    
    try:
        rclpy.spin(visualizer)
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 