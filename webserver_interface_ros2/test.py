#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

import json


class JsonPublisher(Node):
    def __init__(self):
        super().__init__("json_publisher")

        self.publisher_ = self.create_publisher(String, "/vda5050/order", 1)

        self.timer = self.create_timer(3.0, self.publish_json)

    def publish_json(self):
        """Python dict → JSON string 변환 후 발행"""
        order_data = {
            "headerId": 1,
            "timestamp": "2025-02-11T12:34:56.78Z",
            "version": "1.0.0",
            "manufacturer": "AGV Corp",
            "serialNumber": "AGV123456",
            "orderId": "ORD98765",
            "orderUpdateId": 42,
            "nodes": [
                {
                    "nodeId": "node_1",
                    "sequenceId": 1,
                    "released": True,
                    "actions": [],
                    "nodePosition": {
                        "x": 10.5,
                        "y": 20.3,
                        "mapId": "map_123"
                    }
                }
            ],
            "edges": [
                {
                    "edgeId": "edge_1",
                    "sequenceId": 1,
                    "released": True,
                    "startNodeId": "node_1",
                    "endNodeId": "node_2",
                    "actions": []
                }
            ]
        }

        msg = String()
        msg.data = json.dumps(order_data)  # dict → JSON String 변환
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published Order Data: {msg.data}")

def main():
    rclpy.init()
    node = JsonPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()