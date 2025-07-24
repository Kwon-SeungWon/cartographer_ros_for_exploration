#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class VDA5050Tester(Node):
    def __init__(self):
        super().__init__('vda5050_tester')

        self.publisher_ = self.create_publisher(String, 'vda5050/instantactions', 10)

        self.subscription = self.create_subscription(
            String,
            'vda5050/instantactions/response',
            self.result_callback,
            10
        )

        self.create_timer(1.0, self.publish_test_json)
        self.request_sent = False

    def publish_test_json(self):
        if self.request_sent:
            return
        
        # request_json = {
        #     "headerId": 1,
        #     "timestamp": "2025-02-11T12:34:56.78Z",
        #     "version": "0.0.0",
        #     "manufacturer": "CASELAB",
        #     "serialNumber": "SERIALNUMBER",
        #     "actions": [
        #         {
        #             "actionId": "buildMap",
        #             "actionType": "buildMap",
        #             "actionDescription": "request",
        #             "blockingType": "SOFT",
        #             "actionParameters": [
        #                 {
        #                     "key": "signal",
        #                     "value": ["3", "new_map", "new_map"]
        #                 }
        #             ]
        #         }
        #     ]
        # }

        # request_json = {
        #     "headerId": 1,
        #     "timestamp": "2025-02-11T12:34:56.78Z",
        #     "version": "0.0.0",
        #     "manufacturer": "CASELAB",
        #     "serialNumber": "SERIALNUMBER",
        #     "actions": [
        #         {
        #             "actionId": "controlMap",
        #             "actionType": "controlMap",
        #             "actionDescription": "request",
        #             "blockingType": "SOFT",
        #             "actionParameters": [
        #                 {
        #                     "key": "signal",
        #                     "value": ["1", "map_name", "map_id"]
        #                 }
        #             ]
        #         }
        #     ]
        # }

        # request_json = {
        #     "headerId": 1,
        #     "timestamp": "2025-02-11T12:34:56.78Z",
        #     "version": "0.0.0",
        #     "manufacturer": "CASELAB",
        #     "serialNumber": "SERIALNUMBER",
        #     "actions": [
        #         {
        #             "actionId": "controlNodelist",
        #             "actionType": "controlNodelist",
        #             "actionDescription": "request",
        #             "blockingType": "SOFT",
        #             "actionParameters": [
        #                 {
        #                     "key": "signal",
        #                     "value": ["1", "map_name", "map_id"]
        #                 }
        #             ]
        #         }
        #     ]
        # }

        request_json = {
            "headerId": 1,
            "timestamp": "2025-02-11T12:34:56.78Z",
            "version": "0.0.0",
            "manufacturer": "CASELAB",
            "serialNumber": "SERIALNUMBER",
            "actions": [
                {
                    "actionId": "controlManipulator",
                    "actionType": "controlManipulator",
                    "actionDescription": "request",
                    "blockingType": "SOFT",
                    "actionParameters": [
                        {
                            "key": "signal",
                            "value": ["1","1.0","2.0","3.0","4.0","5.0","6.0"]
                        }
                    ]
                }
            ]
        }

        # request_json = {
        #     "headerId": 1,
        #     "timestamp": "2025-02-11T12:34:56.78Z",
        #     "version": "0.0.0",
        #     "manufacturer": "CASELAB",
        #     "serialNumber": "SERIALNUMBER",
        #     "actions": [
        #         {
        #             "actionId": "setMapNode",
        #             "actionType": "setMapNode",
        #             "actionDescription": "request",
        #             "blockingType": "SOFT",
        #             "actionParameters": [
        #                 {
        #                     "key": "signal",
        #                     "value": ["1111111","2222222"]
        #                 }
        #             ]
        #         }
        #     ]
        # }

        msg = String()
        msg.data = json.dumps(request_json)
        self.publisher_.publish(msg)
        self.get_logger().info('Published test request to vda5050/instantactions')
        self.request_sent = True

    def result_callback(self, msg):
        self.get_logger().info('Received result from vda5050/instantactions/response:')
        self.get_logger().info(msg.data)

        
        self.get_logger().info('Shutting down node after response received.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = VDA5050Tester()
    rclpy.spin(node)