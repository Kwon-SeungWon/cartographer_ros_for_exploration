#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import threading
import time
from amr_serial.srv import GetMode
from amr_serial.msg import Joystick
from geometry_msgs.msg import Twist

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Declare parameters
        self.declare_parameter('port_name', '/dev/ttySERIAL')
        self.declare_parameter('baud_rate', 19200)
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 0.5)

        # Get parameters
        port_name = self.get_parameter('port_name').value
        baud_rate = self.get_parameter('baud_rate').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Initialize serial port
        try:
            self.serial_port = serial.Serial(port_name, baud_rate, timeout=0.01)
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return

        # Publishers
        self.joystick_pub = self.create_publisher(Joystick, 'joystick_states', 10)
        self.joy_vel_pub = self.create_publisher(Twist, 'joy_vel', 10)

        # Service
        self.service = self.create_service(GetMode, 'get_select_switch_status', self.get_select_switch_status)
        
        self.lock = threading.Lock()
        self.keep_running = True

        # Timer for publishing GPIO states
        self.timer = self.create_timer(0.05, self.publish_gpio_states)  # 20 Hz

    def __del__(self):
        self.keep_running = False
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()

    def read_gpio(self, gpio_num):
        with self.lock:
            try:
                command = f"gpio read {gpio_num}\r"
                self.serial_port.write(command.encode())

                response = self.serial_port.read(25).decode()

                if len(response) >= 4 and response[-4] == "1":
                    return "1"
                elif len(response) >= 4 and response[-4] == "0":
                    return "0"
                else:
                    return f"Unexpected response from device for GPIO {gpio_num}: {response}"

            except serial.SerialException as e:
                return f"Error opening or communicating with serial port for GPIO {gpio_num}: {e}"

    def get_select_switch_status(self, request, response):
        try:
            gpio0_status = self.read_gpio(0)
            gpio1_status = self.read_gpio(1)

            joystick_mode = (gpio0_status == "0")
            autonomous_mode = (gpio1_status == "0")

            # self.get_logger().info(f"GPIO 0 status: {gpio0_status}, GPIO 1 status: {gpio1_status}")
            # self.get_logger().info(f"Joystick mode: {joystick_mode}, Autonomous mode: {autonomous_mode}")

            response.joystick = joystick_mode
            response.autonomous = autonomous_mode
            return response
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            response.joystick = False
            response.autonomous = False
            return response

    def publish_gpio_states(self):
        if not self.keep_running:
            return

        try:
            # Check if joystick mode is active
            gpio0_status = self.read_gpio(0)
            joystick_mode = (gpio0_status == "0")

            if joystick_mode:
                # Read joystick GPIO states
                up = (self.read_gpio(2) == "0")
                down = (self.read_gpio(3) == "0")
                right = (self.read_gpio(4) == "0")
                left = (self.read_gpio(5) == "0")

                # Publish joystick states
                joystick_msg = Joystick()
                joystick_msg.up = up
                joystick_msg.down = down
                joystick_msg.right = right
                joystick_msg.left = left
                # self.joystick_pub.publish(joystick_msg)

                # Publish joy_vel based on GPIO states
                joy_vel_msg = Twist()
                
                # Forward/Backward
                if up and not down:
                    joy_vel_msg.linear.x = self.linear_speed
                elif down and not up:
                    joy_vel_msg.linear.x = -self.linear_speed
                else:
                    joy_vel_msg.linear.x = 0.0

                # Left/Right turn
                if left and not right:
                    joy_vel_msg.angular.z = self.angular_speed
                elif right and not left:
                    joy_vel_msg.angular.z = -self.angular_speed
                else:
                    joy_vel_msg.angular.z = 0.0

                self.joy_vel_pub.publish(joy_vel_msg)

                # self.get_logger().info(f"Joystick states - Up: {up}, Down: {down}, Right: {right}, Left: {left}")
                # self.get_logger().info(f"Joy vel - Linear: {joy_vel_msg.linear.x:.2f}, Angular: {joy_vel_msg.angular.z:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = SerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

