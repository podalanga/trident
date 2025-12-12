#!/usr/bin/env python3
"""Arduino Interface - Serial Communication Bridge"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import serial
import json
import threading
from typing import Optional


class ArduinoInterface(Node):
    """ROS2 node for Arduino serial communication"""
    
    def __init__(self):
        super().__init__('arduino_interface')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 1.0)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('timeout').value
        
        # Serial connection
        self.serial_port: Optional[serial.Serial] = None
        try:
            self.serial_port = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=timeout
            )
            self.get_logger().info(f'Connected to Arduino on {serial_port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            return
        
        # Publishers (Arduino -> ROS)
        self.ir_sensor_pub = self.create_publisher(Int32MultiArray, 'arduino/ir_sensors', 10)
        self.gripper_feedback_pub = self.create_publisher(String, 'arduino/gripper_feedback', 10)
        
        # Subscribers (ROS -> Arduino)
        self.motor_cmd_sub = self.create_subscription(
            String, 'arduino/motor_command', self.motor_command_callback, 10)
        self.gripper_cmd_sub = self.create_subscription(
            String, 'arduino/gripper_command', self.gripper_command_callback, 10)
        self.buzzer_cmd_sub = self.create_subscription(
            String, 'arduino/buzzer_command', self.buzzer_command_callback, 10)
        
        # Start serial reading thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()
        
        self.get_logger().info('Arduino interface ready')
    
    def motor_command_callback(self, msg):
        """Handle motor command from ROS"""
        try:
            cmd_data = json.loads(msg.data)
            self.send_to_arduino(cmd_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid motor command JSON: {e}')
    
    def gripper_command_callback(self, msg):
        """Handle gripper command from ROS"""
        try:
            cmd_data = json.loads(msg.data)
            self.send_to_arduino(cmd_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid gripper command JSON: {e}')
    
    def buzzer_command_callback(self, msg):
        """Handle buzzer command from ROS"""
        try:
            cmd_data = json.loads(msg.data)
            self.send_to_arduino(cmd_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid buzzer command JSON: {e}')
    
    def send_to_arduino(self, data: dict):
        """Send JSON command to Arduino"""
        if self.serial_port is None or not self.serial_port.is_open:
            self.get_logger().error('Serial port not open')
            return
        
        try:
            json_str = json.dumps(data) + '\n'
            self.serial_port.write(json_str.encode('utf-8'))
            self.serial_port.flush()
        except Exception as e:
            self.get_logger().error(f'Failed to send to Arduino: {e}')
    
    def read_serial_loop(self):
        """Continuous serial reading thread"""
        while self.running and rclpy.ok():
            try:
                if self.serial_port and self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    
                    if line:
                        try:
                            data = json.loads(line)
                            self.process_arduino_data(data)
                        except json.JSONDecodeError:
                            self.get_logger().warn(f'Invalid JSON from Arduino: {line}')
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')
    
    def process_arduino_data(self, data: dict):
        """Process data received from Arduino"""
        msg_type = data.get('type', '')
        
        if msg_type == 'ir_sensors':
            # Publish IR sensor data
            msg = Int32MultiArray()
            msg.data = data.get('values', [0] * 6)
            self.ir_sensor_pub.publish(msg)
        
        elif msg_type == 'gripper_feedback':
            # Publish gripper feedback
            msg = String()
            msg.data = json.dumps(data)
            self.gripper_feedback_pub.publish(msg)
        
        else:
            self.get_logger().warn(f'Unknown message type from Arduino: {msg_type}')
    
    def destroy_node(self):
        """Clean up on node shutdown"""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
