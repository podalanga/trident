#!/usr/bin/env python3
"""Zone A Controller - Line Following Node"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Bool
from geometry_msgs.msg import Twist
import json
import time

from trident_competition.pid_controller import PIDController, calculate_line_position_error, detect_end_square


class ZoneAController(Node):
    """ROS2 Node for Zone A line following control"""
    
    def __init__(self):
        super().__init__('zone_a_controller')
        
        # Parameters
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.3)
        self.declare_parameter('base_speed', 150)
        self.declare_parameter('max_speed', 200)
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        # Get parameters
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        self.base_speed = self.get_parameter('base_speed').value
        self.max_speed = self.get_parameter('max_speed').value
        update_rate = self.get_parameter('update_rate').value
        
        # Initialize PID controller
        self.pid = PIDController(kp, ki, kd)
        self.dt = 1.0 / update_rate
        
        # State machine
        self.state = 'idle'  # idle, waiting_start, line_following, aligning, complete
        self.sensor_values = [0] * 6
        
        # Publishers
        self.motor_cmd_pub = self.create_publisher(String, 'arduino/motor_command', 10)
        self.buzzer_cmd_pub = self.create_publisher(String, 'arduino/buzzer_command', 10)
        self.state_pub = self.create_publisher(String, 'zone_a/state', 10)
        
        # Subscribers
        self.ir_sub = self.create_subscription(
            Int32MultiArray,
            'arduino/ir_sensors',
            self.ir_callback,
            10
        )
        self.start_sub = self.create_subscription(
            Bool,
            'zone_a/start',
            self.start_callback,
            10
        )
        
        # Control loop timer
        self.timer = self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info('Zone A Controller initialized')
        self.get_logger().info(f'PID gains - Kp: {kp}, Ki: {ki}, Kd: {kd}')
        self.get_logger().info(f'Base speed: {self.base_speed}, Max speed: {self.max_speed}')
    
    def ir_callback(self, msg):
        """Callback for IR sensor data from Arduino"""
        self.sensor_values = msg.data
    
    def start_callback(self, msg):
        """Callback to start Zone A mission"""
        if msg.data and self.state == 'idle':
            self.get_logger().info('Zone A mission started')
            self.state = 'waiting_start'
    
    def check_start_square(self) -> bool:
        """Check if robot is positioned in start square (all sensors see black)"""
        threshold = 800
        return all(v > threshold for v in self.sensor_values)
    
    def control_loop(self):
        """Main control loop"""
        # Publish current state
        state_msg = String()
        state_msg.data = self.state
        self.state_pub.publish(state_msg)
        
        if self.state == 'waiting_start':
            # Wait for robot to be positioned in start square
            if self.check_start_square():
                self.get_logger().info('Start square detected - beginning line following')
                self.state = 'line_following'
                self.pid.reset()
        
        elif self.state == 'line_following':
            # Check for end square first
            if detect_end_square(self.sensor_values):
                self.get_logger().info('End square detected!')
                self.stop_motors()
                self.trigger_buzzer('single')
                self.state = 'aligning'
                return
            
            # Calculate line position error
            error = calculate_line_position_error(self.sensor_values)
            
            # Compute PID output
            pid_output = self.pid.compute(error, self.dt)
            
            # Calculate motor speeds
            left_speed = self.base_speed + pid_output
            right_speed = self.base_speed - pid_output
            
            # Clamp speeds
            left_speed = max(0, min(self.max_speed, left_speed))
            right_speed = max(0, min(self.max_speed, right_speed))
            
            # Send motor command
            self.send_motor_command(left_speed, right_speed, 'forward', 'forward')
        
        elif self.state == 'aligning':
            # Alignment procedure - move to right edge of square
            self.align_for_zone_b()
    
    def align_for_zone_b(self):
        """Align robot at right edge of black square for Zone B entry"""
        # Simple alignment: move slowly right until sensors detect edge
        # This is a simplified version - implement full alignment logic as needed
        
        # For now, just mark as complete after a short delay
        self.get_logger().info('Aligning for Zone B entry...')
        time.sleep(1.0)
        self.stop_motors()
        self.state = 'complete'
        self.get_logger().info('Zone A complete!')
    
    def send_motor_command(self, left_speed: float, right_speed: float, 
                          left_dir: str = 'forward', right_dir: str = 'forward'):
        """Send motor command to Arduino"""
        cmd = {
            'type': 'motor',
            'left_speed': int(left_speed),
            'right_speed': int(right_speed),
            'left_direction': left_dir,
            'right_direction': right_dir
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.motor_cmd_pub.publish(msg)
    
    def stop_motors(self):
        """Stop both motors"""
        self.send_motor_command(0, 0, 'forward', 'forward')
    
    def trigger_buzzer(self, pattern: str):
        """Trigger buzzer on Arduino"""
        cmd = {
            'type': 'buzzer',
            'pattern': pattern
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.buzzer_cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ZoneAController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motors()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
