#!/usr/bin/env python3
"""Zone A Controller - Line Following Node"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Int32MultiArray, Bool
from geometry_msgs.msg import Twist
import json
import time

from trident_competition.pid_controller import PIDController, calculate_line_position_error, detect_end_square


class ZoneAController(Node):
    """ROS2 Node for Zone A line following control"""
    
    def __init__(self):
        super().__init__('zone_a_controller')
        
        # Parameters - Updated based on test_firmware calibration
        self.declare_parameter('kp', 45.0)  # Increased for sharper turns
        self.declare_parameter('ki', 0.0)   # Usually 0 for simple line following
        self.declare_parameter('kd', 12.0)  # Added to predict error and reduce oscillation
        self.declare_parameter('base_speed', 100)  # Safe starting speed
        self.declare_parameter('max_speed', 255)   # Cap speed to prevent runaways
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
        
        # Calibration constants from test_firmware
        self.cal_min = [44, 42, 44, 43, 43, 41]
        self.cal_max = [335, 314, 439, 394, 374, 237]
        
        # State machine
        self.state = 'idle'  # idle, waiting_start, line_following, aligning, complete
        self.sensor_values = [0] * 6
        self.calibrated_values = [0] * 6
        self.previous_error = 0.0
        
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
        
        # Add parameter callback for runtime tuning
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Zone A Controller initialized')
        self.get_logger().info(f'PID gains - Kp: {kp}, Ki: {ki}, Kd: {kd}')
        self.get_logger().info(f'Base speed: {self.base_speed}, Max speed: {self.max_speed}')
        self.get_logger().info('Runtime parameter tuning enabled')
    
    def ir_callback(self, msg):
        """Callback for IR sensor data from Arduino"""
        self.sensor_values = msg.data
        # Calibrate sensor values based on test_firmware logic
        self.calibrate_sensors()
    
    def start_callback(self, msg):
        """Callback to start Zone A mission"""
        if msg.data and self.state == 'idle':
            self.get_logger().info('Zone A mission started')
            self.state = 'waiting_start'
    
    def parameter_callback(self, params):
        """Handle runtime parameter changes"""
        for param in params:
            if param.name == 'kp':
                self.pid.kp = param.value
                self.get_logger().info(f'Updated Kp to {param.value}')
            elif param.name == 'ki':
                self.pid.ki = param.value
                self.get_logger().info(f'Updated Ki to {param.value}')
            elif param.name == 'kd':
                self.pid.kd = param.value
                self.get_logger().info(f'Updated Kd to {param.value}')
            elif param.name == 'base_speed':
                self.base_speed = param.value
                self.get_logger().info(f'Updated base speed to {param.value}')
            elif param.name == 'max_speed':
                self.max_speed = param.value
                self.get_logger().info(f'Updated max speed to {param.value}')
            elif param.name == 'update_rate':
                # Update rate requires timer recreation
                self.dt = 1.0 / param.value
                self.get_logger().info(f'Updated control rate to {param.value} Hz')
        
        return SetParametersResult(successful=True)
    
    def calibrate_sensors(self):
        """Calibrate raw sensor values to 0-1000 range based on test_firmware"""
        for i in range(6):
            if i < len(self.sensor_values):
                raw = self.sensor_values[i]
                # Map raw reading to 0-1000 based on calibration
                calibrated = self.map_value(raw, self.cal_min[i], self.cal_max[i], 0, 1000)
                self.calibrated_values[i] = max(0, min(1000, calibrated))
    
    def map_value(self, value, from_low, from_high, to_low, to_high):
        """Map a value from one range to another (Arduino map equivalent)"""
        return (value - from_low) * (to_high - to_low) // (from_high - from_low) + to_low
    
    def calculate_calibrated_line_error(self):
        """Calculate line position error using calibrated values from test_firmware"""
        weighted_sum = 0
        total_value = 0
        
        # Standard Weighted Average
        for i in range(6):
            weighted_sum += self.calibrated_values[i] * i * 1000  # Multiply by 1000 for precision
            total_value += self.calibrated_values[i]
        
        # Dead reckoning: If we lose the line, keep turning in the direction of the last known error
        if total_value < 500:
            if self.previous_error > 0:
                return 3.0  # Hard right
            if self.previous_error < 0:
                return -3.0  # Hard left
            return 0.0
        
        # Result is 0 to 5000, Center is 2500
        position = weighted_sum / total_value
        
        # Map to -2.5 to +2.5 for easier PID math
        error = (position - 2500) / 1000.0
        self.previous_error = error
        
        return error
    
    def check_start_square(self) -> bool:
        """Check if robot is positioned in start square (all sensors see black)"""
        threshold = 800
        return all(v > threshold for v in self.calibrated_values)
    
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
            # Check for end square first (all sensors detect black)
            if self.detect_end_square():
                self.get_logger().info('End square detected!')
                self.stop_motors()
                self.trigger_buzzer('single')
                self.state = 'aligning'
                return
            
            # Calculate line position error using calibrated values
            error = self.calculate_calibrated_line_error()
            
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
    
    def detect_end_square(self):
        """Detect end square using calibrated sensor values"""
        threshold = 800  # High threshold for black detection
        return all(v > threshold for v in self.calibrated_values)
    
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
