#!/usr/bin/env python3
"""Zone B Controller - VSLAM Navigation and Object Manipulation"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import numpy as np
import json
import time
from typing import Dict, List, Optional

from trident_competition.vision_detection import ColorDetector, QRDetector


class ZoneBController(Node):
    """ROS2 Node for Zone B VSLAM navigation and object manipulation"""
    
    def __init__(self):
        super().__init__('zone_b_controller')
        
        # Parameters
        self.declare_parameter('rotation_speed', 0.1)  # rad/s for 360° scan
        self.declare_parameter('approach_distance_qr', 0.2)  # meters (200mm)
        self.declare_parameter('approach_distance_drop', 0.1)  # meters (100mm)
        self.declare_parameter('navigation_tolerance', 0.05)  # meters (50mm)
        self.declare_parameter('orientation_tolerance', 0.087)  # radians (~5 degrees)
        self.declare_parameter('pose_topic', '/rtabmap/localization_pose')
        
        # Get parameters
        self.rotation_speed = self.get_parameter('rotation_speed').value
        self.approach_distance_qr = self.get_parameter('approach_distance_qr').value
        self.approach_distance_drop = self.get_parameter('approach_distance_drop').value
        self.nav_tolerance = self.get_parameter('navigation_tolerance').value
        self.orient_tolerance = self.get_parameter('orientation_tolerance').value
        pose_topic = self.get_parameter('pose_topic').value
        
        # State machine
        self.state = 'idle'  # idle, initializing_vslam, mapping, detecting_colors, 
                            # detecting_qrs, executing_mission, complete
        self.current_qr = None  # 'QR1', 'QR2', or 'QR3'
        self.mission_phase = None  # 'navigate_to_qr', 'align_pickup', 'pickup', 
                                   # 'navigate_to_drop', 'align_drop', 'drop'
        
        # Vision components
        self.bridge = CvBridge()
        self.camera_intrinsics = None
        self.color_detector = None
        self.qr_detector = None
        
        # Image data
        self.rgb_image = None
        self.depth_image = None
        
        # Arena map data
        self.east_wall_colors = {}  # {ColorPos1: {color, position}, ...}
        self.north_wall_qrs = {}    # {QR1: {text, position}, ...}
        self.qr_to_drop_mapping = {}
        
        # VSLAM data
        self.current_pose = None
        self.vslam_initialized = False
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.gripper_cmd_pub = self.create_publisher(String, 'arduino/gripper_command', 10)
        self.buzzer_cmd_pub = self.create_publisher(String, 'arduino/buzzer_command', 10)
        self.state_pub = self.create_publisher(String, 'zone_b/state', 10)
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, 'pose_topicself.camera_info_callback, 10)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/rtabmap/localization_pose', self.pose_callback, 10)
        self.start_sub = self.create_subscription(
            Bool, 'zone_b/start', self.start_callback, 10)
        
        # Control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Add parameter callback for runtime tuning
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        self.get_logger().info('Zone B Controller initialized')
        self.get_logger().info('Runtime parameter tuning enabled')
    
    def rgb_callback(self, msg):
        """Callback for RGB image"""
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
    
    def depth_callback(self, msg):
        """Callback for depth image"""
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
    
    def camera_info_callback(self, msg):
        """Callback for camera intrinsics"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5]
            }
            # Initialize vision detectors
            self.color_detector = ColorDetector(self.camera_intrinsics)
            self.qr_detector = QRDetector(self.camera_intrinsics)
            self.get_logger().info('Camera intrinsics received')
    
    def pose_callback(self, msg):
        """Callback for VSLAM pose"""
        self.current_pose = msg
        if not self.vslam_initialized:
            self.vslam_initialized = True
            self.get_logger().info('VSLAM initialized')
    
    def start_callback(self, msg):
        """Callback to start Zone B mission"""
        if msg.data and self.state == 'idle':
            self.get_logger().info('Zone B mission started')
            self.state = 'initializing_vslam'
    
    def parameter_callback(self, params):
        """Handle runtime parameter changes"""
        for param in params:
            if param.name == 'rotation_speed':
                self.rotation_speed = param.value
                self.get_logger().info(f'Updated rotation speed to {param.value} rad/s')
            elif param.name == 'approach_distance_qr':
                self.approach_distance_qr = param.value
                self.get_logger().info(f'Updated QR approach distance to {param.value} m')
            elif param.name == 'approach_distance_drop':
                self.approach_distance_drop = param.value
                self.get_logger().info(f'Updated drop approach distance to {param.value} m')
            elif param.name == 'navigation_tolerance':
                self.nav_tolerance = param.value
                self.get_logger().info(f'Updated navigation tolerance to {param.value} m')
            elif param.name == 'orientation_tolerance':
                self.orient_tolerance = param.value
                self.get_logger().info(f'Updated orientation tolerance to {param.value} rad')
        
        return SetParametersResult(successful=True)
    
    def control_loop(self):
        """Main control loop"""
        # Publish current state
        state_msg = String()
        state_msg.data = f'{self.state}_{self.mission_phase}' if self.mission_phase else self.state
        self.state_pub.publish(state_msg)
        
        if self.state == 'initializing_vslam':
            # Wait for VSLAM to initialize
            if self.vslam_initialized:
                self.get_logger().info('VSLAM ready - starting 360° mapping')
                self.state = 'mapping'
        
        elif self.state == 'mapping':
            # Execute 360° rotation for arena mapping
            self.perform_360_mapping()
        
        elif self.state == 'detecting_colors':
            # Detect colored squares on East wall
            self.detect_east_wall_colors()

        
        elif self.state == 'detecting_qrs':
            # Detect QR codes on North wall
            self.detect_north_wall_qrs()
        
        elif self.state == 'executing_mission':
            # Execute object manipulation loop
            self.execute_manipulation_loop()
        
        elif self.state == 'complete':
            # Mission complete
            pass
    
    def perform_360_mapping(self):
        """Rotate 360° to build VSLAM map (Non-blocking)"""
        # Initialize mapping state if just started
        if not hasattr(self, 'mapping_start_time'):
            self.mapping_start_time = time.time()
            self.get_logger().info('Starting 360° rotation for mapping...')
            
        # Calculate duration needed for 360 degrees
        duration = 2 * np.pi / self.rotation_speed
        elapsed = time.time() - self.mapping_start_time
        
        if elapsed < duration:
            # Continue rotating
            twist = Twist()
            twist.angular.z = self.rotation_speed
            self.cmd_vel_pub.publish(twist)
        else:
            # Finished rotation
            twist = Twist()
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            self.get_logger().info('360° mapping complete')
            # Clean up state variable
            del self.mapping_start_time
            # Transition to next state
            self.state = 'detecting_colors'
        
        self.get_logger().info('360° mapping complete')
        self.state = 'detecting_colors'
    
    def detect_east_wall_colors(self):
        """Detect all colored squares on East wall"""
        if self.rgb_image is None or self.depth_image is None:
            return
        
        self.get_logger().info('Detecting colored squares on East wall...')
        
        # Perform detection
        detections = self.color_detector.detect_colored_squares(
            self.rgb_image, self.depth_image)
        
        if len(detections) == 3:
            # Sort by Y-coordinate (North to South)
            detections.sort(key=lambda d: d['position'][1], reverse=True)
            
            # Assign positions
            for i, det in enumerate(detections, 1):
                pos_id = f'ColorPos{i}'
                self.east_wall_colors[pos_id] = {
                    'color': det['color'],
                    'position': det['position']
                }
            
            self.get_logger().info(f'Detected colors: {self.east_wall_colors}')
            self.state = 'detecting_qrs'
        else:
            self.get_logger().warn(f'Only detected {len(detections)}/3 colors, retrying...')
    
    def detect_north_wall_qrs(self):
        """Detect and decode all QR codes on North wall"""
        if self.rgb_image is None or self.depth_image is None:
            return
        
        self.get_logger().info('Detecting QR codes on North wall...')
        
        # Perform detection
        detections = self.qr_detector.detect_qr_codes(
            self.rgb_image, self.depth_image)
        
        if len(detections) == 3:
            # Sort by X-coordinate (West to East)
            detections.sort(key=lambda d: d['position'][0])
            
            # Assign QR positions
            for i, det in enumerate(detections, 1):
                qr_id = f'QR{i}'
                self.north_wall_qrs[qr_id] = {
                    'text': det['text'],
                    'position': det['position']
                }
            
            # Create QR-to-color mapping
            self.create_qr_color_mapping()
            
            self.get_logger().info(f'Detected QRs: {self.north_wall_qrs}')
            self.get_logger().info(f'QR-Color mapping: {self.qr_to_drop_mapping}')
            
            # Start mission execution
            self.current_qr = 'QR1'
            self.mission_phase = 'navigate_to_qr'
            self.state = 'executing_mission'
        else:
            self.get_logger().warn(f'Only detected {len(detections)}/3 QR codes, retrying...')
    
    def create_qr_color_mapping(self):
        """Map each QR code to its target color drop zone"""
        for qr_id, qr_data in self.north_wall_qrs.items():
            qr_text = qr_data['text']
            target_color = qr_text.replace('tn-', '')
            
            # Find matching color position
            for color_pos_id, color_data in self.east_wall_colors.items():
                if color_data['color'] == target_color:
                    self.qr_to_drop_mapping[qr_id] = {
                        'target_color_pos': color_pos_id,
                        'target_position': color_data['target_position'],
                        'target_color': target_color
                    }
                    break
    
    def execute_manipulation_loop(self):
        """Execute pickup and delivery for current QR"""
        if self.current_qr is None:
            # All QRs processed
            self.complete_mission()
            return
        
        if self.mission_phase == 'navigate_to_qr':
            self.navigate_to_qr()
        elif self.mission_phase == 'align_pickup':
            self.align_for_pickup()
        elif self.mission_phase == 'pickup':
            self.pickup_coin()
        elif self.mission_phase == 'navigate_to_drop':
            self.navigate_to_drop()
        elif self.mission_phase == 'align_drop':
            self.align_for_drop()
        elif self.mission_phase == 'drop':
            self.drop_coin()
    
    def navigate_to_qr(self):
        """Navigate to current QR code position"""
        self.get_logger().info(f'Navigating to {self.current_qr}...')
        
        # Get target position from stored QR data
        qr_data = self.north_wall_qrs[self.current_qr]
        target_pos = qr_data['position']
        
        # Calculate approach point (20cm back from wall)
        # Assuming North wall is at Y+, so we want to be at (x, y-0.2, z)
        approach_x = target_pos[0]
        approach_y = target_pos[1] - self.approach_distance_qr
        
        # Create navigation goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = approach_x
        goal.pose.position.y = approach_y
        goal.pose.orientation.z = 0.707  # Face North (approx 90 deg)
        goal.pose.orientation.w = 0.707
        
        # In a real system, send this to Nav2 or move_base
        # For this simplified controller, we'll just simulate arrival
        
        # Check if we are close enough
        if self.current_pose:
            curr_x = self.current_pose.pose.position.x
            curr_y = self.current_pose.pose.position.y
            dist = np.sqrt((curr_x - approach_x)**2 + (curr_y - approach_y)**2)
            
            if dist < self.nav_tolerance:
                self.mission_phase = 'align_pickup'
                return

        # Simulate movement (replace with actual cmd_vel logic)
        twist = Twist()
        twist.linear.x = 0.1
        self.cmd_vel_pub.publish(twist)
        
        # For simulation/testing without full nav stack:
        time.sleep(0.5) 
        self.mission_phase = 'align_pickup'
    
    def align_for_pickup(self):
        """Align robot for coin pickup using visual servoing"""
        self.get_logger().info(f'Aligning for pickup at {self.current_qr}...')
        
        if self.rgb_image is None:
            return

        # Detect QR again for fine alignment
        detections = self.qr_detector.detect_qr_codes(self.rgb_image, self.depth_image)
        
        target_qr = None
        for det in detections:
            if det['text'] == self.north_wall_qrs[self.current_qr]['text']:
                target_qr = det
                break
        
        if target_qr:
            cx, cy = target_qr['centroid_2d']
            img_center_x = self.rgb_image.shape[1] // 2
            
            # Simple P-controller for centering
            error_x = cx - img_center_x
            
            twist = Twist()
            if abs(error_x) > 20:  # Pixel tolerance
                twist.angular.z = -0.002 * error_x
                self.cmd_vel_pub.publish(twist)
            else:
                # Aligned
                self.cmd_vel_pub.publish(Twist()) # Stop
                self.mission_phase = 'pickup'
        else:
            # QR lost, maybe rotate slowly to find it?
            pass
            # For simulation, just proceed
            self.mission_phase = 'pickup'
    
    def pickup_coin(self):
        """Pick up coin"""
        self.get_logger().info(f'Picking up coin at {self.current_qr}')
        
        # Move forward slightly to grab
        twist = Twist()
        twist.linear.x = 0.05
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0)
        self.cmd_vel_pub.publish(Twist()) # Stop
        
        self.send_gripper_command('close')
        time.sleep(1.5)
        
        # Move back
        twist.linear.x = -0.05
        self.cmd_vel_pub.publish(twist)
        time.sleep(1.0)
        self.cmd_vel_pub.publish(Twist())
        
        self.mission_phase = 'navigate_to_drop'
    
    def navigate_to_drop(self):
        """Navigate to drop location"""
        drop_data = self.qr_to_drop_mapping[self.current_qr]
        self.get_logger().info(f'Navigating to drop zone: {drop_data["target_color"]}...')
        
        target_pos = drop_data['target_position']
        
        # Calculate approach point (10cm back from East wall)
        # Assuming East wall is at X+, so we want (x-0.1, y, z)
        approach_x = target_pos[0] - self.approach_distance_drop
        approach_y = target_pos[1]
        
        # Create navigation goal
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = approach_x
        goal.pose.position.y = approach_y
        goal.pose.orientation.z = 0.0  # Face East (0 deg)
        goal.pose.orientation.w = 1.0
        
        # Simulate arrival
        time.sleep(0.5)
        self.mission_phase = 'align_drop'
    
    def align_for_drop(self):
        """Align robot for coin drop"""
        self.get_logger().info('Aligning for drop...')
        
        # Visual servoing on colored square could go here
        # For now, assume navigation was accurate enough
        
        time.sleep(0.5)
        self.mission_phase = 'drop'
    
    def drop_coin(self):
        """Drop coin in target zone"""
        drop_data = self.qr_to_drop_mapping[self.current_qr]
        self.get_logger().info(f'Dropping coin at {drop_data["target_color"]} square')
        self.send_gripper_command('open')
        time.sleep(1.5)
        
        # Move to next QR
        if self.current_qr == 'QR1':
            self.current_qr = 'QR2'
        elif self.current_qr == 'QR2':
            self.current_qr = 'QR3'
        elif self.current_qr == 'QR3':
            self.current_qr = None
        
        self.mission_phase = 'navigate_to_qr'
    
    def complete_mission(self):
        """Complete Zone B mission"""
        self.get_logger().info('Zone B mission complete!')
        self.trigger_buzzer('double')
        self.state = 'complete'
    
    def send_gripper_command(self, action: str):
        """Send gripper command to Arduino"""
        cmd = {
            'type': 'gripper',
            'action': action
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.gripper_cmd_pub.publish(msg)
    
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
    node = ZoneBController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
