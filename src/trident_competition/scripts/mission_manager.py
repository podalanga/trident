#!/usr/bin/env python3
"""Mission Manager - High-level mission coordination"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import time


class MissionManager(Node):
    """High-level mission coordinator for the competition"""
    
    def __init__(self):
        super().__init__('mission_manager')
        
        # Mission state
        self.zone_a_complete = False
        self.zone_b_complete = False
        self.mission_start_time = None
        
        # Publishers
        self.zone_a_start_pub = self.create_publisher(Bool, 'zone_a/start', 10)
        self.zone_b_start_pub = self.create_publisher(Bool, 'zone_b/start', 10)
        
        # Subscribers
        self.zone_a_state_sub = self.create_subscription(
            String, 'zone_a/state', self.zone_a_state_callback, 10)
        self.zone_b_state_sub = self.create_subscription(
            String, 'zone_b/state', self.zone_b_state_callback, 10)
        self.mission_start_sub = self.create_subscription(
            Bool, 'mission/start', self.mission_start_callback, 10)
        
        # Status timer
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info('Mission Manager initialized')
        self.get_logger().info('Waiting for mission start command...')
    
    def mission_start_callback(self, msg):
        """Start the complete mission"""
        if msg.data and self.mission_start_time is None:
            self.get_logger().info('=' * 60)
            self.get_logger().info('MISSION START')
            self.get_logger().info('=' * 60)
            self.mission_start_time = time.time()
            
            # Start Zone A
            self.start_zone_a()
    
    def zone_a_state_callback(self, msg):
        """Monitor Zone A state"""
        state = msg.data
        
        if state == 'complete' and not self.zone_a_complete:
            self.zone_a_complete = True
            elapsed = time.time() - self.mission_start_time
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'ZONE A COMPLETE - Time: {elapsed:.2f}s')
            self.get_logger().info('=' * 60)
            
            # Start Zone B
            time.sleep(2.0)  # Brief pause
            self.start_zone_b()
    
    def zone_b_state_callback(self, msg):
        """Monitor Zone B state"""
        state = msg.data
        
        if 'complete' in state and not self.zone_b_complete:
            self.zone_b_complete = True
            elapsed = time.time() - self.mission_start_time
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'ZONE B COMPLETE - Time: {elapsed:.2f}s')
            self.get_logger().info('=' * 60)
            self.complete_mission()
    
    def start_zone_a(self):
        """Send start signal to Zone A controller"""
        self.get_logger().info('Starting Zone A - Line Following')
        msg = Bool()
        msg.data = True
        self.zone_a_start_pub.publish(msg)
    
    def start_zone_b(self):
        """Send start signal to Zone B controller"""
        self.get_logger().info('Starting Zone B - VSLAM and Object Manipulation')
        msg = Bool()
        msg.data = True
        self.zone_b_start_pub.publish(msg)
    
    def complete_mission(self):
        """Mission completion handler"""
        total_time = time.time() - self.mission_start_time
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('MISSION COMPLETE!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total Mission Time: {total_time:.2f}s')
        self.get_logger().info(f'Zone A: {"COMPLETE" if self.zone_a_complete else "INCOMPLETE"}')
        self.get_logger().info(f'Zone B: {"COMPLETE" if self.zone_b_complete else "INCOMPLETE"}')
        self.get_logger().info('=' * 60)
        
        # Generate mission report
        self.generate_mission_report()
    
    def generate_mission_report(self):
        """Generate detailed mission report"""
        report = {
            'status': 'MISSION_COMPLETE',
            'total_time': time.time() - self.mission_start_time,
            'zone_a': 'completed' if self.zone_a_complete else 'incomplete',
            'zone_b': 'completed' if self.zone_b_complete else 'incomplete'
        }
        
        self.get_logger().info(f'Mission Report: {report}')
    
    def publish_status(self):
        """Periodic status update"""
        if self.mission_start_time is not None:
            elapsed = time.time() - self.mission_start_time
            self.get_logger().info(
                f'Mission Status - Elapsed: {elapsed:.1f}s | '
                f'Zone A: {"✓" if self.zone_a_complete else "○"} | '
                f'Zone B: {"✓" if self.zone_b_complete else "○"}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
