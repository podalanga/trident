"""Launch file for testing Zone A only - Updated with test_firmware calibration
Modifications based on test_firmware:
- Updated PID parameters (Kp=45.0, Ki=0.0, Kd=12.0)
- Improved calibrated sensor processing
- Enhanced line following algorithm
- Better end square detection
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for Zone A testing"""
    
    pkg_share = FindPackageShare('trident_competition')
    
    # Parameters file
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'params.yaml'
    ])
    
    # Zone A Controller Node - Updated with test_firmware calibration
    zone_a_controller = Node(
        package='trident_competition',
        executable='zone_a_controller.py',
        name='zone_a_controller',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,  # Better terminal output formatting
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    # Arduino Interface Node
    arduino_interface = Node(
        package='trident_competition',
        executable='arduino_interface.py',
        name='arduino_interface',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,  # Better terminal output formatting
        arguments=['--ros-args', '--log-level', 'info']
    )
    
    return LaunchDescription([
        arduino_interface,
        zone_a_controller
    ])
