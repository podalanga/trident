"""Launch file for testing Zone A only"""

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
    
    # Zone A Controller Node
    zone_a_controller = Node(
        package='trident_competition',
        executable='zone_a_controller.py',
        name='zone_a_controller',
        parameters=[params_file],
        output='screen'
    )
    
    # Arduino Interface Node
    arduino_interface = Node(
        package='trident_competition',
        executable='arduino_interface.py',
        name='arduino_interface',
        parameters=[params_file],
        output='screen'
    )
    
    return LaunchDescription([
        arduino_interface,
        zone_a_controller
    ])
