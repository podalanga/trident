"""Simple launch file without camera dependencies for testing"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate simple launch for testing without camera"""
    
    pkg_share = FindPackageShare('trident_competition')
    params_file = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])
    
    return LaunchDescription([
        # Mission Manager
        Node(
            package='trident_competition',
            executable='mission_manager.py',
            name='mission_manager',
            parameters=[params_file],
            output='screen'
        ),
        
        # Zone A Controller
        Node(
            package='trident_competition',
            executable='zone_a_controller.py',
            name='zone_a_controller',
            parameters=[params_file],
            output='screen'
        ),
        
        # Arduino Interface
        Node(
            package='trident_competition',
            executable='arduino_interface.py',
            name='arduino_interface',
            parameters=[params_file],
            output='screen'
        )
    ])
