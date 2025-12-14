"""Launch file for complete mission"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for complete mission"""
    
    pkg_share = FindPackageShare('trident_competition')
    
    # Parameters file
    params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'params.yaml'
    ])
    
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    enable_camera = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense camera (requires realsense2_camera package)'
    )
    
    # Mission Manager Node
    mission_manager = Node(
        package='trident_competition',
        executable='mission_manager.py',
        name='mission_manager',
        parameters=[params_file],
        output='screen'
    )
    
    # Zone A Controller Node
    zone_a_controller = Node(
        package='trident_competition',
        executable='zone_a_controller.py',
        name='zone_a_controller',
        parameters=[params_file],
        output='screen'
    )
    
    # Zone B Controller Node
    zone_b_controller = Node(
        package='trident_competition',
        executable='zone_b_controller.py',
        name='zone_b_controller',
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
    
    # RealSense Camera (optional - can be disabled with enable_camera:=false)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'align_depth.enable': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            # NOTE: realsense2_camera expects an integer in range [0, 2]
            # 0=none, 1=copy, 2=linear_interpolation
            'unite_imu_method': '1'
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_camera'))
    )
    
    return LaunchDescription([
        use_sim_time,
        enable_camera,
        realsense_launch,
        arduino_interface,
        mission_manager,
        zone_a_controller,
        zone_b_controller
    ])
