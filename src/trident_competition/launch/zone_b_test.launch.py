"""Launch file for testing Zone B functionality with VSLAM localization"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import os


def generate_launch_description():
    """Generate launch description for Zone B test"""
    
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
        description='Enable RealSense camera'
    )
    
    # RealSense Camera
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
    
    # RTAB-Map VSLAM (Localization)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'localization': 'false'  # Start in mapping mode to establish 0,0 or true if map exists
        }.items()
    )
    
    # Arduino Interface Node
    arduino_interface = Node(
        package='trident_competition',
        executable='arduino_interface.py',
        name='arduino_interface',
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
    
    # Static Transform for Camera (if not provided by URDF)
    # Assuming camera is mounted on robot base
    # base_link -> camera_link
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0', '0.025', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time,
        enable_camera,
        camera_tf,
        realsense_launch,
        rtabmap_launch,
        arduino_interface,
        zone_b_controller
    ])
