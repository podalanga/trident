from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = FindPackageShare('trident_competition')
    
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
            'unite_imu_method': '1',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'initial_reset': 'false'
        }.items()
    )
    
    # RTAB-Map VSLAM (Mapping Mode)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_share,
                'launch',
                'rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'localization': 'false'
        }.items()
    )

    # Static Transform for Camera
    # base_link -> camera_link
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0', '0.025', '0', '0', '0', 'base_link', 'camera_link'],
        output='screen'
    )

    return LaunchDescription([
        realsense_launch,
        rtabmap_launch,
        camera_tf
    ])
