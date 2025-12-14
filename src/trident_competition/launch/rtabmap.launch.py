"""Launch file for RTAB-Map VSLAM"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RTAB-Map VSLAM"""
    
    pkg_share = FindPackageShare('trident_competition')
    
    # Declare arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Start in localization mode'
    )
    
    # RTAB-Map parameters
    parameters = {
        'frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan': False,
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'approx_sync': True,
        
        # RTAB-Map core parameters
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.01',
        'RGBD/LinearUpdate': '0.01',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Grid/FromDepth': 'true',
        'Reg/Force3DoF': 'true',
        'Reg/Strategy': '1',  # Use ICP
        
        # Odometry parameters
        'Vis/MinInliers': '15',
        'Vis/InlierDistance': '0.1',
        'OdomF2M/ScanSubtractRadius': '0.1',
        'OdomF2M/ScanMaxSize': '15000',
    }
    
    # RTAB-Map Node
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw')
        ],
        arguments=['--delete_db_on_start']
    )
    
    # RTAB-Map Visual Odometry
    rtabmap_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw')
        ]
    )
    
    # RTAB-Map Visualization
    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmapviz',
        name='rtabmapviz',
        output='screen',
        parameters=[parameters],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_rect_raw')
        ]
    )
    
    return LaunchDescription([
        use_sim_time,
        localization,
        rtabmap,
        rtabmap_odom,
        # rtabmap_viz  # Uncomment for visualization
    ])
