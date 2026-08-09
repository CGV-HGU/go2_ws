import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Unitree Go2 Built-in RGB Camera + L2 LiDAR + IMU RTAB-Map LIVO Launch Script
    =============================================================================
    Uses strictly the Go2 built-in sensor suite:
    - Front Ultra-Wide RGB Camera (/camera/front/image_raw)
    - Unitree L2 LiDAR (/utlidar/cloud_deskewed)
    - Go2 Body IMU (/utlidar/imu)
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # RTAB-Map Configuration for Go2 Onboard Built-in Sensors
    rtabmap_parameters = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'use_sim_time': use_sim_time,
        
        # RTAB-Map LIVO (LiDAR-Visual-Inertial Odometry) with Go2 Built-in Sensors
        'subscribe_depth': False,             # Using Go2 Built-in RGB Camera (No External Depth Camera)
        'subscribe_rgb': True,                # Go2 Front Ultra-Wide RGB Camera
        'subscribe_scan_cloud': True,         # Go2 L2 LiDAR Pointcloud
        
        # SLAM & Loop Closure Tuning
        'Rtabmap/DetectionRate': '2.0',       # 2Hz Loop Closure Detection
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',         # Update map every 0.05 rad rotation
        'RGBD/LinearUpdate': '0.1',           # Update map every 0.1m movement
        'Mem/ReconstructData': 'true',
        'Mem/IncrementalMemory': 'true',
    }

    remappings = [
        ('rgb/image', '/camera/front/image_raw'),
        ('rgb/camera_info', '/camera/front/camera_info'),
        ('scan_cloud', '/utlidar/cloud_deskewed'),
        ('imu', '/utlidar/imu'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        
        # RTAB-Map Single Node Execution
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=remappings,
            arguments=['-d'] # Delete database on start
        )
    ])
