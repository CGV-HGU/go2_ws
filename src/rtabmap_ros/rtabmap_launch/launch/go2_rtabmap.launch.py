import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    """
    Unitree Go2 Built-in RGB Camera + L1/L2 LiDAR + IMU RTAB-Map LIVO Launch Script
    =============================================================================
    Uses strictly the Go2 built-in sensor suite:
    - Front Ultra-Wide RGB Camera (/camera/front/image_raw & /camera/front/camera_info)
    - Unitree 4D LiDAR (/pointcloud or /utlidar/cloud)
    - Go2 Body IMU (/imu or /utlidar/imu)

    Modes:
    - localization:=false (Default): 3D Mapping Mode (IncrementalMemory: true, deletes DB on start)
    - localization:=true           : Pure Odometry Mode for Online S2E Testing (IncrementalMemory: false)
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    localization = LaunchConfiguration('localization', default='false')
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic', default='pointcloud')
    subscribe_scan_cloud = LaunchConfiguration('subscribe_scan_cloud', default='true')

    # Common LIVO parameters for both modes
    base_parameters = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'use_sim_time': use_sim_time,
        
        # RTAB-Map LIVO Sensor Modalities
        'subscribe_depth': False,
        'subscribe_rgb': True,
        'subscribe_odom': True,
        'subscribe_scan_cloud': subscribe_scan_cloud,
        'subscribe_imu': True,
        
        # QoS Reliability (2 = SensorData / Best Effort, 0 = System Default / Reliable)
        'qos': 0,
        'qos_scan': 2,
        'qos_imu': 0,
        'qos_image': 0,
        'qos_camera_info': 0,
        'qos_odom': 0,
        
        # Asynchronous Timestamp Synchronization (Camera 30Hz, LiDAR 15Hz, IMU 50Hz)
        'approx_sync': True,
        'approx_sync_max_interval': 0.1,
        'queue_size': 50,
        'subscribe_scan_cloud': LaunchConfiguration('subscribe_scan_cloud'),
        
        # SLAM & Loop Closure Tuning
        'Rtabmap/DetectionRate': '2.0',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.1',
        'Mem/ReconstructData': 'true',

        # 3D Point Cloud Map & 2D Occupancy Grid Generation Parameters (From LiDAR & Motion)
        'gen_depth': True,
        'gen_scan': True,
        'Grid/FromDepth': 'false',
        'Grid/Sensor': '0',            # 0 = scan_cloud (3D Point Cloud LiDAR)
        'Grid/RangeMax': '15.0',       # Max range 15m
        'Grid/RangeMin': '0.2',
        'Grid/CellSize': '0.05',       # 5cm grid resolution
        'Grid/3D': 'true',             # Real-time 3D voxel/octomap
        'Grid/RayTracing': 'true',     # Ray tracing for clearing free space
        'Icp/PointToPlane': 'true',
        'Icp/VoxelSize': '0.05',
    }

    # Mode 1: Mapping Parameters
    mapping_parameters = dict(base_parameters)
    mapping_parameters.update({
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
    })

    # Mode 2: Pure Odometry / Localization Parameters (No Map Interference to S2E)
    localization_parameters = dict(base_parameters)
    localization_parameters.update({
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',
    })

    remappings = [
        ('rgb/image', '/camera/front/image_raw'),
        ('rgb/camera_info', '/camera/front/camera_info'),
        ('odom', '/odom'),
        ('scan_cloud', scan_cloud_topic),
        ('imu', '/imu'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('localization', default_value='false', description='Run in localization/pure odometry mode'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/utlidar/cloud', description='PointCloud2 topic for LiDAR input'),
        DeclareLaunchArgument('subscribe_scan_cloud', default_value='true', description='Subscribe to LiDAR PointCloud (set true when lidar is active)'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false', description='Launch RTAB-Map real-time 3D GUI visualizer'),
        
        # Static Transforms for Sensor Frames (Self-contained TF tree)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=['0.285', '0', '0.01', '0', '0', '0', 'base_link', 'camera_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_utlidar_tf',
            arguments=['0.285', '0', '0.01', '0', '0', '0', 'base_link', 'utlidar_lidar']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_unilidar_tf',
            arguments=['0.285', '0', '0.01', '0', '0', '0', 'base_link', 'unilidar_lidar']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_radar_tf',
            arguments=['0.285', '0', '0.01', '0', '0', '0', 'base_link', 'radar']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_raw_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_unilidar_imu_tf',
            arguments=['0.285', '0', '0.01', '0', '0', '0', 'base_link', 'unilidar_imu']
        ),

        # Node 1: Mapping Mode Node (when localization:=false)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[mapping_parameters],
            remappings=remappings,
            arguments=['-d'], # Delete database on start for clean new map
            condition=UnlessCondition(localization)
        ),

        # Node 2: Localization / Pure Odometry Mode Node (when localization:=true)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[localization_parameters],
            remappings=remappings,
            arguments=[], # Load saved map without deleting
            condition=IfCondition(localization)
        ),

        # Node 3: Real-time 3D GUI Visualizer Node (when rtabmap_viz:=true)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[mapping_parameters],
            remappings=remappings,
            condition=IfCondition(LaunchConfiguration('rtabmap_viz'))
        )
    ])
