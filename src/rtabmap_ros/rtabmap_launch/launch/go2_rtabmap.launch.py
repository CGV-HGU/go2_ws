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
        'wait_for_transform': 0.2,
        'wait_for_transform_duration': 0.2,
        'tf_delay': 0.05,
        'tf_tolerance': 0.1,
        
        # QoS Reliability (0 = Reliable, 2 = SensorData / Best Effort for Go2 Driver)
        'qos': 2,
        'qos_scan': 2,
        'qos_scan_cloud': 2,
        'qos_imu': 2,
        'qos_image': 2,
        'qos_camera_info': 2,
        'qos_odom': 2,
        
        # Asynchronous Timestamp Synchronization (Camera 30Hz, LiDAR 15Hz, IMU 50Hz)
        'approx_sync': True,
        'approx_sync_max_interval': 0.2,
        'queue_size': 100,
        
        # Full 6-DoF 3D SLAM, Registration & Loop Closure Tuning (Utilizing 4D LiDAR)
        'Reg/Strategy': '1',                   # 1 = ICP (3D LiDAR Point Cloud Scan Matching for Loop Closures)
        'Reg/Force3DoF': 'false',              # Full 6-DoF 3D motion tracking (x, y, z, roll, pitch, yaw)
        'Optimizer/Slam2D': 'false',           # Full 3D Pose Graph Optimization
        'Optimizer/Strategy': '1',             # 1 = g2o 6-DoF 3D Graph Optimizer
        'Rtabmap/DetectionRate': '4.0',        # 4.0 Hz high-frequency keyframe detection
        'RGBD/NeighborLinkRefining': 'true',   # Refine 3D odometry links with ICP
        'RGBD/ProximityBySpace': 'true',       # Proximity-based loop closure detection (3D space search)
        'RGBD/ProximityAngle': '180',          # Enable 3D LiDAR loop closure even when returning in opposite direction!
        'RGBD/ProximityPathMaxNeighbors': '10',# Check up to 10 neighboring nodes for 3D ICP loop closure
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.1',
        'Mem/ReconstructData': 'true',
        'Icp/CorrespondenceRatio': '0.2',     # Minimum 20% point overlap required to accept 3D loop closure

        # 3D Point Cloud Map & 2D Occupancy Grid Generation Parameters (From Native 3D LiDAR)
        'gen_depth': False,
        'gen_scan': False,
        'Grid/FromDepth': 'false',
        'Grid/Sensor': '0',                    # 0 = scan_cloud (Direct Native 3D LiDAR Point Cloud)
        'Grid/RangeMax': '5.0',                # 5.0m crisp indoor corridor range (eliminates glass/multipath ghost points)
        'Grid/RangeMin': '0.25',               # 25cm minimum distance (ignores LiDAR housing reflections)
        'Grid/CellSize': '0.05',               # 5cm sharp grid resolution
        'Grid/3D': 'true',                     # Real-time 3D voxel/octomap
        'Grid/RayTracing': 'true',             # Ray tracing for clearing free space
        'Grid/NormalsSegmentation': 'true',    # 3D Surface normal segmentation (flawless floor vs vertical wall separation)
        'Grid/MaxGroundAngle': '45',           # Surfaces with slope <45 deg are 100% ground (immune to body bobbing)
        'Grid/NormalK': '10',                  # Surface normal computation with 10 neighbors
        'Grid/ClusterRadius': '0.10',          # Cluster radius for normal grouping
        'Grid/MinGroundHeight': '-0.60',       # Ground lower bound (-60cm)
        'Grid/MaxGroundHeight': '0.10',        # Ground upper bound (+10cm)
        'Grid/MaxObstacleHeight': '1.50',      # Obstacles up to 1.5m
        'Grid/NoiseFilteringRadius': '0.15',   # 15cm spatial noise filtering radius
        'Grid/NoiseFilteringMinNeighbors': '5',# Minimum 5 neighbor points required (eliminates scattered ghost noise)
        'Grid/FootprintRadius': '0.40',        # Clear robot body footprint
        'cloud_voxel_size': 0.05,              # 5cm 3D Voxel downsampling (removes 70% point cloud overload)
        'Icp/PointToPlane': 'true',
        'Icp/VoxelSize': '0.05',
        'Icp/MaxCorrespondenceDistance': '0.10',
        'Icp/MaxTranslation': '0.30',          # Rejects jump drift >30cm between frames
        'Icp/MaxRotation': '0.40',             # Rejects rotational jumps >23 degrees
        'Icp/Iterations': '30',                # Deep convergence iterations
        'Icp/Epsilon': '0.001',                # 1mm convergence threshold
        'RGBD/OptimizeFromGraphEnd': 'false',  # Anchors origin [0,0,0] for stable global map
        'Mem/NotLinkedNodesKept': 'false',     # Discard unlinked outlier nodes
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
        DeclareLaunchArgument('scan_cloud_topic', default_value='/pointcloud', description='PointCloud2 topic for LiDAR input'),
        DeclareLaunchArgument('subscribe_scan_cloud', default_value='true', description='Subscribe to LiDAR PointCloud (set true when lidar is active)'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false', description='Launch RTAB-Map real-time 3D GUI visualizer'),
        
        # Static Transforms for Sensor Frames (Aligned with official Go2 URDF kinematics)
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
            arguments=['0.289', '0', '-0.047', '0', '0', '0', 'base_link', 'utlidar_lidar']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_unilidar_tf',
            arguments=['0.289', '0', '-0.047', '0', '0', '0', 'base_link', 'unilidar_lidar']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_radar_tf',
            arguments=['0.289', '0', '-0.047', '0', '0', '0', 'base_link', 'radar']
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
