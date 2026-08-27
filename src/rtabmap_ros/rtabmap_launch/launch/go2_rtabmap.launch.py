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
        'approx_sync_max_interval': 0.15,
        'queue_size': 100,
        
        # SLAM, Registration & Loop Closure Tuning (Proven 0833.pgm Gold Standard Baseline)
        'Reg/Strategy': '1',                   # 1 = ICP (3D LiDAR Point Cloud Scan Matching for Loop Closures)
        'Reg/Force3DoF': 'true',               # Ground robot flat terrain 3DoF constraint (x, y, yaw) - PREVENTS CORRIDOR TWISTING
        'Optimizer/Slam2D': 'true',            # 2D Pose Graph Optimization
        'Rtabmap/DetectionRate': '2.0',
        'RGBD/NeighborLinkRefining': 'true',   # Refine odometry links with ICP
        'RGBD/ProximityBySpace': 'true',       # Proximity-based loop closure detection
        'RGBD/ProximityAngle': '180',          # Enable 3D LiDAR loop closure from any approach angle (including reverse)
        'RGBD/ProximityMaxGraphDepth': '0',    # 0 = Unlimited graph search depth (enables closing big full-corridor loops)
        'RGBD/ProximityPathMaxNeighbors': '10',# Check up to 10 nearest candidate nodes
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.1',
        'RGBD/OptimizeFromGraphEnd': 'false',  # Anchors origin [0,0,0] for rock-solid map coordinate stability
        'Mem/ReconstructData': 'true',
        'Mem/UseOdomGravity': 'true',          # Fuse 50Hz DSP odometry gravity quaternion to maintain level horizon
        'Icp/CorrespondenceRatio': '0.15',     # Robust 15% overlap threshold for reliable loop closure acceptance
        'Icp/PointToPlane': 'true',            # 3D Point-to-Plane ICP
        'Icp/PointToPlaneK': '15',             # 15 nearest neighbors for accurate normal calculation
        'Icp/PointToPlaneGroundNormalsUp': '0.9',# Force ground normals upward during quadruped gait pitch wobbles
        'Icp/VoxelSize': '0.05',               # 5cm Voxelization for ICP
        'Icp/MaxCorrespondenceDistance': '0.20',# 20cm correspondence distance for fast convergence

        # 3D Point Cloud Map & 2D Occupancy Grid Generation Parameters (From Native 3D LiDAR)
        'gen_depth': False,
        'gen_scan': False,
        'Grid/FromDepth': 'false',
        'Grid/Sensor': '0',                    # 0 = scan_cloud (Direct Native 3D LiDAR Point Cloud)
        'Grid/RangeMax': '6.0',                # 6.0m high-confidence indoor range (eliminates glass multipath)
        'Grid/RangeMin': '0.35',               # 35cm near-body blind zone (cuts off front nose & antenna reflection)
        'Grid/CellSize': '0.05',               # 5cm sharp grid resolution
        'Grid/3D': 'true',                     # Real-time 3D voxel/octomap
        'Grid/RayTracing': 'true',             # Ray tracing for clearing free space
        'Grid/NormalsSegmentation': 'true',    # 3D Surface Normal Vector Segmentation ON (separates ground vs walls)
        'Grid/MaxGroundAngle': '40',           # Planes <= 40 deg are classified as free ground
        'Grid/NormalKSearch': '15',            # 15 nearest neighbors for robust normal calculation
        'Grid/MinGroundHeight': '-0.45',       # Ground height lower bound (-45cm encompasses standing floor at -35cm)
        'Grid/MaxGroundHeight': '-0.20',       # Ground height upper bound (-20cm clamps floor roughness)
        'Grid/MaxObstacleHeight': '1.80',      # Captures full door frame, ignores ceiling lights (>1.8m)
        'Grid/NoiseFilteringRadius': '0.15',   # 15cm radius noise filter
        'Grid/NoiseFilteringMinNeighbors': '5',# Minimum 5 neighbor points required
        'Grid/FootprintRadius': '0.45',        # 45cm Clear robot body footprint & swinging legs (cuts radial spikes)
        'Grid/FlatObstacleDetected': 'false',  # Disables flat floor obstacle spikes
        'cloud_voxel_size': 0.05,              # 5cm 3D Voxel downsampling (removes 70% point cloud overload)
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
