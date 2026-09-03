from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    """
    Unitree Go2 built-in LIO + RGB RTAB-Map 3D mapping launch.
    =========================================================
    - /livo/odom: Unitree LiDAR+IMU odometry (external odometry input)
    - /livo/cloud: motion-deskewed cloud transformed back into base_link
    - /livo/imu: Unitree LiDAR IMU, with a shared host-clock offset
    - front RGB: appearance/visual-place recognition for loop candidates

    The Go2 front camera is monocular.  Without calibrated depth or stereo it
    cannot provide metric visual odometry to RTAB-Map, so this launch is LIO
    mapping with visual place recognition, not a standalone RTAB-Map VIO node.

    Modes:
    - localization:=false (default): new 3D map (deletes the selected DB with -d)
    - localization:=true: localize against the existing RTAB-Map database
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    localization = LaunchConfiguration('localization', default='false')
    scan_cloud_topic = LaunchConfiguration('scan_cloud_topic', default='/livo/cloud')
    subscribe_scan_cloud = LaunchConfiguration('subscribe_scan_cloud', default='true')
    reg_force_3dof = LaunchConfiguration('reg_force_3dof', default='true')
    icp_force_4dof = LaunchConfiguration('icp_force_4dof', default='false')
    loop_closure_identity_guess = LaunchConfiguration('loop_closure_identity_guess', default='true')
    proximity_by_space = LaunchConfiguration('proximity_by_space', default='false')

    # Common LIO mapping parameters for both modes.
    base_parameters = {
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': True,
        'use_sim_time': use_sim_time,
        
        # Sensor modalities. RGB contributes visual words/place recognition;
        # geometric registration is performed with the 3D LiDAR cloud.
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
        
        # Camera and LiDAR are independent streams, aligned approximately.
        'approx_sync': True,
        'approx_sync_max_interval': 0.12,
        'queue_size': 30,
        
        # 3D LiDAR registration with a planar graph for the flat-floor campaign.
        # RGB retrieves global candidates; 3D LiDAR ICP still verifies them.
        'Reg/Strategy': '1',                   # 1 = ICP; RGB still detects loop candidates
        # RTAB-Map's core parameter table is string-valued. Foxy otherwise
        # applies YAML coercion to LaunchConfiguration("true"/"false") and
        # passes ROS booleans, making the rtabmap process abort at startup.
        'Reg/Force3DoF': ParameterValue(reg_force_3dof, value_type=str),
        'Icp/Force4DoF': ParameterValue(icp_force_4dof, value_type=str),
        'RGBD/LoopClosureIdentityGuess': ParameterValue(loop_closure_identity_guess, value_type=str),
        'Optimizer/GravitySigma': '0.3',
        # Keep RTAB-Map's documented loop acceptance safeguards explicit. Do
        # not enable Optimizer/Robust together with RGBD/OptimizeMaxError.
        'Rtabmap/LoopThr': '0.11',
        'RGBD/OptimizeMaxError': '3.0',
        'Optimizer/Robust': 'false',
        'Rtabmap/DetectionRate': '2.0',
        'Rtabmap/PublishStats': 'true',        # Required by the headless loop-event logger
        # Preserve the built-in Unitree LIO neighbor links. The 2026-08-28
        # full-map ablation showed that accepted spatial-proximity (Type-2)
        # links caused the severe corridor folding, while Type-1 visual loops
        # alone preserved the two physical 90-degree corners. Keep Type-2 off
        # in the canonical profile; RGB retrieval + 3D LiDAR ICP remains on.
        'RGBD/NeighborLinkRefining': 'false',
        'RGBD/ProximityBySpace': ParameterValue(proximity_by_space, value_type=str),
        # Restore RTAB-Map's conservative defaults as guardrails if proximity
        # is ever enabled for a separate, explicitly labelled diagnostic.
        'RGBD/ProximityAngle': '45',
        'RGBD/ProximityMaxGraphDepth': '50',
        'RGBD/ProximityPathMaxNeighbors': '0',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.1',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Mem/UseOdomGravity': 'false',         # Use /livo/imu for gravity links, not odometry attitude
        'Icp/CorrespondenceRatio': '0.15',     # Robust 15% overlap threshold for reliable loop closure acceptance
        'Icp/PointToPlane': 'true',            # 3D Point-to-Plane ICP
        'Icp/PointToPlaneK': '15',             # 15 nearest neighbors for accurate normal calculation
        'Icp/PointToPlaneGroundNormalsUp': '0.9',# Force ground normals upward during quadruped gait pitch wobbles
        'Icp/VoxelSize': '0.05',               # 5cm Voxelization for ICP
        'Icp/MaxCorrespondenceDistance': '0.20',# 20cm correspondence distance for fast convergence

        # Local 3D occupancy data generated directly from the scan cloud.
        'gen_depth': False,
        'gen_scan': False,
        'Grid/Sensor': '0',                    # 0 = laser scan / scan_cloud
        'Grid/RangeMax': '6.0',                # 6.0m high-confidence indoor range (eliminates glass multipath)
        'Grid/RangeMin': '0.35',               # 35cm near-body blind zone (cuts off front nose & antenna reflection)
        'Grid/CellSize': '0.05',               # 5cm sharp grid resolution
        'Grid/3D': 'true',                     # Real-time 3D voxel/octomap
        'Grid/RayTracing': 'true',             # Ray tracing for clearing free space
        'Grid/NormalsSegmentation': 'true',    # 3D Surface Normal Vector Segmentation ON (separates ground vs walls)
        'Grid/MaxGroundAngle': '40',           # Planes <= 40 deg are classified as free ground
        'Grid/NormalK': '15',                  # 15 nearest neighbors for robust normal calculation
        'Grid/MinGroundHeight': '-0.45',       # Ground height lower bound (-45cm encompasses standing floor at -35cm)
        'Grid/MaxGroundHeight': '-0.20',       # Ground height upper bound (-20cm clamps floor roughness)
        'Grid/MaxObstacleHeight': '1.80',      # Captures full door frame, ignores ceiling lights (>1.8m)
        'Grid/NoiseFilteringRadius': '0.15',   # 15cm radius noise filter
        'Grid/NoiseFilteringMinNeighbors': '5',# Minimum 5 neighbor points required
        'GridGlobal/FootprintRadius': '0.45',  # Clear robot body footprint in the assembled grid
        'Grid/FlatObstacleDetected': 'false',  # Disables flat floor obstacle spikes
    }

    # Mode 1: Mapping Parameters
    mapping_parameters = dict(base_parameters)
    mapping_parameters.update({
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
    })

    # Mode 2: localization against the existing database.
    localization_parameters = dict(base_parameters)
    localization_parameters.update({
        'Mem/IncrementalMemory': 'false',
        'Mem/InitWMWithAllNodes': 'true',
    })

    remappings = [
        ('rgb/image', '/camera/front/image_raw'),
        ('rgb/camera_info', '/camera/front/camera_info'),
        ('odom', '/livo/odom'),
        ('scan_cloud', scan_cloud_topic),
        ('imu', '/livo/imu'),
        ('localization_pose', '/rtabmap/localization_pose'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('localization', default_value='false', description='Localize against the existing database instead of creating a new map'),
        DeclareLaunchArgument('scan_cloud_topic', default_value='/livo/cloud', description='Base-frame PointCloud2 topic prepared by go2_livo_sensor_bridge.py'),
        DeclareLaunchArgument('subscribe_scan_cloud', default_value='true', description='Subscribe to LiDAR PointCloud (set true when lidar is active)'),
        DeclareLaunchArgument('rtabmap_viz', default_value='false', description='Launch RTAB-Map real-time 3D GUI visualizer'),
        DeclareLaunchArgument('reg_force_3dof', default_value='true', description='Constrain registration transforms to x/y/yaw'),
        DeclareLaunchArgument('icp_force_4dof', default_value='false', description='Allow ICP x/y/z/yaw correction only for an explicit 4DoF diagnostic'),
        DeclareLaunchArgument('loop_closure_identity_guess', default_value='false', description='Let RGB retrieval seed the 3D LiDAR ICP global-loop check'),
        DeclareLaunchArgument('proximity_by_space', default_value='false', description='Enable Type-2 spatial proximity loops only for a separately labelled diagnostic'),
        
        # Camera mount is the existing project estimate. Calibrated camera
        # intrinsics/extrinsics are still required for reliable visual loops.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_optical_tf',
            # x-forward/y-left/z-up -> optical z-forward/x-right/y-down.
            arguments=['0.285', '0', '0.01', '-1.5707963', '0', '-1.5707963', 'base_link', 'camera_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_utlidar_imu_tf',
            # Built-in IMU acceleration shows z-up alignment with base_link.
            # Translation is approximate and does not affect gravity links.
            arguments=['0.28945', '0', '-0.046825', '0', '0', '0', 'base_link', 'utlidar_imu']
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

        # Node 2: Existing-map localization mode (when localization:=true)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[localization_parameters],
            remappings=remappings,
            arguments=[], # Load the saved database without deleting it
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
