"""Full maintained asynchronous PointNav policy with measured robot boundaries."""

import os
from pathlib import Path
from datetime import datetime, timezone
from ament_index_python.packages import get_package_share_directory
from s2e_vlm_robot.full_profile import PROFILE_NAME, full_policy_environment
from s2e_vlm_robot.body_look_control import BODY_COMMAND_TIMEOUT_S
from s2e_vlm_nodes.look_execution import LOOK_EXECUTION_MODES

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    motion_enabled = LaunchConfiguration("motion_enabled")
    vlm_backend = LaunchConfiguration("vlm_backend")
    localization_pose_topic = LaunchConfiguration("localization_pose_topic")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    odometry_topic = LaunchConfiguration("odometry_topic")
    direct = PythonExpression(["'", LaunchConfiguration('navigation_mode'), "' == 'direct_goal'"])
    profile = Path(get_package_share_directory("s2e_vlm_robot")) / "config" / PROFILE_NAME
    environment = full_policy_environment(profile)
    run_dir = os.environ.get("ESCAPE_RUN_DIR", os.environ.get("ESCAPE_RUN_ROOT", "/tmp/escape-runs").rstrip("/") + "/" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ"))
    environment.update({
        "VLM_ASYNC_TRACE_DIR": run_dir,
        "PIXNAV_ASYNC_TRACE_DIR": run_dir,
        "VLM_CALL_LOG_PATH": run_dir + "/vlm_calls.jsonl",
        "VLM_QWEN_CALL_LOG_PATH": run_dir + "/qwen_calls.jsonl",
        "PIXNAV_WORKER_STARTUP_TIMEOUT_S": "120",
    })
    return LaunchDescription(
        [
            DeclareLaunchArgument("motion_enabled", default_value="false"),
            DeclareLaunchArgument("sensor_input_enabled", default_value="true"),
            DeclareLaunchArgument('navigation_mode', default_value=os.environ.get('ROBOT_NAVIGATION_MODE','full'), choices=['full','direct_goal']),
            DeclareLaunchArgument('record_model_inputs', default_value='false', choices=['true','false']),
            # Direct PixelNav keeps one final-goal session for the episode.
            # Full's 50-step cap is per intermediate waypoint, so applying it
            # to that single final-goal session would truncate the comparison.
            DeclareLaunchArgument('direct_goal_max_policy_steps', default_value='500'),
            DeclareLaunchArgument('look_execution', default_value=os.environ.get('PIXNAV_LOOK_EXECUTION','physical'), choices=list(LOOK_EXECUTION_MODES)),
            DeclareLaunchArgument("vlm_backend", default_value="live_vlm"),
            DeclareLaunchArgument("odometry_topic", default_value="/s2e/robot/sensors/odom"),
            DeclareLaunchArgument("pose_source", default_value="odometry"),
            DeclareLaunchArgument("success_distance_m", default_value="1.0"),
            DeclareLaunchArgument("minimum_rotate_speed_radps", default_value="0.40"),
            DeclareLaunchArgument("turn_speed_radps", default_value="0.45"),
            DeclareLaunchArgument("rotate_settle_window_s", default_value="0.0"),
            DeclareLaunchArgument("rotate_recoil_limit_deg", default_value="0.0"),
            DeclareLaunchArgument("rotate_min_timeout_s", default_value="0.0"),
            DeclareLaunchArgument(
                "localization_pose_topic",
                default_value="/rtabmap/localization_pose",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/robot_nav/sensors/front_camera/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/robot_nav/sensors/front_camera/camera_info",
            ),
            *[SetEnvironmentVariable(key, value) for key, value in environment.items()],
            SetEnvironmentVariable("VLM_ROTATE_MIN_TIMEOUT_S", LaunchConfiguration("rotate_min_timeout_s")),
            # Observation turns and PixelNav turns share the physical speed setting.
            SetEnvironmentVariable("VLM_ROBOT_ROTATE_SPEED_RADPS", LaunchConfiguration("turn_speed_radps")),
            SetEnvironmentVariable("PIXNAV_COMMAND_RESULT_TIMEOUT_S", PythonExpression([
                "str(max(", environment["PIXNAV_COMMAND_RESULT_TIMEOUT_S"], ", ",
                str(BODY_COMMAND_TIMEOUT_S + 5.0), " + float('",
                LaunchConfiguration("rotate_min_timeout_s"), "')))"
            ])),
            SetEnvironmentVariable('PIXNAV_LOOK_EXECUTION',LaunchConfiguration('look_execution')),
            SetEnvironmentVariable('VLM_MODEL_CALL_LOG_INCLUDE_REQUEST_PAYLOAD',LaunchConfiguration('record_model_inputs')),
            SetEnvironmentVariable('PIXNAV_MAX_GO_STEPS',
                LaunchConfiguration('direct_goal_max_policy_steps'),condition=IfCondition(direct)),
            SetEnvironmentVariable("VLM_BACKEND", vlm_backend),
            SetEnvironmentVariable("VLM_MODEL_ADAPTER", ""),
            Node(package="s2e_vlm_nodes", executable="sweep_scheduler_node", output="screen",condition=UnlessCondition(direct)),
            Node(package="s2e_vlm_robot", executable="go2_sensor_input", output="screen",
                 condition=IfCondition(LaunchConfiguration("sensor_input_enabled"))),
            Node(
                package="s2e_vlm_robot",
                executable="rtab_pointgoal_adapter",
                output="screen",
                parameters=[
                    {
                        "localization_pose_topic": localization_pose_topic,
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                        "odometry_topic": odometry_topic,
                        "pose_source": LaunchConfiguration("pose_source"),
                        "success_distance_m": ParameterValue(LaunchConfiguration("success_distance_m"), value_type=float),
                        "scene_id": "local-odometry",
                        "output_image_width": 640,
                    }
                ],
            ),
            Node(
                package="s2e_vlm_nodes",
                executable="vlm_node",
                output="screen",
                condition=UnlessCondition(direct),
            ),
            Node(package='s2e_vlm_robot',executable='direct_goal_pixnav',output='screen',condition=IfCondition(direct)),
            Node(
                package="s2e_vlm_nodes",
                executable="pixnav_runtime_node",
                output="screen",
            ),
            Node(
                package="s2e_vlm_robot",
                executable="go2_action_executor",
                output="screen",
                parameters=[
                    {
                        "odometry_topic": odometry_topic,
                        "motion_enabled": ParameterValue(
                            motion_enabled,
                            value_type=bool,
                        ),
                        "body_look_enabled": True,
                        "minimum_rotate_speed_radps": ParameterValue(LaunchConfiguration("minimum_rotate_speed_radps"), value_type=float),
                        "turn_speed_radps": ParameterValue(LaunchConfiguration("turn_speed_radps"), value_type=float),
                        "rotate_settle_window_s": ParameterValue(LaunchConfiguration("rotate_settle_window_s"), value_type=float),
                        "rotate_recoil_limit_deg": ParameterValue(LaunchConfiguration("rotate_recoil_limit_deg"), value_type=float),
                        "rotate_min_timeout_s": ParameterValue(LaunchConfiguration("rotate_min_timeout_s"), value_type=float),
                        'planner_status_node_name': ParameterValue(PythonExpression([
                            "'direct_goal_pixnav' if ",direct," else 'vlm_node'"]),value_type=str),
                    }
                ],
            ),
        ]
    )
