from __future__ import annotations

# pyright: reportMissingImports=false, reportAttributeAccessIssue=false


COMMON_ARGUMENTS = {
    "use_mock_hardware": "true",
    "use_mock_models": "true",
    "sensor_config_dir": "",
    "enable_debug_visualizer": "false",
    "namespace": "",
}


def make_launch_description(node_names: list[str], *, enable_debug_visualizer_default: str = "false"):
    arguments = {**COMMON_ARGUMENTS, "enable_debug_visualizer": enable_debug_visualizer_default}
    try:
        from launch.actions import DeclareLaunchArgument
        from launch.conditions import IfCondition
        from launch.substitutions import LaunchConfiguration
        from launch import LaunchDescription
        from launch_ros.actions import Node
    except ImportError:
        fallback_nodes = list(node_names)
        if enable_debug_visualizer_default == "true":
            fallback_nodes.append("debug_visualizer_node")
        return {"nodes": fallback_nodes, "arguments": arguments}

    namespace = LaunchConfiguration("namespace")
    actions = [DeclareLaunchArgument(name, default_value=value) for name, value in arguments.items()]
    node_parameters = [{name: LaunchConfiguration(name) for name in arguments}]
    actions.extend(
        Node(
            package="s2e_vlm_nodes",
            executable=node_name,
            name=node_name,
            namespace=namespace,
            output="screen",
            parameters=node_parameters,
        )
        for node_name in node_names
    )
    actions.append(
        Node(
            package="s2e_vlm_nodes",
            executable="debug_visualizer_node",
            name="debug_visualizer_node",
            namespace=namespace,
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_debug_visualizer")),
        )
    )
    return LaunchDescription(
        actions
    )
