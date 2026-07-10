from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="debug_visualizer_node",
    publishes=("/s2e/debug/visualizer/image", "/s2e/status/debug_visualizer_node"),
    subscribes=(
        "/s2e/sensors/camera/image",
        "/s2e/sensors/camera/camera_info",
        "/s2e/vlm/reasoning",
        "/s2e/e2e/trajectory",
        "/s2e/e2e/status",
        "/s2e/controller/status",
        "/s2e/supervisor/health",
        "/s2e/status/static_tf_node",
        "/s2e/status/lidar_node",
        "/s2e/status/camera_node",
        "/s2e/status/imu_node",
        "/s2e/status/odometry_node",
        "/s2e/status/vlm_node",
        "/s2e/status/e2e_node",
        "/s2e/status/controller_node",
        "/s2e/status/supervisor_node",
    ),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
