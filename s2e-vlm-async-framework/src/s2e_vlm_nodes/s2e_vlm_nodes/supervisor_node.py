from .node_contracts import NodeContract, run_ros_node

STATUS_TOPICS = (
    "/s2e/status/static_tf_node",
    "/s2e/status/lidar_node",
    "/s2e/status/camera_node",
    "/s2e/status/imu_node",
    "/s2e/status/odometry_node",
    "/s2e/status/vlm_node",
    "/s2e/status/e2e_node",
    "/s2e/status/controller_node",
)

NODE_CONTRACT = NodeContract(
    node_name="supervisor_node",
    publishes=("/s2e/supervisor/health", "/s2e/status/supervisor_node"),
    subscribes=STATUS_TOPICS,
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
