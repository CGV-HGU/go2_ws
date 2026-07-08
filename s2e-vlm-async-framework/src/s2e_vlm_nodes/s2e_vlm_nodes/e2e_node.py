from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="e2e_node",
    publishes=("/s2e/e2e/trajectory", "/s2e/e2e/status", "/s2e/status/e2e_node"),
    subscribes=("/s2e/sensors/camera/image", "/s2e/odometry/pose", "/s2e/vlm/reasoning", "/s2e/supervisor/health"),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
