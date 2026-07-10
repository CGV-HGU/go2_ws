from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="controller_node",
    publishes=("/s2e/controller/command", "/s2e/controller/status", "/s2e/status/controller_node"),
    subscribes=("/s2e/e2e/trajectory", "/s2e/odometry/pose", "/s2e/e2e/status", "/s2e/supervisor/health"),
    actions=("/s2e/controller/rotate",),
    motion_authority=True,
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
