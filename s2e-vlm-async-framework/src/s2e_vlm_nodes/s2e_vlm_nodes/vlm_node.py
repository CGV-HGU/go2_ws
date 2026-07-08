from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="vlm_node",
    publishes=("/s2e/vlm/reasoning", "/s2e/status/vlm_node"),
    subscribes=("/s2e/sensors/camera/image", "/s2e/odometry/pose"),
    actions=("/s2e/controller/rotate",),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
