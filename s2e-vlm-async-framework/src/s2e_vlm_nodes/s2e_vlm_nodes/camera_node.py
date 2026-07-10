from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="camera_node",
    publishes=("/s2e/sensors/camera/image", "/s2e/sensors/camera/camera_info", "/s2e/status/camera_node"),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
