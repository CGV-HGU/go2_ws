from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="lidar_node",
    publishes=("/s2e/sensors/lidar/points", "/s2e/status/lidar_node"),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
