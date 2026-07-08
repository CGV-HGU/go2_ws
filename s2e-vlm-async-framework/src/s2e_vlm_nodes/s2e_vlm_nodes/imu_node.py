from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="imu_node",
    publishes=("/s2e/sensors/imu", "/s2e/status/imu_node"),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
