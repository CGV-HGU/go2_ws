from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="odometry_node",
    publishes=("/s2e/odometry/pose", "/s2e/status/odometry_node"),
    subscribes=("/s2e/sensors/lidar/points", "/s2e/sensors/camera/image", "/s2e/sensors/imu"),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
