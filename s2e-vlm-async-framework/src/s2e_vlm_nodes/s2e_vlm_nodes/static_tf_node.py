from .node_contracts import NodeContract, run_ros_node

NODE_CONTRACT = NodeContract(
    node_name="static_tf_node",
    publishes=("/tf_static", "/s2e/status/static_tf_node"),
)


def main(args=None):
    run_ros_node(NODE_CONTRACT, args)
