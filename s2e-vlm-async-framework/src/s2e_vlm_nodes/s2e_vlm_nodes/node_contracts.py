from __future__ import annotations

# pyright: reportMissingImports=false

from dataclasses import dataclass, field


@dataclass(frozen=True)
class NodeContract:
    node_name: str
    publishes: tuple[str, ...] = field(default_factory=tuple)
    subscribes: tuple[str, ...] = field(default_factory=tuple)
    actions: tuple[str, ...] = field(default_factory=tuple)
    motion_authority: bool = False


def run_ros_node(contract: NodeContract, args: list[str] | None = None) -> None:
    try:
        from .ros_mock_runtime import run_mock_ros_node
    except ImportError as exc:
        raise RuntimeError("ROS 2 rclpy is required to run this node") from exc
    run_mock_ros_node(contract, args)
