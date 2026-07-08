from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from .pose_buffer import Pose2D


@dataclass(frozen=True)
class StampedPoseData:
    stamp: float
    pose: Pose2D
    frame_id: str = "odom"
    child_frame_id: str = "base_link"
    confidence: float = 1.0
    status: str = "OK"


@dataclass(frozen=True)
class TrajectoryPlan:
    points: list[tuple[float, float]]
    goal_point_base_link: tuple[float, float]
    has_goal_point: bool
    status: str
    frame_id: str = "base_link"


@dataclass(frozen=True)
class NodeStatusData:
    node_name: str
    state: str
    active_mode: str
    is_healthy: bool
    is_motion_critical: bool
    last_input_age_s: float
    last_output_age_s: float
    error_code: str = ""
    message: str = ""


@dataclass(frozen=True)
class SystemHealthData:
    ok_to_move: bool
    overall_state: str
    unhealthy_nodes: list[str]
    missing_critical_nodes: list[str]
    reason: str


class OdometryEstimator(Protocol):
    def estimate(self, stamp: float) -> StampedPoseData:
        ...


class VlmReasoner(Protocol):
    def reason(self, stamp: float, pose: Pose2D) -> str:
        ...


class E2EPlanner(Protocol):
    def plan(self, goal_point_base_link: tuple[float, float], current_pose: Pose2D) -> TrajectoryPlan:
        ...
