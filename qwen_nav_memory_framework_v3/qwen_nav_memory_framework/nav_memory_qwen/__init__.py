"""Qwen navigation memory framework.

This package provides a production-oriented scaffold for a slow VLM semantic
supervisor plus fast navigation-skill backend. It keeps the original
``nav_vlm_waypoint_v1`` input/output envelope while expanding the ``memory``
field into a relative-pose topo-metric episodic graph.
"""

from .agent import NavMemoryAgent, NavAgentConfig, EpisodeResult, StepResult
from .memory_graph import MemoryGraph
from .vlm_client import BaseVLMClient, OpenAICompatibleVLMClient, HeuristicVLMClient
from .robot_backend import ActionOutcome, StaticImageBackend, RobotBackend
try:
    from .ros2_backend import Ros2RobotBackend
except ImportError:
    Ros2RobotBackend = None
from .schema import CoarseGoal, RobotState, Observation, ObservationView, RelativePose2D

__all__ = [
    "NavMemoryAgent",
    "NavAgentConfig",
    "EpisodeResult",
    "StepResult",
    "MemoryGraph",
    "BaseVLMClient",
    "OpenAICompatibleVLMClient",
    "HeuristicVLMClient",
    "ActionOutcome",
    "StaticImageBackend",
    "RobotBackend",
    "Ros2RobotBackend",
    "CoarseGoal",
    "RobotState",
    "Observation",
    "ObservationView",
    "RelativePose2D",
]
