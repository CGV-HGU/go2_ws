from __future__ import annotations

# pyright: reportMissingImports=false

from collections import defaultdict
from dataclasses import dataclass, field

from s2e_vlm_core.algorithms import NodeStatusData, SystemHealthData, TrajectoryPlan
from s2e_vlm_core.mock_algorithms import MockE2EPlanner, MockOdometryEstimator, MockVlmReasoner, image_goal_to_base_link
from s2e_vlm_core.pose_buffer import Pose2D
from s2e_vlm_core.vlm_schema import VlmAction, parse_vlm_reasoning


@dataclass(frozen=True)
class RotateResult:
    success: bool
    result_code: str
    final_yaw_delta_deg: float
    message: str


@dataclass(frozen=True)
class OverlayFrame:
    labels: list[str]


@dataclass
class TopicBus:
    messages: dict[str, list[object]] = field(default_factory=lambda: defaultdict(list))

    def publish(self, topic: str, message: object) -> None:
        self.messages[topic].append(message)

    def count(self, topic: str) -> int:
        return len(self.messages.get(topic, []))

    def latest(self, topic: str) -> object | None:
        values = self.messages.get(topic, [])
        return values[-1] if values else None


class MockController:
    def __init__(self, bus: TopicBus, *, trajectory_ttl_s: float) -> None:
        self.bus = bus
        self.current_trajectory: TrajectoryPlan | None = None
        self._trajectory_received_at: float | None = None
        self._trajectory_ttl_s = trajectory_ttl_s
        self.status = NodeStatusData("controller_node", "ACTIVE", "WAITING_TRAJECTORY", True, True, 0.0, 0.0)

    def receive_trajectory(self, trajectory: TrajectoryPlan, stamp: float) -> None:
        if trajectory.frame_id != "base_link":
            self.current_trajectory = None
            self._trajectory_received_at = None
            self.status = NodeStatusData("controller_node", "FAULT", "FAULT", False, True, 0.0, 0.0, "FRAME_MISMATCH", "trajectory frame must be base_link")
            self.bus.publish("/s2e/controller/status", self.status)
            return
        self.current_trajectory = trajectory
        self._trajectory_received_at = stamp
        self.status = NodeStatusData("controller_node", "ACTIVE", "FOLLOWING", True, True, 0.0, 0.0)
        self.bus.publish("/s2e/controller/status", self.status)

    def tick(self, stamp: float) -> None:
        if self.current_trajectory is None or self._trajectory_received_at is None:
            return
        if stamp - self._trajectory_received_at > self._trajectory_ttl_s:
            self.current_trajectory = None
            self._trajectory_received_at = None
            self.status = NodeStatusData("controller_node", "ACTIVE", "WAITING_TRAJECTORY", True, True, 0.0, 0.0, "TRAJECTORY_STALE", "trajectory TTL expired")
            self.bus.publish("/s2e/controller/status", self.status)

    def rotate(self, target_yaw_delta_deg: float, timeout_s: float) -> RotateResult:
        del timeout_s
        self.current_trajectory = None
        self.status = NodeStatusData("controller_node", "ACTIVE", "ROTATING", True, True, 0.0, 0.0)
        self.bus.publish("/s2e/controller/status", self.status)
        self.status = NodeStatusData("controller_node", "ACTIVE", "WAITING_TRAJECTORY", True, True, 0.0, 0.0)
        self.bus.publish("/s2e/controller/status", self.status)
        return RotateResult(True, "SUCCESS", target_yaw_delta_deg, "mock rotate complete")


class MockGraphSimulator:
    def __init__(
        self,
        *,
        vlm_period_s: float = 2.0,
        e2e_period_s: float = 0.2,
        first_vlm_delay_s: float = 0.0,
        vlm_scenarios: list[str] | None = None,
        vlm_ttl_s: float = 8.0,
        trajectory_ttl_s: float = 0.5,
    ) -> None:
        self.time_s = 0.0
        self.vlm_period_s = vlm_period_s
        self.e2e_period_s = e2e_period_s
        self.vlm_ttl_s = vlm_ttl_s
        self.next_vlm_s = first_vlm_delay_s
        self.next_e2e_s = 0.0
        self._e2e_enabled = True
        self.bus = TopicBus()
        self.odometry = MockOdometryEstimator()
        self.vlm = MockVlmReasoner(vlm_scenarios or ["go"])
        self.e2e = MockE2EPlanner()
        self.controller = MockController(self.bus, trajectory_ttl_s=trajectory_ttl_s)
        self._last_vlm_payload: str | None = None
        self._vlm_missing = False
        self._health = SystemHealthData(True, "OK", [], [], "")
        self._e2e_status = NodeStatusData("e2e_node", "WAITING_FIRST_VLM", "WAITING_FIRST_VLM", True, True, 0.0, 0.0)

    def run_for(self, duration_s: float) -> None:
        end = self.time_s + duration_s
        while self.time_s <= end + 1e-9:
            current_pose = self.odometry.estimate(self.time_s).pose
            self.controller.tick(self.time_s)
            if not self._vlm_missing and self.time_s >= self.next_vlm_s - 1e-9:
                self._last_vlm_payload = self.vlm.reason(self.time_s, current_pose)
                self.bus.publish("/s2e/vlm/reasoning", self._last_vlm_payload)
                self.next_vlm_s += self.vlm_period_s
            if self._e2e_enabled and self.time_s >= self.next_e2e_s - 1e-9:
                self._run_e2e_cycle(current_pose)
                self.next_e2e_s += self.e2e_period_s
            self.time_s = round(self.time_s + min(self.e2e_period_s, self.vlm_period_s, 0.05), 6)

    def _run_e2e_cycle(self, current_pose: Pose2D) -> None:
        if not self._health.ok_to_move or "vlm_node" in self._health.unhealthy_nodes or "vlm_node" in self._health.missing_critical_nodes:
            self._publish_e2e_status("DEGRADED", "SUPERVISOR_BLOCKED", False, "VLM health unavailable")
            return
        if self._last_vlm_payload is None:
            self._publish_e2e_status("WAITING_FIRST_VLM", "WAITING_FIRST_VLM", True, "No VLM reasoning yet")
            return
        parsed = parse_vlm_reasoning(self._last_vlm_payload)
        if not parsed.valid:
            self._publish_e2e_status("DEGRADED", "INVALID_VLM", False, parsed.reason)
            return
        if parsed.stamp is None or self.time_s - parsed.stamp >= self.vlm_ttl_s:
            self._publish_e2e_status("DEGRADED", "VLM_STALE", False, "Cached VLM reasoning expired")
            return
        if parsed.action == VlmAction.STOP:
            self._publish_e2e_status("STOPPED_BY_VLM", "STOPPED_BY_VLM", True, "VLM requested stop")
            return
        if parsed.action != VlmAction.GO or parsed.goal_uv is None:
            self._publish_e2e_status("DEGRADED", "INVALID_VLM", False, "No trajectory action")
            return
        trajectory = self.e2e.plan(image_goal_to_base_link(parsed.goal_uv), current_pose)
        self.bus.publish("/s2e/e2e/trajectory", trajectory)
        self._publish_e2e_status("ACTIVE", "ACTIVE", True, "Trajectory fresh")
        self.controller.receive_trajectory(trajectory, self.time_s)

    def _publish_e2e_status(self, state: str, active_mode: str, healthy: bool, message: str) -> None:
        self._e2e_status = NodeStatusData("e2e_node", state, active_mode, healthy, True, 0.0, 0.0, message=message)
        self.bus.publish("/s2e/e2e/status", self._e2e_status)

    def mark_vlm_missing(self) -> None:
        self._vlm_missing = True
        self._health = SystemHealthData(False, "DEGRADED", ["vlm_node"], ["vlm_node"], "vlm_node heartbeat missing")
        self.bus.publish("/s2e/supervisor/health", self._health)

    def disable_e2e(self) -> None:
        self._e2e_enabled = False

    def inject_trajectory_with_frame(self, frame_id: str) -> None:
        trajectory = self.e2e.plan((1.0, 0.0), Pose2D(0.0, 0.0, 0.0))
        bad_trajectory = TrajectoryPlan(
            points=trajectory.points,
            goal_point_base_link=trajectory.goal_point_base_link,
            has_goal_point=trajectory.has_goal_point,
            status=trajectory.status,
            frame_id=frame_id,
        )
        self.controller.receive_trajectory(bad_trajectory, self.time_s)

    def count_topic(self, topic: str) -> int:
        return self.bus.count(topic)

    def latest_e2e_status(self) -> NodeStatusData:
        return self._e2e_status

    def latest_controller_status(self) -> NodeStatusData:
        return self.controller.status

    def send_rotate_goal(self, *, target_yaw_delta_deg: float, timeout_s: float) -> RotateResult:
        return self.controller.rotate(target_yaw_delta_deg, timeout_s)

    def render_visualizer_frame(self, *, vlm_payload: str) -> OverlayFrame:
        parsed = parse_vlm_reasoning(vlm_payload)
        labels = ["INVALID_VLM" if not parsed.valid else parsed.action.value]
        return OverlayFrame(labels=labels)
