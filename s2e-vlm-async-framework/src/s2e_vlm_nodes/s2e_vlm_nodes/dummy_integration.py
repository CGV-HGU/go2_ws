from __future__ import annotations

# pyright: reportMissingImports=false

from collections import defaultdict
from dataclasses import dataclass, field
from typing import Mapping

from .camera_node import NODE_CONTRACT as CAMERA_CONTRACT
from .controller_node import NODE_CONTRACT as CONTROLLER_CONTRACT
from .debug_visualizer_node import NODE_CONTRACT as DEBUG_VISUALIZER_CONTRACT
from .e2e_node import NODE_CONTRACT as E2E_CONTRACT
from .imu_node import NODE_CONTRACT as IMU_CONTRACT
from .lidar_node import NODE_CONTRACT as LIDAR_CONTRACT
from .node_contracts import NodeContract
from .odometry_node import NODE_CONTRACT as ODOMETRY_CONTRACT
from .static_tf_node import NODE_CONTRACT as STATIC_TF_CONTRACT
from .supervisor_node import NODE_CONTRACT as SUPERVISOR_CONTRACT
from .vlm_node import NODE_CONTRACT as VLM_CONTRACT

NODE_EXECUTION_ORDER = (
    STATIC_TF_CONTRACT,
    LIDAR_CONTRACT,
    CAMERA_CONTRACT,
    IMU_CONTRACT,
    ODOMETRY_CONTRACT,
    SUPERVISOR_CONTRACT,
    VLM_CONTRACT,
    E2E_CONTRACT,
    CONTROLLER_CONTRACT,
    DEBUG_VISUALIZER_CONTRACT,
)

DEFAULT_PERIODS_S: Mapping[str, float] = {
    "static_tf_node": 1.00,
    "imu_node": 0.05,
    "controller_node": 0.05,
    "lidar_node": 0.10,
    "odometry_node": 0.10,
    "camera_node": 0.20,
    "e2e_node": 0.20,
    "debug_visualizer_node": 0.25,
    "supervisor_node": 0.50,
    "vlm_node": 1.00,
}

DEFAULT_TRIAL_SCENARIOS = (
    "waiting_first_vlm",
    "normal_go",
    "stale_inputs",
    "faults",
    "vlm_stop",
    "supervisor_blocked",
    "rotate",
    "visualizer_degraded",
    "waiting_inputs",
)

DEFAULT_STATE_COVERAGE: Mapping[str, frozenset[str]] = {
    "static_tf_node": frozenset({"INIT", "ACTIVE", "FAULT"}),
    "lidar_node": frozenset({"INIT", "ACTIVE", "STALE_INPUT", "FAULT"}),
    "camera_node": frozenset({"INIT", "ACTIVE", "STALE_INPUT", "FAULT"}),
    "imu_node": frozenset({"INIT", "ACTIVE", "STALE_INPUT", "FAULT"}),
    "odometry_node": frozenset({"INIT", "WAITING_INPUTS", "ACTIVE", "DEGRADED", "FAULT"}),
    "vlm_node": frozenset({"INIT", "WAITING_SYNC", "ACTIVE", "FROZEN_ROTATING", "STALE_INPUT", "FAULT"}),
    "e2e_node": frozenset({"INIT", "WAITING_FIRST_VLM", "ACTIVE", "STOPPED_BY_VLM", "DEGRADED", "FAULT"}),
    "controller_node": frozenset({"INIT", "WAITING_TRAJECTORY", "FOLLOWING", "ROTATING", "STOPPING", "DEGRADED", "FAULT"}),
    "debug_visualizer_node": frozenset({"INIT", "ACTIVE", "DEGRADED", "FAULT"}),
    "supervisor_node": frozenset({"INIT", "ACTIVE", "DEGRADED", "FAULT"}),
}


@dataclass(frozen=True)
class DummyMessage:
    topic: str
    node_name: str
    trial_index: int
    scenario: str
    stamp_s: float
    state: str


@dataclass
class DummyIntegrationResult:
    trial_count: int
    topic_counts: dict[str, int] = field(default_factory=dict)
    published_by_node: dict[str, dict[str, int]] = field(default_factory=dict)
    consumed_by_node: dict[str, dict[str, int]] = field(default_factory=dict)
    actions_by_node: dict[str, dict[str, int]] = field(default_factory=dict)
    ticks_by_node: dict[str, list[float]] = field(default_factory=dict)
    states_by_node: dict[str, set[str]] = field(default_factory=dict)
    contracts: tuple[NodeContract, ...] = NODE_EXECUTION_ORDER

    def tick_count(self, node_name: str) -> int:
        return len(self.ticks_by_node.get(node_name, []))

    def missing_publish_topics(self) -> dict[str, list[str]]:
        missing: dict[str, list[str]] = {}
        for contract in self.contracts:
            node_missing = [topic for topic in contract.publishes if self.published_by_node.get(contract.node_name, {}).get(topic, 0) == 0]
            if node_missing:
                missing[contract.node_name] = node_missing
        return missing

    def missing_subscribe_topics(self) -> dict[str, list[str]]:
        missing: dict[str, list[str]] = {}
        for contract in self.contracts:
            node_missing = [topic for topic in contract.subscribes if self.consumed_by_node.get(contract.node_name, {}).get(topic, 0) == 0]
            if node_missing:
                missing[contract.node_name] = node_missing
        return missing

    def missing_action_interfaces(self) -> dict[str, list[str]]:
        missing: dict[str, list[str]] = {}
        for contract in self.contracts:
            node_missing = [action for action in contract.actions if self.actions_by_node.get(contract.node_name, {}).get(action, 0) == 0]
            if node_missing:
                missing[contract.node_name] = node_missing
        return missing

    def missing_states(self, expected: Mapping[str, frozenset[str]]) -> dict[str, list[str]]:
        missing: dict[str, list[str]] = {}
        for node_name, expected_states in expected.items():
            observed = self.states_by_node.get(node_name, set())
            node_missing = sorted(expected_states - observed)
            if node_missing:
                missing[node_name] = node_missing
        return missing


class DummyIntegrationRunner:
    def __init__(self, *, periods_s: Mapping[str, float] = DEFAULT_PERIODS_S) -> None:
        self.periods_s = periods_s
        self._latest_by_topic: dict[str, DummyMessage] = {}
        self._topic_counts: dict[str, int] = defaultdict(int)
        self._published_by_node: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
        self._consumed_by_node: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
        self._actions_by_node: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
        self._ticks_by_node: dict[str, list[float]] = defaultdict(list)
        self._states_by_node: dict[str, set[str]] = defaultdict(set)

    def run(self, *, trial_count: int, duration_s: float) -> DummyIntegrationResult:
        if trial_count < 1:
            raise ValueError("trial_count must be at least 1")
        for trial_index in range(trial_count):
            scenario = DEFAULT_TRIAL_SCENARIOS[trial_index % len(DEFAULT_TRIAL_SCENARIOS)]
            self._record_init_states()
            self._run_trial(trial_index=trial_index, scenario=scenario, duration_s=duration_s)
        return DummyIntegrationResult(
            trial_count=trial_count,
            topic_counts={topic: count for topic, count in self._topic_counts.items()},
            published_by_node={node: dict(counts) for node, counts in self._published_by_node.items()},
            consumed_by_node={node: dict(counts) for node, counts in self._consumed_by_node.items()},
            actions_by_node={node: dict(counts) for node, counts in self._actions_by_node.items()},
            ticks_by_node={node: list(times) for node, times in self._ticks_by_node.items()},
            states_by_node={node: set(states) for node, states in self._states_by_node.items()},
        )

    def _record_init_states(self) -> None:
        for contract in NODE_EXECUTION_ORDER:
            self._states_by_node[contract.node_name].add("INIT")

    def _run_trial(self, *, trial_index: int, scenario: str, duration_s: float) -> None:
        next_tick_s = {contract.node_name: 0.0 for contract in NODE_EXECUTION_ORDER}
        time_s = 0.0
        while time_s <= duration_s + 1e-9:
            for contract in NODE_EXECUTION_ORDER:
                if time_s >= next_tick_s[contract.node_name] - 1e-9:
                    self._tick_node(contract, trial_index=trial_index, scenario=scenario, time_s=time_s)
                    next_tick_s[contract.node_name] += self.periods_s[contract.node_name]
            time_s = round(time_s + 0.01, 6)

    def _tick_node(self, contract: NodeContract, *, trial_index: int, scenario: str, time_s: float) -> None:
        state = self._state_for(contract.node_name, scenario)
        self._ticks_by_node[contract.node_name].append(time_s)
        self._states_by_node[contract.node_name].add(state)
        for topic in contract.subscribes:
            if topic in self._latest_by_topic:
                self._consumed_by_node[contract.node_name][topic] += 1
        for topic in contract.publishes:
            self._publish(topic, contract.node_name, trial_index, scenario, time_s, state)
        if scenario == "rotate" and contract.actions:
            for action in contract.actions:
                self._actions_by_node[contract.node_name][action] += 1

    def _publish(self, topic: str, node_name: str, trial_index: int, scenario: str, time_s: float, state: str) -> None:
        message = DummyMessage(topic, node_name, trial_index, scenario, time_s, state)
        self._latest_by_topic[topic] = message
        self._topic_counts[topic] += 1
        self._published_by_node[node_name][topic] += 1

    def _state_for(self, node_name: str, scenario: str) -> str:
        if node_name == "static_tf_node":
            if scenario == "faults":
                return "FAULT"
            return "ACTIVE"
        if node_name in {"lidar_node", "camera_node", "imu_node"}:
            if scenario == "stale_inputs":
                return "STALE_INPUT"
            if scenario == "faults":
                return "FAULT"
            return "ACTIVE"
        if node_name == "odometry_node":
            if scenario == "waiting_inputs":
                return "WAITING_INPUTS"
            if scenario == "stale_inputs":
                return "DEGRADED"
            if scenario == "faults":
                return "FAULT"
            return "ACTIVE"
        if node_name == "vlm_node":
            if scenario == "waiting_inputs":
                return "WAITING_SYNC"
            if scenario == "rotate":
                return "FROZEN_ROTATING"
            if scenario == "stale_inputs":
                return "STALE_INPUT"
            if scenario == "faults":
                return "FAULT"
            return "ACTIVE"
        if node_name == "e2e_node":
            if scenario == "waiting_first_vlm":
                return "WAITING_FIRST_VLM"
            if scenario == "vlm_stop":
                return "STOPPED_BY_VLM"
            if scenario in {"stale_inputs", "supervisor_blocked"}:
                return "DEGRADED"
            if scenario == "faults":
                return "FAULT"
            return "ACTIVE"
        if node_name == "controller_node":
            if scenario == "waiting_first_vlm":
                return "WAITING_TRAJECTORY"
            if scenario == "rotate":
                return "ROTATING"
            if scenario == "vlm_stop":
                return "STOPPING"
            if scenario == "supervisor_blocked":
                return "DEGRADED"
            if scenario == "faults":
                return "FAULT"
            return "FOLLOWING"
        if node_name == "debug_visualizer_node":
            if scenario in {"stale_inputs", "visualizer_degraded"}:
                return "DEGRADED"
            if scenario == "faults":
                return "FAULT"
            return "ACTIVE"
        if node_name == "supervisor_node":
            if scenario == "supervisor_blocked":
                return "DEGRADED"
            if scenario == "faults":
                return "FAULT"
            return "ACTIVE"
        raise ValueError(f"Unknown node: {node_name}")


def run_dummy_integration_trials(*, trial_count: int, duration_s: float) -> DummyIntegrationResult:
    return DummyIntegrationRunner().run(trial_count=trial_count, duration_s=duration_s)
