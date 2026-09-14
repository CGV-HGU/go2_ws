"""Habitat ObjectNav/PointNav lifecycle isolated behind a private Unix socket."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import socket
import threading
import uuid
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Callable, Dict, Mapping, Optional, Sequence, Tuple

import numpy as np

from s2e_vlm_core.habitat_actions import (
    HabitatAction,
    HabitatActionError,
    HabitatStepBudget,
)
from s2e_vlm_core.execution import (
    ExecutionContractError,
    ExecutionPose,
    HabitatActionResult,
    HabitatCommand,
    MAX_FORWARD_DISTANCE_M,
    UINT32_MAX,
)

from .protocol import (
    MAX_CAUSAL_ID_BYTES,
    MAX_ERROR_CODE_BYTES,
    MAX_ERROR_MESSAGE_BYTES,
    MAX_PAYLOAD_BYTES,
    MAX_REQUEST_ID_BYTES,
    HabitatProtocolError,
    bounded_metadata_value,
    preflight_packet,
    recv_packet,
    send_packet,
    truncate_utf8,
    validate_bounded_text,
)
from .room_regions import RoomRegionMapStore


ACTION_VALUES = frozenset(
    {
        "hold",
        "stop",
        "move_forward",
        "move_backward",
        "turn_left",
        "turn_right",
        "look_up",
        "look_down",
    }
)
REFERENCE_ACTION_IDS = MappingProxyType(
    {
        "stop": 0,
        "move_forward": 1,
        "turn_left": 2,
        "turn_right": 3,
        "look_up": 4,
        "look_down": 5,
    }
)
DEFAULT_COMMAND_REPLAY_CAPACITY = 16
DEFAULT_COMMAND_HISTORY_CAPACITY = 4096
DEFAULT_MAX_TOPDOWN_PAYLOAD_BYTES = 16 * 1024 * 1024
REFERENCE_EFFECTIVE_LOOK_ACTION_DEG = 30
ACTION_RESULT_METADATA_RESERVE_BYTES = 64 * 1024
EVALUATION_METRIC_KEYS = (
    "distance_to_goal",
    "success",
    "spl",
    "soft_spl",
    "num_steps",
    "collisions",
)
POINTNAV_REQUIRED_EVALUATION_METRICS = (
    "distance_to_goal",
    "success",
    "spl",
    "soft_spl",
)
OBJECTNAV_TASK_KEY = "objectnav"
POINTNAV_TASK_KEY = "pointnav"
OBJECTNAV_TASK_TYPE = "ObjectNav"
POINTNAV_TASK_TYPE = "PointNav"
POINTGOAL_SENSOR_UUID = "pointgoal_with_gps_compass"
POINTNAV_ROOM_REGION_MAP_DIR = "/data/room_region_maps"
POINTNAV_TARGET_LABEL = "pointgoal"
POINTNAV_SUCCESS_DISTANCE_M = 1.0
POINTNAV_DEFAULT_MAX_EPISODE_STEPS = 500
POINTNAV_DEFAULT_CAMERA_HFOV_DEG = 90


class HabitatWorkerError(ValueError):
    def __init__(self, code: str, detail: str = "") -> None:
        self.code = code
        self.detail = detail
        message = code if not detail else "{}: {}".format(code, detail)
        super().__init__(message)


@dataclass(frozen=True)
class HabitatTask:
    episode_id: str
    scene_id: str
    task_type: str
    target_object: str


@dataclass(frozen=True)
class HabitatBenchmarkSettings:
    task_key: str
    task_type: str
    config_path: str
    dataset_path: str
    scenes_dir: str
    scene_dataset_config: str
    success_distance_m: float
    allow_sliding: bool
    camera_hfov_deg: int
    max_episode_steps: int


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    hfov_deg: float
    camera_matrix_row_major: Tuple[float, ...]


@dataclass(frozen=True)
class HabitatObservation:
    task: HabitatTask
    rgb: np.ndarray
    depth: np.ndarray
    camera: CameraModel
    position_xyz: Tuple[float, float, float]
    rotation_xyzw: Tuple[float, float, float, float]
    collided: bool
    episode_over: bool
    policy_metadata: Mapping[str, object]
    evaluation_metrics: Mapping[str, object]
    topdown_rgb: Optional[np.ndarray] = None
    artifact_metadata: Mapping[str, object] = field(default_factory=dict)
    # Evaluator-only precomputed coordinate-map evidence. This field is
    # intentionally omitted from the worker protocol so policies cannot use it.
    room_region_id: Optional[int] = None


@dataclass(frozen=True)
class HabitatCommandExecution:
    """One immutable causal result paired with its exact post-step observation."""

    result: HabitatActionResult
    observation: HabitatObservation


@dataclass(frozen=True)
class _CommandHistoryEntry:
    command: HabitatCommand
    state: str
    error_code: str = ""
    error_detail: str = ""


def _freeze_value(value: object) -> object:
    if isinstance(value, Mapping):
        return MappingProxyType(
            {str(key): _freeze_value(item) for key, item in value.items()}
        )
    if isinstance(value, (list, tuple)):
        return tuple(_freeze_value(item) for item in value)
    return value


def _freeze_mapping(value: Mapping[str, object]) -> Mapping[str, object]:
    projected = _bounded_observation_metadata(value)
    if not isinstance(projected, Mapping):
        raise HabitatWorkerError("INVALID_OBSERVATION_METADATA")
    return MappingProxyType(
        {str(key): _freeze_value(item) for key, item in projected.items()}
    )


def _array_debug_ref(array: np.ndarray) -> str:
    value = np.ascontiguousarray(array)
    payload = json.dumps(
        {
            "dtype": str(value.dtype),
            "shape": list(value.shape),
            "bytes_sha256": hashlib.sha256(value.tobytes()).hexdigest(),
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return "sha256:" + hashlib.sha256(payload).hexdigest()


def _json_native_value(value: object) -> object:
    return _bounded_observation_metadata(value)


def _bounded_observation_metadata(value: object) -> object:
    try:
        return bounded_metadata_value(value)
    except HabitatProtocolError as error:
        raise HabitatWorkerError(error.code, error.detail) from error


def _bounded_worker_text(
    value: object,
    code: str,
    maximum_bytes: int,
    *,
    empty_allowed: bool = False,
) -> str:
    try:
        return validate_bounded_text(
            value,
            maximum_bytes=maximum_bytes,
            empty_allowed=empty_allowed,
            code=code,
        )
    except HabitatProtocolError as error:
        raise HabitatWorkerError(error.code, error.detail) from error


def _readonly_array(value: np.ndarray) -> np.ndarray:
    contiguous = np.ascontiguousarray(value)
    return np.frombuffer(
        contiguous.tobytes(order="C"),
        dtype=contiguous.dtype,
    ).reshape(contiguous.shape)


def _immutable_observation(value: HabitatObservation) -> HabitatObservation:
    _observation_packet_description(value)
    return HabitatObservation(
        task=value.task,
        rgb=_readonly_array(value.rgb),
        depth=_readonly_array(value.depth),
        camera=value.camera,
        position_xyz=value.position_xyz,
        rotation_xyzw=value.rotation_xyzw,
        collided=value.collided,
        episode_over=value.episode_over,
        policy_metadata=_freeze_mapping(value.policy_metadata),
        evaluation_metrics=_freeze_mapping(value.evaluation_metrics),
        topdown_rgb=(
            None if value.topdown_rgb is None else _readonly_array(value.topdown_rgb)
        ),
        artifact_metadata=_freeze_mapping(value.artifact_metadata),
        room_region_id=value.room_region_id,
    )


def _stable_command_error(error: Exception) -> HabitatWorkerError:
    if isinstance(error, HabitatWorkerError):
        return HabitatWorkerError(error.code, error.detail)
    return HabitatWorkerError(
        "COMMAND_EXECUTION_FAILED",
        "{}: {}".format(type(error).__name__, error),
    )


def habitat_quaternion_xyzw(rotation: object) -> Tuple[float, float, float, float]:
    if all(hasattr(rotation, field) for field in ("x", "y", "z", "w")):
        values = tuple(
            float(getattr(rotation, field)) for field in ("x", "y", "z", "w")
        )
    elif hasattr(rotation, "imag") and hasattr(rotation, "real"):
        imaginary = getattr(rotation, "imag")
        values = (
            float(imaginary[0]),
            float(imaginary[1]),
            float(imaginary[2]),
            float(getattr(rotation, "real")),
        )
    else:
        try:
            sequence = tuple(float(value) for value in rotation)  # type: ignore[arg-type]
        except (TypeError, ValueError, OverflowError) as error:
            raise HabitatWorkerError("INVALID_ROTATION", str(error)) from error
        if len(sequence) != 4:
            raise HabitatWorkerError("INVALID_ROTATION")
        values = sequence
    if not all(math.isfinite(value) for value in values):
        raise HabitatWorkerError("NON_FINITE_ROTATION")
    norm = math.sqrt(sum(value * value for value in values))
    if norm <= 0.0:
        raise HabitatWorkerError("INVALID_ROTATION")
    return tuple(value / norm for value in values)  # type: ignore[return-value]


def camera_model(width: int, height: int, hfov_deg: float) -> CameraModel:
    if (
        width <= 0
        or height <= 0
        or not math.isfinite(hfov_deg)
        or not 0.0 < hfov_deg < 180.0
    ):
        raise HabitatWorkerError("INVALID_CAMERA_MODEL")
    focal = (float(width) / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    cx = (float(width) - 1.0) / 2.0
    cy = (float(height) - 1.0) / 2.0
    return CameraModel(
        width,
        height,
        float(hfov_deg),
        (focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0),
    )


def habitat_sensor_hfov(value: str) -> int:
    """Parse Habitat-Lab's integer-valued sensor HFOV config field."""
    hfov = int(value)
    if not 0 < hfov < 180:
        raise ValueError("HABITAT_CAMERA_HFOV_DEG must be between 0 and 180")
    return hfov


def habitat_turn_angle(value: str) -> int:
    """Parse Habitat-Lab's integer-valued discrete turn angle."""
    angle = int(value)
    if not 0 < angle <= 180:
        raise ValueError("HABITAT_TURN_ANGLE_DEG must be between 0 and 180")
    return angle


def habitat_look_angle_from_environment(
    environ: Optional[Mapping[str, str]] = None,
) -> int:
    environment = os.environ if environ is None else environ
    return habitat_turn_angle(
        environment.get(
            "HABITAT_LOOK_ANGLE_DEG",
            str(REFERENCE_EFFECTIVE_LOOK_ACTION_DEG),
        )
    )


def _environment_flag(
    environment: Mapping[str, str],
    name: str,
    default: str,
) -> bool:
    return str(environment.get(name, default)).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _positive_environment_int(
    environment: Mapping[str, str],
    name: str,
    default: int,
) -> int:
    try:
        value = int(environment.get(name, str(default)))
    except (TypeError, ValueError) as error:
        raise HabitatWorkerError("INVALID_{}".format(name)) from error
    if value <= 0:
        raise HabitatWorkerError("INVALID_{}".format(name))
    return value


def habitat_benchmark_settings(
    package_dir: Path,
    *,
    environ: Optional[Mapping[str, str]] = None,
) -> HabitatBenchmarkSettings:
    """Resolve one explicit Habitat benchmark without changing ObjectNav defaults."""

    environment = os.environ if environ is None else environ
    task_key = str(
        environment.get("HABITAT_TASK_TYPE", OBJECTNAV_TASK_KEY)
    ).strip().lower()
    if task_key not in {OBJECTNAV_TASK_KEY, POINTNAV_TASK_KEY}:
        raise HabitatWorkerError(
            "INVALID_HABITAT_TASK_TYPE",
            "expected objectnav or pointnav, found {!r}".format(task_key),
        )

    scenes_dir = str(
        environment.get("HABITAT_SCENES_DIR", "/data/scene_datasets/")
    )
    episodes_dir = str(
        environment.get("HABITAT_EPISODES_DIR", "/data/datasets/")
    )
    scene_dataset_config = str(
        environment.get("HABITAT_SCENE_DATASET_CONFIG_PATH")
        or os.path.join(
            scenes_dir,
            "hm3d_v0.2/hm3d_annotated_basis.scene_dataset_config.json",
        )
    )

    if task_key == POINTNAV_TASK_KEY:
        config_path = str(
            environment.get("HABITAT_POINTNAV_CONFIG_PATH")
            or environment.get("HABITAT_CONFIG_PATH")
            or package_dir / "config/benchmark/nav/pointnav/pointnav_hm3d.yaml"
        )
        dataset_path = str(
            environment.get("HABITAT_POINTNAV_DATA_PATH")
            or os.path.join(
                episodes_dir,
                "pointnav/hm3d/v1/{split}/{split}.json.gz",
            )
        )
        try:
            success_distance_m = float(
                environment.get(
                    "HABITAT_POINTNAV_SUCCESS_DISTANCE_M",
                    str(POINTNAV_SUCCESS_DISTANCE_M),
                )
            )
        except (TypeError, ValueError) as error:
            raise HabitatWorkerError(
                "INVALID_HABITAT_POINTNAV_SUCCESS_DISTANCE_M"
            ) from error
        if not math.isfinite(success_distance_m) or success_distance_m <= 0.0:
            raise HabitatWorkerError("INVALID_HABITAT_POINTNAV_SUCCESS_DISTANCE_M")
        allow_sliding = _environment_flag(
            environment,
            "HABITAT_POINTNAV_ALLOW_SLIDING",
            "0",
        )
        if allow_sliding:
            raise HabitatWorkerError(
                "INVALID_HABITAT_POINTNAV_ALLOW_SLIDING",
                "official HM3D PointNav requires false",
            )
        try:
            camera_hfov_deg = habitat_sensor_hfov(
                str(
                    environment.get(
                        "HABITAT_POINTNAV_CAMERA_HFOV_DEG",
                        str(POINTNAV_DEFAULT_CAMERA_HFOV_DEG),
                    )
                )
            )
        except (TypeError, ValueError) as error:
            raise HabitatWorkerError(
                "INVALID_HABITAT_POINTNAV_CAMERA_HFOV_DEG"
            ) from error
        if camera_hfov_deg != POINTNAV_DEFAULT_CAMERA_HFOV_DEG:
            raise HabitatWorkerError(
                "INVALID_HABITAT_POINTNAV_CAMERA_HFOV_DEG",
                "official HM3D PointNav requires 90 degrees",
            )
        max_episode_steps = _positive_environment_int(
            environment,
            "HABITAT_POINTNAV_MAX_EPISODE_STEPS",
            POINTNAV_DEFAULT_MAX_EPISODE_STEPS,
        )
        task_type = POINTNAV_TASK_TYPE
    else:
        config_path = str(
            environment.get("HABITAT_CONFIG_PATH")
            or package_dir / "config/benchmark/nav/objectnav/objectnav_hm3d.yaml"
        )
        dataset_path = str(
            environment.get("HABITAT_OBJECTNAV_DATA_PATH")
            or os.path.join(
                episodes_dir,
                "objectnav/hm3d/v2/{split}/{split}.json.gz",
            )
        )
        try:
            success_distance_m = float(
                environment.get("HABITAT_SUCCESS_DISTANCE_M", "1.0")
            )
        except (TypeError, ValueError) as error:
            raise HabitatWorkerError("INVALID_HABITAT_SUCCESS_DISTANCE_M") from error
        if not math.isfinite(success_distance_m) or success_distance_m <= 0.0:
            raise HabitatWorkerError("INVALID_HABITAT_SUCCESS_DISTANCE_M")
        allow_sliding = _environment_flag(
            environment,
            "HABITAT_ALLOW_SLIDING",
            "0",
        )
        camera_hfov_deg = habitat_sensor_hfov(
            str(environment.get("HABITAT_CAMERA_HFOV_DEG", "79"))
        )
        max_episode_steps = _positive_environment_int(
            environment,
            "HABITAT_MAX_EPISODE_STEPS",
            400,
        )
        task_type = OBJECTNAV_TASK_TYPE

    return HabitatBenchmarkSettings(
        task_key=task_key,
        task_type=task_type,
        config_path=config_path,
        dataset_path=dataset_path,
        scenes_dir=scenes_dir,
        scene_dataset_config=scene_dataset_config,
        success_distance_m=success_distance_m,
        allow_sliding=allow_sliding,
        camera_hfov_deg=camera_hfov_deg,
        max_episode_steps=max_episode_steps,
    )


def _pointgoal_observation(raw: Mapping[str, np.ndarray]) -> list[float]:
    if POINTGOAL_SENSOR_UUID not in raw:
        raise HabitatWorkerError("MISSING_POINTGOAL_OBSERVATION")
    try:
        values = np.asarray(raw[POINTGOAL_SENSOR_UUID], dtype=np.float64)
    except (TypeError, ValueError, OverflowError) as error:
        raise HabitatWorkerError("INVALID_POINTGOAL_OBSERVATION") from error
    if (
        values.shape != (2,)
        or not bool(np.isfinite(values).all())
        or float(values[0]) < 0.0
    ):
        raise HabitatWorkerError("INVALID_POINTGOAL_OBSERVATION")
    return [float(values[0]), float(values[1])]


def _episode_artifact_metadata(episode: object) -> Dict[str, object]:
    """Extract evaluator evidence without exposing it through policy metadata."""
    positions: list[list[float]] = []
    raw_goals = getattr(episode, "goals", ())
    if isinstance(raw_goals, (list, tuple)):
        for goal in raw_goals:
            raw_position = getattr(goal, "position", None)
            if not isinstance(raw_position, (list, tuple, np.ndarray)):
                continue
            try:
                position = [float(value) for value in raw_position]
            except (TypeError, ValueError, OverflowError):
                continue
            if len(position) == 3 and all(math.isfinite(value) for value in position):
                positions.append(position)
    return {
        "scene_id": str(getattr(episode, "scene_id", "")),
        "goal_positions": positions,
    }


class HabitatWorker:
    def __init__(
        self,
        env: object,
        *,
        camera_hfov_deg: float = 79.0,
        initial_episode_index: int = 0,
        topdown_renderer: Optional[Callable[[object, int], np.ndarray]] = None,
        topdown_height: int = 1024,
        max_topdown_payload_bytes: int = DEFAULT_MAX_TOPDOWN_PAYLOAD_BYTES,
        max_turn_angle_deg: float = 30.0,
        max_forward_distance_m: float = MAX_FORWARD_DISTANCE_M,
        max_episode_steps: int = 400,
        reserved_stop_steps: int = 1,
        command_replay_capacity: int = DEFAULT_COMMAND_REPLAY_CAPACITY,
        command_history_capacity: int | None = None,
        dispatch_reference_action_ids: bool = False,
        task_type: str = OBJECTNAV_TASK_TYPE,
        room_region_maps: Optional[RoomRegionMapStore] = None,
    ) -> None:
        if initial_episode_index < 0:
            raise HabitatWorkerError("INVALID_EPISODE_INDEX")
        if topdown_height <= 0:
            raise HabitatWorkerError("INVALID_TOPDOWN_HEIGHT")
        if (
            not isinstance(max_topdown_payload_bytes, int)
            or isinstance(max_topdown_payload_bytes, bool)
            or max_topdown_payload_bytes <= 0
            or max_topdown_payload_bytes > MAX_PAYLOAD_BYTES
        ):
            raise HabitatWorkerError("INVALID_MAX_TOPDOWN_PAYLOAD_BYTES")
        if (
            isinstance(max_turn_angle_deg, bool)
            or not math.isfinite(float(max_turn_angle_deg))
            or float(max_turn_angle_deg) <= 0.0
            or float(max_turn_angle_deg) > 180.0
        ):
            raise HabitatWorkerError("INVALID_MAX_TURN_ANGLE")
        try:
            maximum_forward_distance = float(max_forward_distance_m)
        except (TypeError, ValueError, OverflowError) as error:
            raise HabitatWorkerError("INVALID_MAX_FORWARD_DISTANCE") from error
        if (
            isinstance(max_forward_distance_m, bool)
            or not math.isfinite(maximum_forward_distance)
            or maximum_forward_distance <= 0.0
            or maximum_forward_distance > MAX_FORWARD_DISTANCE_M
        ):
            raise HabitatWorkerError("INVALID_MAX_FORWARD_DISTANCE")
        try:
            step_budget = HabitatStepBudget(
                max_episode_steps=max_episode_steps,
                reserved_stop_steps=reserved_stop_steps,
            )
        except HabitatActionError as error:
            raise HabitatWorkerError(error.code, error.detail) from error
        if (
            not isinstance(command_replay_capacity, int)
            or isinstance(command_replay_capacity, bool)
            or command_replay_capacity <= 0
        ):
            raise HabitatWorkerError("INVALID_COMMAND_REPLAY_CAPACITY")
        if command_history_capacity is None:
            resolved_command_history_capacity = max(
                DEFAULT_COMMAND_HISTORY_CAPACITY,
                step_budget.max_episode_steps + 1,
            )
        elif (
            not isinstance(command_history_capacity, int)
            or isinstance(command_history_capacity, bool)
            or command_history_capacity <= 0
        ):
            raise HabitatWorkerError("INVALID_COMMAND_HISTORY_CAPACITY")
        else:
            resolved_command_history_capacity = command_history_capacity
        if command_replay_capacity > resolved_command_history_capacity:
            raise HabitatWorkerError("INVALID_COMMAND_REPLAY_CAPACITY")
        if task_type not in {OBJECTNAV_TASK_TYPE, POINTNAV_TASK_TYPE}:
            raise HabitatWorkerError("INVALID_HABITAT_TASK_TYPE")
        self.env = env
        self.camera_hfov_deg = float(camera_hfov_deg)
        self.initial_episode_index = int(initial_episode_index)
        self.task_type = task_type
        self.room_region_maps = room_region_maps
        self.topdown_renderer = topdown_renderer
        self.topdown_height = int(topdown_height)
        self.max_topdown_payload_bytes = (
            int(max_topdown_payload_bytes) if topdown_renderer is not None else 0
        )
        self.max_turn_angle_deg = float(max_turn_angle_deg)
        self.max_forward_distance_m = maximum_forward_distance
        self.step_budget = step_budget
        self.command_replay_capacity = command_replay_capacity
        self.command_history_capacity = resolved_command_history_capacity
        self.dispatch_reference_action_ids = bool(dispatch_reference_action_ids)
        self._first_reset = True
        self._step_index = 0
        self._last_raw_observation: Optional[Mapping[str, np.ndarray]] = None
        self._sensor_layout: Optional[
            Tuple[Tuple[int, ...], str, Tuple[int, ...], str]
        ] = None
        self._sensor_payload_bytes: Optional[int] = None
        self._command_lock = threading.Lock()
        self._command_results: OrderedDict[str, HabitatCommandExecution] = OrderedDict()
        self._command_history: Dict[str, _CommandHistoryEntry] = {}
        self._stop_command_id: Optional[str] = None
        self._debug_trace_path = os.environ.get("HABITAT_DEBUG_TRACE_PATH", "").strip()

    def reset(self) -> HabitatObservation:
        with self._command_lock:
            reset_count = self.initial_episode_index + 1 if self._first_reset else 1
            raw = None
            for _ in range(reset_count):
                raw = self.env.reset()  # type: ignore[attr-defined]
            self._first_reset = False
            self._step_index = 0
            if not isinstance(raw, Mapping):
                raise HabitatWorkerError("INVALID_RESET_OBSERVATION")
            self._last_raw_observation = raw
            observation = self._observation(
                raw,
                collided=False,
                enforce_sensor_layout=False,
            )
            self._sensor_layout = _sensor_layout(observation)
            self._sensor_payload_bytes = (
                observation.rgb.nbytes + observation.depth.nbytes
            )
            self._command_results.clear()
            self._command_history.clear()
            self._stop_command_id = None
            self._debug_trace(
                "reset",
                observation=observation,
                reset_count=reset_count,
                initial_episode_index=self.initial_episode_index,
            )
            return observation

    def observe(self) -> HabitatObservation:
        if self._last_raw_observation is None:
            raise HabitatWorkerError("RESET_REQUIRED")
        return self._observation(self._last_raw_observation, collided=False)

    def step(
        self,
        action: object,
        *,
        forward_distance_m: Optional[float] = None,
        turn_angle_deg: Optional[float] = None,
    ) -> HabitatObservation:
        action_value = getattr(action, "value", action)
        if not isinstance(action_value, str) or action_value not in ACTION_VALUES:
            raise HabitatWorkerError("INVALID_ACTION")
        try:
            self.step_budget.validate_action(
                action_value,
                current_step_index=self._step_index,
            )
        except HabitatActionError as error:
            raise HabitatWorkerError(error.code, error.detail) from error
        if action_value == "hold":
            if forward_distance_m not in (None, 0, 0.0):
                raise HabitatWorkerError("INVALID_FORWARD_DISTANCE")
            if turn_angle_deg not in (None, 0, 0.0):
                raise HabitatWorkerError("INVALID_TURN_ANGLE")
            return self.observe()
        restore_forward_actuation: Optional[float] = None
        if action_value in {"move_forward", "move_backward"}:
            if forward_distance_m is not None:
                amount = self._validated_forward_distance(forward_distance_m)
                previous_amount = self._set_forward_actuation(
                    -amount if action_value == "move_backward" else amount
                )
                if action_value == "move_backward":
                    restore_forward_actuation = previous_amount
        elif forward_distance_m not in (None, 0, 0.0):
            raise HabitatWorkerError("INVALID_FORWARD_DISTANCE")
        if action_value in {"turn_left", "turn_right"}:
            amount = self._validated_turn_angle(turn_angle_deg)
            self._set_turn_actuation(action_value, amount)
        elif turn_angle_deg not in (None, 0, 0.0):
            raise HabitatWorkerError("INVALID_TURN_ANGLE")
        dispatched_action = self._step_action_value(
            "move_forward" if action_value == "move_backward" else action_value
        )
        before_observation = self.observe()
        try:
            raw = self.env.step(dispatched_action)  # type: ignore[attr-defined]
        finally:
            if restore_forward_actuation is not None:
                self._set_forward_actuation(restore_forward_actuation)
        if not isinstance(raw, Mapping):
            raise HabitatWorkerError("INVALID_STEP_OBSERVATION")
        self._last_raw_observation = raw
        self._step_index += 1
        collided = bool(
            getattr(self.env.sim, "previous_step_collided", False)  # type: ignore[attr-defined]
        )
        observation = self._observation(raw, collided=collided)
        self._debug_trace(
            "step",
            observation=observation,
            before_observation=before_observation,
            requested_action=action_value,
            dispatched_action=dispatched_action,
            forward_distance_m=forward_distance_m,
            turn_angle_deg=turn_angle_deg,
        )
        return observation

    def step_velocity(
        self,
        *,
        linear_x_mps: float,
        angular_z_radps: float,
        duration_s: float,
    ) -> HabitatObservation:
        """Execute one non-holonomic velocity interval in a supporting backend."""

        values = (linear_x_mps, angular_z_radps, duration_s)
        if any(isinstance(value, bool) for value in values):
            raise HabitatWorkerError("INVALID_VELOCITY_COMMAND")
        try:
            linear_x, angular_z, duration = (float(value) for value in values)
        except (TypeError, ValueError, OverflowError) as error:
            raise HabitatWorkerError("INVALID_VELOCITY_COMMAND") from error
        if (
            not all(math.isfinite(value) for value in (linear_x, angular_z, duration))
            or duration <= 0.0
        ):
            raise HabitatWorkerError("INVALID_VELOCITY_COMMAND")
        if self._last_raw_observation is None:
            raise HabitatWorkerError("RESET_REQUIRED")
        try:
            self.step_budget.validate_action(
                HabitatAction.MOVE_FORWARD.value,
                current_step_index=self._step_index,
            )
        except HabitatActionError as error:
            raise HabitatWorkerError(error.code, error.detail) from error
        backend_step = getattr(self.env, "step_velocity", None)
        if not callable(backend_step):
            raise HabitatWorkerError("CONTINUOUS_VELOCITY_UNSUPPORTED")
        before_observation = self.observe()
        raw = backend_step(
            linear_x_mps=linear_x,
            angular_z_radps=angular_z,
            duration_s=duration,
        )
        if not isinstance(raw, Mapping):
            raise HabitatWorkerError("INVALID_STEP_OBSERVATION")
        self._last_raw_observation = raw
        self._step_index += 1
        collided = bool(
            getattr(self.env.sim, "previous_step_collided", False)  # type: ignore[attr-defined]
        )
        observation = self._observation(raw, collided=collided)
        self._debug_trace(
            "step_velocity",
            observation=observation,
            before_observation=before_observation,
            linear_x_mps=linear_x,
            angular_z_radps=angular_z,
            duration_s=duration,
        )
        return observation

    def _step_action_value(self, action_value: str) -> object:
        if self.dispatch_reference_action_ids:
            return REFERENCE_ACTION_IDS.get(action_value, action_value)
        return action_value

    def execute_command(self, command: HabitatCommand) -> HabitatCommandExecution:
        """Execute or replay one non-HOLD command within the active episode."""
        if not isinstance(command, HabitatCommand):
            raise HabitatWorkerError("INVALID_COMMAND")
        if command.action == "hold":
            raise HabitatWorkerError("HOLD_COMMAND_NOT_EXECUTABLE")
        with self._command_lock:
            return self._execute_command_locked(command)

    def lookup_command_result(
        self,
        command: HabitatCommand,
    ) -> Optional[HabitatCommandExecution]:
        """Return an exact cached result without consulting live observation state."""
        if not isinstance(command, HabitatCommand):
            raise HabitatWorkerError("INVALID_COMMAND")
        if command.action == "hold":
            raise HabitatWorkerError("HOLD_COMMAND_NOT_EXECUTABLE")
        with self._command_lock:
            self._validate_active_command_locked(command)
            return self._lookup_command_result_locked(command)

    @property
    def command_payload_upper_bound(self) -> int:
        """Fixed policy sensors plus the configured dynamic debug-frame budget."""
        if self._sensor_payload_bytes is None:
            raise HabitatWorkerError("RESET_REQUIRED")
        return self._sensor_payload_bytes + self.max_topdown_payload_bytes

    def _execute_command_locked(
        self,
        command: HabitatCommand,
    ) -> HabitatCommandExecution:
        self._validate_active_command_locked(command)
        cached = self._lookup_command_result_locked(command)
        if cached is not None:
            return cached

        if self._stop_command_id is not None:
            if command.action == "stop":
                raise HabitatWorkerError("STOP_ALREADY_REQUESTED")
            raise HabitatWorkerError("EPISODE_STOPPING")
        is_safety_stop = command.action == "stop"
        history_size = len(self._command_history)
        if history_size >= self.command_history_capacity and not is_safety_stop:
            raise HabitatWorkerError("COMMAND_HISTORY_EXHAUSTED")
        if history_size >= self.command_history_capacity + 1:
            raise HabitatWorkerError("COMMAND_HISTORY_EXHAUSTED")
        if is_safety_stop:
            self._stop_command_id = command.command_id
        self._command_history[command.command_id] = _CommandHistoryEntry(
            command=command,
            state="RESERVED",
        )

        try:
            forward_distance_m = (
                command.forward_distance_m
                if command.action in {"move_forward", "move_backward"}
                else None
            )
            turn_angle_deg = (
                command.turn_angle_deg
                if command.action in {"turn_left", "turn_right"}
                else None
            )
            observation = _immutable_observation(
                self.step(
                    command.action,
                    forward_distance_m=forward_distance_m,
                    turn_angle_deg=turn_angle_deg,
                )
            )
            return self._complete_command_locked(
                command,
                observation,
                executed=True,
                status="EXECUTED",
                error_code="",
            )
        except Exception as error:
            stable_error = _stable_command_error(error)
            if stable_error.code == "STOP_STEP_RESERVED" and command.action != "stop":
                observation = _immutable_observation(self.observe())
                return self._complete_command_locked(
                    command,
                    observation,
                    executed=False,
                    status="REJECTED",
                    error_code=stable_error.code,
                )
            self._command_history[command.command_id] = _CommandHistoryEntry(
                command=command,
                state="FAILED",
                error_code=stable_error.code,
                error_detail=stable_error.detail,
            )
            if stable_error.code == getattr(error, "code", None):
                raise stable_error
            raise stable_error from error

    def _complete_command_locked(
        self,
        command: HabitatCommand,
        observation: HabitatObservation,
        *,
        executed: bool,
        status: str,
        error_code: str,
    ) -> HabitatCommandExecution:
        pose = ExecutionPose(
            frame_id="habitat_world",
            x=observation.position_xyz[0],
            y=observation.position_xyz[1],
            z=observation.position_xyz[2],
            qx=observation.rotation_xyzw[0],
            qy=observation.rotation_xyzw[1],
            qz=observation.rotation_xyzw[2],
            qw=observation.rotation_xyzw[3],
        )
        execution = HabitatCommandExecution(
            result=HabitatActionResult(
                episode_id=command.episode_id,
                command_id=command.command_id,
                action_result_id=uuid.uuid4().hex,
                trajectory_id=command.trajectory_id,
                generation=command.generation,
                habitat_step_index=self._step_index,
                action=command.action,
                pose_after=pose,
                executed=executed,
                collided=observation.collided if executed else False,
                status=status,
                error_code=error_code,
            ),
            observation=observation,
        )
        if len(self._command_results) == self.command_replay_capacity:
            self._command_results.popitem(last=False)
        self._command_results[command.command_id] = execution
        self._command_history[command.command_id] = _CommandHistoryEntry(
            command=command,
            state="SUCCEEDED",
        )
        return execution

    def _validate_active_command_locked(self, command: HabitatCommand) -> None:
        if self._last_raw_observation is None:
            raise HabitatWorkerError("RESET_REQUIRED")
        episode = getattr(self.env, "current_episode", None)
        episode_id = str(getattr(episode, "episode_id", ""))
        if command.episode_id != episode_id:
            raise HabitatWorkerError("EPISODE_ID_MISMATCH")

    def _lookup_command_result_locked(
        self,
        command: HabitatCommand,
    ) -> Optional[HabitatCommandExecution]:
        history = self._command_history.get(command.command_id)
        if history is None:
            return None
        if history.command != command:
            raise HabitatWorkerError("COMMAND_ID_CONFLICT")
        if history.state == "FAILED":
            raise HabitatWorkerError(history.error_code, history.error_detail)
        if history.state == "RESERVED":
            raise HabitatWorkerError("COMMAND_EXECUTION_INCOMPLETE")
        cached = self._command_results.get(command.command_id)
        if cached is None:
            raise HabitatWorkerError("COMMAND_RESULT_EXPIRED")
        return cached

    def _validated_turn_angle(self, value: Optional[float]) -> float:
        angle = self.max_turn_angle_deg if value is None else value
        if (
            isinstance(angle, bool)
            or not math.isfinite(float(angle))
            or float(angle) <= 0.0
            or float(angle) > self.max_turn_angle_deg
        ):
            raise HabitatWorkerError("INVALID_TURN_ANGLE")
        return float(angle)

    def _validated_forward_distance(self, value: float) -> float:
        try:
            distance = float(value)
        except (TypeError, ValueError, OverflowError) as error:
            raise HabitatWorkerError("INVALID_FORWARD_DISTANCE") from error
        if (
            isinstance(value, bool)
            or not math.isfinite(distance)
            or distance <= 0.0
            or distance > self.max_forward_distance_m
        ):
            raise HabitatWorkerError("INVALID_FORWARD_DISTANCE")
        return distance

    def _set_forward_actuation(self, amount: float) -> float:
        try:
            agent = self.env.sim.get_agent(0)  # type: ignore[attr-defined]
            action_spec = next(
                spec
                for spec in agent.agent_config.action_space.values()
                if getattr(spec, "name", None) == "move_forward"
            )
            previous_amount = float(action_spec.actuation.amount)
            action_spec.actuation.amount = float(amount)
            return previous_amount
        except Exception as error:
            raise HabitatWorkerError("FORWARD_ACTUATION_UNAVAILABLE") from error

    def _set_turn_actuation(self, action: str, amount: float) -> None:
        try:
            agent = self.env.sim.get_agent(0)  # type: ignore[attr-defined]
            action_spec = next(
                spec
                for spec in agent.agent_config.action_space.values()
                if getattr(spec, "name", None) == action
            )
            action_spec.actuation.amount = float(amount)
        except Exception as error:
            raise HabitatWorkerError("TURN_ACTUATION_UNAVAILABLE") from error

    def close(self) -> None:
        close = getattr(self.env, "close", None)
        if callable(close):
            close()

    def _debug_trace(
        self,
        event: str,
        *,
        observation: HabitatObservation,
        before_observation: Optional[HabitatObservation] = None,
        requested_action: object = None,
        dispatched_action: object = None,
        reset_count: Optional[int] = None,
        initial_episode_index: Optional[int] = None,
        forward_distance_m: Optional[float] = None,
        turn_angle_deg: Optional[float] = None,
        linear_x_mps: Optional[float] = None,
        angular_z_radps: Optional[float] = None,
        duration_s: Optional[float] = None,
    ) -> None:
        if not self._debug_trace_path:
            return
        payload: Dict[str, object] = {
            "event": event,
            "worker_step_index": self._step_index,
            "episode_id": observation.task.episode_id,
            "scene_id": observation.task.scene_id,
            "target_object": observation.task.target_object,
            "rgb_ref": _array_debug_ref(observation.rgb),
            "depth_ref": _array_debug_ref(observation.depth),
            "position_xyz": list(observation.position_xyz),
            "rotation_xyzw": list(observation.rotation_xyzw),
            "collided": observation.collided,
            "episode_over": observation.episode_over,
            "metrics": dict(observation.evaluation_metrics),
            "room_region_id": observation.room_region_id,
        }
        if before_observation is not None:
            payload.update(
                {
                    "before_rgb_ref": _array_debug_ref(before_observation.rgb),
                    "before_position_xyz": list(before_observation.position_xyz),
                    "before_rotation_xyzw": list(before_observation.rotation_xyzw),
                    "before_metrics": dict(before_observation.evaluation_metrics),
                    "before_room_region_id": before_observation.room_region_id,
                }
            )
        if requested_action is not None:
            payload["requested_action"] = requested_action
        if dispatched_action is not None:
            payload["dispatched_action"] = dispatched_action
        if reset_count is not None:
            payload["reset_count"] = reset_count
        if initial_episode_index is not None:
            payload["initial_episode_index"] = initial_episode_index
        if forward_distance_m is not None:
            payload["forward_distance_m"] = forward_distance_m
        if turn_angle_deg is not None:
            payload["turn_angle_deg"] = turn_angle_deg
        if linear_x_mps is not None:
            payload["linear_x_mps"] = linear_x_mps
        if angular_z_radps is not None:
            payload["angular_z_radps"] = angular_z_radps
        if duration_s is not None:
            payload["duration_s"] = duration_s
        path = Path(self._debug_trace_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n")

    def _observation(
        self,
        raw: Mapping[str, np.ndarray],
        *,
        collided: bool,
        enforce_sensor_layout: bool = True,
    ) -> HabitatObservation:
        if "rgb" not in raw or "depth" not in raw:
            raise HabitatWorkerError("MISSING_SENSOR_OBSERVATION")
        rgb_raw = np.asarray(raw["rgb"])
        if rgb_raw.ndim != 3 or rgb_raw.shape[2] not in (3, 4):
            raise HabitatWorkerError("INVALID_RGB_SHAPE")
        rgb = np.ascontiguousarray(rgb_raw[..., :3], dtype=np.uint8)
        depth_raw = np.asarray(raw["depth"])
        if depth_raw.ndim == 3 and depth_raw.shape[2] == 1:
            depth_raw = depth_raw[..., 0]
        if depth_raw.shape != rgb.shape[:2]:
            raise HabitatWorkerError("INVALID_DEPTH_SHAPE")
        depth = np.ascontiguousarray(depth_raw, dtype=np.float32)
        if not bool(np.isfinite(depth).all()):
            raise HabitatWorkerError("NON_FINITE_DEPTH")
        if enforce_sensor_layout and self._sensor_layout is not None:
            candidate_layout = (
                tuple(rgb.shape),
                rgb.dtype.str,
                tuple(depth.shape),
                depth.dtype.str,
            )
            if candidate_layout != self._sensor_layout:
                raise HabitatWorkerError("SENSOR_LAYOUT_CHANGED")

        episode = getattr(self.env, "current_episode", None)
        if episode is None:
            raise HabitatWorkerError("EPISODE_UNAVAILABLE")
        target_object = (
            str(getattr(episode, "object_category", ""))
            if self.task_type == OBJECTNAV_TASK_TYPE
            else POINTNAV_TARGET_LABEL
        )
        task = HabitatTask(
            str(getattr(episode, "episode_id", "")),
            str(getattr(episode, "scene_id", "")),
            self.task_type,
            target_object,
        )
        if not all((task.episode_id, task.scene_id, task.target_object)):
            raise HabitatWorkerError("INVALID_EPISODE")
        state = self.env.sim.get_agent_state()  # type: ignore[attr-defined]
        position = tuple(float(value) for value in state.position)
        if len(position) != 3 or not all(math.isfinite(value) for value in position):
            raise HabitatWorkerError("INVALID_POSITION")
        raw_metrics = self._raw_metrics()
        metrics = dict(self._evaluation_metrics(raw_metrics))
        if self.task_type == POINTNAV_TASK_TYPE:
            invalid_metrics = [
                name
                for name in POINTNAV_REQUIRED_EVALUATION_METRICS
                if name not in metrics
            ]
            if invalid_metrics:
                raise HabitatWorkerError(
                    "INVALID_POINTNAV_EVALUATION_METRICS",
                    ",".join(invalid_metrics),
                )
        metrics.setdefault("num_steps", self._step_index)
        topdown_rgb = self._topdown_frame(raw_metrics.get("top_down_map"))
        episode_over = bool(getattr(self.env, "episode_over", False))
        policy_metadata: Dict[str, object] = {"episode_over": episode_over}
        if self.task_type == POINTNAV_TASK_TYPE:
            policy_metadata[POINTGOAL_SENSOR_UUID] = _pointgoal_observation(raw)
        room_match = (
            self.room_region_maps.lookup(task.scene_id, position)
            if self.task_type == POINTNAV_TASK_TYPE
            and self.room_region_maps is not None
            else None
        )
        room_region_id = None if room_match is None else room_match.region_id
        return HabitatObservation(
            task=task,
            rgb=rgb,
            depth=depth,
            camera=camera_model(rgb.shape[1], rgb.shape[0], self.camera_hfov_deg),
            position_xyz=position,  # type: ignore[arg-type]
            rotation_xyzw=habitat_quaternion_xyzw(state.rotation),
            collided=bool(collided),
            episode_over=episode_over,
            policy_metadata=policy_metadata,
            evaluation_metrics=metrics,
            topdown_rgb=topdown_rgb,
            artifact_metadata=_episode_artifact_metadata(episode),
            room_region_id=room_region_id,
        )

    def _raw_metrics(self) -> Mapping[str, object]:
        get_metrics = getattr(self.env, "get_metrics", None)
        if not callable(get_metrics):
            return {}
        raw = get_metrics()
        return raw if isinstance(raw, Mapping) else {}

    @staticmethod
    def _evaluation_metrics(raw: Mapping[str, object]) -> Mapping[str, object]:
        return {
            key: _json_safe_metric(raw[key])
            for key in EVALUATION_METRIC_KEYS
            if key in raw and _json_safe_metric(raw[key]) is not None
        }

    def _topdown_frame(self, metric: object) -> Optional[np.ndarray]:
        if metric is None or self.topdown_renderer is None:
            return None
        try:
            rendered = self.topdown_renderer(metric, self.topdown_height)
        except Exception:
            return None
        if not isinstance(rendered, np.ndarray):
            return None
        if (
            rendered.dtype != np.uint8
            or rendered.ndim != 3
            or rendered.shape[2] != 3
            or rendered.shape[0] <= 0
            or rendered.shape[1] <= 0
        ):
            return None
        frame = _bounded_topdown_frame(rendered, self.max_topdown_payload_bytes)
        if frame is None:
            return None
        frame.setflags(write=False)
        return frame


def _json_safe_metric(value: object) -> object:
    if isinstance(value, (str, bool, int)) or value is None:
        return value
    if isinstance(value, (float, np.floating, np.integer)):
        converted = float(value)
        return converted if math.isfinite(converted) else None
    if isinstance(value, Mapping):
        return {
            str(key): safe
            for key, item in value.items()
            if (safe := _json_safe_metric(item)) is not None
        }
    return None


def _sensor_layout(
    observation: HabitatObservation,
) -> Tuple[Tuple[int, ...], str, Tuple[int, ...], str]:
    return (
        tuple(observation.rgb.shape),
        observation.rgb.dtype.str,
        tuple(observation.depth.shape),
        observation.depth.dtype.str,
    )


def _bounded_topdown_frame(
    rendered: np.ndarray,
    maximum_bytes: int,
) -> Optional[np.ndarray]:
    """Detach a nearest-neighbor debug frame without copying an oversize source."""
    maximum_pixels = maximum_bytes // 3
    if maximum_pixels <= 0:
        return None
    height, width, _ = rendered.shape
    if rendered.nbytes <= maximum_bytes:
        return np.array(rendered, dtype=np.uint8, order="C", copy=True)

    scale = math.sqrt(maximum_pixels / float(height * width))
    target_height = max(1, min(height, int(height * scale)))
    target_width = max(1, min(width, int(width * scale)))
    if target_height * target_width > maximum_pixels:
        if target_height >= target_width:
            target_height = max(1, maximum_pixels // target_width)
        else:
            target_width = max(1, maximum_pixels // target_height)
    row_indices = np.linspace(0, height - 1, target_height, dtype=np.intp)
    column_indices = np.linspace(0, width - 1, target_width, dtype=np.intp)
    sampled = rendered[row_indices[:, None], column_indices[None, :], :]
    return np.ascontiguousarray(sampled, dtype=np.uint8)


def smoke_habitat_worker(worker: HabitatWorker) -> Mapping[str, object]:
    """Prove reset sensors and one forward action against a real worker."""
    before = worker.reset()
    after = worker.step("move_forward")
    translation_m = math.sqrt(
        sum(
            (after_value - before_value) ** 2
            for before_value, after_value in zip(
                before.position_xyz,
                after.position_xyz,
            )
        )
    )
    if translation_m <= 1e-6:
        raise HabitatWorkerError("POSE_UNCHANGED")
    result = {
        "smoke_verified": True,
        "episode_id": before.task.episode_id,
        "scene_id": before.task.scene_id,
        "task_type": before.task.task_type,
        "target_object": before.task.target_object,
        "rgb_shape": list(before.rgb.shape),
        "depth_shape": list(before.depth.shape),
        "action": "move_forward",
        "collided": after.collided,
        "pose_before": list(before.position_xyz),
        "pose_after": list(after.position_xyz),
        "translation_m": translation_m,
        "pose_changed": True,
    }
    if POINTGOAL_SENSOR_UUID in before.policy_metadata:
        result[POINTGOAL_SENSOR_UUID] = list(
            before.policy_metadata[POINTGOAL_SENSOR_UUID]  # type: ignore[arg-type]
        )
    return result


def observation_packet(
    observation: HabitatObservation,
    request_id: str,
) -> Tuple[Mapping[str, object], bytes]:
    _bounded_worker_text(request_id, "REQUEST_ID_TOO_LARGE", MAX_REQUEST_ID_BYTES)
    observation_metadata, payload_length = _observation_packet_description(observation)
    metadata: Dict[str, object] = {
        "kind": "observation",
        "request_id": request_id,
    }
    metadata.update(observation_metadata)
    _preflight_worker_packet(metadata, payload_length)
    payload = _observation_packet_payload(observation)
    return metadata, payload


def action_result_packet(
    execution: HabitatCommandExecution,
    request_id: str,
) -> Tuple[Mapping[str, object], bytes]:
    """Project one causal execution and its exact observation onto the wire."""
    if not isinstance(execution, HabitatCommandExecution):
        raise HabitatWorkerError("INVALID_COMMAND_EXECUTION")
    result = execution.result
    _bounded_worker_text(request_id, "REQUEST_ID_TOO_LARGE", MAX_REQUEST_ID_BYTES)
    observation_metadata, payload_length = _observation_packet_description(
        execution.observation
    )
    metadata: Dict[str, object] = {
        "kind": "action_result",
        "request_id": request_id,
        "result": _action_result_metadata(result),
    }
    metadata.update(observation_metadata)
    _preflight_worker_packet(metadata, payload_length)
    payload = _observation_packet_payload(execution.observation)
    return metadata, payload


def _action_result_metadata(result: HabitatActionResult) -> Mapping[str, object]:
    if not isinstance(result, HabitatActionResult):
        raise HabitatWorkerError("INVALID_ACTION_RESULT")
    _bounded_worker_text(
        result.episode_id,
        "EPISODE_ID_TOO_LARGE",
        MAX_CAUSAL_ID_BYTES,
    )
    _bounded_worker_text(
        result.command_id,
        "COMMAND_ID_TOO_LARGE",
        MAX_CAUSAL_ID_BYTES,
    )
    _bounded_worker_text(
        result.action_result_id,
        "ACTION_RESULT_ID_TOO_LARGE",
        MAX_CAUSAL_ID_BYTES,
    )
    _bounded_worker_text(
        result.trajectory_id,
        "TRAJECTORY_ID_TOO_LARGE",
        MAX_CAUSAL_ID_BYTES,
    )
    _bounded_worker_text(
        result.pose_after.frame_id,
        "FRAME_ID_TOO_LARGE",
        MAX_CAUSAL_ID_BYTES,
    )
    _bounded_worker_text(
        result.error_code,
        "ERROR_CODE_TOO_LARGE",
        MAX_ERROR_CODE_BYTES,
        empty_allowed=True,
    )
    pose = result.pose_after
    return {
        "episode_id": result.episode_id,
        "command_id": result.command_id,
        "action_result_id": result.action_result_id,
        "trajectory_id": result.trajectory_id,
        "generation": result.generation,
        "habitat_step_index": result.habitat_step_index,
        "action": result.action,
        "pose_after": {
            "frame_id": pose.frame_id,
            "x": pose.x,
            "y": pose.y,
            "z": pose.z,
            "qx": pose.qx,
            "qy": pose.qy,
            "qz": pose.qz,
            "qw": pose.qw,
        },
        "executed": result.executed,
        "collided": result.collided,
        "status": result.status,
        "error_code": result.error_code,
    }


def _observation_packet_description(
    observation: HabitatObservation,
) -> Tuple[Dict[str, object], int]:
    if not isinstance(observation, HabitatObservation):
        raise HabitatWorkerError("INVALID_OBSERVATION")
    task = observation.task
    if not isinstance(task, HabitatTask):
        raise HabitatWorkerError("INVALID_OBSERVATION_METADATA")
    for value, code in (
        (task.episode_id, "EPISODE_ID_TOO_LARGE"),
        (task.scene_id, "SCENE_ID_TOO_LARGE"),
        (task.task_type, "TASK_TYPE_TOO_LARGE"),
        (task.target_object, "TARGET_OBJECT_TOO_LARGE"),
    ):
        _bounded_worker_text(value, code, MAX_CAUSAL_ID_BYTES)
    rgb_bytes = _validated_array_nbytes(
        observation.rgb,
        dtype=np.dtype(np.uint8),
        dimensions=3,
        channels=3,
        code="INVALID_RGB_ARRAY",
    )
    depth_bytes = _validated_array_nbytes(
        observation.depth,
        dtype=np.dtype(np.float32),
        dimensions=2,
        channels=None,
        code="INVALID_DEPTH_ARRAY",
    )
    if observation.rgb.shape[:2] != observation.depth.shape:
        raise HabitatWorkerError("SENSOR_SHAPE_MISMATCH")
    topdown_bytes = 0
    if observation.topdown_rgb is not None:
        topdown_bytes = _validated_array_nbytes(
            observation.topdown_rgb,
            dtype=np.dtype(np.uint8),
            dimensions=3,
            channels=3,
            code="INVALID_TOPDOWN_ARRAY",
        )
    payload_length = rgb_bytes + depth_bytes + topdown_bytes
    if payload_length > MAX_PAYLOAD_BYTES:
        raise HabitatWorkerError("PAYLOAD_TOO_LARGE")
    metadata: Dict[str, object] = {
        "task": {
            "episode_id": task.episode_id,
            "scene_id": task.scene_id,
            "task_type": task.task_type,
            "target_object": task.target_object,
        },
        "rgb_shape": list(observation.rgb.shape),
        "rgb_dtype": "uint8",
        "rgb_bytes": rgb_bytes,
        "depth_shape": list(observation.depth.shape),
        "depth_dtype": "float32-le",
        "depth_bytes": depth_bytes,
        "topdown_shape": (
            []
            if observation.topdown_rgb is None
            else list(observation.topdown_rgb.shape)
        ),
        "topdown_dtype": "uint8",
        "topdown_bytes": topdown_bytes,
        "camera": {
            "width": observation.camera.width,
            "height": observation.camera.height,
            "hfov_deg": observation.camera.hfov_deg,
            "camera_matrix_row_major": list(observation.camera.camera_matrix_row_major),
        },
        "position_xyz": list(observation.position_xyz),
        "rotation_xyzw": list(observation.rotation_xyzw),
        "collided": observation.collided,
        "episode_over": observation.episode_over,
        "policy_metadata": _json_native_value(observation.policy_metadata),
        "evaluation_metrics": _json_native_value(observation.evaluation_metrics),
        "artifact_metadata": _json_native_value(observation.artifact_metadata),
    }
    return metadata, payload_length


def _validated_array_nbytes(
    value: object,
    *,
    dtype: np.dtype,
    dimensions: int,
    channels: Optional[int],
    code: str,
) -> int:
    if not isinstance(value, np.ndarray) or value.dtype != dtype:
        raise HabitatWorkerError(code)
    shape = tuple(value.shape)
    if (
        len(shape) != dimensions
        or any(
            not isinstance(size, int) or isinstance(size, bool) or size <= 0
            for size in shape
        )
        or (channels is not None and shape[-1] != channels)
    ):
        raise HabitatWorkerError(code)
    expected_nbytes = dtype.itemsize
    for size in shape:
        expected_nbytes *= size
        if expected_nbytes > MAX_PAYLOAD_BYTES:
            raise HabitatWorkerError("PAYLOAD_TOO_LARGE")
    if value.nbytes != expected_nbytes:
        raise HabitatWorkerError(code)
    return expected_nbytes


def _observation_packet_payload(observation: HabitatObservation) -> bytes:
    rgb_payload = observation.rgb.tobytes(order="C")
    depth_payload = observation.depth.astype("<f4", copy=False).tobytes(order="C")
    topdown_payload = (
        b""
        if observation.topdown_rgb is None
        else observation.topdown_rgb.tobytes(order="C")
    )
    return rgb_payload + depth_payload + topdown_payload


def _preflight_worker_packet(
    metadata: Mapping[str, object],
    payload_length: int,
    *,
    metadata_reserve_bytes: int = 0,
) -> None:
    try:
        preflight_packet(
            metadata,
            payload_length,
            metadata_reserve_bytes=metadata_reserve_bytes,
        )
    except HabitatProtocolError as error:
        raise HabitatWorkerError(error.code, error.detail) from error


_BASE_REQUEST_FIELDS = frozenset({"kind", "request_id"})
_STEP_REQUEST_FIELDS = _BASE_REQUEST_FIELDS | frozenset(
    {
        "episode_id",
        "command_id",
        "trajectory_id",
        "generation",
        "action",
        "forward_distance_m",
        "turn_angle_deg",
    }
)
_VELOCITY_REQUEST_FIELDS = _BASE_REQUEST_FIELDS | frozenset(
    {"linear_x_mps", "angular_z_radps", "duration_s"}
)


def _reject_unexpected_request_metadata(
    metadata: Mapping[str, object],
    allowed_fields: frozenset,
) -> None:
    if set(metadata) - allowed_fields:
        raise HabitatWorkerError("UNEXPECTED_REQUEST_METADATA")


def _command_from_request(metadata: Mapping[str, object]) -> HabitatCommand:
    _reject_unexpected_request_metadata(metadata, _STEP_REQUEST_FIELDS)
    try:
        command = HabitatCommand(
            episode_id=metadata.get("episode_id"),  # type: ignore[arg-type]
            command_id=metadata.get("command_id"),  # type: ignore[arg-type]
            trajectory_id=metadata.get("trajectory_id"),  # type: ignore[arg-type]
            generation=metadata.get("generation"),  # type: ignore[arg-type]
            action=metadata.get("action"),  # type: ignore[arg-type]
            forward_distance_m=metadata.get("forward_distance_m"),  # type: ignore[arg-type]
            turn_angle_deg=metadata.get("turn_angle_deg"),  # type: ignore[arg-type]
        )
    except ExecutionContractError as error:
        raise HabitatWorkerError(error.code, error.detail) from error
    for value, code in (
        (command.episode_id, "EPISODE_ID_TOO_LARGE"),
        (command.command_id, "COMMAND_ID_TOO_LARGE"),
        (command.trajectory_id, "TRAJECTORY_ID_TOO_LARGE"),
    ):
        _bounded_worker_text(value, code, MAX_CAUSAL_ID_BYTES)
    return command


def _preflight_command_response(
    worker: HabitatWorker,
    command: HabitatCommand,
    request_id: str,
) -> None:
    observation = worker.observe()
    observation_metadata, _ = _observation_packet_description(observation)
    pose = ExecutionPose(
        frame_id="habitat_world",
        x=observation.position_xyz[0],
        y=observation.position_xyz[1],
        z=observation.position_xyz[2],
        qx=observation.rotation_xyzw[0],
        qy=observation.rotation_xyzw[1],
        qz=observation.rotation_xyzw[2],
        qw=observation.rotation_xyzw[3],
    )
    placeholder = HabitatActionResult(
        episode_id=command.episode_id,
        command_id=command.command_id,
        action_result_id="0" * 32,
        trajectory_id=command.trajectory_id,
        generation=command.generation,
        habitat_step_index=UINT32_MAX,
        action=command.action,
        pose_after=pose,
        executed=True,
        collided=False,
        status="EXECUTED",
        error_code="",
    )
    metadata: Dict[str, object] = {
        "kind": "action_result",
        "request_id": request_id,
        "result": _action_result_metadata(placeholder),
    }
    metadata.update(observation_metadata)
    _preflight_worker_packet(
        metadata,
        worker.command_payload_upper_bound,
        metadata_reserve_bytes=ACTION_RESULT_METADATA_RESERVE_BYTES,
    )


def render_habitat_topdown(metric: object, height: int) -> np.ndarray:
    """Render Habitat-Lab's evaluation-only TopDownMap measurement."""
    from habitat.utils.visualizations.maps import (
        colorize_draw_agent_and_fit_to_height,
    )

    image = colorize_draw_agent_and_fit_to_height(metric, height)
    return np.ascontiguousarray(image[..., :3], dtype=np.uint8)


def serve_connection(connection: socket.socket, worker: HabitatWorker) -> None:
    while True:
        try:
            packet = recv_packet(connection)
        except HabitatProtocolError as error:
            if error.code == "UNEXPECTED_EOF":
                return
            _try_send_error(connection, "", error)
            return
        request_id = packet.metadata.get("request_id")
        try:
            if not isinstance(request_id, str) or not request_id.strip():
                raise HabitatWorkerError("INVALID_REQUEST_ID")
            _bounded_worker_text(
                request_id,
                "REQUEST_ID_TOO_LARGE",
                MAX_REQUEST_ID_BYTES,
            )
        except HabitatWorkerError as error:
            if not _try_send_error(connection, request_id, error):
                return
            continue
        try:
            kind = packet.metadata.get("kind")
            if packet.payload:
                raise HabitatWorkerError("UNEXPECTED_REQUEST_PAYLOAD")
            if kind == "health":
                _reject_unexpected_request_metadata(
                    packet.metadata,
                    _BASE_REQUEST_FIELDS,
                )
                send_packet(
                    connection,
                    {"kind": "health", "request_id": request_id, "ready": True},
                    b"",
                )
                continue
            if kind == "reset":
                _reject_unexpected_request_metadata(
                    packet.metadata,
                    _BASE_REQUEST_FIELDS,
                )
                observation = worker.reset()
            elif kind == "observe":
                _reject_unexpected_request_metadata(
                    packet.metadata,
                    _BASE_REQUEST_FIELDS,
                )
                observation = worker.observe()
            elif kind == "step":
                command = _command_from_request(packet.metadata)
                execution = worker.lookup_command_result(command)
                if execution is None:
                    _preflight_command_response(worker, command, request_id)
                    execution = worker.execute_command(command)
                metadata, payload = action_result_packet(execution, request_id)
                send_packet(connection, metadata, payload)
                continue
            elif kind == "step_velocity":
                _reject_unexpected_request_metadata(
                    packet.metadata,
                    _VELOCITY_REQUEST_FIELDS,
                )
                observation = worker.step_velocity(
                    linear_x_mps=packet.metadata.get("linear_x_mps"),  # type: ignore[arg-type]
                    angular_z_radps=packet.metadata.get(  # type: ignore[arg-type]
                        "angular_z_radps"
                    ),
                    duration_s=packet.metadata.get("duration_s"),  # type: ignore[arg-type]
                )
            else:
                raise HabitatWorkerError("INVALID_REQUEST_KIND")
            metadata, payload = observation_packet(observation, request_id)
            send_packet(connection, metadata, payload)
        except Exception as error:
            if not _try_send_error(connection, request_id, error):
                return


def _try_send_error(
    connection: socket.socket,
    request_id: object,
    error: Exception,
) -> bool:
    try:
        echoed_request_id = validate_bounded_text(
            request_id,
            maximum_bytes=MAX_REQUEST_ID_BYTES,
            code="INVALID_REQUEST_ID",
        )
    except HabitatProtocolError:
        echoed_request_id = ""
    code = truncate_utf8(
        getattr(error, "code", "HABITAT_WORKER_ERROR"),
        maximum_bytes=MAX_ERROR_CODE_BYTES,
        fallback="HABITAT_WORKER_ERROR",
    )
    if not code.strip():
        code = "HABITAT_WORKER_ERROR"
    message = truncate_utf8(
        error,
        maximum_bytes=MAX_ERROR_MESSAGE_BYTES,
        fallback=code,
    )
    try:
        send_packet(
            connection,
            {
                "kind": "error",
                "request_id": echoed_request_id,
                "code": code,
                "message": message,
            },
            b"",
        )
    except (HabitatProtocolError, OSError, TimeoutError):
        return False
    return True


def _truthy_environment_flag(name: str) -> bool:
    return os.environ.get(name, "0").strip().lower() in {"1", "true", "yes", "on"}


def _parse_episode_indices(raw: str) -> Tuple[int, ...]:
    if not str(raw or "").strip():
        raise HabitatWorkerError("MISSING_EPISODE_INDICES")
    indices = []
    for item in str(raw).split(","):
        value = item.strip()
        if not value:
            continue
        try:
            index = int(value)
        except ValueError as error:
            raise HabitatWorkerError("INVALID_EPISODE_INDEX", value) from error
        if index < 0:
            raise HabitatWorkerError("INVALID_EPISODE_INDEX", value)
        indices.append(index)
    if not indices:
        raise HabitatWorkerError("MISSING_EPISODE_INDICES")
    if len(indices) != len(set(indices)):
        raise HabitatWorkerError("DUPLICATE_EPISODE_INDEX")
    return tuple(indices)


def _normalized_scene_id(value: object) -> str:
    scene = Path(str(value or "")).name
    for suffix in (".basis.glb", ".glb", ".basis"):
        if scene.endswith(suffix):
            return scene[: -len(suffix)]
    return scene


def _parse_expected_episode_identities(
    raw_json: str,
    episode_indices: Sequence[int],
) -> Mapping[int, Mapping[str, str]]:
    try:
        keys = json.loads(str(raw_json or "[]"))
    except json.JSONDecodeError as error:
        raise HabitatWorkerError("INVALID_EXPECTED_EPISODE_KEYS_JSON") from error
    if not isinstance(keys, list) or not all(isinstance(key, str) for key in keys):
        raise HabitatWorkerError("INVALID_EXPECTED_EPISODE_KEYS_JSON")
    indices = tuple(int(index) for index in episode_indices)
    if len(keys) != len(indices):
        raise HabitatWorkerError("EXPECTED_EPISODE_KEY_COUNT_MISMATCH")
    if len(keys) != len(set(keys)):
        raise HabitatWorkerError("DUPLICATE_EXPECTED_EPISODE_KEY")

    identities: Dict[int, Mapping[str, str]] = {}
    for index, key in zip(indices, keys):
        fields = key.split("|")
        if len(fields) != 4 or any(not field.strip() for field in fields):
            raise HabitatWorkerError("INVALID_EXPECTED_EPISODE_KEY", key)
        _stage, scene_id, episode_id, object_goal = fields
        identities[int(index)] = MappingProxyType(
            {
                "episode_id": episode_id,
                "scene_id": scene_id,
                "episode_key": key,
                "object_goal": object_goal,
            }
        )
    return MappingProxyType(identities)


def _select_dataset_episodes_by_identity(
    dataset: object,
    *,
    episode_indices: Sequence[int],
    expected_episode_identities: Mapping[int, Mapping[str, str]],
) -> object:
    indices = tuple(int(index) for index in episode_indices)
    if set(expected_episode_identities) != set(indices):
        raise HabitatWorkerError("FROZEN_EPISODE_IDENTITY_COVERAGE_MISMATCH")

    matches: Dict[Tuple[str, str, str], list[object]] = {}
    for episode in list(getattr(dataset, "episodes", []) or []):
        identity = (
            _normalized_scene_id(getattr(episode, "scene_id", "")),
            str(getattr(episode, "episode_id", "") or "").strip(),
            str(getattr(episode, "object_category", "") or "").strip(),
        )
        matches.setdefault(identity, []).append(episode)

    selected = []
    for index in indices:
        identity = expected_episode_identities[index]
        expected_goal = str(identity.get("object_goal") or "").strip()
        if expected_goal == "pointnav":
            expected_goal = ""
        key = (
            str(identity.get("scene_id") or "").strip(),
            str(identity.get("episode_id") or "").strip(),
            expected_goal,
        )
        candidates = matches.get(key, [])
        if not candidates:
            raise HabitatWorkerError(
                "MISSING_FROZEN_EPISODE",
                str(identity.get("episode_key") or ""),
            )
        if len(candidates) != 1:
            raise HabitatWorkerError(
                "AMBIGUOUS_FROZEN_EPISODE",
                str(identity.get("episode_key") or ""),
            )
        selected.append(candidates[0])

    setattr(dataset, "episodes", selected)
    return dataset


def _habitat_initial_episode_index_from_environment() -> int:
    if _truthy_environment_flag("VOCA_DIRECT_EPISODE_KEY_SELECTION"):
        return 0
    return int(os.environ.get("HABITAT_EPISODE_INDEX", "0"))


def build_habitat_config_from_environment(
    settings: Optional[HabitatBenchmarkSettings] = None,
    *,
    environ: Optional[Mapping[str, str]] = None,
) -> object:
    import habitat
    from habitat.config.read_write import read_write

    environment = os.environ if environ is None else environ
    habitat_file = getattr(habitat, "__file__", None)
    if not habitat_file:
        raise HabitatWorkerError("HABITAT_PACKAGE_PATH_UNAVAILABLE")
    package_dir = Path(str(habitat_file)).resolve().parent
    benchmark = settings or habitat_benchmark_settings(
        package_dir,
        environ=environment,
    )
    config = habitat.get_config(benchmark.config_path)
    width = int(environment.get("HABITAT_CAMERA_WIDTH", "640"))
    height = int(environment.get("HABITAT_CAMERA_HEIGHT", "480"))
    hfov = benchmark.camera_hfov_deg
    sensor_height = float(environment.get("HABITAT_SENSOR_HEIGHT_M", "0.88"))
    record_episodes = environment.get(
        "HABITAT_RECORD_EPISODES", "0"
    ).strip().lower() in {"1", "true", "yes", "on"}
    with read_write(config):
        config.habitat.dataset.split = environment.get("HABITAT_SPLIT", "val")
        config.habitat.dataset.scenes_dir = benchmark.scenes_dir
        config.habitat.dataset.data_path = benchmark.dataset_path
        config.habitat.simulator.scene_dataset = benchmark.scene_dataset_config
        config.habitat.environment.iterator_options.num_episode_sample = int(
            environment.get("HABITAT_EPISODE_COUNT", "50")
        )
        config.habitat.environment.max_episode_steps = benchmark.max_episode_steps
        seed = int(environment.get("HABITAT_SEED", "20260710"))
        config.habitat.seed = seed
        config.habitat.simulator.seed = seed
        agent = config.habitat.simulator.agents.main_agent
        default_height = "1.50" if benchmark.task_type == POINTNAV_TASK_TYPE else "0.88"
        default_radius = "0.10" if benchmark.task_type == POINTNAV_TASK_TYPE else "0.18"
        agent.height = float(environment.get("HABITAT_ROBOT_HEIGHT_M", default_height))
        agent.radius = float(environment.get("HABITAT_ROBOT_RADIUS_M", default_radius))
        for sensor in (agent.sim_sensors.rgb_sensor, agent.sim_sensors.depth_sensor):
            sensor.width = width
            sensor.height = height
            sensor.hfov = hfov
            sensor.position = [0.0, sensor_height, 0.0]
        agent.sim_sensors.depth_sensor.max_depth = float(
            environment.get("HABITAT_MAX_DEPTH_M", "5.0")
        )
        agent.sim_sensors.depth_sensor.normalize_depth = False
        config.habitat.simulator.forward_step_size = float(
            environment.get("HABITAT_FORWARD_STEP_M", "0.25")
        )
        config.habitat.simulator.turn_angle = habitat_turn_angle(
            environment.get("HABITAT_TURN_ANGLE_DEG", "30")
        )
        from habitat.config.default_structured_configs import (
            LookDownActionConfig,
            LookUpActionConfig,
        )

        look_angle = habitat_look_angle_from_environment(environment)
        from omegaconf import open_dict

        with open_dict(config.habitat.simulator):
            config.habitat.simulator.tilt_angle = look_angle
        config.habitat.task.actions["look_up"] = LookUpActionConfig(
            tilt_angle=look_angle
        )
        config.habitat.task.actions["look_down"] = LookDownActionConfig(
            tilt_angle=look_angle
        )
        config.habitat.task.measurements.success.success_distance = (
            benchmark.success_distance_m
        )
        config.habitat.simulator.habitat_sim_v0.allow_sliding = (
            benchmark.allow_sliding
        )
        if benchmark.task_key == POINTNAV_TASK_KEY:
            from habitat.config.default_structured_configs import (
                CollisionsMeasurementConfig,
                NumStepsMeasurementConfig,
                PointGoalWithGPSCompassSensorConfig,
                SoftSPLMeasurementConfig,
            )

            config.habitat.task.lab_sensors[
                "pointgoal_with_gps_compass_sensor"
            ] = PointGoalWithGPSCompassSensorConfig()
            config.habitat.task.measurements["collisions"] = (
                CollisionsMeasurementConfig()
            )
            config.habitat.task.measurements["num_steps"] = (
                NumStepsMeasurementConfig()
            )
            config.habitat.task.measurements["soft_spl"] = (
                SoftSPLMeasurementConfig()
            )
        if record_episodes:
            from habitat.config.default_structured_configs import (
                TopDownMapMeasurementConfig,
            )

            config.habitat.task.measurements["top_down_map"] = (
                TopDownMapMeasurementConfig()
            )
    return config


def build_habitat_env_from_environment(
    settings: Optional[HabitatBenchmarkSettings] = None,
) -> object:
    import habitat
    from habitat.config.read_write import read_write

    config = build_habitat_config_from_environment(settings)
    habitat_config = getattr(config, "habitat")
    direct_episode_dataset = None
    if _truthy_environment_flag("VOCA_DIRECT_EPISODE_KEY_SELECTION"):
        episode_indices = _parse_episode_indices(
            os.environ.get("VOCA_EPISODE_INDICES", "")
        )
        expected_episode_identities = _parse_expected_episode_identities(
            os.environ.get("VOCA_EXPECTED_EPISODE_KEYS_JSON", ""),
            episode_indices,
        )
        direct_episode_dataset = habitat.make_dataset(
            habitat_config.dataset.type,
            config=habitat_config.dataset,
        )
        _select_dataset_episodes_by_identity(
            direct_episode_dataset,
            episode_indices=episode_indices,
            expected_episode_identities=expected_episode_identities,
        )
        with read_write(config):
            habitat_config.environment.iterator_options.num_episode_sample = -1
            habitat_config.environment.iterator_options.shuffle = False
            habitat_config.environment.iterator_options.group_by_scene = False
            habitat_config.environment.iterator_options.cycle = False
    if direct_episode_dataset is not None:
        return habitat.Env(config, dataset=direct_episode_dataset)
    return habitat.Env(config)


def serve_unix(socket_path: str, worker: HabitatWorker) -> None:
    path = Path(socket_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        path.unlink()
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        server.bind(str(path))
        server.listen(1)
        while True:
            connection, _ = server.accept()
            with connection:
                try:
                    serve_connection(connection, worker)
                except (HabitatProtocolError, OSError, TimeoutError):
                    continue
    finally:
        server.close()
        worker.close()
        if path.exists():
            path.unlink()


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--socket",
        default=os.environ.get("HABITAT_WORKER_SOCKET", "/run/habitat/worker.sock"),
    )
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args(argv)
    record_episodes = os.environ.get(
        "HABITAT_RECORD_EPISODES", "0"
    ).strip().lower() in {"1", "true", "yes", "on"}
    import habitat

    habitat_file = getattr(habitat, "__file__", None)
    if not habitat_file:
        raise HabitatWorkerError("HABITAT_PACKAGE_PATH_UNAVAILABLE")
    benchmark = habitat_benchmark_settings(
        Path(str(habitat_file)).resolve().parent
    )
    room_region_maps = (
        RoomRegionMapStore(
            Path(
                os.environ.get(
                    "HABITAT_POINTNAV_ROOM_REGION_MAP_DIR",
                    POINTNAV_ROOM_REGION_MAP_DIR,
                )
            )
        )
        if benchmark.task_type == POINTNAV_TASK_TYPE
        and _environment_flag(os.environ, "HABITAT_POINTNAV_ROOM_METRICS", "0")
        else None
    )
    worker = HabitatWorker(
        build_habitat_env_from_environment(benchmark),
        camera_hfov_deg=float(benchmark.camera_hfov_deg),
        initial_episode_index=_habitat_initial_episode_index_from_environment(),
        topdown_renderer=render_habitat_topdown if record_episodes else None,
        topdown_height=int(os.environ.get("HABITAT_TOPDOWN_HEIGHT", "1024")),
        max_topdown_payload_bytes=int(
            os.environ.get(
                "HABITAT_MAX_TOPDOWN_PAYLOAD_BYTES",
                str(DEFAULT_MAX_TOPDOWN_PAYLOAD_BYTES),
            )
        ),
        max_turn_angle_deg=float(os.environ.get("HABITAT_TURN_ANGLE_DEG", "30")),
        max_forward_distance_m=float(
            os.environ.get("HABITAT_MAX_FORWARD_STEP_M", "0.25")
        ),
        max_episode_steps=benchmark.max_episode_steps,
        reserved_stop_steps=int(os.environ.get("HABITAT_RESERVED_STOP_STEPS", "1")),
        dispatch_reference_action_ids=os.environ.get(
            "HABITAT_REFERENCE_ACTION_IDS", "1"
        ).strip().lower()
        in {"1", "true", "yes", "on"},
        task_type=benchmark.task_type,
        room_region_maps=room_region_maps,
    )
    if args.smoke:
        try:
            print(json.dumps(smoke_habitat_worker(worker), sort_keys=True))
        finally:
            worker.close()
        return
    serve_unix(args.socket, worker)


if __name__ == "__main__":
    main()
