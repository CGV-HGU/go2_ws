from __future__ import annotations

# pyright: reportMissingImports=false

from collections import deque
import math
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from .algorithms import TrajectoryPlan
from .pose_buffer import Pose2D


S2E_CONTEXT_SIZE = 11
S2E_IMAGE_SIZE = 256
S2E_WAYPOINT_SCALE = 0.25


def _resize_nearest_hwc(frame: np.ndarray, *, size: int = S2E_IMAGE_SIZE) -> np.ndarray:
    height, width, _ = frame.shape
    y_indices = np.linspace(0, height - 1, size).astype(np.int64)
    x_indices = np.linspace(0, width - 1, size).astype(np.int64)
    return frame[y_indices][:, x_indices]


def ros_rgb8_to_chw_float(data: bytes | bytearray | memoryview, *, width: int, height: int) -> np.ndarray:
    expected_size = width * height * 3
    if len(data) != expected_size:
        raise ValueError(f"rgb8 image data has {len(data)} bytes, expected {expected_size}")
    frame = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
    frame = _resize_nearest_hwc(frame, size=S2E_IMAGE_SIZE)
    frame_float = frame.astype(np.float32) / 255.0
    return frame_float.transpose(2, 0, 1).copy()


class S2EFrameContext:
    def __init__(self, context_size: int = S2E_CONTEXT_SIZE) -> None:
        self.context_size = context_size
        self._frames: deque[np.ndarray] = deque(maxlen=context_size)

    def append(self, frame_chw: np.ndarray) -> None:
        if frame_chw.shape != (3, S2E_IMAGE_SIZE, S2E_IMAGE_SIZE):
            raise ValueError(f"S2E frame must have shape (3, 256, 256), got {frame_chw.shape}")
        if frame_chw.dtype != np.float32:
            raise ValueError(f"S2E frame must be float32, got {frame_chw.dtype}")
        self._frames.append(frame_chw)

    @property
    def frame_count(self) -> int:
        return len(self._frames)

    def batch(self) -> np.ndarray | None:
        if len(self._frames) < self.context_size:
            return None
        return np.stack(list(self._frames), axis=0)[np.newaxis].astype(np.float32, copy=False)


def convert_s2e_output_to_points(output: np.ndarray) -> list[tuple[float, float]]:
    trajectory = np.asarray(output, dtype=np.float32)
    if trajectory.shape != (1, 1, 10, 2):
        raise ValueError(f"S2E trajectory output must have shape (1, 1, 10, 2), got {trajectory.shape}")
    points = trajectory[0, 0]
    if not np.isfinite(points).all():
        raise ValueError("S2E trajectory output contains non-finite values")
    return [(float(x), float(y)) for x, y in points]


class S2ENavigatorLike(Protocol):
    def inference_trajectory(self, obs: np.ndarray, goal_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ...


class OnnxS2ENavigator:
    context_size = S2E_CONTEXT_SIZE

    def __init__(
        self,
        onnx_path: str | Path | None = None,
        *,
        device: str = "cuda",
        session: Any | None = None,
    ) -> None:
        if session is None:
            if onnx_path is None:
                raise ValueError("onnx_path is required when session is not provided")
            import onnxruntime as ort

            ort.set_default_logger_severity(3)
            providers: list[Any]
            if device == "cuda":
                providers = [("CUDAExecutionProvider", {"arena_extend_strategy": "kSameAsRequested"}), "CPUExecutionProvider"]
            else:
                providers = ["CPUExecutionProvider"]
            session = ort.InferenceSession(str(onnx_path), providers=providers)
        if session is None:
            raise ValueError("S2E ONNX session could not be initialized")
        self._session = session
        self._input_names = [input_meta.name for input_meta in self._session.get_inputs()]

    @staticmethod
    def _goal_to_input(goal_xy: np.ndarray) -> np.ndarray:
        x = float(goal_xy[0])
        y = float(goal_xy[1])
        distance = math.sqrt(x * x + y * y)
        normalized_distance = max(min(distance, 200.0), 0.1) / 200.0
        angle = math.atan2(y, x)
        return np.array([normalized_distance, math.cos(angle), math.sin(angle)], dtype=np.float32)

    def inference_trajectory(self, obs: np.ndarray, goal_xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        obs_np = np.asarray(obs, dtype=np.float32)
        if obs_np.ndim != 5 or obs_np.shape[1:] != (S2E_CONTEXT_SIZE, 3, S2E_IMAGE_SIZE, S2E_IMAGE_SIZE):
            raise ValueError(f"S2E obs must have shape (B, 11, 3, 256, 256), got {obs_np.shape}")
        batch_size = obs_np.shape[0]
        goal_input = self._goal_to_input(np.asarray(goal_xy, dtype=np.float32))
        goal_batch = np.tile(goal_input, (batch_size, 1)).astype(np.float32, copy=False)

        trajectories = []
        for index in range(batch_size):
            feed = {
                "obs_images": obs_np[index : index + 1],
                "goal": goal_batch[index : index + 1],
            }
            feed = {name: feed[name] for name in self._input_names if name in feed}
            output = self._session.run(None, feed)
            raw_waypoints = np.asarray(output[0], dtype=np.float32)
            if raw_waypoints.shape != (1, 10, 3):
                raise ValueError(f"S2E ONNX output must have shape (1, 10, 3), got {raw_waypoints.shape}")
            trajectories.append(raw_waypoints[:, :, :2] * S2E_WAYPOINT_SCALE)

        trajectory = np.concatenate(trajectories, axis=0)[:, np.newaxis].astype(np.float32, copy=False)
        scores = np.ones((batch_size, 1), dtype=np.float32)
        return trajectory, scores


class S2EPlanner:
    def __init__(self, model_path: str | Path, *, device: str = "cuda", navigator: S2ENavigatorLike | None = None) -> None:
        if navigator is None:
            model_dir = Path(model_path)
            navigator = OnnxS2ENavigator(model_dir / "s2e.onnx", device=device)
        if navigator is None:
            raise ValueError("S2E navigator could not be initialized")
        self.navigator = navigator

    def plan(self, obs: np.ndarray, goal_point_base_link: tuple[float, float], current_pose: Pose2D) -> TrajectoryPlan:
        del current_pose
        goal_xy = np.array(goal_point_base_link, dtype=np.float32)
        trajectory, _scores = self.navigator.inference_trajectory(obs, goal_xy=goal_xy)
        points = convert_s2e_output_to_points(trajectory)
        return TrajectoryPlan(points=points, goal_point_base_link=goal_point_base_link, has_goal_point=True, status="S2E_FRESH")
