from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from .algorithms import StampedPoseData, TrajectoryPlan
from .pose_buffer import Pose2D
from .vlm_schema import VlmAction, encode_vlm_reasoning


def image_goal_to_base_link(
    goal_uv: tuple[float, float],
    *,
    image_size: tuple[int, int] = (640, 480),
    min_forward_m: float = 1.0,
    max_forward_m: float = 4.0,
    max_lateral_m: float = 2.0,
) -> tuple[float, float]:
    width, height = image_size
    u, v = goal_uv
    normalized_u = max(-1.0, min(1.0, (u - width / 2.0) / (width / 2.0)))
    normalized_v = max(0.0, min(1.0, 1.0 - v / max(height, 1)))
    forward = min_forward_m + (max_forward_m - min_forward_m) * normalized_v
    lateral = -max_lateral_m * normalized_u
    return round(forward, 6), round(lateral, 6)


def camera_image_to_base_link_ground(
    goal_uv: tuple[float, float],
    *,
    camera_matrix_row_major: tuple[float, ...],
    base_from_camera_translation_m: tuple[float, float, float],
    base_from_camera_rotation_matrix_row_major: tuple[float, ...],
    ground_z_m: float = 0.0,
    max_ground_distance_m: float | None = None,
) -> tuple[float, float] | None:
    fx, _, cx, _, fy, cy, _, _, _ = camera_matrix_row_major
    if abs(fx) <= 1e-12 or abs(fy) <= 1e-12:
        return None
    u, v = goal_uv
    ray_camera = ((u - cx) / fx, (v - cy) / fy, 1.0)
    ray_base = _matrix_vector_multiply(base_from_camera_rotation_matrix_row_major, ray_camera)
    origin_x, origin_y, origin_z = base_from_camera_translation_m
    if abs(ray_base[2]) <= 1e-12:
        return None
    distance = (ground_z_m - origin_z) / ray_base[2]
    if distance <= 1e-9:
        return None
    x = origin_x + distance * ray_base[0]
    y = origin_y + distance * ray_base[1]
    if not math.isfinite(x) or not math.isfinite(y):
        return None
    if max_ground_distance_m is not None and math.hypot(x, y) > max_ground_distance_m:
        return None
    return x, y


def base_link_ground_point_to_image(
    point_base_link: tuple[float, float, float],
    *,
    camera_matrix_row_major: tuple[float, ...],
    base_from_camera_translation_m: tuple[float, float, float],
    base_from_camera_rotation_matrix_row_major: tuple[float, ...],
) -> tuple[float, float] | None:
    fx, _, cx, _, fy, cy, _, _, _ = camera_matrix_row_major
    if abs(fx) <= 1e-12 or abs(fy) <= 1e-12:
        return None
    translated = (
        point_base_link[0] - base_from_camera_translation_m[0],
        point_base_link[1] - base_from_camera_translation_m[1],
        point_base_link[2] - base_from_camera_translation_m[2],
    )
    camera_x, camera_y, camera_z = _transpose_matrix_vector_multiply(base_from_camera_rotation_matrix_row_major, translated)
    if camera_z <= 1e-9:
        return None
    u = fx * camera_x / camera_z + cx
    v = fy * camera_y / camera_z + cy
    if not math.isfinite(u) or not math.isfinite(v):
        return None
    return u, v


def _matrix_vector_multiply(matrix: tuple[float, ...], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = matrix
    x, y, z = vector
    return (
        m00 * x + m01 * y + m02 * z,
        m10 * x + m11 * y + m12 * z,
        m20 * x + m21 * y + m22 * z,
    )


def _transpose_matrix_vector_multiply(matrix: tuple[float, ...], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = matrix
    x, y, z = vector
    return (
        m00 * x + m10 * y + m20 * z,
        m01 * x + m11 * y + m21 * z,
        m02 * x + m12 * y + m22 * z,
    )


def resample_path_to_ten_points(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    if not points:
        raise ValueError("points must not be empty")
    if len(points) == 1:
        return [points[0]] * 10
    result: list[tuple[float, float]] = []
    max_source = len(points) - 1
    for target_index in range(10):
        source_position = target_index * max_source / 9.0
        lower = int(math.floor(source_position))
        upper = int(math.ceil(source_position))
        ratio = source_position - lower
        lx, ly = points[lower]
        ux, uy = points[upper]
        result.append((lx + (ux - lx) * ratio, ly + (uy - ly) * ratio))
    return result


@dataclass
class MockOdometryEstimator:
    forward_velocity_mps: float = 0.2
    yaw_rate_radps: float = 0.05

    def estimate(self, stamp: float) -> StampedPoseData:
        return StampedPoseData(
            stamp=stamp,
            pose=Pose2D(x=self.forward_velocity_mps * stamp, y=0.1 * math.sin(stamp), yaw=self.yaw_rate_radps * stamp),
        )


class MockVlmReasoner:
    def __init__(self, scenarios: list[str] | None = None, *, image_size: tuple[int, int] = (640, 480)) -> None:
        self._scenarios = itertools.cycle(scenarios or ["go"])
        self._image_size = image_size
        self._first_stamp: float | None = None

    def _smooth_goal_uv(self, stamp: float) -> tuple[float, float]:
        if self._first_stamp is None:
            self._first_stamp = stamp
        elapsed = stamp - self._first_stamp
        width, height = self._image_size
        center_u = width / 2.0
        center_v = height * 0.68
        amplitude_u = width * 0.22
        amplitude_v = height * 0.08
        u = center_u + amplitude_u * math.sin(elapsed * 0.65)
        v = center_v + amplitude_v * math.cos(elapsed * 0.45)
        margin = 8.0
        return (
            round(max(margin, min(width - margin, u)), 3),
            round(max(margin, min(height - margin, v)), 3),
        )

    def reason(self, stamp: float, pose: Pose2D) -> str:
        scenario = next(self._scenarios)
        if scenario == "malformed":
            return "{bad-json"
        action = {
            "go": VlmAction.GO,
            "stop": VlmAction.STOP,
            "rotate": VlmAction.ROTATE,
        }[scenario]
        return encode_vlm_reasoning(
            stamp=stamp,
            action=action,
            goal_uv=self._smooth_goal_uv(stamp) if action == VlmAction.GO else None,
            pose_frame="odom",
            pose_child_frame="base_link",
            pose_xy_yaw=(pose.x, pose.y, pose.yaw),
            reasoning=f"mock {scenario}",
            rotate_deg=30.0 if action == VlmAction.ROTATE else 0.0,
        )


class MockE2EPlanner:
    def plan(self, goal_point_base_link: tuple[float, float], current_pose: Pose2D) -> TrajectoryPlan:
        del current_pose
        gx, gy = goal_point_base_link
        points = [(gx * (index + 1) / 10.0, gy * (index + 1) / 10.0) for index in range(10)]
        return TrajectoryPlan(points=points, goal_point_base_link=goal_point_base_link, has_goal_point=True, status="FRESH")
