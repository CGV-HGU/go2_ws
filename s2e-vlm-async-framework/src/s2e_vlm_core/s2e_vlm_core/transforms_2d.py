from __future__ import annotations

import math

from .pose_buffer import Pose2D


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def relative_pose_2d(reference: Pose2D, current: Pose2D) -> Pose2D:
    dx = current.x - reference.x
    dy = current.y - reference.y
    cos_yaw = math.cos(-reference.yaw)
    sin_yaw = math.sin(-reference.yaw)
    return Pose2D(
        x=cos_yaw * dx - sin_yaw * dy,
        y=sin_yaw * dx + cos_yaw * dy,
        yaw=wrap_angle(current.yaw - reference.yaw),
    )


def transform_point_2d(point: tuple[float, float], relative: Pose2D) -> tuple[float, float]:
    compensated_x = point[0] - relative.x
    compensated_y = point[1] - relative.y - math.sin(relative.yaw) * compensated_x
    return compensated_x, compensated_y


def compensation_within_bounds(relative: Pose2D, max_translation_m: float, max_yaw_rad: float) -> bool:
    return math.hypot(relative.x, relative.y) <= max_translation_m and abs(wrap_angle(relative.yaw)) <= max_yaw_rad
