"""Deterministic conversion from ROS velocity commands to Habitat actions."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class HabitatAction(str, Enum):
    HOLD = "hold"
    STOP = "stop"
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    LOOK_UP = "look_up"
    LOOK_DOWN = "look_down"


class HabitatActionError(ValueError):
    """Fail-closed action conversion error with a stable machine code."""

    def __init__(self, code: str, detail: str = "") -> None:
        self.code = code
        self.detail = detail
        message = code if not detail else "{}: {}".format(code, detail)
        super().__init__(message)


def _finite_number(value: object, code: str) -> float:
    if isinstance(value, bool):
        raise HabitatActionError(code)
    try:
        converted = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise HabitatActionError(code, str(error)) from error
    if not math.isfinite(converted):
        raise HabitatActionError(code)
    return converted


@dataclass(frozen=True)
class HabitatStepBudget:
    """Admission rules that preserve exactly one final native STOP action."""

    max_episode_steps: int = 400
    reserved_stop_steps: int = 1

    def __post_init__(self) -> None:
        if (
            not isinstance(self.max_episode_steps, int)
            or isinstance(self.max_episode_steps, bool)
            or self.max_episode_steps <= 0
        ):
            raise HabitatActionError("INVALID_MAX_EPISODE_STEPS")
        if self.reserved_stop_steps != 1 or isinstance(self.reserved_stop_steps, bool):
            raise HabitatActionError("INVALID_RESERVED_STOP_STEPS")

    @property
    def stop_threshold(self) -> int:
        return self.max_episode_steps - self.reserved_stop_steps

    def validate_action(self, action: str, *, current_step_index: int) -> None:
        self._validate_step_index(current_step_index)
        if action == HabitatAction.HOLD.value:
            return
        if action not in {
            HabitatAction.STOP.value,
            HabitatAction.MOVE_FORWARD.value,
            HabitatAction.MOVE_BACKWARD.value,
            HabitatAction.TURN_LEFT.value,
            HabitatAction.TURN_RIGHT.value,
            HabitatAction.LOOK_UP.value,
            HabitatAction.LOOK_DOWN.value,
        }:
            raise HabitatActionError("INVALID_ACTION")
        if current_step_index >= self.max_episode_steps:
            raise HabitatActionError("EPISODE_STEP_LIMIT_REACHED")
        if (
            action != HabitatAction.STOP.value
            and current_step_index >= self.stop_threshold
        ):
            raise HabitatActionError("STOP_STEP_RESERVED")

    def should_latch_stop(self, completed_step_index: int) -> bool:
        self._validate_step_index(completed_step_index)
        return completed_step_index >= self.stop_threshold

    @staticmethod
    def _validate_step_index(value: int) -> None:
        if not isinstance(value, int) or isinstance(value, bool) or value < 0:
            raise HabitatActionError("INVALID_HABITAT_STEP_INDEX")


def bounded_forward_distance_m(
    *,
    desired_speed_mps: float,
    controller_max_speed_mps: float,
    action_period_s: float,
    remaining_distance_m: float,
    max_forward_step_m: float,
) -> float:
    """Return one physical step without changing the metric trajectory."""
    source_values = (
        desired_speed_mps,
        controller_max_speed_mps,
        action_period_s,
        remaining_distance_m,
        max_forward_step_m,
    )
    if any(isinstance(value, bool) for value in source_values):
        raise HabitatActionError("INVALID_FORWARD_MOTION")
    values = tuple(
        _finite_number(value, "INVALID_FORWARD_MOTION") for value in source_values
    )
    if any(value <= 0.0 for value in values):
        raise HabitatActionError("INVALID_FORWARD_MOTION")
    desired_speed, controller_max_speed, period, remaining, max_step = values
    return min(
        min(desired_speed, controller_max_speed) * period,
        remaining,
        max_step,
    )


class HabitatActionQuantizer:
    """Give turns priority and reject motion Habitat cannot represent safely."""

    def __init__(
        self,
        *,
        linear_deadband_mps: float = 0.02,
        angular_deadband_radps: float = 0.05,
    ) -> None:
        self.linear_deadband_mps = _finite_number(
            linear_deadband_mps, "INVALID_LINEAR_DEADBAND"
        )
        self.angular_deadband_radps = _finite_number(
            angular_deadband_radps, "INVALID_ANGULAR_DEADBAND"
        )
        if self.linear_deadband_mps < 0.0:
            raise HabitatActionError("INVALID_LINEAR_DEADBAND")
        if self.angular_deadband_radps < 0.0:
            raise HabitatActionError("INVALID_ANGULAR_DEADBAND")

    def choose(
        self,
        linear_x_mps: float,
        angular_z_radps: float,
        *,
        linear_y_mps: float = 0.0,
    ) -> HabitatAction:
        linear_x = _finite_number(linear_x_mps, "NON_FINITE_TWIST")
        linear_y = _finite_number(linear_y_mps, "NON_FINITE_TWIST")
        angular_z = _finite_number(angular_z_radps, "NON_FINITE_TWIST")
        if abs(linear_y) > self.linear_deadband_mps:
            raise HabitatActionError("LATERAL_UNSUPPORTED")
        if linear_x < -self.linear_deadband_mps:
            raise HabitatActionError("REVERSE_UNSUPPORTED")
        if abs(angular_z) > self.angular_deadband_radps:
            return (
                HabitatAction.TURN_LEFT if angular_z > 0.0 else HabitatAction.TURN_RIGHT
            )
        if linear_x > self.linear_deadband_mps:
            return HabitatAction.MOVE_FORWARD
        return HabitatAction.HOLD
