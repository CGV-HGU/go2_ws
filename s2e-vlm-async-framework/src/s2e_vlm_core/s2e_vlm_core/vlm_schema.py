from __future__ import annotations

import json
import math
from dataclasses import dataclass
from enum import Enum
from typing import Any


class VlmAction(str, Enum):
    GO = "go"
    STOP = "stop"
    ROTATE = "rotate"
    NO_COMMAND = "NO_COMMAND"


@dataclass(frozen=True)
class PoseSnapshot:
    frame_id: str
    child_frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class VlmParseResult:
    valid: bool
    action: VlmAction
    reason: str
    stamp: float | None = None
    frame_id: str | None = None
    goal_uv: tuple[float, float] | None = None
    rotate_deg: float = 0.0
    pose: PoseSnapshot | None = None
    reasoning: str = ""
    source_json: str = ""


def _invalid(reason: str, source_json: str) -> VlmParseResult:
    return VlmParseResult(False, VlmAction.NO_COMMAND, reason, source_json=source_json)


def _stamp_to_float(stamp: dict[str, Any]) -> float:
    return float(stamp["sec"]) + float(stamp["nanosec"]) / 1_000_000_000.0


def _reject_json_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON numeric constant: {value}")


def _finite_float(value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("numeric field must be finite")
    return result


def parse_vlm_reasoning(payload: str) -> VlmParseResult:
    try:
        raw = json.loads(payload, parse_constant=_reject_json_constant)
    except json.JSONDecodeError:
        return _invalid("MALFORMED_JSON", payload)
    except ValueError:
        return _invalid("NON_FINITE_NUMERIC_FIELD", payload)

    if raw.get("schema_version") != 0:
        return _invalid("UNSUPPORTED_SCHEMA", payload)

    raw_action = raw.get("action")
    try:
        action = VlmAction(raw_action)
    except ValueError:
        return _invalid("INVALID_ACTION", payload)

    try:
        stamp = _stamp_to_float(raw["stamp"])
        pose_raw = raw["pose"]
        frame_id = str(pose_raw["frame_id"])
        child_frame_id = str(pose_raw["child_frame_id"])
    except (KeyError, TypeError, ValueError):
        return _invalid("MISSING_REQUIRED_FIELD", payload)
    if not math.isfinite(stamp):
        return _invalid("NON_FINITE_NUMERIC_FIELD", payload)
    try:
        pose = PoseSnapshot(
            frame_id=frame_id,
            child_frame_id=child_frame_id,
            x=_finite_float(pose_raw["x"]),
            y=_finite_float(pose_raw["y"]),
            z=_finite_float(pose_raw["z"]),
            qx=_finite_float(pose_raw["qx"]),
            qy=_finite_float(pose_raw["qy"]),
            qz=_finite_float(pose_raw["qz"]),
            qw=_finite_float(pose_raw["qw"]),
        )
    except (KeyError, TypeError):
        return _invalid("MISSING_REQUIRED_FIELD", payload)
    except ValueError:
        return _invalid("NON_FINITE_NUMERIC_FIELD", payload)

    goal_uv = None
    if action == VlmAction.GO:
        try:
            goal_uv = (_finite_float(raw["goal_uv"]["u"]), _finite_float(raw["goal_uv"]["v"]))
        except (KeyError, TypeError, ValueError):
            return _invalid("NON_FINITE_NUMERIC_FIELD" if "goal_uv" in raw else "MISSING_GOAL_UV", payload)

    try:
        rotate_deg = _finite_float(raw.get("rotate_deg", 0.0) or 0.0)
    except (TypeError, ValueError):
        return _invalid("NON_FINITE_NUMERIC_FIELD", payload)
    if action == VlmAction.ROTATE and rotate_deg == 0.0:
        return _invalid("MISSING_ROTATE_DEG", payload)

    return VlmParseResult(
        valid=True,
        action=action,
        reason="OK",
        stamp=stamp,
        frame_id=str(raw.get("frame_id", "")),
        goal_uv=goal_uv,
        rotate_deg=rotate_deg,
        pose=pose,
        reasoning=str(raw.get("reasoning", "")),
        source_json=payload,
    )


def encode_vlm_reasoning(
    *,
    stamp: float,
    action: VlmAction,
    goal_uv: tuple[float, float] | None,
    pose_frame: str,
    pose_child_frame: str,
    pose_xy_yaw: tuple[float, float, float],
    reasoning: str,
    rotate_deg: float = 0.0,
) -> str:
    sec = int(stamp)
    nanosec = int(round((stamp - sec) * 1_000_000_000))
    x, y, yaw = pose_xy_yaw
    payload: dict[str, Any] = {
        "schema_version": 0,
        "stamp": {"sec": sec, "nanosec": nanosec},
        "frame_id": "camera",
        "action": action.value,
        "rotate_deg": rotate_deg,
        "pose": {
            "frame_id": pose_frame,
            "child_frame_id": pose_child_frame,
            "x": x,
            "y": y,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": math.sin(yaw / 2.0),
            "qw": math.cos(yaw / 2.0),
        },
        "reasoning": reasoning,
    }
    if goal_uv is not None:
        payload["goal_uv"] = {"u": goal_uv[0], "v": goal_uv[1]}
    return json.dumps(payload, separators=(",", ":"), sort_keys=True)
