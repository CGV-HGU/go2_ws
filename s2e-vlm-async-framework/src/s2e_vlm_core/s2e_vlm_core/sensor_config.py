from __future__ import annotations

# pyright: reportMissingImports=false

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class SensorConfigError(ValueError):
    pass


@dataclass(frozen=True)
class CameraIntrinsic:
    image_width: int
    image_height: int
    distortion_model: str
    camera_matrix_row_major: tuple[float, ...]
    distortion_coefficients: tuple[float, ...]
    rectification_matrix_row_major: tuple[float, ...]
    projection_matrix_row_major: tuple[float, ...]


@dataclass(frozen=True)
class SensorConfig:
    sensor_name: str
    parent_frame: str
    child_frame: str
    translation_m: tuple[float, float, float]
    rotation_quaternion_xyzw: tuple[float, float, float, float]
    rotation_matrix_row_major: tuple[float, ...]
    intrinsic: CameraIntrinsic | None = None


def default_sensor_config_dir() -> Path:
    env_path = os.environ.get("S2E_SENSOR_CONFIG_DIR")
    if env_path:
        return Path(env_path)
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("s2e_vlm_bringup")) / "config" / "sensors"
    except Exception:
        return Path(__file__).resolve().parents[2] / "s2e_vlm_bringup" / "config" / "sensors"


def load_all_sensor_configs(config_dir: str | Path | None = None) -> dict[str, SensorConfig]:
    directory = Path(config_dir) if config_dir is not None else default_sensor_config_dir()
    if not directory.is_dir():
        raise SensorConfigError(f"sensor config directory does not exist: {directory}")
    configs: dict[str, SensorConfig] = {}
    for path in sorted(directory.glob("*.yaml")):
        config = _load_sensor_config_path(path)
        configs[config.sensor_name] = config
    if not configs:
        raise SensorConfigError(f"no sensor YAML files found in {directory}")
    return configs


def load_sensor_config(sensor_name: str, config_dir: str | Path | None = None) -> SensorConfig:
    directory = Path(config_dir) if config_dir is not None else default_sensor_config_dir()
    return _load_sensor_config_path(directory / f"{sensor_name}.yaml")


def _load_sensor_config_path(path: Path) -> SensorConfig:
    if not path.is_file():
        raise SensorConfigError(f"sensor config file does not exist: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise SensorConfigError(f"sensor config must be a mapping: {path}")
    sensor_name = _required_str(data, "sensor_name")
    parent_frame = _required_str(data, "parent_frame")
    child_frame = _required_str(data, "child_frame")
    translation_values = _fixed_float_tuple(data.get("translation_m"), 3, "translation_m")
    translation = (translation_values[0], translation_values[1], translation_values[2])
    matrix = _rotation_matrix(data)
    quaternion = _rotation_quaternion(data, matrix)
    intrinsic = _camera_intrinsic(data.get("intrinsic"))
    return SensorConfig(sensor_name, parent_frame, child_frame, translation, quaternion, matrix, intrinsic)


def _required_str(data: dict[str, Any], key: str) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        raise SensorConfigError(f"{key} must be a non-empty string")
    return value


def _float_tuple(value: Any, key: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)):
        raise SensorConfigError(f"{key} must be a numeric list")
    try:
        result = tuple(float(item) for item in value)
    except (TypeError, ValueError) as exc:
        raise SensorConfigError(f"{key} must contain numeric values") from exc
    if not all(math.isfinite(item) for item in result):
        raise SensorConfigError(f"{key} must contain finite values")
    return result


def _fixed_float_tuple(value: Any, length: int, key: str) -> tuple[float, ...]:
    result = _float_tuple(value, key)
    if len(result) != length:
        raise SensorConfigError(f"{key} must contain {length} numeric values")
    return result


def _rotation_matrix(data: dict[str, Any]) -> tuple[float, ...]:
    if "rotation_matrix_row_major" in data:
        return _fixed_float_tuple(data["rotation_matrix_row_major"], 9, "rotation_matrix_row_major")
    quaternion = _fixed_float_tuple(data.get("rotation_quaternion_xyzw"), 4, "rotation_quaternion_xyzw")
    return _quaternion_to_matrix(quaternion)


def _rotation_quaternion(data: dict[str, Any], matrix: tuple[float, ...]) -> tuple[float, float, float, float]:
    if "rotation_quaternion_xyzw" in data:
        qx, qy, qz, qw = _fixed_float_tuple(data["rotation_quaternion_xyzw"], 4, "rotation_quaternion_xyzw")
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1e-12:
            raise SensorConfigError("rotation_quaternion_xyzw must not be zero length")
        return qx / norm, qy / norm, qz / norm, qw / norm
    return _matrix_to_quaternion(matrix)


def _camera_intrinsic(value: Any) -> CameraIntrinsic | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise SensorConfigError("intrinsic must be a mapping")
    image_width = int(value.get("image_width", 0))
    image_height = int(value.get("image_height", 0))
    if image_width <= 0 or image_height <= 0:
        raise SensorConfigError("intrinsic image_width and image_height must be positive")
    k = _fixed_float_tuple(value.get("camera_matrix_row_major"), 9, "camera_matrix_row_major")
    d = _float_tuple(value.get("distortion_coefficients", []), "distortion_coefficients")
    r = _fixed_float_tuple(value.get("rectification_matrix_row_major", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]), 9, "rectification_matrix_row_major")
    p = _fixed_float_tuple(value.get("projection_matrix_row_major", [k[0], k[1], k[2], 0.0, k[3], k[4], k[5], 0.0, k[6], k[7], k[8], 0.0]), 12, "projection_matrix_row_major")
    return CameraIntrinsic(
        image_width=image_width,
        image_height=image_height,
        distortion_model=str(value.get("distortion_model", "plumb_bob")),
        camera_matrix_row_major=k,
        distortion_coefficients=d,
        rectification_matrix_row_major=r,
        projection_matrix_row_major=p,
    )


def _matrix_to_quaternion(matrix: tuple[float, ...]) -> tuple[float, float, float, float]:
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = matrix
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (m21 - m12) / scale
        qy = (m02 - m20) / scale
        qz = (m10 - m01) / scale
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / scale
        qx = 0.25 * scale
        qy = (m01 + m10) / scale
        qz = (m02 + m20) / scale
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / scale
        qx = (m01 + m10) / scale
        qy = 0.25 * scale
        qz = (m12 + m21) / scale
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / scale
        qx = (m02 + m20) / scale
        qy = (m12 + m21) / scale
        qz = 0.25 * scale
    return _normalize_quaternion(qx, qy, qz, qw)


def _quaternion_to_matrix(quaternion: tuple[float, ...]) -> tuple[float, ...]:
    qx, qy, qz, qw = _normalize_quaternion(*quaternion)
    return (
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy - qz * qw),
        2.0 * (qx * qz + qy * qw),
        2.0 * (qx * qy + qz * qw),
        1.0 - 2.0 * (qx * qx + qz * qz),
        2.0 * (qy * qz - qx * qw),
        2.0 * (qx * qz - qy * qw),
        2.0 * (qy * qz + qx * qw),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def _normalize_quaternion(qx: float, qy: float, qz: float, qw: float) -> tuple[float, float, float, float]:
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        raise SensorConfigError("rotation quaternion must not be zero length")
    return qx / norm, qy / norm, qz / norm, qw / norm
