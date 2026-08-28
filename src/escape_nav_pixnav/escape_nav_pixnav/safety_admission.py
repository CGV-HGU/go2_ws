"""Pure P7 safety admission for file-only PixNav macro proposals."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping

from .contracts import sha256_canonical


@dataclass(frozen=True)
class SafetyAdmissionConfig:
    sensor_ttl_s: float = 0.50
    source_frame_ttl_s: float = 1.00
    pixnav_decision_ttl_s: float = 0.50
    cloud_odom_sync_max_s: float = 0.10
    valid_cloud_points_min: int = 100
    front_clearance_min_m: float = 0.55
    rotation_clearance_min_m: float = 0.45
    odom_step_max_m: float = 0.20
    odom_yaw_step_max_deg: float = 25.0
    require_global_localization: bool = False

    def __post_init__(self) -> None:
        positive = (
            self.sensor_ttl_s,
            self.source_frame_ttl_s,
            self.pixnav_decision_ttl_s,
            self.cloud_odom_sync_max_s,
            self.front_clearance_min_m,
            self.rotation_clearance_min_m,
            self.odom_step_max_m,
            self.odom_yaw_step_max_deg,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("safety thresholds must be finite and positive")
        if self.valid_cloud_points_min < 1:
            raise ValueError("valid_cloud_points_min must be positive")

    @property
    def sha256(self) -> str:
        return sha256_canonical(asdict(self))


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _nonnegative_finite(value: Any) -> bool:
    return _finite_number(value) and float(value) >= 0.0


def evaluate_safety_admission(
    proposal: Mapping[str, Any],
    sensor_snapshot: Mapping[str, Any],
    *,
    evaluated_at_ns: int,
    decision_observed_at_ns: int | None,
    decision_inferred_at_ns: int | None,
    operator_enabled: bool,
    estop_clear: bool,
    global_localization_available: bool,
    config: SafetyAdmissionConfig | None = None,
) -> dict[str, Any]:
    """Evaluate a proposal without creating or authorizing a robot command.

    ``admitted_to_gateway`` only means the candidate passed this P7 software
    gate.  P8 remains a separate authority and this artifact always carries
    ``actuation_permitted=false``.
    """

    cfg = config or SafetyAdmissionConfig()
    reasons: list[str] = []
    if isinstance(evaluated_at_ns, bool) or not isinstance(evaluated_at_ns, int) or evaluated_at_ns < 0:
        raise ValueError("evaluated_at_ns must be a nonnegative integer")
    proposal_kind = proposal.get("proposal_kind")
    if proposal.get("schema_version") != "go2_pixnav_macro_proposal_v1":
        reasons.append("UNSUPPORTED_PROPOSAL_SCHEMA")
    if proposal.get("actuation_permitted") is not False:
        reasons.append("UPSTREAM_ACTUATION_INTERLOCK_VIOLATION")
    moving_candidate = proposal.get("accepted") is True and proposal_kind in {"translate", "rotate"}
    if not moving_candidate:
        reasons.append("UPSTREAM_PROPOSAL_NOT_MOTION_CANDIDATE")
    else:
        if (
            isinstance(decision_observed_at_ns, bool)
            or not isinstance(decision_observed_at_ns, int)
            or isinstance(decision_inferred_at_ns, bool)
            or not isinstance(decision_inferred_at_ns, int)
        ):
            reasons.append("DECISION_TIMESTAMPS_INVALID")
        elif not 0 <= decision_observed_at_ns <= decision_inferred_at_ns <= evaluated_at_ns:
            reasons.append("DECISION_TIMESTAMP_ORDER_INVALID")
        else:
            frame_age_s = (evaluated_at_ns - decision_observed_at_ns) / 1e9
            decision_age_s = (evaluated_at_ns - decision_inferred_at_ns) / 1e9
            if frame_age_s > cfg.source_frame_ttl_s:
                reasons.append("SOURCE_FRAME_STALE_AT_SAFETY_GATE")
            if decision_age_s > cfg.pixnav_decision_ttl_s:
                reasons.append("PIXNAV_DECISION_STALE_AT_SAFETY_GATE")
    if operator_enabled is not True:
        reasons.append("OPERATOR_ENABLE_NOT_ASSERTED")
    if estop_clear is not True:
        reasons.append("ESTOP_STATE_NOT_CLEAR")
    if cfg.require_global_localization and global_localization_available is not True:
        reasons.append("GLOBAL_LOCALIZATION_UNAVAILABLE")

    if sensor_snapshot.get("status") != "PASS_LIVE_L2_ODOM_SNAPSHOT":
        reasons.append("LIVE_SENSOR_SNAPSHOT_INVALID")
    cloud_received_ns = sensor_snapshot.get("cloud_received_monotonic_ns")
    odom_received_ns = sensor_snapshot.get("odom_received_monotonic_ns")
    for label, received_ns in (
        ("CLOUD", cloud_received_ns),
        ("ODOM", odom_received_ns),
    ):
        if isinstance(received_ns, bool) or not isinstance(received_ns, int):
            reasons.append(f"{label}_RECEIPT_TIMESTAMP_INVALID")
            continue
        if received_ns > evaluated_at_ns:
            reasons.append(f"{label}_RECEIPT_AFTER_EVALUATION")
        elif (evaluated_at_ns - received_ns) / 1e9 > cfg.sensor_ttl_s:
            reasons.append(f"{label}_STALE")

    sync_delta_s = sensor_snapshot.get("cloud_odom_stamp_delta_s")
    if not _nonnegative_finite(sync_delta_s) or float(sync_delta_s) > cfg.cloud_odom_sync_max_s:
        reasons.append("CLOUD_ODOM_NOT_SYNCHRONIZED")
    valid_points = sensor_snapshot.get("valid_cloud_points")
    if isinstance(valid_points, bool) or not isinstance(valid_points, int):
        reasons.append("VALID_CLOUD_POINT_COUNT_INVALID")
    elif valid_points < cfg.valid_cloud_points_min:
        reasons.append("VALID_CLOUD_POINTS_TOO_FEW")
    odom_step_m = sensor_snapshot.get("max_odom_step_m")
    if not _nonnegative_finite(odom_step_m) or float(odom_step_m) > cfg.odom_step_max_m:
        reasons.append("ODOM_POSITION_JUMP")
    yaw_step_deg = sensor_snapshot.get("max_odom_yaw_step_deg")
    if not _nonnegative_finite(yaw_step_deg) or float(yaw_step_deg) > cfg.odom_yaw_step_max_deg:
        reasons.append("ODOM_YAW_JUMP")

    if proposal_kind == "translate":
        clearance = sensor_snapshot.get("front_clearance_m")
        if not _finite_number(clearance) or float(clearance) < cfg.front_clearance_min_m:
            reasons.append("FRONT_CLEARANCE_BLOCKED")
    elif proposal_kind == "rotate":
        clearance = sensor_snapshot.get("rotation_clearance_m")
        if not _finite_number(clearance) or float(clearance) < cfg.rotation_clearance_min_m:
            reasons.append("ROTATION_CLEARANCE_BLOCKED")

    admitted = moving_candidate and not reasons
    return {
        "schema_version": "go2_pixnav_safety_admission_v1",
        "proposal_sha256": sha256_canonical(dict(proposal)),
        "sensor_snapshot_sha256": sha256_canonical(dict(sensor_snapshot)),
        "safety_config_sha256": cfg.sha256,
        "evaluated_at_ns": evaluated_at_ns,
        "decision_observed_at_ns": decision_observed_at_ns,
        "decision_inferred_at_ns": decision_inferred_at_ns,
        "operator_enabled": operator_enabled,
        "estop_clear": estop_clear,
        "global_localization_available": global_localization_available,
        "require_global_localization": cfg.require_global_localization,
        "admitted_to_gateway": admitted,
        "reasons": reasons if reasons else ["P7_SAFETY_CHECKS_PASS"],
        "actuation_permitted": False,
        "claim_scope": (
            "P7 candidate admission only; not a P8 command authority, E-stop wiring, "
            "controller, stop-latency, or robot-motion proof."
        ),
        "canonical_sha256": sha256_canonical(
            {
                "proposal": dict(proposal),
                "snapshot": dict(sensor_snapshot),
                "config": asdict(cfg),
                "evaluated_at_ns": evaluated_at_ns,
                "decision_observed_at_ns": decision_observed_at_ns,
                "decision_inferred_at_ns": decision_inferred_at_ns,
                "operator_enabled": operator_enabled,
                "estop_clear": estop_clear,
                "global_localization_available": global_localization_available,
            }
        ),
    }
