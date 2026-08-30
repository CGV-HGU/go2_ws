"""Pure P8-A gateway admission core with no transport or actuator access.

This module is deliberately limited to validating a future command candidate.
It never imports ROS, sockets, Unitree SDKs, or message types and every result
keeps physical dispatch disabled.  A separately reviewed P8-B process would
still be required to own the one real robot command authority.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Mapping

from .contracts import sha256_canonical


@dataclass(frozen=True)
class GatewayConfig:
    authority_id: str = "escape_nav_pixnav_gateway"
    admission_ttl_s: float = 0.25
    deadman_timeout_s: float = 0.50
    max_translation_step_m: float = 0.25
    max_turn_step_deg: float = 30.0
    max_linear_speed_mps: float = 0.10
    max_angular_speed_rps: float = 0.25
    max_linear_accel_mps2: float = 0.20
    max_angular_accel_rps2: float = 0.50
    max_motion_timeout_s: float = 4.0

    def __post_init__(self) -> None:
        if not self.authority_id or len(self.authority_id) > 128:
            raise ValueError("authority_id must contain 1..128 characters")
        positive = (
            self.admission_ttl_s,
            self.deadman_timeout_s,
            self.max_translation_step_m,
            self.max_turn_step_deg,
            self.max_linear_speed_mps,
            self.max_angular_speed_rps,
            self.max_linear_accel_mps2,
            self.max_angular_accel_rps2,
            self.max_motion_timeout_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("gateway limits must be finite and positive")

    @property
    def sha256(self) -> str:
        return sha256_canonical(asdict(self))


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


class NoActuationGatewayCore:
    """Stateful single-authority checker for a future physical gateway.

    Startup is disarmed and E-stop latched.  Clearing the external E-stop does
    not arm the core: a deliberate manual reset with operator enable is also
    required.  No method in this class can permit or dispatch actuation.
    """

    def __init__(self, config: GatewayConfig | None = None) -> None:
        self.config = config or GatewayConfig()
        self._armed = False
        self._estop_latched = True
        self._shutdown_latched = False
        self._last_sequence_id = -1
        self._last_candidate_ns: int | None = None

    def state(self) -> dict[str, Any]:
        return {
            "schema_version": "go2_p8a_gateway_state_v1",
            "authority_id": self.config.authority_id,
            "gateway_config_sha256": self.config.sha256,
            "armed": self._armed,
            "estop_latched": self._estop_latched,
            "shutdown_latched": self._shutdown_latched,
            "last_sequence_id": self._last_sequence_id,
            "last_candidate_monotonic_ns": self._last_candidate_ns,
            "physical_dispatch_permitted": False,
            "actuation_calls": 0,
        }

    def reset_interlocks(
        self,
        *,
        manual_reset: bool,
        operator_enabled: bool,
        estop_clear: bool,
    ) -> dict[str, Any]:
        reasons: list[str] = []
        if self._shutdown_latched:
            reasons.append("SHUTDOWN_LATCHED")
        if manual_reset is not True:
            reasons.append("MANUAL_RESET_NOT_ASSERTED")
        if operator_enabled is not True:
            reasons.append("OPERATOR_ENABLE_NOT_ASSERTED")
        if estop_clear is not True:
            reasons.append("ESTOP_STATE_NOT_CLEAR")
        reset = not reasons
        if reset:
            self._estop_latched = False
            self._armed = True
        return {
            **self.state(),
            "reset_accepted": reset,
            "reasons": reasons if reasons else ["INTERLOCKS_RESET_FOR_AUDIT"],
        }

    def latch_estop(self, reason: str = "ESTOP_ASSERTED") -> dict[str, Any]:
        self._estop_latched = True
        self._armed = False
        return self._zero_result(reason)

    def shutdown(self) -> dict[str, Any]:
        self._shutdown_latched = True
        self._armed = False
        return self._zero_result("SHUTDOWN_LATCHED")

    def _zero_result(self, reason: str) -> dict[str, Any]:
        return {
            "schema_version": "go2_p8a_gateway_audit_v1",
            "authority_id": self.config.authority_id,
            "gateway_config_sha256": self.config.sha256,
            "gateway_candidate_valid": False,
            "reasons": [reason],
            "intent": {
                "kind": "zero_hold",
                "target_dx_m": 0.0,
                "target_dyaw_deg": 0.0,
                "max_linear_speed_mps": 0.0,
                "max_angular_speed_rps": 0.0,
            },
            "physical_dispatch_permitted": False,
            "actuation_calls": 0,
        }

    def _validate_envelope(self, proposal: Mapping[str, Any]) -> list[str]:
        reasons: list[str] = []
        kind = proposal.get("proposal_kind")
        numeric = {
            name: proposal.get(name)
            for name in (
                "target_dx_m",
                "target_dyaw_deg",
                "max_linear_speed_mps",
                "max_angular_speed_rps",
                "max_linear_accel_mps2",
                "max_angular_accel_rps2",
                "timeout_s",
            )
        }
        if not all(_finite_number(value) for value in numeric.values()):
            return ["COMMAND_ENVELOPE_NONFINITE_OR_MISSING"]

        target_dx = float(numeric["target_dx_m"])
        target_yaw = float(numeric["target_dyaw_deg"])
        if kind == "translate":
            if not 0.0 < target_dx <= self.config.max_translation_step_m or target_yaw != 0.0:
                reasons.append("TRANSLATION_TARGET_OUT_OF_BOUNDS")
        elif kind == "rotate":
            if target_dx != 0.0 or not 0.0 < abs(target_yaw) <= self.config.max_turn_step_deg:
                reasons.append("ROTATION_TARGET_OUT_OF_BOUNDS")
        else:
            reasons.append("NON_MOTION_PROPOSAL")

        bounds = (
            ("LINEAR_SPEED_OUT_OF_BOUNDS", numeric["max_linear_speed_mps"], self.config.max_linear_speed_mps),
            ("ANGULAR_SPEED_OUT_OF_BOUNDS", numeric["max_angular_speed_rps"], self.config.max_angular_speed_rps),
            ("LINEAR_ACCEL_OUT_OF_BOUNDS", numeric["max_linear_accel_mps2"], self.config.max_linear_accel_mps2),
            ("ANGULAR_ACCEL_OUT_OF_BOUNDS", numeric["max_angular_accel_rps2"], self.config.max_angular_accel_rps2),
            ("MOTION_TIMEOUT_OUT_OF_BOUNDS", numeric["timeout_s"], self.config.max_motion_timeout_s),
        )
        for reason, value, maximum in bounds:
            if not 0.0 < float(value) <= maximum:
                reasons.append(reason)
        return reasons

    def evaluate(
        self,
        proposal: Mapping[str, Any],
        safety_admission: Mapping[str, Any],
        *,
        authority_id: str,
        received_at_ns: int,
        evaluated_at_ns: int,
        operator_enabled: bool,
        estop_clear: bool,
    ) -> dict[str, Any]:
        """Validate one P7-linked candidate while keeping dispatch disabled."""

        if any(
            isinstance(value, bool) or not isinstance(value, int) or value < 0
            for value in (received_at_ns, evaluated_at_ns)
        ):
            raise ValueError("gateway timestamps must be nonnegative integers")
        reasons: list[str] = []
        if self._shutdown_latched:
            reasons.append("SHUTDOWN_LATCHED")
        if authority_id != self.config.authority_id:
            reasons.append("AUTHORITY_ID_MISMATCH")
        if estop_clear is not True:
            self._estop_latched = True
            self._armed = False
            reasons.append("ESTOP_ASSERTED")
        if self._estop_latched:
            reasons.append("ESTOP_LATCHED_MANUAL_RESET_REQUIRED")
        if operator_enabled is not True:
            reasons.append("OPERATOR_ENABLE_NOT_ASSERTED")
        if not self._armed:
            reasons.append("GATEWAY_NOT_ARMED")

        sequence_id = proposal.get("sequence_id")
        if isinstance(sequence_id, bool) or not isinstance(sequence_id, int) or sequence_id < 0:
            reasons.append("INVALID_SEQUENCE_ID")
        elif sequence_id <= self._last_sequence_id:
            reasons.append("DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE")

        if proposal.get("schema_version") != "go2_pixnav_macro_proposal_v1":
            reasons.append("UNSUPPORTED_PROPOSAL_SCHEMA")
        if proposal.get("accepted") is not True:
            reasons.append("UPSTREAM_PROPOSAL_REJECTED")
        if proposal.get("actuation_permitted") is not False:
            reasons.append("UPSTREAM_ACTUATION_INTERLOCK_VIOLATION")
        reasons.extend(self._validate_envelope(proposal))

        if safety_admission.get("schema_version") != "go2_pixnav_safety_admission_v1":
            reasons.append("UNSUPPORTED_SAFETY_ADMISSION_SCHEMA")
        if safety_admission.get("actuation_permitted") is not False:
            reasons.append("P7_ACTUATION_INTERLOCK_VIOLATION")
        if safety_admission.get("admitted_to_gateway") is not True:
            reasons.append("P7_ADMISSION_REJECTED")
        if safety_admission.get("proposal_sha256") != sha256_canonical(dict(proposal)):
            reasons.append("P7_PROPOSAL_HASH_MISMATCH")

        admission_at_ns = safety_admission.get("evaluated_at_ns")
        if (
            isinstance(admission_at_ns, bool)
            or not isinstance(admission_at_ns, int)
            or not 0 <= admission_at_ns <= received_at_ns <= evaluated_at_ns
        ):
            reasons.append("P7_GATEWAY_TIMESTAMP_ORDER_INVALID")
        elif (evaluated_at_ns - admission_at_ns) / 1e9 > self.config.admission_ttl_s:
            reasons.append("P7_ADMISSION_STALE")

        valid = not reasons
        if valid:
            self._last_sequence_id = int(sequence_id)
            self._last_candidate_ns = evaluated_at_ns
        intent = {
            "kind": proposal.get("proposal_kind") if valid else "zero_hold",
            "target_dx_m": float(proposal["target_dx_m"]) if valid else 0.0,
            "target_dyaw_deg": float(proposal["target_dyaw_deg"]) if valid else 0.0,
            "max_linear_speed_mps": (
                float(proposal["max_linear_speed_mps"]) if valid else 0.0
            ),
            "max_angular_speed_rps": (
                float(proposal["max_angular_speed_rps"]) if valid else 0.0
            ),
        }
        result = {
            "schema_version": "go2_p8a_gateway_audit_v1",
            "authority_id": authority_id,
            "gateway_config_sha256": self.config.sha256,
            "proposal_sha256": sha256_canonical(dict(proposal)),
            "safety_admission_sha256": sha256_canonical(dict(safety_admission)),
            "sequence_id": sequence_id,
            "received_at_ns": received_at_ns,
            "evaluated_at_ns": evaluated_at_ns,
            "gateway_candidate_valid": valid,
            "reasons": reasons if reasons else ["P8A_GATEWAY_CONTRACT_PASS"],
            "intent": intent,
            "physical_dispatch_permitted": False,
            "actuation_calls": 0,
            "claim_scope": (
                "P8-A pure gateway admission only; not a ROS/SDK dispatcher, command ACK, "
                "measured stop latency, physical E-stop, or robot-motion proof."
            ),
        }
        result["canonical_sha256"] = sha256_canonical(result)
        return result

    def deadman(self, *, evaluated_at_ns: int) -> dict[str, Any]:
        if isinstance(evaluated_at_ns, bool) or not isinstance(evaluated_at_ns, int) or evaluated_at_ns < 0:
            raise ValueError("evaluated_at_ns must be a nonnegative integer")
        if self._last_candidate_ns is None:
            reason = "NO_ACCEPTED_GATEWAY_CANDIDATE"
            zero_required = True
        elif evaluated_at_ns < self._last_candidate_ns:
            reason = "DEADMAN_CLOCK_REVERSAL"
            zero_required = True
        elif (evaluated_at_ns - self._last_candidate_ns) / 1e9 > self.config.deadman_timeout_s:
            reason = "DEADMAN_TIMEOUT_ZERO_REQUIRED"
            zero_required = True
        else:
            reason = "DEADMAN_WINDOW_ACTIVE_NO_DISPATCH"
            zero_required = False
        result = self._zero_result(reason)
        result["zero_required"] = zero_required
        result["evaluated_at_ns"] = evaluated_at_ns
        result["last_candidate_monotonic_ns"] = self._last_candidate_ns
        return result

