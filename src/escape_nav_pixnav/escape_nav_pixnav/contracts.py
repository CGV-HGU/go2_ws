"""Pure data contracts for PixNav decisions and file-only macro proposals."""

from __future__ import annotations

import hashlib
import json
import math
import re
from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any, Mapping, Sequence


ACTION_NAMES = (
    "stop",
    "forward",
    "turn_left",
    "turn_right",
    "look_up",
    "look_down",
)
PIXNAV_CHECKPOINT_A_SHA256 = (
    "0b1faff7631962351bbbfe8cb115a3a03069f33fab499865f887ffbb5a3cabe3"
)
PIXNAV_REFERENCE_COMMIT = "6341a5d33903131ddfce74498c04e1c0ae04ec61"
_EVENT_ID_RE = re.compile(r"^[A-Za-z0-9_.:-]{1,160}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


class PixNavAction(str, Enum):
    STOP = "stop"
    FORWARD = "forward"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    LOOK_UP = "look_up"
    LOOK_DOWN = "look_down"


class TimeBasis(str, Enum):
    MONOTONIC_LIVE = "monotonic_live"
    OFFLINE_REPLAY = "offline_replay"


class ProposalKind(str, Enum):
    ZERO_HOLD = "zero_hold"
    TRANSLATE = "translate"
    ROTATE = "rotate"
    REOBSERVE = "reobserve"


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def sha256_canonical(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()


def is_sha256(value: str) -> bool:
    return bool(_SHA256_RE.fullmatch(value))


@dataclass(frozen=True)
class AdapterConfig:
    expected_checkpoint_sha256: str = PIXNAV_CHECKPOINT_A_SHA256
    selected_probability_min: float = 0.55
    source_frame_ttl_s: float = 1.0
    decision_ttl_s: float = 0.50
    translation_step_m: float = 0.25
    turn_step_deg: float = 30.0
    max_linear_speed_mps: float = 0.10
    max_angular_speed_rps: float = 0.25
    max_linear_accel_mps2: float = 0.20
    max_angular_accel_rps2: float = 0.50
    translation_timeout_s: float = 4.0
    rotation_timeout_s: float = 4.0
    position_tolerance_m: float = 0.03
    yaw_tolerance_deg: float = 3.0

    def __post_init__(self) -> None:
        if not is_sha256(self.expected_checkpoint_sha256):
            raise ValueError("expected_checkpoint_sha256 must be lowercase SHA-256")
        if not 0.0 <= self.selected_probability_min <= 1.0:
            raise ValueError("selected_probability_min must be in [0, 1]")
        positive = (
            self.source_frame_ttl_s,
            self.decision_ttl_s,
            self.translation_step_m,
            self.turn_step_deg,
            self.max_linear_speed_mps,
            self.max_angular_speed_rps,
            self.max_linear_accel_mps2,
            self.max_angular_accel_rps2,
            self.translation_timeout_s,
            self.rotation_timeout_s,
            self.position_tolerance_m,
            self.yaw_tolerance_deg,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in positive):
            raise ValueError("all adapter limits must be finite and positive")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @property
    def sha256(self) -> str:
        return sha256_canonical(self.to_dict())


@dataclass(frozen=True)
class PixNavDecision:
    schema_version: str
    event_id: str
    sequence_id: int
    source_frame_sha256: str
    checkpoint_sha256: str
    observed_at_ns: int
    inferred_at_ns: int
    time_basis: TimeBasis
    action_id: int
    action: PixNavAction
    action_probabilities: tuple[float, float, float, float, float, float]
    finite: bool

    @property
    def selected_probability(self) -> float:
        return self.action_probabilities[self.action_id]

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["time_basis"] = self.time_basis.value
        value["action"] = self.action.value
        value["action_probabilities"] = {
            name: probability
            for name, probability in zip(ACTION_NAMES, self.action_probabilities)
        }
        return value

    @classmethod
    def from_mapping(cls, raw: Mapping[str, Any]) -> "PixNavDecision":
        if str(raw.get("schema_version")) != "go2_pixnav_decision_v1":
            raise ValueError("UNSUPPORTED_DECISION_SCHEMA")
        event_id = str(raw.get("event_id", ""))
        if not _EVENT_ID_RE.fullmatch(event_id):
            raise ValueError("INVALID_EVENT_ID")
        if isinstance(raw.get("sequence_id"), bool) or not isinstance(raw.get("sequence_id"), int):
            raise ValueError("INVALID_SEQUENCE_ID")
        sequence_id = raw["sequence_id"]
        if sequence_id < 0:
            raise ValueError("INVALID_SEQUENCE_ID")
        source_hash = str(raw.get("source_frame_sha256", ""))
        checkpoint_hash = str(raw.get("checkpoint_sha256", ""))
        if not is_sha256(source_hash):
            raise ValueError("INVALID_SOURCE_FRAME_HASH")
        if not is_sha256(checkpoint_hash):
            raise ValueError("INVALID_CHECKPOINT_HASH")

        if isinstance(raw.get("action_id"), bool) or not isinstance(raw.get("action_id"), int):
            raise ValueError("INVALID_ACTION_ID")
        action_id = raw["action_id"]
        if not 0 <= action_id < len(ACTION_NAMES):
            raise ValueError("INVALID_ACTION_ID")
        action = PixNavAction(str(raw["action"]))
        if action.value != ACTION_NAMES[action_id]:
            raise ValueError("ACTION_ID_NAME_MISMATCH")

        probability_raw = raw["action_probabilities"]
        if isinstance(probability_raw, Mapping):
            probabilities: Sequence[Any] = [probability_raw[name] for name in ACTION_NAMES]
        else:
            probabilities = probability_raw
        if len(probabilities) != len(ACTION_NAMES):
            raise ValueError("INVALID_PROBABILITY_SHAPE")
        probability_tuple = tuple(float(value) for value in probabilities)
        if not all(math.isfinite(value) and 0.0 <= value <= 1.0 for value in probability_tuple):
            raise ValueError("NON_FINITE_OR_OUT_OF_RANGE_PROBABILITY")
        if abs(sum(probability_tuple) - 1.0) > 1e-3:
            raise ValueError("PROBABILITIES_DO_NOT_SUM_TO_ONE")
        if max(range(len(probability_tuple)), key=probability_tuple.__getitem__) != action_id:
            raise ValueError("ACTION_IS_NOT_ARGMAX")

        if (
            isinstance(raw.get("observed_at_ns"), bool)
            or not isinstance(raw.get("observed_at_ns"), int)
            or isinstance(raw.get("inferred_at_ns"), bool)
            or not isinstance(raw.get("inferred_at_ns"), int)
        ):
            raise ValueError("INVALID_TIMESTAMP_TYPE")
        observed_at_ns = raw["observed_at_ns"]
        inferred_at_ns = raw["inferred_at_ns"]
        if observed_at_ns < 0 or inferred_at_ns < 0:
            raise ValueError("NEGATIVE_TIMESTAMP")
        time_basis = TimeBasis(str(raw["time_basis"]))
        if time_basis == TimeBasis.MONOTONIC_LIVE and observed_at_ns > inferred_at_ns:
            raise ValueError("TIMESTAMP_ORDER_INVALID")
        if time_basis == TimeBasis.OFFLINE_REPLAY and (observed_at_ns != 0 or inferred_at_ns != 0):
            raise ValueError("OFFLINE_REPLAY_REQUIRES_ZERO_TIMESTAMPS")

        finite = raw.get("finite")
        if not isinstance(finite, bool):
            raise ValueError("INVALID_FINITE_FLAG")

        return cls(
            schema_version="go2_pixnav_decision_v1",
            event_id=event_id,
            sequence_id=sequence_id,
            source_frame_sha256=source_hash,
            checkpoint_sha256=checkpoint_hash,
            observed_at_ns=observed_at_ns,
            inferred_at_ns=inferred_at_ns,
            time_basis=time_basis,
            action_id=action_id,
            action=action,
            action_probabilities=probability_tuple,  # type: ignore[arg-type]
            finite=finite,
        )


@dataclass(frozen=True)
class MacroActionProposal:
    schema_version: str
    event_id: str
    sequence_id: int
    source_frame_sha256: str
    checkpoint_sha256: str
    adapter_config_sha256: str
    time_basis: TimeBasis
    pixnav_action_id: int
    pixnav_action: str
    selected_probability: float
    accepted: bool
    proposal_kind: ProposalKind
    reason: str
    target_dx_m: float
    target_dyaw_deg: float
    max_linear_speed_mps: float
    max_angular_speed_rps: float
    max_linear_accel_mps2: float
    max_angular_accel_rps2: float
    timeout_s: float
    position_tolerance_m: float
    yaw_tolerance_deg: float
    requires_reobservation: bool
    actuation_permitted: bool = False

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["time_basis"] = self.time_basis.value
        value["proposal_kind"] = self.proposal_kind.value
        return value
