"""In-memory causal admission and deadman contracts for a future live chain."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Any

from .contracts import is_sha256, sha256_canonical


ZERO_HASH = "0" * 64


class EventStage(str, Enum):
    FRAME_CAPTURED = "frame_captured"
    VLM_SUBMITTED = "vlm_submitted"
    VLM_COMPLETED = "vlm_completed"
    PIXNAV_COMPLETED = "pixnav_completed"
    MACRO_AUDITED = "macro_audited"


STAGE_ORDER = (
    EventStage.FRAME_CAPTURED,
    EventStage.VLM_SUBMITTED,
    EventStage.VLM_COMPLETED,
    EventStage.PIXNAV_COMPLETED,
    EventStage.MACRO_AUDITED,
)


@dataclass(frozen=True)
class CausalEvent:
    schema_version: str
    causal_id_sha256: str
    sequence_id: int
    stage: EventStage
    event_at_ns: int
    expires_at_ns: int
    payload_sha256: str
    parent_event_sha256: str
    actuation_permitted: bool = False

    def to_dict(self) -> dict[str, Any]:
        value = asdict(self)
        value["stage"] = self.stage.value if isinstance(self.stage, EventStage) else str(self.stage)
        return value

    @property
    def sha256(self) -> str:
        return sha256_canonical(self.to_dict())


@dataclass(frozen=True)
class AdmissionResult:
    accepted: bool
    safe_hold: bool
    reason: str
    causal_id_sha256: str
    stage: str
    event_sha256: str
    actuation_permitted: bool = False


def make_event(
    *,
    causal_id_sha256: str,
    sequence_id: int,
    stage: EventStage,
    event_at_ns: int,
    expires_at_ns: int,
    payload_sha256: str,
    parent_event_sha256: str = ZERO_HASH,
) -> CausalEvent:
    return CausalEvent(
        schema_version="go2_pixnav_causal_event_v1",
        causal_id_sha256=causal_id_sha256,
        sequence_id=sequence_id,
        stage=stage,
        event_at_ns=event_at_ns,
        expires_at_ns=expires_at_ns,
        payload_sha256=payload_sha256,
        parent_event_sha256=parent_event_sha256,
        actuation_permitted=False,
    )


class CausalAdmissionLedger:
    """Reject stale, duplicate, out-of-order or disconnected chain events."""

    def __init__(self) -> None:
        self._last_global_sequence = -1
        self._events: dict[str, list[CausalEvent]] = {}
        self._rejections: list[AdmissionResult] = []

    def append(self, event: CausalEvent, *, now_ns: int) -> AdmissionResult:
        reason = self._validate(event, now_ns)
        stage_label = event.stage.value if isinstance(event.stage, EventStage) else str(event.stage)
        if reason:
            result = AdmissionResult(
                accepted=False,
                safe_hold=True,
                reason=reason,
                causal_id_sha256=event.causal_id_sha256,
                stage=stage_label,
                event_sha256=event.sha256,
                actuation_permitted=False,
            )
            self._rejections.append(result)
            return result
        self._events.setdefault(event.causal_id_sha256, []).append(event)
        self._last_global_sequence = event.sequence_id
        return AdmissionResult(
            accepted=True,
            # This ledger validates file/event causality only. Even a complete
            # chain never transfers authority to a motor-facing component.
            safe_hold=True,
            reason="ACCEPTED_FILE_ONLY_EVENT",
            causal_id_sha256=event.causal_id_sha256,
            stage=stage_label,
            event_sha256=event.sha256,
            actuation_permitted=False,
        )

    def _validate(self, event: CausalEvent, now_ns: int) -> str:
        if event.schema_version != "go2_pixnav_causal_event_v1":
            return "UNSUPPORTED_EVENT_SCHEMA"
        if not isinstance(event.stage, EventStage):
            return "INVALID_EVENT_STAGE"
        if event.actuation_permitted:
            return "ACTUATION_INTERLOCK_VIOLATION"
        if not is_sha256(event.causal_id_sha256):
            return "INVALID_CAUSAL_ID"
        if not is_sha256(event.payload_sha256):
            return "INVALID_PAYLOAD_HASH"
        if not is_sha256(event.parent_event_sha256):
            return "INVALID_PARENT_HASH"
        if event.sequence_id <= self._last_global_sequence:
            return "DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE"
        if event.event_at_ns < 0 or event.expires_at_ns <= event.event_at_ns:
            return "INVALID_EVENT_TIME_RANGE"
        if now_ns < event.event_at_ns:
            return "EVENT_FROM_FUTURE"
        if now_ns > event.expires_at_ns:
            return "EVENT_STALE"

        chain = self._events.get(event.causal_id_sha256, [])
        expected_index = len(chain)
        if expected_index >= len(STAGE_ORDER):
            return "CAUSAL_CHAIN_ALREADY_COMPLETE"
        if event.stage != STAGE_ORDER[expected_index]:
            return f"STAGE_OUT_OF_ORDER_EXPECTED_{STAGE_ORDER[expected_index].value.upper()}"
        expected_parent = chain[-1].sha256 if chain else ZERO_HASH
        if event.parent_event_sha256 != expected_parent:
            return "PARENT_EVENT_HASH_MISMATCH"
        return ""

    def deadman_holds(self, *, now_ns: int) -> list[dict[str, Any]]:
        holds = []
        for causal_id, chain in sorted(self._events.items()):
            if len(chain) >= len(STAGE_ORDER):
                continue
            last = chain[-1]
            if now_ns <= last.expires_at_ns:
                continue
            expected = STAGE_ORDER[len(chain)]
            reason_by_stage = {
                EventStage.VLM_SUBMITTED: "VLM_SUBMIT_TIMEOUT",
                EventStage.VLM_COMPLETED: "VLM_RESPONSE_TIMEOUT",
                EventStage.PIXNAV_COMPLETED: "PIXNAV_TIMEOUT",
                EventStage.MACRO_AUDITED: "MACRO_AUDIT_TIMEOUT",
            }
            holds.append(
                {
                    "causal_id_sha256": causal_id,
                    "expected_stage": expected.value,
                    "reason": reason_by_stage.get(expected, "CAUSAL_STAGE_TIMEOUT"),
                    "safe_hold": True,
                    "target_dx_m": 0.0,
                    "target_dyaw_deg": 0.0,
                    "actuation_permitted": False,
                }
            )
        return holds

    def snapshot(self) -> dict[str, Any]:
        return {
            "last_global_sequence": self._last_global_sequence,
            "chains": {
                causal_id: [event.to_dict() for event in events]
                for causal_id, events in sorted(self._events.items())
            },
            "rejections": [asdict(result) for result in self._rejections],
            "actuation_permitted": False,
        }
