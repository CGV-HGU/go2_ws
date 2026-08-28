"""Fail-closed conversion from PixNav output to file-only macro proposals."""

from __future__ import annotations

import hashlib
from typing import Any, Mapping

from .contracts import (
    AdapterConfig,
    MacroActionProposal,
    PixNavAction,
    PixNavDecision,
    ProposalKind,
    TimeBasis,
    canonical_json,
)


class PixNavMacroAdapter:
    """Validate a decision and return a proposal that can never actuate by itself."""

    def __init__(self, config: AdapterConfig | None = None) -> None:
        self.config = config or AdapterConfig()

    def adapt(
        self,
        raw: Mapping[str, Any],
        *,
        evaluated_at_ns: int,
    ) -> MacroActionProposal:
        try:
            decision = PixNavDecision.from_mapping(raw)
        except (KeyError, TypeError, ValueError) as error:
            return self._invalid_hold(raw, str(error))

        if decision.checkpoint_sha256 != self.config.expected_checkpoint_sha256:
            return self._hold(decision, "CHECKPOINT_HASH_MISMATCH")
        if not decision.finite:
            return self._hold(decision, "MODEL_OUTPUT_NOT_FINITE")
        if evaluated_at_ns < 0:
            return self._hold(decision, "NEGATIVE_EVALUATION_TIMESTAMP")

        if decision.time_basis == TimeBasis.MONOTONIC_LIVE:
            if evaluated_at_ns < decision.inferred_at_ns:
                return self._hold(decision, "EVALUATION_PRECEDES_INFERENCE")
            frame_age_s = (evaluated_at_ns - decision.observed_at_ns) / 1_000_000_000.0
            decision_age_s = (evaluated_at_ns - decision.inferred_at_ns) / 1_000_000_000.0
            if frame_age_s > self.config.source_frame_ttl_s:
                return self._hold(decision, "SOURCE_FRAME_STALE")
            if decision_age_s > self.config.decision_ttl_s:
                return self._hold(decision, "PIXNAV_DECISION_STALE")
        elif evaluated_at_ns != 0:
            return self._hold(decision, "OFFLINE_REPLAY_REQUIRES_ZERO_EVALUATION_TIME")

        if decision.action in {PixNavAction.LOOK_UP, PixNavAction.LOOK_DOWN}:
            return self._proposal(
                decision,
                accepted=False,
                kind=ProposalKind.REOBSERVE,
                reason="FIXED_CAMERA_VERTICAL_ACTION_UNSUPPORTED",
                requires_reobservation=True,
            )
        if decision.action == PixNavAction.STOP:
            return self._proposal(
                decision,
                accepted=True,
                kind=ProposalKind.ZERO_HOLD,
                reason="PIXNAV_STOP",
            )
        if decision.selected_probability < self.config.selected_probability_min:
            return self._hold(decision, "SELECTED_PROBABILITY_BELOW_THRESHOLD")
        if decision.action == PixNavAction.FORWARD:
            return self._proposal(
                decision,
                accepted=True,
                kind=ProposalKind.TRANSLATE,
                reason="VALIDATED_FILE_ONLY_PROPOSAL",
                target_dx_m=self.config.translation_step_m,
                timeout_s=self.config.translation_timeout_s,
            )
        if decision.action in {PixNavAction.TURN_LEFT, PixNavAction.TURN_RIGHT}:
            direction = 1.0 if decision.action == PixNavAction.TURN_LEFT else -1.0
            return self._proposal(
                decision,
                accepted=True,
                kind=ProposalKind.ROTATE,
                reason="VALIDATED_FILE_ONLY_PROPOSAL",
                target_dyaw_deg=direction * self.config.turn_step_deg,
                timeout_s=self.config.rotation_timeout_s,
            )
        return self._hold(decision, "UNHANDLED_ACTION")

    def _invalid_hold(
        self,
        raw: Mapping[str, Any],
        reason: str,
    ) -> MacroActionProposal:
        try:
            raw_digest = hashlib.sha256(canonical_json(dict(raw)).encode("utf-8")).hexdigest()
        except (TypeError, ValueError):
            raw_digest = hashlib.sha256(repr(raw).encode("utf-8", errors="replace")).hexdigest()
        sequence_id = raw.get("sequence_id", -1)
        try:
            sequence_id = int(sequence_id)
        except (TypeError, ValueError):
            sequence_id = -1
        return MacroActionProposal(
            schema_version="go2_pixnav_macro_proposal_v1",
            event_id=f"invalid:{raw_digest[:24]}",
            sequence_id=sequence_id,
            source_frame_sha256=str(raw.get("source_frame_sha256", "")),
            checkpoint_sha256=str(raw.get("checkpoint_sha256", "")),
            adapter_config_sha256=self.config.sha256,
            time_basis=TimeBasis.OFFLINE_REPLAY,
            pixnav_action_id=-1,
            pixnav_action="invalid",
            selected_probability=0.0,
            accepted=False,
            proposal_kind=ProposalKind.ZERO_HOLD,
            reason=f"INVALID_DECISION:{reason}",
            target_dx_m=0.0,
            target_dyaw_deg=0.0,
            max_linear_speed_mps=0.0,
            max_angular_speed_rps=0.0,
            max_linear_accel_mps2=0.0,
            max_angular_accel_rps2=0.0,
            timeout_s=0.0,
            position_tolerance_m=self.config.position_tolerance_m,
            yaw_tolerance_deg=self.config.yaw_tolerance_deg,
            requires_reobservation=False,
            actuation_permitted=False,
        )

    def _hold(self, decision: PixNavDecision, reason: str) -> MacroActionProposal:
        return self._proposal(
            decision,
            accepted=False,
            kind=ProposalKind.ZERO_HOLD,
            reason=reason,
        )

    def _proposal(
        self,
        decision: PixNavDecision,
        *,
        accepted: bool,
        kind: ProposalKind,
        reason: str,
        target_dx_m: float = 0.0,
        target_dyaw_deg: float = 0.0,
        timeout_s: float = 0.0,
        requires_reobservation: bool = False,
    ) -> MacroActionProposal:
        moving = kind in {ProposalKind.TRANSLATE, ProposalKind.ROTATE}
        return MacroActionProposal(
            schema_version="go2_pixnav_macro_proposal_v1",
            event_id=decision.event_id,
            sequence_id=decision.sequence_id,
            source_frame_sha256=decision.source_frame_sha256,
            checkpoint_sha256=decision.checkpoint_sha256,
            adapter_config_sha256=self.config.sha256,
            time_basis=decision.time_basis,
            pixnav_action_id=decision.action_id,
            pixnav_action=decision.action.value,
            selected_probability=round(decision.selected_probability, 9),
            accepted=accepted,
            proposal_kind=kind,
            reason=reason,
            target_dx_m=target_dx_m,
            target_dyaw_deg=target_dyaw_deg,
            max_linear_speed_mps=self.config.max_linear_speed_mps if moving else 0.0,
            max_angular_speed_rps=self.config.max_angular_speed_rps if moving else 0.0,
            max_linear_accel_mps2=self.config.max_linear_accel_mps2 if moving else 0.0,
            max_angular_accel_rps2=self.config.max_angular_accel_rps2 if moving else 0.0,
            timeout_s=timeout_s,
            position_tolerance_m=self.config.position_tolerance_m,
            yaw_tolerance_deg=self.config.yaw_tolerance_deg,
            requires_reobservation=requires_reobservation,
            actuation_permitted=False,
        )
