"""Pure contracts used by the real-camera, no-actuation PixNav P6 runner."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

from .contracts import (
    AdapterConfig,
    MacroActionProposal,
    PIXNAV_CHECKPOINT_A_SHA256,
    ProposalKind,
    TimeBasis,
)


def make_upstream_hold(
    *,
    event_id: str,
    sequence_id: int,
    source_frame_sha256: str,
    reason: str,
) -> MacroActionProposal:
    """Create a zero-target file-only proposal when an upstream stage fails."""

    config = AdapterConfig()
    return MacroActionProposal(
        schema_version="go2_pixnav_macro_proposal_v1",
        event_id=event_id,
        sequence_id=sequence_id,
        source_frame_sha256=source_frame_sha256,
        checkpoint_sha256=PIXNAV_CHECKPOINT_A_SHA256,
        adapter_config_sha256=config.sha256,
        time_basis=TimeBasis.MONOTONIC_LIVE,
        pixnav_action_id=-1,
        pixnav_action="not_executed",
        selected_probability=0.0,
        accepted=False,
        proposal_kind=ProposalKind.ZERO_HOLD,
        reason=reason,
        target_dx_m=0.0,
        target_dyaw_deg=0.0,
        max_linear_speed_mps=0.0,
        max_angular_speed_rps=0.0,
        max_linear_accel_mps2=0.0,
        max_angular_accel_rps2=0.0,
        timeout_s=0.0,
        position_tolerance_m=config.position_tolerance_m,
        yaw_tolerance_deg=config.yaw_tolerance_deg,
        requires_reobservation=False,
        actuation_permitted=False,
    )


def live_decision_from_report(
    pixnav_report: Mapping[str, Any],
    frame_metadata: Sequence[Mapping[str, Any]],
    *,
    run_id: str,
    sequence_id: int,
    inferred_at_ns: int,
) -> dict[str, Any]:
    """Convert the final offline-format model result into a timed live decision."""

    predictions = pixnav_report.get("predictions")
    if not isinstance(predictions, list) or not predictions:
        raise ValueError("PixNav report contains no predictions")
    prediction = predictions[-1]
    if not isinstance(prediction, Mapping):
        raise ValueError("PixNav prediction is not an object")
    frame_index = int(prediction["frame_index"])
    if frame_index < 0 or frame_index >= len(frame_metadata):
        raise ValueError("PixNav prediction frame index is outside capture metadata")
    source = frame_metadata[frame_index]
    probabilities = prediction.get("action_probabilities")
    if not isinstance(probabilities, Mapping):
        raise ValueError("PixNav action probabilities are not an object")
    return {
        "schema_version": "go2_pixnav_decision_v1",
        "event_id": f"p6.{run_id}.{frame_index}",
        "sequence_id": sequence_id,
        "source_frame_sha256": source["sha256_file"],
        "checkpoint_sha256": pixnav_report["checkpoint_sha256_actual"],
        "observed_at_ns": int(source["capture_monotonic_ns"]),
        "inferred_at_ns": inferred_at_ns,
        "time_basis": "monotonic_live",
        "action_id": int(prediction["action_id"]),
        "action": str(prediction["action"]),
        "action_probabilities": dict(probabilities),
        "finite": bool(prediction["finite"]),
    }
