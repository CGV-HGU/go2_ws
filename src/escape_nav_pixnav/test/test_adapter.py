import math

import pytest

from escape_nav_pixnav import (
    ACTION_NAMES,
    AdapterConfig,
    PixNavMacroAdapter,
    ProposalKind,
)
from escape_nav_pixnav.contracts import PIXNAV_CHECKPOINT_A_SHA256


FRAME_HASH = "1" * 64


def decision(
    action_id=1,
    probability=0.90,
    *,
    sequence_id=0,
    observed_at_ns=1_000_000_000,
    inferred_at_ns=1_100_000_000,
    time_basis="monotonic_live",
):
    remainder = (1.0 - probability) / 5.0
    probabilities = {name: remainder for name in ACTION_NAMES}
    probabilities[ACTION_NAMES[action_id]] = probability
    return {
        "schema_version": "go2_pixnav_decision_v1",
        "event_id": f"event:{sequence_id}",
        "sequence_id": sequence_id,
        "source_frame_sha256": FRAME_HASH,
        "checkpoint_sha256": PIXNAV_CHECKPOINT_A_SHA256,
        "observed_at_ns": observed_at_ns,
        "inferred_at_ns": inferred_at_ns,
        "time_basis": time_basis,
        "action_id": action_id,
        "action": ACTION_NAMES[action_id],
        "action_probabilities": probabilities,
        "finite": True,
    }


def test_forward_maps_to_bounded_translation_proposal_only():
    proposal = PixNavMacroAdapter().adapt(
        decision(1),
        evaluated_at_ns=1_200_000_000,
    )

    assert proposal.accepted is True
    assert proposal.proposal_kind == ProposalKind.TRANSLATE
    assert proposal.target_dx_m == 0.25
    assert proposal.target_dyaw_deg == 0.0
    assert proposal.max_linear_speed_mps == 0.10
    assert proposal.actuation_permitted is False


@pytest.mark.parametrize(
    ("action_id", "expected_yaw"),
    [(2, 30.0), (3, -30.0)],
)
def test_turns_map_to_signed_bounded_rotation(action_id, expected_yaw):
    proposal = PixNavMacroAdapter().adapt(
        decision(action_id),
        evaluated_at_ns=1_200_000_000,
    )

    assert proposal.accepted is True
    assert proposal.proposal_kind == ProposalKind.ROTATE
    assert proposal.target_dx_m == 0.0
    assert proposal.target_dyaw_deg == expected_yaw
    assert proposal.actuation_permitted is False


def test_stop_is_always_zero_hold():
    proposal = PixNavMacroAdapter().adapt(
        decision(0, probability=0.30),
        evaluated_at_ns=1_200_000_000,
    )

    assert proposal.accepted is True
    assert proposal.proposal_kind == ProposalKind.ZERO_HOLD
    assert proposal.target_dx_m == 0.0
    assert proposal.target_dyaw_deg == 0.0
    assert proposal.reason == "PIXNAV_STOP"


@pytest.mark.parametrize("action_id", [4, 5])
def test_fixed_camera_vertical_actions_require_reobservation(action_id):
    proposal = PixNavMacroAdapter().adapt(
        decision(action_id),
        evaluated_at_ns=1_200_000_000,
    )

    assert proposal.accepted is False
    assert proposal.proposal_kind == ProposalKind.REOBSERVE
    assert proposal.requires_reobservation is True
    assert proposal.max_linear_speed_mps == 0.0
    assert proposal.max_angular_speed_rps == 0.0


@pytest.mark.parametrize(
    ("mutate", "now_ns", "reason"),
    [
        (lambda value: value.update(observed_at_ns=0), 1_200_000_000, "SOURCE_FRAME_STALE"),
        (
            lambda value: value.update(
                observed_at_ns=800_000_000,
                inferred_at_ns=900_000_000,
            ),
            1_500_000_000,
            "PIXNAV_DECISION_STALE",
        ),
        (
            lambda value: value.update(checkpoint_sha256="2" * 64),
            1_200_000_000,
            "CHECKPOINT_HASH_MISMATCH",
        ),
        (
            lambda value: value.update(finite=False),
            1_200_000_000,
            "MODEL_OUTPUT_NOT_FINITE",
        ),
    ],
)
def test_live_safety_failures_become_zero_hold(mutate, now_ns, reason):
    raw = decision(1)
    mutate(raw)

    proposal = PixNavMacroAdapter().adapt(raw, evaluated_at_ns=now_ns)

    assert proposal.accepted is False
    assert proposal.proposal_kind == ProposalKind.ZERO_HOLD
    assert proposal.reason == reason
    assert proposal.actuation_permitted is False


def test_low_probability_motion_becomes_zero_hold():
    raw = decision(1, probability=0.40)

    proposal = PixNavMacroAdapter().adapt(raw, evaluated_at_ns=1_200_000_000)

    assert proposal.accepted is False
    assert proposal.reason == "SELECTED_PROBABILITY_BELOW_THRESHOLD"


@pytest.mark.parametrize(
    "mutate",
    [
        lambda value: value.update(action="turn_left"),
        lambda value: value.update(action_id=99),
        lambda value: value["action_probabilities"].update(forward=float("nan")),
        lambda value: value.update(source_frame_sha256="bad"),
        lambda value: value.update(schema_version="unknown"),
        lambda value: value.update(finite="false"),
        lambda value: value.update(sequence_id="1"),
    ],
)
def test_malformed_decisions_fail_closed(mutate):
    raw = decision(1)
    mutate(raw)

    proposal = PixNavMacroAdapter().adapt(raw, evaluated_at_ns=1_200_000_000)

    assert proposal.accepted is False
    assert proposal.proposal_kind == ProposalKind.ZERO_HOLD
    assert proposal.reason.startswith("INVALID_DECISION:")
    assert proposal.actuation_permitted is False


def test_offline_replay_requires_explicit_zero_time_and_never_actuates():
    raw = decision(
        1,
        observed_at_ns=0,
        inferred_at_ns=0,
        time_basis="offline_replay",
    )
    adapter = PixNavMacroAdapter(AdapterConfig(selected_probability_min=0.5))

    accepted = adapter.adapt(raw, evaluated_at_ns=0)
    blocked = adapter.adapt(raw, evaluated_at_ns=1)

    assert accepted.accepted is True
    assert accepted.actuation_permitted is False
    assert blocked.accepted is False
    assert blocked.reason == "OFFLINE_REPLAY_REQUIRES_ZERO_EVALUATION_TIME"


def test_configuration_rejects_nonfinite_or_nonpositive_limits():
    with pytest.raises(ValueError):
        AdapterConfig(max_linear_speed_mps=0.0)
    with pytest.raises(ValueError):
        AdapterConfig(decision_ttl_s=math.nan)
