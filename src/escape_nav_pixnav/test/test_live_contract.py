from escape_nav_pixnav.contracts import PIXNAV_CHECKPOINT_A_SHA256
from escape_nav_pixnav.live_contract import live_decision_from_report, make_upstream_hold


def test_upstream_hold_never_allows_actuation():
    proposal = make_upstream_hold(
        event_id="p6.fixture.blocked",
        sequence_id=0,
        source_frame_sha256="1" * 64,
        reason="VLM_SCHEMA_REJECTED",
    )
    assert proposal.proposal_kind.value == "zero_hold"
    assert proposal.target_dx_m == 0.0
    assert proposal.target_dyaw_deg == 0.0
    assert proposal.actuation_permitted is False


def test_live_decision_uses_prediction_frame_timestamp_and_hash():
    report = {
        "checkpoint_sha256_actual": PIXNAV_CHECKPOINT_A_SHA256,
        "predictions": [
            {
                "frame_index": 1,
                "action_id": 1,
                "action": "forward",
                "action_probabilities": {
                    "stop": 0.01,
                    "forward": 0.90,
                    "turn_left": 0.03,
                    "turn_right": 0.02,
                    "look_up": 0.02,
                    "look_down": 0.02,
                },
                "finite": True,
            }
        ],
    }
    frames = [
        {"capture_monotonic_ns": 10, "sha256_file": "1" * 64},
        {"capture_monotonic_ns": 20, "sha256_file": "2" * 64},
    ]
    decision = live_decision_from_report(
        report,
        frames,
        run_id="fixture",
        sequence_id=7,
        inferred_at_ns=30,
    )
    assert decision["observed_at_ns"] == 20
    assert decision["inferred_at_ns"] == 30
    assert decision["source_frame_sha256"] == "2" * 64
    assert decision["sequence_id"] == 7
