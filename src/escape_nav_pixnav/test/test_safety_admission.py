from dataclasses import replace

from escape_nav_pixnav.safety_admission import (
    SafetyAdmissionConfig,
    evaluate_safety_admission,
)


NOW = 2_000_000_000


def proposal(kind="translate", accepted=True):
    return {
        "schema_version": "go2_pixnav_macro_proposal_v1",
        "accepted": accepted,
        "proposal_kind": kind,
        "target_dx_m": 0.25 if kind == "translate" else 0.0,
        "target_dyaw_deg": 30.0 if kind == "rotate" else 0.0,
        "actuation_permitted": False,
    }


def snapshot():
    return {
        "schema_version": "go2_l2_odom_safety_snapshot_v1",
        "status": "PASS_LIVE_L2_ODOM_SNAPSHOT",
        "cloud_received_monotonic_ns": NOW - 20_000_000,
        "odom_received_monotonic_ns": NOW - 10_000_000,
        "cloud_odom_stamp_delta_s": 0.01,
        "valid_cloud_points": 1000,
        "max_odom_step_m": 0.01,
        "max_odom_yaw_step_deg": 1.0,
        "front_clearance_m": 1.2,
        "rotation_clearance_m": 0.8,
    }


def test_all_checks_can_admit_candidate_but_never_actuate():
    result = evaluate_safety_admission(
        proposal(),
        snapshot(),
        evaluated_at_ns=NOW,
        decision_observed_at_ns=NOW - 100_000_000,
        decision_inferred_at_ns=NOW - 20_000_000,
        operator_enabled=True,
        estop_clear=True,
        global_localization_available=False,
    )
    assert result["admitted_to_gateway"] is True
    assert result["actuation_permitted"] is False
    assert result["reasons"] == ["P7_SAFETY_CHECKS_PASS"]


def test_operator_and_estop_unknown_fail_closed():
    result = evaluate_safety_admission(
        proposal(),
        snapshot(),
        evaluated_at_ns=NOW,
        decision_observed_at_ns=NOW - 100_000_000,
        decision_inferred_at_ns=NOW - 20_000_000,
        operator_enabled=False,
        estop_clear=False,
        global_localization_available=False,
    )
    assert result["admitted_to_gateway"] is False
    assert "OPERATOR_ENABLE_NOT_ASSERTED" in result["reasons"]
    assert "ESTOP_STATE_NOT_CLEAR" in result["reasons"]


def test_stale_or_blocked_front_sensor_fails_closed():
    value = snapshot()
    value["cloud_received_monotonic_ns"] = NOW - 600_000_000
    value["front_clearance_m"] = 0.2
    result = evaluate_safety_admission(
        proposal(),
        value,
        evaluated_at_ns=NOW,
        decision_observed_at_ns=NOW - 100_000_000,
        decision_inferred_at_ns=NOW - 20_000_000,
        operator_enabled=True,
        estop_clear=True,
        global_localization_available=False,
    )
    assert result["admitted_to_gateway"] is False
    assert "CLOUD_STALE" in result["reasons"]
    assert "FRONT_CLEARANCE_BLOCKED" in result["reasons"]


def test_rotation_uses_rotation_clearance_and_optional_global_pose_gate():
    value = snapshot()
    value["rotation_clearance_m"] = 0.3
    config = replace(SafetyAdmissionConfig(), require_global_localization=True)
    result = evaluate_safety_admission(
        proposal("rotate"),
        value,
        evaluated_at_ns=NOW,
        decision_observed_at_ns=NOW - 100_000_000,
        decision_inferred_at_ns=NOW - 20_000_000,
        operator_enabled=True,
        estop_clear=True,
        global_localization_available=False,
        config=config,
    )
    assert result["admitted_to_gateway"] is False
    assert "ROTATION_CLEARANCE_BLOCKED" in result["reasons"]
    assert "GLOBAL_LOCALIZATION_UNAVAILABLE" in result["reasons"]


def test_nonmoving_upstream_proposal_never_enters_gateway():
    result = evaluate_safety_admission(
        proposal("reobserve", accepted=False),
        snapshot(),
        evaluated_at_ns=NOW,
        decision_observed_at_ns=None,
        decision_inferred_at_ns=None,
        operator_enabled=True,
        estop_clear=True,
        global_localization_available=True,
    )
    assert result["admitted_to_gateway"] is False
    assert "UPSTREAM_PROPOSAL_NOT_MOTION_CANDIDATE" in result["reasons"]


def test_decision_is_rechecked_for_staleness_at_p7_gate():
    result = evaluate_safety_admission(
        proposal(),
        snapshot(),
        evaluated_at_ns=NOW,
        decision_observed_at_ns=NOW - 1_100_000_000,
        decision_inferred_at_ns=NOW - 600_000_000,
        operator_enabled=True,
        estop_clear=True,
        global_localization_available=True,
    )
    assert result["admitted_to_gateway"] is False
    assert "SOURCE_FRAME_STALE_AT_SAFETY_GATE" in result["reasons"]
    assert "PIXNAV_DECISION_STALE_AT_SAFETY_GATE" in result["reasons"]


def test_tampered_upstream_authority_or_nonboolean_acceptance_is_rejected():
    value = proposal()
    value["accepted"] = "true"
    value["actuation_permitted"] = True
    result = evaluate_safety_admission(
        value,
        snapshot(),
        evaluated_at_ns=NOW,
        decision_observed_at_ns=NOW - 100_000_000,
        decision_inferred_at_ns=NOW - 20_000_000,
        operator_enabled=True,
        estop_clear=True,
        global_localization_available=True,
    )
    assert result["admitted_to_gateway"] is False
    assert "UPSTREAM_PROPOSAL_NOT_MOTION_CANDIDATE" in result["reasons"]
    assert "UPSTREAM_ACTUATION_INTERLOCK_VIOLATION" in result["reasons"]
