from copy import deepcopy

from escape_nav_pixnav import GatewayConfig, NoActuationGatewayCore, PixNavMacroAdapter
from escape_nav_pixnav.safety_admission import evaluate_safety_admission

from test_adapter import decision


NOW = 2_000_000_000
AUTHORITY = "escape_nav_pixnav_gateway"


def sensor_snapshot():
    return {
        "schema_version": "go2_l2_odom_safety_snapshot_v1",
        "status": "PASS_LIVE_L2_ODOM_SNAPSHOT",
        "cloud_received_monotonic_ns": NOW - 60_000_000,
        "odom_received_monotonic_ns": NOW - 50_000_000,
        "cloud_odom_stamp_delta_s": 0.01,
        "valid_cloud_points": 1000,
        "max_odom_step_m": 0.01,
        "max_odom_yaw_step_deg": 1.0,
        "front_clearance_m": 1.2,
        "rotation_clearance_m": 0.8,
    }


def candidate(sequence_id=0, action_id=1):
    proposal = PixNavMacroAdapter().adapt(
        decision(
            action_id,
            sequence_id=sequence_id,
            observed_at_ns=NOW - 100_000_000,
            inferred_at_ns=NOW - 40_000_000,
        ),
        evaluated_at_ns=NOW - 30_000_000,
    ).to_dict()
    admission = evaluate_safety_admission(
        proposal,
        sensor_snapshot(),
        evaluated_at_ns=NOW - 20_000_000,
        decision_observed_at_ns=NOW - 100_000_000,
        decision_inferred_at_ns=NOW - 40_000_000,
        operator_enabled=True,
        estop_clear=True,
        global_localization_available=False,
    )
    return proposal, admission


def armed_core():
    core = NoActuationGatewayCore()
    reset = core.reset_interlocks(
        manual_reset=True,
        operator_enabled=True,
        estop_clear=True,
    )
    assert reset["reset_accepted"] is True
    return core


def evaluate(core, proposal, admission, **overrides):
    values = {
        "authority_id": AUTHORITY,
        "received_at_ns": NOW - 10_000_000,
        "evaluated_at_ns": NOW,
        "operator_enabled": True,
        "estop_clear": True,
    }
    values.update(overrides)
    return core.evaluate(proposal, admission, **values)


def test_valid_candidate_passes_contract_but_never_dispatches():
    proposal, admission = candidate()
    result = evaluate(armed_core(), proposal, admission)

    assert result["gateway_candidate_valid"] is True
    assert result["intent"]["kind"] == "translate"
    assert result["physical_dispatch_permitted"] is False
    assert result["actuation_calls"] == 0


def test_startup_requires_explicit_manual_reset():
    proposal, admission = candidate()
    result = evaluate(NoActuationGatewayCore(), proposal, admission)

    assert result["gateway_candidate_valid"] is False
    assert "ESTOP_LATCHED_MANUAL_RESET_REQUIRED" in result["reasons"]
    assert "GATEWAY_NOT_ARMED" in result["reasons"]


def test_estop_latches_and_does_not_auto_clear():
    core = armed_core()
    proposal, admission = candidate()
    blocked = evaluate(core, proposal, admission, estop_clear=False)
    assert "ESTOP_ASSERTED" in blocked["reasons"]

    still_blocked = evaluate(core, proposal, admission, estop_clear=True)
    assert "ESTOP_LATCHED_MANUAL_RESET_REQUIRED" in still_blocked["reasons"]
    reset = core.reset_interlocks(
        manual_reset=True,
        operator_enabled=True,
        estop_clear=True,
    )
    assert reset["reset_accepted"] is True


def test_reset_needs_manual_operator_and_clear_estop():
    core = NoActuationGatewayCore()
    result = core.reset_interlocks(
        manual_reset=False,
        operator_enabled=False,
        estop_clear=False,
    )
    assert result["reset_accepted"] is False
    assert result["armed"] is False


def test_wrong_authority_fails_closed():
    proposal, admission = candidate()
    result = evaluate(
        armed_core(), proposal, admission, authority_id="second_controller"
    )
    assert result["gateway_candidate_valid"] is False
    assert "AUTHORITY_ID_MISMATCH" in result["reasons"]


def test_duplicate_sequence_is_rejected():
    core = armed_core()
    proposal, admission = candidate()
    assert evaluate(core, proposal, admission)["gateway_candidate_valid"] is True
    duplicate = evaluate(core, proposal, admission)
    assert "DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE" in duplicate["reasons"]


def test_stale_p7_admission_is_rejected():
    proposal, admission = candidate()
    admission["evaluated_at_ns"] = NOW - 300_000_000
    result = evaluate(armed_core(), proposal, admission)
    assert "P7_ADMISSION_STALE" in result["reasons"]


def test_p7_proposal_hash_mismatch_is_rejected():
    proposal, admission = candidate()
    changed = deepcopy(proposal)
    changed["target_dx_m"] = 0.20
    result = evaluate(armed_core(), changed, admission)
    assert "P7_PROPOSAL_HASH_MISMATCH" in result["reasons"]


def test_out_of_bounds_motion_is_rejected_instead_of_clamped():
    proposal, admission = candidate()
    proposal["max_linear_speed_mps"] = 0.20
    result = evaluate(armed_core(), proposal, admission)
    assert "LINEAR_SPEED_OUT_OF_BOUNDS" in result["reasons"]
    assert result["intent"]["kind"] == "zero_hold"


def test_nonmotion_reobserve_never_enters_gateway():
    proposal, admission = candidate(action_id=5)
    result = evaluate(armed_core(), proposal, admission)
    assert result["gateway_candidate_valid"] is False
    assert "NON_MOTION_PROPOSAL" in result["reasons"]
    assert "P7_ADMISSION_REJECTED" in result["reasons"]


def test_deadman_requires_zero_after_timeout():
    core = armed_core()
    proposal, admission = candidate()
    assert evaluate(core, proposal, admission)["gateway_candidate_valid"] is True

    active = core.deadman(evaluated_at_ns=NOW + 400_000_000)
    expired = core.deadman(evaluated_at_ns=NOW + 600_000_000)
    assert active["zero_required"] is False
    assert expired["zero_required"] is True
    assert expired["reasons"] == ["DEADMAN_TIMEOUT_ZERO_REQUIRED"]
    assert expired["physical_dispatch_permitted"] is False


def test_shutdown_is_irreversible_and_zero_only():
    core = armed_core()
    stopped = core.shutdown()
    assert stopped["intent"]["kind"] == "zero_hold"
    assert stopped["physical_dispatch_permitted"] is False
    assert core.reset_interlocks(
        manual_reset=True,
        operator_enabled=True,
        estop_clear=True,
    )["reset_accepted"] is False


def test_gateway_config_rejects_invalid_limits():
    try:
        GatewayConfig(deadman_timeout_s=float("nan"))
    except ValueError as error:
        assert "finite and positive" in str(error)
    else:
        raise AssertionError("invalid gateway config was accepted")
