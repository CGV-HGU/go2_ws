from dataclasses import replace

from escape_nav_pixnav.event_ledger import (
    STAGE_ORDER,
    CausalAdmissionLedger,
    EventStage,
    make_event,
)


CAUSAL_ID = "a" * 64
PAYLOAD_HASH = "b" * 64


def event(stage, sequence, *, parent="0" * 64, at=1_000, expires=2_000):
    return make_event(
        causal_id_sha256=CAUSAL_ID,
        sequence_id=sequence,
        stage=stage,
        event_at_ns=at,
        expires_at_ns=expires,
        payload_sha256=PAYLOAD_HASH,
        parent_event_sha256=parent,
    )


def append_next(ledger, stage, sequence, parent):
    value = event(stage, sequence, parent=parent)
    result = ledger.append(value, now_ns=1_500)
    assert result.accepted is True
    assert result.safe_hold is True
    assert result.actuation_permitted is False
    return value


def test_complete_ordered_chain_is_accepted_but_never_authorizes_actuation():
    ledger = CausalAdmissionLedger()
    parent = "0" * 64
    for sequence, stage in enumerate(STAGE_ORDER):
        value = append_next(ledger, stage, sequence, parent)
        parent = value.sha256

    snapshot = ledger.snapshot()
    assert len(snapshot["chains"][CAUSAL_ID]) == 5
    assert snapshot["actuation_permitted"] is False
    assert ledger.deadman_holds(now_ns=3_000) == []


def test_duplicate_and_out_of_order_sequences_fail_closed():
    ledger = CausalAdmissionLedger()
    first = append_next(ledger, EventStage.FRAME_CAPTURED, 1, "0" * 64)

    duplicate = event(EventStage.VLM_SUBMITTED, 1, parent=first.sha256)
    earlier = event(EventStage.VLM_SUBMITTED, 0, parent=first.sha256)

    for value in (duplicate, earlier):
        result = ledger.append(value, now_ns=1_500)
        assert result.accepted is False
        assert result.safe_hold is True
        assert result.reason == "DUPLICATE_OR_OUT_OF_ORDER_SEQUENCE"


def test_stage_and_parent_hash_order_are_enforced():
    ledger = CausalAdmissionLedger()
    first = append_next(ledger, EventStage.FRAME_CAPTURED, 0, "0" * 64)

    wrong_stage = event(EventStage.PIXNAV_COMPLETED, 1, parent=first.sha256)
    wrong_parent = event(EventStage.VLM_SUBMITTED, 2, parent="c" * 64)

    assert "STAGE_OUT_OF_ORDER" in ledger.append(wrong_stage, now_ns=1_500).reason
    assert ledger.append(wrong_parent, now_ns=1_500).reason == "PARENT_EVENT_HASH_MISMATCH"


def test_stale_future_and_invalid_time_events_fail_closed():
    ledger = CausalAdmissionLedger()
    stale = event(EventStage.FRAME_CAPTURED, 0, at=1_000, expires=1_100)
    future = event(EventStage.FRAME_CAPTURED, 1, at=2_000, expires=3_000)
    invalid = event(EventStage.FRAME_CAPTURED, 2, at=2_000, expires=2_000)

    assert ledger.append(stale, now_ns=1_500).reason == "EVENT_STALE"
    assert ledger.append(future, now_ns=1_500).reason == "EVENT_FROM_FUTURE"
    assert ledger.append(invalid, now_ns=2_000).reason == "INVALID_EVENT_TIME_RANGE"


def test_actuation_flag_and_bad_hashes_fail_closed():
    ledger = CausalAdmissionLedger()
    unsafe = replace(event(EventStage.FRAME_CAPTURED, 0), actuation_permitted=True)
    bad_payload = replace(event(EventStage.FRAME_CAPTURED, 1), payload_sha256="bad")

    assert ledger.append(unsafe, now_ns=1_500).reason == "ACTUATION_INTERLOCK_VIOLATION"
    assert ledger.append(bad_payload, now_ns=1_500).reason == "INVALID_PAYLOAD_HASH"


def test_invalid_stage_type_fails_closed_without_throwing():
    ledger = CausalAdmissionLedger()
    invalid = replace(event(EventStage.FRAME_CAPTURED, 0), stage="frame_captured")

    result = ledger.append(invalid, now_ns=1_500)

    assert result.accepted is False
    assert result.safe_hold is True
    assert result.reason == "INVALID_EVENT_STAGE"


def test_missing_vlm_response_triggers_zero_deadman_hold():
    ledger = CausalAdmissionLedger()
    captured = append_next(ledger, EventStage.FRAME_CAPTURED, 0, "0" * 64)
    submitted = append_next(ledger, EventStage.VLM_SUBMITTED, 1, captured.sha256)

    holds = ledger.deadman_holds(now_ns=submitted.expires_at_ns + 1)

    assert holds == [
        {
            "causal_id_sha256": CAUSAL_ID,
            "expected_stage": "vlm_completed",
            "reason": "VLM_RESPONSE_TIMEOUT",
            "safe_hold": True,
            "target_dx_m": 0.0,
            "target_dyaw_deg": 0.0,
            "actuation_permitted": False,
        }
    ]


def test_missing_pixnav_or_macro_stage_has_specific_timeout_reason():
    for stop_after, expected in (
        (EventStage.VLM_COMPLETED, "PIXNAV_TIMEOUT"),
        (EventStage.PIXNAV_COMPLETED, "MACRO_AUDIT_TIMEOUT"),
    ):
        ledger = CausalAdmissionLedger()
        parent = "0" * 64
        last = None
        for sequence, stage in enumerate(STAGE_ORDER):
            last = append_next(ledger, stage, sequence, parent)
            parent = last.sha256
            if stage == stop_after:
                break
        assert last is not None
        holds = ledger.deadman_holds(now_ns=last.expires_at_ns + 1)
        assert holds[0]["reason"] == expected
        assert holds[0]["actuation_permitted"] is False
