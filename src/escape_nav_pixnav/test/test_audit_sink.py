import json
import os
from dataclasses import replace

import pytest

from escape_nav_pixnav import AuditJsonlSink, PixNavMacroAdapter, verify_audit_chain

from test_adapter import decision


def proposal(sequence_id):
    return PixNavMacroAdapter().adapt(
        decision(sequence_id=sequence_id),
        evaluated_at_ns=1_200_000_000,
    )


def test_sink_writes_hash_chained_no_actuation_records(tmp_path):
    output = tmp_path / "actions.jsonl"
    sink = AuditJsonlSink(output, fsync=False)

    first_hash = sink.append(proposal(0))
    second_hash = sink.append(proposal(1))
    result = verify_audit_chain(output)

    assert result["valid"] is True
    assert result["record_count"] == 2
    assert result["last_sequence_id"] == 1
    assert result["last_record_sha256"] == second_hash
    assert first_hash != second_hash
    assert result["actuation_permitted_count"] == 0
    assert output.stat().st_mode & 0o777 == 0o600


def test_sink_rejects_duplicate_or_reordered_sequence(tmp_path):
    sink = AuditJsonlSink(tmp_path / "actions.jsonl", fsync=False)
    sink.append(proposal(1))

    with pytest.raises(ValueError, match="strictly increasing"):
        sink.append(proposal(1))
    with pytest.raises(ValueError, match="strictly increasing"):
        sink.append(proposal(0))


def test_sink_refuses_actuation_permitted_true(tmp_path):
    unsafe = replace(proposal(0), actuation_permitted=True)

    with pytest.raises(ValueError, match="refuses"):
        AuditJsonlSink(tmp_path / "actions.jsonl").append(unsafe)


def test_sink_tightens_existing_file_permissions(tmp_path):
    output = tmp_path / "actions.jsonl"
    output.touch(mode=0o666)
    os.chmod(str(output), 0o666)

    AuditJsonlSink(output, fsync=False).append(proposal(0))

    assert output.stat().st_mode & 0o777 == 0o600


def test_verifier_detects_tampering(tmp_path):
    output = tmp_path / "actions.jsonl"
    AuditJsonlSink(output, fsync=False).append(proposal(0))
    record = json.loads(output.read_text(encoding="utf-8"))
    record["proposal"]["target_dx_m"] = 99.0
    output.write_text(json.dumps(record) + "\n", encoding="utf-8")

    with pytest.raises(ValueError, match="hash mismatch"):
        verify_audit_chain(output)
