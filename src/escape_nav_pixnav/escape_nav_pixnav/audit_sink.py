"""Append-only, hash-chained JSONL sink for non-actuating proposals."""

from __future__ import annotations

import fcntl
import json
import os
import time
from pathlib import Path
from typing import Any

from .contracts import MacroActionProposal, canonical_json, sha256_canonical


ZERO_HASH = "0" * 64


def _parse_lines(text: str) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(text.splitlines(), start=1):
        if not line.strip():
            continue
        try:
            record = json.loads(line)
        except json.JSONDecodeError as error:
            raise ValueError(f"invalid JSONL at line {line_number}: {error}") from error
        if not isinstance(record, dict):
            raise ValueError(f"audit record at line {line_number} is not an object")
        records.append(record)
    return records


def _verify_records(records: list[dict[str, Any]]) -> dict[str, Any]:
    previous_hash = ZERO_HASH
    previous_sequence = -1
    for index, record in enumerate(records):
        actual_hash = str(record.get("record_sha256", ""))
        body = dict(record)
        body.pop("record_sha256", None)
        expected_hash = sha256_canonical(body)
        if actual_hash != expected_hash:
            raise ValueError(f"record hash mismatch at index {index}")
        if body.get("previous_record_sha256") != previous_hash:
            raise ValueError(f"chain hash mismatch at index {index}")
        proposal = body.get("proposal")
        if not isinstance(proposal, dict):
            raise ValueError(f"missing proposal at index {index}")
        if proposal.get("actuation_permitted") is not False:
            raise ValueError(f"actuation interlock violation at index {index}")
        sequence_id = int(proposal["sequence_id"])
        if sequence_id <= previous_sequence:
            raise ValueError(f"sequence is not strictly increasing at index {index}")
        previous_sequence = sequence_id
        previous_hash = actual_hash
    return {
        "valid": True,
        "record_count": len(records),
        "last_sequence_id": previous_sequence if records else None,
        "last_record_sha256": previous_hash,
        "actuation_permitted_count": 0,
    }


def verify_audit_chain(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(path)
    return _verify_records(_parse_lines(path.read_text(encoding="utf-8")))


class AuditJsonlSink:
    """Persist proposals and reject any record that could claim actuation."""

    def __init__(self, path: Path, *, fsync: bool = True) -> None:
        self.path = path.expanduser().resolve()
        self.fsync = fsync
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def append(self, proposal: MacroActionProposal) -> str:
        if proposal.actuation_permitted:
            raise ValueError("file-only sink refuses actuation_permitted=true")
        descriptor = os.open(
            str(self.path),
            os.O_APPEND | os.O_CREAT | os.O_RDWR | os.O_CLOEXEC,
            0o600,
        )
        try:
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "r+", encoding="utf-8") as stream:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                stream.seek(0)
                existing = _parse_lines(stream.read())
                verified = _verify_records(existing)
                last_sequence = verified["last_sequence_id"]
                if last_sequence is not None and proposal.sequence_id <= last_sequence:
                    raise ValueError("proposal sequence_id must be strictly increasing")
                body = {
                    "schema_version": "go2_pixnav_macro_audit_v1",
                    "written_at_ns": time.time_ns(),
                    "previous_record_sha256": verified["last_record_sha256"],
                    "proposal": proposal.to_dict(),
                }
                record_hash = sha256_canonical(body)
                record = {**body, "record_sha256": record_hash}
                stream.seek(0, os.SEEK_END)
                stream.write(canonical_json(record) + "\n")
                stream.flush()
                if self.fsync:
                    os.fsync(stream.fileno())
                fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
                return record_hash
        except Exception:
            # fdopen owns and closes descriptor once entered.
            raise
