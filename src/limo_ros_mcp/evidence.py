"""Canonical serialization and integrity helpers for LIMO evidence."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping, Sequence
from typing import Any


def _validate_json_value(value: Any, path: str = "$") -> None:
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"non-finite number at {path}")
        return
    if isinstance(value, Mapping):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f"non-string object key at {path}")
            _validate_json_value(item, f"{path}.{key}")
        return
    if isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for index, item in enumerate(value):
            _validate_json_value(item, f"{path}[{index}]")
        return
    raise ValueError(f"unsupported JSON value {type(value).__name__} at {path}")


def canonical_json(value: Any) -> bytes:
    """Serialize JSON deterministically and reject ambiguous non-finite values."""

    _validate_json_value(value)
    return json.dumps(
        value,
        ensure_ascii=False,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_document(value: Any) -> str:
    """Return a tagged SHA-256 digest of a canonical JSON document."""

    return f"sha256:{hashlib.sha256(canonical_json(value)).hexdigest()}"


def seal_snapshot(snapshot: Mapping[str, Any]) -> dict[str, Any]:
    """Return a copy with a hash that excludes the hash field itself."""

    payload = dict(snapshot)
    payload.pop("snapshot_hash", None)
    payload["snapshot_hash"] = sha256_document(payload)
    return payload


def verify_snapshot(snapshot: Mapping[str, Any]) -> bool:
    """Verify a snapshot produced by :func:`seal_snapshot`."""

    claimed = snapshot.get("snapshot_hash")
    if not isinstance(claimed, str):
        return False
    payload = dict(snapshot)
    payload.pop("snapshot_hash", None)
    return claimed == sha256_document(payload)
