"""Speaker tone action-intent validation and construction."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def _block(error_code: str, message: str) -> dict[str, Any]:
    return {
        "ok": False,
        "decision": "BLOCK",
        "schema_version": "limo.navigation.v2",
        "error_code": error_code,
        "message": message,
        "command_dispatched": False,
        "usable_for_real_execution": False,
    }


def validate_tone(
    *,
    frequency_hz: int,
    duration_sec: float,
    volume_percent: int,
    body_snapshot_hash: str,
) -> dict[str, Any]:
    if not str(body_snapshot_hash).strip():
        return _block(
            "BODY_SNAPSHOT_REQUIRED",
            "Speaker requests require an immutable LIMO body snapshot.",
        )
    if isinstance(frequency_hz, bool) or frequency_hz not in {440, 660, 880}:
        return _block(
            "LIMO_TONE_FREQUENCY_INVALID",
            "frequency_hz must be one of 440, 660, or 880.",
        )
    if (
        isinstance(duration_sec, bool)
        or not isinstance(duration_sec, (int, float))
        or not 0.2 <= float(duration_sec) <= 1.0
    ):
        return _block(
            "LIMO_TONE_DURATION_INVALID",
            "duration_sec must be within [0.2, 1.0].",
        )
    if (
        isinstance(volume_percent, bool)
        or not isinstance(volume_percent, int)
        or not 5 <= volume_percent <= 25
    ):
        return _block(
            "LIMO_TONE_VOLUME_INVALID",
            "volume_percent must be an integer within [5, 25].",
        )
    return {
        "ok": True,
        "decision": "ALLOW",
        "normalized_tone": {
            "frequency_hz": frequency_hz,
            "duration_sec": float(duration_sec),
            "volume_percent": volume_percent,
        },
    }


def tone_arguments(tone: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": "limo.tone.v1",
        "frequency_hz": tone["frequency_hz"],
        "duration_sec": tone["duration_sec"],
        "volume_percent": tone["volume_percent"],
        "expected_effect": {
            "kind": "speaker_tone",
            "playback_required": True,
            "mixer_restore_required": True,
            "microphone_loopback_required": True,
        },
    }


def tone_display(tone: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": "rosclaw.action-display.v2",
        "title": "Play a short LIMO speaker tone",
        "summary": (
            "Play one bounded synthesized tone, verify it through the onboard microphone, "
            "and restore the mixer state."
        ),
        "body": {"robot_id": "limo", "tone": dict(tone)},
        "risk_tier": "MEDIUM",
        "physical_effects": ["The robot speaker will emit a short audible tone."],
        "constraints": ["Volume and duration remain within the bounded tone policy."],
        "verification": ["Verify microphone loopback and mixer restoration."],
        "abort": ["Playback stops automatically at the bounded duration."],
    }
