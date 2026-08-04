"""Bounded text-to-speech action validation and construction."""

from __future__ import annotations

import unicodedata
from collections.abc import Mapping
from typing import Any

MAX_TEXT_CHARS = 80


def _block(error_code: str, message: str) -> dict[str, Any]:
    return {
        "ok": False,
        "decision": "BLOCK",
        "schema_version": "limo.speech.v1",
        "error_code": error_code,
        "message": message,
        "command_dispatched": False,
        "usable_for_real_execution": False,
    }


def validate_speech(
    *,
    text: str,
    language: str,
    volume_percent: int,
    rate_wpm: int,
    body_snapshot_hash: str,
) -> dict[str, Any]:
    if not str(body_snapshot_hash).strip():
        return _block(
            "BODY_SNAPSHOT_REQUIRED",
            "Speech requests require an immutable LIMO body snapshot.",
        )
    if not isinstance(text, str) or not text or len(text) > MAX_TEXT_CHARS:
        return _block(
            "LIMO_SPEECH_TEXT_INVALID",
            f"text length must be within [1, {MAX_TEXT_CHARS}] characters.",
        )
    if text != text.strip() or any(
        unicodedata.category(character).startswith("C") for character in text
    ):
        return _block(
            "LIMO_SPEECH_TEXT_INVALID",
            "text must not contain surrounding whitespace, controls, or formatting characters.",
        )
    if language not in {"cmn", "en"}:
        return _block("LIMO_SPEECH_LANGUAGE_INVALID", "language must be cmn or en.")
    if (
        isinstance(volume_percent, bool)
        or not isinstance(volume_percent, int)
        or not 10 <= volume_percent <= 25
    ):
        return _block(
            "LIMO_SPEECH_VOLUME_INVALID",
            "volume_percent must be an integer within [10, 25].",
        )
    if isinstance(rate_wpm, bool) or not isinstance(rate_wpm, int) or not 120 <= rate_wpm <= 200:
        return _block(
            "LIMO_SPEECH_RATE_INVALID",
            "rate_wpm must be an integer within [120, 200].",
        )
    return {
        "ok": True,
        "decision": "ALLOW",
        "normalized_speech": {
            "text": text,
            "language": language,
            "volume_percent": volume_percent,
            "rate_wpm": rate_wpm,
        },
    }


def speech_arguments(speech: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": "limo.speech.v1",
        "text": speech["text"],
        "language": speech["language"],
        "volume_percent": speech["volume_percent"],
        "rate_wpm": speech["rate_wpm"],
        "expected_effect": {
            "kind": "speaker_speech",
            "playback_required": True,
            "mixer_restore_required": True,
            "microphone_loopback_required": True,
            "content_recognition_required": False,
        },
    }


def speech_display(speech: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": "rosclaw.action-display.v2",
        "title": "Speak a short message through the LIMO speaker",
        "summary": (
            "Synthesize one bounded message, verify acoustic output with the onboard "
            "microphone, and restore the mixer state."
        ),
        "body": {"robot_id": "limo", "speech": dict(speech)},
        "risk_tier": "MEDIUM",
        "physical_effects": ["The robot speaker will emit synthesized speech."],
        "constraints": ["Text, language, rate, and volume remain within the speech policy."],
        "verification": [
            "Verify synthesis completion, microphone energy gain, and mixer restoration."
        ],
        "abort": ["Playback ends after the bounded text has been spoken."],
    }
