from __future__ import annotations

import array
import math
import wave
from datetime import UTC, datetime
from pathlib import Path

import pytest

from scripts.limo_find_person_greet import (
    action_deadline,
    analyze_wav,
    angle_distance,
    angle_normalize,
    normalize_body_snapshot_hash,
    parse_json_object,
    parse_pulse_source_inventory,
    record_pulse_response,
    transcribe_google,
    validate_detection,
)


def test_action_deadline_allows_cross_turn_confirmation() -> None:
    now = datetime(2030, 1, 2, 3, 4, 5, tzinfo=UTC)

    assert action_deadline(now=now) == "2030-01-02T03:19:05Z"


def test_action_deadline_rejects_naive_reference() -> None:
    with pytest.raises(ValueError, match="timezone-aware"):
        action_deadline(now=datetime(2030, 1, 2, 3, 4, 5))


def test_body_snapshot_hash_matches_robot_instance_raw_digest() -> None:
    digest = "a" * 64

    assert normalize_body_snapshot_hash(digest) == digest
    assert normalize_body_snapshot_hash(f"sha256:{digest}") == digest


def test_body_snapshot_hash_rejects_non_sha256_value() -> None:
    with pytest.raises(RuntimeError, match="body hash is invalid"):
        normalize_body_snapshot_hash("sha256:test-body")


def test_parse_fenced_detection_and_reject_low_confidence() -> None:
    parsed = parse_json_object(
        '```json\n{"person_present":true,"people_count":1,'
        '"selected_person":{"bbox_norm":[0.1,0.2,0.4,0.9],'
        '"horizontal":"left","seated":true,"confidence":0.64},'
        '"scene":"room","hazards":[]}\n```'
    )

    result = validate_detection(parsed)

    assert result["person_present"] is False
    assert result["selected_person"] is None


def test_validate_detection_accepts_seated_person_bbox() -> None:
    result = validate_detection(
        {
            "person_present": True,
            "people_count": 1,
            "selected_person": {
                "bbox_norm": [0.3, 0.25, 0.7, 0.95],
                "horizontal": "center",
                "seated": True,
                "confidence": 0.92,
            },
            "scene": "office",
            "hazards": [],
        }
    )

    assert result["person_present"] is True
    assert result["selected_person"]["seated"] is True


def test_audio_metrics_detect_response_energy(tmp_path: pytest.TempPathFactory) -> None:
    path = tmp_path / "response.wav"
    rate = 16000
    samples = array.array(
        "h", [int(7000 * math.sin(2 * math.pi * 440 * index / rate)) for index in range(rate)]
    )
    with wave.open(str(path), "wb") as writer:
        writer.setnchannels(1)
        writer.setsampwidth(2)
        writer.setframerate(rate)
        writer.writeframes(samples.tobytes())

    metrics = analyze_wav(path)

    assert metrics["duration_sec"] == 1.0
    assert metrics["speech_energy_detected"] is True


def test_pulse_source_inventory_selects_only_usb_microphone() -> None:
    inventory = (
        "1\talsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo.monitor\tmodule\n"
        "2\talsa_input.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo\tmodule\n"
        "3\talsa_input.platform-sound.analog-stereo\tmodule\n"
    )

    assert parse_pulse_source_inventory(inventory) == (
        "alsa_input.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo"
    )


def test_response_capture_retries_truncated_pulse_audio(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    full_pcm = b"\x01\x00" * 16000
    captures = iter([b"\x00\x00" * 100, full_pcm])
    monkeypatch.setattr(
        "scripts.limo_find_person_greet.pulse_response_source", lambda: "allowlisted-source"
    )
    monkeypatch.setattr(
        "scripts.limo_find_person_greet._capture_pulse_pcm",
        lambda _source, _duration: next(captures),
    )
    path = tmp_path / "response.wav"

    record_pulse_response(path, duration_sec=1.0)

    metrics = analyze_wav(path)
    assert metrics["duration_sec"] == 1.0
    assert metrics["sample_count"] == 16000


def test_response_capture_rejects_two_truncated_attempts(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(
        "scripts.limo_find_person_greet.pulse_response_source", lambda: "allowlisted-source"
    )
    monkeypatch.setattr(
        "scripts.limo_find_person_greet._capture_pulse_pcm",
        lambda _source, _duration: b"\x00\x00" * 100,
    )

    with pytest.raises(RuntimeError, match="truncated"):
        record_pulse_response(tmp_path / "response.wav", duration_sec=1.0)


def test_angle_helpers_cross_pi_boundary() -> None:
    assert angle_normalize(3 * math.pi) == pytest.approx(math.pi)
    assert angle_distance(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(0.2)


def test_cloud_asr_requires_protected_environment_key(
    monkeypatch: pytest.MonkeyPatch, tmp_path: Path
) -> None:
    monkeypatch.delenv("ROSCLAW_LIMO_GOOGLE_SPEECH_API_KEY", raising=False)

    result = transcribe_google(tmp_path / "response.wav")

    assert result == {
        "ok": False,
        "error": "ROSCLAW_LIMO_GOOGLE_SPEECH_API_KEY is not configured",
        "engine": "google-speech-zh-CN",
    }
