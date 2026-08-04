"""Protocol tests for bounded tone and navigation daemon workers."""

from __future__ import annotations

import importlib.util
import json
import math
import os
import struct
import subprocess
import sys
from io import StringIO
from pathlib import Path
from types import ModuleType

import pytest

ROOT = Path(__file__).parents[1]


def _load(name: str) -> ModuleType:
    path = ROOT / "worker" / name
    spec = importlib.util.spec_from_file_location(name.removesuffix(".py"), path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TONE = _load("limo_tone_worker.py")
NAVIGATION = _load("limo_navigation_worker.py")
sys.modules["limo_tone_worker"] = TONE
SPEECH = _load("limo_speech_worker.py")


def _tone_request() -> dict[str, object]:
    return {
        "protocol": "rosclaw.limo.worker.v1",
        "operation": "PLAY_TONE",
        "schema_version": "limo.tone.v1",
        "action_id": "action-tone-test",
        "body_id": "limo",
        "body_snapshot_hash": "sha256:test-body",
        "frequency_hz": 660,
        "duration_sec": 0.6,
        "volume_percent": 18,
    }


def _raw_tone(frequency_hz: int, duration_sec: float, amplitude: float) -> bytes:
    frames = int(TONE.SAMPLE_RATE_HZ * duration_sec)
    return b"".join(
        struct.pack(
            "<h",
            int(
                32767
                * amplitude
                * math.sin(2.0 * math.pi * frequency_hz * index / TONE.SAMPLE_RATE_HZ)
            ),
        )
        for index in range(frames)
    )


def _navigation_request() -> dict[str, object]:
    return {
        "protocol": "rosclaw.limo.worker.v1",
        "operation": "NAVIGATE_TO_POSE",
        "schema_version": "limo.navigation.v2",
        "action_id": "action-navigation-test",
        "body_id": "limo",
        "body_snapshot_hash": "sha256:test-body",
        "target_pose": {"frame_id": "map", "x": 0.4, "y": 0.0, "yaw": 0.0},
        "goal_tolerance": {"xy_m": 0.15, "yaw_rad": 0.2},
        "server_timeout_sec": 3.0,
        "navigation_timeout_sec": 30.0,
        "verification_timeout_sec": 5.0,
    }


def _speech_request() -> dict[str, object]:
    return {
        "protocol": "rosclaw.limo.worker.v1",
        "operation": "SPEAK_TEXT",
        "schema_version": "limo.speech.v1",
        "action_id": "action-speech-test",
        "body_id": "limo",
        "body_snapshot_hash": "sha256:test-body",
        "text": "你好，我是 LIMO 巡检机器人。",
        "language": "cmn",
        "volume_percent": 18,
        "rate_wpm": 160,
    }


@pytest.mark.parametrize(
    ("module", "payload"),
    [(TONE, _tone_request()), (NAVIGATION, _navigation_request()), (SPEECH, _speech_request())],
)
def test_worker_contract_accepts_exact_bounded_request(
    module: ModuleType, payload: dict[str, object]
) -> None:
    normalized = module.validate_request(payload)

    assert normalized["action_id"] == payload["action_id"]
    assert normalized["body_snapshot_hash"] == payload["body_snapshot_hash"]


@pytest.mark.parametrize(
    ("module", "payload"),
    [(TONE, _tone_request()), (NAVIGATION, _navigation_request()), (SPEECH, _speech_request())],
)
def test_worker_contract_rejects_unknown_surface(
    module: ModuleType, payload: dict[str, object]
) -> None:
    payload["command"] = "arbitrary"

    with pytest.raises(module.RequestError, match="unknown request fields"):
        module.validate_request(payload)


@pytest.mark.parametrize(
    "override",
    [
        {"frequency_hz": 1000},
        {"duration_sec": 5.0},
        {"volume_percent": 80},
    ],
)
def test_tone_worker_rejects_unbounded_parameters(override: dict[str, object]) -> None:
    request = {**_tone_request(), **override}

    with pytest.raises(TONE.RequestError):
        TONE.validate_request(request)


@pytest.mark.parametrize("card_name", ["USB PnP Audio Device", "USB PnP Sound Device"])
def test_tone_worker_accepts_known_usb_audio_card_names(monkeypatch, card_name: str) -> None:
    inventory = (
        " 0 [tegrahda       ]: tegra-hda - tegra-hda\n"
        "                      built-in audio\n"
        f" 2 [Device         ]: USB-Audio - {card_name}\n"
        f"                      {card_name} at usb-test\n"
    )
    monkeypatch.setattr("builtins.open", lambda *_args, **_kwargs: StringIO(inventory))

    assert TONE._usb_audio_card() == 2


def test_tone_worker_uses_allowlisted_pulseaudio_sink(monkeypatch) -> None:
    calls: list[tuple[list[str], bytes | None]] = []

    def fake_run(command, input_bytes=None, env=None):
        del env
        calls.append((command, input_bytes))
        if command[0] == "/usr/bin/pactl":
            return (
                0,
                (
                    b"0\talsa_output.platform-sound.analog-stereo\tmodule-alsa-card.c\n"
                    b"1\talsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo"
                    b"\tmodule-alsa-card.c\n"
                ),
                b"",
            )
        return 0, b"", b""

    monkeypatch.setattr(TONE, "_pulse_server", lambda: "unix:/run/user/1000/pulse/native")
    monkeypatch.setattr(TONE, "_run", fake_run)

    backend, device = TONE._play_tone(2, b"RIFF-test")

    assert backend == "pulseaudio"
    assert device == "pulse:alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo"
    assert calls[1][0] == [
        "/usr/bin/paplay",
        "--server",
        "unix:/run/user/1000/pulse/native",
        "--device",
        "alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo",
        "--client-name",
        "rosclaw-limo-tone",
        "--stream-name",
        "bounded-tone",
    ]
    assert calls[1][1] == b"RIFF-test"


def test_tone_worker_falls_back_to_fixed_alsa_device(monkeypatch) -> None:
    calls: list[tuple[list[str], bytes | None]] = []

    def fake_run(command, input_bytes=None, env=None):
        del env
        calls.append((command, input_bytes))
        return 0, b"", b""

    monkeypatch.setattr(TONE, "_pulse_server", lambda: None)
    monkeypatch.setattr(TONE, "_run", fake_run)

    backend, device = TONE._play_tone(2, b"RIFF-test")

    assert backend == "alsa"
    assert device == "plughw:2,0"
    assert calls == [(["/usr/bin/aplay", "-q", "-D", "plughw:2,0"], b"RIFF-test")]


def test_tone_worker_retries_transient_busy_microphone(monkeypatch) -> None:
    calls = 0

    def fake_run(_command, input_bytes=None, env=None):
        nonlocal calls
        del input_bytes, env
        calls += 1
        if calls < 3:
            return 1, b"", b"audio open error: Device or resource busy"
        return 0, b"pcm", b""

    sleeps: list[float] = []
    monkeypatch.setattr(TONE, "_run", fake_run)
    monkeypatch.setattr(TONE.time, "sleep", sleeps.append)

    assert TONE._capture_pcm(2, 1) == (b"pcm", "plughw:2,0")
    assert calls == 3
    assert sleeps == [TONE.CAPTURE_BUSY_RETRY_DELAY_SEC] * 2


def test_tone_worker_does_not_retry_non_busy_microphone_error(monkeypatch) -> None:
    monkeypatch.setattr(
        TONE,
        "_run",
        lambda *_args, **_kwargs: (1, b"", b"audio open error: no such device"),
    )

    with pytest.raises(TONE.RequestError, match="no such device"):
        TONE._capture_pcm(2, 1)


def test_tone_worker_rejects_ambiguous_pulseaudio_sinks(monkeypatch) -> None:
    inventory = (
        b"1\talsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo\tmodule\n"
        b"2\talsa_output.usb-0c76_USB_PnP_Sound_Device-00.analog-stereo\tmodule\n"
    )
    monkeypatch.setattr(TONE, "_run", lambda *_args, **_kwargs: (0, inventory, b""))

    with pytest.raises(TONE.RequestError, match="exactly one"):
        TONE._pulse_sink("unix:/run/user/1000/pulse/native")


def test_tone_worker_selects_allowlisted_pulseaudio_source(monkeypatch) -> None:
    inventory = (
        b"1\talsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo.monitor\tmodule\n"
        b"2\talsa_input.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo\tmodule\n"
        b"3\talsa_input.platform-sound.analog-stereo\tmodule\n"
    )
    monkeypatch.setattr(TONE, "_run", lambda *_args, **_kwargs: (0, inventory, b""))

    assert TONE._pulse_source(TONE.ALLOWED_PULSE_SERVER) == (
        "alsa_input.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo"
    )


def test_tone_worker_rejects_unallowlisted_explicit_pulse_server(monkeypatch) -> None:
    monkeypatch.setenv("ROSCLAW_LIMO_PULSE_SERVER", "unix:/tmp/untrusted-pulse")

    with pytest.raises(TONE.RequestError, match="not allowlisted"):
        TONE._pulse_server()


def test_tone_worker_uses_reference_gain_and_restores_alsa_state(monkeypatch) -> None:
    calls: list[tuple[int, int, bool, bool]] = []
    monkeypatch.setattr(TONE, "_usb_audio_card", lambda: 2)
    monkeypatch.setattr(TONE, "_pulse_server", lambda: None)
    monkeypatch.setattr(TONE, "_speaker_state", lambda _card: (0, True))
    monkeypatch.setattr(
        TONE,
        "_set_speaker_state",
        lambda card, percent, unmuted, mapped=False: calls.append((card, percent, unmuted, mapped)),
    )
    monkeypatch.setattr(
        TONE,
        "_capture_pcm",
        lambda _card, _duration, _server=None: (
            _raw_tone(440, 1.0, 0.0001),
            "plughw:2,0",
        ),
    )
    monkeypatch.setattr(
        TONE,
        "_capture_during_playback",
        lambda _card, _wav: (
            "alsa",
            "plughw:2,0",
            "plughw:2,0",
            _raw_tone(660, 2.0, 0.1),
        ),
    )

    result = TONE._run_audio(_tone_request())

    assert calls == [(2, 100, True, True), (2, 0, True, False)]
    assert result["volume_mapping"] == "pcm_linear_percent"
    assert result["digital_peak_scale"] == pytest.approx(0.162)
    assert result["reference_output_gain_percent"] == 100
    assert result["acoustic_loopback_detected"] is True
    assert result["acoustic_loopback"]["audio_retained"] is False


def test_tone_worker_parses_and_restores_pulseaudio_sink_state(monkeypatch) -> None:
    inventory = b"""Sink #1
\tName: alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo
\tMute: no
\tVolume: front-left: 5841 / 9% / -63.00 dB, front-right: 5841 / 9% / -63.00 dB
"""
    monkeypatch.setattr(TONE, "_run", lambda *_args, **_kwargs: (0, inventory, b""))

    state = TONE._pulse_sink_state(
        "unix:/run/user/1000/pulse/native",
        "alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo",
    )

    assert state == {"channel_volumes": [5841, 5841], "unmuted": True}


def test_tone_wave_avoids_double_attenuation() -> None:
    wav_bytes, frame_count = TONE._tone_wav(660, 0.6, 18)

    assert frame_count == 9600
    assert len(wav_bytes) > frame_count * 2
    assert pytest.approx(0.9) == TONE.MAX_AMPLITUDE


def test_tone_loopback_detects_target_frequency_gain() -> None:
    baseline = _raw_tone(440, 1.0, 0.001)
    observed = _raw_tone(660, 2.0, 0.08)

    evidence = TONE._loopback_evidence(baseline, observed, 660, "plughw:2,0")

    assert evidence["detected"] is True
    assert evidence["capture_device"] == "plughw:2,0"
    assert evidence["target_gain_db"] >= 30.0
    assert evidence["during_playback"]["target_prominence_db"] >= 30.0
    assert evidence["audio_content_returned"] is False


def test_tone_loopback_rejects_unrelated_loud_audio() -> None:
    baseline = _raw_tone(440, 1.0, 0.001)
    observed = _raw_tone(880, 2.0, 0.2)

    evidence = TONE._loopback_evidence(baseline, observed, 660, "plughw:2,0")

    assert evidence["detected"] is False


def test_speech_worker_rejects_controls_and_unbounded_text() -> None:
    control = {**_speech_request(), "text": "hello\nworld"}
    too_long = {**_speech_request(), "text": "巡" * 81}

    with pytest.raises(SPEECH.RequestError, match="control"):
        SPEECH.validate_request(control)
    with pytest.raises(SPEECH.RequestError, match="text length"):
        SPEECH.validate_request(too_long)


def test_speech_worker_normalizes_pcm_to_requested_peak() -> None:
    raw = struct.pack("<4h", -1000, 0, 500, 1000)

    normalized, frame_count, peak_scale = SPEECH._normalize_pcm(raw, 18)
    values = struct.unpack("<4h", normalized)

    assert frame_count == 4
    assert peak_scale == pytest.approx(0.162, abs=0.0001)
    assert max(abs(value) for value in values) == pytest.approx(32767 * 0.162, abs=2)


def test_speech_loopback_requires_rms_gain() -> None:
    quiet = struct.pack("<16000h", *([20] * 16000))
    audible = struct.pack("<32000h", *([4000, -4000] * 16000))

    evidence = SPEECH._speech_loopback(quiet, audible, "plughw:2,0")

    assert evidence["detected"] is True
    assert evidence["content_recognition_performed"] is False


def test_navigation_worker_rejects_non_map_and_unbounded_timeout() -> None:
    wrong_frame = _navigation_request()
    wrong_frame["target_pose"] = {"frame_id": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0}
    long_timeout = {**_navigation_request(), "navigation_timeout_sec": 121.0}

    with pytest.raises(NAVIGATION.RequestError, match="frame_id must be map"):
        NAVIGATION.validate_request(wrong_frame)
    with pytest.raises(NAVIGATION.RequestError, match="navigation_timeout_sec"):
        NAVIGATION.validate_request(long_timeout)


def test_navigation_worker_waits_for_post_dispatch_amcl(monkeypatch) -> None:
    def message(stamp: float):
        return type(
            "Message",
            (),
            {
                "header": type(
                    "Header", (), {"stamp": type("Stamp", (), {"to_sec": lambda self: stamp})()}
                )()
            },
        )()

    before = message(99.0)
    after = message(100.1)
    state = {"message": before, "received_wall_time": 9.0}

    class FakeRospy:
        sleeps = 0

        @staticmethod
        def is_shutdown() -> bool:
            return False

        def sleep(self, _duration: float) -> None:
            self.sleeps += 1
            state["message"] = after
            state["received_wall_time"] = 10.1

    clock = iter([10.0, 10.0, 10.2])
    monkeypatch.setattr(NAVIGATION.time, "time", lambda: next(clock))
    rospy = FakeRospy()

    observed = NAVIGATION._wait_for_post_dispatch_amcl(rospy, state, 10.0, 100.0, 1.0)

    assert observed is after
    assert rospy.sleeps == 1


def test_navigation_worker_returns_none_when_post_dispatch_amcl_is_event_silent(
    monkeypatch,
) -> None:
    state = {"message": object(), "received_wall_time": 9.0}

    class FakeRospy:
        @staticmethod
        def is_shutdown() -> bool:
            return False

        @staticmethod
        def sleep(_duration: float) -> None:
            pass

    clock = iter([10.0, 10.0, 10.2])
    monkeypatch.setattr(NAVIGATION.time, "time", lambda: next(clock))

    observed = NAVIGATION._wait_for_post_dispatch_amcl(FakeRospy(), state, 10.0, 100.0, 0.1)

    assert observed is None


def test_navigation_worker_rejects_late_delivery_of_stale_amcl() -> None:
    class Stamp:
        @staticmethod
        def to_sec() -> float:
            return 99.0

    message = type("Message", (), {"header": type("Header", (), {"stamp": Stamp()})()})()

    assert not NAVIGATION._amcl_is_post_dispatch(message, 10.1, 10.0, 100.0)


def test_navigation_worker_builds_map_pose_from_live_tf() -> None:
    observed = NAVIGATION._pose_from_transform(
        (0.3, -0.1, 0.0),
        (0.0, 0.0, 0.0998334166468, 0.995004165278),
    )

    assert observed["frame_id"] == "map"
    assert observed["x"] == pytest.approx(0.3)
    assert observed["y"] == pytest.approx(-0.1)
    assert observed["yaw"] == pytest.approx(0.2)


def test_navigation_worker_classifies_already_satisfied_goal_and_motion() -> None:
    before = {"frame_id": "map", "x": 0.17, "y": 0.0, "yaw": 0.0}
    after = {"frame_id": "map", "x": 0.18, "y": 0.0, "yaw": 0.01}

    translation, rotation = NAVIGATION._pose_delta(before, after)

    assert translation == pytest.approx(0.01)
    assert rotation == pytest.approx(0.01)
    assert translation < NAVIGATION.MIN_OBSERVED_TRANSLATION_M
    assert rotation < NAVIGATION.MIN_OBSERVED_ROTATION_RAD


def test_navigation_worker_reads_active_trajectory_planner_tolerance() -> None:
    class FakeRospy:
        @staticmethod
        def get_param(name: str, default: float) -> float:
            return {
                "/move_base/TrajectoryPlannerROS/xy_goal_tolerance": 0.2,
                "/move_base/TrajectoryPlannerROS/yaw_goal_tolerance": 0.15,
            }.get(name, default)

    tolerance = NAVIGATION._active_goal_tolerance(FakeRospy(), {"xy_m": 0.1, "yaw_rad": 0.1})

    assert tolerance == {"xy_m": 0.2, "yaw_rad": 0.15}


@pytest.mark.parametrize(
    ("worker", "payload"),
    [
        ("limo_tone_worker.py", _tone_request()),
        ("limo_navigation_worker.py", _navigation_request()),
        ("limo_speech_worker.py", _speech_request()),
    ],
)
def test_worker_validate_only_subprocess(worker: str, payload: dict[str, object]) -> None:
    completed = subprocess.run(
        [sys.executable, str(ROOT / "worker" / worker)],
        input=json.dumps(payload),
        text=True,
        capture_output=True,
        check=False,
        env={**os.environ, "ROSCLAW_LIMO_VALIDATE_ONLY": "1"},
    )

    assert completed.returncode == 0, completed.stderr
    result = json.loads(completed.stdout)
    assert result["ok"] is True
