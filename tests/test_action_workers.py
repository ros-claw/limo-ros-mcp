"""Protocol tests for bounded tone and navigation daemon workers."""

from __future__ import annotations

import importlib.util
import json
import os
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


@pytest.mark.parametrize(
    ("module", "payload"),
    [(TONE, _tone_request()), (NAVIGATION, _navigation_request())],
)
def test_worker_contract_accepts_exact_bounded_request(
    module: ModuleType, payload: dict[str, object]
) -> None:
    normalized = module.validate_request(payload)

    assert normalized["action_id"] == payload["action_id"]
    assert normalized["body_snapshot_hash"] == payload["body_snapshot_hash"]


@pytest.mark.parametrize(
    ("module", "payload"),
    [(TONE, _tone_request()), (NAVIGATION, _navigation_request())],
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


def test_tone_worker_rejects_ambiguous_pulseaudio_sinks(monkeypatch) -> None:
    inventory = (
        b"1\talsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo\tmodule\n"
        b"2\talsa_output.usb-0c76_USB_PnP_Sound_Device-00.analog-stereo\tmodule\n"
    )
    monkeypatch.setattr(TONE, "_run", lambda *_args, **_kwargs: (0, inventory, b""))

    with pytest.raises(TONE.RequestError, match="exactly one"):
        TONE._pulse_sink("unix:/run/user/1000/pulse/native")


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
    monkeypatch.setattr(TONE, "_play_tone", lambda _card, _wav: ("alsa", "plughw:2,0"))

    result = TONE._run_audio(_tone_request())

    assert calls == [(2, 100, True, True), (2, 0, True, False)]
    assert result["volume_mapping"] == "pcm_linear_percent"
    assert result["digital_peak_scale"] == pytest.approx(0.162)
    assert result["reference_output_gain_percent"] == 100


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


def test_navigation_worker_rejects_non_map_and_unbounded_timeout() -> None:
    wrong_frame = _navigation_request()
    wrong_frame["target_pose"] = {"frame_id": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0}
    long_timeout = {**_navigation_request(), "navigation_timeout_sec": 121.0}

    with pytest.raises(NAVIGATION.RequestError, match="frame_id must be map"):
        NAVIGATION.validate_request(wrong_frame)
    with pytest.raises(NAVIGATION.RequestError, match="navigation_timeout_sec"):
        NAVIGATION.validate_request(long_timeout)


def test_navigation_worker_waits_for_post_dispatch_amcl(monkeypatch) -> None:
    before = object()
    after = object()
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

    observed = NAVIGATION._wait_for_post_dispatch_amcl(rospy, state, 10.0, 1.0)

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

    observed = NAVIGATION._wait_for_post_dispatch_amcl(FakeRospy(), state, 10.0, 0.1)

    assert observed is None


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
