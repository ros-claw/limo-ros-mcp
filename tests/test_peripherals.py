"""Tests for bounded LIMO host-peripheral inspection."""

from __future__ import annotations

import array
import subprocess
from pathlib import Path
from typing import Any

import pytest

from limo_ros_mcp.peripherals import PeripheralInspector, _default_text_runner


def _write(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8")


def _inspector(tmp_path: Path) -> PeripheralInspector:
    sys_root = tmp_path / "sys"
    proc_root = tmp_path / "proc"
    dev_root = tmp_path / "dev"
    for vendor, product, name, usb_name in (
        ("2bc5", "0557", "Dabai DC1", "1-1"),
        ("0c76", "161f", "USB PnP Audio Device", "1-2"),
        ("1a86", "e5e3", "USB2IIC_CTP_CONTROL", "1-3"),
    ):
        root = sys_root / "bus" / "usb" / "devices" / usb_name
        _write(root / "idVendor", vendor)
        _write(root / "idProduct", product)
        _write(root / "product", name)
    _write(sys_root / "class" / "graphics" / "fb0" / "name", "tegra_fb")
    _write(sys_root / "class" / "graphics" / "fb0" / "virtual_size", "1024,600")
    _write(sys_root / "class" / "graphics" / "fb0" / "bits_per_pixel", "32")
    _write(
        sys_root / "class" / "input" / "input3" / "name",
        "wch.cn USB2IIC_CTP_CONTROL",
    )
    _write(sys_root / "class" / "input" / "input3" / "id" / "vendor", "1a86")
    _write(sys_root / "class" / "input" / "input3" / "id" / "product", "e5e3")
    _write(sys_root / "class" / "thermal" / "thermal_zone0" / "type", "CPU-therm")
    _write(sys_root / "class" / "thermal" / "thermal_zone0" / "temp", "43000")
    _write(
        proc_root / "asound" / "cards",
        " 2 [Device         ]: USB-Audio - USB PnP Audio Device\n",
    )
    _write(
        proc_root / "asound" / "pcm",
        "02-00: USB Audio : USB Audio : playback 1 : capture 1\n",
    )
    _write(
        proc_root / "meminfo",
        "MemTotal:        4000000 kB\nMemAvailable:    2000000 kB\n"
        "SwapTotal:       1000000 kB\nSwapFree:         500000 kB\n",
    )
    _write(proc_root / "uptime", "1234.5 100.0\n")

    def text_runner(command: list[str], _timeout_sec: float) -> tuple[int, str, str]:
        if command == ["rostopic", "list"]:
            return 0, "/limo_status\n/odom\n/imu\n", ""
        if command[:2] == ["rosservice", "call"]:
            service_values = {
                "/camera/get_device_info": (
                    "info:\n  name: Astra\n  serial_number: CC15C43008A\n"
                    "success: true\nmessage: ''\n"
                ),
                "/camera/get_device_type": "data: Orbbec Astra DaBai DC1\nsuccess: true\n",
                "/camera/get_serial": "data: CC15C43008A\nsuccess: true\n",
                "/camera/get_version": (
                    'data: \'{"firmware_version": "RD1001", "ros_sdk_version": "1.2.7"}\'\n'
                    "success: true\n"
                ),
                "/camera/get_ir_temperature": "data: 34.2\nsuccess: true\n",
                "/camera/get_ldp_status": "data: false\nsuccess: true\n",
            }
            return 0, service_values[command[2]], ""
        if command[-1] == "Speaker":
            return (
                0,
                "Front Left: Playback 504 [50%] [-20.00dB] [on]\n"
                "Front Right: Playback 504 [50%] [-20.00dB] [on]\n",
                "",
            )
        if command[-1] == "Mic":
            return 0, "Mono: Capture 408 [82%] [25.50dB] [on]\n", ""
        return 1, "", "unexpected command"

    samples = array.array("h", [1000, -1000] * 8000).tobytes()

    def binary_runner(_command: list[str], _timeout_sec: float) -> tuple[int, bytes, str]:
        return 0, samples, ""

    return PeripheralInspector(
        sys_root=sys_root,
        proc_root=proc_root,
        dev_root=dev_root,
        text_runner=text_runner,
        binary_runner=binary_runner,
    )


def test_inventory_correlates_usb_ros_and_declared_unbound_devices(tmp_path: Path) -> None:
    result = _inspector(tmp_path).inventory()

    assert result["ok"] is True
    assert result["peripheral_count"] == 12
    by_id = {item["id"]: item for item in result["peripherals"]}
    assert by_id["dabai_color_camera"]["status"] == "connected"
    assert by_id["voice_audio"]["status"] == "connected"
    assert by_id["chassis_controller"]["status"] == "connected"
    assert by_id["rear_display"]["status"] == "connected"
    assert by_id["front_oled"]["status"] == "declared_unbound"
    assert by_id["chassis_rgb_lights"]["mcp_bound"] is False
    assert result["command_dispatched"] is False


def test_audio_state_reports_usb_mixer_without_changing_it(tmp_path: Path) -> None:
    result = _inspector(tmp_path).audio_state()

    assert result["ok"] is True
    assert result["alsa_card"] == 2
    assert result["playback_ready"] is True
    assert result["capture_ready"] is True
    assert result["speaker"]["volume_percent"] == 50.0
    assert result["speaker"]["muted"] is False
    assert result["microphone"]["volume_percent"] == 82.0
    assert result["physical_amplifier_interface"] is False
    assert result["command_dispatched"] is False


def test_microphone_level_returns_statistics_without_audio_content(tmp_path: Path) -> None:
    result = _inspector(tmp_path).microphone_level(duration_sec=1, sample_rate_hz=16000)

    assert result["ok"] is True
    assert result["sample_count"] == 16000
    assert result["rms_dbfs"] < 0
    assert result["peak_dbfs"] < 0
    assert result["audio_retained"] is False
    assert result["audio_content_returned"] is False
    assert "audio" not in result
    assert result["command_dispatched"] is False


def test_audio_and_microphone_use_allowlisted_pulse_bridge(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    proc_root = tmp_path / "proc"
    _write(
        proc_root / "asound" / "cards",
        " 2 [Device         ]: USB-Audio - USB PnP Audio Device\n",
    )
    _write(
        proc_root / "asound" / "pcm",
        "02-00: USB Audio : USB Audio : playback 1 : capture 1\n",
    )
    sink = "alsa_output.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo"
    source = "alsa_input.usb-0c76_USB_PnP_Audio_Device-00.analog-stereo"
    monkeypatch.setenv("ROSCLAW_LIMO_PULSE_SERVER", "unix:/run/rosclaw/pulse/native")

    def text_runner(command: list[str], _timeout_sec: float) -> tuple[int, str, str]:
        if command[-3:] == ["list", "short", "sinks"]:
            return 0, f"1\t{sink}\tmodule-alsa-card.c\ts16le 2ch 48000Hz\n", ""
        if command[-3:] == ["list", "short", "sources"]:
            return 0, f"2\t{source}\tmodule-alsa-card.c\ts16le 2ch 48000Hz\n", ""
        if command[-2:] == ["list", "sinks"]:
            return (
                0,
                f"Sink #1\n\tName: {sink}\n\tMute: no\n"
                "\tVolume: front-left: 11796 / 18% / -44.68 dB, "
                "front-right: 11796 / 18% / -44.68 dB\n",
                "",
            )
        if command[-2:] == ["list", "sources"]:
            return (
                0,
                f"Source #2\n\tName: {source}\n\tMute: no\n"
                "\tVolume: front-left: 65536 / 100% / 0.00 dB, "
                "front-right: 65536 / 100% / 0.00 dB\n",
                "",
            )
        return 1, "", "unexpected command"

    samples = array.array("h", [1200, -1200] * 10_000).tobytes()

    def binary_runner(command: list[str], _timeout_sec: float) -> tuple[int, bytes, str]:
        assert command[0] == "/usr/bin/timeout"
        assert command[3] == "/usr/bin/parec"
        assert source in command
        return 124, samples, ""

    inspector = PeripheralInspector(
        sys_root=tmp_path / "sys",
        proc_root=proc_root,
        dev_root=tmp_path / "dev",
        text_runner=text_runner,
        binary_runner=binary_runner,
    )

    state = inspector.audio_state()
    level = inspector.microphone_level(duration_sec=1, sample_rate_hz=16000)

    assert state["ok"] is True
    assert state["audio_transport"] == "pulseaudio"
    assert state["speaker"]["available"] is True
    assert state["speaker"]["volume_percent"] == 18.0
    assert state["microphone"]["available"] is True
    assert level["ok"] is True
    assert level["capture_transport"] == "pulseaudio"
    assert level["capture_device"] == source
    assert level["sample_count"] == 16000
    assert level["audio_retained"] is False


@pytest.mark.parametrize(
    ("duration_sec", "sample_rate_hz"),
    [(0, 16000), (4, 16000), (True, 16000), (1, 44100)],
)
def test_microphone_level_rejects_unbounded_parameters(
    tmp_path: Path, duration_sec: int, sample_rate_hz: int
) -> None:
    with pytest.raises(ValueError):
        _inspector(tmp_path).microphone_level(
            duration_sec=duration_sec,
            sample_rate_hz=sample_rate_hz,
        )


def test_display_and_platform_health_are_read_only(tmp_path: Path) -> None:
    inspector = _inspector(tmp_path)

    display = inspector.display_state()
    assert display["rear_display_ready"] is True
    assert display["framebuffers"][0]["width"] == 1024
    assert display["touchscreens"][0]["vendor_id"] == "1a86"
    assert display["front_oled"]["interface_bound"] is False

    health = inspector.platform_health()
    assert health["ok"] is True
    assert health["maximum_temperature_c"] == 43.0
    assert health["memory"]["total_bytes"] == 4_000_000 * 1024
    assert health["uptime_sec"] == 1234.5
    assert health["command_dispatched"] is False


def test_dabai_device_state_uses_getter_only_service_allowlist(tmp_path: Path) -> None:
    result = _inspector(tmp_path).dabai_device_state()

    assert result["ok"] is True
    assert result["successful_service_count"] == 6
    assert result["device"]["serial"] == "CC15C43008A"
    assert result["device"]["version"]["firmware_version"] == "RD1001"
    assert result["device"]["ir_temperature_c"] == 34.2
    assert result["device"]["ldp_enabled"] is False
    assert all("/get_" in service for service in result["read_only_services"])
    assert result["command_dispatched"] is False


def test_default_text_runner_supplies_ros1_cli_environment(monkeypatch: Any) -> None:
    captured: dict[str, Any] = {}

    def run(command: list[str], **kwargs: Any) -> subprocess.CompletedProcess[str]:
        captured.update(kwargs)
        return subprocess.CompletedProcess(command, 0, stdout="success: true\n", stderr="")

    monkeypatch.setattr(
        "limo_ros_mcp.roscli.Path.is_dir",
        lambda path: str(path).startswith("/opt/ros/melodic"),
    )
    monkeypatch.setattr("subprocess.run", run)

    code, _stdout, _stderr = _default_text_runner(
        ["rosservice", "call", "/camera/get_serial"],
        5.0,
    )

    assert code == 0
    assert captured["env"]["ROS_PYTHON_VERSION"] == "2"
    assert "/opt/ros/melodic/lib/python2.7/dist-packages" in captured["env"]["PYTHONPATH"]
