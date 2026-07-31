"""Bounded, read-only inspection of LIMO host peripherals."""

from __future__ import annotations

import array
import contextlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path
from typing import Any

import yaml

from limo_ros_mcp.roscli import build_ros1_cli_environment

TextRunner = Callable[[list[str], float], tuple[int, str, str]]
BinaryRunner = Callable[[list[str], float], tuple[int, bytes, str]]

USB_PERIPHERALS: tuple[dict[str, Any], ...] = (
    {
        "id": "dabai_color_camera",
        "name": "Orbbec Dabai RGB/IR camera",
        "category": "camera",
        "vendor_id": "2bc5",
        "product_id": "0557",
        "interfaces": [
            "color_image",
            "infrared_image",
            "color_camera_info",
            "infrared_camera_info",
        ],
    },
    {
        "id": "dabai_depth_sensor",
        "name": "Orbbec Dabai depth sensor",
        "category": "camera",
        "vendor_id": "2bc5",
        "product_id": "0657",
        "interfaces": ["depth_image", "depth_points", "depth_camera_info"],
    },
    {
        "id": "ydlidar_uart",
        "name": "EAI X2L lidar USB-UART",
        "category": "lidar",
        "vendor_id": "10c4",
        "product_id": "ea60",
        "interfaces": ["laser_scan"],
    },
    {
        "id": "voice_audio",
        "name": "LIMO USB microphone and stereo speakers",
        "category": "audio",
        "vendor_id": "0c76",
        "product_id": "161f",
        "interfaces": ["audio_state", "microphone_level"],
    },
    {
        "id": "rear_touchscreen",
        "name": "LIMO rear display touchscreen",
        "category": "display",
        "vendor_id": "1a86",
        "product_id": "e5e3",
        "interfaces": ["display_state"],
    },
    {
        "id": "bluetooth_adapter",
        "name": "LIMO Bluetooth adapter",
        "category": "connectivity",
        "vendor_id": "8087",
        "product_id": "0a2b",
        "interfaces": ["peripheral_inventory"],
    },
)

ROS_PERIPHERALS: tuple[dict[str, Any], ...] = (
    {
        "id": "chassis_controller",
        "name": "AgileX LIMO chassis controller",
        "category": "base",
        "topics": ["/limo_status", "/odom"],
        "interfaces": ["status", "odometry"],
    },
    {
        "id": "chassis_imu",
        "name": "LIMO chassis IMU",
        "category": "imu",
        "topics": ["/imu"],
        "interfaces": ["imu"],
    },
    {
        "id": "battery_monitor",
        "name": "LIMO battery voltage and error monitor",
        "category": "power",
        "topics": ["/limo_status"],
        "interfaces": ["status", "base_state"],
    },
)

DECLARED_UNBOUND_PERIPHERALS: tuple[dict[str, Any], ...] = (
    {
        "id": "front_oled",
        "name": "LIMO 128x64 front OLED",
        "category": "display",
        "reason": "Documented by AgileX, but no stable host or ROS interface was found.",
    },
    {
        "id": "chassis_rgb_lights",
        "name": "LIMO chassis RGB status lights",
        "category": "lighting",
        "reason": "Owned by chassis firmware; the upstream ROS driver exposes no light API.",
    },
)

DABAI_READ_ONLY_SERVICES: tuple[tuple[str, str], ...] = (
    ("device_info", "/camera/get_device_info"),
    ("device_type", "/camera/get_device_type"),
    ("serial", "/camera/get_serial"),
    ("version", "/camera/get_version"),
    ("ir_temperature_c", "/camera/get_ir_temperature"),
    ("ldp_enabled", "/camera/get_ldp_status"),
)


def _default_text_runner(command: list[str], timeout_sec: float) -> tuple[int, str, str]:
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
        timeout=timeout_sec,
        env=build_ros1_cli_environment(),
    )
    return completed.returncode, completed.stdout, completed.stderr


def _default_binary_runner(command: list[str], timeout_sec: float) -> tuple[int, bytes, str]:
    completed = subprocess.run(
        command,
        capture_output=True,
        check=False,
        timeout=timeout_sec,
    )
    return (
        completed.returncode,
        completed.stdout,
        completed.stderr.decode("utf-8", errors="replace"),
    )


class PeripheralInspector:
    """Inspect fixed, allowlisted host interfaces without changing their state."""

    def __init__(
        self,
        *,
        sys_root: Path = Path("/sys"),
        proc_root: Path = Path("/proc"),
        dev_root: Path = Path("/dev"),
        text_runner: TextRunner = _default_text_runner,
        binary_runner: BinaryRunner = _default_binary_runner,
    ) -> None:
        self.sys_root = sys_root
        self.proc_root = proc_root
        self.dev_root = dev_root
        self._text_runner = text_runner
        self._binary_runner = binary_runner

    @staticmethod
    def _read_text(path: Path) -> str:
        try:
            return path.read_text(encoding="utf-8").strip()
        except (OSError, UnicodeError):
            return ""

    def _usb_devices(self) -> list[dict[str, str]]:
        devices: list[dict[str, str]] = []
        root = self.sys_root / "bus" / "usb" / "devices"
        try:
            entries = sorted(root.iterdir(), key=lambda path: path.name)
        except OSError:
            return devices
        for entry in entries:
            vendor_id = self._read_text(entry / "idVendor").lower()
            product_id = self._read_text(entry / "idProduct").lower()
            if not vendor_id or not product_id:
                continue
            devices.append(
                {
                    "sysfs_name": entry.name,
                    "vendor_id": vendor_id,
                    "product_id": product_id,
                    "manufacturer": self._read_text(entry / "manufacturer"),
                    "product": self._read_text(entry / "product"),
                    "serial": self._read_text(entry / "serial"),
                }
            )
        return devices

    def _ros_topics(self) -> set[str]:
        try:
            code, stdout, _stderr = self._text_runner(["rostopic", "list"], 5.0)
        except (OSError, subprocess.SubprocessError):
            return set()
        if code != 0:
            return set()
        return {line.strip() for line in stdout.splitlines() if line.strip().startswith("/")}

    def inventory(self) -> dict[str, Any]:
        usb_devices = self._usb_devices()
        ros_topics = self._ros_topics()
        peripherals: list[dict[str, Any]] = []
        for spec in USB_PERIPHERALS:
            matches = [
                device
                for device in usb_devices
                if device["vendor_id"] == spec["vendor_id"]
                and device["product_id"] == spec["product_id"]
            ]
            peripherals.append(
                {
                    **spec,
                    "status": "connected" if matches else "not_detected",
                    "evidence": {"usb_devices": matches},
                    "mcp_bound": bool(matches),
                }
            )
        for spec in ROS_PERIPHERALS:
            required_topics = set(spec["topics"])
            present_topics = sorted(required_topics & ros_topics)
            connected = required_topics.issubset(ros_topics)
            peripherals.append(
                {
                    **spec,
                    "status": "connected" if connected else "not_detected",
                    "evidence": {
                        "present_topics": present_topics,
                        "missing_topics": sorted(required_topics - ros_topics),
                    },
                    "mcp_bound": connected,
                }
            )
        display = self.display_state()
        peripherals.append(
            {
                "id": "rear_display",
                "name": "LIMO 1024x600 rear display",
                "category": "display",
                "interfaces": ["display_state"],
                "status": "connected" if display["rear_display_ready"] else "not_detected",
                "evidence": {"framebuffers": display["framebuffers"]},
                "mcp_bound": bool(display["rear_display_ready"]),
            }
        )
        for spec in DECLARED_UNBOUND_PERIPHERALS:
            peripherals.append(
                {
                    **spec,
                    "status": "declared_unbound",
                    "interfaces": [],
                    "evidence": {"source": "AgileX LIMO user manual"},
                    "mcp_bound": False,
                }
            )
        return {
            "ok": True,
            "schema_version": "limo.peripherals.v1",
            "peripheral_count": len(peripherals),
            "connected_count": sum(item["status"] == "connected" for item in peripherals),
            "mcp_bound_count": sum(bool(item["mcp_bound"]) for item in peripherals),
            "declared_unbound_count": sum(
                item["status"] == "declared_unbound" for item in peripherals
            ),
            "peripherals": peripherals,
            "usb_device_count": len(usb_devices),
            "ros_topic_probe_available": bool(ros_topics),
            "trust_level": "LIVE_HOST_AND_ROS_OBSERVATION",
            "command_dispatched": False,
        }

    def _usb_audio_card(self) -> int | None:
        cards = self._read_text(self.proc_root / "asound" / "cards")
        for line in cards.splitlines():
            match = re.match(r"^\s*(\d+)\s+\[([^\]]+)\]:", line)
            if match and ("Device" in match.group(2) or "USB" in line):
                card = int(match.group(1))
                if 0 <= card <= 32:
                    return card
        return None

    def _mixer_state(self, card: int, control: str, direction: str) -> dict[str, Any]:
        try:
            code, stdout, stderr = self._text_runner(
                ["amixer", "-c", str(card), "sget", control], 3.0
            )
        except (OSError, subprocess.SubprocessError) as exc:
            return {"available": False, "error": str(exc)}
        if code != 0:
            return {"available": False, "error": stderr.strip() or stdout.strip()}
        pattern = rf"{direction}\s+\d+\s+\[(\d+)%\].*?\[(on|off)\]"
        matches = re.findall(pattern, stdout)
        return {
            "available": bool(matches),
            "volume_percent": (
                round(sum(int(volume) for volume, _switch in matches) / len(matches), 1)
                if matches
                else None
            ),
            "muted": all(switch == "off" for _volume, switch in matches) if matches else None,
            "channel_count": len(matches),
        }

    def audio_state(self) -> dict[str, Any]:
        card = self._usb_audio_card()
        if card is None:
            return {
                "ok": False,
                "error_code": "LIMO_AUDIO_DEVICE_UNAVAILABLE",
                "message": "The LIMO USB audio card is not present in /proc/asound/cards.",
                "command_dispatched": False,
            }
        pcm = self._read_text(self.proc_root / "asound" / "pcm")
        card_prefix = f"{card:02d}-"
        playback_ready = any(
            line.startswith(card_prefix) and "playback" in line for line in pcm.splitlines()
        )
        capture_ready = any(
            line.startswith(card_prefix) and "capture" in line for line in pcm.splitlines()
        )
        speaker = self._mixer_state(card, "Speaker", "Playback")
        microphone = self._mixer_state(card, "Mic", "Capture")
        return {
            "ok": playback_ready and capture_ready,
            "schema_version": "limo.audio-state.v1",
            "alsa_card": card,
            "alsa_device": f"plughw:{card},0",
            "playback_ready": playback_ready,
            "capture_ready": capture_ready,
            "speaker": speaker,
            "microphone": microphone,
            "physical_amplifier_interface": False,
            "amplifier_note": (
                "Only USB sound-card Speaker gain/mute is exposed; no independent amplifier "
                "power, temperature, or fault interface was detected."
            ),
            "trust_level": "LIVE_HOST_PERIPHERAL_OBSERVATION",
            "command_dispatched": False,
        }

    def microphone_level(
        self, *, duration_sec: int = 1, sample_rate_hz: int = 16000
    ) -> dict[str, Any]:
        if isinstance(duration_sec, bool) or not isinstance(duration_sec, int):
            raise ValueError("duration_sec must be an integer")
        if not 1 <= duration_sec <= 3:
            raise ValueError("duration_sec must be within [1, 3]")
        if sample_rate_hz not in {8000, 16000, 32000, 48000}:
            raise ValueError("sample_rate_hz must be one of 8000, 16000, 32000, 48000")
        card = self._usb_audio_card()
        if card is None:
            return {
                "ok": False,
                "error_code": "LIMO_AUDIO_DEVICE_UNAVAILABLE",
                "command_dispatched": False,
            }
        command = [
            "arecord",
            "-q",
            "-D",
            f"plughw:{card},0",
            "-f",
            "S16_LE",
            "-r",
            str(sample_rate_hz),
            "-c",
            "1",
            "-t",
            "raw",
            "-d",
            str(duration_sec),
        ]
        try:
            code, payload, stderr = self._binary_runner(command, float(duration_sec + 3))
        except (OSError, subprocess.SubprocessError) as exc:
            return {
                "ok": False,
                "error_code": "LIMO_MICROPHONE_CAPTURE_FAILED",
                "error": str(exc),
                "command_dispatched": False,
            }
        if code != 0:
            return {
                "ok": False,
                "error_code": "LIMO_MICROPHONE_CAPTURE_FAILED",
                "error": stderr.strip(),
                "command_dispatched": False,
            }
        samples = array.array("h")
        samples.frombytes(payload[: len(payload) - len(payload) % 2])
        if sys.byteorder != "little":
            samples.byteswap()
        if not samples:
            return {
                "ok": False,
                "error_code": "LIMO_MICROPHONE_EMPTY_CAPTURE",
                "command_dispatched": False,
            }
        peak = max(abs(int(value)) for value in samples)
        rms = math.sqrt(sum(int(value) ** 2 for value in samples) / len(samples))
        full_scale = 32768.0
        return {
            "ok": True,
            "schema_version": "limo.microphone-level.v1",
            "alsa_card": card,
            "duration_sec": duration_sec,
            "sample_rate_hz": sample_rate_hz,
            "sample_count": len(samples),
            "rms_dbfs": round(20.0 * math.log10(max(rms / full_scale, 1e-12)), 2),
            "peak_dbfs": round(20.0 * math.log10(max(peak / full_scale, 1e-12)), 2),
            "clipped_sample_count": sum(abs(int(value)) >= 32767 for value in samples),
            "audio_retained": False,
            "audio_content_returned": False,
            "privacy_note": "Samples are analyzed in memory and discarded.",
            "trust_level": "LIVE_AUDIO_SENSOR_OBSERVATION",
            "command_dispatched": False,
        }

    def display_state(self) -> dict[str, Any]:
        framebuffers: list[dict[str, Any]] = []
        graphics_root = self.sys_root / "class" / "graphics"
        for entry in sorted(graphics_root.glob("fb*"), key=lambda path: path.name):
            size = self._read_text(entry / "virtual_size")
            match = re.fullmatch(r"(\d+),(\d+)", size)
            width = int(match.group(1)) if match else 0
            height = int(match.group(2)) if match else 0
            framebuffers.append(
                {
                    "id": entry.name,
                    "name": self._read_text(entry / "name"),
                    "width": width,
                    "height": height,
                    "bits_per_pixel": self._integer_file(entry / "bits_per_pixel"),
                    "active": width > 0 and height > 0,
                }
            )
        input_devices: list[dict[str, Any]] = []
        input_root = self.sys_root / "class" / "input"
        for name_path in sorted(input_root.glob("input*/name"), key=lambda path: str(path)):
            name = self._read_text(name_path)
            if "CTP_CONTROL" not in name and "touch" not in name.casefold():
                continue
            device_root = name_path.parent
            input_devices.append(
                {
                    "id": device_root.name,
                    "name": name,
                    "vendor_id": self._read_text(device_root / "id" / "vendor"),
                    "product_id": self._read_text(device_root / "id" / "product"),
                }
            )
        rear_display_ready = any(
            item["active"] and item["width"] == 1024 and item["height"] == 600
            for item in framebuffers
        )
        return {
            "ok": bool(framebuffers or input_devices),
            "schema_version": "limo.display-state.v1",
            "framebuffers": framebuffers,
            "touchscreens": input_devices,
            "rear_display_ready": rear_display_ready and bool(input_devices),
            "front_oled": {
                "documented": True,
                "resolution": [128, 64],
                "interface_bound": False,
                "reason": "No stable host or ROS interface was found.",
            },
            "trust_level": "LIVE_HOST_PERIPHERAL_OBSERVATION",
            "command_dispatched": False,
        }

    def dabai_device_state(self) -> dict[str, Any]:
        """Call only fixed, read-only astra_camera getter services."""

        results: dict[str, Any] = {}
        failures: dict[str, str] = {}
        for name, service in DABAI_READ_ONLY_SERVICES:
            try:
                code, stdout, stderr = self._text_runner(["rosservice", "call", service], 5.0)
            except (OSError, subprocess.SubprocessError) as exc:
                failures[name] = str(exc)
                continue
            if code != 0:
                failures[name] = stderr.strip() or stdout.strip()
                continue
            try:
                payload = yaml.safe_load(stdout)
            except yaml.YAMLError as exc:
                failures[name] = f"invalid service YAML: {exc}"
                continue
            if not isinstance(payload, dict) or payload.get("success") is not True:
                failures[name] = str(
                    payload.get("message", "service did not report success")
                    if isinstance(payload, dict)
                    else "service returned no object"
                )
                continue
            value = payload.get("info") if name == "device_info" else payload.get("data")
            if name == "version" and isinstance(value, str):
                with contextlib.suppress(json.JSONDecodeError):
                    value = json.loads(value)
            results[name] = value
        return {
            "ok": not failures,
            "schema_version": "limo.dabai-device-state.v1",
            "device": results,
            "failures": failures,
            "service_count": len(DABAI_READ_ONLY_SERVICES),
            "successful_service_count": len(results),
            "read_only_services": [service for _name, service in DABAI_READ_ONLY_SERVICES],
            "trust_level": "LIVE_ROS_SERVICE_OBSERVATION" if results else "UNAVAILABLE",
            "command_dispatched": False,
        }

    def platform_health(self) -> dict[str, Any]:
        thermal_zones: list[dict[str, Any]] = []
        thermal_root = self.sys_root / "class" / "thermal"
        for zone in sorted(thermal_root.glob("thermal_zone*"), key=lambda path: path.name):
            raw_temp = self._integer_file(zone / "temp")
            thermal_zones.append(
                {
                    "id": zone.name,
                    "name": self._read_text(zone / "type"),
                    "temperature_c": round(raw_temp / 1000.0, 2) if raw_temp is not None else None,
                }
            )
        meminfo: dict[str, int] = {}
        for line in self._read_text(self.proc_root / "meminfo").splitlines():
            match = re.match(r"^([A-Za-z_()]+):\s+(\d+)\s+kB$", line)
            if match:
                meminfo[match.group(1)] = int(match.group(2)) * 1024
        uptime_raw = self._read_text(self.proc_root / "uptime").split()
        try:
            uptime_sec = float(uptime_raw[0])
        except (IndexError, ValueError):
            uptime_sec = None
        disk = shutil.disk_usage("/")
        temperatures = [
            float(item["temperature_c"])
            for item in thermal_zones
            if isinstance(item["temperature_c"], (int, float))
        ]
        return {
            "ok": bool(thermal_zones),
            "schema_version": "limo.platform-health.v1",
            "thermal_zones": thermal_zones,
            "maximum_temperature_c": max(temperatures, default=None),
            "memory": {
                "total_bytes": meminfo.get("MemTotal"),
                "available_bytes": meminfo.get("MemAvailable"),
                "swap_total_bytes": meminfo.get("SwapTotal"),
                "swap_free_bytes": meminfo.get("SwapFree"),
            },
            "disk": {
                "total_bytes": disk.total,
                "used_bytes": disk.used,
                "free_bytes": disk.free,
                "used_ratio": round(disk.used / disk.total, 4) if disk.total else None,
            },
            "load_average": list(os.getloadavg()),
            "uptime_sec": uptime_sec,
            "trust_level": "LIVE_HOST_HEALTH_OBSERVATION",
            "command_dispatched": False,
        }

    def _integer_file(self, path: Path) -> int | None:
        value = self._read_text(path)
        try:
            return int(value)
        except ValueError:
            return None
