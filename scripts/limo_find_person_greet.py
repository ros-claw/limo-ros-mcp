#!/usr/bin/env python3
"""Find a person, approach conservatively, greet, record, and transcribe a reply.

All physical actions go through the LIMO MCP and rosclawd.  The application is
interactive by design: every exact REAL action is shown by MCP elicitation and
must be accepted by the operator in the same terminal.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import hashlib
import json
import math
import os
import re
import select
import statistics
import subprocess
import sys
import time
import urllib.parse
import urllib.request
import uuid
import wave
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

from mcp import ClientSession, StdioServerParameters, types
from mcp.client.stdio import stdio_client

from limo_ros_mcp.roscli import build_ros1_cli_environment

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE_ROOT = ROOT.parent / "evidence" / "person_interaction"
BODY_REF = Path.home() / ".rosclaw/bodies/limo/refs/effective_body.json"
QWEN_ENDPOINT = "http://10.10.217.108:30521/v1/chat/completions"
QWEN_MODEL = "qwen3.6-27b"
WHISPER_ROOT = Path.home() / ".cache/rosclaw-limo-asr/whisper.cpp-v1.7.3"
WHISPER_CLI = WHISPER_ROOT / "build/bin/main"
WHISPER_MODEL = WHISPER_ROOT / "models/ggml-tiny.bin"
CAMERA_KEEPALIVE = ROOT / "src/limo_ros_mcp/workers/limo_camera_keepalive_worker.py"
ACTION_CONFIRMATION_WINDOW_SEC = 15 * 60
PULSE_SERVER = "unix:/run/rosclaw/pulse/native"
PULSE_SOURCE_PATTERN = re.compile(
    r"^alsa_input\.usb-[A-Za-z0-9_.:-]*USB_PnP_(?:Audio|Sound)_Device"
    r"[A-Za-z0-9_.:-]*\.analog-stereo$"
)
RESPONSE_SAMPLE_RATE_HZ = 16000
RESPONSE_DURATION_SEC = 8.0
RESPONSE_MIN_DURATION_RATIO = 0.75
RESPONSE_CAPTURE_ATTEMPTS = 2


def action_deadline(*, now: datetime | None = None) -> str:
    """Return a deadline long enough for confirmation across chat turns."""

    reference = now or datetime.now(UTC)
    if reference.tzinfo is None:
        raise ValueError("action deadline reference must be timezone-aware")
    return (
        (reference.astimezone(UTC) + timedelta(seconds=ACTION_CONFIRMATION_WINDOW_SEC))
        .isoformat()
        .replace("+00:00", "Z")
    )


def angle_normalize(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def angle_distance(first: float, second: float) -> float:
    return abs(angle_normalize(first - second))


def body_snapshot_hash() -> str:
    data = json.loads(BODY_REF.read_text(encoding="utf-8"))
    digest = normalize_body_snapshot_hash(str(data["effective_body_hash"]))
    return digest


def normalize_body_snapshot_hash(value: str) -> str:
    """Match the raw 64-hex RobotInstance body_snapshot_hash contract."""

    digest = value.removeprefix("sha256:")
    if not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise RuntimeError("effective LIMO body hash is invalid")
    return digest


def parse_json_object(text: str) -> dict[str, Any]:
    candidate = text.strip()
    fenced = re.search(r"```(?:json)?\s*(\{.*\})\s*```", candidate, re.DOTALL)
    if fenced:
        candidate = fenced.group(1)
    else:
        start, end = candidate.find("{"), candidate.rfind("}")
        if start < 0 or end <= start:
            raise ValueError("vision model returned no JSON object")
        candidate = candidate[start : end + 1]
    result = json.loads(candidate)
    if not isinstance(result, dict):
        raise ValueError("vision response must be an object")
    return result


def validate_detection(raw: dict[str, Any]) -> dict[str, Any]:
    present = raw.get("person_present") is True
    person = raw.get("selected_person")
    if not present:
        return {**raw, "person_present": False, "selected_person": None}
    if not isinstance(person, dict):
        raise ValueError("person_present requires selected_person")
    bbox = person.get("bbox_norm")
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise ValueError("selected person bbox is invalid")
    coords = [float(value) for value in bbox]
    if not all(0.0 <= value <= 1.0 for value in coords):
        raise ValueError("selected person bbox is outside the image")
    if coords[0] >= coords[2] or coords[1] >= coords[3]:
        raise ValueError("selected person bbox has invalid ordering")
    confidence = float(person.get("confidence", 0.0))
    if confidence < 0.65:
        return {**raw, "person_present": False, "selected_person": None}
    person["bbox_norm"] = coords
    person["confidence"] = confidence
    return {**raw, "person_present": True, "selected_person": person}


def analyze_wav(path: Path) -> dict[str, Any]:
    with wave.open(str(path), "rb") as reader:
        if reader.getnchannels() != 1 or reader.getsampwidth() != 2:
            raise RuntimeError("response WAV must be mono 16-bit PCM")
        rate = reader.getframerate()
        frames = reader.readframes(reader.getnframes())
    import array

    samples = array.array("h")
    samples.frombytes(frames)
    if sys.byteorder != "little":
        samples.byteswap()
    if not samples:
        raise RuntimeError("response WAV has no samples")
    rms = math.sqrt(statistics.fmean(float(value) * value for value in samples))
    peak = max(abs(value) for value in samples)

    def dbfs(value: float) -> float:
        return -120.0 if value <= 0 else 20.0 * math.log10(value / 32768.0)

    return {
        "sample_rate_hz": rate,
        "sample_count": len(samples),
        "duration_sec": round(len(samples) / float(rate), 3),
        "rms_dbfs": round(dbfs(rms), 2),
        "peak_dbfs": round(dbfs(float(peak)), 2),
        "sha256": f"sha256:{hashlib.sha256(path.read_bytes()).hexdigest()}",
        "speech_energy_detected": dbfs(rms) >= -48.0 and dbfs(float(peak)) >= -32.0,
    }


def parse_pulse_source_inventory(inventory: str) -> str:
    """Select exactly one fixed USB microphone from a pactl inventory."""

    matches = []
    for line in inventory.splitlines():
        fields = line.split()
        if len(fields) >= 2 and PULSE_SOURCE_PATTERN.fullmatch(fields[1]):
            matches.append(fields[1])
    if len(matches) != 1:
        raise RuntimeError("expected exactly one allowlisted USB PnP PulseAudio source")
    return matches[0]


def pulse_response_source() -> str:
    listed = subprocess.run(
        ["/usr/bin/pactl", "--server", PULSE_SERVER, "list", "short", "sources"],
        check=False,
        capture_output=True,
        text=True,
        timeout=5,
    )
    if listed.returncode != 0:
        raise RuntimeError("cannot list the fixed PulseAudio microphone source")
    return parse_pulse_source_inventory(listed.stdout)


def _capture_pulse_pcm(source: str, duration_sec: float) -> bytes:
    """Drain parec while it records so its stdout pipe cannot stall the capture."""

    recorder = subprocess.Popen(
        [
            "/usr/bin/parec",
            "--server",
            PULSE_SERVER,
            "--device",
            source,
            "--client-name",
            "rosclaw-limo-person-mission",
            "--stream-name",
            "bounded-response-capture",
            "--format=s16le",
            f"--rate={RESPONSE_SAMPLE_RATE_HZ}",
            "--channels=1",
            "--latency-msec=20",
            "--process-time-msec=20",
            "--raw",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        stdout, stderr = recorder.communicate(timeout=duration_sec)
    except subprocess.TimeoutExpired:
        recorder.terminate()
        try:
            stdout, stderr = recorder.communicate(timeout=2)
        except subprocess.TimeoutExpired:
            recorder.kill()
            stdout, stderr = recorder.communicate(timeout=2)
    if recorder.returncode not in {0, -15}:
        detail = stderr.decode("utf-8", "replace").strip()
        raise RuntimeError(f"PulseAudio response capture failed: {detail or 'unknown error'}")
    return stdout


def record_pulse_response(path: Path, duration_sec: float = RESPONSE_DURATION_SEC) -> None:
    """Retain a complete bounded mono WAV, retrying one truncated Pulse capture."""

    if not 1.0 <= duration_sec <= RESPONSE_DURATION_SEC:
        raise ValueError(f"duration_sec must be within [1.0, {RESPONSE_DURATION_SEC}]")
    source = pulse_response_source()
    expected_bytes = int(RESPONSE_SAMPLE_RATE_HZ * 2 * duration_sec)
    minimum_bytes = int(expected_bytes * RESPONSE_MIN_DURATION_RATIO)
    received = 0
    for _attempt in range(RESPONSE_CAPTURE_ATTEMPTS):
        pcm = _capture_pulse_pcm(source, duration_sec)
        received = len(pcm)
        if received < minimum_bytes:
            continue
        with wave.open(str(path), "wb") as writer:
            writer.setnchannels(1)
            writer.setsampwidth(2)
            writer.setframerate(RESPONSE_SAMPLE_RATE_HZ)
            writer.writeframes(pcm[:expected_bytes])
        return
    received_sec = received / float(RESPONSE_SAMPLE_RATE_HZ * 2)
    raise RuntimeError(
        f"PulseAudio response capture was truncated ({received_sec:.2f}s after "
        f"{RESPONSE_CAPTURE_ATTEMPTS} attempts)"
    )


def transcribe_local(path: Path) -> dict[str, Any]:
    if not WHISPER_CLI.is_file() or not WHISPER_MODEL.is_file():
        return {"ok": False, "error": "offline whisper runtime is unavailable"}
    started = time.monotonic()
    result = subprocess.run(
        [
            str(WHISPER_CLI),
            "-m",
            str(WHISPER_MODEL),
            "-f",
            str(path),
            "-l",
            "zh",
            "-nt",
            "-np",
            "-t",
            "4",
        ],
        check=False,
        capture_output=True,
        text=True,
        timeout=90,
    )
    transcript = result.stdout.strip()
    return {
        "ok": result.returncode == 0 and bool(transcript),
        "transcript": transcript or None,
        "elapsed_sec": round(time.monotonic() - started, 3),
        "engine": "whisper.cpp-tiny-zh",
        "error": None if result.returncode == 0 else result.stderr.strip()[-500:],
    }


def transcribe_google(path: Path) -> dict[str, Any]:
    key = os.environ.get("ROSCLAW_LIMO_GOOGLE_SPEECH_API_KEY", "").strip()
    if not key:
        return {
            "ok": False,
            "error": "ROSCLAW_LIMO_GOOGLE_SPEECH_API_KEY is not configured",
            "engine": "google-speech-zh-CN",
        }
    flac = path.with_suffix(".flac")
    converted = subprocess.run(
        [
            "/usr/bin/ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-i",
            str(path),
            "-ar",
            "16000",
            "-ac",
            "1",
            "-sample_fmt",
            "s16",
            str(flac),
        ],
        check=False,
        capture_output=True,
        timeout=20,
    )
    if converted.returncode != 0:
        return {"ok": False, "error": "ffmpeg FLAC conversion failed"}
    url = "https://www.google.com/speech-api/v2/recognize?" + urllib.parse.urlencode(
        {"client": "chromium", "lang": "zh-CN", "key": key}
    )
    request = urllib.request.Request(
        url,
        data=flac.read_bytes(),
        headers={"Content-Type": "audio/x-flac; rate=16000"},
    )
    started = time.monotonic()
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            body = response.read().decode("utf-8")
        candidates: list[dict[str, Any]] = []
        for line in body.splitlines():
            values = json.loads(line).get("result", [])
            if values:
                candidates = list(values[0].get("alternative", []))
                break
        transcript = str(candidates[0].get("transcript", "")).strip() if candidates else ""
        return {
            "ok": bool(transcript),
            "transcript": transcript or None,
            "alternatives": candidates[:3],
            "elapsed_sec": round(time.monotonic() - started, 3),
            "engine": "google-speech-zh-CN",
            "privacy": "response audio was sent to Google Speech Recognition",
        }
    except Exception as exc:  # noqa: BLE001 - optional network recognizer
        return {"ok": False, "error": str(exc), "engine": "google-speech-zh-CN"}


def start_camera_keepalive() -> subprocess.Popen[str]:
    process = subprocess.Popen(
        ["/usr/bin/python2", str(CAMERA_KEEPALIVE)],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        env=build_ros1_cli_environment(),
    )
    if process.stdout is None:
        raise RuntimeError("camera keepalive stdout is unavailable")
    readable, _, _ = select.select([process.stdout], [], [], 15.0)
    if not readable:
        process.terminate()
        process.wait(timeout=3)
        raise RuntimeError("camera keepalive did not become ready")
    line = process.stdout.readline()
    try:
        result = json.loads(line)
    except json.JSONDecodeError as exc:
        process.terminate()
        process.wait(timeout=3)
        raise RuntimeError("camera keepalive returned invalid JSON") from exc
    if (
        process.poll() is not None
        or result.get("ok") is not True
        or result.get("protocol") != "rosclaw.limo.camera-keepalive-worker.v1"
    ):
        raise RuntimeError(f"camera keepalive failed: {result}")
    return process


class Mission:
    def __init__(
        self, session: ClientSession, run_dir: Path, execute: bool, cloud_asr: bool
    ) -> None:
        self.session = session
        self.run_dir = run_dir
        self.execute = execute
        self.cloud_asr = cloud_asr
        self.body_hash = body_snapshot_hash()
        self.event_log: list[dict[str, Any]] = []

    async def call(
        self, name: str, arguments: dict[str, Any]
    ) -> tuple[dict[str, Any], bytes | None]:
        result = await self.session.call_tool(name, arguments)
        payload: dict[str, Any] | None = None
        image: bytes | None = None
        for block in result.content:
            if block.type == "text":
                payload = json.loads(block.text)
            elif block.type == "image":
                image = base64.b64decode(block.data, validate=True)
        if payload is None:
            raise RuntimeError(f"{name} returned no JSON payload")
        self.event_log.append({"tool": name, "arguments": arguments, "result": payload})
        return payload, image

    async def frame_and_detection(self, label: str) -> dict[str, Any]:
        metadata, image = await self.call(
            "limo_capture_camera_frame",
            {"stream": "color", "timeout_sec": 5.0, "max_dimension": 640},
        )
        if metadata.get("ok") is not True or image is None:
            raise RuntimeError(f"camera capture failed: {metadata}")
        frame_path = self.run_dir / f"{label}.png"
        frame_path.write_bytes(image)
        body = {
            "model": QWEN_MODEL,
            "messages": [
                {
                    "role": "system",
                    "content": "You are a mobile robot visual safety detector. Return JSON only.",
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": (
                                "Find real people, including a seated person. Never count chairs, "
                                "screens, posters, or reflections. Return exactly: "
                                '{"person_present":boolean,"people_count":integer,'
                                '"selected_person":null or {"bbox_norm":[x1,y1,x2,y2],'
                                '"horizontal":"left|center|right","seated":boolean,'
                                '"confidence":number},"scene":string,"hazards":[string]}.'
                            ),
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": "data:image/png;base64," + base64.b64encode(image).decode()
                            },
                        },
                    ],
                },
            ],
            "temperature": 0.0,
            "max_tokens": 384,
            "chat_template_kwargs": {"enable_thinking": False},
        }
        request = urllib.request.Request(
            QWEN_ENDPOINT,
            data=json.dumps(body).encode(),
            headers={"Content-Type": "application/json"},
        )
        opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        with opener.open(request, timeout=60) as response:
            model_result = json.load(response)
        detection = validate_detection(
            parse_json_object(model_result["choices"][0]["message"]["content"])
        )
        record = {"frame": frame_path.name, "camera": metadata, "detection": detection}
        (self.run_dir / f"{label}.json").write_text(
            json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return detection

    async def pose(self) -> dict[str, float]:
        payload, _ = await self.call("limo_get_robot_pose", {"timeout_sec": 5.0})
        if payload.get("ok") is not True:
            raise RuntimeError(f"robot pose unavailable: {payload}")
        return {
            "x": float(payload["translation"]["x"]),
            "y": float(payload["translation"]["y"]),
            "yaw": float(payload["rotation"]["yaw_rad"]),
        }

    async def readiness(self) -> dict[str, Any]:
        payload, _ = await self.call(
            "limo_get_readiness",
            {
                "transport": "rosbridge",
                "timeout_sec": 8.0,
                "body_id": "limo",
                "body_snapshot_hash": self.body_hash,
            },
        )
        readiness = payload.get("readiness", {})
        if readiness.get("state") == "BLOCKED" or readiness.get("blockers"):
            raise RuntimeError(f"motion readiness is blocked: {readiness.get('blockers')}")
        return readiness

    async def navigate(self, target: dict[str, float], label: str) -> None:
        before = await self.pose()
        readiness = await self.readiness()
        payload, _ = await self.call(
            "limo_request_navigation",
            {
                "x": target["x"],
                "y": target["y"],
                "yaw": angle_normalize(target["yaw"]),
                "body_snapshot_hash": self.body_hash,
                "readiness_snapshot_hash": readiness["snapshot_hash"],
                "execution_mode": "REAL",
                "action_id": f"action_person_mission_{label}_{uuid.uuid4().hex}",
                "deadline_at": action_deadline(),
                "wait_timeout_sec": 30.0,
            },
        )
        receipt = payload.get("receipt")
        if (
            payload.get("state") != "FINISHED"
            or not isinstance(receipt, dict)
            or receipt.get("final_state") != "COMPLETED"
            or receipt.get("usable_for_real_execution") is not True
        ):
            raise RuntimeError(f"navigation did not finish: {payload}")
        after = await self.pose()
        linear = math.hypot(after["x"] - before["x"], after["y"] - before["y"])
        angular = angle_distance(after["yaw"], before["yaw"])
        if linear < 0.05 and angular < 0.08:
            raise RuntimeError("navigation receipt returned but TF proves no physical motion")

    async def speak(self, text: str, label: str) -> None:
        payload, _ = await self.call(
            "limo_request_speech",
            {
                "text": text,
                "language": "cmn",
                "volume_percent": 22,
                "rate_wpm": 145,
                "body_snapshot_hash": self.body_hash,
                "execution_mode": "REAL",
                "action_id": f"action_person_speech_{label}_{uuid.uuid4().hex}",
                "deadline_at": action_deadline(),
                "wait_timeout_sec": 30.0,
            },
        )
        receipt = payload.get("receipt")
        if (
            payload.get("state") != "FINISHED"
            or not isinstance(receipt, dict)
            or receipt.get("final_state") != "COMPLETED"
            or receipt.get("usable_for_real_execution") is not True
        ):
            raise RuntimeError(f"speech did not finish: {payload}")

    async def record_response(self, attempt: int) -> dict[str, Any]:
        audio, _ = await self.call("limo_get_audio_state", {})
        if audio.get("capture_ready") is not True:
            raise RuntimeError("microphone is not capture-ready")
        await asyncio.sleep(0.8)
        wav_path = self.run_dir / f"response_{attempt}.wav"
        await asyncio.to_thread(record_pulse_response, wav_path)
        metrics = analyze_wav(wav_path)
        local = await asyncio.to_thread(transcribe_local, wav_path)
        google = (
            await asyncio.to_thread(transcribe_google, wav_path)
            if self.cloud_asr
            else {
                "ok": False,
                "disabled": True,
                "privacy": "cloud ASR requires the explicit --cloud-asr flag",
            }
        )
        result = {"wav": wav_path.name, "metrics": metrics, "local": local, "google": google}
        (self.run_dir / f"response_{attempt}.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        return result

    async def run(self) -> dict[str, Any]:
        runtime, _ = await self.call("limo_get_runtime_status", {})
        audio, _ = await self.call("limo_get_audio_state", {})
        pose = await self.pose()
        readiness = await self.readiness()
        detection = await self.frame_and_detection("search_00")
        preflight = {
            "runtime_running": runtime.get("running"),
            "supervision_state": runtime.get("supervision_state"),
            "audio_capture_ready": audio.get("capture_ready"),
            "audio_playback_ready": audio.get("playback_ready"),
            "pose": pose,
            "readiness_state": readiness.get("state"),
            "initial_detection": detection,
            "execute": self.execute,
        }
        (self.run_dir / "preflight.json").write_text(
            json.dumps(preflight, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        if not all(
            [
                preflight["runtime_running"],
                preflight["supervision_state"] == "ARMED",
                preflight["audio_capture_ready"],
                preflight["audio_playback_ready"],
            ]
        ):
            raise RuntimeError(f"mission preflight failed: {preflight}")
        if not self.execute:
            return {"state": "PREFLIGHT_ONLY", **preflight}

        for index in range(1, 9):
            if detection.get("person_present"):
                break
            current = await self.pose()
            await self.navigate(
                {"x": current["x"], "y": current["y"], "yaw": current["yaw"] + math.pi / 4},
                f"search_{index:02d}",
            )
            detection = await self.frame_and_detection(f"search_{index:02d}")
        if not detection.get("person_present"):
            raise RuntimeError("no person found after one bounded 360-degree search")

        for index in range(1, 5):
            person = detection["selected_person"]
            horizontal = person.get("horizontal")
            if horizontal in {"left", "right"}:
                current = await self.pose()
                offset = 0.25 if horizontal == "left" else -0.25
                await self.navigate(
                    {"x": current["x"], "y": current["y"], "yaw": current["yaw"] + offset},
                    f"center_{index:02d}",
                )
                detection = await self.frame_and_detection(f"center_{index:02d}")
                continue
            bbox = person["bbox_norm"]
            height = float(bbox[3]) - float(bbox[1])
            laser, _ = await self.call(
                "limo_get_laser_summary", {"transport": "rosbridge", "timeout_sec": 5.0}
            )
            front = float(laser["summaries"]["laser_scan"]["sectors"]["front_min_m"])
            if height >= 0.42 or front <= 1.35:
                break
            current = await self.pose()
            step = min(0.30, max(0.0, front - 1.25))
            if step < 0.10:
                break
            await self.navigate(
                {
                    "x": current["x"] + step * math.cos(current["yaw"]),
                    "y": current["y"] + step * math.sin(current["yaw"]),
                    "yaw": current["yaw"],
                },
                f"approach_{index:02d}",
            )
            detection = await self.frame_and_detection(f"approach_{index:02d}")
            if not detection.get("person_present"):
                raise RuntimeError("person was lost during approach; robot stopped")

        await self.speak("你好！我是 LIMO。你吃饭了吗？", "greeting")
        response = await self.record_response(1)
        if not response["metrics"]["speech_energy_detected"] or not (
            response["local"].get("ok") or response["google"].get("ok")
        ):
            await self.speak("不好意思，我没听清。请再说一遍。", "retry")
            response = await self.record_response(2)
        summary = {
            "state": "COMPLETED",
            "final_detection": detection,
            "response": response,
            "event_count": len(self.event_log),
        }
        (self.run_dir / "mission.json").write_text(
            json.dumps({**summary, "events": self.event_log}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return summary


async def main_async(args: argparse.Namespace) -> int:
    run_dir = EVIDENCE_ROOT / time.strftime("%Y%m%d-%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=False)

    async def confirm(_context: Any, params: Any) -> types.ElicitResult:
        if not args.execute:
            return types.ElicitResult(action="decline")
        print("\nROSClaw 精确动作确认：", file=sys.stderr)
        print(str(params.message), file=sys.stderr)
        answer = await asyncio.to_thread(input, "输入 CONFIRM 执行这个动作，否则停止：")
        return types.ElicitResult(
            action="accept" if answer.strip() == "CONFIRM" else "decline",
            content={} if answer.strip() == "CONFIRM" else None,
        )

    keepalive = await asyncio.to_thread(start_camera_keepalive)
    try:
        parameters = StdioServerParameters(
            command=sys.executable,
            args=["-m", "limo_ros_mcp.server", "--profile", "full"],
            env={**os.environ, "PYTHONPATH": str(ROOT / "src")},
        )
        async with asyncio.timeout(timedelta(hours=2).total_seconds()):
            async with stdio_client(parameters) as (read, write):
                async with ClientSession(read, write, elicitation_callback=confirm) as session:
                    await session.initialize()
                    result = await Mission(session, run_dir, args.execute, args.cloud_asr).run()
    finally:
        keepalive.terminate()
        try:
            keepalive.wait(timeout=3)
        except subprocess.TimeoutExpired:
            keepalive.kill()
            keepalive.wait(timeout=3)
    print(json.dumps({"run_dir": str(run_dir), "result": result}, ensure_ascii=False, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--execute",
        action="store_true",
        help="allow exact, interactively confirmed REAL actions; default is read-only preflight",
    )
    parser.add_argument(
        "--cloud-asr",
        action="store_true",
        help="send retained response audio to Google Speech Recognition for a second transcript",
    )
    return asyncio.run(main_async(parser.parse_args()))


if __name__ == "__main__":
    raise SystemExit(main())
