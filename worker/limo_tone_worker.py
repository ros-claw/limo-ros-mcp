#!/usr/bin/env python2
"""Bounded daemon-owned ALSA worker for one LIMO speaker tone.

The worker accepts one exact JSON request on stdin and writes one JSON result
on stdout.  It exposes no arbitrary audio file, mixer control, command, or
device selection surface.
"""

import json
import math
import os
import re
import struct
import subprocess
import sys
import time
import wave

try:
    from cStringIO import StringIO
except ImportError:
    from io import BytesIO as StringIO

PROTOCOL = "rosclaw.limo.worker.v1"
SCHEMA = "limo.tone.v1"
MAX_REQUEST_BYTES = 65536
SAMPLE_RATE_HZ = 16000
MAX_AMPLITUDE = 0.25
ALLOWED_FREQUENCIES_HZ = (440, 660, 880)
ALLOWED_USB_CARD_NAMES = ("USB PnP Audio Device", "USB PnP Sound Device")
try:
    STRING_TYPES = (basestring,)  # type: ignore[name-defined]  # noqa: F821
except NameError:
    STRING_TYPES = (str,)


class RequestError(Exception):
    pass


def _finite(name, value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RequestError("%s must be a number" % name)
    result = float(value)
    if math.isnan(result) or math.isinf(result):
        raise RequestError("%s must be finite" % name)
    return result


def validate_request(raw):
    if not isinstance(raw, dict):
        raise RequestError("request must be an object")
    allowed = set(
        [
            "protocol",
            "operation",
            "schema_version",
            "action_id",
            "body_id",
            "body_snapshot_hash",
            "frequency_hz",
            "duration_sec",
            "volume_percent",
        ]
    )
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise RequestError("unknown request fields: %s" % ", ".join(unknown))
    if raw.get("protocol") != PROTOCOL:
        raise RequestError("unsupported worker protocol")
    if raw.get("operation") != "PLAY_TONE":
        raise RequestError("unsupported operation")
    if raw.get("schema_version") != SCHEMA:
        raise RequestError("unsupported tone schema")
    action_id = raw.get("action_id")
    body_hash = raw.get("body_snapshot_hash")
    if not isinstance(action_id, STRING_TYPES) or not action_id:
        raise RequestError("action_id is required")
    if not isinstance(body_hash, STRING_TYPES) or not body_hash:
        raise RequestError("body_snapshot_hash is required")
    frequency_hz = _finite("frequency_hz", raw.get("frequency_hz"))
    if int(frequency_hz) not in ALLOWED_FREQUENCIES_HZ or frequency_hz != int(frequency_hz):
        raise RequestError("frequency_hz must be one of 440, 660, or 880")
    duration_sec = _finite("duration_sec", raw.get("duration_sec"))
    if not 0.2 <= duration_sec <= 1.0:
        raise RequestError("duration_sec must be within [0.2, 1.0]")
    volume_percent = raw.get("volume_percent")
    if isinstance(volume_percent, bool) or not isinstance(volume_percent, int):
        raise RequestError("volume_percent must be an integer")
    if not 5 <= volume_percent <= 25:
        raise RequestError("volume_percent must be within [5, 25]")
    return {
        "protocol": PROTOCOL,
        "operation": "PLAY_TONE",
        "schema_version": SCHEMA,
        "action_id": action_id,
        "body_id": raw.get("body_id"),
        "body_snapshot_hash": body_hash,
        "frequency_hz": int(frequency_hz),
        "duration_sec": duration_sec,
        "volume_percent": volume_percent,
    }


def _usb_audio_card():
    try:
        with open("/proc/asound/cards", "r") as handle:
            cards = handle.read()
    except IOError as exc:
        raise RequestError("ALSA card inventory unavailable: %s" % exc)
    matches = []
    headers = list(re.finditer(r"^[ \t]*(\d+)[ \t]+\[", cards, flags=re.MULTILINE))
    for index, header in enumerate(headers):
        end = headers[index + 1].start() if index + 1 < len(headers) else len(cards)
        block = cards[header.start() : end]
        if "USB-Audio" not in block or not any(name in block for name in ALLOWED_USB_CARD_NAMES):
            continue
        matches.append(int(header.group(1)))
    if len(matches) != 1:
        raise RequestError("expected exactly one allowlisted USB PnP audio ALSA card")
    return matches[0]


def _run(command, input_bytes=None):
    process = subprocess.Popen(
        command,
        stdin=subprocess.PIPE if input_bytes is not None else None,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    stdout, stderr = process.communicate(input_bytes)
    return process.returncode, stdout, stderr


def _speaker_state(card):
    code, stdout, stderr = _run(["/usr/bin/amixer", "-c", str(card), "sget", "Speaker"])
    if code != 0:
        raise RequestError("cannot read Speaker mixer: %s" % stderr.strip())
    matches = re.findall(r"\[(\d+)%\].*?\[(on|off)\]", stdout)
    if not matches:
        raise RequestError("cannot parse Speaker mixer state")
    percent, switch = matches[-1]
    return int(percent), switch == "on"


def _set_speaker_state(card, percent, unmuted):
    switch = "unmute" if unmuted else "mute"
    code, _stdout, stderr = _run(
        ["/usr/bin/amixer", "-c", str(card), "sset", "Speaker", "%d%%" % percent, switch]
    )
    if code != 0:
        raise RequestError("cannot set Speaker mixer: %s" % stderr.strip())


def _tone_wav(frequency_hz, duration_sec):
    frame_count = int(round(SAMPLE_RATE_HZ * duration_sec))
    frames = []
    fade_frames = min(int(SAMPLE_RATE_HZ * 0.02), frame_count // 2)
    for index in range(frame_count):
        envelope = 1.0
        if fade_frames:
            if index < fade_frames:
                envelope = float(index) / fade_frames
            elif index >= frame_count - fade_frames:
                envelope = float(frame_count - index - 1) / fade_frames
        value = int(
            32767.0
            * MAX_AMPLITUDE
            * envelope
            * math.sin(2.0 * math.pi * frequency_hz * index / SAMPLE_RATE_HZ)
        )
        frames.append(struct.pack("<h", value))
    output = StringIO()
    writer = wave.open(output, "wb")
    writer.setnchannels(1)
    writer.setsampwidth(2)
    writer.setframerate(SAMPLE_RATE_HZ)
    writer.writeframes(b"".join(frames))
    writer.close()
    return output.getvalue(), frame_count


def _run_audio(request):
    card = _usb_audio_card()
    original_percent, original_unmuted = _speaker_state(card)
    wav_bytes, frame_count = _tone_wav(request["frequency_hz"], request["duration_sec"])
    started = time.time()
    restored = False
    try:
        _set_speaker_state(card, request["volume_percent"], True)
        code, _stdout, stderr = _run(
            ["/usr/bin/aplay", "-q", "-D", "plughw:%d,0" % card],
            input_bytes=wav_bytes,
        )
        if code != 0:
            raise RequestError("ALSA playback failed: %s" % stderr.strip())
    finally:
        _set_speaker_state(card, original_percent, original_unmuted)
        restored = True
    return {
        "protocol": PROTOCOL,
        "ok": True,
        "accepted": True,
        "action_id": request["action_id"],
        "operation": request["operation"],
        "device": "plughw:%d,0" % card,
        "alsa_card_index": card,
        "frequency_hz": request["frequency_hz"],
        "duration_sec": request["duration_sec"],
        "volume_percent": request["volume_percent"],
        "sample_rate_hz": SAMPLE_RATE_HZ,
        "frame_count": frame_count,
        "started_wall_time": started,
        "completed_wall_time": time.time(),
        "mixer_restored": restored,
        "original_speaker_state": {
            "volume_percent": original_percent,
            "unmuted": original_unmuted,
        },
    }


def main():
    raw_bytes = sys.stdin.read(MAX_REQUEST_BYTES + 1)
    if len(raw_bytes) > MAX_REQUEST_BYTES:
        raise RequestError("request exceeds byte limit")
    request = validate_request(json.loads(raw_bytes))
    if os.environ.get("ROSCLAW_LIMO_VALIDATE_ONLY") == "1":
        return {"protocol": PROTOCOL, "ok": True, "validated_request": request}
    return _run_audio(request)


if __name__ == "__main__":
    try:
        result = main()
        print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    except Exception as exc:
        print(
            json.dumps(
                {
                    "protocol": PROTOCOL,
                    "ok": False,
                    "error_code": "LIMO_TONE_WORKER_FAILED",
                    "error": str(exc),
                },
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        sys.exit(1)
