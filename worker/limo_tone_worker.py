#!/usr/bin/env python2
"""Bounded daemon-owned audio worker for one LIMO speaker tone.

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
# The requested volume is applied once to the synthesized PCM.  The selected
# output is temporarily placed at a reference gain and then restored exactly.
MAX_AMPLITUDE = 0.9
PULSE_REFERENCE_VOLUME = 65536
ALLOWED_FREQUENCIES_HZ = (440, 660, 880)
ALLOWED_USB_CARD_NAMES = ("USB PnP Audio Device", "USB PnP Sound Device")
BASELINE_CAPTURE_SEC = 1
LOOPBACK_CAPTURE_SEC = 2
LOOPBACK_PREROLL_SEC = 0.15
CAPTURE_BUSY_RETRIES = 3
CAPTURE_BUSY_RETRY_DELAY_SEC = 0.2
CAPTURE_START_SETTLE_SEC = 0.05
LOOPBACK_MIN_TARGET_DBFS = -45.0
LOOPBACK_MIN_GAIN_DB = 10.0
LOOPBACK_MIN_PROMINENCE_DB = 8.0
PULSE_SINK_PATTERN = re.compile(
    r"^alsa_output\.usb-[A-Za-z0-9_.:-]*USB_PnP_(?:Audio|Sound)_Device"
    r"[A-Za-z0-9_.:-]*\.analog-stereo$"
)
PULSE_SOURCE_PATTERN = re.compile(
    r"^alsa_input\.usb-[A-Za-z0-9_.:-]*USB_PnP_(?:Audio|Sound)_Device"
    r"[A-Za-z0-9_.:-]*\.analog-stereo$"
)
ALLOWED_PULSE_SERVER = "unix:/run/rosclaw/pulse/native"
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


def _run(command, input_bytes=None, env=None):
    process = subprocess.Popen(
        command,
        stdin=subprocess.PIPE if input_bytes is not None else None,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        env=env,
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


def _set_speaker_state(card, percent, unmuted, mapped=False):
    switch = "unmute" if unmuted else "mute"
    command = ["/usr/bin/amixer"]
    if mapped:
        command.append("-M")
    command.extend(["-c", str(card), "sset", "Speaker", "%d%%" % percent, switch])
    code, _stdout, stderr = _run(command)
    if code != 0:
        raise RequestError("cannot set Speaker mixer: %s" % stderr.strip())


def _tone_wav(frequency_hz, duration_sec, volume_percent):
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
            * volume_percent
            / 100.0
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


def _pulse_server():
    configured = os.environ.get("ROSCLAW_LIMO_PULSE_SERVER")
    if configured:
        if configured != ALLOWED_PULSE_SERVER:
            raise RequestError("configured PulseAudio server is not allowlisted")
        if not os.path.exists(configured[len("unix:") :]):
            raise RequestError("configured PulseAudio server is unavailable")
        return configured
    runtime_dir = "/run/user/%d" % os.getuid()
    socket_path = os.path.join(runtime_dir, "pulse", "native")
    if os.path.exists(socket_path):
        return "unix:%s" % socket_path
    return None


def _pulse_sink(server):
    code, stdout, stderr = _run(["/usr/bin/pactl", "--server", server, "list", "short", "sinks"])
    if code != 0:
        raise RequestError("cannot list PulseAudio sinks: %s" % stderr.strip())
    if not isinstance(stdout, str):
        stdout = stdout.decode("utf-8", "replace")
    matches = []
    for line in stdout.splitlines():
        fields = line.split()
        if len(fields) >= 2 and PULSE_SINK_PATTERN.match(fields[1]):
            matches.append(fields[1])
    if len(matches) != 1:
        raise RequestError("expected exactly one allowlisted USB PnP PulseAudio sink")
    return matches[0]


def _pulse_source(server):
    code, stdout, stderr = _run(
        ["/usr/bin/pactl", "--server", server, "list", "short", "sources"]
    )
    if code != 0:
        raise RequestError("cannot list PulseAudio sources: %s" % stderr.strip())
    if not isinstance(stdout, str):
        stdout = stdout.decode("utf-8", "replace")
    matches = []
    for line in stdout.splitlines():
        fields = line.split()
        if len(fields) >= 2 and PULSE_SOURCE_PATTERN.match(fields[1]):
            matches.append(fields[1])
    if len(matches) != 1:
        raise RequestError("expected exactly one allowlisted USB PnP PulseAudio source")
    return matches[0]


def _pulse_sink_state(server, sink):
    code, stdout, stderr = _run(["/usr/bin/pactl", "--server", server, "list", "sinks"])
    if code != 0:
        raise RequestError("cannot read PulseAudio sink state: %s" % stderr.strip())
    if not isinstance(stdout, str):
        stdout = stdout.decode("utf-8", "replace")
    for block in re.split(r"(?m)^Sink #\d+\s*$", stdout):
        name = re.search(r"(?m)^\s*Name:\s*(\S+)\s*$", block)
        if name is None or name.group(1) != sink:
            continue
        mute = re.search(r"(?m)^\s*Mute:\s*(yes|no)\s*$", block)
        volume = re.search(r"(?m)^\s*Volume:\s*(.+)$", block)
        if mute is None or volume is None:
            raise RequestError("cannot parse PulseAudio sink state")
        channel_volumes = [int(value) for value in re.findall(r":\s*(\d+)\s*/", volume.group(1))]
        if not channel_volumes:
            raise RequestError("cannot parse PulseAudio sink channel volumes")
        return {
            "channel_volumes": channel_volumes,
            "unmuted": mute.group(1) == "no",
        }
    raise RequestError("allowlisted PulseAudio sink state is unavailable")


def _set_pulse_sink_state(server, sink, channel_volumes, unmuted):
    volume_command = [
        "/usr/bin/pactl",
        "--server",
        server,
        "set-sink-volume",
        sink,
    ] + [str(value) for value in channel_volumes]
    code, _stdout, stderr = _run(volume_command)
    if code != 0:
        raise RequestError("cannot set PulseAudio sink volume: %s" % stderr.strip())
    code, _stdout, stderr = _run(
        [
            "/usr/bin/pactl",
            "--server",
            server,
            "set-sink-mute",
            sink,
            "0" if unmuted else "1",
        ]
    )
    if code != 0:
        raise RequestError("cannot set PulseAudio sink mute: %s" % stderr.strip())


def _play_tone(card, wav_bytes):
    server = _pulse_server()
    if server is not None:
        sink = _pulse_sink(server)
        code, _stdout, stderr = _run(
            [
                "/usr/bin/paplay",
                "--server",
                server,
                "--device",
                sink,
                "--client-name",
                "rosclaw-limo-tone",
                "--stream-name",
                "bounded-tone",
            ],
            input_bytes=wav_bytes,
        )
        if code != 0:
            raise RequestError("PulseAudio playback failed: %s" % stderr.strip())
        return "pulseaudio", "pulse:%s" % sink
    device = "plughw:%d,0" % card
    code, _stdout, stderr = _run(
        ["/usr/bin/aplay", "-q", "-D", device],
        input_bytes=wav_bytes,
    )
    if code != 0:
        raise RequestError("ALSA playback failed: %s" % stderr.strip())
    return "alsa", device


def _capture_command(card, duration_sec):
    return [
        "/usr/bin/arecord",
        "-q",
        "-D",
        "plughw:%d,0" % card,
        "-f",
        "S16_LE",
        "-r",
        str(SAMPLE_RATE_HZ),
        "-c",
        "1",
        "-t",
        "raw",
        "-d",
        str(duration_sec),
    ]


def _device_busy(stderr):
    return "Device or resource busy" in str(stderr)


def _pulse_capture_command(server, source):
    return [
        "/usr/bin/parec",
        "--server",
        server,
        "--device",
        source,
        "--client-name",
        "rosclaw-limo-loopback",
        "--stream-name",
        "bounded-microphone-capture",
        "--format=s16le",
        "--rate=%d" % SAMPLE_RATE_HZ,
        "--channels=1",
        "--latency-msec=20",
        "--process-time-msec=20",
        "--raw",
    ]


def _finish_bounded_capture(recorder, started_at, duration_sec):
    remaining = max(0.0, duration_sec - (time.time() - started_at))
    if remaining:
        time.sleep(remaining)
    if recorder.poll() is None:
        recorder.terminate()
    stdout, stderr = recorder.communicate()
    if recorder.returncode not in (0, -15):
        raise RequestError("microphone capture failed: %s" % stderr.strip())
    if not stdout:
        raise RequestError("microphone capture returned no samples")
    return stdout


def _capture_pcm(card, duration_sec, server=None):
    if server is not None:
        source = _pulse_source(server)
        recorder = subprocess.Popen(
            _pulse_capture_command(server, source),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        started_at = time.time()
        time.sleep(CAPTURE_START_SETTLE_SEC)
        if recorder.poll() is not None:
            _stdout, stderr = recorder.communicate()
            raise RequestError("microphone capture failed: %s" % stderr.strip())
        return _finish_bounded_capture(recorder, started_at, duration_sec), "pulse:%s" % source
    for attempt in range(CAPTURE_BUSY_RETRIES):
        code, stdout, stderr = _run(_capture_command(card, duration_sec))
        if code == 0:
            if not stdout:
                raise RequestError("microphone capture returned no samples")
            return stdout, "plughw:%d,0" % card
        if not _device_busy(stderr) or attempt + 1 >= CAPTURE_BUSY_RETRIES:
            raise RequestError("microphone capture failed: %s" % stderr.strip())
        time.sleep(CAPTURE_BUSY_RETRY_DELAY_SEC)
    raise RequestError("microphone capture failed after bounded retries")


def _start_capture(card, duration_sec, server=None):
    if server is not None:
        source = _pulse_source(server)
        recorder = subprocess.Popen(
            _pulse_capture_command(server, source),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(CAPTURE_START_SETTLE_SEC)
        if recorder.poll() is not None:
            _stdout, stderr = recorder.communicate()
            raise RequestError("microphone loopback capture failed: %s" % stderr.strip())
        return recorder, "pulse:%s" % source
    for attempt in range(CAPTURE_BUSY_RETRIES):
        recorder = subprocess.Popen(
            _capture_command(card, duration_sec),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        time.sleep(CAPTURE_START_SETTLE_SEC)
        if recorder.poll() is None:
            return recorder, "plughw:%d,0" % card
        _stdout, stderr = recorder.communicate()
        if not _device_busy(stderr) or attempt + 1 >= CAPTURE_BUSY_RETRIES:
            raise RequestError("microphone loopback capture failed: %s" % stderr.strip())
        time.sleep(CAPTURE_BUSY_RETRY_DELAY_SEC)
    raise RequestError("microphone loopback capture failed after bounded retries")


def _capture_during_playback(card, wav_bytes, server=None):
    recorder, capture_device = _start_capture(card, LOOPBACK_CAPTURE_SEC, server)
    started_at = time.time()
    try:
        time.sleep(LOOPBACK_PREROLL_SEC)
        playback_backend, device = _play_tone(card, wav_bytes)
        if server is not None:
            stdout = _finish_bounded_capture(recorder, started_at, LOOPBACK_CAPTURE_SEC)
            stderr = b""
        else:
            stdout, stderr = recorder.communicate()
    except Exception:
        if recorder.poll() is None:
            recorder.terminate()
            recorder.communicate()
        raise
    allowed_returncodes = (0, -15) if server is not None else (0,)
    if recorder.returncode not in allowed_returncodes:
        raise RequestError("microphone loopback capture failed: %s" % stderr.strip())
    if not stdout:
        raise RequestError("microphone loopback capture returned no samples")
    return playback_backend, device, capture_device, stdout


def _pcm_samples(payload):
    sample_bytes = payload[: len(payload) - len(payload) % 2]
    if not sample_bytes:
        raise RequestError("microphone capture returned no complete samples")
    return struct.unpack("<%dh" % (len(sample_bytes) // 2), sample_bytes)


def _dbfs(amplitude):
    return 20.0 * math.log10(max(float(amplitude) / 32768.0, 1e-12))


def _frequency_amplitude(samples, frequency_hz):
    count = len(samples)
    if count < 1:
        return 0.0
    cosine = 0.0
    sine = 0.0
    scale = 2.0 * math.pi * frequency_hz / SAMPLE_RATE_HZ
    for index, value in enumerate(samples):
        angle = scale * index
        cosine += value * math.cos(angle)
        sine += value * math.sin(angle)
    return 2.0 * math.sqrt(cosine * cosine + sine * sine) / count


def _pcm_metrics(payload, frequency_hz):
    samples = _pcm_samples(payload)
    window_size = min(4096, len(samples))
    step = max(1, window_size // 2)
    starts = list(range(0, max(1, len(samples) - window_size + 1), step))
    final_start = max(0, len(samples) - window_size)
    if final_start not in starts:
        starts.append(final_start)
    best = None
    for start in starts:
        window = samples[start : start + window_size]
        target = _frequency_amplitude(window, frequency_hz)
        if best is None or target > best[0]:
            left = _frequency_amplitude(window, frequency_hz - 80)
            right = _frequency_amplitude(window, frequency_hz + 80)
            rms = math.sqrt(sum(float(value) * value for value in window) / len(window))
            peak = max(abs(value) for value in window)
            best = (target, max(left, right), rms, peak, start)
    target, adjacent, rms, peak, start = best
    return {
        "sample_count": len(samples),
        "analysis_window_samples": window_size,
        "analysis_window_start_sample": start,
        "target_dbfs": round(_dbfs(target), 2),
        "adjacent_dbfs": round(_dbfs(adjacent), 2),
        "target_prominence_db": round(_dbfs(target) - _dbfs(adjacent), 2),
        "rms_dbfs": round(_dbfs(rms), 2),
        "peak_dbfs": round(_dbfs(peak), 2),
    }


def _loopback_evidence(baseline_pcm, observed_pcm, frequency_hz, capture_device):
    baseline = _pcm_metrics(baseline_pcm, frequency_hz)
    observed = _pcm_metrics(observed_pcm, frequency_hz)
    target_gain_db = round(observed["target_dbfs"] - baseline["target_dbfs"], 2)
    detected = (
        observed["target_dbfs"] >= LOOPBACK_MIN_TARGET_DBFS
        and target_gain_db >= LOOPBACK_MIN_GAIN_DB
        and observed["target_prominence_db"] >= LOOPBACK_MIN_PROMINENCE_DB
    )
    return {
        "detected": detected,
        "sensor": "onboard_usb_microphone",
        "capture_device": capture_device,
        "target_frequency_hz": frequency_hz,
        "target_gain_db": target_gain_db,
        "thresholds": {
            "minimum_target_dbfs": LOOPBACK_MIN_TARGET_DBFS,
            "minimum_gain_db": LOOPBACK_MIN_GAIN_DB,
            "minimum_prominence_db": LOOPBACK_MIN_PROMINENCE_DB,
        },
        "baseline": baseline,
        "during_playback": observed,
        "audio_retained": False,
        "audio_content_returned": False,
        "privacy_note": "PCM was analyzed in memory and discarded.",
    }


def _run_audio(request):
    card = _usb_audio_card()
    server = _pulse_server()
    baseline_pcm, capture_device = _capture_pcm(card, BASELINE_CAPTURE_SEC, server)
    wav_bytes, frame_count = _tone_wav(
        request["frequency_hz"], request["duration_sec"], request["volume_percent"]
    )
    started = time.time()
    restored = False
    if server is not None:
        sink = _pulse_sink(server)
        original_state = _pulse_sink_state(server, sink)
        reference_volumes = [PULSE_REFERENCE_VOLUME] * len(original_state["channel_volumes"])
        try:
            _set_pulse_sink_state(server, sink, reference_volumes, True)
            playback_backend, device, observed_capture_device, observed_pcm = (
                _capture_during_playback(card, wav_bytes, server)
            )
        finally:
            _set_pulse_sink_state(
                server,
                sink,
                original_state["channel_volumes"],
                original_state["unmuted"],
            )
            restored = True
        original_output_state = {
            "backend": "pulseaudio",
            "channel_volumes": original_state["channel_volumes"],
            "unmuted": original_state["unmuted"],
        }
    else:
        original_percent, original_unmuted = _speaker_state(card)
        try:
            _set_speaker_state(card, 100, True, mapped=True)
            playback_backend, device, observed_capture_device, observed_pcm = (
                _capture_during_playback(card, wav_bytes)
            )
        finally:
            _set_speaker_state(card, original_percent, original_unmuted)
            restored = True
        original_output_state = {
            "backend": "alsa",
            "volume_percent": original_percent,
            "unmuted": original_unmuted,
        }
    if observed_capture_device != capture_device:
        raise RequestError("microphone capture device changed during loopback")
    loopback = _loopback_evidence(
        baseline_pcm, observed_pcm, request["frequency_hz"], capture_device
    )
    return {
        "protocol": PROTOCOL,
        "ok": True,
        "accepted": True,
        "action_id": request["action_id"],
        "operation": request["operation"],
        "device": device,
        "playback_backend": playback_backend,
        "alsa_card_index": card,
        "frequency_hz": request["frequency_hz"],
        "duration_sec": request["duration_sec"],
        "volume_percent": request["volume_percent"],
        "sample_rate_hz": SAMPLE_RATE_HZ,
        "frame_count": frame_count,
        "volume_mapping": "pcm_linear_percent",
        "digital_peak_scale": MAX_AMPLITUDE * request["volume_percent"] / 100.0,
        "reference_output_gain_percent": 100,
        "started_wall_time": started,
        "completed_wall_time": time.time(),
        "mixer_restored": restored,
        "original_output_state": original_output_state,
        "acoustic_loopback": loopback,
        "acoustic_loopback_detected": loopback["detected"],
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
