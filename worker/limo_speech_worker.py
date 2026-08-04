#!/usr/bin/env python2
"""Bounded daemon-owned speech worker for the LIMO USB speaker.

The worker accepts one exact JSON request on stdin. It synthesizes speech with
the locally installed eSpeak-NG shared library, plays a normalized in-memory WAV
through the same fixed USB-audio path as the tone worker, verifies non-silent
acoustic output with the onboard microphone, restores mixer state, and returns
one JSON result. No shell, arbitrary command, file output, or generic audio
surface is exposed.
"""

import array
import ctypes
import hashlib
import io
import json
import math
import os
import sys
import time
import unicodedata
import wave

import limo_tone_worker as audio

PROTOCOL = "rosclaw.limo.worker.v1"
SCHEMA = "limo.speech.v1"
MAX_REQUEST_BYTES = 65536
MAX_TEXT_CHARS = 80
LANGUAGE_VOICES = {"cmn": "cmn", "en": "en-us"}
MIN_VOLUME_PERCENT = 10
MAX_VOLUME_PERCENT = 25
MIN_RATE_WPM = 120
MAX_RATE_WPM = 200
LOOPBACK_MIN_RMS_DBFS = -45.0
LOOPBACK_MIN_GAIN_DB = 8.0
ESPEAK_OUTPUT_RETRIEVAL = 1
ESPEAK_RATE_PARAMETER = 1
ESPEAK_VOLUME_PARAMETER = 2
ESPEAK_POS_CHARACTER = 1
ESPEAK_CHARS_UTF8 = 1
ESPEAK_SUCCESS = 0
try:
    STRING_TYPES = (basestring,)  # type: ignore[name-defined]  # noqa: F821
    TEXT_TYPE = unicode  # type: ignore[name-defined]  # noqa: F821
except NameError:
    STRING_TYPES = (str,)
    TEXT_TYPE = str


class RequestError(Exception):
    pass


def _text_value(value):
    if not isinstance(value, STRING_TYPES):
        raise RequestError("text must be a string")
    if not isinstance(value, TEXT_TYPE):
        value = value.decode("utf-8")
    if not value or len(value) > MAX_TEXT_CHARS:
        raise RequestError("text length must be within [1, %d] characters" % MAX_TEXT_CHARS)
    if value != value.strip():
        raise RequestError("text must not have leading or trailing whitespace")
    if any(unicodedata.category(character).startswith("C") for character in value):
        raise RequestError("text must not contain control or formatting characters")
    return value


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
            "text",
            "language",
            "volume_percent",
            "rate_wpm",
        ]
    )
    unknown = sorted(set(raw) - allowed)
    if unknown:
        raise RequestError("unknown request fields: %s" % ", ".join(unknown))
    if raw.get("protocol") != PROTOCOL:
        raise RequestError("unsupported worker protocol")
    if raw.get("operation") != "SPEAK_TEXT":
        raise RequestError("unsupported operation")
    if raw.get("schema_version") != SCHEMA:
        raise RequestError("unsupported speech schema")
    action_id = raw.get("action_id")
    body_hash = raw.get("body_snapshot_hash")
    if not isinstance(action_id, STRING_TYPES) or not action_id:
        raise RequestError("action_id is required")
    if not isinstance(body_hash, STRING_TYPES) or not body_hash:
        raise RequestError("body_snapshot_hash is required")
    language = raw.get("language")
    if language not in LANGUAGE_VOICES:
        raise RequestError("language must be cmn or en")
    volume = raw.get("volume_percent")
    if isinstance(volume, bool) or not isinstance(volume, int):
        raise RequestError("volume_percent must be an integer")
    if not MIN_VOLUME_PERCENT <= volume <= MAX_VOLUME_PERCENT:
        raise RequestError("volume_percent must be within [10, 25]")
    rate = raw.get("rate_wpm")
    if isinstance(rate, bool) or not isinstance(rate, int):
        raise RequestError("rate_wpm must be an integer")
    if not MIN_RATE_WPM <= rate <= MAX_RATE_WPM:
        raise RequestError("rate_wpm must be within [120, 200]")
    return {
        "protocol": PROTOCOL,
        "operation": "SPEAK_TEXT",
        "schema_version": SCHEMA,
        "action_id": action_id,
        "body_id": raw.get("body_id"),
        "body_snapshot_hash": body_hash,
        "text": _text_value(raw.get("text")),
        "language": language,
        "volume_percent": volume,
        "rate_wpm": rate,
    }


def _espeak_library():
    try:
        return ctypes.CDLL("libespeak-ng.so.1")
    except OSError as exc:
        raise RequestError("eSpeak-NG shared library unavailable: %s" % exc)


def _synthesize(text, language, rate_wpm):
    library = _espeak_library()
    chunks = []
    callback_type = ctypes.CFUNCTYPE(
        ctypes.c_int,
        ctypes.POINTER(ctypes.c_short),
        ctypes.c_int,
        ctypes.c_void_p,
    )

    def receive(wav_pointer, sample_count, _events):
        if wav_pointer and sample_count > 0:
            chunks.append(ctypes.string_at(wav_pointer, sample_count * 2))
        return 0

    callback = callback_type(receive)
    library.espeak_Initialize.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_char_p, ctypes.c_int]
    library.espeak_Initialize.restype = ctypes.c_int
    library.espeak_SetSynthCallback.argtypes = [callback_type]
    library.espeak_SetVoiceByName.argtypes = [ctypes.c_char_p]
    library.espeak_SetParameter.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int]
    library.espeak_Synth.argtypes = [
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_uint,
        ctypes.c_int,
        ctypes.c_uint,
        ctypes.c_uint,
        ctypes.c_void_p,
        ctypes.c_void_p,
    ]
    sample_rate = int(library.espeak_Initialize(ESPEAK_OUTPUT_RETRIEVAL, 0, None, 0))
    if sample_rate <= 0:
        raise RequestError("eSpeak-NG initialization failed")
    library.espeak_SetSynthCallback(callback)
    voice = LANGUAGE_VOICES[language].encode("ascii")
    if library.espeak_SetVoiceByName(voice) != ESPEAK_SUCCESS:
        raise RequestError("requested eSpeak-NG voice is unavailable")
    if library.espeak_SetParameter(ESPEAK_RATE_PARAMETER, rate_wpm, 0) != ESPEAK_SUCCESS:
        raise RequestError("eSpeak-NG rejected speech rate")
    library.espeak_SetParameter(ESPEAK_VOLUME_PARAMETER, 100, 0)
    payload = text.encode("utf-8")
    buffer_value = ctypes.create_string_buffer(payload + b"\0")
    result = library.espeak_Synth(
        ctypes.cast(buffer_value, ctypes.c_void_p),
        len(payload) + 1,
        0,
        ESPEAK_POS_CHARACTER,
        0,
        ESPEAK_CHARS_UTF8,
        None,
        None,
    )
    if result != ESPEAK_SUCCESS:
        raise RequestError("eSpeak-NG synthesis failed with code %d" % result)
    library.espeak_Synchronize()
    pcm = b"".join(chunks)
    if not pcm:
        raise RequestError("eSpeak-NG returned no audio samples")
    return sample_rate, pcm


def _normalize_pcm(pcm, volume_percent):
    samples = array.array("h")
    if hasattr(samples, "frombytes"):
        samples.frombytes(pcm)
    else:  # Python 2 worker runtime
        samples.fromstring(pcm)
    if sys.byteorder != "little":
        samples.byteswap()
    peak = max(abs(value) for value in samples)
    if peak <= 0:
        raise RequestError("synthesized speech is silent")
    target_peak = 32767.0 * 0.9 * volume_percent / 100.0
    scale = target_peak / peak
    normalized = array.array(
        "h", [int(max(-32768, min(32767, value * scale))) for value in samples]
    )
    if sys.byteorder != "little":
        normalized.byteswap()
    normalized_bytes = (
        normalized.tobytes() if hasattr(normalized, "tobytes") else normalized.tostring()
    )
    return normalized_bytes, len(normalized), round(target_peak / 32767.0, 4)


def _wav_bytes(sample_rate, pcm):
    buffer_value = io.BytesIO()
    writer = wave.open(buffer_value, "wb")
    writer.setnchannels(1)
    writer.setsampwidth(2)
    writer.setframerate(sample_rate)
    writer.writeframes(pcm)
    writer.close()
    return buffer_value.getvalue()


def _level_metrics(pcm):
    samples = audio._pcm_samples(pcm)
    rms = math.sqrt(sum(float(value) * value for value in samples) / len(samples))
    peak = max(abs(value) for value in samples)
    return {
        "sample_count": len(samples),
        "rms_dbfs": round(audio._dbfs(rms), 2),
        "peak_dbfs": round(audio._dbfs(peak), 2),
    }


def _speech_loopback(baseline_pcm, observed_pcm, capture_device):
    baseline = _level_metrics(baseline_pcm)
    observed = _level_metrics(observed_pcm)
    gain = round(observed["rms_dbfs"] - baseline["rms_dbfs"], 2)
    detected = observed["rms_dbfs"] >= LOOPBACK_MIN_RMS_DBFS and gain >= LOOPBACK_MIN_GAIN_DB
    return {
        "detected": detected,
        "sensor": "onboard_usb_microphone",
        "capture_device": capture_device,
        "rms_gain_db": gain,
        "thresholds": {
            "minimum_rms_dbfs": LOOPBACK_MIN_RMS_DBFS,
            "minimum_gain_db": LOOPBACK_MIN_GAIN_DB,
        },
        "baseline": baseline,
        "during_playback": observed,
        "content_recognition_performed": False,
        "audio_retained": False,
        "audio_content_returned": False,
        "privacy_note": "PCM was analyzed in memory and discarded.",
    }


def _run_speech(request):
    card = audio._usb_audio_card()
    server = audio._pulse_server()
    baseline_pcm, capture_device = audio._capture_pcm(card, audio.BASELINE_CAPTURE_SEC, server)
    sample_rate, raw_pcm = _synthesize(request["text"], request["language"], request["rate_wpm"])
    pcm, frame_count, digital_peak_scale = _normalize_pcm(raw_pcm, request["volume_percent"])
    speech_wav = _wav_bytes(sample_rate, pcm)
    started = time.time()
    restored = False
    if server is not None:
        sink = audio._pulse_sink(server)
        original_state = audio._pulse_sink_state(server, sink)
        reference_volumes = [audio.PULSE_REFERENCE_VOLUME] * len(original_state["channel_volumes"])
        try:
            audio._set_pulse_sink_state(server, sink, reference_volumes, True)
            playback_backend, device, observed_capture_device, observed_pcm = (
                audio._capture_during_playback(card, speech_wav, server)
            )
        finally:
            audio._set_pulse_sink_state(
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
        original_percent, original_unmuted = audio._speaker_state(card)
        try:
            audio._set_speaker_state(card, 100, True, mapped=True)
            playback_backend, device, observed_capture_device, observed_pcm = (
                audio._capture_during_playback(card, speech_wav)
            )
        finally:
            audio._set_speaker_state(card, original_percent, original_unmuted)
            restored = True
        original_output_state = {
            "backend": "alsa",
            "volume_percent": original_percent,
            "unmuted": original_unmuted,
        }
    if observed_capture_device != capture_device:
        raise RequestError("microphone capture device changed during loopback")
    loopback = _speech_loopback(baseline_pcm, observed_pcm, capture_device)
    text_utf8 = request["text"].encode("utf-8")
    return {
        "protocol": PROTOCOL,
        "ok": True,
        "accepted": True,
        "action_id": request["action_id"],
        "operation": request["operation"],
        "device": device,
        "playback_backend": playback_backend,
        "alsa_card_index": card,
        "language": request["language"],
        "rate_wpm": request["rate_wpm"],
        "volume_percent": request["volume_percent"],
        "sample_rate_hz": sample_rate,
        "frame_count": frame_count,
        "text_character_count": len(request["text"]),
        "text_sha256": "sha256:" + hashlib.sha256(text_utf8).hexdigest(),
        "volume_mapping": "normalized_pcm_linear_percent",
        "digital_peak_scale": digital_peak_scale,
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
    return _run_speech(request)


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
                    "error_code": "LIMO_SPEECH_WORKER_FAILED",
                    "error": str(exc),
                },
                sort_keys=True,
                separators=(",", ":"),
            )
        )
        sys.exit(1)
