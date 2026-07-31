# LIMO peripheral inspection contract

This contract extends the ROS inspection surface with host peripherals that are physically
present on the tested LIMO. Detection is correlated across the AgileX manual, Linux USB/sysfs,
ALSA, framebuffer/input state, and the live ROS graph. A documented device is never marked
connected from documentation alone.

## Read-only tools

| Tool | Evidence | Side effect |
| --- | --- | --- |
| `limo_list_peripherals` | USB IDs, ROS topics, framebuffer/input presence | None |
| `limo_get_camera_state` | Color/depth core streams and optional IR streams | None |
| `limo_get_dabai_device_state` | Six getter-only `astra_camera` services | None |
| `limo_get_audio_state` | ALSA PCM and mixer state | None |
| `limo_measure_microphone` | Bounded PCM RMS/peak statistics | Samples discarded; no file or audio content |
| `limo_get_display_state` | Framebuffer geometry and touchscreen identity | None |
| `limo_get_platform_health` | Thermal, memory, swap, disk, load, uptime | None |

`limo_get_dabai_device_state` has an exact service allowlist:

- `/camera/get_device_info`
- `/camera/get_device_type`
- `/camera/get_serial`
- `/camera/get_version`
- `/camera/get_ir_temperature`
- `/camera/get_ldp_status`

Setter, toggle, laser, fan, save-image, and save-point-cloud services are not exposed.

## Camera readiness

Color image/CameraInfo, depth image/CameraInfo, and depth point cloud are the core Dabai readiness
set. IR image and IR CameraInfo are optional because the tested `dabai_u3.launch` node advertises
them with `enable_ir=true` but does not emit messages while the current RGB/depth combination is
running. `limo_get_camera_state` remains healthy when every core stream is live and reports the IR
streams in `optional_inactive`.

## Audio boundary

The tested USB audio codec provides playback, capture, Speaker gain/mute, and microphone capture
gain. `limo_measure_microphone` accepts only one to three seconds and a small sample-rate allowlist.
It reads mono signed 16-bit PCM into memory, computes RMS/peak/clipping statistics, then discards
the samples. It never returns raw audio.

`limo_request_tone` exposes one deliberately narrow physical effect through a
signed Robot Pack and `rosclawd`: a 440, 660, or 880 Hz synthesized tone lasting
0.2–1.0 seconds at 5–25% temporary Speaker gain. REAL playback requires exact
in-context operator confirmation. The revision-locked worker selects the single
detected USB PnP audio card and restores the previous Speaker gain/mute state.
It accepts no file path, arbitrary command, mixer name, or ALSA device. A
successful receipt is `DRIVER_CONFIRMED`; human hearing is separate evidence.

## Declared but unbound

- The 128x64 front OLED is documented, but no stable Linux or ROS interface was found.
- Chassis RGB status lights are controlled by firmware; the upstream `limo_ros` driver exposes no
  light API.
- No separate power, temperature, or fault interface for the physical audio amplifier was found.

These devices are returned as `declared_unbound`, not as guessed or generic shell controls.
