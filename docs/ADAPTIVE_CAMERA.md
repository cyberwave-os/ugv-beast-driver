# Adaptive camera streaming (UGV Beast) — internal

Internal architecture notes for the runtime-selectable, self-adapting camera path.
(No user-facing surface beyond the `set_camera_format` command; keep out of public docs.)

## Pipeline

```
CameraDeviceManager ──spawns──▶ usb_cam ──/image_raw──▶ ROSVideoStreamTrack ──▶ WebRTC
   (owns /dev/video0,                                     (re-encode + adapt)
    runtime format switch)
```

The bridge **owns the usb_cam process** (`camera.managed_by_bridge: true`). The
static launch-file `usb_cam` node is disabled in that mode (launch arg
`camera_managed_by_bridge:=true`) to avoid two owners of `/dev/video0`.

## Config (single source of truth)

All camera config lives in `config/mappings/robot_ugv_beast_v1.yaml` under `camera:`
(the old `camera:` override in `config/params.yaml` was removed — it silently forced
YUYV). Key fields: `pixel_format`, `image_width/height`, `capture_fps`, `stream_fps`,
`video_device` (`"auto"` ⇒ resolved via `device_utils`), `supported_formats`,
`adaptation`.

USB-2 reality: uncompressed **YUYV** can't do high resolution on the Pi bus; **MJPG**
is the only way to stream the high modes. Default capture is MJPG 1920×1080@30.

## Modules

| Module | Responsibility | Host-testable |
|---|---|---|
| `plugins/device_utils.py` | `v4l2-ctl` discovery; resolve real USB cam, skip Pi platform nodes (`pispbe`/`rpivid`) | yes |
| `plugins/camera_device_manager.py` | usb_cam lifecycle: validate → kill → **reap** → device-free → relaunch; watchdog `decide_recovery` | yes |
| `plugins/camera_frame.py` | resize + yuv420p encode (off the event loop) | yes |
| `plugins/camera_adaptation.py` | CPU ladder walker, fps-floor rule, hysteresis | yes |
| `plugins/ros_camera.py` | `ROSVideoStreamTrack` (mutable fps/scale, wall-clock pts) + adaptation loop + `_build_stream_config` | partial |
| `plugins/webrtc_preflight.py` | STUN/TURN reachability (NAT/Docker) | yes |

## Runtime selection

`start_video` data and the `set_camera_format` command accept
`{format,width,height,fps}`. `node.set_camera_format()` validates → stops stream →
`CameraDeviceManager.reconfigure()` → waits for a frame → restarts. Unsupported
requests (e.g. YUYV 1920×1080@30) are rejected as a no-op (stream stays up).

## Adaptation rule (CPU-driven)

Reduce **fps to `fps_floor` (15) first, then only shrink dimension — never below 15 fps**.
Cheap rungs (send-fps throttle + `cv2` downscale) apply bridge-side with no camera
touch; capture-resolution relaunch is reserved for explicit selection / watchdog.
Hysteresis: step down when `cpu_ema > cpu_high`; step up only after `cooldown_s`.

## NAT / Docker

`force_turn: true` ⇒ relay-only ICE. Only **outbound** reach to `turn.cyberwave.com`
is needed (works through Docker bridge NAT — no host networking / port mapping). A
STUN preflight logs reachability at startup; a failure is the usual cause of a silent
no-media stream inside the container.

## Tests

Host (no ROS/container): `mqtt_bridge/tests/test_device_utils.py`,
`test_camera_format_switch.py`, `test_camera_frame.py`, `test_camera_adaptation.py`,
`test_ros_camera_adaptive.py`, `test_camera_command_video.py`, `test_camera_watchdog.py`,
`test_webrtc_preflight.py`, and `test_stream_e2e.py` (real aiortc loopback — proves frames
are decoded on the far peer at full and adapted resolution). Needs `pytest`, `pyyaml`,
`aiortc` plus `cyberwave-edge-common` on `PYTHONPATH`; ROS msgs are stubbed by `conftest.py`.
