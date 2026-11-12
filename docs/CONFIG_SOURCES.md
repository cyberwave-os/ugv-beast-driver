# Configuration sources — twin JSON vs mapping YAML (internal)

This driver reads config from two places. Getting the split right is what keeps the
digital twin authoritative for identity while ROS/hardware config stays local.

**Golden rule:** twin / asset / environment / sensor data → the edge-core **JSON
files** (or the Cyberwave SDK). ROS 2 / mechanical / hardware-capture config → the
**mapping YAML** (`config/mappings/robot_ugv_beast_v1.yaml`).

## The one reader

All JSON + `CYBERWAVE_*` env reading is centralized in
[`mqtt_bridge/edge_driver_env.py`](../mqtt_bridge/edge_driver_env.py):

- `build_twin_config(environ, config_dir=...)` — pure, testable; reads all six files once.
- `load_twin_config()` — process entrypoint, memoized on the twin file's mtime.
- `log_twin_config(logger, cfg)` — surfaces every read value (anti-discard guard).

Files (edge-core materializes these in `CONFIG_DIR`, bind-mounted at `/app/.cyberwave`):
`{twin_uuid}.json` (via `CYBERWAVE_TWIN_JSON_FILE`), `edge.json`, `environment.json`,
`cameras.json`, `fingerprint.json`, `credentials.json`.

## Source-of-truth matrix

| Setting | Source | Path / precedence |
|---------|--------|-------------------|
| Camera sensor id (WebRTC `sensor`) | Twin JSON | `capabilities.sensors[].id` (first `rgb`); fallbacks: `asset.capabilities` → `universal_schema.sensors` → `_production_capabilities.sensors` |
| Sensor type | Twin JSON | normalized (`camera`/`rgb_camera`/`rgbd`→`rgb`, `depth_camera`→`depth`) |
| Camera TF `frame_id` | Twin JSON | `sensors[].parent_link` (e.g. `camera_link`) |
| Video device | env + JSON | `CYBERWAVE_METADATA_VIDEO_DEVICE` > `metadata.sensors_devices[id]` > `edge.json.metadata.cameras` > `cameras.json` (`twin_to_device`→`devices[index].primary_path`); YAML `video_device: "auto"` last resort |
| `edge_configs` | Twin JSON | `metadata.edge_configs` (`{}` when absent) |
| `edge_fingerprint` | Twin JSON | `metadata.edge_fingerprint` (cross-checked vs `fingerprint.json`) |
| `registry_id` | Twin JSON | `asset.registry_id` |
| `environment_uuid` | environment.json | `uuid` > `CYBERWAVE_ENVIRONMENT_UUID` env |
| MQTT host/port/TLS, base_url, environment, log levels | env | `CYBERWAVE_*`; `credentials.json.envs` best-effort fallback |
| Camera capture: `image_topic`, `pixel_format`, `image_width/height`, `capture_fps`, `stream_fps`, `supported_formats`, `adaptation.*`, `io_method`, `managed_by_bridge`, `camera_info_url`, `format`, `auto_*` | Mapping YAML | ROS/usb_cam hardware |
| `joints`, `internal_odometry`, `robot_constants`, `command_registry`, `capabilities.upstream_*` | Mapping YAML | ROS/mechanical |

## TF frame bridge (two-URDF nuance)

The twin's `parent_link` is `camera_link`. The **deployed ROS URDF** (`ugv_description/
urdf/ugv_beast.urdf`, cloned from `github.com/DUDULRX/ugv_ws`, loaded by
`robot_state_publisher`) names the RGB link **`pt_camera_link`** and has no
`camera_link`. usb_cam stamps `camera_info` in the twin `frame_id` (`camera_link`), so
the node publishes a static transform `pt_camera_link → camera_link`
(`_start_camera_frame_bridge`, using `URDF_RGB_CAMERA_LINK` +
`static_tf_command` in `plugins/camera_device_manager.py`) to keep TF valid. WebRTC
itself does not use `frame_id`. Follow-up: align the twin's `parent_link` with the
URDF (`pt_camera_link`) on the backend and drop the shim.

## Tests

- `tests/test_twin_config.py` — resolver unit tests (golden fixtures under `tests/fixtures/ugv_beast/`).
- `tests/test_ros_camera_identity.py`, `tests/test_camera_device_manager_identity.py` — identity wiring + static TF.
- `tests/test_settings_not_discarded.py` — anti-discard.
- `tests/test_twin_config_actuation.py` — sensors/edge_configs/sensors_devices/device selection.
- `tests/test_twin_config_integration.py` — all six files compose.
- `tests/test_webrtc_camera_compliance.py` — the WebRTC contract (frame invariants, offer/SDP, answer matching).
