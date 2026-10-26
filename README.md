<p align="center">
  <a href="https://cyberwave.com">
    <img src="https://cyberwave.com/cyberwave-logo-black.svg" alt="Cyberwave logo" width="240" />
  </a>
</p>

# Welcome to the Cyberwave UGV Beast Driver!

This module is part of **Cyberwave: Making the physical world programmable**.

[![License](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](https://opensource.org/licenses/Apache-2.0)
[![Documentation](https://img.shields.io/badge/Documentation-docs.cyberwave.com-orange)](https://docs.cyberwave.com)
[![Discord](https://badgen.net/badge/icon/discord?icon=discord&label&color=orange)](https://discord.gg/dfGhNrawyF)
[![Docker Build](https://github.com/cyberwave-os/cyberwave-edge-ros-ugv/actions/workflows/push-to-docker-hub.yml/badge.svg)](https://github.com/cyberwave-os/cyberwave-edge-ros-ugv/actions/workflows/push-to-docker-hub.yml)

We are happy to present the official Cyberwave integration for the **Waveshare UGV Beast** — a tracked rover built on a Raspberry Pi + ESP32, with a pan-tilt camera, headlights, LiDAR and IMU.

This repo bridges the rover's **ROS 2** stack to the Cyberwave cloud over **MQTT** (with **WebRTC** for video), so a physical UGV Beast and its Cyberwave digital twin stay in sync in real time. Teleoperate it, stream its camera, watch its odometry, and drive it from the web app, the Python SDK, or a controller — from anywhere.

We build on the excellent open-source [Waveshare `ugv_ws`](https://github.com/waveshareteam/ugv_ws) ROS 2 stack and the [UGV Beast PI ROS2 wiki](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2). Cyberwave adds the cloud bridge, digital twin, teleop, and fleet management on top.

> [!IMPORTANT]
> **New here? Start with the [UGV Beast first-config guide](https://cyberwave.com/waveshare/ugv-beast) on cyberwave.com.** It walks you through flashing, networking and pairing the rover end-to-end. This README is the driver reference that sits underneath it.

## Project RoadMap:

1. Bidirectional ROS 2 ↔ MQTT bridge :white_check_mark:
2. Digital twin joint-state sync (wheels + pan-tilt) :white_check_mark:
3. Odometry / pose sync (internal dead-reckoning) :white_check_mark:
4. Real-time teleoperation (analog velocity + keyboard) :white_check_mark:
5. Camera streaming over WebRTC :white_check_mark:
6. Headlight (LED) control :white_check_mark:
7. Pan-tilt camera control :white_check_mark:
8. OLED display + photo capture :white_check_mark:
9. Emergency stop + movement watchdog :white_check_mark:
10. Multi-robot / fleet namespacing :white_check_mark:
11. One-command managed install via the Cyberwave CLI :white_check_mark:
12. IMU telemetry + battery reporting :white_check_mark:
13. Recording & Replay of camera streams :white_check_mark:
14. Nav2 autonomous navigation (goto / follow-path) :white_check_mark:
15. SLAM map creation & storage :construction:

## Your feedback and support mean the world to us.

If you're as enthusiastic about programmable robotics as we are, please consider giving this repo a :star: star!!!

Your encouragement fuels our passion and helps us push the RoadMap further. We welcome any help or suggestions you can offer — come say hi on [Discord](https://discord.gg/dfGhNrawyF).

Together, let's make the physical world programmable!

## Exciting Features:

:sparkles: **Zero-to-driving in minutes** — pair the rover with one CLI command and drive it from the browser.

:satellite: **Bidirectional cloud bridge** — ROS 2 telemetry flows up, teleop commands flow down, over a single MQTT connection.

:video_camera: **Live WebRTC video** — the front camera streams to your twin, with optional cloud recording for the Replay tab.

:joystick: **Real-time teleop** — analog velocity or discrete keyboard commands, with a safety watchdog that auto-stops on command silence.

:robot: **Robot-agnostic core** — all robot-specific behavior lives in a YAML mapping + a pluggable command registry; no core code changes to onboard a new robot.

:handshake: **Fleet-ready** — each rover runs under a per-twin ROS 2 namespace so many UGV Beasts can share one graph without colliding.

## What's in the box

This driver connects the following UGV Beast hardware to your Cyberwave twin:

| Capability | Hardware | Direction |
| --- | --- | --- |
| Drive / steer | 4 tracked wheels (ESP32 motor control) | command |
| Look around | Pan-tilt servo camera | command |
| See | Front camera (WebRTC stream + photos) | telemetry |
| Light up | Chassis + camera headlights (0–255 PWM) | command |
| Display | On-board OLED | command |
| Sense | IMU, wheel encoders (odometry), battery voltage | telemetry |
| Map & navigate | LiDAR + Nav2 (optional) | both |

## System requirements

The driver runs **on the rover** (a Raspberry Pi, native `arm64`). It talks to the Cyberwave cloud; you interact with it from your laptop via the web app, SDK or CLI.

| Component | Requirement |
| --- | --- |
| Robot | Waveshare UGV Beast (Raspberry Pi 4/5 + ESP32 sub-controller) |
| OS / ROS | Ubuntu 22.04 + ROS 2 Humble (the Docker image ships this for you) |
| Transport | MQTT (`mqtt.cyberwave.com:1883`) + WebRTC for video |
| Account | A Cyberwave account and an **API key** from your [profile](https://cyberwave.com) |
| Host tools | The [Cyberwave CLI](https://docs.cyberwave.com/feature-reference/edge/overview#use-cyberwave-edge-with-the-cli) and/or [Python SDK](https://docs.cyberwave.com/overview/tools/python-sdk#python-sdk) |

## Before you start

You need a **digital twin** for your rover in Cyberwave. Create it once in the web
app (add the UGV Beast asset to an environment), then copy its **twin UUID** — the
driver uses it to route every command and telemetry message. The
[first-config guide](https://cyberwave.com/waveshare/ugv-beast) walks through this;
you'll set it as `CYBERWAVE_TWIN_UUID` in your `.env` below.

## Installation

There are two ways to run the driver. **Most people should use Path A.**

### Path A — Managed install with the Cyberwave CLI (recommended)

The Cyberwave Edge CLI installs the edge core as a service, pulls the UGV driver container, and keeps it running. Full reference: **[Use Cyberwave Edge with the CLI](https://docs.cyberwave.com/feature-reference/edge/overview#use-cyberwave-edge-with-the-cli)**.

First complete the hardware / network / pairing steps in the **[UGV Beast first-config guide](https://cyberwave.com/waveshare/ugv-beast)**. Then, **on the rover**:

```shell
# 1. Install the edge core + pair this device to your Cyberwave account
curl -fsSL https://cyberwave.com/install.sh | bash
sudo cyberwave pair

# 2. Bring the edge node up (drivers are managed for you)
sudo cyberwave edge start

# 3. Check it's alive
cyberwave edge status
cyberwave edge logs
```

Useful edge commands:

| Command | Purpose |
| --- | --- |
| `cyberwave pair` | Install core + register this device (alias of `cyberwave edge install`) |
| `cyberwave edge start` / `stop` / `restart` | Control the edge node |
| `cyberwave edge status` | Is the edge node running? |
| `cyberwave edge logs` | Tail edge logs |
| `cyberwave edge driver` | Manage driver containers |

Once paired, open your UGV Beast twin in the Cyberwave web app and drive it.

### Path B — Run the driver container directly (advanced / offline)

If you're not using the managed edge core, pull and run the driver image on the rover. Complete the Waveshare prep first (through [1.2 Disable the main program from running automatically](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2_1._Preparation)), then:

```shell
docker pull cyberwaveos/ugv-driver:staging

docker run -dit --name cyb_ugv_beast \
  --privileged --network host --pid host --init \
  -v /dev:/dev -v /sys:/sys -v /run/udev:/run/udev:ro \
  -e CYBERWAVE_API_KEY="cw_your_api_key" \
  -e CYBERWAVE_TWIN_UUID="your-twin-uuid" \
  cyberwaveos/ugv-driver:staging
```

Inside the container, the whole stack starts with one launch file:

```shell
ros2 launch ugv_bringup master_beast.launch.py robot_id:=robot_ugv_beast_v1
```

`master_beast.launch.py` starts the hardware bringup, IMU filter, robot-state publisher, odometry (`base_node`), the MQTT bridge, and the camera — everything needed to appear in Cyberwave.

## Configuration

The driver is configured through environment variables (injected by edge-core in Path A, or set on the container in Path B) and a YAML mapping file.

### Credentials & speed limits (`.env`)

```shell
# --- Cyberwave credentials / MQTT ---
CYBERWAVE_API_KEY=cw_your_api_key_here     # also the MQTT password (user: mqttcyb)
CYBERWAVE_TWIN_UUID=your-twin-uuid-here
CYBERWAVE_MQTT_BROKER=mqtt.cyberwave.com
CYBERWAVE_MQTT_PORT=1883

# Upstream rate limiting (1 Hz = 1 second between publishes)
MQTT_PUBLISH_RATE_LIMIT=1.0

# --- UGV velocity limits (the ONLY software speed cap) ---
# MAX_*    : hard ceiling — every teleop command is clamped to this.
# DEFAULT_*: cruise speed for discrete keyboard commands (move_forward, ...).
# The rover's mechanical top speed is ~1.2 m/s.
CYBERWAVE_UGV_MAX_LINEAR_SPEED=0.8
CYBERWAVE_UGV_MAX_ANGULAR_SPEED=1.0
CYBERWAVE_UGV_DEFAULT_LINEAR_SPEED=0.5
CYBERWAVE_UGV_DEFAULT_ANGULAR_SPEED=1.0
```

> **MQTT broker resolution:** the driver picks its broker as
> `CYBERWAVE_MQTT_HOST` → `broker.host` in `params.yaml` → the Cyberwave default
> `mqtt.cyberwave.com` (and `CYBERWAVE_MQTT_PORT` → `broker.port` → `8883`, with
> TLS auto-enabled on `8883`). You don't need to set the host: with none provided
> the driver connects to `mqtt.cyberwave.com` on its own, so it comes up as long
> as `CYBERWAVE_API_KEY` and `CYBERWAVE_TWIN_UUID` are present. Set
> `CYBERWAVE_MQTT_HOST` only to point at a different broker (self-hosted or local).

### Twin mapping (`config/mappings/robot_ugv_beast_v1.yaml`)

The mapping describes the **shape of the robot** — its joints, camera, odometry
and which telemetry to publish. Your twin UUID is **not** set here; it comes from
`CYBERWAVE_TWIN_UUID` (injected by edge-core in Path A, or set on the container in
Path B). The ready-to-use file already has sensible defaults — the key sections:

```yaml
version: 1
robot_id: "robot_ugv_beast_v1"

# Which upstream telemetry the twin receives
capabilities:
  upstream_mode: "both"          # publish both pose and joint updates
  upstream_topics: [pose, joint]

# Peripheral commands (lights, pan-tilt, e-stop, ...) live in a pluggable registry
command_registry: "mqtt_bridge.plugins.ugv_beast_command_handler.CommandRegistry"

# The Beast has no wheel odometry sensor — we dead-reckon from wheel joints
internal_odometry:
  enabled: true
  track_width: 0.23      # metres
  wheel_radius: 0.04     # metres
  left_wheel_joints:  ["left_up_wheel_link_joint", "left_down_wheel_link_joint"]
  right_wheel_joints: ["right_up_wheel_link_joint", "right_down_wheel_link_joint"]

# Front camera → WebRTC. HARDWARE CAPTURE ONLY (MJPG 1280x720@30, adapts down
# under CPU load). Sensor IDENTITY is NOT here — the camera sensor id and TF
# frame_id come from the twin JSON (see "Configuration sources" below).
camera:
  image_topic: "/image_raw"
  # pixel_format, resolution and the adaptation ladder are configured in the file
```

(A generic `config/mappings/default.yaml` is also provided for non-Beast robots.)

### Configuration sources — twin JSON vs mapping YAML

The driver reads two kinds of config, and it matters which owns what:

| Concern | Source of truth | Notes |
|---------|-----------------|-------|
| Camera **sensor id** (WebRTC `sensor`) | **Twin JSON** `capabilities.sensors[].id` | e.g. `front_camera`; feeds recording/routing |
| Camera **TF `frame_id`** | **Twin JSON** `sensors[].parent_link` | e.g. `camera_link`; usb_cam stamps `camera_info` here |
| **Video device** | env + JSON | `CYBERWAVE_METADATA_VIDEO_DEVICE` > `sensors_devices` > `edge.json`/`cameras.json` |
| Twin/env identity | JSON + env | `edge_fingerprint`, `registry_id`, `environment_uuid`, `edge_configs` |
| Camera **capture** (topic, pixel_format, resolution, fps, adaptation, supported_formats) | **Mapping YAML** | ROS/usb_cam hardware config |
| Joints, odometry, robot_constants, command_registry | **Mapping YAML** | ROS/mechanical config |

All twin/sensor reading is centralized in [`mqtt_bridge/edge_driver_env.py`](mqtt_bridge/edge_driver_env.py)
(`build_twin_config` / `load_twin_config`), which reads the edge-core JSON files
(`{twin_uuid}.json`, `edge.json`, `environment.json`, `cameras.json`,
`fingerprint.json`, `credentials.json`) once at startup. `camera_name` and
`frame_id` were **removed** from the mapping YAML — do not reintroduce them.

**TF note:** the twin's `parent_link` is `camera_link`, but the vendored ROS URDF
(`ugv_description`) names the RGB link `pt_camera_link`. The node publishes a static
transform `pt_camera_link → camera_link` so `camera_info` stays reachable in TF.
See `docs/CONFIG_SOURCES.md` for the full matrix.

> **MQTT auth:** the broker username is the public default `mqttcyb`; the password is **your Cyberwave API key**. Never hardcode it — export `CYBERWAVE_API_KEY` and let the SDK/token path pick it up.

## Usage

Once the driver is up and the twin UUID is set, you control the rover by publishing JSON commands to `cyberwave/twin/{twin_uuid}/command`. Only messages with `"source_type": "tele"` reach the physical robot (edit/sim/edge are ignored for safety).

Set these once for the examples:

```shell
export CYBERWAVE_API_KEY="cw_your_api_key"   # your token; do not commit
export TWIN_UUID="your-twin-uuid"
```

### Driving (velocity control)

Movement uses the `velocity_command` payload. `linear_x` / `angular_z` are clamped to the `MAX_*` limits, and `duration_ms` arms a deadman stop — **resend while driving** or the rover stops on its own after the duration elapses (and after 0.5 s of command silence).

```shell
# Forward at 0.3 m/s for 600 ms
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"velocity_command","source_type":"tele","velocity_command":{"linear_x":0.3,"angular_z":0,"duration_ms":600}}'

# Turn in place
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"velocity_command","source_type":"tele","velocity_command":{"linear_x":0,"angular_z":0.5,"duration_ms":600}}'

# Stop
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"stop","source_type":"tele"}'
```

Discrete keyboard-style actuations (`move_forward`, `move_backward`, `turn_left`, `turn_right`, `stop`) are also supported via the `actuation` command and drive at the `DEFAULT_*` cruise speed.

### Headlights

```shell
# Both lights full brightness (0–255)
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"lights","source_type":"tele","data":{"all":255}}'

# Chassis light only
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"lights","source_type":"tele","data":{"chassis_light":255,"camera_light":0}}'
```

### Pan-tilt camera

```shell
# Absolute pan/tilt (radians). pan: -3.14..3.14, tilt: -0.785..1.57
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"camera_servo","source_type":"tele","data":{"pan":0.5,"tilt":0.3}}'

# Re-centre
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/command" \
  -m '{"command":"camera_servo","source_type":"tele","data":{"pan":0,"tilt":0}}'
```

### Video, photo, OLED and e-stop

```shell
# Start WebRTC video (with cloud recording)
... -m '{"command":"start_video","source_type":"tele","data":{"recording":true}}'
# Capture a still to cyberwave/twin/{uuid}/camera/photo
... -m '{"command":"take_photo","source_type":"tele","data":{}}'
# Write to the on-board OLED
... -m '{"command":"oled_ctrl","source_type":"tele","data":{"text":"Hello Cyberwave!"}}'
# Emergency stop
... -m '{"command":"estop","source_type":"tele","data":{"activate":true}}'
```

### Driving from the Python SDK

The friendliest way to control the rover programmatically is the **[Cyberwave Python SDK](https://docs.cyberwave.com/overview/tools/python-sdk#python-sdk)**:

```shell
pip install cyberwave
export CYBERWAVE_API_KEY=your_api_key_here
```

```python
from cyberwave import Cyberwave

cw = Cyberwave()
rover = cw.twin(twin_id="your-twin-uuid")

# Read live joint state coming up from the rover
print(rover.joints.get_all())

# Grab a frame from the camera stream
frame = rover.capture_frame("numpy")
```

See the SDK docs for the full command surface (joints, position/rotation, frame capture, and more).

### Autonomous navigation (Nav2)

When Nav2 is running, send goals to `cyberwave/twin/{twin_uuid}/navigate/command`:

```shell
# Go to a point in the map
mosquitto_pub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/twin/$TWIN_UUID/navigate/command" \
  -m '{"action":"goto","source_type":"tele","goal":{"x":2.5,"y":1.0,"theta":0.0}}'
```

`goto`, `path` (follow waypoints), and `stop` / `pause` / `resume` are supported; progress is reported on `.../navigate/status`.

## Command reference (at a glance)

All commands go to `cyberwave/twin/{twin_uuid}/command` (navigation uses
`.../navigate/command`), and each publishes an acknowledgement to
`.../{command}/status`. Full payloads, parameters and responses are documented in
**[docs/edge-ros-ugv-beast-setup/edge-ros-ugv-beast.md](docs/edge-ros-ugv-beast-setup/edge-ros-ugv-beast.md)**.

| Command | What it does |
| --- | --- |
| `velocity_command` | Analog drive (`linear_x`, `angular_z`, `duration_ms`) |
| `actuation` | Discrete drive (`move_forward`, `turn_left`, `stop`, …) |
| `lights` | Chassis + camera headlights (0–255) |
| `camera_servo` | Pan-tilt the camera (absolute or relative) |
| `oled_ctrl` | Write text to the OLED |
| `start_video` / `stop_video` | Start/stop the WebRTC camera stream |
| `take_photo` | Capture a still image |
| `estop` | Emergency stop |
| `battery_check` / `get_status` | Request battery / cached sensor status |
| `trajectory` | Multi-point joint trajectory (advanced) |
| `goto` / `path` | Nav2 navigate-to-point / follow-waypoints |

> The legacy `cmd_vel` and `led_ctrl` commands are **deprecated** — use
> `velocity_command`/`actuation` and `lights` instead.

**Units:** linear velocity in **m/s**, angular velocity in **rad/s**, joint
angles in **radians**, LED brightness as an **integer 0–255**.

## Telemetry (Robot → Cloud)

The bridge publishes rover state upstream automatically, **rate-limited** (default
1 Hz) to keep bandwidth and cloud cost low while staying smooth enough for live
monitoring. Watch it live:

```shell
# Everything for one twin (both twin/ and joint/ scopes)
mosquitto_sub -h mqtt.cyberwave.com -p 1883 -u mqttcyb -P "$CYBERWAVE_API_KEY" \
  -t "cyberwave/+/$TWIN_UUID/#" -v
```

| MQTT topic | Source | Rate |
| --- | --- | --- |
| `cyberwave/joint/{uuid}/update` | `/ugv/joint_states` (wheels + pan-tilt) | 5 Hz |
| `cyberwave/pose/{uuid}/update` | internal odometry (dead-reckoned) | 1 Hz |
| `cyberwave/twin/{uuid}/status/imu` | `/ugv/imu` | 1 Hz |
| `cyberwave/twin/{uuid}/battery_check/status` | cached `/ugv/battery_status` | on request |
| `cyberwave/twin/{uuid}/edge_health` | internal health monitor | periodic |
| `cyberwave/twin/{uuid}/{command}/status` | command acknowledgements | on command |

## Multi robot support

So several UGV Beasts can share one ROS 2 graph, the whole driver runs under a per-robot namespace derived from the twin UUID:

```shell
ugv_beast_<first 6 hex chars of the twin uuid>      # e.g.  ugv_beast_27dca7
```

MQTT topics are unaffected (they stay keyed by the full twin UUID). Set `CYBERWAVE_TWIN_UUID` and the namespace is applied to both the hardware nodes and the bridge automatically; override with `CYBERWAVE_ROS_NAMESPACE` if you need a specific name. With the variable unset, topics are global (`/cmd_vel`, …) for single-robot development.

## Development

Working on the driver on the rover itself? A few helper scripts live under
`scripts/` (and `scripts/ugv_beast/`):

```shell
# Bring up the full stack in a managed tmux session (attach logs)
./scripts/ugv_beast/start_ugv.sh --logs

# Clean-rebuild just the MQTT bridge and run it
./scripts/clean_build_mqtt.sh --logs

# Install the driver as a boot-time systemd service
sudo ./scripts/ugv_services_install.sh
```

Build-from-source, SSH access and the launch-file internals are covered in
[docs/edge-ros-ugv-beast-setup/UGV-Beast-conf.md](docs/edge-ros-ugv-beast-setup/UGV-Beast-conf.md).

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| Commands ignored | Ensure the payload has `"source_type": "tele"`. |
| Twin not updating | Confirm `CYBERWAVE_TWIN_UUID` matches your twin, and the twin's asset has the matching capabilities enabled. |
| Rover stops on its own | Expected — `duration_ms` / the 0.5 s watchdog. Resend commands while driving. |
| Garbled JSON / `KeyError: 'gx'` / "device busy" | **Serial contention** — two processes on `/dev/ttyAMA0`. Run only `master_beast.launch.py`, never the manual driver *and* a launch file at once. |
| Camera not streaming | Check WebRTC config; the WebRTC `sensor` is the twin's `capabilities.sensors[].id` (from the twin JSON) — confirm the twin declares an rgb sensor. Log line: `camera sensor=<id> … frame_id=…`. |
| Bridge won't connect | Verify `CYBERWAVE_API_KEY`; check logs with `cyberwave edge logs` (Path A) or `docker logs -f cyb_ugv_beast` (Path B). |

Enable debug logging:

```shell
ros2 launch mqtt_bridge mqtt_bridge.launch.py log_level:=debug
```

## Documentation & References

**Cyberwave**
- 🚀 **[UGV Beast first-config guide](https://cyberwave.com/waveshare/ugv-beast)** — start here (flash, network, pair).
- 🖥️ **[Cyberwave Edge with the CLI](https://docs.cyberwave.com/feature-reference/edge/overview#use-cyberwave-edge-with-the-cli)** — managed install & operation.
- 🐍 **[Cyberwave Python SDK](https://docs.cyberwave.com/overview/tools/python-sdk#python-sdk)** — control twins from code.
- 📚 [docs.cyberwave.com](https://docs.cyberwave.com) — full platform docs.

**This driver (in-repo deep reference)**
- [docs/edge-ros-ugv-beast-setup/edge-ros-ugv-beast.md](docs/edge-ros-ugv-beast-setup/edge-ros-ugv-beast.md) — complete MQTT command & telemetry reference.
- [docs/edge-ros-ugv-beast-setup/UGV-Beast-conf.md](docs/edge-ros-ugv-beast-setup/UGV-Beast-conf.md) — from-source build, SSH, and launch setup.
- [docs/ADAPTIVE_CAMERA.md](docs/ADAPTIVE_CAMERA.md) — adaptive camera / WebRTC streaming notes.

**Waveshare (upstream hardware)**
- [UGV Beast PI ROS2 wiki](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2) — official robot setup.
- [`waveshareteam/ugv_ws`](https://github.com/waveshareteam/ugv_ws) — upstream ROS 2 stack we build on.

## Contributing

Contributions are welcome! Please open an issue for bugs or feature requests, and
submit a pull request with improvements. When reporting a problem, include your
driver version, the `cyberwave edge logs` output, and the command payload you sent.

## Support

- 📚 Documentation: [docs.cyberwave.com](https://docs.cyberwave.com)
- 💬 Community: [Discord](https://discord.gg/dfGhNrawyF)
- 🐛 Issues: [github.com/cyberwave-os/cyberwave-edge-ros-ugv/issues](https://github.com/cyberwave-os/cyberwave-edge-ros-ugv/issues)

## License

This project is licensed under the Apache 2.0 License — see the [LICENSE](https://opensource.org/licenses/Apache-2.0) file for details.
