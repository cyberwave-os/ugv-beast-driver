<p align="center">
  <a href="https://cyberwave.com">
    <img src="https://cyberwave.com/cyberwave-logo-black.svg" alt="Cyberwave logo" width="240" />
  </a>
</p>

# Cyberwave UGV Driver (ROS 2 MQTT Bridge)

This module is part of **Cyberwave: Making the physical world programmable**.

[![License](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](https://opensource.org/licenses/Apache-2.0)
[![Documentation](https://img.shields.io/badge/Documentation-docs.cyberwave.com-orange)](https://docs.cyberwave.com)
[![Discord](https://badgen.net/badge/icon/discord?icon=discord&label&color=orange)](https://discord.gg/dfGhNrawyF)
[![Docker Build](https://github.com/cyberwave-os/cyberwave-edge-ros-ugv/actions/workflows/push-to-docker-hub.yml/badge.svg)](https://github.com/cyberwave-os/cyberwave-edge-ros-ugv/actions/workflows/push-to-docker-hub.yml)

A bidirectional bridge between ROS 2 topics and MQTT, with integrated support for the Cyberwave digital twin platform. This bridge enables real-time synchronization between physical robots and cloud-based digital twins.

## Features

- **Bidirectional communication**: ROS 2 to MQTT in both directions
- **Cyberwave SDK integration**: Native support for Cyberwave digital twins
- **Rate limiting**: Intelligent upstream traffic control (100 Hz to 1 Hz)
- **Joint state mapping**: Automatic joint name transformation
- **Configurable**: Environment variables and YAML configuration
- **WebRTC streaming**: Integrated ROS 2 image to WebRTC bridge (see **[WEBRTC_STREAMING.md](WEBRTC_STREAMING.md)**)
- **Source type filtering**: Downstream filtering to only process `source_type: "tele"` messages

## Quick Start

- **[UGV Beast Quickstart](README_UGV_QUICKSTART.md)**: Simple guide for UGV video streaming and teleoperation
- **UGV helper scripts**: See [UGV Beast Helper Scripts](#ugv-beast-helper-scripts) for automated startup and building
- **General setup**: See below for detailed configuration

### Docker build layering (for faster SDK bumps)

The Docker image now supports a reusable base stage so the slow dependency install is not repeated on every SDK bump.

- Build/publish base stage once:

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --target ugv_driver_base \
  -f docker-conf/Dockerfile \
  -t cyberwaveos/ugv-driver:base-dev \
  --push .
```

- Build runtime image using that base:

```bash
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  -f docker-conf/Dockerfile \
  --build-arg UGV_DRIVER_BASE_IMAGE=cyberwaveos/ugv-driver:base-dev \
  --build-arg CYBERWAVE_SDK_VERSION=0.5.0 \
  -t cyberwaveos/ugv-driver:dev \
  --push .
```

Notes:

- `CYBERWAVE_SDK_VERSION` is isolated in its own layer (`runtime_base`) so SDK-only bumps avoid redoing heavy apt/WebRTC dependency layers.
- `ugv_driver_base` includes layers through the "Install Additional System Packages" and stable Python dependency install stages.

### Build & publish the `:staging` image

Build the full runtime image and tag it `:staging`. Run from the
**`cyberwave-edge-nodes`** parent directory so the shared `cyberwave-edge-common`
package is in the build context. On a Raspberry Pi (native `arm64`):

```bash
cd cyberwave-edge-nodes

docker buildx build \
  --platform linux/arm64 \
  -f cyberwave-edge-ros-ugv/docker-conf/Dockerfile \
  --build-arg CYBERWAVE_SDK_VERSION=0.5.0 \
  -t cyberwaveos/ugv-driver:staging \
  --load \
  .
```

Then publish it (requires push credentials for the target registry):

```bash
docker login -u cyberwaveos          # paste the Docker Hub access token when prompted
docker push cyberwaveos/ugv-driver:staging
```

Notes:

- `--load` places the image in the local Docker daemon so `docker push` can upload it; for a one-step build+push, replace `--load` with `--push`.
- `:staging` is a **mutable** tag — every staging edge re-pulls it on the next driver (re)start, so publishing updates the whole staging fleet. After publishing, restart the driver on the device: `sudo systemctl restart cyberwave-edge-core`.
- The build is several GB; ensure adequate free disk before building on an SD-card device. Most of that is the upstream `dudulrx0601/ugv_rpi_ros_humble` base image (~3 GB compressed / ~7 GB on disk) — which we cannot shrink from our Dockerfile — so our own layers add only ~40 MB on top.
- **Image footprint:** only the packages the driver actually launches are built — `ugv_bringup`, `ugv_description`, `ugv_vision`, `ugv_interface`, `ugv_base_node`, `ldlidar`, and `mqtt_bridge`. The unused upstream nav/SLAM/gazebo/vizanti/apriltag stack is neither built nor shipped.
- **Slimming is done *in-layer*, on purpose.** Deleting files in a later layer reclaims nothing — the bytes still ship in the lower layer (this is why an image whose workspace *looks* like 23 MB can still be 7.4 GB). So each artifact is removed in the same `RUN` that creates it: STAGE 7 clones with `--depth 1` and removes `.git` (~150 MB) + unused-package source right there; each colcon stage drops its own `build/`/`log/` scratch after building (`--symlink-install` packages are first materialized into real files via `/usr/local/bin/materialize_symlinks.sh`, since their `install/` symlinks point into `build/`). STAGE 14 is only a fail-hard integrity guard (aborts if any dangling `install/` symlink remains). To re-enable a package, add it to STAGE 10b's `--packages-select` and drop it from the STAGE 7 prune lists.

### 1. Set Up Environment

Create a `.env` file in the workspace root:

```bash
# Cyberwave credentials
CYBERWAVE_TOKEN=your_api_token_here
CYBERWAVE_MQTT_BROKER=mqtt.cyberwave.com
CYBERWAVE_MQTT_PORT=1883

# Rate limiting (1 Hz = 1 second between publishes)
MQTT_PUBLISH_RATE_LIMIT=1.0

# UGV velocity limits (these are CAPS, not the robot's mechanical max ~1.2 m/s)
# MAX_*  : hard ceiling — every teleop command is clamped to this.
# DEFAULT_*: cruise speed used by the discrete keyboard commands (move_forward, ...).
# Raise MAX_LINEAR toward ~1.2 for a faster rover; lower it for tight spaces.
CYBERWAVE_UGV_MAX_LINEAR_SPEED=0.8
CYBERWAVE_UGV_MAX_ANGULAR_SPEED=1.0
CYBERWAVE_UGV_DEFAULT_LINEAR_SPEED=0.5
CYBERWAVE_UGV_DEFAULT_ANGULAR_SPEED=1.0
```

> **Speed:** these env vars are the only software speed cap in the path
> (`MQTT → /cmd_vel → STM32 T:13`). The compiled default is now
> `MAX_LINEAR=0.8 m/s` (≈2/3 of the rover's ~1.2 m/s mechanical max); set the env
> var per deployment to go faster/slower. Higher speed means less reaction time —
> the movement watchdog auto-stops after 0.5 s of command silence.

The bridge consumes `locomotion.velocity_command.v1` for UGV movement. Legacy
commands such as `move_forward` and `turn_left` are thin compatibility wrappers
that emit the canonical velocity command before mapping to `/cmd_vel`. The edge
keeps a timed stop as a local safety watchdog; backend timed execution remains
the primary command-duration owner.

#### Twin command payloads on `cyberwave/twin/{uuid}/command`

Two shapes route through the `actuation` handler:

1. **Discrete command** (keyboard teleop). `command` is the binding's actuation;
   per-command params (camera/lights/video) live under `data`:

   ```json
   { "source_type": "tele", "command": "move_forward", "data": {}, "timestamp": 1750000000.1 }
   ```

   Movement actuations drive at the `DEFAULT_*` cruise speed; the 0.5 s movement
   watchdog auto-stops on command silence.

2. **Explicit analog velocity** (top-level `velocity_command`). When a message
   carries a `velocity_command` object it is *authoritative* — the explicit
   kinematics are honored regardless of the `command` string, and the
   `duration_ms` deadman (timed stop) is armed instead of the keyboard watchdog:

   ```json
   {
     "source_type": "tele",
     "command": "move_forward",
     "timestamp": 1750000000.1,
     "velocity_command": { "linear_x": 0.6, "linear_y": 0.0, "angular_z": 0.4, "duration_ms": 300 }
   }
   ```

   The `velocity_command` may be the full `locomotion.velocity_command.v1`
   envelope or this *loose* form (just `linear_x`/`linear_y`/`angular_z`/
   `duration_ms`); the missing `contract`/`gait`/`origin` default to the
   locomotion contract / `walk` / `teleop`. `linear_y` is ignored on the tracked
   UGV Beast, and `linear_x`/`angular_z` are still clamped to the `MAX_*` ceiling.
   Senders may also set `command` to `velocity_command` or `locomotion_velocity`
   with the same object — all routes converge on the locomotion handler.


```bash
# or with pip (for virtualenvs)
python3 -m pip install --user paho-mqtt pyyaml

# or with uv
uv pip install paho-mqtt pyyaml numpy cyberwave
```

### 2. Build the Package

```bash
cd /home/ws/ugv_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mqtt_bridge
source install/setup.bash
```

### 3. Launch the Bridge

```bash
./launch_bridge_with_env.sh
```

## UGV Beast Helper Scripts

For the UGV Beast model, pre-configured scripts are available in `scripts/ugv_beast/` to simplify environment setup and development:

### 1. Full Environment Startup
Automatically cleans up existing processes and starts the entire UGV stack (Bringup, Driver, Vision, BaseNode, and MQTT Bridge) in a managed `tmux` session with separate windows for each component.

```bash
# Start everything
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/start_ugv.sh

# Start everything and automatically attach to logs
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/start_ugv.sh --logs

# Start hardware only (skip MQTT bridge for manual execution)
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/start_ugv.sh --no-bridge
```
*   **View processes**: `tmux attach -t ugv_env`
*   **Navigation**: Use `Ctrl+B` then `n`/`p` to switch between windows.

### 2. MQTT Bridge Clean Build & Run
Removes existing build/install artifacts for the bridge, performs a fresh `colcon build`, and runs the node with the default UGV parameters.

```bash
# Rebuild and run (logs shown by default)
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh

# Rebuild and run with explicit logs flag
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh --logs

# Rebuild ONLY (do not run)
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh --no-run
```

---

You should see:

```
Starting MQTT Bridge with Cyberwave SDK...
  MQTT Broker: mqtt.cyberwave.com
  MQTT Port: 1883
  Token: abc12345... (masked)
  Rate Limit (ROS->MQTT): 1.0s between publishes (1.00 Hz)

[mqtt_bridge_node]: ROS->MQTT rate limiting enabled: 1.00s between publishes (1.00 Hz)
```

## ROS 2 Namespacing (multi-robot / fleet)

So that several UGV Beasts can share one ROS 2 graph without their nodes and
topics colliding, the whole driver runs under a **per-robot ROS 2 namespace
derived from the twin UUID**:

```
ugv_beast_<first 6 hex chars of the twin uuid>      e.g.  ugv_beast_27dca7
```

A robot whose twin UUID is `27dca72f-6e17-…` therefore exposes:

```
/ugv_beast_27dca7/cmd_vel
/ugv_beast_27dca7/ugv/led_ctrl
/ugv_beast_27dca7/ugv/oled_ctrl
/ugv_beast_27dca7/image_raw
/ugv_beast_27dca7/ugv/joint_states
```

Why the `ugv_beast_` prefix: a raw UUID is **not** a valid ROS 2 name — names may
contain only `[A-Za-z0-9_]` and may not start with a digit, while a UUID contains
`-` and often starts with a digit. The prefix guarantees a valid leading letter;
the 6-char hex suffix keeps it unique across a fleet.

How it is applied (both sides must match, or topics won't connect):

- **Hardware nodes** (`ugv_integrated_driver`, `base_node`, `robot_state_publisher`,
  `usb_cam`, …) are launched under the namespace by `master_beast.launch.py`,
  which reads `CYBERWAVE_TWIN_UUID` and computes `ugv_beast_<first6>`.
- **The MQTT bridge** prefixes every ROS topic it publishes/subscribes via the
  same namespace (`mqtt_bridge.ros_topic_namespace`), passed in by the launch as
  the `ros_namespace` parameter.

The two derivations are kept identical on purpose. MQTT topics are unaffected —
they remain keyed by the full twin UUID (`cyberwave/twin/{twin_uuid}/…`).

**Single-robot / dev:** when `CYBERWAVE_TWIN_UUID` is unset the namespace is empty
and topics are global (`/cmd_vel`, `/ugv/oled_ctrl`, …) exactly as before.

**Override:** set `CYBERWAVE_ROS_NAMESPACE` to a valid ROS 2 name to force a
specific namespace (used for both the hardware nodes and the bridge):

```bash
export CYBERWAVE_ROS_NAMESPACE=ugv_beast_lab1
```

## Source Type Filtering

The MQTT bridge implements downstream filtering to ensure only teleoperation commands reach the physical robot:

**Downstream Filtering (MQTT → ROS)**:
- **Processes**: Only messages with `source_type: "tele"` (from frontend live mode)
- ❌ **Ignores**: Messages with `source_type: "edit"`, `"sim"`, or `"edge"`

**Purpose**: Prevents accidental commands from editor or simulator modes from reaching physical robots.

**Upstream Messages (ROS → MQTT)**:
- All upstream messages are tagged with `source_type: "edge"` automatically
- No filtering applied (all edge messages are sent to cloud)

**Implementation**: See `mqtt_bridge_node.py` → `_handle_mqtt_message()` method.

See [cyberwave-backend/docs/SOURCE_TYPE_LOGIC.md](../../../cyberwave-backend/docs/SOURCE_TYPE_LOGIC.md) for complete source type documentation.

## Data Conventions & Units

The bridge follows standard ROS 2 conventions for units and message structures. To avoid confusion, distinguish between **Upstream** and **Downstream** traffic.

### Terminology
*   **Upstream (ROS → MQTT)**: Data flowing from the physical robot or edge device to the Cyberwave cloud (telemetry, status, odometry).
*   **Downstream (MQTT → ROS)**: Commands flowing from the Cyberwave cloud/frontend to the physical robot (movement, IO toggles).

### Standard Units
| Data Type | Unit | Description |
| :--- | :--- | :--- |
| Linear Velocity | meters per second (**m/s**) | Used in `/cmd_vel`. |
| Angular Velocity | radians per second (**rad/s**) | Used in `/cmd_vel` (Z-axis for steering). |
| Joint Position | radians (**rad**) | Standard for revolute joints. |
| Joint Position | meters (**m**) | Standard for prismatic joints. |
| LED Brightness | integer (**0-255**) | 8-bit intensity (e.g., UGV Beast headlights). |

### Command Formats
Note the difference between ROS-native structures and task-specific parameters:

1.  **Twist Messages (`cmd_vel`)**:
    *   **Format**: Uses dictionaries for `linear` and `angular` components.
    *   **Example**: `{"linear": {"x": 0.5, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 1.0}}`
2.  **Teleop Bindings**:
    *   **Format**: Keyboard/Joystick bindings often use **scalars** (e.g., `linear: 1.0`) to represent normalized intensity or direction. These are processed by the controller and scaled by a "max speed" factor before being sent to the robot as the full dictionary format above.

## Upstream Message Flow

### From Robot to Cloud

The bridge handles high-frequency robot data and converts it to efficient cloud updates:

```
UR7e Robot Controller
      ↓ 100 Hz (/joint_states topic)
      └─ Configured in ur_controllers.yaml
      └─ state_publish_rate: 100.0
      └─ action_monitor_rate: 20.0
      
MQTT Bridge (subscribes)
      ↓ Receives: 100 Hz
      ↓ 
Rate Limiter (1.0 second threshold)
      ↓ Filters: 99% of messages dropped
      ↓ Publishes: 1 Hz
      
Cyberwave MQTT Broker
      ↓ Receives: 1 Hz
      
Digital Twin Frontend
      └─ Updates: 1 Hz (smooth visualization)
```

**Bandwidth Reduction:** 100 Hz → 1 Hz = **99% reduction** in network traffic!

### Robot Publishing Frequencies

**Configured in:** `src/Universal_Robots_ROS2_Driver/ur_robot_driver/config/ur_controllers.yaml`

#### state_publish_rate: 100 Hz
- **What it does**: Controls how often `/joint_states` topic is published
- **Frequency**: 100 times per second (every 10 milliseconds)
- **Why 100 Hz**: Provides smooth visualization and responsive control locally

#### action_monitor_rate: 20 Hz  
- **What it does**: Controls how often trajectory execution status is monitored
- **Frequency**: 20 times per second (every 50 milliseconds)
- **Why 20 Hz**: Sufficient for monitoring trajectory completion

**Why these rates?** The 100 Hz rate is necessary for responsive local robot control and visualization, but **NOT necessary for cloud monitoring**. That's why we apply rate limiting.

## Rate Limiting Configuration

### Default: 1 Hz (1 second)

The MQTT bridge limits upstream traffic to **1 Hz by default**, which is optimal for cloud monitoring because:

1. **Human visualization**: Cannot perceive differences faster than ~10-20 Hz
2. **Network efficiency**: 99% bandwidth reduction (100 Hz → 1 Hz)
3. **Server load**: Reduces database writes and processing
4. **Cost optimization**: Less data transfer = lower cloud costs
5. **Battery/power**: Less transmission = longer operation on edge devices

### Configurable Rate Limit

You can adjust this rate based on your needs:

```bash
# More frequent updates (2 Hz = every 0.5 seconds)
export MQTT_PUBLISH_RATE_LIMIT=0.5

# Default: 1 Hz (every 1 second) - RECOMMENDED
export MQTT_PUBLISH_RATE_LIMIT=1.0

# Conservative (0.5 Hz = every 2 seconds)
export MQTT_PUBLISH_RATE_LIMIT=2.0

# Minimal (0.2 Hz = every 5 seconds)
export MQTT_PUBLISH_RATE_LIMIT=5.0

# Disable rate limiting (NOT recommended for cloud)
export MQTT_PUBLISH_RATE_LIMIT=0
```

**Where to set:**
- Environment variable: `MQTT_PUBLISH_RATE_LIMIT` (in `.env` file)
- Config file: `src/mqtt_bridge/config/params.yaml`
- Runtime: `ros2 param set /mqtt_bridge_node ros2mqtt_rate_limit 1.0`

### Recommended Values by Use Case

| Use Case | Rate Limit | Frequency | Reasoning |
|----------|------------|-----------|-----------|
| Production monitoring | 1.0s | 1 Hz | **Recommended** - Balanced |
| Development/debugging | 0.5s | 2 Hz | More responsive |
| Slow operations | 2.0s | 0.5 Hz | Maximum efficiency |
| Demo/visualization | 0.5s | 2 Hz | Smoother updates |
| Minimal bandwidth | 5.0s | 0.2 Hz | Extreme savings |

## Configuration Files

### MQTT Bridge Configuration

**File:** `src/mqtt_bridge/config/params.yaml`

```yaml
/mqtt_bridge_node:
  ros__parameters:
    broker:
      host: mqtt.cyberwave.com
      port: 1883
      # Auth: username is the public default "mqttcyb"; the password is your
      # Cyberwave API key. Do NOT hardcode it — set CYBERWAVE_API_KEY in the env
      # (edge-core injects it) and leave these blank so the SDK/token path wins.
      username: "mqttcyb"
      password: ""
      use_cyberwave: true
    
    # Robot Mapping (MANDATORY)
    # Options: 'robot_arm_v1' or 'robot_ugv_beast_v1'
    robot_id: 'robot_ugv_beast_v1'
    
    # Rate limiting
    ros2mqtt_rate_limit: 1.0  # 1 Hz (1 second between publishes)
    
    bridge:
      ros2mqtt:
        ros_topics:
          - /joint_states
        topics:
          /joint_states:
            mqtt_topic: "cyberwave/joint/{twin_uuid}/update"
            sdk_method: update_joint_state
            type: sensor_msgs/JointState
```

### Robot Joint Mapping

Robot mapping allows the bridge to correctly transform joint names and route commands.

**1. Set Robot ID in `config/params.yaml`:**

```yaml
# Options: 'robot_arm_v1', 'robot_ugv_beast_v1', or 'default'
robot_id: 'robot_ugv_beast_v1'
```

**2. Configure Mapping File:**

Mapping files are located in `config/mappings/{robot_id}.yaml`. They define the `twin_uuid` and joint name transformations.

**Example: `config/mappings/default.yaml`**
```yaml
version: 1
robot_id: "default"
format: "json_by_name"
metadata:
  twin_uuid: "00000000-0000-0000-0000-000000000000"

joints: [] # No remapping by default

capabilities:
  upstream_mode: "joint"
```

**Example: `config/mappings/robot_arm_v1.yaml`**
```yaml
version: 1
robot_id: "robot_arm_v1"
format: "json_by_name"
metadata:
  twin_uuid: "your-twin-uuid-here"

joints:
  - ros_name: "shoulder_pan_joint"
    mqtt_name: "shoulder_pan_joint"
    transform: {scale: 1.0, offset: 0.0}
  - ros_name: "ee_fixed_joint"
    mqtt_name: "ee_fixed_joint"

capabilities:
  upstream_mode: "joint"

# Robot-agnostic IO configuration
io_configuration:
  tool0:
    enabled: true
    joint_name: "ee_fixed_joint"
    service_name: "/io_and_status_controller/set_io"
```

**Example: `config/mappings/robot_ugv_beast_v1.yaml`**
```yaml
version: 1
robot_id: "robot_ugv_beast_v1"
metadata:
  twin_uuid: "your-twin-uuid-here"

# Pluggable command registry
command_registry: "mqtt_bridge.plugins.ugv_beast_command_handler.CommandRegistry"

# Generic internal odometry
internal_odometry:
  enabled: true
  track_width: 0.23
  wheel_radius: 0.04
  left_wheel_joints: ["left_up_wheel_link_joint", "left_down_wheel_link_joint"]
  right_wheel_joints: ["right_up_wheel_link_joint", "right_down_wheel_link_joint"]

# Camera streaming configuration
# camera_name must match a sensor ID on the Cyberwave twin's asset schema.
# It is required for recordings to appear in the Replay tab; without it
# the media service cannot associate the stream with a sensor and skips recording.
camera:
  image_topic: "/image_raw"
  format: "yuv420p"
  camera_name: "default"  # Must match a sensor ID on the twin's asset schema

capabilities:
  upstream_mode: "pose"
```

## Adding a New Robot from Scratch

To integrate a new ROS 2 robot into the Cyberwave platform, follow these steps:

### 1. Create a Mapping File
Create a new YAML file in `config/mappings/your_robot_id.yaml`.

```yaml
version: 1
robot_id: "your_robot_id"
metadata:
  twin_uuid: "GET-FROM-CYBERWAVE-DASHBOARD"

# Define how ROS joints map to Digital Twin joints
joints:
  - ros_name: "base_to_arm_joint"
    mqtt_name: "shoulder_joint"
    transform: {scale: 1.0, offset: 0.0}

capabilities:
  upstream_mode: "joint" # "joint", "pose", or "both"
```

### 2. (Optional) Create a Command Handler
If your robot has custom ROS services or actions (e.g., a gripper, a sprayer, or specialized LEDs), create a new plugin:
1.  Create `mqtt_bridge/plugins/your_robot_handler.py`.
2.  Inherit from `CommandRegistry` and implement your logic.
3.  Register it in your mapping YAML:
    ```yaml
    command_registry: "mqtt_bridge.plugins.your_robot_handler.YourClassName"
    ```

### 3. (Optional) Enable Plugins
*   **Internal Odometry**: If your robot doesn't publish `/odom`, enable the `internal_odometry` block in your YAML.
*   **Navigation**: If using Nav2, ensure your robot has the standard Nav2 action servers running.
*   **Camera**: Add a `camera` block to configure WebRTC streaming. Include `camera_name` to match a sensor ID on the twin's asset schema — this is required for recordings to appear in the Replay tab.

### 4. Launch
Run the bridge with your new `robot_id`:
```bash
ros2 run mqtt_bridge mqtt_bridge_node --ros-args -p robot_id:=your_robot_id
```

## Monitoring

### Check Robot Publishing Rate

```bash
ros2 topic hz /joint_states
```

Expected: `average rate: 100.000`

### Check Bridge Rate Limit

```bash
ros2 param get /mqtt_bridge_node ros2mqtt_rate_limit
```

Expected: `Double value is: 1.0`

### Monitor MQTT Traffic

```bash
mosquitto_sub -h mqtt.cyberwave.com -t '#' -v | ts '[%H:%M:%S]'
```

Should see messages ~1 second apart (1 Hz)

### Enable Debug Logging

```bash
ros2 launch mqtt_bridge mqtt_bridge.launch.py log_level:=debug
```

Look for rate limiting messages:
```
[DEBUG] Rate limit: Skipping ROS->MQTT publish for '/joint_states' 
        (only 0.234s since last publish, limit is 1.000s)
```

## Performance Impact

### Bandwidth Reduction

| Stage | Frequency | Messages/sec | Bandwidth |
|-------|-----------|--------------|-----------|
| UR7e → ROS Topic | 100 Hz | 100 | ~50 KB/s |
| MQTT Bridge (after filter) | 1 Hz | **1** | **~0.5 KB/s** |

**Savings:** 99% reduction in network traffic

### Cost Savings (Example)

Assuming $0.10/GB data transfer:
- **Without rate limiting**: $156/year per robot
- **With 1 Hz rate limiting**: $1.56/year per robot
- **Savings**: $154/year per robot (99% reduction)

## Troubleshooting

### Digital Twin Not Updating

```bash
# Check if rate limiting is too aggressive
ros2 param get /mqtt_bridge_node ros2mqtt_rate_limit

# Temporarily disable
export MQTT_PUBLISH_RATE_LIMIT=0
./launch_bridge_with_env.sh
```

### Too Much Network Traffic

```bash
# Increase rate limit interval
export MQTT_PUBLISH_RATE_LIMIT=2.0  # 0.5 Hz
./launch_bridge_with_env.sh
```

### Bridge Not Connecting

```bash
# Check credentials
echo $CYBERWAVE_TOKEN

# Check logs
ros2 launch mqtt_bridge mqtt_bridge.launch.py log_level:=debug
```

## Testing

### Test Rate Limiting

```bash
# Start bridge
./launch_bridge_with_env.sh

# In another terminal, run test script
./test_rate_limiting.sh
```

### Manual Testing

```bash
# Publish test messages rapidly
for i in {1..10}; do
  ros2 topic pub --once /ping/ros std_msgs/msg/String "data: 'test $i'"
  sleep 0.3
done

# Only ~3 messages should appear in MQTT (with 1 Hz rate limit)
```

## Architecture

The bridge is designed to be fully general-purpose and robot-agnostic. All robot-specific behaviors are driven by the mapping `.yaml` file and pluggable modules.

### Components

1.  **Core Node (`mqtt_bridge_node.py`)**: Orchestrates MQTT connectivity and delegates logic to specialized modules.
2.  **Telemetry (`telemetry.py`)**: Manages joint state accumulation and high-frequency feedback processing.
3.  **Health (`health.py`)**: Handles periodic heartbeats and system status reporting.
4.  **Plugins (`plugins/`)**:
    *   **Command Registry**: Dynamically loaded logic for specialized hardware commands (e.g., `ugv_beast_command_handler.py`).
    *   **Internal Odometry**: Dead-reckoning logic for robots without native odometry.
    *   **Navigation Bridge**: Integration with the ROS 2 Navigation Stack (Nav2).
    *   **ROS Camera**: WebRTC video streaming from ROS Image topics.
5.  **Mapping System (`mapping.py`)**: Precomputes joint name transformations and coordinate transforms.
6.  **Cyberwave Adapter**: SDK wrapper for cloud integration.

### Robot-Agnostic Design

#### 1. Dynamic Command Registry Loading
The bridge does not hardcode command logic. Instead, the mapping file specifies a pluggable command registry class that is dynamically loaded.
*   **Example (`robot_ugv_beast_v1.yaml`)**:
    ```yaml
    command_registry: "mqtt_bridge.plugins.ugv_beast_command_handler.CommandRegistry"
    ```

#### 2. Generic Internal Odometry
Physical constants like wheel radius and track width are moved out of the code and into the mapping.
*   **Example (`robot_ugv_beast_v1.yaml`)**:
    ```yaml
    internal_odometry:
      enabled: true
      track_width: 0.23
      wheel_radius: 0.04
      left_wheel_joints: ["left_up_wheel_link_joint", "left_down_wheel_link_joint"]
      right_wheel_joints: ["right_up_wheel_link_joint", "right_down_wheel_link_joint"]
    ```

#### 3. Configurable IO and Tool Control
The bridge supports a generic `io_configuration` system to manage robot tools (like grippers) with custom service types and activation thresholds.

#### 4. Robot Specific Implementation Examples
*   **UR7 / UR Robots**: Uses `robot_arm_v1.yaml` to toggle tools based on virtual joint positions.
*   **UGV Beast**: Uses `robot_ugv_beast_v1.yaml` to enable internal odometry and load its specialized command registry for peripherals (LEDs, Pan-Tilt, etc.).
*   **Future Robots**: New robots can be added by simply creating a mapping file and defining the appropriate registry, odometry, and IO configurations without modifying the bridge source code.

### Message Flow

```
┌────────────────────────────────────────────────────────────┐
│                  MQTT BRIDGE ARCHITECTURE                   │
└────────────────────────────────────────────────────────────┘

ROS 2 Topics                 MQTT Bridge                MQTT Broker
     │                            │                           │
     │  /joint_states (100 Hz)    │                           │
     ├───────────────────────────►│                           │
     │                       ┌────▼─────┐                     │
     │                       │  Rate    │                     │
     │                       │  Limiter │                     │
     │                       │  (1 Hz)  │                     │
     │                       └────┬─────┘                     │
     │                            │                           │
     │                       ┌────▼─────┐                     │
     │                       │ Cyberwave│                     │
     │                       │ Adapter  │                     │
     │                       └────┬─────┘                     │
     │                            │   (1 Hz)                  │
     │                            ├──────────────────────────►│
     │                            │                           │
     │  (downstream commands)     │                           │
     │◄───────────────────────────┤                           │
     │                            │◄──────────────────────────┤
```

## Documentation

### Core Documentation
- **[docs/COMMUNICATION_AND_SIGNALING.md](docs/COMMUNICATION_AND_SIGNALING.md)** - Critical architecture: WebRTC signaling, MQTT Client IDs, and Source Type enforcement.
- **[WEBRTC_STREAMING.md](WEBRTC_STREAMING.md)** - Integrated ROS 2 WebRTC streaming
- **[UPSTREAM_MESSAGE_FLOW.md](../../UPSTREAM_MESSAGE_FLOW.md)** - Complete upstream flow explanation
- **[RATE_LIMITING.md](../../RATE_LIMITING.md)** - Detailed rate limiting documentation
- **[RATE_LIMITING_QUICKSTART.md](../../RATE_LIMITING_QUICKSTART.md)** - Quick reference guide

### Configuration
- **[JOINT_STATES_RATE_CONFIG.md](../../JOINT_STATES_RATE_CONFIG.md)** - Robot rate configuration
- **[SOURCE_TYPE_FILTERING.md](../../SOURCE_TYPE_FILTERING.md)** - Upstream vs downstream filtering

### Implementation
- **[RATE_LIMITING_IMPLEMENTATION.md](../../RATE_LIMITING_IMPLEMENTATION.md)** - Technical details
- **[SDK_INTEGRATION_FIXED.md](../../SDK_INTEGRATION_FIXED.md)** - Cyberwave SDK integration

## Key Points

- **Robot publishes at 100 Hz** - necessary for local control and visualization
- **Bridge filters to 1 Hz** - efficient for cloud monitoring
- **Rate limit is configurable** - adjust via environment variable or config file
- **99% bandwidth reduction** - significant cost savings
- **1 Hz is optimal for cloud** - smooth enough for human monitoring
- **No impact on local performance** - rate limiting only affects MQTT traffic

## Contributing

Contributions are welcome. Please open an issue for bugs or feature requests, and submit a pull request with improvements.

## Support

For issues or questions:
1. Check the documentation files listed above
2. Enable debug logging: `log_level:=debug`
3. Review bridge logs for error messages
4. Test with rate limiting disabled temporarily

- Documentation: https://docs.cyberwave.com
- Community (Discord): https://discord.gg/dfGhNrawyF
- Issues: https://github.com/cyberwave-os/cyberwave-edge-ros-ugv/issues

## License

See workspace root for license information.
