# UGV Beast ROS 2 + MQTT Bridge Documentation

This documentation covers the complete setup and integration of the Waveshare UGV Beast with the Cyberwave MQTT Bridge for digital twin synchronization.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [MQTT Bridge Components](#mqtt-bridge-components)
- [Setup Guide](#setup-guide)
- [Configuration](#configuration)
- [MQTT Commands Reference](#mqtt-commands-reference)
- [MQTT Status Topics (Responses)](#mqtt-status-topics-responses)
- [Helper Scripts](#helper-scripts)
- [Troubleshooting](#troubleshooting)

---

## Architecture Overview

The UGV Beast MQTT integration follows a bidirectional bridge architecture that connects the physical robot to a cloud-based digital twin via MQTT.

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         CYBERWAVE DIGITAL TWIN PLATFORM                      │
│                                                                              │
│  ┌─────────────┐    ┌─────────────────┐    ┌─────────────────────────────┐  │
│  │  Frontend   │◄──►│  MQTT Broker    │◄──►│    Backend API              │  │
│  │  (React)    │    │  (cyberwave.com)│    │    (Django)                 │  │
│  └─────────────┘    └────────┬────────┘    └─────────────────────────────┘  │
│                              │                                               │
└──────────────────────────────┼───────────────────────────────────────────────┘
                               │
                    MQTT over TCP (port 1883)
                               │
┌──────────────────────────────┼───────────────────────────────────────────────┐
│                         EDGE (UGV Beast)                                     │
│                              │                                               │
│  ┌───────────────────────────▼───────────────────────────────────────────┐  │
│  │                      MQTT BRIDGE NODE                                  │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐  │  │
│  │  │  Cyberwave Adapter  │  Command Registry  │  Telemetry Processor │  │  │
│  │  └─────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐  │  │
│  │  │  Health Publisher   │  Navigation Bridge │  Internal Odometry   │  │  │
│  │  └─────────────────────────────────────────────────────────────────┘  │  │
│  │  ┌─────────────────────────────────────────────────────────────────┐  │  │
│  │  │  ROS Camera (WebRTC)│  Rate Limiter      │  Mapping System      │  │  │
│  │  └─────────────────────────────────────────────────────────────────┘  │  │
│  └───────────────────────────┬───────────────────────────────────────────┘  │
│                              │                                               │
│                      ROS 2 Topics                                            │
│                              │                                               │
│  ┌───────────────────────────▼───────────────────────────────────────────┐  │
│  │                    UGV BEAST ROS 2 STACK                               │  │
│  │  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐  │  │
│  │  │ ugv_bringup  │ │ ugv_driver   │ │ ugv_vision   │ │ base_node    │  │  │
│  │  └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘  │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
│                                                                              │
│  ┌───────────────────────────────────────────────────────────────────────┐  │
│  │                         HARDWARE                                       │  │
│  │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────┐ │  │
│  │  │ 4 Wheels │ │ Pan-Tilt │ │ Camera   │ │ LEDs     │ │ IMU/Sensors  │ │  │
│  │  └──────────┘ └──────────┘ └──────────┘ └──────────┘ └──────────────┘ │  │
│  └───────────────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Data Flow

**Upstream (Robot → Cloud):**
1. Hardware sensors publish to ROS 2 topics at high frequency (e.g., 100 Hz)
2. MQTT Bridge subscribes to ROS topics
3. Rate limiter filters messages (default: 5 Hz for joints, 1 Hz for status)
4. Telemetry processor transforms joint names
5. Cyberwave Adapter publishes to MQTT with `source_type: "edge"`

**Downstream (Cloud → Robot):**
1. Frontend sends commands via MQTT with `source_type: "tele"`
2. MQTT Bridge receives and filters messages (only processes `source_type: "tele"`)
3. Command Registry routes to appropriate handler
4. Handler converts to ROS message and publishes
5. Response sent back to MQTT

---

## MQTT Bridge Components

### Core Components

| Component | Description |
|-----------|-------------|
| **mqtt_bridge_node.py** | Main orchestrator - handles MQTT connectivity and message routing |
| **cyberwave_mqtt_adapter.py** | Wrapper around Cyberwave SDK for MQTT operations |
| **mapping.py** | Loads robot-specific YAML mappings for joint name transformations |
| **plugins/ugv_beast_command_handler.py** | UGV Beast command registry and hardware-specific command handlers |
| **telemetry.py** | Joint state accumulation and high-frequency feedback processing |
| **health.py** | Periodic heartbeat and system status reporting |

### Plugins

| Plugin | Description |
|--------|-------------|
| **ugv_beast_command_handler.py** | UGV Beast command registry implementation |
| **navigation_bridge.py** | Nav2 integration for autonomous navigation |
| **internal_odometry.py** | Dead-reckoning for robots without native odometry |
| **ros_camera.py** | WebRTC video streaming from ROS Image topics |

### UGV Beast Joint Configuration

The UGV Beast has the following joints managed by the bridge:

| Joint Name | Type | Description |
|------------|------|-------------|
| `left_up_wheel_link_joint` | Continuous | Front-left wheel |
| `left_down_wheel_link_joint` | Continuous | Rear-left wheel |
| `right_up_wheel_link_joint` | Continuous | Front-right wheel |
| `right_down_wheel_link_joint` | Continuous | Rear-right wheel |
| `pt_base_link_to_pt_link1` | Revolute | Pan servo (horizontal) |
| `pt_link1_to_pt_link2` | Revolute | Tilt servo (vertical) |
| `base_footprint_joint` | Fixed | Structural |
| `lidar_joint` | Fixed | LiDAR mount |
| `camera_joint` | Fixed | Camera mount |

---

## Setup Guide

### Prerequisites

Follow the official Waveshare setup guide: [UGV Beast PI ROS2](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2)

**Important:** Complete the guide until [1.2 Disable the main program from running automatically](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2_1._Preparation#:~:text=running%20automatically.-,1.2%20Disable%20the%20main%20program%20from%20running%20automatically,-Every%20time%20the)

### Step 1: Pull the Docker Image

```bash
docker pull cyberwaveos/cyb_ugv_beast:latest
```


### Step 2: Update Container Restart Policy

start the container
```bash
docker run -dit --name cyb_ugv_beast --privileged --net=host -v /dev:/dev -e DISPLAY=$DISPLAY cyberwaveos/cyb_ugv_beast:latest
```

or create teh service script:
```bash
cat > /home/ws/cyb_ugv_beast_service_install.sh << 'EOF'
#!/bin/bash
################################################################################
# Cyberwave UGV Beast Docker Container Service Installer
#
# This script creates a systemd service to keep the Docker container
# cyberwaveos/cyb_ugv_beast always running and auto-start on boot.
#
# Features:
#   - Checks for newer image versions and prompts to update
#   - Syncs container /home files to local /home without deleting existing files
#
# Usage:
#   sudo ./cyb_ugv_beast_service_install.sh
#
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

log_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
log_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
log_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1"; }

# Root check
if [[ $EUID -ne 0 ]]; then
   log_error "This script must be run as root (use sudo)"
   exit 1
fi

# Configuration
CONTAINER_IMAGE="cyberwaveos/cyb_ugv_beast:latest"
CONTAINER_NAME="cyb_ugv_beast"
SERVICE_NAME="cyb-ugv-beast"

log_info "========================================"
log_info "Cyberwave UGV Beast Docker Service Installer"
log_info "========================================"
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    log_error "Docker is not installed. Please install Docker first."
    exit 1
fi

# Check if Docker service is running
if ! systemctl is-active --quiet docker; then
    log_info "Starting Docker service..."
    systemctl start docker
    sleep 2
fi

################################################################################
# STEP 1: Check for image updates
################################################################################
log_info "Checking for image updates..."

IMAGE_UPDATED=false

if docker image inspect "$CONTAINER_IMAGE" &> /dev/null; then
    # Get local image digest
    LOCAL_DIGEST=$(docker image inspect "$CONTAINER_IMAGE" --format '{{index .RepoDigests 0}}' 2>/dev/null | cut -d'@' -f2)
    
    log_info "Local image digest: ${LOCAL_DIGEST:-unknown}"
    log_info "Fetching remote image info from Docker Hub..."
    
    # Pull just the manifest to check for updates (doesn't download layers)
    if docker pull "$CONTAINER_IMAGE" --quiet > /dev/null 2>&1; then
        REMOTE_DIGEST=$(docker image inspect "$CONTAINER_IMAGE" --format '{{index .RepoDigests 0}}' 2>/dev/null | cut -d'@' -f2)
        log_info "Remote image digest: ${REMOTE_DIGEST:-unknown}"
        
        if [[ "$LOCAL_DIGEST" != "$REMOTE_DIGEST" ]] && [[ -n "$REMOTE_DIGEST" ]]; then
            log_warning "A newer version of the image is available!"
            echo ""
            read -p "Do you want to update to the latest image? [y/N]: " UPDATE_RESPONSE
            if [[ "$UPDATE_RESPONSE" =~ ^[Yy]$ ]]; then
                log_info "Updating image..."
                if docker pull "$CONTAINER_IMAGE"; then
                    log_success "Image updated successfully!"
                    IMAGE_UPDATED=true
                else
                    log_error "Failed to update image"
                    exit 1
                fi
            else
                log_info "Keeping current image version."
            fi
        else
            log_success "Image is already up to date!"
        fi
    else
        log_warning "Could not check for remote updates. Using local image."
    fi
else
    log_warning "Docker image '$CONTAINER_IMAGE' not found locally."
    log_info "Pulling image from Docker Hub..."
    if docker pull "$CONTAINER_IMAGE"; then
        log_success "Image pulled successfully!"
        IMAGE_UPDATED=true
    else
        log_error "Failed to pull image '$CONTAINER_IMAGE'"
        exit 1
    fi
fi

################################################################################
# STEP 2: Sync container /home to local /home (without deleting local files)
# This does a DEEP RECURSIVE MERGE - adds missing folders/files at all levels
################################################################################
log_info "========================================"
log_info "Syncing container /home to local /home..."
log_info "========================================"

# Create a temporary container to extract /home contents
TEMP_CONTAINER="cyb_ugv_beast_temp_sync_$$"

log_info "Creating temporary container to extract /home contents..."
docker create --name "$TEMP_CONTAINER" "$CONTAINER_IMAGE" /bin/true > /dev/null 2>&1

# Create a temp directory for extraction
TEMP_DIR=$(mktemp -d)

# Copy container's /home to temp directory
log_info "Extracting container /home contents..."
docker cp "$TEMP_CONTAINER:/home/." "$TEMP_DIR/" 2>/dev/null || {
    log_warning "Could not extract /home from container (may not exist)"
}

# Function to recursively merge directories
# Adds missing files and folders without overwriting existing ones
merge_directories() {
    local src="$1"
    local dst="$2"
    local indent="${3:-}"
    
    # Iterate through all items in source directory
    for item in "$src"/*; do
        [[ -e "$item" ]] || continue  # Skip if no items match
        
        local basename=$(basename "$item")
        local dst_item="$dst/$basename"
        
        if [[ -d "$item" ]]; then
            # It's a directory
            if [[ ! -d "$dst_item" ]]; then
                # Directory doesn't exist locally - copy entire directory
                log_info "${indent}[+] Adding directory: $dst_item"
                cp -r "$item" "$dst_item"
            else
                # Directory exists - recurse into it to merge contents
                log_info "${indent}[~] Merging directory: $dst_item"
                merge_directories "$item" "$dst_item" "  $indent"
            fi
        else
            # It's a file
            if [[ ! -e "$dst_item" ]]; then
                # File doesn't exist locally - copy it
                log_info "${indent}[+] Adding file: $dst_item"
                cp "$item" "$dst_item"
            else
                # File exists locally - skip (preserve local version)
                log_info "${indent}[=] Keeping local: $dst_item"
            fi
        fi
    done
}

if [[ -d "$TEMP_DIR" ]] && [[ "$(ls -A $TEMP_DIR 2>/dev/null)" ]]; then
    log_info "Performing deep recursive merge (local files will NOT be overwritten)..."
    echo ""
    merge_directories "$TEMP_DIR" "/home"
    echo ""
    log_success "Home directory sync completed!"
else
    log_info "No /home contents to sync from container."
fi

# Cleanup temp directory
rm -rf "$TEMP_DIR"

# Remove temporary container
docker rm "$TEMP_CONTAINER" > /dev/null 2>&1
log_info "Temporary container removed."

echo ""
log_info "Docker image found locally: $CONTAINER_IMAGE"

# Check if container is already running
CONTAINER_RUNNING=false
if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    CONTAINER_RUNNING=true
    log_info "Container '$CONTAINER_NAME' is already running"
fi

# Stop existing service if running (to reconfigure)
if systemctl is-active --quiet "$SERVICE_NAME" 2>/dev/null; then
    log_info "Stopping existing service for reconfiguration..."
    systemctl stop "$SERVICE_NAME"
    CONTAINER_RUNNING=false
fi

# Stop and remove existing container if it exists (to ensure clean state)
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log_info "Cleaning up existing container..."
    docker stop "$CONTAINER_NAME" 2>/dev/null || true
    docker rm "$CONTAINER_NAME" 2>/dev/null || true
fi

# Create systemd service file
log_info "Creating systemd service file..."

cat > "/etc/systemd/system/${SERVICE_NAME}.service" <<SERVICEFILE
[Unit]
Description=Cyberwave UGV Beast Docker Container
Documentation=https://github.com/cyberwave-os/cyberwave-edge-ros
After=docker.service network-online.target
Requires=docker.service
Wants=network-online.target

[Service]
Type=simple
TimeoutStartSec=300
Restart=always
RestartSec=10

# Pull image if not present (ensures image exists)
ExecStartPre=/bin/bash -c '/usr/bin/docker image inspect ${CONTAINER_IMAGE} >/dev/null 2>&1 || /usr/bin/docker pull ${CONTAINER_IMAGE}'

# Remove existing container on start (if any)
ExecStartPre=-/usr/bin/docker stop ${CONTAINER_NAME}
ExecStartPre=-/usr/bin/docker rm ${CONTAINER_NAME}

# Start the container
# Run the original entrypoint (/ssh_entrypoint.sh) AND keep alive with infinite sleep
# This ensures SSH starts AND the container stays running
ExecStart=/usr/bin/docker run \\
    --name ${CONTAINER_NAME} \\
    --privileged \\
    --network host \\
    --pid host \\
    --init \\
    -v /dev:/dev \\
    -v /sys:/sys \\
    -v /proc:/proc \\
    -v /run/udev:/run/udev:ro \\
    -v /tmp/.X11-unix:/tmp/.X11-unix \\
    -v /home:/home \\
    -e DISPLAY=\${DISPLAY} \\
    -e ROS_DOMAIN_ID=0 \\
    ${CONTAINER_IMAGE} \\
    /bin/bash -c "/ssh_entrypoint.sh && exec tail -f /dev/null"

# Stop the container gracefully
ExecStop=/usr/bin/docker stop -t 10 ${CONTAINER_NAME}

# Cleanup on stop
ExecStopPost=-/usr/bin/docker rm ${CONTAINER_NAME}

[Install]
WantedBy=multi-user.target
SERVICEFILE

log_success "Service file created: /etc/systemd/system/${SERVICE_NAME}.service"

# Reload systemd daemon
log_info "Reloading systemd daemon..."
systemctl daemon-reload

# Enable the service to start on boot
log_info "Enabling service to start on boot..."
systemctl enable "$SERVICE_NAME"

# Start the service
log_info "Starting the service..."
systemctl start "$SERVICE_NAME"

# Wait for container to start (with progress)
log_info "Waiting for container to start..."
for i in {1..30}; do
    if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
        break
    fi
    sleep 1
    echo -n "."
done
echo ""

# Check service and container status
if systemctl is-active --quiet "$SERVICE_NAME"; then
    log_success "Service is running!"
else
    log_warning "Service may still be starting. Check status with:"
    log_info "  sudo systemctl status $SERVICE_NAME"
fi

if docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    log_success "Container is running!"
else
    log_warning "Container may still be starting. Check with:"
    log_info "  docker ps -a"
fi

echo ""
log_success "========================================"
log_success "Installation Complete!"
log_success "========================================"
echo ""
log_info "Service name: $SERVICE_NAME"
log_info "Container name: $CONTAINER_NAME"
log_info "Image: $CONTAINER_IMAGE"
echo ""
log_info "========================================"
log_info "Useful Commands:"
log_info "========================================"
echo ""
log_info "Check service status:"
log_info "  sudo systemctl status $SERVICE_NAME"
echo ""
log_info "View container logs:"
log_info "  docker logs -f $CONTAINER_NAME"
echo ""
log_info "Stop the service:"
log_info "  sudo systemctl stop $SERVICE_NAME"
echo ""
log_info "Start the service:"
log_info "  sudo systemctl start $SERVICE_NAME"
echo ""
log_info "Restart the service:"
log_info "  sudo systemctl restart $SERVICE_NAME"
echo ""
log_info "Disable auto-start on boot:"
log_info "  sudo systemctl disable $SERVICE_NAME"
echo ""
log_info "Enter the running container:"
log_info "  docker exec -it $CONTAINER_NAME bash"
echo ""
log_info "========================================"
log_info "Current Status:"
log_info "========================================"
systemctl status "$SERVICE_NAME" --no-pager || true
echo ""
docker ps --filter "name=$CONTAINER_NAME" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" || true
echo ""
EOF
chmod +x /home/ws/cyb_ugv_beast_service_install.sh
```

then run it 
```bash
sudo ./cyb_ugv_beast_service_install.sh 
```
set 
```bash
cd /home/ws/ugv_ws
sudo docker update --restart=unless-stopped ugv_rpi_ros_humble
```

### Step 3: Connect to the UGV

```bash
ssh root@192.168.0.144 -p 23
```

Replace `192.168.0.144` with your UGV's IP address.

---

## Configuration

### Step 1: Set Your Twin UUID

Edit the mapping file to set your digital twin UUID:

**File:** `src/mqtt_bridge/config/mappings/robot_ugv_beast_v1.yaml`

```yaml
metadata:
  twin_uuid: "your-twin-uuid-here"
```

### Step 2: Set Your Cyberwave Token

Edit the parameters file:

**File:** `src/mqtt_bridge/config/params.yaml`

```yaml
broker:
  cyberwave_token: "your-api-token-here"
```

> **Note:** After changing these values, you must rebuild the mqtt_bridge package.

### Step 3: Build the Bridge

```bash
cd /home/ws/ugv_ws
chmod +x src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh
./src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh --logs
```

### Step 4: Install Services (Production)

For production deployment with automatic startup on boot:

```bash
cd /home/ws/ugv_ws
chmod +x ugv_services_install.sh
sudo ./ugv_services_install.sh
```

### Step 5: Run the UGV Stack

**Option A: Using the run script**

```bash
cd /home/ws/ugv_ws
chmod +x ugv_run.sh
./ugv_run.sh
```

**Option B: Direct launch (for development)**

```bash
cd /home/ws/ugv_ws
source install/setup.bash
# Standard launch
ros2 launch ugv_bringup master_beast.launch.py robot_id:=robot_ugv_beast_v1

# With debug logs
ros2 launch ugv_bringup master_beast.launch.py robot_id:=robot_ugv_beast_v1 debug_logs:=true
```

---

## MQTT Commands Reference

### Command Topic

All commands are sent to the MQTT topic:
```
{prefix}cyberwave/twin/{twin_uuid}/command
```

### Response/Status Topic Pattern

All command responses are published to dedicated status topics following this pattern:
```
{prefix}cyberwave/twin/{twin_uuid}/{command}/status
```

For example:
- `actuation` responses → `cyberwave/twin/{uuid}/actuation/status`
- `lights` responses → `cyberwave/twin/{uuid}/lights/status`
- `camera_servo` responses → `cyberwave/twin/{uuid}/camera_servo/status`

This separation allows the frontend to subscribe only to relevant status updates and prevents command/response mixing on the same topic.

### Command Message Format

```json
{
  "command": "<command_name>",
  "data": { ... },
  "timestamp": 1706547890.123,
  "source_type": "tele"
}
```

### Response Message Format

All responses follow this format:
```json
{
  "command": "<command_name>",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success" | "error",
    ... command-specific data ...
  }
}
```

> **Important:** Only messages with `source_type: "tele"` are processed by the edge. Messages with `"edit"`, `"sim"`, or `"edge"` are ignored for safety.

---

### Deprecated Commands

The following commands are **deprecated** and should not be used:

| Deprecated Command | Replacement |
|-------------------|-------------|
| `cmd_vel` | Use `actuation` with commands like `move_forward`, `turn_left`, `stop` |
| `led_ctrl` | Use `lights` with `pwm` parameter |

---

### Commands Summary Table

| Command | MQTT Command Topic | MQTT Status Topic | ROS 2 Topic | ROS 2 Message Type |
|---------|-------------------|-------------------|-------------|-------------------|
| `actuation` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/actuation/status` | `/cmd_vel` | `geometry_msgs/Twist` |
| `camera_servo` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/camera_servo/status` | `/ugv/joint_states` | `sensor_msgs/JointState` |
| `lights` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/lights/status` | `/ugv/led_ctrl` | `std_msgs/Float32MultiArray` |
| `oled_ctrl` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/oled_ctrl/status` | `/ugv/oled_ctrl` | `std_msgs/String` |
| `trajectory` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/trajectory/status` | `/scaled_joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` |
| `gripper` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/gripper/status` | `/gripper/command` | `std_msgs/String` |
| `estop` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/estop/status` | `/emergency_stop` | `std_msgs/Bool` |
| `start_video` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/start_video/status` | N/A (WebRTC) | N/A |
| `stop_video` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/stop_video/status` | N/A (WebRTC) | N/A |
| `take_photo` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/take_photo/status` | N/A (internal) | N/A |
| `battery_check` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/battery_check/status` | N/A (cached) | N/A |
| `get_status` | `cyberwave/twin/{twin_uuid}/command` | `cyberwave/twin/{twin_uuid}/get_status/status` | N/A (cached) | N/A |
| `goto` | `cyberwave/twin/{twin_uuid}/navigate/command` | `cyberwave/twin/{twin_uuid}/navigate/status` | Nav2 Action: `navigate_to_pose` | `nav2_msgs/NavigateToPose` |
| `path` | `cyberwave/twin/{twin_uuid}/navigate/command` | `cyberwave/twin/{twin_uuid}/navigate/status` | Nav2 Action: `follow_path` | `nav2_msgs/FollowPath` |

> **Note:** `cmd_vel` and `led_ctrl` commands are **deprecated**. Use `actuation` for movement control and `lights` for LED control.

---

### Locomotion Commands

#### `actuation` - Movement Control (Recommended)

Simple command-based control for keyboard/gamepad teleoperation. This is the **primary command** for UGV movement control.

| Property | Value |
|----------|-------|
| **MQTT Command Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **MQTT Status Topic** | `cyberwave/twin/{twin_uuid}/actuation/status` |
| **ROS 2 Topic** | `/cmd_vel` |
| **ROS 2 Message** | `geometry_msgs/msg/Twist` |

**Request:**
```json
{
  "command": "actuation",
  "data": {
    "command": "move_forward"
  },
  "source_type": "tele"
}
```

**Supported Actuations:**

| Actuation | Description | ROS 2 Twist Values |
|-----------|-------------|-------------------|
| `move_forward` | Move forward | `linear.x: 0.3, angular.z: 0.0` |
| `move_backward` | Move backward | `linear.x: -0.3, angular.z: 0.0` |
| `turn_left` | Rotate left | `linear.x: 0.0, angular.z: 1.0` |
| `turn_right` | Rotate right | `linear.x: 0.0, angular.z: -1.0` |
| `stop` | Stop all movement | `linear.x: 0.0, angular.z: 0.0` |

**Response (published to `actuation/status` topic):**
```json
{
  "command": "actuation",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success",
    "linear_x": 0.3,
    "angular_z": 0.0
  }
}
```

---

### Camera Servo Commands

#### `camera_servo` - Pan-Tilt Control

Control the camera pan-tilt mechanism with smooth interpolation.

| Property | Value |
|----------|-------|
| **MQTT Command Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **MQTT Status Topic** | `cyberwave/twin/{twin_uuid}/camera_servo/status` |
| **ROS 2 Topic** | `/ugv/joint_states` |
| **ROS 2 Message** | `sensor_msgs/msg/JointState` |
| **Joint Names** | `pt_base_link_to_pt_link1` (pan), `pt_link1_to_pt_link2` (tilt) |

**Request (Absolute Position):**
```json
{
  "command": "camera_servo",
  "data": {
    "pan": 0.5,
    "tilt": 0.3
  },
  "source_type": "tele"
}
```

**Request (Relative Position):**
```json
{
  "command": "camera_servo",
  "data": {
    "pan_delta": 0.1,
    "tilt_delta": -0.1
  },
  "source_type": "tele"
}
```

**ROS 2 Message Published:**
```
sensor_msgs/msg/JointState
  header:
    frame_id: "tele"
  name: ["pt_base_link_to_pt_link1", "pt_link1_to_pt_link2"]
  position: [0.5, 0.3]
  velocity: []
  effort: []
```

**Parameters:**
| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pan` | float | -3.14 to 3.14 | Absolute pan position (radians) |
| `tilt` | float | -0.785 to 1.57 | Absolute tilt position (radians) |
| `pan_delta` | float | any | Relative pan change (radians) |
| `tilt_delta` | float | any | Relative tilt change (radians) |

**Response (published to `camera_servo/status` topic):**
```json
{
  "command": "camera_servo",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success",
    "pan": 0.5,
    "tilt": 0.3
  }
}
```

**Actuation Shortcuts:**

| Actuation | Description | ROS 2 Effect |
|-----------|-------------|--------------|
| `camera_up` | Tilt up by 0.1 rad | `tilt_delta: +0.1` |
| `camera_down` | Tilt down by 0.1 rad | `tilt_delta: -0.1` |
| `camera_left` | Pan left by 0.1 rad | `pan_delta: -0.1` |
| `camera_right` | Pan right by 0.1 rad | `pan_delta: +0.1` |
| `camera_default` | Reset to (0.0, 0.0) | `pan: 0.0, tilt: 0.0` |

---

### LED Control Commands

#### `lights` - Headlight Control (Recommended)

Control chassis and camera headlights (IO4 and IO5) with PWM brightness support.

> **Note:** `led_ctrl` command is **deprecated**. Use `lights` instead.

| Property | Value |
|----------|-------|
| **MQTT Command Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **MQTT Status Topic** | `cyberwave/twin/{twin_uuid}/lights/status` |
| **ROS 2 Topic** | `/ugv/led_ctrl` |
| **ROS 2 Message** | `std_msgs/msg/Float32MultiArray` |

**Request (PWM Format - Recommended):**
```json
{
  "command": "lights",
  "data": {
    "pwm": 128
  },
  "source_type": "tele"
}
```

**Request (Named Format):**
```json
{
  "command": "lights",
  "data": {
    "chassis_light": 255,
    "camera_light": 128
  },
  "source_type": "tele"
}
```

**Request (Array Format):**
```json
{
  "command": "lights",
  "data": {
    "leds": [255, 128]
  },
  "source_type": "tele"
}
```

**Request (All Lights):**
```json
{
  "command": "lights",
  "data": {
    "all": 255
  },
  "source_type": "tele"
}
```

**ROS 2 Message Published:**
```
std_msgs/msg/Float32MultiArray
  data: [128.0, 128.0]
  # data[0] = IO4 (chassis headlight)
  # data[1] = IO5 (camera headlight)
```

**Parameters:**
| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `pwm` | int | 0-255 | PWM value for both lights (recommended) |
| `chassis_light` / `io4` | int | 0-255 | Chassis headlight (near OKA camera) |
| `camera_light` / `io5` | int | 0-255 | Pan-tilt headlight (USB camera) |
| `all` | int | 0-255 | Set both lights to same value |

**Response (published to `lights/status` topic):**
```json
{
  "command": "lights",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success",
    "io4": 128,
    "io5": 128,
    "chassis_light": "on",
    "camera_light": "on",
    "chassis_light_value": 128,
    "camera_light_value": 128
  }
}
```

**Actuation Shortcuts:**

| Actuation | Description | ROS 2 Effect |
|-----------|-------------|--------------|
| `chassis_light_toggle` | Toggle chassis light on/off | `data: [255/0, current]` |
| `camera_light_toggle` | Toggle camera light on/off | `data: [current, 255/0]` |
| `led_toggle` | Toggle both lights on/off | `data: [255/0, 255/0]` |

---

### Display Commands

#### `oled_ctrl` - OLED Display Control

Display text on the UGV's OLED screen.

| Property | Value |
|----------|-------|
| **MQTT Command Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **MQTT Status Topic** | `cyberwave/twin/{twin_uuid}/oled_ctrl/status` |
| **ROS 2 Topic** | `/ugv/oled_ctrl` |
| **ROS 2 Message** | `std_msgs/msg/String` |

**Request:**
```json
{
  "command": "oled_ctrl",
  "data": {
    "text": "Hello Cyberwave!"
  },
  "source_type": "tele"
}
```

**ROS 2 Message Published:**
```
std_msgs/msg/String
  data: "Hello Cyberwave!"
```

**Response (published to `oled_ctrl/status` topic):**
```json
{
  "command": "oled_ctrl",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "displayed",
    "text": "Hello Cyberwave!"
  }
}
```

---

### Video Streaming Commands

#### `start_video` - Start WebRTC Stream

Start camera video streaming via WebRTC.

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **ROS 2 Topic** | N/A (WebRTC signaling handled internally) |
| **ROS 2 Source** | Subscribes to `/image_raw` (`sensor_msgs/msg/Image`) |

**Request:**
```json
{
  "command": "start_video",
  "data": {
    "recording": true
  },
  "source_type": "tele"
}
```

**Response:**
```json
{
  "status": "ok",
  "type": "video_started"
}
```

#### `stop_video` - Stop WebRTC Stream

Stop camera video streaming.

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **ROS 2 Topic** | N/A (WebRTC signaling handled internally) |

**Request:**
```json
{
  "command": "stop_video",
  "data": {},
  "source_type": "tele"
}
```

**Response:**
```json
{
  "status": "ok",
  "type": "video_stopped"
}
```

#### `take_photo` - Capture Snapshot

Capture a still image from the camera.

| Property | Value |
|----------|-------|
| **MQTT Topic (Command)** | `cyberwave/twin/{twin_uuid}/command` |
| **MQTT Topic (Photo Output)** | `cyberwave/twin/{twin_uuid}/camera/photo` |
| **ROS 2 Source** | Reads from internal camera frame buffer |

**Request:**
```json
{
  "command": "take_photo",
  "data": {},
  "source_type": "tele"
}
```

**Response:**
```json
{
  "command": "take_photo",
  "type": "response",
  "source_type": "edge",
  "data": {
    "status": "success",
    "command": "take_photo",
    "message": "Photo captured successfully",
    "image_size": 45678
  }
}
```

**Photo Published To:**
```
cyberwave/twin/{twin_uuid}/camera/photo
```

**Photo Payload:**
```json
{
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "image": "<base64-encoded-jpeg>",
  "format": "jpeg",
  "width": 640,
  "height": 480
}
```

---

### Status & Diagnostic Commands

#### `battery_check` - Request Battery Status

Request immediate battery status update.

| Property | Value |
|----------|-------|
| **MQTT Command Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **MQTT Status Topic** | `cyberwave/twin/{twin_uuid}/battery_check/status` |
| **ROS 2 Source** | Cached from `/ugv/battery_status` or `/voltage` |

**Request:**
```json
{
  "command": "battery_check",
  "data": {},
  "source_type": "tele"
}
```

**Response:**
```json
{
  "command": "battery_check",
  "type": "response",
  "source_type": "edge",
  "data": {
    "status": "success",
    "command": "battery_check",
    "message": "Battery status published to telemetry topic"
  }
}
```

#### `get_status` - Query Robot Status

Query cached sensor data from the robot.

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **ROS 2 Source** | Cached data from multiple ROS topics |

**Request (All Status):**
```json
{
  "command": "get_status",
  "data": {
    "target": "all"
  },
  "source_type": "tele"
}
```

**Request (Specific Target):**
```json
{
  "command": "get_status",
  "data": {
    "target": "battery"
  },
  "source_type": "tele"
}
```

**Available Targets and ROS 2 Sources:**

| Target | ROS 2 Topic | ROS 2 Message Type |
|--------|-------------|-------------------|
| `battery` | `/ugv/battery_status` | `sensor_msgs/BatteryState` |
| `imu` | `/ugv/imu` | `sensor_msgs/Imu` |
| `odom` | `/odom` | `nav_msgs/Odometry` |
| `joint_states` | `/ugv/joint_states` | `sensor_msgs/JointState` |
| `cmd_vel` | `/cmd_vel` | `geometry_msgs/Twist` |
| `led_ctrl` | `/ugv/led_ctrl` | `std_msgs/Float32MultiArray` |
| `oled_ctrl` | `/ugv/oled_ctrl` | `std_msgs/String` |
| `all` | All above | Multiple |

---

### Safety Commands

#### `estop` - Emergency Stop

Activate or deactivate emergency stop.

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **ROS 2 Topic** | `/emergency_stop` |
| **ROS 2 Message** | `std_msgs/msg/Bool` |

**Request:**
```json
{
  "command": "estop",
  "data": {
    "activate": true
  },
  "source_type": "tele"
}
```

**ROS 2 Message Published:**
```
std_msgs/msg/Bool
  data: true
```

---

### Trajectory Commands

#### `trajectory` - Joint Trajectory

Send multi-point trajectory for precise joint control.

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **ROS 2 Topic** | `/scaled_joint_trajectory_controller/joint_trajectory` |
| **ROS 2 Message** | `trajectory_msgs/msg/JointTrajectory` |

**Request:**
```json
{
  "command": "trajectory",
  "data": {
    "joint_names": ["left_up_wheel_link_joint", "right_up_wheel_link_joint"],
    "points": [
      {
        "positions": [0.0, 0.0],
        "velocities": [1.0, 1.0],
        "time_from_start": {"sec": 1, "nanosec": 0}
      }
    ]
  },
  "source_type": "tele"
}
```

**ROS 2 Message Published:**
```
trajectory_msgs/msg/JointTrajectory
  header:
    stamp: <current_time>
  joint_names: ["left_up_wheel_link_joint", "right_up_wheel_link_joint"]
  points:
    - positions: [0.0, 0.0]
      velocities: [1.0, 1.0]
      time_from_start:
        sec: 1
        nanosec: 0
```

**Response:**
```json
{
  "command": "trajectory",
  "type": "response",
  "source_type": "edge",
  "data": {
    "status": "success",
    "joints": ["left_up_wheel_link_joint", "right_up_wheel_link_joint"],
    "points_count": 1
  }
}
```

---

### Gripper Commands

#### `gripper` - Gripper Control

Control gripper actions (for robots with gripper attachments).

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/command` |
| **ROS 2 Topic** | `/gripper/command` |
| **ROS 2 Message** | `std_msgs/msg/String` |

**Request:**
```json
{
  "command": "gripper",
  "data": {
    "action": "grip"
  },
  "source_type": "tele"
}
```

**Supported Actions:**
| Action | Description |
|--------|-------------|
| `grip` | Close gripper |
| `release` | Open gripper |
| `reset` | Reset gripper to default |

**ROS 2 Message Published:**
```
std_msgs/msg/String
  data: "grip"
```

**Response:**
```json
{
  "command": "gripper",
  "type": "response",
  "source_type": "edge",
  "data": {
    "status": "executed",
    "action": "grip"
  }
}
```

---

### Navigation Commands

Navigation commands are sent to a separate topic and use Nav2 action servers.

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/navigate/command` |
| **MQTT Status Topic** | `cyberwave/twin/{twin_uuid}/navigate/status` |
| **Nav2 Actions** | `navigate_to_pose`, `follow_path` |

#### `goto` - Navigate to Point

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/navigate/command` |
| **ROS 2 Action** | `navigate_to_pose` |
| **ROS 2 Action Type** | `nav2_msgs/action/NavigateToPose` |

**Request:**
```json
{
  "action": "goto",
  "goal": {
    "x": 2.5,
    "y": 1.0,
    "theta": 0.0
  },
  "source_type": "tele"
}
```

**ROS 2 Action Goal:**
```
nav2_msgs/action/NavigateToPose
  pose:
    header:
      frame_id: "map"
    pose:
      position:
        x: 2.5
        y: 1.0
        z: 0.0
      orientation: <quaternion from theta>
```

#### `path` - Follow Path

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/navigate/command` |
| **ROS 2 Action** | `follow_path` |
| **ROS 2 Action Type** | `nav2_msgs/action/FollowPath` |

**Request:**
```json
{
  "action": "path",
  "waypoints": [
    {"x": 1.0, "y": 0.0},
    {"x": 2.0, "y": 1.0},
    {"x": 2.5, "y": 1.5}
  ],
  "source_type": "tele"
}
```

**ROS 2 Action Goal:**
```
nav2_msgs/action/FollowPath
  path:
    header:
      frame_id: "map"
    poses:
      - pose: {position: {x: 1.0, y: 0.0, z: 0.0}, ...}
      - pose: {position: {x: 2.0, y: 1.0, z: 0.0}, ...}
      - pose: {position: {x: 2.5, y: 1.5, z: 0.0}, ...}
```

#### `stop`, `pause`, `resume` - Navigation Control

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/navigate/command` |
| **ROS 2 Effect** | Cancels or controls Nav2 action |

**Request:**
```json
{
  "action": "stop",
  "source_type": "tele"
}
```

**Navigation Status Response (published to status topic):**
```json
{
  "action_id": "nav-12345",
  "status": "running|completed|failed|cancelled",
  "message": "Navigation status message",
  "source_type": "edge"
}
```

---

## MQTT Status Topics (Upstream Telemetry)

The bridge publishes robot status to these MQTT topics automatically by subscribing to ROS 2 topics.

### Status Topics Summary Table

| MQTT Topic | ROS 2 Source Topic | ROS 2 Message Type | Rate |
|------------|-------------------|-------------------|------|
| `cyberwave/joint/{twin_uuid}/update` | `/ugv/joint_states` | `sensor_msgs/JointState` | 5 Hz |
| `cyberwave/pose/{twin_uuid}/update` | Internal odometry | N/A | 1 Hz |
| `cyberwave/twin/{twin_uuid}/actuation/status` | Command handler (`/cmd_vel`) | `geometry_msgs/Twist` | On command |
| `cyberwave/twin/{twin_uuid}/lights/status` | Command handler (`/ugv/led_ctrl`) | `std_msgs/Float32MultiArray` | On command |
| `cyberwave/twin/{twin_uuid}/camera_servo/status` | Command handler (`/ugv/joint_states`) | `sensor_msgs/JointState` | On command |
| `cyberwave/twin/{twin_uuid}/oled_ctrl/status` | Command handler (`/ugv/oled_ctrl`) | `std_msgs/String` | On command |
| `cyberwave/twin/{twin_uuid}/battery_check/status` | Cached from `/ugv/battery_status` | `sensor_msgs/BatteryState` | On command |
| `cyberwave/twin/{twin_uuid}/trajectory/status` | Command handler | `trajectory_msgs/JointTrajectory` | On command |
| `cyberwave/twin/{twin_uuid}/start_video/status` | WebRTC handler | N/A | On command |
| `cyberwave/twin/{twin_uuid}/stop_video/status` | WebRTC handler | N/A | On command |
| `cyberwave/twin/{twin_uuid}/take_photo/status` | Camera frame buffer | N/A | On command |
| `cyberwave/twin/{twin_uuid}/navigate/status` | Nav2 action feedback | N/A | On change |
| `cyberwave/twin/{twin_uuid}/edge_health` | Internal | N/A | Periodic |
| `cyberwave/twin/{twin_uuid}/camera/photo` | Camera frame buffer | N/A | On request |

---

### Joint States

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/joint/{twin_uuid}/update` |
| **ROS 2 Source** | `/ugv/joint_states` or `/joint_states` |
| **ROS 2 Message** | `sensor_msgs/msg/JointState` |
| **Rate** | 5 Hz |

```json
{
  "source_type": "edge",
  "positions": {
    "left_up_wheel_link_joint": 1.234,
    "left_down_wheel_link_joint": 1.235,
    "right_up_wheel_link_joint": 1.230,
    "right_down_wheel_link_joint": 1.231,
    "pt_base_link_to_pt_link1": 0.5,
    "pt_link1_to_pt_link2": 0.3
  },
  "velocities": {
    "left_up_wheel_link_joint": 0.5,
    "left_down_wheel_link_joint": 0.5,
    "right_up_wheel_link_joint": 0.5,
    "right_down_wheel_link_joint": 0.5
  },
  "ts": 1706547890.123
}
```

### Pose (Position + Rotation)

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/pose/{twin_uuid}/update` |
| **ROS 2 Source** | Internal odometry (calculated from wheel encoders) |
| **Rate** | 1 Hz |

```json
{
  "source_type": "edge",
  "position": {
    "x": 1.5,
    "y": 2.3,
    "z": 0.0
  },
  "rotation": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.707,
    "w": 0.707
  },
  "ts": 1706547890.123
}
```

### IMU Data

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/status/imu` |
| **ROS 2 Source** | `/ugv/imu` |
| **ROS 2 Message** | `sensor_msgs/msg/Imu` |
| **Rate** | 1 Hz |

### Battery Check Status

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/battery_check/status` |
| **ROS 2 Source** | Cached from `/ugv/battery_status` or `/voltage` |
| **ROS 2 Message** | `sensor_msgs/msg/BatteryState` or `std_msgs/msg/Float32` |
| **Rate** | On `battery_check` command |

```json
{
  "command": "battery_check",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success",
    "percentage": 85,
    "voltage": 12.6
  }
}
```

### Actuation Status

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/actuation/status` |
| **ROS 2 Topic** | `/cmd_vel` |
| **ROS 2 Message** | `geometry_msgs/msg/Twist` |
| **Rate** | On `actuation` command |

```json
{
  "command": "actuation",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success",
    "linear_x": 0.3,
    "angular_z": 0.0
  }
}
```

### Lights Status

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/lights/status` |
| **ROS 2 Topic** | `/ugv/led_ctrl` |
| **ROS 2 Message** | `std_msgs/msg/Float32MultiArray` |
| **Rate** | On `lights` command |

```json
{
  "command": "lights",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success",
    "io4": 128,
    "io5": 128,
    "chassis_light": "on",
    "camera_light": "on",
    "chassis_light_value": 128,
    "camera_light_value": 128
  }
}
```

### Camera Servo Status

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/camera_servo/status` |
| **ROS 2 Topic** | `/ugv/joint_states` |
| **ROS 2 Message** | `sensor_msgs/msg/JointState` |
| **Rate** | On `camera_servo` command |

```json
{
  "command": "camera_servo",
  "type": "response",
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "data": {
    "status": "success",
    "pan": 0.5,
    "tilt": 0.3,
    "pan_degrees": 28.6,
    "tilt_degrees": 17.2,
    "pan_radians": 0.5,
    "tilt_radians": 0.3
  }
}
```

### Navigation Status

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/navigate/status` |
| **ROS 2 Source** | Nav2 action server feedback |
| **Rate** | On change |

```json
{
  "action_id": "nav-12345",
  "status": "completed",
  "message": "Navigation goal reached",
  "source_type": "edge"
}
```

**Status Values:**
- `running` - Navigation in progress
- `completed` - Goal reached successfully
- `failed` - Navigation failed
- `cancelled` - Navigation cancelled by user

### Edge Health

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/edge_health` |
| **ROS 2 Source** | Internal health monitor |
| **Rate** | Periodic (configurable) |

```json
{
  "type": "edge_health",
  "timestamp": 1706547890.123,
  "uptime_seconds": 3600,
  "streams": {
    "video": "active"
  }
}
```

### Camera Photo

| Property | Value |
|----------|-------|
| **MQTT Topic** | `cyberwave/twin/{twin_uuid}/camera/photo` |
| **ROS 2 Source** | Camera frame buffer (from `/image_raw` subscriber) |
| **Rate** | On request (via `take_photo` command) |

```json
{
  "source_type": "edge",
  "timestamp": 1706547890.123,
  "image": "<base64-encoded-jpeg>",
  "format": "jpeg",
  "width": 640,
  "height": 480
}
```

---

## Helper Scripts

### Full Environment Startup

```bash
# Start everything
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/start_ugv.sh

# Start with automatic log attachment
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/start_ugv.sh --logs

# Start hardware only (skip MQTT bridge)
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/start_ugv.sh --no-bridge
```

**View processes:** `tmux attach -t ugv_env`
**Navigate windows:** `Ctrl+B` then `n`/`p`

### MQTT Bridge Rebuild

```bash
# Rebuild and run
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh --logs

# Rebuild only (no run)
/root/ugv_ws/src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh --no-run
```

---

## Troubleshooting

### Check ROS Topics

```bash
# List all topics
ros2 topic list

# Monitor joint states
ros2 topic echo /ugv/joint_states

# Check publishing rate
ros2 topic hz /ugv/joint_states
```

### Check MQTT Traffic

```bash
mosquitto_sub -h mqtt.cyberwave.com -t '#' -v | ts '[%H:%M:%S]'
```

### Enable Debug Logging

```bash
ros2 launch mqtt_bridge mqtt_bridge.launch.py log_level:=debug
```

### Common Issues

| Issue | Solution |
|-------|----------|
| Commands not received | Check `source_type: "tele"` in payload |
| High bandwidth usage | Increase rate limit in `params.yaml` |
| Joint states not updating | Verify `twin_uuid` matches |
| Camera not streaming | Check WebRTC configuration |

---

## Configuration Reference

### params.yaml Key Settings

```yaml
broker:
  host: mqtt.cyberwave.com
  port: 1883
  use_cyberwave: true
  cyberwave_token: "<your-token>"

robot_id: "robot_ugv_beast_v1"
ros2mqtt_rate_limit: 5.0  # Hz

webrtc:
  auto_start: true
  fps: 15.0
  force_turn: true
```

### robot_ugv_beast_v1.yaml Key Settings

```yaml
metadata:
  twin_uuid: "<your-twin-uuid>"

command_registry: "mqtt_bridge.plugins.ugv_beast_command_handler.CommandRegistry"

internal_odometry:
  enabled: true
  track_width: 0.23  # meters
  wheel_radius: 0.04  # meters

camera:
  format: "yuv420p"
  fps: 15
  image_width: 640
  image_height: 480

robot_constants:
  max_velocities:
    left_up_wheel_link_joint: 10.0
    pt_base_link_to_pt_link1: 1.5
  min_trajectory_time: 0.2
```
