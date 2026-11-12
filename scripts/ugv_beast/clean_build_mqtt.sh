#!/bin/bash

clear
# MQTT Bridge Clean Build and Run Script
# This script removes existing build/install artifacts for mqtt_bridge, rebuilds it, and optionally runs the node.

WORKSPACE_ROOT="/home/ws/ugv_ws"
PACKAGE_NAME="mqtt_bridge"
RUN_NODE=false
SHOW_LOGS=false

# Processes to stop before rebuilding/running
PROCESS_PATTERNS=(
    "component_container"
    "usb_cam_node_exe"
    "mqtt_bridge_node"
    "ugv_integrated_driver"
    "joint_state_publisher"
    "robot_state_publisher"
    "ldlidar"
    "base_node"
    "complementary_filter"
    "v4l2-ctl"
    "ffmpeg"
    "gst-launch-1.0"
    "v4l2_camera"
    "camera_node"
    "ros2"
    "launch_ros"
)

# Parse arguments
for arg in "$@"
do
    case $arg in
        --run)
        RUN_NODE=true
        shift
        ;;
        --logs)
        SHOW_LOGS=true
        shift
        ;;
        --help)
        echo "Usage: ./clean_build_mqtt.sh [OPTIONS]"
        echo "Options:"
        echo "  --run       Build and run the node"
        echo "  --logs      Show logs if running"
        exit 0
        ;;
    esac
done

echo "Navigating to workspace: $WORKSPACE_ROOT"
cd $WORKSPACE_ROOT

# 0. Stop any previous running processes
echo "Stopping previous UGV/MQTT processes (if any)..."

# Force stop critical ROS components
# We use pkill with -f to match the process name. 
# We avoid using sudo here to prevent "unable to resolve host" warnings and 
# potential shell signal issues, as most ROS processes run as the current user.
# CRITICAL: We must NOT match our own script's path in the pkill pattern.
# We use a regex trick [m] to avoid matching the pkill command itself in the process list.
pkill -9 -f "ros2" || true
pkill -9 -f "usb_cam" || true
pkill -9 -f "[m]qtt_bridge_node" || true
pkill -9 -f "ugv_integrated_driver" || true

# Note: ros2 daemon stop/start removed - not available before sourcing ROS environment
# and not critical for the build process

for pattern in "${PROCESS_PATTERNS[@]}"; do
    if pgrep -f "$pattern" > /dev/null; then
        echo " - Killing processes matching: $pattern"
        pkill -15 -f "$pattern" || true
    fi
done

# Wait a moment for graceful shutdown
sleep 2

# Force kill any remaining processes
for pattern in "${PROCESS_PATTERNS[@]}"; do
    if pgrep -f "$pattern" > /dev/null; then
        echo " - Force killing remaining processes matching: $pattern"
        pkill -9 -f "$pattern" || true
    fi
done

# Robust device-level cleanup for /dev/video0
echo " - Ensuring /dev/video0 is released..."
find /proc/*/fd -lname "/dev/video0" 2>/dev/null | cut -d/ -f3 | xargs -r kill -9 || true
sleep 1

# 1. Clean existing build and install files for the package
echo "Cleaning existing build and install artifacts for $PACKAGE_NAME..."
rm -rf install/$PACKAGE_NAME build/$PACKAGE_NAME

# 2. Build the package
echo "Building $PACKAGE_NAME..."
colcon build --packages-select $PACKAGE_NAME --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Source environment
echo "Sourcing environments..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 4. Run the node (Optional)
if [ "$RUN_NODE" = true ]; then
    if [ "$SHOW_LOGS" = true ]; then
        echo "Starting $PACKAGE_NAME node and showing logs..."
    else
        echo "Starting $PACKAGE_NAME node..."
    fi
    ros2 run mqtt_bridge mqtt_bridge_node --ros-args --params-file $WORKSPACE_ROOT/install/mqtt_bridge/share/mqtt_bridge/config/params.yaml -p robot_id:=robot_ugv_beast_v1
else
    echo "Build complete. skipping execution as requested."
    if [ "$SHOW_LOGS" = true ]; then
        echo "Note: --logs was specified but --run was not set. If you want to see logs of the running environment, use: tmux attach -t ugv_env"
    fi
fi
