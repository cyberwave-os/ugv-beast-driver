#!/bin/bash
# =============================================================================
# UGV Beast Docker Integration Test
# =============================================================================
#
# Builds the Docker image and runs a smoke test that verifies the ROS 2
# workspace registration, UGV executables, and command registry imports.
#
# Steps:
#   1. Builds the root Docker image
#   2. Runs the smoke test inside the container
#   3. Optionally builds the mqtt_bridge Docker image
#
# The smoke test verifies:
#   - mqtt_bridge and ugv_bringup are registered ROS packages
#   - mqtt_bridge_node and ugv_integrated_driver executables are installed
#   - UGV service install script is present and executable
#   - Official UGV CommandRegistry imports successfully
#
# Usage:
#   cd cyberwave-edge-nodes/cyberwave-edge-ros-ugv && bash test-ugv.sh
#   bash test-ugv.sh --skip-build    # reuse existing image
#   bash test-ugv.sh --mqtt-bridge   # also build mqtt_bridge Dockerfile
#
# Prerequisites:
#   - Docker installed and running
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
EDGE_NODES_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_NAME="cyberwave-edge-ros-ugv"
SKIP_BUILD=false
BUILD_MQTT_BRIDGE=false

for arg in "$@"; do
    case $arg in
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --mqtt-bridge)
            BUILD_MQTT_BRIDGE=true
            shift
            ;;
        --help)
            echo "Usage: bash test-ugv.sh [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --skip-build     Reuse existing Docker image (skip docker build)"
            echo "  --mqtt-bridge    Also build the mqtt_bridge Dockerfile"
            echo "  --help           Show this help"
            exit 0
            ;;
    esac
done

# =============================================================================
# Step 1: Build Docker image
# =============================================================================
echo ""
echo "=========================================="
echo " Step 1: Building Docker image"
echo "=========================================="

cd "$EDGE_NODES_DIR"

if [ "$SKIP_BUILD" = true ]; then
    if docker image inspect "$IMAGE_NAME" &>/dev/null; then
        echo "Skipping build — reusing existing image: $IMAGE_NAME"
    else
        echo "ERROR: --skip-build specified but image '$IMAGE_NAME' not found"
        echo "Run without --skip-build first."
        exit 1
    fi
else
    echo "Building $IMAGE_NAME (this takes 10-15 min on first build)..."
    docker build -f cyberwave-edge-ros-ugv/docker-conf/Dockerfile -t "$IMAGE_NAME" .
    echo "  ✅ Docker image built successfully"
fi

# =============================================================================
# Step 2: Smoke test ROS package and command registry
# =============================================================================
echo ""
echo "=========================================="
echo " Step 2: Smoke test — ROS package wiring"
echo "=========================================="

docker run --rm --privileged \
    --entrypoint bash \
    -v "$SCRIPT_DIR/tests:/home/ws/tests" \
    -e CYBERWAVE_API_KEY=smoke-test-token \
    -e CYBERWAVE_TWIN_UUID=00000000-0000-0000-0000-smoke-test \
    "$IMAGE_NAME" \
    /home/ws/tests/smoke_test.sh

echo "  ✅ Smoke test passed"

# =============================================================================
# Step 2b: Build verification — required packages present & launch parses
# =============================================================================
# Guards against the crash-loop bug where a package the driver needs at runtime
# (e.g. ugv_description) silently fails to build and is missing from the image.
echo ""
echo "=========================================="
echo " Step 2b: Build verification"
echo "=========================================="

docker run --rm \
    --entrypoint bash \
    -v "$SCRIPT_DIR/tests:/home/ws/tests" \
    "$IMAGE_NAME" \
    /home/ws/tests/verify_build.sh

echo "  ✅ Build verification passed"

# =============================================================================
# Step 3: Build mqtt_bridge Dockerfile (optional)
# =============================================================================
if [ "$BUILD_MQTT_BRIDGE" = true ]; then
    echo ""
    echo "=========================================="
    echo " Step 3: Building mqtt_bridge Dockerfile"
    echo "=========================================="

    docker build -f cyberwave-edge-ros-ugv/docker-conf/Dockerfile -t "${IMAGE_NAME}-mqtt" .
    echo "  ✅ mqtt_bridge Dockerfile built successfully"
fi

# =============================================================================
# Done
# =============================================================================
echo ""
echo "=========================================="
echo " ✅ All UGV tests passed!"
echo "=========================================="
echo ""
echo "Summary:"
echo "  - Docker image built: $IMAGE_NAME"
echo "  - ROS package smoke test passed"
echo "    - mqtt_bridge and ugv_bringup packages registered"
echo "    - mqtt_bridge_node and ugv_integrated_driver executables installed"
echo "    - UGV service install script executable"
echo "  - Build verification passed"
echo "    - all required packages present in install space (incl. ugv_description)"
echo "    - ugv_beast.urdf present"
echo "    - master_beast.launch.py generates a LaunchDescription without error"
echo "    - Official UGV command registry import verified"
if [ "$BUILD_MQTT_BRIDGE" = true ]; then
    echo "  - mqtt_bridge Dockerfile built: ${IMAGE_NAME}-mqtt"
fi
echo ""
