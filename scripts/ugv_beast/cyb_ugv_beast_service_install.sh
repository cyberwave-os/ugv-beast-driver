#!/bin/bash

set -e

WORKSPACE_PATH="/home/ws/ugv_ws"
ROBOT_ID="robot_ugv_beast_v1"
MASTER_SERVICE="cyberwave-beast-master.service"
DEBUG_LOGS="false"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            DEBUG_LOGS="true"
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --debug     Enable debug logging (debug_logs:=true)"
            echo "  -h, --help  Show this help message"
            echo ""
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

echo "============================================================"
echo "🚀 CYBERWAVE UGV BEAST - AUTO SETUP & LAUNCH"
echo "============================================================"
if [ "$DEBUG_LOGS" = "true" ]; then
    echo "🐛 DEBUG MODE ENABLED"
fi
echo ""

# Function to check and fix ugv_integrated_driver entry point
check_and_fix_entry_point() {
    local SETUP_PY="${WORKSPACE_PATH}/src/ugv_main/ugv_bringup/setup.py"
    
    echo "● Checking ugv_integrated_driver entry point..."
    
    if [ ! -f "$SETUP_PY" ]; then
        echo "⚠️  Warning: $SETUP_PY not found, skipping entry point check."
        return 0
    fi
    
    # Check if ugv_integrated_driver entry point exists
    if grep -q "'ugv_integrated_driver = ugv_bringup.ugv_integrated_driver:main'" "$SETUP_PY"; then
        echo "✅ ugv_integrated_driver entry point is already configured."
        return 0
    fi
    
    echo "⚠️  ugv_integrated_driver entry point missing, adding it now..."
    
    # Backup the original file
    cp "$SETUP_PY" "${SETUP_PY}.backup"
    
    # Add the entry point using sed
    sed -i "/^            'ugv_driver = ugv_bringup.ugv_driver:main',$/a\\            'ugv_integrated_driver = ugv_bringup.ugv_integrated_driver:main'," "$SETUP_PY"
    
    echo "✅ Added ugv_integrated_driver entry point to setup.py"
}

# Function to build mqtt_bridge
build_mqtt_bridge() {
    echo ""
    echo "============================================================"
    echo "📦 BUILDING MQTT BRIDGE"
    echo "============================================================"
    
    if [ -f "${WORKSPACE_PATH}/src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh" ]; then
        cd "${WORKSPACE_PATH}"
        echo "● Running clean_build_mqtt.sh --logs..."
        bash "${WORKSPACE_PATH}/src/mqtt_bridge/scripts/ugv_beast/clean_build_mqtt.sh" --logs
        echo "✅ MQTT Bridge built successfully"
    else
        echo "⚠️  clean_build_mqtt.sh not found, building manually..."
        cd "${WORKSPACE_PATH}"
        source /opt/ros/humble/setup.bash
        colcon build --packages-select mqtt_bridge --symlink-install
        echo "✅ MQTT Bridge built successfully"
    fi
}

# Function to build ugv_bringup if entry point was fixed
build_ugv_bringup() {
    echo ""
    echo "============================================================"
    echo "📦 BUILDING UGV BRINGUP"
    echo "============================================================"
    
    cd "${WORKSPACE_PATH}"
    source /opt/ros/humble/setup.bash
    
    echo "● Building ugv_bringup package..."
    if colcon build --packages-select ugv_bringup --symlink-install; then
        echo "✅ UGV Bringup built successfully"
        
        # Verify the build created the executable
        if [ -f "${WORKSPACE_PATH}/install/ugv_bringup/lib/ugv_bringup/ugv_integrated_driver" ]; then
            echo "✅ Verified: ugv_integrated_driver executable exists"
        else
            echo "⚠️  Warning: ugv_integrated_driver executable not found after build"
        fi
    else
        echo "❌ Error: Failed to build ugv_bringup"
        exit 1
    fi
}

# Function to start services directly (for Docker)
start_services_docker() {
    echo ""
    echo "============================================================"
    echo "🐳 DOCKER MODE - STARTING SERVICES"
    echo "============================================================"
    echo "PID 1 is: $(cat /proc/1/comm)"
    echo ""
    
    # Clean up /dev/video0 if it's locked
    echo "● Checking /dev/video0..."
    find /proc/*/fd -lname "/dev/video0" 2>/dev/null | cut -d/ -f3 | xargs -r kill -9 || true
    sleep 2
    
    # Source the environment
    echo "● Sourcing ROS 2 and workspace..."
    cd "${WORKSPACE_PATH}"
    
    # Source in the current shell context
    set +e  # Don't exit on error for sourcing
    source /opt/ros/humble/setup.bash
    ROS_SOURCE_RC=$?
    source "${WORKSPACE_PATH}/install/setup.bash"
    WS_SOURCE_RC=$?
    set -e  # Re-enable exit on error
    
    if [ $ROS_SOURCE_RC -ne 0 ]; then
        echo "⚠️  Warning: ROS sourcing returned code $ROS_SOURCE_RC"
    fi
    if [ $WS_SOURCE_RC -ne 0 ]; then
        echo "⚠️  Warning: Workspace sourcing returned code $WS_SOURCE_RC"
    fi
    
    export AMENT_PREFIX_PATH="${WORKSPACE_PATH}/install:/opt/ros/humble"
    export PATH="${WORKSPACE_PATH}/install/ugv_bringup/lib/ugv_bringup:$PATH"
    export PYTHONPATH="${WORKSPACE_PATH}/install/ugv_bringup/lib/python3.10/site-packages:${PYTHONPATH}"
    
    echo "✅ Environment sourced"
    echo ""
    
    echo "● Launching Master Beast..."
    echo "● Logs will appear below..."
    echo "============================================================"
    echo ""
    
    # Launch with all environment setup in one command to ensure sourcing works
    if [ "$DEBUG_LOGS" = "true" ]; then
        echo "🐛 Debug logging enabled"
        echo ""
        /bin/bash -c "cd ${WORKSPACE_PATH} && source /opt/ros/humble/setup.bash && source ${WORKSPACE_PATH}/install/setup.bash && ros2 launch ugv_bringup master_beast.launch.py robot_id:=${ROBOT_ID} debug_logs:=true"
    else
        /bin/bash -c "cd ${WORKSPACE_PATH} && source /opt/ros/humble/setup.bash && source ${WORKSPACE_PATH}/install/setup.bash && ros2 launch ugv_bringup master_beast.launch.py robot_id:=${ROBOT_ID}"
    fi
    
    # If launch exits, show exit code
    EXIT_CODE=$?
    echo ""
    echo "============================================================"
    echo "⚠️  Launch process exited with code: $EXIT_CODE"
    echo "============================================================"
}

# Function to setup systemd services
setup_systemd_services() {
    echo ""
    echo "============================================================"
    echo "⚙️  SYSTEMD MODE - CREATING BOOT SERVICES"
    echo "============================================================"
    
    # Check if running as root
    if [ "$EUID" -ne 0 ]; then
        echo "❌ Error: Systemd service installation requires root privileges"
        echo "Please run: sudo $0"
        exit 1
    fi
    
    # Final check if we're actually able to use systemctl
    if systemctl is-system-running 2>&1 | grep -q "chroot"; then
        echo "⚠️  WARNING: Running in chroot - systemd services will not be functional"
        echo "    Falling back to direct launch..."
        echo ""
        start_services_docker
        return
    fi
    
    # Test if systemctl actually works
    if ! systemctl list-units >/dev/null 2>&1; then
        echo "⚠️  WARNING: systemctl commands not functional"
        echo "    Falling back to direct launch..."
        echo ""
        start_services_docker
        return
    fi
    
    echo "● Creating ${MASTER_SERVICE}..."
    
    # Determine debug flag for systemd service
    if [ "$DEBUG_LOGS" = "true" ]; then
        DEBUG_FLAG="debug_logs:=true"
        echo "🐛 Systemd service will run with debug logging"
    else
        DEBUG_FLAG=""
    fi
    
    # Create the Master UGV service
    cat > /etc/systemd/system/${MASTER_SERVICE} << EOF
[Unit]
Description=Cyberwave UGV Beast Master Service (Core, Bridge, Camera)
Documentation=https://github.com/cyberwave-os/cyberwave-edge-ros-ugv
After=docker.service network-online.target
Requires=docker.service
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=${WORKSPACE_PATH}

# 1. Clean up /dev/video0 if it's locked by a previous crashed session
ExecStartPre=/bin/bash -c 'find /proc/*/fd -lname "/dev/video0" 2>/dev/null | cut -d/ -f3 | xargs -r kill -9 || true'
ExecStartPre=/bin/sleep 2

# 2. Launch the consolidated master launch file
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source ${WORKSPACE_PATH}/install/setup.bash && export AMENT_PREFIX_PATH=${WORKSPACE_PATH}/install:/opt/ros/humble && export PATH=${WORKSPACE_PATH}/install/ugv_bringup/lib/ugv_bringup:\$PATH && export PYTHONPATH=${WORKSPACE_PATH}/install/ugv_bringup/lib/python3.10/site-packages:\$PYTHONPATH && ros2 launch ugv_bringup master_beast.launch.py robot_id:=${ROBOT_ID} ${DEBUG_FLAG}'

Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    # Create the video grabber service
    echo "● Creating cyberwave-video-offer.service..."
    cat > /etc/systemd/system/cyberwave-video-offer.service << EOF
[Unit]
Description=Cyberwave Video Offer Service (Trigger WebRTC Start)
After=${MASTER_SERVICE}
Wants=${MASTER_SERVICE}

[Service]
Type=oneshot
User=root
WorkingDirectory=${WORKSPACE_PATH}
# Wait for ROS nodes to initialize and bridge to connect to MQTT
ExecStartPre=/bin/sleep 15
# Trigger the start_video command via the ROS 2 service call provided by mqtt_bridge_node
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source ${WORKSPACE_PATH}/install/setup.bash && ros2 service call /mqtt_bridge_node/start_video std_srvs/srv/Trigger'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    # Reload systemd and enable services
    echo "● Reloading systemd daemon..."
    systemctl daemon-reload
    
    echo "● Enabling ${MASTER_SERVICE}..."
    systemctl enable ${MASTER_SERVICE}
    
    echo "● Enabling cyberwave-video-offer.service..."
    systemctl enable cyberwave-video-offer.service
    
    echo "● Starting ${MASTER_SERVICE}..."
    systemctl restart ${MASTER_SERVICE}
    
    echo "● Starting cyberwave-video-offer.service..."
    systemctl restart cyberwave-video-offer.service
    
    echo ""
    echo "============================================================"
    echo "✅ SYSTEMD SERVICES INSTALLED & STARTED"
    echo "============================================================"
    echo "🎉 Auto-start on boot is now ENABLED!"
    echo "The UGV Beast will start automatically on every reboot!"
    echo ""
    if [ "$DEBUG_LOGS" = "true" ]; then
        echo "📋 Service Mode: Debug logging enabled"
    else
        echo "📋 Service Mode: Standard logging"
    fi
    echo "📋 Auto-restart: Enabled (10 second delay on failure)"
    echo ""
    echo "📊 Check status:"
    echo "  systemctl status ${MASTER_SERVICE}"
    echo "  systemctl status cyberwave-video-offer.service"
    echo ""
    echo "📜 View logs:"
    echo "  journalctl -u ${MASTER_SERVICE} -f"
    echo ""
    echo "🔄 Restart:"
    echo "  sudo systemctl restart ${MASTER_SERVICE}"
    echo ""
    echo "🛑 Stop:"
    echo "  sudo systemctl stop ${MASTER_SERVICE}"
    echo ""
    echo "❌ Disable auto-start:"
    echo "  sudo systemctl disable ${MASTER_SERVICE}"
    echo "============================================================"
}

# Main execution flow
main() {
    # Step 1: Check and fix entry point
    check_and_fix_entry_point
    
    # Step 2: Build mqtt_bridge
    build_mqtt_bridge
    
    # Step 3: Always build ugv_bringup to ensure it's properly installed
    # (This is needed because the package might exist but not be properly registered)
    echo ""
    echo "● Building ugv_bringup to ensure proper installation..."
    build_ugv_bringup
    
    # Step 4: Source the environment
    echo ""
    echo "● Sourcing environment..."
    source /opt/ros/humble/setup.bash
    source "${WORKSPACE_PATH}/install/setup.bash"
    export AMENT_PREFIX_PATH="${WORKSPACE_PATH}/install"
    export PATH="${WORKSPACE_PATH}/install/ugv_bringup/lib/ugv_bringup:$PATH"
    echo "✅ Environment sourced"
    
    # Step 5: Detect environment and proceed accordingly
    # Check if systemd is actually functional (not just present)
    SYSTEMD_FUNCTIONAL=false
    
    # First check if systemd is the init system
    if pidof systemd >/dev/null 2>&1 || [ "$(cat /proc/1/comm 2>/dev/null)" = "systemd" ]; then
        # Test if systemctl actually works
        if systemctl --version >/dev/null 2>&1; then
            # Check for chroot or container restrictions
            SYSTEMD_STATE=$(systemctl is-system-running 2>&1 || true)
            
            if echo "$SYSTEMD_STATE" | grep -qE "chroot|offline"; then
                echo ""
                echo "⚠️  Systemd detected but not functional (state: $SYSTEMD_STATE)"
                echo "    This typically happens in chroot or restricted environments"
                echo "    Falling back to direct launch mode..."
            elif systemctl list-units >/dev/null 2>&1; then
                SYSTEMD_FUNCTIONAL=true
            else
                echo ""
                echo "⚠️  Systemd present but systemctl commands are restricted"
                echo "    Falling back to direct launch mode..."
            fi
        fi
    fi
    
    if [ "$SYSTEMD_FUNCTIONAL" = "true" ]; then
        # Systemd environment - create boot services
        setup_systemd_services
    else
        # Docker/Container/chroot environment - start services directly
        start_services_docker
        # Note: This function will exec and never return
    fi
}

# Run main function
main
