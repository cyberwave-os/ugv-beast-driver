#!/bin/bash

set -e

WORKSPACE_PATH="/home/ws/ugv_ws"
ROBOT_ID="robot_ugv_beast_v1"
MASTER_SERVICE="cyberwave-beast-master.service"
DEBUG_LOGS="false"

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

check_and_fix_entry_point() {
    local SETUP_PY="${WORKSPACE_PATH}/src/ugv_main/ugv_bringup/setup.py"
    
    echo "● Checking ugv_integrated_driver entry point..."
    
    if [ ! -f "$SETUP_PY" ]; then
        echo "⚠️  Warning: $SETUP_PY not found, skipping entry point check."
        return 0
    fi
    
    if grep -q "'ugv_integrated_driver = ugv_bringup.ugv_integrated_driver:main'" "$SETUP_PY"; then
        echo "✅ ugv_integrated_driver entry point is already configured."
        return 0
    fi
    
    echo "⚠️  ugv_integrated_driver entry point missing, adding it now..."
    
    cp "$SETUP_PY" "${SETUP_PY}.backup"

    sed -i "/^            'ugv_driver = ugv_bringup.ugv_driver:main',$/a\\            'ugv_integrated_driver = ugv_bringup.ugv_integrated_driver:main'," "$SETUP_PY"
    
    echo "✅ Added ugv_integrated_driver entry point to setup.py"
}

build_mqtt_bridge() {
    echo ""
    echo "============================================================"
    echo "📦 BUILDING MQTT BRIDGE"
    echo "============================================================"

    # Idempotency guard: a built image ships install/mqtt_bridge — skip compile-at-boot.
    # Rebuild path runs only when install/ is absent (dev bind-mount of src/).
    if [ -d "${WORKSPACE_PATH}/install/mqtt_bridge" ]; then
        echo "✅ mqtt_bridge already installed, skipping rebuild"
        return 0
    fi

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

build_ugv_base_node() {
    echo ""
    echo "============================================================"
    echo "📦 BUILDING UGV BASE NODE"
    echo "============================================================"
    
    cd "${WORKSPACE_PATH}"
    source /opt/ros/humble/setup.bash
    
    if [ ! -d "${WORKSPACE_PATH}/src/ugv_main/ugv_base_node" ] && \
       [ ! -d "${WORKSPACE_PATH}/src/ugv_base_node" ]; then
        echo "⚠️  ugv_base_node source not found, skipping build"
        return 0
    fi
    
    # Validate the ament-index marker, not just the dir: a partial build leaves an
    # empty install/ugv_base_node that would otherwise be skipped forever.
    if [ -f "${WORKSPACE_PATH}/install/ugv_base_node/share/ament_index/resource_index/packages/ugv_base_node" ]; then
        echo "✅ ugv_base_node already installed, skipping build"
        return 0
    fi

    # Clean stale/partial install before rebuild.
    rm -rf "${WORKSPACE_PATH}/build/ugv_base_node" "${WORKSPACE_PATH}/install/ugv_base_node"

    echo "● Building ugv_base_node package..."
    if colcon build --packages-select ugv_base_node --parallel-workers 2; then
        echo "✅ ugv_base_node built successfully"
    else
        echo "⚠️  Warning: ugv_base_node failed to build - odometry will not be available"
        echo "    The UGV will still launch but without wheel odometry."
    fi
}

# master_beast.launch.py hard-depends on the ugv_description URDF; a flaky build can
# leave a partial install (env hooks but no marker/URDF). Self-heal so already-deployed
# broken images recover on restart instead of crash-looping.
build_ugv_description() {
    echo ""
    echo "============================================================"
    echo "📦 ENSURING UGV DESCRIPTION (URDF)"
    echo "============================================================"

    cd "${WORKSPACE_PATH}"
    source /opt/ros/humble/setup.bash

    local MARKER="${WORKSPACE_PATH}/install/ugv_description/share/ament_index/resource_index/packages/ugv_description"
    local URDF="${WORKSPACE_PATH}/install/ugv_description/share/ugv_description/urdf/ugv_beast.urdf"

    if [ -f "$MARKER" ] && [ -f "$URDF" ]; then
        echo "✅ ugv_description already installed, skipping build"
        return 0
    fi

    echo "● ugv_description install incomplete — building..."
    rm -rf "${WORKSPACE_PATH}/build/ugv_description" "${WORKSPACE_PATH}/install/ugv_description"
    if colcon build --packages-select ugv_description --symlink-install --parallel-workers 1 \
        && [ -f "$MARKER" ] && [ -f "$URDF" ]; then
        echo "✅ ugv_description built successfully"
    else
        echo "❌ Error: Failed to build ugv_description (URDF)."
        echo "    master_beast.launch.py cannot start without it."
        exit 1
    fi
}

build_ugv_bringup() {
    echo ""
    echo "============================================================"
    echo "📦 BUILDING UGV BRINGUP"
    echo "============================================================"

    # Idempotency guard: skip compile-at-boot if the driver executable ships (see build_mqtt_bridge).
    if [ -f "${WORKSPACE_PATH}/install/ugv_bringup/lib/ugv_bringup/ugv_integrated_driver" ]; then
        echo "✅ ugv_bringup already installed, skipping rebuild"
        return 0
    fi

    cd "${WORKSPACE_PATH}"
    source /opt/ros/humble/setup.bash

    echo "● Building ugv_bringup package..."
    if colcon build --packages-select ugv_bringup --symlink-install; then
        echo "✅ UGV Bringup built successfully"
        
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

start_services_docker() {
    echo ""
    echo "============================================================"
    echo "🐳 DOCKER MODE - STARTING SERVICES"
    echo "============================================================"
    echo "PID 1 is: $(cat /proc/1/comm)"
    echo ""
    
    echo "● Checking /dev/video0..."
    find /proc/*/fd -lname "/dev/video0" 2>/dev/null | cut -d/ -f3 | xargs -r kill -9 || true
    sleep 2
    
    echo "● Sourcing ROS 2 and workspace..."
    cd "${WORKSPACE_PATH}"

    set +e  # don't exit on sourcing error
    source /opt/ros/humble/setup.bash
    ROS_SOURCE_RC=$?
    source "${WORKSPACE_PATH}/install/setup.bash"
    WS_SOURCE_RC=$?
    set -e
    
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
    
    # Launch ros2 in the BACKGROUND and forward stop signals so `docker stop` triggers a
    # GRACEFUL ROS shutdown: nodes close the WebRTC peer (backend frees the UDP port) and
    # release /dev/video0. Without this, SIGTERM hits PID 1, launch is hard-killed, and the
    # backend leaks a WebRTC port each restart → "no more available ports".
    if [ "$DEBUG_LOGS" = "true" ]; then
        echo "🐛 Debug logging enabled"
        echo ""
        DEBUG_FLAG="debug_logs:=true"
    else
        DEBUG_FLAG=""
    fi

    set +e  # manage child + signals manually below
    # exec so LAUNCH_PID is ros2 launch itself (signals reach it, not a wrapper bash).
    /bin/bash -c "cd ${WORKSPACE_PATH} && source /opt/ros/humble/setup.bash && source ${WORKSPACE_PATH}/install/setup.bash && exec ros2 launch ugv_bringup master_beast.launch.py robot_id:=${ROBOT_ID} ${DEBUG_FLAG}" &
    LAUNCH_PID=$!

    forward_shutdown() {
        echo ""
        echo "● Container stop received — shutting down ROS gracefully (releasing WebRTC + camera)..."
        # SIGINT = ros2 launch's clean-shutdown signal (relays to the nodes).
        kill -INT "$LAUNCH_PID" 2>/dev/null || true
        # Give nodes time to close WebRTC + release /dev/video0 before exit.
        for _ in $(seq 1 50); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 0.2
        done
        kill -TERM "$LAUNCH_PID" 2>/dev/null || true
    }
    trap forward_shutdown SIGTERM SIGINT

    wait "$LAUNCH_PID"
    EXIT_CODE=$?
    trap - SIGTERM SIGINT
    echo ""
    echo "============================================================"
    echo "⚠️  Launch process exited with code: $EXIT_CODE"
    echo "============================================================"
}

setup_systemd_services() {
    echo ""
    echo "============================================================"
    echo "⚙️  SYSTEMD MODE - CREATING BOOT SERVICES"
    echo "============================================================"
    
    if [ "$EUID" -ne 0 ]; then
        echo "❌ Error: Systemd service installation requires root privileges"
        echo "Please run: sudo $0"
        exit 1
    fi
    
    if systemctl is-system-running 2>&1 | grep -q "chroot"; then
        echo "⚠️  WARNING: Running in chroot - systemd services will not be functional"
        echo "    Falling back to direct launch..."
        echo ""
        start_services_docker
        return
    fi
    
    if ! systemctl list-units >/dev/null 2>&1; then
        echo "⚠️  WARNING: systemctl commands not functional"
        echo "    Falling back to direct launch..."
        echo ""
        start_services_docker
        return
    fi
    
    echo "● Creating ${MASTER_SERVICE}..."
    
    if [ "$DEBUG_LOGS" = "true" ]; then
        DEBUG_FLAG="debug_logs:=true"
        echo "🐛 Systemd service will run with debug logging"
    else
        DEBUG_FLAG=""
    fi
    
    cat > /etc/systemd/system/${MASTER_SERVICE} << EOF
[Unit]
Description=Cyberwave UGV Beast Master Service (Core, Bridge, Camera)
After=network-online.target
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

main() {
    check_and_fix_entry_point

    build_mqtt_bridge

    build_ugv_base_node

    build_ugv_description

    echo ""
    echo "● Building ugv_bringup to ensure proper installation..."
    build_ugv_bringup

    echo ""
    echo "● Sourcing environment..."
    source /opt/ros/humble/setup.bash
    source "${WORKSPACE_PATH}/install/setup.bash"
    export AMENT_PREFIX_PATH="${WORKSPACE_PATH}/install"
    export PATH="${WORKSPACE_PATH}/install/ugv_bringup/lib/ugv_bringup:$PATH"
    echo "✅ Environment sourced"

    SYSTEMD_FUNCTIONAL=false

    if pidof systemd >/dev/null 2>&1 || [ "$(cat /proc/1/comm 2>/dev/null)" = "systemd" ]; then
        if systemctl --version >/dev/null 2>&1; then
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
        setup_systemd_services
    else
        start_services_docker
        # start_services_docker execs and never returns.
    fi
}

# Run main function only when executed directly (not when sourced for tests).
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
