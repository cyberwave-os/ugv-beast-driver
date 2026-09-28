# syntax=docker/dockerfile:1.4
# Cyberwave UGV Beast image — official lean build (ros-base builder → ros-core
# runtime, ~1.6 GB). Replaces the legacy vendor-base build.
# Build context is cyberwave-edge-nodes (PARENT dir) for the shared
# cyberwave-edge-common package. Build on-device, native arm64 (not QEMU/cross).
#   docker build -f cyberwave-edge-nodes/cyberwave-edge-ros-ugv/docker-conf/Dockerfile \
#     -t cyberwaveos/ugv-driver:local cyberwave-edge-nodes

ARG ROS_DISTRO=humble
ARG CYBERWAVE_SDK_VERSION=0.5.3

# Pinned WebRTC/Python deps (match docker-conf/Dockerfile).
ARG PYDANTIC_VERSION=2.12.5
ARG FLASK_VERSION=3.0.3
ARG AV_VERSION=16.1.0
ARG AIORTC_VERSION=1.14.0
ARG AIOICE_VERSION=0.10.2

# STAGE 1: base-system — ros-base build base (has toolchain; stable, cache once).
FROM ros:${ROS_DISTRO}-ros-base AS base-system
# Re-declare ARGs: global ARGs (before first FROM) are usable only in FROM lines.
ARG ROS_DISTRO
ARG PYDANTIC_VERSION
ARG FLASK_VERSION
ARG AV_VERSION
ARG AIORTC_VERSION
ARG AIOICE_VERSION

LABEL maintainer="Cyberwave Robotics Team"
LABEL stage="base-system"
LABEL description="UGV Beast build base: toolchain + -dev deps + pinned pip set"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV PIP_ROOT_USER_ACTION=ignore
# No .pyc in copied artifacts.
ENV PYTHONDONTWRITEBYTECODE=1
# Cap compiler memory on low-RAM Pi.
ENV MAKEFLAGS="-j2"

# Refresh ROS apt key (expired-key issue) + build toolchain and -dev deps.
# libssl-dev + pkg-config: Fast-DDS "Could NOT find OpenSSL".
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries; \
    apt-get update || true; \
    apt-get install -y --no-install-recommends curl ca-certificates gnupg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros2-latest-archive-keyring.gpg \
        || { echo "toolchain prep (curl/key) failed"; exit 1; }; \
    ok=0; \
    for attempt in 1 2 3 4 5; do \
        rm -rf /var/lib/apt/lists/*; \
        if apt-get update && apt-get install -y --no-install-recommends \
            build-essential \
            cmake \
            git \
            pkg-config \
            libssl-dev \
            python3-pip \
            python3-dev \
            python3-argcomplete \
            python3-colcon-common-extensions \
            python3-rosdep \
            python3-vcstool; then ok=1; break; fi; \
        echo "apt attempt ${attempt}/5 failed (mirror lag?); retrying in 15s"; \
        sleep 15; \
    done; \
    [ "$ok" = 1 ] || { echo "apt install failed after 5 attempts"; exit 1; }; \
    rm -rf /var/lib/apt/lists/*

# Pinned heavy pip deps. --prefer-binary avoids resolver backtracking; STAGE 3 harvests /usr/local/lib.
RUN pip3 install --no-cache-dir --prefer-binary \
        pydantic==${PYDANTIC_VERSION} \
        flask==${FLASK_VERSION} \
        av==${AV_VERSION} \
        aiortc==${AIORTC_VERSION} \
        aioice==${AIOICE_VERSION}

# rosdep cache for the cloned vendor deps.
RUN rosdep update --rosdistro ${ROS_DISTRO} || true

# STAGE 2: workspace-builder — clone + compile (rebuild when code changes).
FROM base-system AS workspace-builder
ARG ROS_DISTRO
ARG CYBERWAVE_SDK_VERSION

LABEL stage="workspace-builder"
LABEL description="Cloned ugv_ws + mqtt_bridge, colcon-built (no --symlink-install)"

RUN mkdir -p /home/ws
WORKDIR /home/ws

# Clone vendor workspace (retry), then drop .git — unused at runtime.
RUN --mount=type=secret,id=github_token,required=false \
    trap 'rm -f /root/.gitconfig' EXIT; \
    if [ -s /run/secrets/github_token ]; then \
      git config --global \
        "url.https://x-access-token:$(cat /run/secrets/github_token)@github.com/.insteadOf" \
        "https://github.com/"; \
    fi; \
    bash -c " \
    clone_ok=0; \
    for i in {1..5}; do \
      git clone --depth 1 -b ros2-humble-develop https://github.com/DUDULRX/ugv_ws.git /home/ws/ugv_ws && clone_ok=1 && break || { \
        echo 'Clone attempt '\$i' failed, retrying in 10s...'; sleep 10; }; \
    done; \
    [ \"\$clone_ok\" -eq 1 ] || { echo 'ERROR: failed to clone ugv_ws'; exit 1; }; \
    rm -rf /home/ws/ugv_ws/.git"

WORKDIR /home/ws/ugv_ws

# ROS-source shims (verify_build.sh sources source_ros_setup.sh); carried to STAGE 3.
RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'source /opt/ros/'"${ROS_DISTRO}"'/setup.bash' \
    > /usr/local/bin/source_ros_build.sh && chmod +x /usr/local/bin/source_ros_build.sh && \
    printf '%s\n' \
    '#!/usr/bin/env bash' \
    'source /usr/local/bin/source_ros_build.sh' \
    'if [ -f /home/ws/ugv_ws/install/setup.bash ]; then source /home/ws/ugv_ws/install/setup.bash; fi' \
    > /usr/local/bin/source_ros_setup.sh && chmod +x /usr/local/bin/source_ros_setup.sh

# Shared edge helpers (harvested from /usr/local later).
COPY cyberwave-edge-common /tmp/cyberwave-edge-common
RUN pip3 install --no-cache-dir /tmp/cyberwave-edge-common && rm -rf /tmp/cyberwave-edge-common

# mqtt_bridge source package + Cyberwave customizations of ugv_bringup.
COPY cyberwave-edge-ros-ugv /home/ws/ugv_ws/src/mqtt_bridge
COPY cyberwave-edge-ros-ugv/scripts/ugv_beast/ugv_bringup/launch/master_beast.launch.py /home/ws/ugv_ws/src/ugv_main/ugv_bringup/launch/
COPY cyberwave-edge-ros-ugv/scripts/ugv_beast/ugv_bringup/ugv_bringup/ugv_integrated_driver.py /home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/
# ROS-free core imported by ugv_integrated_driver at launch — MUST be copied or the
# driver crashes with ImportError (no actuation) while the bridge keeps running.
COPY cyberwave-edge-ros-ugv/scripts/ugv_beast/ugv_bringup/ugv_bringup/ugv_driver_core.py /home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/
COPY cyberwave-edge-ros-ugv/scripts/ugv_beast/ugv_bringup/setup.py /home/ws/ugv_ws/src/ugv_main/ugv_bringup/setup.py
COPY cyberwave-edge-ros-ugv/scripts/ugv_beast/ugv_services_install.sh /home/ws/ugv_ws/ugv_services_install.sh
COPY cyberwave-edge-ros-ugv/tests/verify_build.sh /home/ws/tests/verify_build.sh

RUN grep -q "ugv_integrated_driver" /home/ws/ugv_ws/src/ugv_main/ugv_bringup/setup.py \
    || (echo "ERROR: ugv_integrated_driver not found in setup.py" && exit 1)

# Resolve deps for only the packages we build via explicit paths (not `--from-paths src`)
# so rosdep skips the heavy third-party ugv_else trees (g2o/Eigen). apt-get update first:
# the prior RUN cleared apt lists. Skip GUI + ROS1 roslaunch keys. ugv_gazebo excluded:
# sim-only, ROS1 catkin key, no installable gazebo package on Humble/arm64.
RUN apt-get update && \
    rosdep install --from-paths \
        src/ugv_main/ugv_interface \
        src/ugv_main/ugv_base_node \
        src/ugv_main/ugv_description \
        src/ugv_main/ugv_vision \
        src/ugv_main/ugv_bringup \
        src/ugv_main/ugv_chat_ai \
        src/ugv_main/ugv_nav \
        src/ugv_main/ugv_slam \
        src/ugv_main/ugv_tools \
        src/ugv_main/ugv_web_app \
        src/ugv_else/ldlidar \
        src/mqtt_bridge \
        --ignore-src -r -y \
        --skip-keys "rviz2 joint_state_publisher_gui roslaunch" && \
    rm -rf /var/lib/apt/lists/*

# Cyberwave SDK (fast-changing) + headless OpenCV (cloud teleop uses cv2, no HighGUI).
RUN pip3 install --no-cache-dir cyberwave==${CYBERWAVE_SDK_VERSION} opencv-python-headless

# Build the packages the driver needs, STRICTLY. No --symlink-install so install/ holds
# real files (symlinks into src/ would dangle after COPY). Split + --parallel-workers 1 bounds Pi RAM.
RUN bash -c "source /usr/local/bin/source_ros_build.sh && cd /home/ws/ugv_ws && \
    colcon build --packages-select ldlidar ugv_interface --parallel-workers 1 --executor sequential"
RUN bash -c "source /usr/local/bin/source_ros_setup.sh && cd /home/ws/ugv_ws && \
    colcon build --packages-select ugv_base_node --parallel-workers 1"
RUN bash -c "source /usr/local/bin/source_ros_setup.sh && cd /home/ws/ugv_ws && \
    colcon build --packages-select ugv_description ugv_vision ugv_bringup --parallel-workers 1 --executor sequential"
RUN bash -c "source /usr/local/bin/source_ros_setup.sh && cd /home/ws/ugv_ws && \
    colcon build --packages-select mqtt_bridge"

# Supplementary first-party packages (not launched; built for parity + the
# verify_build.sh stack gate), STRICTLY. ugv_gazebo excluded (see above); slim sets
# UGV_VERIFY_SKIP_GAZEBO=1. Heavy ugv_else deps stay excluded (g2o/Eigen OOM arm64).
RUN bash -c "source /usr/local/bin/source_ros_setup.sh && cd /home/ws/ugv_ws && \
    colcon build --packages-select \
    ugv_chat_ai \
    ugv_nav \
    ugv_slam \
    ugv_tools \
    ugv_web_app \
    --parallel-workers 1 --executor sequential"

# Hard gate: required packages + URDF present, launch parses. Skip gazebo (not built in slim).
RUN UGV_VERIFY_SKIP_GAZEBO=1 bash /home/ws/tests/verify_build.sh

# Slim what STAGE 3 copies out (best-effort): strip .so in the ROS workspace ONLY —
# NOT /usr/local/lib pip extension .so (strip drops PyInit_ and breaks import). Drop
# build/ + log/ (only install/ ships), prune __pycache__ (regenerated at runtime).
RUN set -e; \
    find /home/ws/ugv_ws/install -type f \( -name '*.so' -o -name '*.so.*' \) -print0 2>/dev/null \
      | xargs -0 -r strip --strip-unneeded 2>/dev/null || true; \
    rm -rf /home/ws/ugv_ws/build /home/ws/ugv_ws/log || true; \
    find /home/ws/ugv_ws /usr/local/lib -type d -name __pycache__ -prune -exec rm -rf {} + 2>/dev/null || true

# STAGE 3: runtime — SLIM, FROM ros-core, NO toolchain.
FROM ros:${ROS_DISTRO}-ros-core AS runtime
ARG ROS_DISTRO

LABEL maintainer="Cyberwave Robotics Team"
LABEL description="Cyberwave UGV Beast with MQTT Bridge Integration (slim runtime)"
LABEL version="1.0"
LABEL cyberwave.driver.type="ugv_beast"

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV WORKSPACE_PATH=/home/ws/ugv_ws
ENV PYTHONDONTWRITEBYTECODE=1
# ugv_gazebo not built in slim — runtime verify must skip it too.
ENV UGV_VERIFY_SKIP_GAZEBO=1

# Runtime apt set (import-closure verified in a throwaway ros-core container).
# ros-core lacks tf2/launch/ros2cli/msgs — added explicitly. GUI, mediapipe, xacro
# omitted; ssh/udev/v4l/usb/net tooling for hardware + operator parity.
# Wrapped in a fresh-index + retry loop: ports.ubuntu.com periodically rotates a
# package to a new point release and 404s the old .deb while its index still
# lists it (mirror lag). Re-fetching the index each attempt picks up the new
# version (or lands on a synced mirror) instead of failing the whole build.
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries && \
    ok=0; \
    for attempt in 1 2 3 4 5; do \
        rm -rf /var/lib/apt/lists/*; \
        if apt-get update && apt-get install -y --no-install-recommends \
            ros-${ROS_DISTRO}-robot-state-publisher \
            ros-${ROS_DISTRO}-joint-state-publisher \
            ros-${ROS_DISTRO}-usb-cam \
            ros-${ROS_DISTRO}-image-proc \
            ros-${ROS_DISTRO}-image-transport \
            ros-${ROS_DISTRO}-message-filters \
            ros-${ROS_DISTRO}-rclcpp-components \
            ros-${ROS_DISTRO}-tf2 \
            ros-${ROS_DISTRO}-tf2-ros \
            ros-${ROS_DISTRO}-tf2-ros-py \
            ros-${ROS_DISTRO}-launch \
            ros-${ROS_DISTRO}-launch-ros \
            ros-${ROS_DISTRO}-ros2launch \
            ros-${ROS_DISTRO}-ros2cli \
            ros-${ROS_DISTRO}-ros2pkg \
            ros-${ROS_DISTRO}-ros2run \
            ros-${ROS_DISTRO}-rosidl-runtime-py \
            ros-${ROS_DISTRO}-sensor-msgs \
            ros-${ROS_DISTRO}-geometry-msgs \
            ros-${ROS_DISTRO}-nav-msgs \
            ros-${ROS_DISTRO}-nav2-msgs \
            ros-${ROS_DISTRO}-std-msgs \
            ros-${ROS_DISTRO}-std-srvs \
            ros-${ROS_DISTRO}-trajectory-msgs \
            ros-${ROS_DISTRO}-action-msgs \
            ros-${ROS_DISTRO}-ur-msgs \
            python3-paho-mqtt \
            python3-yaml \
            python3-serial \
            openssh-server \
            udev \
            v4l-utils \
            alsa-utils \
            usbutils \
            iproute2; then ok=1; break; fi; \
        echo "apt attempt ${attempt}/5 failed (mirror lag?); retrying in 15s"; \
        sleep 15; \
    done; \
    [ "$ok" = 1 ] || { echo "apt install failed after 5 attempts"; exit 1; }; \
    rm -rf /var/lib/apt/lists/*

# Harvest built artifacts (nothing compiled here): pip site-packages, console scripts +
# shims, and ugv_ws install/ + src/ (entrypoint reads src setup.py and runs scripts by path).
COPY --from=workspace-builder /usr/local/lib /usr/local/lib
COPY --from=workspace-builder /usr/local/bin /usr/local/bin
COPY --from=workspace-builder /home/ws/ugv_ws /home/ws/ugv_ws
COPY --from=workspace-builder /home/ws/tests/verify_build.sh /home/ws/tests/verify_build.sh
RUN ldconfig

# On-robot debugging via `docker exec`; no sshd shipped.

RUN chmod +x /home/ws/ugv_ws/*.sh 2>/dev/null || true && \
    chmod +x /home/ws/ugv_ws/src/ugv_else/ldlidar/scripts/*.sh 2>/dev/null || true

WORKDIR /home/ws/ugv_ws

# SSH (base image parity, port 23) + common ROS ports.
EXPOSE 23 11311 11345

# Entrypoint is compile-free: idempotency guards see install/ and skip the colcon
# rebuild, so this ros-core runtime never needs the toolchain.
ENTRYPOINT ["/home/ws/ugv_ws/ugv_services_install.sh"]
