#!/usr/bin/env bash
# Build cyberwaveos/ugv-driver locally — same stages as CI publish-docker-hub job,
# using docker-conf/Dockerfile.optimized (equivalent to docker-conf/Dockerfile).
#
# Usage (from cyberwave-edge-nodes/cyberwave-edge-ros-ugv):
#   ./docker-conf/build-local.sh                    # base + full image (arm64)
#   ./docker-conf/build-local.sh --skip-base        # reuse base (stage cache or host tag)
#   ./docker-conf/build-local.sh --pull-base        # use Hub base-dev (fastest on Pi)
#   ./docker-conf/build-local.sh --base-only        # only build ugv_driver_base
#   ./docker-conf/build-local.sh --smoke            # build + CI-style smoke test
#   ./docker-conf/build-local.sh --force-rebuild    # bust cache from mqtt_bridge COPY onward
#
# Manual equivalents (run from cyberwave-edge-nodes — the build context must be
# the parent dir so the shared cyberwave-edge-common package is in scope):
#
# Option A — host image tag (after --base-only / --load base-local):
#   cd ~/cyberwave/cyberwave-edge-nodes
#   docker build \
#     -f cyberwave-edge-ros-ugv/docker-conf/Dockerfile.optimized \
#     --build-arg UGV_DRIVER_BASE_IMAGE=cyberwaveos/ugv-driver:base-local \
#     -t cyberwaveos/ugv-driver:test-local \
#     .
#
# Option B — buildx stage cache (same builder as base build; do NOT use image tag with buildx):
#   cd ~/cyberwave/cyberwave-edge-nodes
#   docker buildx build --platform linux/arm64 \
#     -f cyberwave-edge-ros-ugv/docker-conf/Dockerfile.optimized \
#     --build-arg UGV_DRIVER_BASE_IMAGE=ugv_driver_base \
#     -t cyberwaveos/ugv-driver:test-local \
#     --load \
#     .
#
# Option C — script:
#   ./docker-conf/build-local.sh --skip-base --tag test-local
#
# Raspberry Pi notes:
# - Do not pass UGV_DRIVER_BASE_IMAGE=cyberwaveos/ugv-driver:base-local to buildx: container
#   builders cannot see --load'd host images and try Docker Hub instead.
# - The first colcon pass builds only ldlidar + ugv_interface (the heavy upstream
#   nav/SLAM/vizanti stack is commented out in the Dockerfile). The launch-critical
#   second pass (ugv_bringup/ugv_description/ugv_vision + ugv_base_node) is the long
#   part on a Pi — watch package names scroll, it is not frozen. The unused ugv_*
#   stack (gazebo/nav/slam/...) is no longer built, and its source + the clone's
#   .git history are pruned in-layer (STAGE 7) while each colcon stage drops its own
#   build/ scratch — so the slimming actually shrinks the shipped image (~210 MB)
#   rather than just hiding files behind a later-layer whiteout.
# - Prefer ./docker-conf/build-local.sh --pull-base on Pi (skip local base + colcon in base).
# - Use --host-docker (default on linux/arm64) to avoid buildx-in-container overhead.
# - Avoid CACHEBUST on every build; use --force-rebuild only when you need a clean mqtt layer.
# - If the Pi becomes unresponsive, add swap (sudo dmesg | tail while building checks OOM).
#
# On Mac (Apple Silicon): default platform is linux/arm64 (matches Raspberry Pi).
# First full colcon build is slow (~1–2h); unchanged layers cache on later builds.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# Build context is the parent (cyberwave-edge-nodes) so the shared
# cyberwave-edge-common package is in scope for Dockerfile.optimized's COPY.
CONTEXT_DIR="$(cd "$REPO_ROOT/.." && pwd)"

DOCKERFILE="$SCRIPT_DIR/Dockerfile.optimized"
IMAGE="cyberwaveos/ugv-driver"
BASE_TAG="${BASE_TAG:-base-local}"
FINAL_TAG="${FINAL_TAG:-test-local}"
PLATFORM="${PLATFORM:-linux/arm64}"
SDK_VERSION="${CYBERWAVE_SDK_VERSION:-0.5.2}"

SKIP_BASE=false
PULL_BASE=false
BASE_ONLY=false
SMOKE=false
FORCE_REBUILD=false
USE_HOST_DOCKER=false

# buildx-in-container adds RAM/CPU overhead; classic docker build is gentler on Pi.
if [[ "$(uname -s)" == "Linux" && "$(uname -m)" == "aarch64" ]]; then
    USE_HOST_DOCKER=true
fi

usage() {
    sed -n '2,12p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

cache_bust_args() {
    if $FORCE_REBUILD; then
        echo "--build-arg CACHEBUST=$(date +%s)"
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-base) SKIP_BASE=true ;;
        --pull-base) PULL_BASE=true ;;
        --base-only) BASE_ONLY=true ;;
        --smoke) SMOKE=true ;;
        --force-rebuild) FORCE_REBUILD=true ;;
        --host-docker) USE_HOST_DOCKER=true ;;
        --buildx) USE_HOST_DOCKER=false ;;
        --tag) FINAL_TAG="$2"; shift ;;
        --base-tag) BASE_TAG="$2"; shift ;;
        --platform) PLATFORM="$2"; shift ;;
        -h|--help) usage 0 ;;
        *) echo "Unknown option: $1" >&2; usage 1 ;;
    esac
    shift
done

# Plain progress shows colcon package names instead of a silent spinner.
export BUILDKIT_PROGRESS="${BUILDKIT_PROGRESS:-plain}"

cd "$CONTEXT_DIR"

echo "==> Repo:      $REPO_ROOT"
echo "==> Dockerfile: $DOCKERFILE"
echo "==> Platform:  $PLATFORM"
echo "==> Base tag:  ${IMAGE}:${BASE_TAG}"
echo "==> Final tag: ${IMAGE}:${FINAL_TAG}"
echo "==> Builder:   $(if $USE_HOST_DOCKER; then echo 'docker build (host)'; else echo 'docker buildx'; fi)"
if $FORCE_REBUILD; then
    echo "==> Cache:     bust from mqtt_bridge COPY (colcon will rerun)"
else
    echo "==> Cache:     layer cache enabled (pass --force-rebuild to bust)"
fi
echo ""
echo "Pi: [runtime 12/19] colcon can take 45–120+ min — not stuck if package names still appear."
echo ""

if ! docker buildx version &>/dev/null; then
    echo "ERROR: docker buildx is required." >&2
    exit 1
fi

# Ensure a buildx builder exists (Docker Desktop creates one by default).
if ! docker buildx inspect --bootstrap &>/dev/null; then
    docker buildx create --use --name ugv-local-builder
fi

build_base() {
    echo "==> Building base stage (ugv_driver_base) → ${IMAGE}:${BASE_TAG}"
    docker buildx build \
        --platform "$PLATFORM" \
        -f "$DOCKERFILE" \
        --target ugv_driver_base \
        -t "${IMAGE}:${BASE_TAG}" \
        --load \
        .
}

build_final_from_stage_cache() {
    # Same Dockerfile stage name — reuses buildx cache from a prior --target ugv_driver_base
    # build on this builder. Does not need the --load'd host image tag.
    echo "==> Building runtime image → ${IMAGE}:${FINAL_TAG}"
    echo "    UGV_DRIVER_BASE_IMAGE=ugv_driver_base (buildx stage cache)"
    # shellcheck disable=SC2046
    docker buildx build \
        --platform "$PLATFORM" \
        -f "$DOCKERFILE" \
        --build-arg "UGV_DRIVER_BASE_IMAGE=ugv_driver_base" \
        --build-arg "CYBERWAVE_SDK_VERSION=${SDK_VERSION}" \
        $(cache_bust_args) \
        -t "${IMAGE}:${FINAL_TAG}" \
        --load \
        .
}

build_final_from_host_image() {
    # Classic docker build reads images from the host daemon (where --load put base-local).
    # Required when UGV_DRIVER_BASE_IMAGE is a tag like cyberwaveos/ugv-driver:base-local:
    # buildx container builders cannot see those --load'd images and try Docker Hub instead.
    local base_ref="$1"
    echo "==> Building runtime image → ${IMAGE}:${FINAL_TAG}"
    echo "    UGV_DRIVER_BASE_IMAGE=${base_ref} (host image via docker build)"
    # shellcheck disable=SC2046
    docker build \
        -f "$DOCKERFILE" \
        --build-arg "UGV_DRIVER_BASE_IMAGE=${base_ref}" \
        --build-arg "CYBERWAVE_SDK_VERSION=${SDK_VERSION}" \
        $(cache_bust_args) \
        -t "${IMAGE}:${FINAL_TAG}" \
        .
}

build_final_from_registry() {
    local base_ref="$1"
    echo "==> Building runtime image → ${IMAGE}:${FINAL_TAG}"
    if $USE_HOST_DOCKER; then
        echo "    UGV_DRIVER_BASE_IMAGE=${base_ref} (host docker build; pulls base first)"
        docker pull "$base_ref"
        # shellcheck disable=SC2046
        docker build \
            -f "$DOCKERFILE" \
            --build-arg "UGV_DRIVER_BASE_IMAGE=${base_ref}" \
            --build-arg "CYBERWAVE_SDK_VERSION=${SDK_VERSION}" \
            $(cache_bust_args) \
            -t "${IMAGE}:${FINAL_TAG}" \
            .
        return
    fi
    echo "    UGV_DRIVER_BASE_IMAGE=${base_ref} (pull via buildx)"
    # shellcheck disable=SC2046
    docker buildx build \
        --platform "$PLATFORM" \
        -f "$DOCKERFILE" \
        --build-arg "UGV_DRIVER_BASE_IMAGE=${base_ref}" \
        --build-arg "CYBERWAVE_SDK_VERSION=${SDK_VERSION}" \
        $(cache_bust_args) \
        -t "${IMAGE}:${FINAL_TAG}" \
        --load \
        .
}

if $BASE_ONLY; then
    build_base
    echo ""
    echo "✅ Base image loaded: ${IMAGE}:${BASE_TAG}"
    echo "   Next: ./docker-conf/build-local.sh --skip-base --tag ${FINAL_TAG}"
    exit 0
fi

BASE_REF="${IMAGE}:${BASE_TAG}"

if $PULL_BASE; then
    HUB_BASE="${IMAGE}:base-dev"
    if ! $USE_HOST_DOCKER; then
        echo "==> Pulling published base from Docker Hub: ${HUB_BASE}"
        docker pull "$HUB_BASE"
    fi
    build_final_from_registry "$HUB_BASE"
elif $SKIP_BASE; then
    if $USE_HOST_DOCKER; then
        if docker image inspect "${IMAGE}:${BASE_TAG}" &>/dev/null; then
            echo "==> Reusing host image ${BASE_REF}"
            build_final_from_host_image "$BASE_REF"
        else
            echo "ERROR: --skip-base but ${BASE_REF} not found. Run --base-only or use --pull-base." >&2
            exit 1
        fi
    else
        echo "==> Reusing ugv_driver_base buildx stage cache"
        build_final_from_stage_cache
    fi
else
    build_base
    if $USE_HOST_DOCKER; then
        build_final_from_host_image "$BASE_REF"
    else
        build_final_from_stage_cache
    fi
fi

echo ""
echo "✅ Image ready: ${IMAGE}:${FINAL_TAG}"
echo ""
echo "Smoke test (optional):"
echo "  docker run --rm --platform ${PLATFORM} --entrypoint bash ${IMAGE}:${FINAL_TAG} -c \\"
echo "    'source /usr/local/bin/source_ros_setup.sh && ros2 pkg list | grep mqtt_bridge'"

if $SMOKE; then
    echo ""
    echo "==> Running smoke test..."
    docker run --rm --platform "$PLATFORM" --entrypoint bash "${IMAGE}:${FINAL_TAG}" -c "\
        source /usr/local/bin/source_ros_setup.sh && \
        [ -f /home/ws/ugv_ws/install/setup.bash ] && source /home/ws/ugv_ws/install/setup.bash || true && \
        ros2 pkg list | grep -q mqtt_bridge && \
        echo '✅ mqtt_bridge package registered' && \
        ros2 pkg list | grep -q ugv_bringup && \
        echo '✅ ugv_bringup package registered' && \
        test -f /home/ws/ugv_ws/ugv_services_install.sh && \
        echo '✅ Service entrypoint exists' && \
        echo '✅ Smoke test passed'"
fi
