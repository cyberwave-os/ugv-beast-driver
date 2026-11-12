#!/usr/bin/env bash
# UGV Beast build verification: regression guard for the crash-loop bug where a
# required package (e.g. ugv_description) silently fails to build. Verifies every
# needed package + the URDF are installed and master_beast.launch.py parses.
# Run during `docker build` and by test-ugv.sh; exits non-zero on first failure.
set -eo pipefail

# shellcheck disable=SC1091
source /usr/local/bin/source_ros_setup.sh
if [ -f /home/ws/ugv_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /home/ws/ugv_ws/install/setup.bash
fi

echo "=== UGV build verification ==="

# Packages master_beast.launch.py resolves + the driver's runtime deps must be
# registered. Keep in sync with the STRICT build passes in the Dockerfile.
REQUIRED_PACKAGES=(
    ugv_bringup      # provides ugv_integrated_driver (core driver node)
    ugv_vision       # get_package_share_directory('ugv_vision')
    ugv_description  # ships the URDF loaded by robot_state_publisher
    ugv_base_node    # odometry calculator
    ldlidar          # get_package_share_directory('ldlidar')
    ugv_interface    # ROS interfaces used by the driver
    mqtt_bridge      # Cyberwave MQTT bridge node
)

ROS_PACKAGES="$(ros2 pkg list)"
missing=()
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    grep -qx "$pkg" <<< "$ROS_PACKAGES" || missing+=("$pkg")
done
if [ "${#missing[@]}" -ne 0 ]; then
    echo "ERROR: required ROS package(s) missing from install space: ${missing[*]}" >&2
    echo "       This is exactly what causes the driver to crash-loop on the robot." >&2
    exit 1
fi
echo "✓ Required ROS packages registered: ${REQUIRED_PACKAGES[*]}"

# The driver does `from ugv_bringup import ugv_driver_core` at launch. If a Dockerfile
# forgets to COPY ugv_driver_core.py, colcon still builds ugv_bringup and ros2 pkg list
# shows it, but the driver crashes with ImportError. Catch that here (cheap: no rclpy).
if ! python3 -c "from ugv_bringup import ugv_driver_core" 2>/tmp/core_import.err; then
    echo "ERROR: 'from ugv_bringup import ugv_driver_core' failed — the driver will" >&2
    echo "       crash at launch. Ensure ugv_driver_core.py is COPYed into the" >&2
    echo "       ugv_bringup package (see docker-conf/Dockerfile*)." >&2
    sed 's/^/       /' /tmp/core_import.err >&2 || true
    exit 1
fi
echo "✓ ugv_bringup.ugv_driver_core importable (driver won't crash on import)"

# Full first-party stack (not needed by launch, but all Dockerfiles build it).
# `colcon build --packages-select` silently no-ops on an unknown name, so a strict
# build pass won't catch a package that never built — this presence check does.
UGV_STACK_PACKAGES=(
    ugv_chat_ai
    ugv_nav
    ugv_slam
    ugv_tools
    ugv_web_app
)
# ugv_gazebo is sim-only with no arm64/Humble gazebo package, so the slim image sets
# UGV_VERIFY_SKIP_GAZEBO=1; the vendor-base Dockerfiles build it so it stays required.
if [ "${UGV_VERIFY_SKIP_GAZEBO:-0}" != "1" ]; then
    UGV_STACK_PACKAGES+=(ugv_gazebo)
fi
stack_missing=()
for pkg in "${UGV_STACK_PACKAGES[@]}"; do
    grep -qx "$pkg" <<< "$ROS_PACKAGES" || stack_missing+=("$pkg")
done
if [ "${#stack_missing[@]}" -ne 0 ]; then
    echo "ERROR: ugv_beast stack package(s) missing from install space: ${stack_missing[*]}" >&2
    echo "       Expected the full ugv_beast package set in the image; a package" >&2
    echo "       failed to build or was dropped/misspelled from --packages-select." >&2
    exit 1
fi
echo "✓ Full ugv_beast stack registered: ${UGV_STACK_PACKAGES[*]}"

# The URDF that robot_state_publisher loads must exist on disk.
python3 - <<'PY'
import os
import sys
from ament_index_python.packages import get_package_share_directory

urdf = os.path.join(get_package_share_directory("ugv_description"), "urdf", "ugv_beast.urdf")
if not os.path.isfile(urdf):
    sys.exit(f"ERROR: expected URDF not found at {urdf}")
print(f"✓ URDF present: {urdf}")
PY

# master_beast.launch.py must generate a LaunchDescription without raising — the exact
# parse path that runs on the robot, so a missing package/URDF fails here not on-device.
python3 - <<'PY'
import importlib.util
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

launch_file = get_package_share_directory("ugv_bringup") + "/launch/master_beast.launch.py"
spec = importlib.util.spec_from_file_location("master_beast_launch", launch_file)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

try:
    ld = module.generate_launch_description()
except Exception as exc:  # noqa: BLE001 - surface the exact parse-time failure
    sys.exit(f"ERROR: master_beast.launch.py failed to parse: {exc!r}")

if not isinstance(ld, LaunchDescription):
    sys.exit(f"ERROR: generate_launch_description() returned {type(ld)!r}, not LaunchDescription")

print(f"✓ master_beast.launch.py parsed OK ({len(ld.entities)} top-level entities)")
PY

echo "=== UGV build verification passed ==="
