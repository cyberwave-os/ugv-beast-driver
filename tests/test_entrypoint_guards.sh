#!/bin/bash
# Regression test for the entrypoint idempotency guards: build_mqtt_bridge and
# build_ugv_bringup must SKIP (no colcon) when install/ exists and rebuild when absent.
# No ROS/Docker: sources the entrypoint with main() off, stubs colcon, asserts behavior.
set -u
SCRIPT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../scripts/ugv_beast" && pwd)/ugv_services_install.sh}"
FAIL=0
assert() { if eval "$2"; then echo "  PASS: $1"; else echo "  FAIL: $1"; FAIL=1; fi; }

set --                       # clear argv so the sourced arg-parser is a no-op
source "$SCRIPT"
set +e                       # relax after sourcing (script uses set -e)

COLCON_SENTINEL=""
colcon() { COLCON_SENTINEL="called: $*"; return 0; }
make_ws() { WS="$(mktemp -d)"; WORKSPACE_PATH="$WS"; }

echo "--- build_mqtt_bridge: installed -> SKIP ---"
make_ws; mkdir -p "$WS/install/mqtt_bridge"; COLCON_SENTINEL=""
OUT="$(build_mqtt_bridge 2>&1)"
assert "skips when install/mqtt_bridge present" '[[ "$OUT" == *"skipping rebuild"* ]]'
assert "no colcon on skip"                      '[[ -z "$COLCON_SENTINEL" ]]'

echo "--- build_mqtt_bridge: NOT installed -> REBUILD ---"
make_ws; COLCON_SENTINEL=""
OUT="$(build_mqtt_bridge 2>&1; echo "SENT=$COLCON_SENTINEL")"
assert "rebuilds when install/mqtt_bridge absent" '[[ "$OUT" == *"SENT=called:"* || "$OUT" == *"building manually"* ]]'

echo "--- build_ugv_bringup: installed -> SKIP ---"
make_ws; mkdir -p "$WS/install/ugv_bringup/lib/ugv_bringup"; touch "$WS/install/ugv_bringup/lib/ugv_bringup/ugv_integrated_driver"; COLCON_SENTINEL=""
OUT="$(build_ugv_bringup 2>&1)"
assert "skips when ugv_integrated_driver present" '[[ "$OUT" == *"skipping rebuild"* ]]'
assert "no colcon on skip"                        '[[ -z "$COLCON_SENTINEL" ]]'

echo "--- build_ugv_bringup: NOT installed -> REBUILD ---"
make_ws; COLCON_SENTINEL=""
build_ugv_bringup >/dev/null 2>&1
assert "invokes colcon when executable absent" '[[ "$COLCON_SENTINEL" == called:* ]]'

echo "--- build_ugv_base_node: pre-existing guard intact ---"
# Guard validates the ament-index marker (not just the dir), so create that marker.
make_ws
mkdir -p "$WS/src/ugv_main/ugv_base_node" \
    "$WS/install/ugv_base_node/share/ament_index/resource_index/packages"
touch "$WS/install/ugv_base_node/share/ament_index/resource_index/packages/ugv_base_node"
COLCON_SENTINEL=""
OUT="$(build_ugv_base_node 2>&1)"
assert "skips when install/ugv_base_node present" '[[ "$OUT" == *"already installed"* ]]'

if [ "$FAIL" -eq 0 ]; then echo "=== entrypoint guard tests passed ==="; else echo "=== entrypoint guard tests FAILED ==="; fi
exit $FAIL
