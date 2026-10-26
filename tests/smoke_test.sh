#!/usr/bin/env bash
set -eo pipefail

source /usr/local/bin/source_ros_setup.sh
if [ -f /home/ws/ugv_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /home/ws/ugv_ws/install/setup.bash
fi

ROS_PACKAGES="$(ros2 pkg list)"
grep -qx 'mqtt_bridge' <<< "$ROS_PACKAGES"
grep -qx 'ugv_bringup' <<< "$ROS_PACKAGES"

MQTT_BRIDGE_EXECUTABLES="$(ros2 pkg executables mqtt_bridge)"
UGV_BRINGUP_EXECUTABLES="$(ros2 pkg executables ugv_bringup)"
grep -q 'mqtt_bridge_node' <<< "$MQTT_BRIDGE_EXECUTABLES"
grep -q 'ugv_integrated_driver' <<< "$UGV_BRINGUP_EXECUTABLES"
test -x /home/ws/ugv_ws/ugv_services_install.sh

python3 - <<'PY'
from dataclasses import dataclass, field
from typing import Any

from mqtt_bridge.plugins.ugv_beast_command_handler import CommandRegistry


@dataclass
class _FakePublisher:
    messages: list[Any] = field(default_factory=list)

    def publish(self, message: Any) -> None:
        self.messages.append(message)


class _FakeNode:
    def __init__(self) -> None:
        self._mapping = type("M", (), {"twin_uuid": "smoke-test-twin"})()
        self.ros_prefix = "dev"
        self._command_registry = None
        self._publishers: dict[str, _FakePublisher] = {}

    def get_logger(self):
        return self

    def debug(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass

    def warning(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def create_publisher(self, _msg_type, topic, _qos):
        pub = _FakePublisher()
        self._publishers[topic] = pub
        return pub

    def create_timer(self, _period, _callback):
        class _T:
            def cancel(self):
                pass

        return _T()

    def get_clock(self):
        class _C:
            nanoseconds = 1

            def to_msg(self):
                return object()

        clock = type("Clock", (), {"now": lambda _self: _C()})()
        return clock


node = _FakeNode()
registry = CommandRegistry(node)
node._command_registry = registry
assert "actuation" in registry.get_registered_commands()

payload = {"command": "move_forward", "source_type": "tele", "timestamp": 1.0}
assert registry.handle_command("actuation", payload) is True
assert node._publishers["/cmd_vel"].messages[-1].linear.x > 0.0
PY

echo "UGV smoke test passed"
