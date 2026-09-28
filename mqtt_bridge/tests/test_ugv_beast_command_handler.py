"""Unit tests for UGV Beast keyboard teleop command routing and handlers."""

from __future__ import annotations

from typing import Any

import pytest

from mqtt_bridge.tests.conftest import FakeNode

from mqtt_bridge.plugins.ugv_beast_command_handler import CommandRegistry

# Keyboard bindings from seed_controllers.py (controller:ugv-beast:v1)
KEYBOARD_ACTUATIONS = [
    "move_forward",
    "move_backward",
    "turn_left",
    "turn_right",
    "stop",
    "chassis_light_toggle",
    "camera_light_toggle",
    "camera_up",
    "camera_down",
    "camera_left",
    "camera_right",
    "take_photo",
    "battery_check",
    "camera_default",
]

ACTUATION_COMMANDS = [
    "move_forward",
    "move_backward",
    "turn_left",
    "turn_right",
    "stop",
    "locomotion_velocity",
    "velocity_command",
    "camera_up",
    "camera_down",
    "camera_left",
    "camera_right",
    "camera_default",
    "chassis_light_toggle",
    "camera_light_toggle",
    "led_toggle",
    "take_photo",
    "battery_check",
    "sit_down",
    "stand_up",
    "obstacle_avoidance_toggle",
    "start_video",
    "stop_video",
]


def teleop_payload(command: str, **extra: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "command": command,
        "source_type": "tele",
        "timestamp": 1706547890.123,
    }
    payload.update(extra)
    return payload


def route_twin_command(registry: CommandRegistry, payload: dict[str, Any]) -> bool:
    """Mirror mqtt_bridge_node actuation routing for /twin/{uuid}/command."""
    command = payload.get("command")
    if not command:
        return False
    if command in ACTUATION_COMMANDS or isinstance(
        payload.get("velocity_command"), dict
    ):
        return registry.handle_command("actuation", payload)
    command_data = payload.get("data")
    if not isinstance(command_data, dict):
        command_data = {}
    return registry.handle_command(command, command_data)


@pytest.fixture
def registry() -> CommandRegistry:
    node = FakeNode()
    command_registry = CommandRegistry(node)
    node._command_registry = command_registry
    command_registry.set_mqtt_context(
        node._mqtt_adapter,
        "devcyberwave/twin/00000000-0000-0000-0000-test-twin/command",
    )
    return command_registry


def test_command_registry_initializes_core_handlers(registry: CommandRegistry) -> None:
    registered = registry.get_registered_commands()
    assert "actuation" in registered
    assert "lights" in registered
    assert "camera_servo" in registered
    assert "battery_check" in registered
    assert "take_photo" in registered
    assert len(registered) >= 10


def test_move_forward_publishes_positive_linear_velocity(
    registry: CommandRegistry,
) -> None:
    node: FakeNode = registry.node  # type: ignore[assignment]
    assert route_twin_command(registry, teleop_payload("move_forward")) is True

    cmd_vel = node.publisher_for(node.resolve_ros_topic("/cmd_vel"))
    assert len(cmd_vel.messages) == 1
    twist = cmd_vel.messages[0]
    assert twist.linear.x > 0.0
    assert twist.angular.z == 0.0


def test_stop_publishes_zero_velocity(registry: CommandRegistry) -> None:
    node: FakeNode = registry.node  # type: ignore[assignment]
    assert route_twin_command(registry, teleop_payload("move_forward")) is True
    assert route_twin_command(registry, teleop_payload("stop")) is True

    cmd_vel = node.publisher_for(node.resolve_ros_topic("/cmd_vel"))
    assert cmd_vel.messages
    twist = cmd_vel.messages[-1]
    assert twist.linear.x == 0.0
    assert twist.angular.z == 0.0


def test_turn_left_publishes_positive_angular_velocity(
    registry: CommandRegistry,
) -> None:
    node: FakeNode = registry.node  # type: ignore[assignment]
    assert route_twin_command(registry, teleop_payload("turn_left")) is True

    twist = node.publisher_for(node.resolve_ros_topic("/cmd_vel")).messages[-1]
    assert twist.linear.x == 0.0
    assert twist.angular.z > 0.0


def test_chassis_light_toggle_publishes_led_command(registry: CommandRegistry) -> None:
    node: FakeNode = registry.node  # type: ignore[assignment]
    assert route_twin_command(registry, teleop_payload("chassis_light_toggle")) is True

    led_pub = node.publisher_for(node.resolve_ros_topic("/ugv/led_ctrl"))
    assert len(led_pub.messages) == 1
    assert led_pub.messages[0].data[0] == 255.0


def test_camera_up_starts_servo_interpolation(registry: CommandRegistry) -> None:
    node: FakeNode = registry.node  # type: ignore[assignment]
    assert route_twin_command(registry, teleop_payload("camera_up")) is True

    servo = registry._handlers["camera_servo"]
    assert servo._target_tilt > servo._interp_start_tilt
    assert any(not timer.cancelled for timer in node.timers)


@pytest.mark.parametrize("actuation", KEYBOARD_ACTUATIONS)
def test_keyboard_bindings_route_without_registry_init_error(
    registry: CommandRegistry,
    actuation: str,
) -> None:
    """Every UGV Beast keyboard binding must route through the actuation handler."""
    payload = teleop_payload(actuation)
    result = route_twin_command(registry, payload)

    if actuation == "take_photo":
        # No camera frame in unit tests; handler still responds deterministically.
        assert result is False
        return

    assert result is True, f"Expected handler success for actuation={actuation}"


def test_locomotion_velocity_policy_publishes_twist(
    registry: CommandRegistry,
) -> None:
    node: FakeNode = registry.node  # type: ignore[assignment]
    payload = teleop_payload("locomotion_velocity")
    payload["velocity_command"] = {
        "contract": "locomotion.velocity_command.v1",
        "linear_x": 0.2,
        "linear_y": 0.0,
        "angular_z": 0.0,
        "duration_ms": 500,
        "gait": "walk",
        "origin": "teleop",
    }
    assert route_twin_command(registry, payload) is True

    twist = node.publisher_for(node.resolve_ros_topic("/cmd_vel")).messages[-1]
    assert twist.linear.x == 0.2
    assert twist.angular.z == 0.0


def test_loose_velocity_command_without_contract_publishes_twist(
    registry: CommandRegistry,
) -> None:
    """A velocity_command lacking the contract/gait/origin envelope is accepted."""
    node: FakeNode = registry.node  # type: ignore[assignment]
    payload = teleop_payload("velocity_command")
    payload["velocity_command"] = {
        "linear_x": 0.6,
        "linear_y": 0.0,
        "angular_z": 0.4,
        "duration_ms": 300,
    }
    assert route_twin_command(registry, payload) is True

    twist = node.publisher_for(node.resolve_ros_topic("/cmd_vel")).messages[-1]
    assert twist.linear.x == pytest.approx(0.6)
    assert twist.angular.z == pytest.approx(0.4)


def test_top_level_velocity_command_overrides_discrete_actuation(
    registry: CommandRegistry,
) -> None:
    """An explicit velocity_command wins over the discrete ``command`` string.

    The frontend may attach precise velocities to a keyboard actuation such as
    "move_forward"; the explicit kinematics must be honored rather than the
    default cruise speed used by the discrete movement path.
    """
    node: FakeNode = registry.node  # type: ignore[assignment]
    payload = teleop_payload("move_forward")
    payload["velocity_command"] = {
        "linear_x": 0.6,
        "linear_y": 0.0,
        "angular_z": 0.4,
        "duration_ms": 300,
    }
    assert route_twin_command(registry, payload) is True

    twist = node.publisher_for(node.resolve_ros_topic("/cmd_vel")).messages[-1]
    # 0.6 m/s is the explicit value, not the discrete default cruise speed.
    assert twist.linear.x == pytest.approx(0.6)
    assert twist.angular.z == pytest.approx(0.4)


def test_top_level_velocity_command_arms_timed_stop(
    registry: CommandRegistry,
) -> None:
    """Explicit-velocity teleop arms the duration_ms deadman (unlike keyboard)."""
    actuation = registry._handlers["actuation"]
    schedule_calls: list[int] = []
    original_schedule = actuation._locomotion_stop.schedule_ms

    def track_schedule(duration_ms: int) -> None:
        schedule_calls.append(duration_ms)
        original_schedule(duration_ms)

    actuation._locomotion_stop.schedule_ms = track_schedule  # type: ignore[method-assign]
    payload = teleop_payload("velocity_command")
    payload["velocity_command"] = {
        "linear_x": 0.5,
        "angular_z": 0.0,
        "duration_ms": 300,
    }
    assert route_twin_command(registry, payload) is True
    assert schedule_calls == [300]


def test_combined_diagonal_uses_locomotion_contract(
    registry: CommandRegistry,
) -> None:
    node: FakeNode = registry.node  # type: ignore[assignment]
    assert route_twin_command(registry, teleop_payload("move_forward")) is True
    assert route_twin_command(registry, teleop_payload("turn_right")) is True

    twist = node.publisher_for(node.resolve_ros_topic("/cmd_vel")).messages[-1]
    assert twist.linear.x > 0.0
    assert twist.angular.z < 0.0


def test_movement_commands_do_not_leak_timers(registry: CommandRegistry) -> None:
    """Each teleop command must not create a short-lived debounce timer."""
    node: FakeNode = registry.node  # type: ignore[assignment]
    timer_count_after_init = len(node.timers)

    for _ in range(50):
        assert route_twin_command(registry, teleop_payload("move_forward")) is True

    assert len(node.timers) == timer_count_after_init


def test_keyboard_teleop_does_not_arm_timed_stop(registry: CommandRegistry) -> None:
    actuation = registry._handlers["actuation"]
    schedule_calls: list[int] = []
    original_schedule = actuation._locomotion_stop.schedule_ms

    def track_schedule(duration_ms: int) -> None:
        schedule_calls.append(duration_ms)
        original_schedule(duration_ms)

    actuation._locomotion_stop.schedule_ms = track_schedule  # type: ignore[method-assign]
    assert route_twin_command(registry, teleop_payload("move_forward")) is True
    assert schedule_calls == []


def test_command_registry_requires_locomotion_contracts() -> None:
    """Registry init depends on vendored locomotion contracts in edge-common."""
    from cyberwave_edge_common.locomotion_contracts import (
        LOCOMOTION_VELOCITY_COMMAND_CONTRACT,
        build_locomotion_velocity_command,
    )

    assert LOCOMOTION_VELOCITY_COMMAND_CONTRACT == "locomotion.velocity_command.v1"
    payload = build_locomotion_velocity_command(linear_x=0.1).to_payload()
    assert payload["contract"] == LOCOMOTION_VELOCITY_COMMAND_CONTRACT

    node = FakeNode()
    command_registry = CommandRegistry(node)
    assert "actuation" in command_registry.get_registered_commands()
