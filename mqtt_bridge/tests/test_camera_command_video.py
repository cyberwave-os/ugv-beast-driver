"""Step 5 gate: start_video param pass-through + set_camera_format command."""

from __future__ import annotations

from typing import Any

import pytest

from mqtt_bridge.tests.conftest import FakeNode
from mqtt_bridge.plugins.ugv_beast_command_handler import CommandRegistry

ACTUATION_VIDEO = {"start_video", "stop_video"}


class CameraNode(FakeNode):
    def __init__(self) -> None:
        super().__init__()
        self.start_calls: list[dict] = []
        self.stop_calls = 0
        self.format_calls: list[dict] = []
        self.set_format_result: Any = {"status": "ok", "applied": {}}

    def start_camera_stream(self, **kwargs):
        self.start_calls.append(kwargs)

    def stop_camera_stream(self):
        self.stop_calls += 1

    def set_camera_format(self, **kwargs):
        self.format_calls.append(kwargs)
        return self.set_format_result


def route(registry: CommandRegistry, command: str, data=None) -> bool:
    """Mirror mqtt_bridge_node routing.

    Video start/stop go via the 'actuation' handler, which forwards the nested
    `data` dict to the sub-handler (see _process_actuation: data.get('data')).
    """
    payload = {"command": command, "source_type": "tele", "data": data or {}}
    if command in ACTUATION_VIDEO:
        return registry.handle_command("actuation", payload)
    return registry.handle_command(command, data or {})


@pytest.fixture
def reg():
    node = CameraNode()
    registry = CommandRegistry(node)
    node._command_registry = registry
    registry.set_mqtt_context(node._mqtt_adapter, "dev/cmd")
    return registry


def test_set_camera_format_registered(reg):
    assert "set_camera_format" in reg.get_registered_commands()


def test_start_video_passes_through_profile(reg):
    node: CameraNode = reg.node  # type: ignore[assignment]
    assert route(
        reg, "start_video",
        data={"format": "mjpeg2rgb", "width": 800, "height": 600, "fps": 15, "recording": True},
    ) is True
    assert node.start_calls == [
        {"recording": True, "pixel_format": "mjpeg2rgb", "width": 800, "height": 600, "fps": 15}
    ]


def test_start_video_defaults_when_no_params(reg):
    node: CameraNode = reg.node  # type: ignore[assignment]
    assert route(reg, "start_video") is True
    assert node.start_calls == [{}]  # no forced params -> node uses config defaults


def test_set_camera_format_forwards_and_acks(reg):
    node: CameraNode = reg.node  # type: ignore[assignment]
    ok = reg.handle_command(
        "set_camera_format",
        {"format": "mjpeg2rgb", "width": 1280, "height": 720, "fps": 30},
    )
    assert ok is True
    assert node.format_calls == [
        {"pixel_format": "mjpeg2rgb", "width": 1280, "height": 720, "fps": 30}
    ]


def test_set_camera_format_validation_error_returns_false(reg):
    node: CameraNode = reg.node  # type: ignore[assignment]
    node.set_format_result = {"status": "error", "message": "unsupported"}
    ok = reg.handle_command(
        "set_camera_format", {"format": "yuyv", "width": 1920, "height": 1080, "fps": 30}
    )
    assert ok is False
