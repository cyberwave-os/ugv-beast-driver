"""Shared pytest fixtures for mqtt_bridge unit tests (no ROS runtime required)."""

from __future__ import annotations

import sys
import types
from dataclasses import dataclass, field
from typing import Any, Callable
from unittest.mock import MagicMock


class _Vector3:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Twist:
    def __init__(self) -> None:
        self.linear = _Vector3()
        self.angular = _Vector3()


class _Bool:
    def __init__(self) -> None:
        self.data = False


class _String:
    def __init__(self) -> None:
        self.data = ""


class _Float32MultiArray:
    def __init__(self) -> None:
        self.data: list[float] = []


class _Time:
    nanoseconds = 1_000_000_000

    def to_msg(self) -> MagicMock:
        return MagicMock()


class _Header:
    def __init__(self) -> None:
        self.stamp = _Time()
        self.frame_id = ""


class _JointState:
    def __init__(self) -> None:
        self.header = _Header()
        self.name: list[str] = []
        self.position: list[float] = []
        self.velocity: list[float] = []
        self.effort: list[float] = []


class _JointTrajectoryPoint:
    def __init__(self) -> None:
        self.positions: list[float] = []
        self.velocities: list[float] = []
        self.time_from_start = _Time()


class _JointTrajectory:
    def __init__(self) -> None:
        self.joint_names: list[str] = []
        self.points: list[_JointTrajectoryPoint] = []


def _install_ros_stubs() -> None:
    """Install lightweight ROS/rclpy stubs so handler code can import offline."""
    if "rclpy" in sys.modules:
        return

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_publisher = types.ModuleType("rclpy.publisher")

    class Node:  # noqa: N801 - matches ROS API
        pass

    class Publisher:  # noqa: N801 - matches ROS API
        pass

    rclpy_node.Node = Node
    rclpy_publisher.Publisher = Publisher
    rclpy.node = rclpy_node
    rclpy.publisher = rclpy_publisher

    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.Twist = _Twist

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = _Bool
    std_msgs_msg.String = _String
    std_msgs_msg.Float32MultiArray = _Float32MultiArray

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = _JointState

    trajectory_msgs = types.ModuleType("trajectory_msgs")
    trajectory_msgs_msg = types.ModuleType("trajectory_msgs.msg")
    trajectory_msgs_msg.JointTrajectory = _JointTrajectory
    trajectory_msgs_msg.JointTrajectoryPoint = _JointTrajectoryPoint

    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.publisher": rclpy_publisher,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "trajectory_msgs": trajectory_msgs,
        "trajectory_msgs.msg": trajectory_msgs_msg,
    }.items():
        sys.modules[name] = module


def _install_camera_stubs() -> None:
    """Stub the cyberwave SDK camera base + sensor_msgs.Image for camera tests.

    Uses the REAL aiortc VideoStreamTrack as the BaseVideoTrack base when aiortc
    is installed (so the e2e loopback works); otherwise stubs aiortc/aioice just
    enough for ros_camera to import. Installed once, consistently, so test files
    don't fight over sys.modules.
    """
    sm = sys.modules.get("sensor_msgs.msg")
    if sm is not None and not hasattr(sm, "Image"):
        sm.Image = type("Image", (), {})

    base: Any = object
    try:
        from aiortc import VideoStreamTrack as _VST  # real aiortc if present

        base = _VST
    except Exception:
        if "aiortc.rtcicetransport" not in sys.modules:
            aiortc = sys.modules.get("aiortc") or types.ModuleType("aiortc")
            rtcice = types.ModuleType("aiortc.rtcicetransport")
            rtcice.RTCIceGatherer = type("RTCIceGatherer", (), {})
            rtcice.connection_kwargs = lambda servers: {}
            aiortc.rtcicetransport = rtcice
            sys.modules["aiortc"] = aiortc
            sys.modules["aiortc.rtcicetransport"] = rtcice
        if "aioice" not in sys.modules:
            aioice = types.ModuleType("aioice")
            aioice.Connection = type("Connection", (), {})
            aioice.TransportPolicy = type("TransportPolicy", (), {"RELAY": "relay"})
            sys.modules["aioice"] = aioice

    if "cyberwave.camera" not in sys.modules:
        cyberwave = sys.modules.get("cyberwave") or types.ModuleType("cyberwave")
        cam = types.ModuleType("cyberwave.camera")

        class BaseVideoTrack(base):  # type: ignore[misc,valid-type]
            def __init__(self) -> None:
                if base is not object:
                    super().__init__()
                self.frame_count = 0
                self.frame_0_timestamp = None
                self.frame_0_timestamp_monotonic = None
                self.sync_frame_target = 30
                self.sync_frame_pts = None

            def _capture_sync_frame(self, *a: Any, **k: Any) -> None:
                pass

            def stop(self) -> None:
                parent_stop = getattr(super(), "stop", None)
                if callable(parent_stop):
                    try:
                        parent_stop()
                    except Exception:
                        pass

        class BaseVideoStreamer:
            def __init__(self, *a: Any, **k: Any) -> None:
                self.streamer = None

        cam.BaseVideoTrack = BaseVideoTrack
        cam.BaseVideoStreamer = BaseVideoStreamer
        cyberwave.camera = cam
        sys.modules["cyberwave"] = cyberwave
        sys.modules["cyberwave.camera"] = cam


_install_ros_stubs()
_install_camera_stubs()


@dataclass
class FakePublisher:
    messages: list[Any] = field(default_factory=list)

    def publish(self, message: Any) -> None:
        self.messages.append(message)


@dataclass
class FakeTimer:
    period: float
    callback: Callable[[], Any]
    cancelled: bool = False

    def cancel(self) -> None:
        self.cancelled = True

    def run_once(self) -> None:
        if not self.cancelled:
            self.callback()


class FakeLogger:
    def debug(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def info(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def warning(self, *_args: Any, **_kwargs: Any) -> None:
        pass

    def error(self, *_args: Any, **_kwargs: Any) -> None:
        pass


class FakeMapping:
    twin_uuid = "00000000-0000-0000-0000-test-twin"


class FakeMqttAdapter:
    def __init__(self) -> None:
        self.published: list[tuple[str, str]] = []

    def publish(self, topic: str, payload: str) -> None:
        self.published.append((topic, payload))


class FakeNode:
    """Minimal ROS node stand-in for command-handler unit tests."""

    def __init__(self) -> None:
        self._mapping = FakeMapping()
        self.ros_prefix = "dev"
        self._command_registry = None
        self._mqtt_adapter = FakeMqttAdapter()
        self._publishers_by_topic: dict[str, FakePublisher] = {}
        self.timers: list[FakeTimer] = []

    def get_logger(self) -> FakeLogger:
        return FakeLogger()

    def resolve_ros_topic(self, topic: str) -> str:
        from mqtt_bridge.ros_topic_namespace import resolve_ros_topic

        namespace = str(getattr(self._mapping, "twin_uuid", "") or "").strip("/")
        return resolve_ros_topic(topic, namespace)

    def create_publisher(self, _msg_type: Any, topic: str, _qos: int) -> FakePublisher:
        publisher = FakePublisher()
        self._publishers_by_topic[topic] = publisher
        return publisher

    def create_timer(self, period: float, callback: Callable[[], Any]) -> FakeTimer:
        timer = FakeTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_clock(self) -> Any:
        clock = MagicMock()
        clock.now.return_value = _Time()
        return clock

    def publisher_for(self, topic: str) -> FakePublisher:
        return self._publishers_by_topic[topic]
