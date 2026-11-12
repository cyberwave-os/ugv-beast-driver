"""M2 §7.2 — camera sensor identity comes from the twin, never the mapping YAML."""

from __future__ import annotations

from typing import Any

import pytest

# ros_camera imports numpy/cv2/av at module load; skip in minimal CI envs.
pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("av")

from mqtt_bridge.plugins.ros_camera import ROSCameraStreamer  # noqa: E402


class _Logger:
    def info(self, *a: Any, **k: Any) -> None:
        pass

    def warning(self, *a: Any, **k: Any) -> None:
        pass

    def error(self, *a: Any, **k: Any) -> None:
        pass


class _Mapping:
    def __init__(self, raw: dict) -> None:
        self.raw = raw


class _Node:
    def __init__(self, mapping: Any = None) -> None:
        self._mapping = mapping

    def get_logger(self) -> _Logger:
        return _Logger()


def _mapping_with_camera_name(name: str) -> _Mapping:
    return _Mapping({"camera": {"camera_name": name, "sensor_id": name, "image_topic": "/image_raw"}})


def test_camera_name_from_twin_kwarg() -> None:
    node = _Node(_mapping_with_camera_name("pt_camera"))
    streamer = ROSCameraStreamer(node, client=object(), camera_name="front_camera")
    assert streamer.camera_name == "front_camera"


def test_camera_name_never_read_from_yaml() -> None:
    # Mapping YAML declares camera_name=pt_camera, but no twin identity is passed:
    # the streamer must NOT silently adopt the YAML value.
    node = _Node(_mapping_with_camera_name("pt_camera"))
    streamer = ROSCameraStreamer(node, client=object())
    assert streamer.camera_name != "pt_camera"
    assert streamer.camera_name is None  # recording disabled, never a stale YAML id


def test_hardware_capture_still_from_yaml() -> None:
    # Identity is twin-sourced; capture config stays YAML-sourced.
    node = _Node(_Mapping({"camera": {"image_topic": "/cam/image", "image_width": 800,
                                      "image_height": 600, "stream_fps": 20}}))
    streamer = ROSCameraStreamer(node, client=object(), camera_name="front_camera")
    assert streamer.image_topic == "/cam/image"
    assert streamer._capture_w == 800
    assert streamer._capture_h == 600
    assert streamer.fps == 20
