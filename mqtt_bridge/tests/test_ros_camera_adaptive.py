"""Step 3 gate: adaptive ROSVideoStreamTrack (set_quality, off-loop encode, pts).

The SDK / aiortc / sensor_msgs.Image stubs are installed once by conftest
(_install_camera_stubs), so the track's NEW logic — mutable fps/scale, scaled
encode, wall-clock pts — can be driven with real numpy/cv2/av frames on the host.
"""

from __future__ import annotations

import asyncio

import pytest

# ros_camera imports cv2/av; skip in minimal CI envs without the media libs.
pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("av")

import numpy as np  # noqa: E402

from mqtt_bridge.plugins.ros_camera import ROSVideoStreamTrack  # noqa: E402


class _Logger:
    def info(self, *a, **k):
        pass

    debug = warning = error = info


class _Mapping:
    raw = {"camera": {"image_width": 1920, "image_height": 1080}}


class _Node:
    _mapping = _Mapping()
    _last_image_time = 0.0

    def get_logger(self):
        return _Logger()

    def resolve_ros_topic(self, t):
        return t

    def create_subscription(self, *a, **k):
        return object()

    def destroy_subscription(self, *a, **k):
        pass


def _make_track(fps=30):
    track = ROSVideoStreamTrack(_Node(), "/image_raw", fps=fps)
    track.actual_width, track.actual_height = 1920, 1080
    track.latest_frame = np.full((1080, 1920, 3), 128, dtype=np.uint8)
    track.latest_frame_encoding = "bgr8"
    track._frame_ready_event.set()
    return track


def test_set_quality_clamps():
    t = _make_track()
    t.set_quality(fps=0, scale=2.0)
    fps, scale = t.get_quality()
    assert fps == 1.0 and scale == 1.0
    t.set_quality(fps=15, scale=0.0)
    fps, scale = t.get_quality()
    assert fps == 15.0 and scale == 0.05


def test_stream_attributes_reflect_scale_and_fps():
    t = _make_track()
    t.set_quality(fps=15, scale=0.5)
    attrs = t.get_stream_attributes()
    assert (attrs["width"], attrs["height"]) == (960, 540)
    assert attrs["fps"] == 15


def test_recv_produces_yuv420p_at_adapted_resolution():
    t = _make_track(fps=120)  # high fps -> negligible pacing sleeps in test

    async def drive():
        f1 = await t.recv()  # full scale
        t.set_quality(scale=0.5)
        f2 = await t.recv()  # half scale
        return f1, f2

    f1, f2 = asyncio.run(drive())
    assert f1.format.name == "yuv420p"
    assert (f1.width, f1.height) == (1920, 1080)
    assert (f2.width, f2.height) == (960, 540)
    # Wall-clock pts strictly increasing, fixed 90kHz time base.
    assert f2.pts > f1.pts
    assert f1.time_base.denominator == 90000


def test_recv_blank_frame_when_no_data():
    t = _make_track()
    t.latest_frame = None  # force blank-frame path
    frame = asyncio.run(t.recv())
    assert frame.format.name == "yuv420p"
    assert (frame.width, frame.height) == (1920, 1080)
