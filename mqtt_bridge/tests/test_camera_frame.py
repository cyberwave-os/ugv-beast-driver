"""Step 3 gate: pure frame-transform / downscale helpers (camera_frame)."""

from __future__ import annotations

import pytest

# Needs the media libs (present on the Pi host / in the container, not in minimal CI).
pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("av")

import numpy as np  # noqa: E402

from mqtt_bridge.plugins import camera_frame as cf  # noqa: E402


def test_even_and_scaled_size():
    assert cf.even(641) == 640
    assert cf.even(480) == 480
    assert cf.scaled_size(1920, 1080, 1.0) == (1920, 1080)
    assert cf.scaled_size(1280, 720, 0.5) == (640, 360)
    assert cf.scaled_size(641, 481, 1.0) == (640, 480)  # odd -> even
    # never collapses below 2px even at tiny scale
    w, h = cf.scaled_size(10, 10, 0.01)
    assert w >= 2 and h >= 2 and w % 2 == 0


@pytest.mark.parametrize(
    "encoding,shape",
    [("rgb8", (480, 640, 3)), ("bgr8", (480, 640, 3)), ("yuyv", (480, 640, 2))],
)
def test_encode_yuv420p_full_scale(encoding, shape):
    buf = np.random.randint(0, 255, size=shape, dtype=np.uint8)
    vf = cf.encode_yuv420p(buf, encoding, scale=1.0)
    assert vf.format.name == "yuv420p"
    assert (vf.width, vf.height) == (640, 480)


def test_encode_yuv420p_downscale_halves_dimensions():
    buf = np.random.randint(0, 255, size=(1080, 1920, 3), dtype=np.uint8)
    vf = cf.encode_yuv420p(buf, "rgb8", scale=0.5)
    assert vf.format.name == "yuv420p"
    assert (vf.width, vf.height) == (960, 540)


def test_encode_handles_odd_dimensions_full_scale():
    # odd width/height must be cropped to even, not crash the encoder
    buf = np.random.randint(0, 255, size=(481, 641, 3), dtype=np.uint8)
    vf = cf.encode_yuv420p(buf, "bgr8", scale=1.0)
    assert (vf.width, vf.height) == (640, 480)


def test_yuyv_roundtrip_is_valid_image():
    # A flat YUYV buffer should decode to a uniform-ish BGR image (no exceptions).
    buf = np.full((480, 640, 2), 128, dtype=np.uint8)
    bgr = cf.to_bgr(buf, "yuyv")
    assert bgr.shape == (480, 640, 3)
