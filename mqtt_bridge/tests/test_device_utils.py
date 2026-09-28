"""Tests for camera device discovery (mqtt_bridge.plugins.device_utils)."""

from __future__ import annotations

import shutil
import subprocess

import pytest

from mqtt_bridge.plugins import device_utils as du

# Exact `v4l2-ctl --list-devices` output from the target Pi (USB cam at video0,
# plus Pi platform nodes pispbe/rpivid that must be excluded).
LIST_DEVICES_FIXTURE = """\
pispbe (platform:1000880000.pisp_be):
\t/dev/video20
\t/dev/video21
\t/dev/video22
\t/dev/media1
\t/dev/media2

rpivid (platform:rpivid):
\t/dev/video19
\t/dev/media0

USB Camera: USB Camera (usb-xhci-hcd.1-2):
\t/dev/video0
\t/dev/video1
\t/dev/media3
"""

LIST_FORMATS_EXT_FIXTURE = """\
ioctl: VIDIOC_ENUM_FMT
\tType: Video Capture

\t[0]: 'MJPG' (Motion-JPEG, compressed)
\t\tSize: Discrete 1920x1080
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t[1]: 'YUYV' (YUYV 4:2:2)
\t\tSize: Discrete 640x480
\t\t\tInterval: Discrete 0.033s (30.000 fps)
\t\tSize: Discrete 1280x720
\t\t\tInterval: Discrete 0.100s (10.000 fps)
"""


def test_parse_list_devices_groups_paths_by_card():
    devices = du.parse_list_devices(LIST_DEVICES_FIXTURE)
    cards = {d.card: d for d in devices}
    assert "USB Camera: USB Camera" in cards
    usb = cards["USB Camera: USB Camera"]
    assert usb.paths == ["/dev/video0", "/dev/video1"]  # /dev/media3 excluded
    assert usb.is_usb is True
    assert usb.is_platform is False


def test_platform_nodes_excluded():
    devices = du.parse_list_devices(LIST_DEVICES_FIXTURE)
    by_card = {d.card: d for d in devices}
    assert by_card["pispbe"].is_platform is True
    assert by_card["rpivid"].is_platform is True  # the gap in the SO101 util
    assert by_card["pispbe"].is_usb is False


def test_resolve_prefers_usb_capture_node(monkeypatch):
    monkeypatch.setattr(du, "discover_cameras", lambda: du.parse_list_devices(LIST_DEVICES_FIXTURE))
    # Only /dev/video0 advertises capture formats; /dev/video1 is metadata-only.
    monkeypatch.setattr(
        du, "list_capture_formats", lambda p: ["MJPG", "YUYV"] if p == "/dev/video0" else []
    )
    assert du.resolve_camera_device("auto") == "/dev/video0"


def test_resolve_skips_platform_even_if_first(monkeypatch):
    monkeypatch.setattr(du, "discover_cameras", lambda: du.parse_list_devices(LIST_DEVICES_FIXTURE))
    # Pretend a platform node also reports a capture format; it must still be skipped.
    monkeypatch.setattr(
        du,
        "list_capture_formats",
        lambda p: ["MJPG"] if p in ("/dev/video0", "/dev/video19") else [],
    )
    assert du.resolve_camera_device("auto") == "/dev/video0"


def test_explicit_prefer_path_used_when_valid(monkeypatch):
    monkeypatch.setattr(du, "list_capture_formats", lambda p: ["YUYV"])
    assert du.resolve_camera_device("/dev/video0") == "/dev/video0"


def test_parse_formats_ext_builds_capability_map():
    caps = du.parse_formats_ext(LIST_FORMATS_EXT_FIXTURE)
    assert set(caps) == {"MJPG", "YUYV"}
    assert caps["MJPG"][(1920, 1080)] == [30.0]
    assert caps["YUYV"][(1280, 720)] == [10.0]


# --- Real-hardware checks (only when run on the Pi with a camera attached) ---

_HAS_V4L2 = shutil.which("v4l2-ctl") is not None


@pytest.mark.skipif(not _HAS_V4L2, reason="v4l2-ctl not available")
def test_real_discovery_finds_usb_camera():
    cams = du.discover_cameras()
    assert cams, "no V4L2 devices discovered"
    # On the target Pi there is a real USB camera.
    usb = [c for c in cams if c.is_usb and not c.is_platform]
    if not usb:
        pytest.skip("no USB camera attached to this host")
    path = du.resolve_camera_device("auto")
    assert path and path.startswith("/dev/video")
    fmts = du.list_capture_formats(path)
    assert fmts, f"resolved {path} but it advertises no capture formats"
