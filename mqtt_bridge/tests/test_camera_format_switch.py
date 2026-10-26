"""Step 2 gate: CameraDeviceManager managed-relaunch + the item-2 format switch.

Verifies the YUYV 640x480@15 -> MJPG 800x600@15 transition and, crucially, that
the old usb_cam process is signalled AND reaped BEFORE the new one spawns (the
fix for the EBUSY restart freeze). Runs on the host with a fake process — no ROS
or usb_cam needed.
"""

from __future__ import annotations

import signal

import pytest

from mqtt_bridge.plugins.camera_device_manager import (
    CameraDeviceManager,
    FormatValidationError,
)

# Mirrors config/mappings/robot_ugv_beast_v1.yaml `camera` (hardware only — sensor
# identity like camera_name/frame_id is NOT in the mapping; it comes from the twin).
CAMERA_CONFIG = {
    "video_device": "auto",
    "io_method": "mmap",
    "supported_formats": {
        "mjpeg2rgb": {"sizes": [[1920, 1080], [800, 600], [640, 480]], "max_fps": 30},
        "yuyv": {"sizes": [[640, 480]], "max_fps": 30},
    },
}


def make_manager(namespace=""):
    events = []

    class FakeProc:
        _next_pid = 4000

        def __init__(self, cmd, **kwargs):
            FakeProc._next_pid += 1
            self.pid = FakeProc._next_pid
            self.cmd = cmd
            self._alive = True
            events.append(("spawn", cmd))

        def poll(self):
            return None if self._alive else 0

        def wait(self, timeout=None):
            self._alive = False
            events.append(("reaped", self.pid))
            return 0

        def send_signal(self, sig):
            events.append(("signal", sig))

    mgr = CameraDeviceManager(
        CAMERA_CONFIG,
        popen_factory=lambda cmd, **kw: FakeProc(cmd, **kw),
        device_resolver=lambda prefer: "/dev/video-test-missing",  # ENOENT -> instant "free"
        sleep=lambda s: None,
        namespace=namespace,
    )
    return mgr, events


def _params(cmd):
    """Extract -p key:=value pairs from a usb_cam command list."""
    out = {}
    for i, tok in enumerate(cmd):
        if tok == "-p" and i + 1 < len(cmd):
            k, _, v = cmd[i + 1].partition(":=")
            out[k] = v
    return out


def test_start_yuyv_then_switch_to_mjpg():
    mgr, events = make_manager()

    assert mgr.start("yuyv", 640, 480, 15) is True
    p1 = _params(mgr._proc.cmd)
    assert p1["pixel_format"] == "yuyv"
    assert (p1["image_width"], p1["image_height"]) == ("640", "480")
    assert p1["framerate"] == "15.0"
    assert p1["video_device"] == "/dev/video-test-missing"

    assert mgr.reconfigure("mjpeg2rgb", 800, 600, 15) is True
    p2 = _params(mgr._proc.cmd)
    assert p2["pixel_format"] == "mjpeg2rgb"
    assert (p2["image_width"], p2["image_height"]) == ("800", "600")

    # Ordering: spawn(yuyv) -> signal -> reaped -> spawn(mjpg).
    kinds = [e[0] for e in events]
    assert kinds == ["spawn", "signal", "reaped", "spawn"], kinds
    # The old process was reaped strictly before the new spawn (no EBUSY race).
    assert kinds.index("reaped") < len(kinds) - 1 and kinds[-1] == "spawn"
    # SIGTERM (graceful) was used, not an immediate SIGKILL.
    assert events[1][1] == signal.SIGTERM


def test_reconfigure_to_unsupported_is_a_noop():
    mgr, events = make_manager()
    mgr.start("yuyv", 640, 480, 15)
    running_before = mgr._proc

    # YUYV 1920x1080 is not in the capability table (USB2 can't do it).
    with pytest.raises(FormatValidationError):
        mgr.reconfigure("yuyv", 1920, 1080, 30)

    # Bad request must NOT tear down the working stream.
    assert mgr._proc is running_before
    assert mgr.is_running() is True
    assert [e[0] for e in events] == ["spawn"]  # no teardown, no second spawn


def test_usb_cam_launched_in_namespace_so_image_raw_matches():
    # Regression: with a per-robot namespace, usb_cam must publish /<ns>/image_raw
    # or the bridge's namespaced subscription gets 0 frames.
    mgr, _ = make_manager(namespace="ugv_beast_cb5f53")
    mgr.start("mjpeg2rgb", 800, 600, 30)
    cmd = mgr._proc.cmd
    assert "-r" in cmd and "__ns:=/ugv_beast_cb5f53" in cmd

    # No namespace (single-robot/dev) -> publishes global /image_raw, no __ns remap.
    mgr2, _ = make_manager(namespace="")
    mgr2.start("mjpeg2rgb", 800, 600, 30)
    assert not any(tok.startswith("__ns:=") for tok in mgr2._proc.cmd)


def test_validate_rejects_unknown_format_and_excess_fps():
    mgr, _ = make_manager()
    with pytest.raises(FormatValidationError):
        mgr.validate("h264", 640, 480, 15)
    with pytest.raises(FormatValidationError):
        mgr.validate("mjpeg2rgb", 800, 600, 60)  # exceeds max_fps 30
    mgr.validate("mjpeg2rgb", 1920, 1080, 30)  # ok, no raise
