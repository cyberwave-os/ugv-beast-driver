"""Resolve the real USB camera's /dev/video* capture node via v4l2-ctl.

Robust to the index changing between boots/replugs, and skips Pi platform V4L2
nodes (pispbe, rpivid, bcm2835-*). Pure stdlib — safe to import without ROS.
"""

from __future__ import annotations

import logging
import re
import shutil
import subprocess
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

logger = logging.getLogger(__name__)

# SoC codec/ISP V4L2 nodes (not streamable cameras). Matched against card +
# bus_info case-insensitively, so the "platform:" prefix is handled.
EXCLUDED_DEVICE_TOKENS = frozenset(
    {
        "pispbe",
        "rpivid",
        "rpi-hevc-dec",
        "bcm2835-codec-decode",
        "bcm2835-isp",
        "unicam",
        "pisp-fe",
    }
)


@dataclass
class CameraDevice:
    """A camera discovered via ``v4l2-ctl --list-devices``."""

    card: str
    bus_info: str
    paths: List[str] = field(default_factory=list)

    @property
    def is_usb(self) -> bool:
        return "usb" in (self.bus_info or "").lower()

    @property
    def is_platform(self) -> bool:
        """True for non-streamable SoC platform nodes (ISP/codec)."""
        haystack = f"{self.card} {self.bus_info}".lower()
        return any(tok in haystack for tok in EXCLUDED_DEVICE_TOKENS)

    @property
    def primary_path(self) -> Optional[str]:
        return self.paths[0] if self.paths else None

    def to_dict(self) -> dict:
        return {
            "card": self.card,
            "bus_info": self.bus_info,
            "paths": list(self.paths),
            "primary_path": self.primary_path,
            "is_usb": self.is_usb,
            "is_platform": self.is_platform,
        }


def _run(cmd: List[str], timeout: float = 5.0) -> Optional[str]:
    if not shutil.which(cmd[0]):
        logger.warning("%s not found; install v4l-utils to enable camera discovery", cmd[0])
        return None
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except Exception as exc:  # pragma: no cover - defensive
        logger.debug("command failed %s: %s", cmd, exc)
        return None
    if result.returncode != 0:
        logger.debug("command returned %s: %s", result.returncode, cmd)
        return None
    return result.stdout


def parse_list_devices(output: str) -> List[CameraDevice]:
    """Parse ``v4l2-ctl --list-devices`` output into CameraDevice objects.

    Lines look like::

        USB Camera: USB Camera (usb-xhci-hcd.1-2):
                /dev/video0
                /dev/video1
                /dev/media3
    """
    devices: List[CameraDevice] = []
    current: Optional[CameraDevice] = None
    for raw in output.splitlines():
        if not raw.strip():
            continue
        if raw[0].isspace():
            path = raw.strip()
            if current is not None and path.startswith("/dev/video"):
                current.paths.append(path)
            continue
        # Header line: "Card (bus_info):"
        match = re.match(r"^(.+?)\s*\(([^)]*)\):\s*$", raw)
        if match:
            current = CameraDevice(card=match.group(1).strip(), bus_info=match.group(2).strip())
        else:
            current = CameraDevice(card=raw.rstrip(":").strip(), bus_info="")
        devices.append(current)
    return [d for d in devices if d.paths]


def discover_cameras() -> List[CameraDevice]:
    """Return all V4L2 video devices with at least one ``/dev/video*`` capture node."""
    output = _run(["v4l2-ctl", "--list-devices"])
    if output is None:
        return []
    return parse_list_devices(output)


def list_capture_formats(device_path: str) -> List[str]:
    """FOURCC capture formats a device advertises (e.g. ['MJPG','YUYV']); empty for
    metadata-only nodes — that's how we tell the capture node from the metadata one."""
    output = _run(["v4l2-ctl", f"--device={device_path}", "--list-formats-ext"])
    if not output:
        return []
    formats: List[str] = []
    for m in re.finditer(r"\[\d+\]:\s*'([A-Z0-9]+)'", output):
        if m.group(1) not in formats:
            formats.append(m.group(1))
    return formats


def _device_has_capture(device_path: str) -> bool:
    return len(list_capture_formats(device_path)) > 0


def resolve_camera_device(prefer: Optional[str] = None) -> Optional[str]:
    """Best /dev/video* capture path for the real USB camera (None if none).

    `prefer` (a /dev/video* path) wins if it has capture formats; else pick the
    first USB-bus, non-platform node that advertises a capture format, then any
    non-platform one. `prefer` None/"auto" = full auto-resolution.
    """
    if prefer and prefer != "auto" and prefer.startswith("/dev/video"):
        if _device_has_capture(prefer):
            return prefer
        logger.warning("Configured video_device %s has no capture format; auto-resolving", prefer)

    cameras = discover_cameras()
    if not cameras:
        logger.warning("No V4L2 devices found via v4l2-ctl")
        return None

    usb_cams = [c for c in cameras if c.is_usb and not c.is_platform]
    other_cams = [c for c in cameras if not c.is_usb and not c.is_platform]

    for cam in usb_cams + other_cams:
        for path in cam.paths:
            if _device_has_capture(path):
                logger.info(
                    "Resolved camera device %s (card=%r bus=%r)", path, cam.card, cam.bus_info
                )
                return path

    logger.warning(
        "No streamable capture device found among %d camera(s)", len(cameras)
    )
    return None


def parse_formats_ext(output: str) -> dict:
    """Parse --list-formats-ext into {FOURCC: {(w,h): [fps,...]}}."""
    result: dict = {}
    current_fmt: Optional[str] = None
    current_size: Optional[Tuple[int, int]] = None
    for raw in output.splitlines():
        line = raw.strip()
        fmt_m = re.search(r"\[\d+\]:\s*'([A-Z0-9]+)'", line)
        if fmt_m:
            current_fmt = fmt_m.group(1)
            result.setdefault(current_fmt, {})
            current_size = None
            continue
        size_m = re.search(r"Size:\s*Discrete\s*(\d+)x(\d+)", line)
        if size_m and current_fmt:
            current_size = (int(size_m.group(1)), int(size_m.group(2)))
            result[current_fmt].setdefault(current_size, [])
            continue
        fps_m = re.search(r"\(([\d.]+)\s*fps\)", line)
        if fps_m and current_fmt and current_size is not None:
            result[current_fmt][current_size].append(float(fps_m.group(1)))
    return result


def get_supported_formats(device_path: str) -> dict:
    """Query the hardware capability map for a device via ``--list-formats-ext``."""
    output = _run(["v4l2-ctl", f"--device={device_path}", "--list-formats-ext"])
    if not output:
        return {}
    return parse_formats_ext(output)
