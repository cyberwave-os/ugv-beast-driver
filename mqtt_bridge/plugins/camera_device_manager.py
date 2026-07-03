"""Managed usb_cam lifecycle so the bridge can change format at runtime.

Single ownership with confirmed teardown: on a format change, SIGTERM the old
usb_cam, wait for it to be reaped, confirm /dev/video0 is free, then spawn the
new one — this kills the EBUSY crash-loop of the old respawn=True launch node.
ROS-free (subprocess only) and unit-testable; spawn/resolve/sleep are injectable.
"""

from __future__ import annotations

import errno
import logging
import os
import signal
import subprocess
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from . import device_utils

logger = logging.getLogger(__name__)

Format = Tuple[str, int, int, int]  # (pixel_format, width, height, fps)


class FormatValidationError(ValueError):
    """Raised when a requested camera format is not supported."""


# Watchdog recovery actions (pure decision, executed by the node).
RECOVERY_NONE = "none"
RECOVERY_RESTART = "restart"        # usb_cam process gone -> respawn it
RECOVERY_RECONFIGURE = "reconfigure"  # device alive but no frames -> clean cycle
RECOVERY_RERESOLVE = "reresolve"    # device path missing -> re-resolve (USB replug)


def decide_recovery(
    *,
    managed: bool,
    device_exists: bool,
    usb_cam_running: bool,
    receiving: bool,
    streaming: bool,
    silent_secs: float,
    silent_threshold: float = 10.0,
) -> str:
    """Pick a watchdog recovery action (managed camera only; pure, no side effects).

    Priority: device gone -> re-resolve; usb_cam dead -> restart; alive but silent
    while streaming -> reconfigure.
    """
    if not managed:
        return RECOVERY_NONE
    if not device_exists:
        return RECOVERY_RERESOLVE
    if not usb_cam_running:
        return RECOVERY_RESTART
    if not receiving and streaming and silent_secs > silent_threshold:
        return RECOVERY_RECONFIGURE
    return RECOVERY_NONE


class CameraDeviceManager:
    def __init__(
        self,
        camera_config: Dict[str, Any],
        log: Any = None,
        *,
        popen_factory: Callable[..., Any] = subprocess.Popen,
        device_resolver: Optional[Callable[[Optional[str]], Optional[str]]] = None,
        sleep: Callable[[float], None] = time.sleep,
        env: Optional[Dict[str, str]] = None,
        namespace: str = "",
    ) -> None:
        self._cfg = camera_config or {}
        # Per-robot ROS namespace: usb_cam must publish /<ns>/image_raw to match
        # the bridge's namespaced subscription (else the track gets 0 frames).
        self.namespace = str(namespace or "").strip().strip("/")
        self._log = log or logging.getLogger(__name__)
        self._popen = popen_factory
        self._resolve = device_resolver or device_utils.resolve_camera_device
        self._sleep = sleep
        self._env = env

        self.supported_formats: Dict[str, Any] = self._cfg.get("supported_formats", {}) or {}
        self.io_method: str = self._cfg.get("io_method", "mmap")
        self.frame_id: str = self._cfg.get("frame_id", "camera_link")
        self.camera_name: str = self._cfg.get("camera_name", "camera")
        self.camera_info_url: str = self._cfg.get("camera_info_url", "")

        # Resolve the device path now (handles "auto" / index drift).
        configured = self._cfg.get("video_device", "auto")
        self.video_device: str = self._resolve(configured) or (
            configured if isinstance(configured, str) and configured.startswith("/dev/video") else "/dev/video0"
        )

        self._proc: Optional[Any] = None
        self.current: Optional[Format] = None

    # ------------------------------------------------------------------ helpers
    def _info(self, msg: str) -> None:
        try:
            self._log.info(msg)
        except Exception:
            logger.info(msg)

    def _warn(self, msg: str) -> None:
        try:
            self._log.warning(msg)
        except Exception:
            logger.warning(msg)

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def reresolve_device(self) -> str:
        """Re-resolve the camera path (e.g. after a USB re-plug moved the index)."""
        resolved = self._resolve(self._cfg.get("video_device", "auto"))
        if resolved and resolved != self.video_device:
            self._info(f"Camera device path changed {self.video_device} -> {resolved}")
            self.video_device = resolved
        return self.video_device

    # --------------------------------------------------------------- validation
    def validate(self, pixel_format: str, width: int, height: int, fps: int) -> None:
        """Reject formats not in the capability table (e.g. YUYV 1080p30 over USB2)
        before they reach usb_cam and crash-loop. Raises FormatValidationError."""
        if not self.supported_formats:
            return  # no table configured -> trust the caller
        entry = self.supported_formats.get(pixel_format)
        if entry is None:
            raise FormatValidationError(
                f"pixel_format {pixel_format!r} not supported; "
                f"available: {sorted(self.supported_formats)}"
            )
        sizes = {tuple(s) for s in entry.get("sizes", [])}
        if (width, height) not in sizes:
            raise FormatValidationError(
                f"{pixel_format} {width}x{height} not supported; "
                f"available sizes: {sorted(sizes)}"
            )
        max_fps = entry.get("max_fps")
        if max_fps is not None and fps > max_fps:
            raise FormatValidationError(
                f"{pixel_format} {width}x{height}@{fps} exceeds max_fps {max_fps}"
            )

    # ------------------------------------------------------------------- spawn
    def _build_command(self, pixel_format: str, width: int, height: int, fps: int) -> List[str]:
        cmd = [
            "ros2", "run", "usb_cam", "usb_cam_node_exe",
            "--ros-args",
            "-r", "__node:=usb_cam",
        ]
        if self.namespace:
            # Namespace usb_cam so it publishes /<ns>/image_raw, matching the
            # bridge's namespaced subscription (twin-scoped fleet graph).
            cmd += ["-r", f"__ns:=/{self.namespace}"]
        cmd += [
            "-p", f"video_device:={self.video_device}",
            "-p", f"pixel_format:={pixel_format}",
            "-p", f"image_width:={int(width)}",
            "-p", f"image_height:={int(height)}",
            "-p", f"framerate:={float(fps)}",
            "-p", f"io_method:={self.io_method}",
            "-p", f"frame_id:={self.frame_id}",
            "-p", f"camera_name:={self.camera_name}",
        ]
        if self.camera_info_url:
            cmd += ["-p", f"camera_info_url:={self.camera_info_url}"]
        return cmd

    def start(self, pixel_format: str, width: int, height: int, fps: int) -> bool:
        """Validate + spawn usb_cam with the given format. Returns True on spawn."""
        self.validate(pixel_format, width, height, fps)
        if self.is_running():
            self._warn("CameraDeviceManager.start called while already running; stopping first")
            self.stop()
        cmd = self._build_command(pixel_format, width, height, fps)
        self._info(f"Starting usb_cam: {' '.join(cmd)}")
        # Own process group so stop() can signal the whole group (ros2 children too).
        self._proc = self._popen(cmd, env=self._env, start_new_session=True)
        self.current = (pixel_format, int(width), int(height), int(fps))
        return True

    def stop(self, timeout: float = 5.0) -> None:
        """Terminate usb_cam, wait for it to be reaped, then confirm device free
        (reaping before the next spawn is what avoids the EBUSY race)."""
        proc = self._proc
        if proc is None:
            return
        self._info("Stopping usb_cam (SIGTERM -> wait -> SIGKILL fallback)")
        try:
            self._signal_group(proc, signal.SIGTERM)
        except Exception as exc:
            self._warn(f"SIGTERM failed: {exc}")
        if not self._wait_proc(proc, timeout):
            self._warn("usb_cam did not exit on SIGTERM; sending SIGKILL")
            try:
                self._signal_group(proc, signal.SIGKILL)
            except Exception as exc:
                self._warn(f"SIGKILL failed: {exc}")
            self._wait_proc(proc, timeout)
        self._proc = None
        self.current = None
        self._wait_device_free(self.video_device, timeout=timeout)

    def reconfigure(self, pixel_format: str, width: int, height: int, fps: int) -> bool:
        """Switch format: validate -> stop(old) -> start(new)."""
        # Validate first so a bad request is a no-op, not a dead camera.
        self.validate(pixel_format, width, height, fps)
        self.stop()
        return self.start(pixel_format, width, height, fps)

    # ------------------------------------------------------------- process util
    @staticmethod
    def _signal_group(proc: Any, sig: int) -> None:
        pid = proc.pid
        try:
            os.killpg(os.getpgid(pid), sig)
        except (ProcessLookupError, PermissionError, OSError):
            # Fall back to signalling the process directly.
            try:
                proc.send_signal(sig)
            except Exception:
                pass

    def _wait_proc(self, proc: Any, timeout: float) -> bool:
        try:
            proc.wait(timeout=timeout)
            return True
        except Exception:
            return proc.poll() is not None

    def _wait_device_free(self, device: str, timeout: float = 5.0, poll: float = 0.2) -> bool:
        """Best-effort open-probe so the next spawn doesn't race buffer release.
        (Real safety is stop() reaping the old process; EBUSY only hits STREAMON.)"""
        deadline_steps = max(1, int(timeout / poll))
        for _ in range(deadline_steps):
            try:
                fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
                os.close(fd)
                return True
            except OSError as exc:
                if exc.errno not in (errno.EBUSY, errno.EACCES):
                    return True  # ENOENT/other: not a busy condition we can wait out
                self._sleep(poll)
        self._warn(f"Device {device} still not free after {timeout}s")
        return False
