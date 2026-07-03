import asyncio
import fractions
import time
import threading
from typing import Optional, Any, Dict
import logging

import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

from cyberwave.camera import BaseVideoTrack, BaseVideoStreamer
from aiortc.rtcicetransport import RTCIceGatherer, connection_kwargs
from aioice import Connection, TransportPolicy

from . import camera_frame as cf
from .camera_adaptation import AdaptationConfig, AdaptationController

# Fixed 90 kHz presentation clock: wall-clock-derived pts keeps timestamps
# continuous when the adaptive controller changes send fps at runtime.
_PTS_CLOCK_HZ = 90000

# Absolute last-resort size for the blank pre-roll frame, used ONLY if neither
# the camera config nor a received frame has provided a size yet. Real dimensions
# come from the camera config and are corrected from the first frame.
_LAST_RESORT_SIZE = (640, 480)

logger = logging.getLogger(__name__)

# ROS sensor_msgs / usb_cam encodings treated as packed YUYV (no BGR in callback).
_YUYV_ENCODINGS = frozenset({"yuyv", "yuv422_yuy2"})


class RelayOnlyRTCIceGatherer(RTCIceGatherer):
    """
    Custom RTCIceGatherer that forces TURN relay-only mode.

    This is needed because aiortc's RTCConfiguration doesn't support iceTransportPolicy,
    but the underlying aioice.Connection does support transport_policy.
    """

    def __init__(
        self,
        iceServers=None,
        local_username: Optional[str] = None,
        local_password: Optional[str] = None,
    ) -> None:
        from pyee.asyncio import AsyncIOEventEmitter
        AsyncIOEventEmitter.__init__(self)

        if iceServers is None:
            iceServers = self.getDefaultIceServers()
        ice_kwargs = connection_kwargs(iceServers)

        # Force RELAY mode
        ice_kwargs['transport_policy'] = TransportPolicy.RELAY
        logger.info("ICE transport policy set to RELAY (force_turn enabled)")

        self._connection = Connection(ice_controlling=False, **ice_kwargs)
        self._remote_candidates_end = False
        self._RTCIceGatherer__state = "new"


def enable_relay_only_ice_mode() -> None:
    """Permanently patch aiortc to use relay-only ICE mode for this process.

    Unlike a context manager, this patch persists across WebRTC reconnections so
    every peer connection created after this call (including auto-reconnect attempts)
    will use TURN relay-only transport.  Call this once during streamer initialisation
    when force_relay=True.
    """
    import aiortc.rtcpeerconnection as rtcpc
    if rtcpc.RTCIceGatherer is not RelayOnlyRTCIceGatherer:
        rtcpc.RTCIceGatherer = RelayOnlyRTCIceGatherer
        logger.info("Permanently enabled relay-only ICE mode (force_turn)")


class ROSVideoStreamTrack(BaseVideoTrack):
    """
    Video stream track that gets frames from a ROS 2 topic.
    """
    def __init__(
        self,
        node: Node,
        topic: str = "/image_raw",
        fps: int = 30,
        width: Optional[int] = None,
        height: Optional[int] = None,
    ):
        super().__init__()
        self.node = node
        # Resolve the (logical) image topic through the node's per-robot ROS
        # namespace — exactly like every other bridge topic (resolve_ros_topic) —
        # so the subscription matches the namespaced /image_raw the usb_cam node
        # publishes. Without this, video silently never arrives in a namespaced
        # (fleet) deployment. self.topic is the resolved name everywhere below.
        # Falls back to the raw topic if the node exposes no resolver.
        resolve = getattr(node, "resolve_ros_topic", None)
        self.topic = resolve(topic) if callable(resolve) else topic
        self.fps = fps
        self.encoding = "yuv420p"
        # Adaptive knobs (set by the controller): _target_fps = WebRTC send rate,
        # _scale = pre-encode downscale. Bridge-side only; never touch the camera.
        self._target_fps = float(fps)
        self._scale = 1.0
        self._quality_lock = threading.Lock()
        self._last_pts = -1
        self.latest_frame = None
        self.latest_frame_encoding: Optional[str] = None
        self._frame_lock = threading.Lock()
        self._last_time = None
        self._last_log_time = 0
        self._frames_received = 0
        self._frame_ready_event = threading.Event()
        
        # Initial dimensions: explicit args (from the streamer's camera config),
        # else the mapping's camera block. May stay None until the first frame —
        # _image_callback sets the authoritative size from the frame itself.
        self.actual_width = width
        self.actual_height = height
        if (self.actual_width is None or self.actual_height is None) and \
                hasattr(self.node, '_mapping') and self.node._mapping:
            camera_config = self.node._mapping.raw.get('camera', {})
            self.actual_width = self.actual_width or camera_config.get('image_width')
            self.actual_height = self.actual_height or camera_config.get('image_height')

        # Subscribe to the ROS image topic (already namespace-resolved above).
        self.subscription = self.node.create_subscription(
            Image, self.topic, self._image_callback, 10
        )
        self.node.get_logger().info(f"ROSVideoStreamTrack subscribed to {self.topic}")

    def _image_callback(self, msg):
        try:
            now = time.time()
            if now - self._last_log_time > 10:
                self.node.get_logger().info(
                    f"ROSCameraStreamer: {self.topic} {msg.encoding} {msg.width}x{msg.height}"
                )
                self._last_log_time = now

            if self.latest_frame is None:
                self.node.get_logger().info(f"FIRST FRAME on {self.topic}!")

            # Store native ROS payload; convert to yuv420p only in recv() (stream rate).
            if msg.encoding in _YUYV_ENCODINGS:
                frame_buffer = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 2)
                )
                frame_encoding = "yuyv"
            elif msg.encoding in ("rgb8", "bgr8"):
                frame_buffer = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    (msg.height, msg.width, 3)
                )
                frame_encoding = msg.encoding
            else:
                self.node.get_logger().error(f"Unsupported image encoding: {msg.encoding}")
                return

            # H.264 / yuv420p require even width and height.
            h, w = frame_buffer.shape[:2]
            if h % 2 != 0 or w % 2 != 0:
                frame_buffer = frame_buffer[: h & ~1, : w & ~1]

            with self._frame_lock:
                self.latest_frame = np.ascontiguousarray(frame_buffer, dtype=np.uint8)
                self.latest_frame_encoding = frame_encoding
                self.actual_height, self.actual_width = frame_buffer.shape[:2]
                self._frames_received += 1
                if hasattr(self.node, '_last_image_time'):
                    self.node._last_image_time = now
            
            # Signal that we have at least one frame ready
            if not self._frame_ready_event.is_set():
                self._frame_ready_event.set()
                self.node.get_logger().info(f"Frame buffer ready for WebRTC streaming")
                
        except Exception as e:
            self.node.get_logger().error(f"Error processing ROS image: {e}")

    def set_quality(self, fps: Optional[float] = None, scale: Optional[float] = None) -> None:
        """Set the adaptive send rate / downscale (thread-safe). fps clamped >=1,
        scale to (0.05, 1.0]; applies from the next recv()."""
        with self._quality_lock:
            if fps is not None:
                self._target_fps = max(1.0, float(fps))
            if scale is not None:
                self._scale = min(1.0, max(0.05, float(scale)))

    def get_quality(self) -> tuple:
        with self._quality_lock:
            return self._target_fps, self._scale

    def get_stream_attributes(self) -> Dict[str, Any]:
        target_fps, scale = self.get_quality()
        if self.actual_width and self.actual_height:
            width, height = cf.scaled_size(self.actual_width, self.actual_height, scale)
        else:
            width = height = 0  # unknown until the first frame / config
        return {
            "camera_type": "ros",
            "camera_id": self.topic,
            "width": width,
            "height": height,
            "fps": int(round(target_fps)),
        }
    
    def wait_for_frames(self, timeout: float = 5.0) -> bool:
        """
        Wait for frames to be available before starting WebRTC.
        
        Args:
            timeout: Maximum time to wait in seconds
            
        Returns:
            True if frames are ready, False if timeout occurred
        """
        return self._frame_ready_event.wait(timeout)
    
    def has_frames(self) -> bool:
        """Check if any frames have been received."""
        return self._frame_ready_event.is_set()

    def get_latest_frame_bgr(self) -> Optional[np.ndarray]:
        """Return the latest frame as BGR for JPEG snapshot (take_photo)."""
        with self._frame_lock:
            frame_data = self.latest_frame
            encoding = self.latest_frame_encoding
        if frame_data is None or encoding is None:
            return None
        if encoding in _YUYV_ENCODINGS or encoding == "yuyv":
            return cv2.cvtColor(frame_data, cv2.COLOR_YUV2BGR_YUYV)
        if encoding == "rgb8":
            return cv2.cvtColor(frame_data, cv2.COLOR_RGB2BGR)
        return frame_data.copy()

    async def recv(self):
        # Wait for at least one frame to be ready before starting WebRTC streaming
        if self.frame_count == 0:
            # Wait up to 5 seconds for the first frame
            self.node.get_logger().info(f"Waiting for first frame on {self.topic}...")
            await asyncio.get_event_loop().run_in_executor(
                None, self._frame_ready_event.wait, 5.0
            )
            if not self._frame_ready_event.is_set():
                self.node.get_logger().error(
                    f"No frames received on {self.topic} after 5s, starting with blank frame"
                )
            else:
                self.node.get_logger().info(f"First frame ready on {self.topic}, starting WebRTC transmission")
        
        target_fps, scale = self.get_quality()

        # Pace at the adaptive send rate (skip first frame to avoid SDK timeout).
        if self.frame_count > 0:
            now = time.time()
            if self._last_time is not None:
                wait = max(0, (1.0 / target_fps) - (now - self._last_time))
                if wait > 0:
                    await asyncio.sleep(wait)
        self._last_time = time.time()

        self.frame_count += 1

        with self._frame_lock:
            frame_data = self.latest_frame
            frame_encoding = self.latest_frame_encoding
            frames_received = self._frames_received

        if frame_data is None:
            # Blank gray pre-roll frame; size from config, else last-resort default.
            bw = self.actual_width or _LAST_RESORT_SIZE[0]
            bh = self.actual_height or _LAST_RESORT_SIZE[1]
            self.node.get_logger().warning(
                f"Frame {self.frame_count}: No frame data available, sending blank {bw}x{bh} frame (received {frames_received} total)"
            )
            frame_data = np.full((bh, bw, 3), 128, dtype=np.uint8)
            frame_encoding = "bgr8"
        elif self.frame_count == 1:
            self.node.get_logger().info(
                f"Starting WebRTC stream with cached frame (received {frames_received} frames so far)"
            )
        elif self.frame_count % 100 == 0:
            # Log every 100 frames to confirm streaming
            self.node.get_logger().debug(
                f"Frame {self.frame_count}: Sending frame to WebRTC ({frames_received} total received)"
            )

        now = time.time()
        now_monotonic = time.monotonic()

        if self.frame_count == 1:
            self.frame_0_timestamp = now
            self.frame_0_timestamp_monotonic = now_monotonic

        # Wall-clock pts in a fixed 90 kHz time_base (robust to dynamic fps).
        time_base = fractions.Fraction(1, _PTS_CLOCK_HZ)
        pts = int(round((now_monotonic - self.frame_0_timestamp_monotonic) * _PTS_CLOCK_HZ))
        if pts <= self._last_pts:
            pts = self._last_pts + 1
        self._last_pts = pts

        # Convert+downscale off the event loop so a big frame can't stall signaling.
        loop = asyncio.get_event_loop()
        frame = await loop.run_in_executor(
            None, cf.encode_yuv420p, frame_data, frame_encoding or "bgr8", scale
        )
        frame.pts = pts
        frame.time_base = time_base

        # Capture sync frame data so the SDK can publish a camera_sync_frame MQTT
        # message after streaming starts. This anchor is required for the backend to
        # correctly trim and timestamp the recording for Replay.
        self._capture_sync_frame(
            now,
            now_monotonic,
            frame_index=self.frame_count,
            pts=pts,
            time_base_num=time_base.numerator,
            time_base_den=time_base.denominator,
        )

        # Keyframe ~every 4s or first 10 frames. key_frame is read-only in some av
        # versions and only a hint, so guard it.
        keyframe_interval = max(1, int(round(target_fps)) * 4)
        if self.frame_count % keyframe_interval == 1 or self.frame_count < 10:
            try:
                frame.key_frame = True
            except (AttributeError, TypeError):
                pass

        return frame

    def close(self):
        if self.subscription:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None
        super().stop()


class ROSCameraStreamer(BaseVideoStreamer):
    """
    Uses SDK's BaseVideoStreamer with ROS image source.

    Args:
        node: ROS 2 node instance
        client: MQTT client for signaling
        force_relay: If True, forces all WebRTC traffic through TURN relay servers.
                    This bypasses NAT/firewall issues but adds latency.  The relay
                    patch is applied permanently (process-wide) so it survives
                    auto-reconnect cycles.
    """
    def __init__(self, node: Node, client: Any, *args, **kwargs):
        self.fps = kwargs.pop('fps', 30)
        kwargs.pop('time_reference', None)
        # Extract force_relay before passing to parent (parent doesn't know about it)
        self.force_relay = kwargs.pop('force_relay', False)

        # Apply the relay-only ICE patch before the parent creates any peer connection.
        # Using a permanent patch (not a context manager) ensures every future
        # reconnect attempt also uses relay-only transport.
        if self.force_relay:
            enable_relay_only_ice_mode()

        # Populate camera_name from mapping if not explicitly provided.
        # The media service requires a non-None sensor field in the WebRTC offer to
        # start a recording; without it the backend logs an error and skips recording,
        # which is why UGV streams never appeared in the Replay tab.
        if 'camera_name' not in kwargs or kwargs.get('camera_name') is None:
            mapping_camera_name = None
            if hasattr(node, '_mapping') and node._mapping:
                camera_config = node._mapping.raw.get('camera', {})
                mapping_camera_name = camera_config.get('camera_name') or camera_config.get('sensor_id')
            if mapping_camera_name:
                kwargs['camera_name'] = mapping_camera_name

        super().__init__(client, *args, **kwargs)
        self.node = node

        # Get camera settings from robot mapping (preferred) or fall back to defaults
        camera_config: Dict[str, Any] = {}
        if hasattr(self.node, '_mapping') and self.node._mapping:
            camera_config = self.node._mapping.raw.get('camera', {})
            self.image_topic = camera_config.get('image_topic', '/image_raw')
            # Prefer the explicit WebRTC send rate; fall back to legacy 'fps'.
            self.fps = camera_config.get('stream_fps', camera_config.get('fps', self.fps))
        else:
            self.image_topic = "/image_raw"

        # CPU-driven adaptation (bridge-side downscale + send-fps throttle).
        self._adapt_cfg = AdaptationConfig.from_mapping(camera_config)
        self._adapt_enabled = bool(
            (camera_config.get('adaptation', {}) or {}).get('enabled', False)
            and self._adapt_cfg.ladder
        )
        # Capture resolution -> aspect-preserving downscale factor for the resize.
        self._capture_w = int(camera_config.get('image_width', 1920) or 1920)
        self._capture_h = int(camera_config.get('image_height', 1080) or 1080)
        self._pixel_format = camera_config.get('pixel_format', 'mjpeg2rgb')
        self._adaptation: Optional[AdaptationController] = None
        self._adaptation_task = None

        mode_str = " (TURN relay-only)" if self.force_relay else ""
        self.node.get_logger().info(
            f"ROSCameraStreamer: {self.image_topic} @ {self.fps}fps{mode_str}"
            + (f", camera_name={self.camera_name}" if self.camera_name else ", camera_name=None (recording disabled)")
        )

    def initialize_track(self) -> ROSVideoStreamTrack:
        """Required by BaseVideoStreamer: create the video track."""
        if self.streamer is not None:
            return self.streamer
        self.streamer = ROSVideoStreamTrack(
            self.node, self.image_topic, self.fps,
            width=self._capture_w, height=self._capture_h,
        )
        return self.streamer

    def _build_stream_config(self) -> Optional[Dict[str, Any]]:
        """Effective (post-adaptation) stream config for the SDK health heartbeat:
        live format/resolution/send fps/scale + current rung and CPU load."""
        track = self.streamer
        if track is None:
            return None
        try:
            target_fps, scale = track.get_quality()
            width, height = cf.scaled_size(track.actual_width, track.actual_height, scale)
            cfg: Dict[str, Any] = {
                "kind": "ros_webrtc",
                "source": self.image_topic,
                "pixel_format": self._pixel_format,
                "width": width,
                "height": height,
                "actual_fps": int(round(target_fps)),
                "scale": round(float(scale), 3),
            }
            if self._adaptation is not None:
                cfg["ladder_step"] = self._adaptation.index
                if self._adaptation.cpu_ema is not None:
                    cfg["cpu_ema"] = round(float(self._adaptation.cpu_ema), 1)
            return cfg
        except Exception:
            return None

    async def start(self, *args, **kwargs):
        """
        Start the WebRTC camera stream.

        Waits for frames to be available before starting WebRTC to avoid
        'Timeout waiting for first frame' warnings.
        """
        # Ensure track is initialized
        if self.streamer is None:
            self.initialize_track()

        # Wait for frames to be ready (up to 10 seconds)
        self.node.get_logger().info("Waiting for camera frames before starting WebRTC...")
        frame_ready = await asyncio.get_event_loop().run_in_executor(
            None, self.streamer.wait_for_frames, 10.0
        )

        if frame_ready:
            self.node.get_logger().info(
                f"Camera frames ready! Starting WebRTC with {self.streamer._frames_received} cached frames"
            )
        else:
            self.node.get_logger().warning(
                "No camera frames after 10s wait. Starting WebRTC anyway (will send blank frames)"
            )

        # Delegate to the SDK's WebRTC setup (don't override its signaling — races
        # with the SDK state machine on reconnect).
        result = await super().start(*args, **kwargs)
        self._start_adaptation()
        return result

    async def stop(self, *args, **kwargs):
        self._stop_adaptation()
        return await super().stop(*args, **kwargs)

    # --------------------------------------------------------- CPU adaptation
    def _apply_rung(self, rung) -> None:
        """Apply a ladder rung to the live track as (fps, aspect-preserving scale)."""
        if self.streamer is None:
            return
        width, height, fps = rung
        cap_w = getattr(self.streamer, "actual_width", 0) or self._capture_w
        cap_h = getattr(self.streamer, "actual_height", 0) or self._capture_h
        scale = 1.0
        if cap_w and cap_h:
            scale = min(1.0, min(width / float(cap_w), height / float(cap_h)))
        self.streamer.set_quality(fps=fps, scale=scale)
        self.node.get_logger().info(
            f"Adaptation: rung {width}x{height}@{fps} -> scale={scale:.3f} on {cap_w}x{cap_h} capture"
        )

    def _start_adaptation(self) -> None:
        if not self._adapt_enabled or self.streamer is None:
            return
        if self._adaptation_task is not None and not self._adaptation_task.done():
            return
        self._adaptation = AdaptationController(
            self._adapt_cfg, apply_fn=self._apply_rung, clock=time.monotonic
        )
        self._adaptation_task = asyncio.ensure_future(self._run_adaptation())
        self.node.get_logger().info(
            f"CPU adaptation started (cpu_high={self._adapt_cfg.cpu_high}, "
            f"fps_floor={self._adapt_cfg.fps_floor}, rungs={len(self._adapt_cfg.ladder or [])})"
        )

    def _stop_adaptation(self) -> None:
        task = self._adaptation_task
        self._adaptation_task = None
        if task is not None and not task.done():
            task.cancel()

    async def _run_adaptation(self) -> None:
        try:
            import psutil
        except Exception as exc:  # pragma: no cover - psutil always present in container
            self.node.get_logger().warning(f"psutil unavailable, adaptation disabled: {exc}")
            return
        psutil.cpu_percent(None)  # prime the first (meaningless) sample
        try:
            while self.pc is not None and self.streamer is not None:
                await asyncio.sleep(self._adapt_cfg.check_period_s)
                try:
                    cpu = psutil.cpu_percent(None)
                    changed = self._adaptation.update_cpu(cpu)
                    if changed is not None:
                        self.node.get_logger().info(
                            f"CPU {cpu:.0f}% (ema {self._adaptation.cpu_ema:.0f}) "
                            f"-> rung {self._adaptation.index} {changed}"
                        )
                except Exception as exc:
                    self.node.get_logger().debug(f"adaptation tick error: {exc}")
        except asyncio.CancelledError:
            pass
