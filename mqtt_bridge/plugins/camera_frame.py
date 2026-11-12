"""Pure frame transforms for the ROS->WebRTC track (numpy/cv2/av only, host-testable).

Encoding strings follow usb_cam's sensor_msgs/Image: yuyv / yuv422_yuy2, rgb8, bgr8.
"""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np
from av import VideoFrame

# usb_cam / sensor_msgs encodings treated as packed YUYV.
YUYV_ENCODINGS = frozenset({"yuyv", "yuv422_yuy2"})


def even(n: int) -> int:
    """Largest even integer <= n (H.264 / yuv420p require even dimensions)."""
    return int(n) & ~1


def scaled_size(width: int, height: int, scale: float) -> Tuple[int, int]:
    """Target (w, h) for a downscale factor, snapped to even and clamped >= 2."""
    if scale >= 0.999:
        return even(width), even(height)
    w = max(2, even(int(round(width * scale))))
    h = max(2, even(int(round(height * scale))))
    return w, h


def to_bgr(frame_data: np.ndarray, encoding: str) -> np.ndarray:
    """Convert a raw ROS frame buffer to BGR24 (OpenCV's native layout)."""
    if encoding in YUYV_ENCODINGS:
        return cv2.cvtColor(frame_data, cv2.COLOR_YUV2BGR_YUYV)
    if encoding == "rgb8":
        return cv2.cvtColor(frame_data, cv2.COLOR_RGB2BGR)
    # bgr8 / bgr24 / anything else already BGR-like
    return frame_data


def encode_yuv420p(frame_data: np.ndarray, encoding: str, scale: float = 1.0) -> VideoFrame:
    """Build a yuv420p av.VideoFrame, optionally downscaled by `scale`.

    scale==1.0 takes a fast path (even-dim crop only); scale<1.0 resizes
    (INTER_AREA) before reformat — the main CPU saving when adapting under load.
    """
    if scale >= 0.999:
        if encoding in YUYV_ENCODINGS:
            h, w = frame_data.shape[:2]
            if h % 2 or w % 2:
                frame_data = frame_data[: even(h), : even(w)]
            vf = VideoFrame.from_ndarray(np.ascontiguousarray(frame_data), format="yuyv422")
            return vf.reformat(format="yuv420p")
        bgr = to_bgr(frame_data, encoding)
        h, w = bgr.shape[:2]
        if h % 2 or w % 2:
            bgr = bgr[: even(h), : even(w)]
        vf = VideoFrame.from_ndarray(np.ascontiguousarray(bgr), format="bgr24")
        return vf.reformat(format="yuv420p")

    # Downscale path
    bgr = to_bgr(frame_data, encoding)
    h, w = bgr.shape[:2]
    tw, th = scaled_size(w, h, scale)
    if (tw, th) != (w, h):
        bgr = cv2.resize(bgr, (tw, th), interpolation=cv2.INTER_AREA)
    vf = VideoFrame.from_ndarray(np.ascontiguousarray(bgr), format="bgr24")
    return vf.reformat(format="yuv420p")
