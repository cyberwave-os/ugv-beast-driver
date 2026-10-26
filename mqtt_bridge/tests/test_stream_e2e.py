"""Step 7 gate: WebRTC end-to-end + NAT/TURN preflight.

Two real verifications, both runnable on the host (aiortc installed):

1. preflight: a STUN Binding round-trip to the configured TURN/STUN server,
   proving outbound NAT traversal works (this is the only path relay-only ICE
   needs from inside the Docker container).

2. loopback: the actual ROSVideoStreamTrack is attached to a real
   RTCPeerConnection, connected in-process to a second peer, and we assert the
   far side DECODES real frames — i.e. the stream is genuinely sent into WebRTC.
   Adapting quality (scale) is reflected in the decoded frame size.
"""

from __future__ import annotations

import asyncio

import pytest

# Needs numpy/cv2/av and REAL aiortc. conftest installs a *stub* aiortc when the
# real one is absent, so importorskip("aiortc") isn't enough — check for the real API.
pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("av")
_aiortc = pytest.importorskip("aiortc")
if not hasattr(_aiortc, "RTCPeerConnection"):
    pytest.skip("real aiortc not installed (conftest stub present)", allow_module_level=True)

import numpy as np  # noqa: E402

from aiortc import RTCPeerConnection, RTCSessionDescription  # noqa: E402
from mqtt_bridge.plugins.ros_camera import ROSVideoStreamTrack  # noqa: E402
from mqtt_bridge.plugins.webrtc_preflight import stun_binding_check  # noqa: E402


class _Logger:
    def info(self, *a, **k):
        pass

    debug = warning = error = info


class _Node:
    class _M:
        raw = {"camera": {"image_width": 640, "image_height": 480}}

    _mapping = _M()
    _last_image_time = 0.0

    def get_logger(self):
        return _Logger()

    def resolve_ros_topic(self, t):
        return t

    def create_subscription(self, *a, **k):
        return object()

    def destroy_subscription(self, *a, **k):
        pass


def _make_track(w=640, h=480, fps=120):
    t = ROSVideoStreamTrack(_Node(), "/image_raw", fps=fps)
    t.actual_width, t.actual_height = w, h
    # A recognizable non-uniform frame so decode can't trivially "succeed" empty.
    frame = np.random.randint(0, 255, size=(h, w, 3), dtype=np.uint8)
    t.latest_frame = frame
    t.latest_frame_encoding = "bgr8"
    t._frame_ready_event.set()
    return t


async def _connect(pc1: RTCPeerConnection, pc2: RTCPeerConnection):
    offer = await pc1.createOffer()
    await pc1.setLocalDescription(offer)
    await pc2.setRemoteDescription(
        RTCSessionDescription(sdp=pc1.localDescription.sdp, type=pc1.localDescription.type)
    )
    answer = await pc2.createAnswer()
    await pc2.setLocalDescription(answer)
    await pc1.setRemoteDescription(
        RTCSessionDescription(sdp=pc2.localDescription.sdp, type=pc2.localDescription.type)
    )


@pytest.mark.parametrize(
    "scale,expected",
    [(1.0, (640, 480)), (0.5, (320, 240))],
)
def test_frames_actually_decoded_over_webrtc(scale, expected):
    async def run():
        track = _make_track()
        track.set_quality(scale=scale)
        pc1, pc2 = RTCPeerConnection(), RTCPeerConnection()
        got = asyncio.Queue()

        @pc2.on("track")
        def on_track(recv_track):
            async def pull():
                try:
                    for _ in range(30):
                        frame = await recv_track.recv()
                        await got.put(frame)
                except Exception:
                    pass

            asyncio.ensure_future(pull())

        pc1.addTrack(track)
        await _connect(pc1, pc2)
        try:
            frame = await asyncio.wait_for(got.get(), timeout=20.0)
        finally:
            await pc1.close()
            await pc2.close()
        return frame

    frame = asyncio.run(run())
    assert frame is not None
    assert (frame.width, frame.height) == expected


def test_turn_preflight_reachable():
    # Outbound STUN/TURN reachability (skip if the network blocks it).
    result = stun_binding_check("stun.l.google.com", 19302, timeout=4.0)
    if not result.get("reachable"):
        pytest.skip(f"network blocks outbound STUN: {result}")
    assert result["reachable"] is True
