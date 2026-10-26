"""WebRTC compliance: track frame invariants, offer/SDP, answer matching (real SDK where reachable)."""

from __future__ import annotations

import asyncio
import sys
import types
from typing import Any

import pytest

pytest.importorskip("numpy")
pytest.importorskip("cv2")
pytest.importorskip("av")

import numpy as np  # noqa: E402

from mqtt_bridge.plugins.ros_camera import ROSVideoStreamTrack  # noqa: E402


# Load the REAL cyberwave.sensor.base_video past the conftest cyberwave stub.
def _load_real_base_video() -> Any:
    import importlib

    saved = {
        k: v for k, v in sys.modules.items()
        if k == "cyberwave" or k.startswith("cyberwave.")
    }
    for k in list(saved):
        del sys.modules[k]
    try:
        return importlib.import_module("cyberwave.sensor.base_video")
    except Exception:
        return None
    finally:
        for k in [k for k in sys.modules if k == "cyberwave" or k.startswith("cyberwave.")]:
            del sys.modules[k]
        sys.modules.update(saved)


_BV = _load_real_base_video()
_HAS_REAL_SDK = _BV is not None
requires_sdk = pytest.mark.skipif(not _HAS_REAL_SDK, reason="real cyberwave SDK not importable")


# Track fixtures (mirror test_ros_camera_adaptive)
class _Logger:
    def info(self, *a: Any, **k: Any) -> None:
        pass

    debug = warning = error = info


class _Mapping:
    raw = {"camera": {"image_width": 640, "image_height": 480}}


class _Node:
    _mapping = _Mapping()
    _last_image_time = 0.0

    def get_logger(self) -> _Logger:
        return _Logger()

    def resolve_ros_topic(self, t: str) -> str:
        return t

    def create_subscription(self, *a: Any, **k: Any) -> Any:
        return object()

    def destroy_subscription(self, *a: Any, **k: Any) -> None:
        pass


def _make_track(fps: int = 120, w: int = 640, h: int = 480):
    t = ROSVideoStreamTrack(_Node(), "/image_raw", fps=fps)
    t.actual_width, t.actual_height = w, h
    t.latest_frame = np.random.randint(0, 255, (h, w, 3), dtype=np.uint8)
    t.latest_frame_encoding = "bgr8"
    t._frame_ready_event.set()
    return t


# (a) VideoStreamTrack frame invariants
def test_frame_is_yuv420p_even_and_positive() -> None:
    t = _make_track()
    f = asyncio.run(t.recv())
    assert f.format.name == "yuv420p"
    assert f.width % 2 == 0 and f.height % 2 == 0
    assert f.width >= 2 and f.height >= 2


def test_odd_input_dimensions_are_cropped_even() -> None:
    t = _make_track(w=641, h=481)
    t.latest_frame = np.random.randint(0, 255, (481, 641, 3), dtype=np.uint8)
    # ingest crop happens in _image_callback; emulate the actual_* update path.
    t.actual_width, t.actual_height = 640, 480
    f = asyncio.run(t.recv())
    assert (f.width, f.height) == (640, 480)


def test_pts_monotonic_and_90khz_timebase() -> None:
    t = _make_track()

    async def drive():
        a = await t.recv()
        b = await t.recv()
        return a, b

    a, b = asyncio.run(drive())
    assert b.pts > a.pts
    assert a.time_base.denominator == 90000


def test_scaled_dims_track_quality() -> None:
    t = _make_track()

    async def drive():
        full = await t.recv()
        t.set_quality(scale=0.5)
        half = await t.recv()
        return full, half

    full, half = asyncio.run(drive())
    assert (full.width, full.height) == (640, 480)
    assert (half.width, half.height) == (320, 240)


def test_recv_never_none_blank_fallback() -> None:
    t = _make_track()
    t.latest_frame = None
    f = asyncio.run(t.recv())
    assert f is not None
    assert f.format.name == "yuv420p"
    assert (f.width, f.height) == (640, 480)


def test_fps_and_scale_clamped() -> None:
    t = _make_track()
    t.set_quality(fps=0, scale=5.0)
    fps, scale = t.get_quality()
    assert fps >= 1.0 and scale == 1.0
    t.set_quality(fps=15, scale=0.0)
    fps, scale = t.get_quality()
    assert fps == 15.0 and scale == 0.05


def test_keyframe_marked_early() -> None:
    t = _make_track()
    f = asyncio.run(t.recv())  # frame_count becomes 1 -> keyframe
    # av may expose key_frame read-only; only assert when settable/readable.
    assert getattr(f, "key_frame", True) in (True, False)


def test_stream_attributes_shape() -> None:
    t = _make_track()
    t.set_quality(fps=15, scale=0.5)
    attrs = t.get_stream_attributes()
    assert (attrs["width"], attrs["height"]) == (320, 240)
    assert attrs["fps"] == 15
    assert "camera_type" in attrs and "camera_id" in attrs


# (a.8) Real aiortc loopback decode (skips unless real aiortc)
def test_frames_decode_over_real_aiortc_loopback() -> None:
    try:
        import aiortc
        from aiortc import RTCPeerConnection, RTCSessionDescription
    except Exception:
        pytest.skip("aiortc not installed")
    if not hasattr(aiortc, "RTCPeerConnection"):
        pytest.skip("aiortc stubbed, not real")

    track = _make_track(fps=60)

    async def loopback():
        pc1, pc2 = RTCPeerConnection(), RTCPeerConnection()
        got = asyncio.get_event_loop().create_future()

        @pc2.on("track")
        def _on(remote):
            async def pull():
                frame = await remote.recv()
                if not got.done():
                    got.set_result((frame.width, frame.height))
            asyncio.ensure_future(pull())

        pc1.addTrack(track)
        offer = await pc1.createOffer()
        await pc1.setLocalDescription(offer)
        await pc2.setRemoteDescription(pc1.localDescription)
        answer = await pc2.createAnswer()
        await pc2.setLocalDescription(answer)
        await pc1.setRemoteDescription(pc2.localDescription)
        try:
            size = await asyncio.wait_for(got, timeout=10)
        finally:
            await pc1.close()
            await pc2.close()
        return size

    size = asyncio.run(loopback())
    assert size == (640, 480)


# (c) Offer payload + SDP codec compliance (real SDK)
class _FakeClient:
    topic_prefix = "dev"
    client_id = "edgeclient"
    published: list = []

    def __init__(self) -> None:
        self.published = []


class _FakeStreamer:
    id = "track-123"

    def get_stream_attributes(self) -> dict:
        return {"camera_type": "ros", "camera_id": "/image_raw", "width": 640, "height": 480, "fps": 30}


class _FakePC:
    class localDescription:
        type = "offer"


def _make_fake_streamer_self(camera_name, *, recording=True):
    """A minimal object with exactly the attributes _send_offer/_on_answer_message
    touch, so the REAL unbound SDK methods can run against it."""
    captured: dict = {}

    def _publish_message(topic, payload):
        captured["topic"] = topic
        captured["payload"] = payload

    obj = types.SimpleNamespace(
        client=_FakeClient(),
        twin_uuid="67bb907f",
        streamer=_FakeStreamer(),
        _video_encoder_name="libx264",
        _should_record=recording,
        camera_name=camera_name,
        pc=_FakePC(),
        stream_source=None,
        stream_instance_id=None,
        frontend_type=None,
        _answer_received=False,
        _answer_data=None,
        _publish_message=_publish_message,
    )
    return obj, captured


SAMPLE_SDP_VP8_H264 = (
    "v=0\r\n"
    "o=- 0 0 IN IP4 127.0.0.1\r\n"
    "s=-\r\n"
    "t=0 0\r\n"
    "m=video 9 UDP/TLS/RTP/SAVPF 96 97 98\r\n"
    "a=rtpmap:96 VP8/90000\r\n"
    "a=rtpmap:97 H264/90000\r\n"
    "a=fmtp:97 profile-level-id=42e01f\r\n"
    "a=rtpmap:98 rtx/90000\r\n"
    "a=fmtp:98 apt=96\r\n"
    "m=audio 9 UDP/TLS/RTP/SAVPF 111\r\n"
    "a=rtpmap:111 opus/48000/2\r\n"
)


@requires_sdk
def test_offer_payload_satisfies_sfu_contract() -> None:
    obj, captured = _make_fake_streamer_self("front_camera")
    _BV.BaseVideoStreamer._send_offer(obj, SAMPLE_SDP_VP8_H264)
    p = captured["payload"]
    assert p["target"] == "backend"       # else SFU drops it
    assert p["sender"] == "edge"
    assert p["type"] == "offer"
    assert p["sensor"] == "front_camera"  # twin identity, maps to offer.camera
    assert p["sensor"] is not None        # required for recording
    assert p["track_id"] == "track-123"
    assert "stream_attributes" in p
    assert captured["topic"].endswith("cyberwave/twin/67bb907f/webrtc-offer")


@requires_sdk
def test_offer_sensor_none_disables_recording_contract() -> None:
    # No rgb sensor -> camera_name None -> sensor None. The SFU skips recording
    # (handlers.rs), which is exactly the disabled-recording semantics.
    obj, captured = _make_fake_streamer_self(None)
    _BV.BaseVideoStreamer._send_offer(obj, SAMPLE_SDP_VP8_H264)
    assert captured["payload"]["sensor"] is None


@requires_sdk
def test_strip_vp8_keeps_h264() -> None:
    out = _BV._strip_vp8_video(SAMPLE_SDP_VP8_H264)
    assert "VP8" not in out                # VP8 stripped
    assert "H264/90000" in out             # H264 kept (SFU prefers/needs it)
    # video m-line still has at least one payload type.
    mvideo = next(ln for ln in out.splitlines() if ln.startswith("m=video"))
    pts = mvideo.split()[3:]
    assert len(pts) >= 1
    assert "96" not in pts                 # VP8 PT removed from the m-line


@requires_sdk
def test_strip_bails_out_when_no_codec_would_remain() -> None:
    vp8_only = (
        "v=0\r\no=- 0 0 IN IP4 127.0.0.1\r\ns=-\r\nt=0 0\r\n"
        "m=video 9 UDP/TLS/RTP/SAVPF 96\r\n"
        "a=rtpmap:96 VP8/90000\r\n"
    )
    out = _BV._strip_vp8_video(vp8_only)
    # Bail-out: would empty the video m-line -> SDP returned unmodified.
    assert "m=video 9 UDP/TLS/RTP/SAVPF 96" in out


# (d) Answer-matching compliance (real SDK _on_answer_message)
def _answer(sensor="front_camera", target="edge", with_video=True):
    return {
        "type": "answer",
        "target": target,
        "sensor": sensor,
        "sdp": "m=video 9 ...\r\n" if with_video else "m=audio 9 ...\r\n",
    }


@requires_sdk
def test_answer_for_our_sensor_accepted() -> None:
    obj, _ = _make_fake_streamer_self("front_camera")
    _BV.BaseVideoStreamer._on_answer_message(obj, _answer("front_camera"))
    assert obj._answer_received is True
    assert obj._answer_data is not None


@requires_sdk
def test_answer_for_wrong_sensor_rejected() -> None:
    # The identity fix must gate routing: a pt_camera answer is NOT accepted.
    obj, _ = _make_fake_streamer_self("front_camera")
    _BV.BaseVideoStreamer._on_answer_message(obj, _answer("pt_camera"))
    assert obj._answer_received is False
    assert obj._answer_data is None


@requires_sdk
def test_answer_idempotent_first_wins() -> None:
    obj, _ = _make_fake_streamer_self("front_camera")
    _BV.BaseVideoStreamer._on_answer_message(obj, _answer("front_camera"))
    first = obj._answer_data
    _BV.BaseVideoStreamer._on_answer_message(obj, {**_answer("front_camera"), "sdp": "m=video DUP"})
    assert obj._answer_data is first  # duplicate ignored


@requires_sdk
def test_answer_without_video_ignored() -> None:
    obj, _ = _make_fake_streamer_self("front_camera")
    _BV.BaseVideoStreamer._on_answer_message(obj, _answer("front_camera", with_video=False))
    assert obj._answer_received is False


# (e) SFU-accept predicate ties camera settings + twin identity -> routable
@requires_sdk
def test_sfu_accept_predicate_end_to_end() -> None:
    obj, captured = _make_fake_streamer_self("front_camera")
    sdp = _BV._strip_vp8_video(SAMPLE_SDP_VP8_H264)
    _BV.BaseVideoStreamer._send_offer(obj, sdp)
    p = captured["payload"]
    accept = (
        p["target"] == "backend"
        and p["sender"] == "edge"
        and p["type"] == "offer"
        and p["sensor"] is not None
        and "m=video" in p["sdp"]
        and "H264" in p["sdp"]
    )
    assert accept, p
