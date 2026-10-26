"""M2 §7.2 — usb_cam identity (camera_name/frame_id/device) comes from the twin."""

from __future__ import annotations

from mqtt_bridge.plugins.camera_device_manager import (
    URDF_RGB_CAMERA_LINK,
    CameraDeviceManager,
    static_tf_command,
)

# A realistic hardware-only camera block (post-M4: no camera_name/frame_id here).
HW_CFG = {
    "io_method": "mmap",
    "camera_info_url": "package://ugv_vision/config/camera_info.yaml",
    "supported_formats": {"mjpeg2rgb": {"sizes": [[1280, 720]], "max_fps": 30}},
}


def _mgr(**overrides) -> CameraDeviceManager:
    return CameraDeviceManager(
        dict(HW_CFG),
        device_resolver=lambda cfg: None,  # no real v4l2 in tests
        **overrides,
    )


def test_identity_from_twin_wins() -> None:
    mgr = _mgr(camera_name="front_camera", frame_id="camera_link", video_device="/dev/video0")
    cmd = mgr._build_command("mjpeg2rgb", 1280, 720, 30)
    assert "camera_name:=front_camera" in cmd
    assert "frame_id:=camera_link" in cmd  # twin parent_link, NOT pt_camera_link
    assert "video_device:=/dev/video0" in cmd
    # hardware params still present
    assert "io_method:=mmap" in cmd
    assert "pixel_format:=mjpeg2rgb" in cmd
    assert "image_width:=1280" in cmd


def test_twin_frame_id_overrides_legacy_yaml() -> None:
    cfg = dict(HW_CFG, camera_name="pt_camera", frame_id="pt_camera_link")
    mgr = CameraDeviceManager(
        cfg, device_resolver=lambda c: None,
        camera_name="front_camera", frame_id="camera_link", video_device="/dev/video0",
    )
    assert mgr.camera_name == "front_camera"
    assert mgr.frame_id == "camera_link"


def test_identity_never_read_from_yaml_cfg() -> None:
    # Even if a (stale) YAML cfg carried identity keys, they must be IGNORED — the
    # manager uses ROS-neutral defaults, never a YAML sensor id/frame.
    cfg = dict(HW_CFG, camera_name="pt_camera", frame_id="pt_camera_link")
    mgr = CameraDeviceManager(cfg, device_resolver=lambda c: "/dev/video9")
    assert mgr.camera_name == "camera"                 # usb_cam default, NOT "pt_camera"
    assert mgr.frame_id == URDF_RGB_CAMERA_LINK         # ROS URDF link, NOT from cfg
    assert mgr.camera_name != "pt_camera"
    assert mgr.video_device == "/dev/video9"


def test_explicit_video_device_skips_resolver() -> None:
    mgr = _mgr(video_device="/dev/video0")
    # resolver returns None, but explicit device is used verbatim.
    assert mgr.video_device == "/dev/video0"


def test_static_tf_command_bridges_urdf_to_twin_frame() -> None:
    cmd = static_tf_command(URDF_RGB_CAMERA_LINK, "camera_link", namespace="ugv_beast_67bb90")
    assert cmd[:4] == ["ros2", "run", "tf2_ros", "static_transform_publisher"]
    assert "--frame-id" in cmd and URDF_RGB_CAMERA_LINK in cmd  # parent = URDF link
    assert "--child-frame-id" in cmd and "camera_link" in cmd  # child = twin frame
    assert "__ns:=/ugv_beast_67bb90" in cmd
    assert URDF_RGB_CAMERA_LINK == "pt_camera_link"
