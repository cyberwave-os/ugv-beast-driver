"""Strict actuation: sensors, edge_configs, sensors_devices, camera device selection."""

from __future__ import annotations

import json
from pathlib import Path

from mqtt_bridge.edge_driver_env import build_twin_config

TWIN_UUID = "67bb907f-b339-495c-9d20-487c905c8a98"


def _write(tmp: Path, twin: dict, *, edge: dict | None = None, cameras: dict | None = None) -> Path:
    (tmp / "twin.json").write_text(json.dumps(twin))
    if edge is not None:
        (tmp / "edge.json").write_text(json.dumps(edge))
    if cameras is not None:
        (tmp / "cameras.json").write_text(json.dumps(cameras))
    return tmp / "twin.json"


def _cfg(tmp: Path, twin_path: Path, **env):
    e = {"CYBERWAVE_TWIN_UUID": TWIN_UUID, "CYBERWAVE_TWIN_JSON_FILE": str(twin_path)}
    e.update(env)
    return build_twin_config(e, config_dir=tmp)


def test_first_rgb_sensor_chosen_over_depth(tmp_path) -> None:
    twin = {
        "uuid": TWIN_UUID,
        "capabilities": {
            "sensors": [
                {"id": "depth0", "type": "depth", "parent_link": "d_link"},
                {"id": "front_camera", "type": "rgb", "parent_link": "camera_link"},
            ]
        },
    }
    p = _write(tmp_path, twin)
    cfg = _cfg(tmp_path, p)
    assert cfg.camera.sensor.id == "front_camera"     # WebRTC sensor
    assert cfg.camera.frame_id == "camera_link"       # usb_cam frame_id
    # both sensors indexed
    assert set(cfg.sensors_by_id) == {"depth0", "front_camera"}


def test_edge_configs_present_surfaced(tmp_path) -> None:
    twin = {
        "uuid": TWIN_UUID,
        "metadata": {
            "edge_configs": {"camera_config": {"fps": 20, "resolution": "1280x720", "camera_id": "cam0"}}
        },
        "capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]},
    }
    cfg = _cfg(tmp_path, _write(tmp_path, twin))
    assert cfg.edge_configs.get("camera_config", {}).get("fps") == 20
    assert cfg.camera.edge_config.get("resolution") == "1280x720"


def test_edge_configs_absent_defaults_empty(tmp_path) -> None:
    twin = {"uuid": TWIN_UUID, "capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]}}
    cfg = _cfg(tmp_path, _write(tmp_path, twin))
    assert cfg.edge_configs == {}
    assert cfg.camera.edge_config == {}


def test_sensors_devices_present_drives_video_device(tmp_path) -> None:
    twin = {
        "uuid": TWIN_UUID,
        "metadata": {"sensors_devices": {"front_camera": "/dev/video2"}},
        "capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]},
    }
    cfg = _cfg(tmp_path, _write(tmp_path, twin))
    assert cfg.sensors_devices == {"front_camera": "/dev/video2"}
    assert cfg.camera.video_device == "/dev/video2"
    assert cfg.camera.video_device_source == "metadata.sensors_devices"


def test_sensors_devices_absent_falls_through_to_cameras(tmp_path) -> None:
    twin = {"uuid": TWIN_UUID, "capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]}}
    cameras = {"devices": [{"index": 0, "primary_path": "/dev/video0"}], "twin_to_device": {TWIN_UUID: 0}, "selected_device": 0}
    cfg = _cfg(tmp_path, _write(tmp_path, twin, cameras=cameras))
    assert cfg.sensors_devices == {}
    assert cfg.camera.video_device == "/dev/video0"
    assert cfg.camera.video_device_source == "cameras_config"


def test_metadata_video_device_env_wins_over_all(tmp_path) -> None:
    twin = {
        "uuid": TWIN_UUID,
        "metadata": {"sensors_devices": {"front_camera": "/dev/video2"}},
        "capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]},
    }
    cameras = {"devices": [{"index": 0, "primary_path": "/dev/video0"}], "selected_device": 0}
    cfg = _cfg(tmp_path, _write(tmp_path, twin, cameras=cameras),
               CYBERWAVE_METADATA_VIDEO_DEVICE="/dev/video1")
    assert cfg.camera.video_device == "/dev/video1"
    assert cfg.camera.video_device_source == "CYBERWAVE_METADATA_VIDEO_DEVICE"
