"""TwinConfig reader unit tests — resolver vs real golden fixtures + synthetic cases."""

from __future__ import annotations

from pathlib import Path

import pytest

from mqtt_bridge import edge_driver_env
from mqtt_bridge.edge_driver_env import (
    EdgeDriverEnv,
    build_twin_config,
    build_twin_config_via_sdk,
    extract_sensors,
    normalize_sensor_type,
    read_json_file,
    resolve_camera_selection,
    resolve_cameras_config,
    resolve_config_dir,
    resolve_video_device_from_cameras,
)

FIX = Path(__file__).resolve().parent / "fixtures" / "ugv_beast"
TWIN_UUID = "67bb907f-b339-495c-9d20-487c905c8a98"


def _golden_env(**overrides: str) -> dict[str, str]:
    env = {
        "CYBERWAVE_TWIN_UUID": TWIN_UUID,
        "CYBERWAVE_TWIN_JSON_FILE": str(FIX / "twin.json"),
    }
    env.update(overrides)
    return env


def _golden_cfg(**overrides: str):
    return build_twin_config(_golden_env(**overrides), config_dir=FIX)


# Golden test — real production shapes
def test_golden_snapshot_matches_real_deployment() -> None:
    cfg = _golden_cfg()

    assert cfg.twin_uuid == TWIN_UUID
    assert cfg.registry_id == "waveshare/ugv-beast"
    assert cfg.edge_fingerprint == "raspberrypi-a4438ddfd350"
    assert cfg.fingerprint == "raspberrypi-a4438ddfd350"
    assert cfg.environment_uuid == "1167750b-35d6-4bf3-9a91-3bcd1f969f1a"
    assert cfg.environment_twin_uuids == (TWIN_UUID,)

    # Sensor identity from JSON — front_camera / rgb / camera_link.
    assert cfg.camera.sensor is not None
    assert cfg.camera.sensor.id == "front_camera"
    assert cfg.camera.sensor.type == "rgb"
    assert cfg.camera.sensor.parent_link == "camera_link"
    # frame_id comes from the twin's parent_link (JSON), NOT the YAML.
    assert cfg.camera.frame_id == "camera_link"
    # Device resolved from the cameras block (index 0 -> primary_path).
    assert cfg.camera.video_device == "/dev/video0"
    assert cfg.camera.video_device_source == "cameras_config"

    # edge_configs / sensors_devices are absent on this twin.
    assert cfg.edge_configs == {}
    assert cfg.sensors_devices == {}

    # Best-effort credentials.json fallback populated the connection env.
    assert cfg.env.base_url == "https://api-dev.cyberwave.com"
    assert cfg.env.mqtt_host == "dev.mqtt.cyberwave.com"


def test_golden_source_files_all_ok() -> None:
    cfg = _golden_cfg()
    for name in ("twin.json", "edge.json", "environment.json", "cameras.json",
                 "fingerprint.json", "credentials.json"):
        assert cfg.source_files[name] == "ok", name


def test_process_env_wins_over_credentials_but_credentials_fill_gaps() -> None:
    # credentials.json has base_url=api-dev; process env overrides it.
    cfg = _golden_cfg(CYBERWAVE_BASE_URL="https://override.example.com")
    assert cfg.env.base_url == "https://override.example.com"
    # mqtt_host not in process env -> filled from credentials.json.
    assert cfg.env.mqtt_host == "dev.mqtt.cyberwave.com"


# Sensor extraction / precedence / normalization
@pytest.mark.parametrize(
    "raw_type,expected",
    [
        ("rgb", "rgb"),
        ("camera", "rgb"),
        ("rgb_camera", "rgb"),
        ("rgbd", "rgb"),
        ("color", "rgb"),
        ("depth", "depth"),
        ("depth_camera", "depth"),
        ("lidar", "lidar_3d"),
        ("imu", "imu"),
        (None, ""),
    ],
)
def test_normalize_sensor_type(raw_type, expected) -> None:
    assert normalize_sensor_type(raw_type) == expected


def test_sensor_precedence_capabilities_top_level_wins() -> None:
    twin = {
        "capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]},
        "universal_schema": {"sensors": [{"name": "front_camera", "type": "camera", "parent_link": "camera_link"}]},
    }
    sensors = extract_sensors(twin, {})
    assert [s.id for s in sensors] == ["front_camera"]
    assert sensors[0].type == "rgb"  # normalized-shape source wins (already rgb)
    assert sensors[0].parent_link == "camera_link"


def test_sensor_precedence_raw_universal_schema_only() -> None:
    # No normalized capabilities anywhere -> fall back to raw universal_schema.
    twin = {
        "universal_schema": {
            "sensors": [{"name": "front_camera", "type": "camera", "parent_link": "camera_link"}]
        }
    }
    sensors = extract_sensors(twin, {})
    assert len(sensors) == 1
    assert sensors[0].id == "front_camera"  # id defaults to name
    assert sensors[0].type == "rgb"  # camera -> rgb
    assert sensors[0].parent_link == "camera_link"


def test_sensor_precedence_production_capabilities_fallback() -> None:
    twin = {
        "metadata": {
            "_production_capabilities": {
                "sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]
            }
        }
    }
    sensors = extract_sensors(twin, {})
    assert [s.id for s in sensors] == ["front_camera"]


def test_sensor_id_falls_back_to_parameters_id() -> None:
    twin = {"capabilities": {"sensors": [{"name": "wrist", "type": "camera", "parameters": {"id": "wrist_cam"}}]}}
    sensors = extract_sensors(twin, {})
    assert sensors[0].id == "wrist_cam"


def test_depth_and_rgb_pick_first_rgb() -> None:
    twin = {
        "capabilities": {
            "sensors": [
                {"id": "depth0", "type": "depth", "parent_link": "d_link"},
                {"id": "front_camera", "type": "rgb", "parent_link": "camera_link"},
            ]
        }
    }
    cfg = build_twin_config(
        {"CYBERWAVE_TWIN_UUID": TWIN_UUID, "CYBERWAVE_TWIN_JSON_FILE": "/nonexistent.json"},
        config_dir=FIX,
    )
    sensors = extract_sensors(twin, {})
    sel = resolve_camera_selection(
        sensors,
        env=cfg.env,
        cameras_config={},
        sensors_devices={},
        edge_configs={},
        twin_uuid=TWIN_UUID,
    )
    assert sel.sensor.id == "front_camera"
    assert sel.frame_id == "camera_link"


# Config-dir + file resilience
def test_resolve_config_dir_env_override(tmp_path) -> None:
    assert resolve_config_dir({"CYBERWAVE_EDGE_CONFIG_DIR": str(tmp_path)}) == tmp_path


def test_resolve_config_dir_twin_sibling(tmp_path) -> None:
    (tmp_path / "edge.json").write_text("{}")
    twin = tmp_path / f"{TWIN_UUID}.json"
    twin.write_text("{}")
    got = resolve_config_dir({"CYBERWAVE_TWIN_JSON_FILE": str(twin)})
    assert got == tmp_path


def test_read_json_file_missing(tmp_path) -> None:
    data, status = read_json_file(tmp_path / "nope.json")
    assert data is None and status == "missing"


def test_read_json_file_corrupt(tmp_path) -> None:
    p = tmp_path / "bad.json"
    p.write_text("{not valid json")
    data, status = read_json_file(p)
    assert data is None and status == "unreadable"


def test_missing_twin_json_degrades_gracefully() -> None:
    cfg = build_twin_config(
        {"CYBERWAVE_TWIN_UUID": TWIN_UUID, "CYBERWAVE_TWIN_JSON_FILE": "/does/not/exist.json"},
        config_dir=FIX,
    )
    assert cfg.source_files["twin.json"] == "missing"
    assert cfg.camera.sensor is None  # no twin -> no sensor
    # sibling files still read.
    assert cfg.source_files["edge.json"] == "ok"


# Video-device resolution
def test_video_device_from_cameras_primary_path() -> None:
    cameras = {
        "devices": [{"index": 0, "primary_path": "/dev/video0", "paths": ["/dev/video0", "/dev/video1"]}],
        "twin_to_device": {TWIN_UUID: 0},
        "selected_device": 0,
    }
    assert resolve_video_device_from_cameras(cameras, TWIN_UUID) == "/dev/video0"


def test_video_device_selected_device_fallback() -> None:
    cameras = {
        "devices": [{"index": 2, "primary_path": "/dev/video2"}],
        "twin_to_device": {},
        "selected_device": 2,
    }
    assert resolve_video_device_from_cameras(cameras, TWIN_UUID) == "/dev/video2"


def test_video_device_naive_fallback_without_devices() -> None:
    cameras = {"twin_to_device": {TWIN_UUID: 3}}
    assert resolve_video_device_from_cameras(cameras, TWIN_UUID) == "/dev/video3"


def test_metadata_video_device_env_wins() -> None:
    cfg = _golden_cfg(CYBERWAVE_METADATA_VIDEO_DEVICE="/dev/video1")
    assert cfg.camera.video_device == "/dev/video1"
    assert cfg.camera.video_device_source == "CYBERWAVE_METADATA_VIDEO_DEVICE"


def test_edge_json_cameras_supersedes_cameras_json() -> None:
    edge = {"metadata": {"cameras": {"selected_device": 1, "devices": [{"index": 1, "primary_path": "/dev/video1"}]}}}
    cams_json = {"selected_device": 0, "devices": [{"index": 0, "primary_path": "/dev/video0"}]}
    resolved = resolve_cameras_config(edge, cams_json)
    assert resolved["selected_device"] == 1


# SDK fallback (req 1/4: "from the json file OR using the cyberwave sdk")
def test_sdk_fallback_used_when_json_missing(monkeypatch) -> None:
    # No twin file, but SDK creds present -> build_twin_config fetches via SDK.
    twin_from_sdk = {
        "uuid": TWIN_UUID,
        "asset_uuid": "asset-1",
        "asset": {"registry_id": "waveshare/ugv-beast"},
        "capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]},
    }

    def fake_via_sdk(env):
        assert env.api_key == "cw_key" and env.twin_uuid == TWIN_UUID
        return twin_from_sdk

    monkeypatch.setattr(edge_driver_env, "build_twin_config_via_sdk", fake_via_sdk)
    cfg = build_twin_config(
        {"CYBERWAVE_TWIN_UUID": TWIN_UUID, "CYBERWAVE_API_KEY": "cw_key",
         "CYBERWAVE_TWIN_JSON_FILE": "/does/not/exist.json"},
        config_dir=FIX,
    )
    assert cfg.source_files["twin.json"] == "sdk"
    assert cfg.camera.sensor.id == "front_camera"
    assert cfg.registry_id == "waveshare/ugv-beast"


def test_sdk_fallback_returns_none_without_credentials() -> None:
    # No api_key/twin_uuid -> SDK path is not attempted (returns None).
    assert build_twin_config_via_sdk(EdgeDriverEnv()) is None


def test_json_file_wins_over_sdk(monkeypatch) -> None:
    # When the JSON file is present, the SDK fallback must NOT be consulted.
    called = {"n": 0}

    def fake_via_sdk(env):
        called["n"] += 1
        return {"uuid": "SDK-SHOULD-NOT-WIN"}

    monkeypatch.setattr(edge_driver_env, "build_twin_config_via_sdk", fake_via_sdk)
    cfg = _golden_cfg()
    assert called["n"] == 0
    assert cfg.twin_uuid == TWIN_UUID
    assert cfg.source_files["twin.json"] == "ok"


def test_sensors_devices_honored_when_present() -> None:
    twin = {"capabilities": {"sensors": [{"id": "front_camera", "type": "rgb", "parent_link": "camera_link"}]}}
    sensors = extract_sensors(twin, {})
    cfg = _golden_cfg()  # for cfg.env only
    sel = resolve_camera_selection(
        sensors,
        env=cfg.env,
        cameras_config={"selected_device": 0, "devices": [{"index": 0, "primary_path": "/dev/video0"}]},
        sensors_devices={"front_camera": "/dev/video2"},
        edge_configs={},
        twin_uuid=TWIN_UUID,
    )
    assert sel.video_device == "/dev/video2"
    assert sel.video_device_source == "metadata.sensors_devices"
