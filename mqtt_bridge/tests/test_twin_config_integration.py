"""Integration: all six edge-core JSON files compose (identity from JSON, hardware from YAML)."""

from __future__ import annotations

import shutil
from pathlib import Path

import yaml

from mqtt_bridge.edge_driver_env import build_twin_config

FIX = Path(__file__).resolve().parent / "fixtures" / "ugv_beast"
MAPPING = Path(__file__).resolve().parents[2] / "config" / "mappings" / "robot_ugv_beast_v1.yaml"
TWIN_UUID = "67bb907f-b339-495c-9d20-487c905c8a98"


def _copy_all(tmp: Path) -> None:
    for f in FIX.glob("*.json"):
        shutil.copy(f, tmp / f.name)


def test_all_six_files_compose(tmp_path) -> None:
    _copy_all(tmp_path)
    # In-container layout: CYBERWAVE_TWIN_JSON_FILE points at the twin file, config
    # dir holds the siblings.
    cfg = build_twin_config(
        {"CYBERWAVE_TWIN_UUID": TWIN_UUID, "CYBERWAVE_TWIN_JSON_FILE": str(tmp_path / "twin.json")},
        config_dir=tmp_path,
    )
    assert all(v == "ok" for v in cfg.source_files.values()), cfg.source_files
    assert cfg.twin_uuid == TWIN_UUID
    assert cfg.registry_id == "waveshare/ugv-beast"
    assert cfg.environment_uuid == "1167750b-35d6-4bf3-9a91-3bcd1f969f1a"
    assert cfg.camera.sensor.id == "front_camera"
    assert cfg.camera.frame_id == "camera_link"
    assert cfg.camera.video_device == "/dev/video0"
    # credentials.json.envs fallback populated connection env.
    assert cfg.env.base_url == "https://api-dev.cyberwave.com"


def test_identity_is_json_sourced_hardware_is_yaml_sourced(tmp_path) -> None:
    _copy_all(tmp_path)
    cfg = build_twin_config(
        {"CYBERWAVE_TWIN_UUID": TWIN_UUID, "CYBERWAVE_TWIN_JSON_FILE": str(tmp_path / "twin.json")},
        config_dir=tmp_path,
    )
    mapping = yaml.safe_load(MAPPING.read_text())
    camera_yaml = mapping["camera"]

    # Identity + frame come from JSON, and are NOT in the YAML anymore.
    assert "camera_name" not in camera_yaml
    assert "frame_id" not in camera_yaml
    assert cfg.camera.sensor.id == "front_camera"
    assert cfg.camera.frame_id == "camera_link"

    # Hardware capture stays YAML-sourced.
    assert camera_yaml["image_topic"] == "/image_raw"
    assert camera_yaml["pixel_format"] == "mjpeg2rgb"
    assert camera_yaml["image_width"] == 1280
    assert camera_yaml["image_height"] == 720
    assert "supported_formats" in camera_yaml
    assert "adaptation" in camera_yaml


def test_environment_uuid_cross_checks_twin(tmp_path) -> None:
    _copy_all(tmp_path)
    cfg = build_twin_config(
        {"CYBERWAVE_TWIN_UUID": TWIN_UUID, "CYBERWAVE_TWIN_JSON_FILE": str(tmp_path / "twin.json")},
        config_dir=tmp_path,
    )
    # environment.json.uuid matches twin.environment_uuid (no drift warning case).
    assert cfg.environment_uuid == cfg.twin.get("environment_uuid")
    # fingerprint.json matches twin.metadata.edge_fingerprint.
    assert cfg.fingerprint == cfg.edge_fingerprint
