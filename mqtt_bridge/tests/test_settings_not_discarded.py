"""Anti-discard: every setting read into TwinConfig is consumed and logged."""

from __future__ import annotations

import re
from pathlib import Path

from mqtt_bridge.edge_driver_env import build_twin_config, log_twin_config

FIX = Path(__file__).resolve().parent / "fixtures" / "ugv_beast"
TWIN_UUID = "67bb907f-b339-495c-9d20-487c905c8a98"

# A fully-populated env covering every CYBERWAVE_* var the driver consumes (§0.4).
FULL_ENV = {
    "CYBERWAVE_TWIN_UUID": TWIN_UUID,
    "CYBERWAVE_TWIN_JSON_FILE": str(FIX / "twin.json"),
    "CYBERWAVE_API_KEY": "cw_key_abcdef123456",
    "CYBERWAVE_BASE_URL": "https://api.example.com",
    "CYBERWAVE_MQTT_HOST": "mqtt.example.com",
    "CYBERWAVE_MQTT_PORT": "8883",
    "CYBERWAVE_MQTT_USE_TLS": "true",
    "CYBERWAVE_ENVIRONMENT": "dev",
    "CYBERWAVE_ENVIRONMENT_UUID": "1167750b-35d6-4bf3-9a91-3bcd1f969f1a",
    "CYBERWAVE_TWIN_UUIDS": TWIN_UUID,
    "CYBERWAVE_CHILD_TWIN_UUIDS": "child-1,child-2",
    "CYBERWAVE_EDGE_LOG_LEVEL": "debug",
    "CYBERWAVE_WORKER_LOG_LEVEL": "info",
    "CYBERWAVE_DATA_BACKEND": "zenoh",
    "ZENOH_CONNECT": "tcp/router:7447",
    "ZENOH_SHARED_MEMORY": "true",
    "CYBERWAVE_METADATA_VIDEO_DEVICE": "/dev/video7",
    "CYBERWAVE_EDGE_CONFIG_DIR": str(FIX),
}


class _CapLogger:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def info(self, msg, *a) -> None:
        self.lines.append(str(msg) % a if a else str(msg))

    def warning(self, msg, *a) -> None:
        self.lines.append(str(msg) % a if a else str(msg))

    debug = error = info


def test_every_twin_config_field_populated() -> None:
    cfg = build_twin_config(FULL_ENV, config_dir=FIX)
    # Twin/env/sensor fields all resolved from the golden fixtures.
    assert cfg.twin_uuid == TWIN_UUID
    assert cfg.registry_id
    assert cfg.edge_fingerprint
    assert cfg.fingerprint
    assert cfg.environment_uuid
    assert cfg.environment_twin_uuids
    assert cfg.sensors and cfg.sensors_by_id
    assert cfg.camera.sensor is not None
    assert cfg.camera.frame_id
    assert cfg.camera.video_device
    assert cfg.cameras_config
    assert cfg.edge_record
    assert cfg.source_files
    # env fields
    assert cfg.env.api_key and cfg.env.base_url and cfg.env.mqtt_host
    assert cfg.env.metadata_video_device == "/dev/video7"
    assert cfg.env.child_twin_uuids == ["child-1", "child-2"]
    assert cfg.env.data_backend == "zenoh"


def test_log_surfaces_every_env_var_and_resolved_value() -> None:
    cfg = build_twin_config(FULL_ENV, config_dir=FIX)
    log = _CapLogger()
    log_twin_config(log, cfg)
    blob = "\n".join(log.lines)

    # Every §0.4 env var appears in the log (masked where secret).
    for var in [
        "CYBERWAVE_ENVIRONMENT", "CYBERWAVE_ENVIRONMENT_UUID", "CYBERWAVE_EDGE_LOG_LEVEL",
        "CYBERWAVE_WORKER_LOG_LEVEL", "CYBERWAVE_BASE_URL", "CYBERWAVE_MQTT_HOST",
        "CYBERWAVE_MQTT_PORT", "CYBERWAVE_API_KEY", "CYBERWAVE_TWIN_UUID",
        "CYBERWAVE_TWIN_JSON_FILE", "CYBERWAVE_TWIN_UUIDS", "CYBERWAVE_CHILD_TWIN_UUIDS",
        "CYBERWAVE_DATA_BACKEND", "ZENOH_CONNECT", "ZENOH_SHARED_MEMORY",
    ]:
        assert var in blob, f"{var} not surfaced in log"

    # Every resolved JSON value is surfaced too.
    for token in [
        "front_camera", "camera_link", "/dev/video7", "waveshare/ugv-beast",
        "raspberrypi-a4438ddfd350", "1167750b-35d6-4bf3-9a91-3bcd1f969f1a",
    ]:
        assert token in blob, f"{token} not surfaced in log"

    # The API key is masked, never printed in full.
    assert "cw_key_abcdef123456" not in blob


def test_node_stores_twin_config_and_no_dead_twin_json_local() -> None:
    src = (Path(__file__).resolve().parents[1] / "mqtt_bridge_node.py").read_text()
    assert "self._twin_config = load_twin_config()" in src
    assert "log_twin_config(" in src
    # The old discard-after-log local must be gone.
    assert not re.search(r"^\s*twin_json = self\._edge_env\.load_twin_json\(\)", src, re.M)
