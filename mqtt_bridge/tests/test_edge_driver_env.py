"""Tests for edge_driver_env."""

from pathlib import Path

from mqtt_bridge.edge_driver_env import (
    DEFAULT_MQTT_HOST,
    EdgeDriverEnv,
    apply_resolved_mqtt_to_environ,
    edge_driver_env_from_environ,
    has_turn_server,
    resolve_broker_host,
    resolve_broker_port,
    resolve_ice_servers,
    resolve_mqtt_topic_prefix,
)


def test_edge_driver_env_from_environ_reads_all_forwarded_vars():
    env = edge_driver_env_from_environ(
        {
            "CYBERWAVE_ENVIRONMENT": "local",
            "CYBERWAVE_EDGE_LOG_LEVEL": "debug",
            "CYBERWAVE_BASE_URL": "http://10.13.4.222:8000",
            "CYBERWAVE_MQTT_HOST": "10.13.4.222",
            "CYBERWAVE_MQTT_PORT": "1883",
            "CYBERWAVE_MQTT_USE_TLS": "false",
            "CYBERWAVE_API_KEY": "cw_test_key_123456",
            "CYBERWAVE_TWIN_UUID": "twin-a",
            "CYBERWAVE_TWIN_JSON_FILE": "/app/twin.json",
            "CYBERWAVE_CHILD_TWIN_UUIDS": "cam-1, cam-2",
        }
    )

    assert env.environment == "local"
    assert env.edge_log_level == "debug"
    assert env.debug_logs_enabled is True
    assert env.base_url == "http://10.13.4.222:8000"
    assert env.mqtt_host == "10.13.4.222"
    assert env.mqtt_port == "1883"
    assert env.mqtt_port_int == 1883
    assert env.mqtt_use_tls is False
    assert env.api_key == "cw_test_key_123456"
    assert env.twin_uuid == "twin-a"
    assert env.twin_json_file == "/app/twin.json"
    assert env.child_twin_uuids == ["cam-1", "cam-2"]


def test_resolve_broker_port_env_overrides_params_yaml():
    env = EdgeDriverEnv(mqtt_port="1883")
    port, source = resolve_broker_port(8883, env)
    assert port == 1883
    assert source == "CYBERWAVE_MQTT_PORT"


def test_resolve_broker_port_uses_params_when_env_unset():
    env = EdgeDriverEnv()
    port, source = resolve_broker_port(8883, env)
    assert port == 8883
    assert source == "broker.port parameter"


def test_edge_driver_env_api_key_falls_back_to_token():
    env = edge_driver_env_from_environ({"CYBERWAVE_TOKEN": "legacy-token"})
    assert env.api_key == "legacy-token"


def test_resolve_broker_host_env_wins_over_params_yaml():
    env = EdgeDriverEnv(mqtt_host="dev.mqtt.cyberwave.com")
    host, source = resolve_broker_host("mqtt.cyberwave.com", env)
    assert host == "dev.mqtt.cyberwave.com"
    assert source == "CYBERWAVE_MQTT_HOST"


def test_resolve_broker_host_uses_params_then_sdk_default():
    env = EdgeDriverEnv()
    host, source = resolve_broker_host("broker.example.com", env)
    assert host == "broker.example.com"
    assert source == "broker.host parameter"

    # so101-inspired: nothing configured → the cyberwave SDK production default
    # (mqtt.cyberwave.com), never localhost. Lets the driver connect when
    # edge-core forwards only API_KEY + TWIN_UUID.
    host, source = resolve_broker_host("", env)
    assert host == DEFAULT_MQTT_HOST == "mqtt.cyberwave.com"
    assert source == "cyberwave SDK default"

    # An explicit localhost (local dev) is still honored.
    host, source = resolve_broker_host("", EdgeDriverEnv(mqtt_host="localhost"))
    assert host == "localhost"
    assert source == "CYBERWAVE_MQTT_HOST"


def test_apply_resolved_mqtt_to_environ_syncs_sdk_env():
    environ: dict[str, str] = {}
    apply_resolved_mqtt_to_environ(
        "dev.mqtt.cyberwave.com",
        8883,
        api_key="cw_test",
        use_tls=True,
        target=environ,
    )

    assert environ["CYBERWAVE_MQTT_HOST"] == "dev.mqtt.cyberwave.com"
    assert environ["CYBERWAVE_MQTT_PORT"] == "8883"
    assert environ["CYBERWAVE_MQTT_USE_TLS"] == "true"
    assert environ["CYBERWAVE_API_KEY"] == "cw_test"


def test_resolve_mqtt_topic_prefix_from_environment():
    assert resolve_mqtt_topic_prefix(environment="dev") == "dev"


def test_resolve_mqtt_topic_prefix_production_is_empty():
    assert resolve_mqtt_topic_prefix(environment="production") == ""


def test_resolve_mqtt_topic_prefix_explicit_overrides_environment():
    assert (
        resolve_mqtt_topic_prefix(mqtt_topic_prefix="custom", environment="dev")
        == "custom"
    )


def test_resolve_mqtt_topic_prefix_reads_from_environ_mapping():
    assert (
        resolve_mqtt_topic_prefix(
            environ={
                "CYBERWAVE_ENVIRONMENT": "staging",
                "CYBERWAVE_MQTT_TOPIC_PREFIX": "override",
            }
        )
        == "override"
    )


def test_committed_params_yaml_leaves_broker_host_empty() -> None:
    """broker.host stays empty in the committed config so CYBERWAVE_MQTT_HOST
    (or the SDK default) wins — nothing local/hardcoded baked into the image."""
    params = (
        Path(__file__).resolve().parents[2] / "config" / "params.yaml"
    ).read_text()
    for line in params.splitlines():
        stripped = line.strip()
        if stripped.startswith("host:"):
            value = stripped.split(":", 1)[1].strip().strip("'\"")
            assert not value, (
                f"params.yaml commits a broker host {value!r} — leave it empty "
                f"so CYBERWAVE_MQTT_HOST / the SDK default wins"
            )


def test_resolve_ice_servers_defaults_to_sdk_relay() -> None:
    """Nothing set → None, i.e. defer to the SDK's DEFAULT_TURN_SERVERS relay (like
    the so101/camera-driver nodes) so the cloud/WAN path connects. No creds baked here."""
    assert resolve_ice_servers({}) is None


def test_resolve_ice_servers_reads_env_stun_and_turn() -> None:
    """STUN + TURN (with creds) come straight from env when provided."""
    servers = resolve_ice_servers(
        {
            "CYBERWAVE_WEBRTC_STUN_URL": "stun:edge.example:3478",
            "CYBERWAVE_WEBRTC_TURN_URL": "turn:edge.example:3478",
            "CYBERWAVE_WEBRTC_TURN_USERNAME": "u",
            "CYBERWAVE_WEBRTC_TURN_CREDENTIAL": "c",
        }
    )
    assert servers[0] == {"urls": ["stun:edge.example:3478"]}
    assert servers[1] == {
        "urls": ["turn:edge.example:3478"],
        "username": "u",
        "credential": "c",
    }
    assert has_turn_server(servers)


def test_resolve_ice_servers_env_override_never_bakes_cyberwave_credentials() -> None:
    """When env provides ICE servers, this image adds no cyberwave-* creds of its own
    (the default relay lives in the SDK, not here). A bare STUN override stays None-free."""
    import json

    env_cases = (
        {"CYBERWAVE_WEBRTC_STUN_URL": "stun:x:3478"},
        {
            "CYBERWAVE_WEBRTC_STUN_URL": "stun:x:3478",
            "CYBERWAVE_WEBRTC_TURN_URL": "turn:x:3478",
            "CYBERWAVE_WEBRTC_TURN_USERNAME": "u",
            "CYBERWAVE_WEBRTC_TURN_CREDENTIAL": "c",
        },
    )
    for env in env_cases:
        blob = json.dumps(resolve_ice_servers(env))
        assert "cyberwave-user" not in blob
        assert "cyberwave-admin" not in blob
        assert "turn.cyberwave.com" not in blob

    # Nothing configured → None (defer to SDK default), so nothing is baked here either.
    assert resolve_ice_servers({}) is None
