"""End-to-end config scenarios: simulate edge-core forwarding CYBERWAVE_* env
vars into the UGV Beast driver container and assert the driver resolves the
right MQTT broker endpoint, TLS mode, topic prefix, SDK base URL and token.

These exercise the *exact* public helpers ``MQTTBridgeNode.__init__`` uses to
load edge-core config (see mqtt_bridge_node.py ~lines 219-258, 491, 500-506):

    edge_driver_env_from_environ  -> read forwarded env
    resolve_broker_settings       -> host/port/TLS precedence (SDK default host)
    apply_resolved_mqtt_to_environ-> sync resolved values back for the SDK
    resolve_mqtt_topic_prefix     -> environment -> topic namespace

so the matrix proves the driver works across production, dev/staging, local
docker-compose, self-hosted and bare-local deployments with different backends
and brokers -- without a running ROS/MQTT stack.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import pytest

from mqtt_bridge.edge_driver_env import (
    apply_resolved_mqtt_to_environ,
    edge_driver_env_from_environ,
    log_edge_driver_env,
    resolve_broker_settings,
    resolve_mqtt_topic_prefix,
)

# Committed params.yaml ships broker.host="" and broker.port=8883 (so env wins).
PARAMS_HOST = ""
PARAMS_PORT = 8883


@dataclass
class DriverBoot:
    """What the driver resolved from the env edge-core forwarded."""

    host: str
    host_source: str
    port: int
    port_source: str
    use_tls: bool
    topic_prefix: str
    base_url: str
    token: str
    sdk_env: dict[str, str]


def simulate_edge_core_boot(
    forwarded_env: Mapping[str, str],
    *,
    params_host: Any = PARAMS_HOST,
    params_port: Any = PARAMS_PORT,
) -> DriverBoot:
    """Reproduce the driver's edge-core config load for a given forwarded env."""
    env = edge_driver_env_from_environ(forwarded_env)
    settings = resolve_broker_settings(
        env, host_param=params_host, port_param=params_port
    )
    sdk_env: dict[str, str] = {}
    apply_resolved_mqtt_to_environ(
        settings.host,
        settings.port,
        api_key=env.api_key,
        use_tls=settings.use_tls,
        target=sdk_env,
    )
    return DriverBoot(
        host=settings.host,
        host_source=settings.host_source,
        port=settings.port,
        port_source=settings.port_source,
        use_tls=settings.use_tls,
        topic_prefix=resolve_mqtt_topic_prefix(environ=forwarded_env),
        base_url=env.base_url,
        token=env.api_key,
        sdk_env=sdk_env,
    )


@pytest.fixture(autouse=True)
def _isolate_process_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Strip real CYBERWAVE_*/ZENOH_* so no host env leaks into resolution."""
    import os

    for key in list(os.environ):
        if key.startswith(("CYBERWAVE_", "ZENOH_")):
            monkeypatch.delenv(key, raising=False)


API_KEY = "cw_scenario_key_1234567890"
TWIN = "7f8dc0a7-414c-43ef-a533-e84c6533e02a"


# (id, forwarded_env, expected host/port/tls/prefix/base_url)
SCENARIOS: list[tuple[str, dict[str, str], dict[str, Any]]] = [
    (
        "production_cloud_tls",
        {
            "CYBERWAVE_BASE_URL": "https://api.cyberwave.com",
            "CYBERWAVE_MQTT_HOST": "mqtt.cyberwave.com",
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_MQTT_USE_TLS": "true",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "mqtt.cyberwave.com",
            "port": 8883,
            "use_tls": True,
            "topic_prefix": "",  # production => no prefix
            "base_url": "https://api.cyberwave.com",
        },
    ),
    (
        "production_tls_inferred_from_port",  # edge-core omitted USE_TLS
        {
            "CYBERWAVE_BASE_URL": "https://api.cyberwave.com",
            "CYBERWAVE_MQTT_HOST": "mqtt.cyberwave.com",
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "mqtt.cyberwave.com",
            "port": 8883,
            "use_tls": True,  # auto: 8883 => TLS
            "topic_prefix": "",
            "base_url": "https://api.cyberwave.com",
        },
    ),
    (
        "dev_staging_prefixed",
        {
            "CYBERWAVE_ENVIRONMENT": "dev",
            "CYBERWAVE_BASE_URL": "https://dev.api.cyberwave.com",
            "CYBERWAVE_MQTT_HOST": "dev.mqtt.cyberwave.com",
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_MQTT_USE_TLS": "true",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "dev.mqtt.cyberwave.com",
            "port": 8883,
            "use_tls": True,
            "topic_prefix": "dev",
            "base_url": "https://dev.api.cyberwave.com",
        },
    ),
    (
        "local_docker_compose_plain_mqtt",
        {
            "CYBERWAVE_ENVIRONMENT": "local",
            "CYBERWAVE_BASE_URL": "http://10.13.4.222:8000",
            "CYBERWAVE_MQTT_HOST": "10.13.4.222",
            "CYBERWAVE_MQTT_PORT": "1883",
            "CYBERWAVE_MQTT_USE_TLS": "false",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "10.13.4.222",
            "port": 1883,
            "use_tls": False,
            "topic_prefix": "local",
            "base_url": "http://10.13.4.222:8000",
        },
    ),
    (
        "explicit_localhost_dev",  # explicit local host must NOT fail loud
        {
            "CYBERWAVE_BASE_URL": "http://localhost:8000",
            "CYBERWAVE_MQTT_HOST": "localhost",
            "CYBERWAVE_MQTT_PORT": "1883",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "localhost",
            "port": 1883,
            "use_tls": False,  # 1883, no explicit flag => no TLS
            "topic_prefix": "",
            "base_url": "http://localhost:8000",
        },
    ),
    (
        "self_hosted_custom_broker_tls",
        {
            "CYBERWAVE_ENVIRONMENT": "acme-prod",
            "CYBERWAVE_BASE_URL": "https://cyberwave.acme.internal",
            "CYBERWAVE_MQTT_HOST": "broker.acme.internal",
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_MQTT_USE_TLS": "true",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "broker.acme.internal",
            "port": 8883,
            "use_tls": True,
            "topic_prefix": "acme-prod",
            "base_url": "https://cyberwave.acme.internal",
        },
    ),
    (
        "mqtt_broker_alias_var",  # host via CYBERWAVE_MQTT_BROKER alias
        {
            "CYBERWAVE_BASE_URL": "https://api.cyberwave.com",
            "CYBERWAVE_MQTT_BROKER": "alias.mqtt.example.com",
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "alias.mqtt.example.com",
            "port": 8883,
            "use_tls": True,
            "topic_prefix": "",
            "base_url": "https://api.cyberwave.com",
        },
    ),
    (
        "custom_topic_prefix_overrides_production",
        {
            "CYBERWAVE_ENVIRONMENT": "production",
            "CYBERWAVE_MQTT_TOPIC_PREFIX": "tenant-x",
            "CYBERWAVE_BASE_URL": "https://api.cyberwave.com",
            "CYBERWAVE_MQTT_HOST": "mqtt.cyberwave.com",
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        {
            "host": "mqtt.cyberwave.com",
            "port": 8883,
            "use_tls": True,
            "topic_prefix": "tenant-x",
            "base_url": "https://api.cyberwave.com",
        },
    ),
]


@pytest.mark.parametrize("name,env,expected", SCENARIOS, ids=[s[0] for s in SCENARIOS])
def test_driver_loads_edge_core_config_per_environment(
    name: str, env: dict[str, str], expected: dict[str, Any]
) -> None:
    boot = simulate_edge_core_boot(env)

    assert boot.host == expected["host"]
    assert boot.port == expected["port"]
    assert boot.use_tls is expected["use_tls"]
    assert boot.topic_prefix == expected["topic_prefix"]
    assert boot.base_url == expected["base_url"]
    assert boot.token == API_KEY

    # The resolved endpoint is synced back into the env the Cyberwave SDK reads.
    assert boot.sdk_env["CYBERWAVE_MQTT_HOST"] == expected["host"]
    assert boot.sdk_env["CYBERWAVE_MQTT_PORT"] == str(expected["port"])
    assert boot.sdk_env["CYBERWAVE_MQTT_USE_TLS"] == (
        "true" if expected["use_tls"] else "false"
    )
    assert boot.sdk_env["CYBERWAVE_API_KEY"] == API_KEY


def test_env_host_and_port_take_precedence_over_params_yaml() -> None:
    """CYBERWAVE_MQTT_* forwarded by edge-core overrides committed params.yaml."""
    boot = simulate_edge_core_boot(
        {
            "CYBERWAVE_MQTT_HOST": "override.mqtt.example.com",
            "CYBERWAVE_MQTT_PORT": "1883",
            "CYBERWAVE_MQTT_USE_TLS": "false",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        params_host="stale.params.example.com",
        params_port=8883,
    )
    assert boot.host == "override.mqtt.example.com"
    assert boot.host_source == "CYBERWAVE_MQTT_HOST"
    assert boot.port == 1883
    assert boot.port_source == "CYBERWAVE_MQTT_PORT"


def test_falls_back_to_params_yaml_host_when_env_host_unset() -> None:
    """When edge-core forwards no host, a params.yaml broker.host is honoured."""
    boot = simulate_edge_core_boot(
        {
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        params_host="broker.from-params.example.com",
        params_port=8883,
    )
    assert boot.host == "broker.from-params.example.com"
    assert boot.host_source == "broker.host parameter"


def test_missing_broker_host_defaults_to_sdk_broker_like_so101() -> None:
    """The exact production incident: edge-core forwards only API_KEY + TWIN_UUID
    (no MQTT host). Taking inspiration from the so101 node, the driver now falls
    back to the cyberwave SDK production broker (mqtt.cyberwave.com) instead of
    refusing to start — it never silently uses localhost."""
    boot = simulate_edge_core_boot(
        {
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        params_host="",  # committed params.yaml default
    )
    assert boot.host == "mqtt.cyberwave.com"
    assert boot.host_source == "cyberwave SDK default"
    assert boot.port == 8883
    assert boot.use_tls is True  # 8883 => TLS, matches the SDK
    assert boot.sdk_env["CYBERWAVE_MQTT_HOST"] == "mqtt.cyberwave.com"


def test_missing_host_and_port_defaults_fully_like_so101() -> None:
    """Nothing MQTT-related forwarded at all: host + port fall back to the SDK
    production defaults, so the driver still reaches the broker."""
    boot = simulate_edge_core_boot(
        {
            "CYBERWAVE_API_KEY": API_KEY,
            "CYBERWAVE_TWIN_UUID": TWIN,
        },
        params_host="",
        params_port=8883,  # committed params.yaml default
    )
    assert boot.host == "mqtt.cyberwave.com"
    assert boot.port == 8883
    assert boot.use_tls is True


class _CapturingLogger:
    def __init__(self) -> None:
        self.infos: list[str] = []
        self.warnings: list[str] = []

    def info(self, msg: str) -> None:
        self.infos.append(str(msg))

    def warning(self, msg: str) -> None:
        self.warnings.append(str(msg))

    def error(self, msg: str) -> None:  # pragma: no cover - defensive
        self.warnings.append(str(msg))

    def debug(self, msg: str) -> None:  # pragma: no cover - defensive
        pass


def test_operator_log_warns_on_missing_required_and_tls_mismatch() -> None:
    """log_edge_driver_env surfaces missing host + a TLS/port mismatch so an
    operator can diagnose a misconfigured edge-core forward from the logs."""
    env = edge_driver_env_from_environ(
        {
            "CYBERWAVE_MQTT_PORT": "8883",
            "CYBERWAVE_MQTT_USE_TLS": "false",  # TLS off on the TLS port
            # no CYBERWAVE_MQTT_HOST, no API key, no twin uuid
        }
    )
    logger = _CapturingLogger()
    log_edge_driver_env(logger, env)

    joined_warnings = "\n".join(logger.warnings)
    assert "CYBERWAVE_MQTT_HOST" in joined_warnings
    assert "CYBERWAVE_API_KEY" in joined_warnings
    assert "CYBERWAVE_TWIN_UUID" in joined_warnings
    assert "TLS port without TLS" in joined_warnings
