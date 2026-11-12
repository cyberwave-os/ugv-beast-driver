"""Edge-core driver container environment (CYBERWAVE_* vars)."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, MutableMapping, Optional

# Sibling JSON files edge-core materializes in CONFIG_DIR (see edge-core README
# "Twin JSON file" + the bidirectional sync section). All are read once at startup.
_EDGE_JSON = "edge.json"
_ENVIRONMENT_JSON = "environment.json"
_CAMERAS_JSON = "cameras.json"
_FINGERPRINT_JSON = "fingerprint.json"
_CREDENTIALS_JSON = "credentials.json"

# Sensor "type" values that the WebRTC/recording path treats as an RGB camera.
_RGB_SENSOR_TYPES = frozenset({"rgb", "camera", "rgb_camera", "rgbd", "color"})

def _strip(value: Optional[str]) -> str:
    return (value or "").strip()


def _parse_csv_uuids(raw: str) -> list[str]:
    out: list[str] = []
    for part in raw.split(","):
        uuid = part.strip()
        if uuid and uuid not in out:
            out.append(uuid)
    return out


def _mask_secret(value: str) -> str:
    if not value:
        return "(not set)"
    if len(value) <= 12:
        return "***"
    return f"{value[:6]}…{value[-4:]}"


def parse_bool_env(value: str | None) -> bool | None:
    """Parse common truthy/falsey env string values."""
    if value is None:
        return None
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    return None


def get_first_env(*names: str, environ: Mapping[str, str] | None = None) -> str | None:
    source = os.environ if environ is None else environ
    for name in names:
        value = source.get(name)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def resolve_mqtt_topic_prefix(
    *,
    mqtt_topic_prefix: str | None = None,
    environment: str | None = None,
    environ: Mapping[str, str] | None = None,
) -> str:
    """Resolve MQTT topic prefix (matches ``cyberwave==0.6.0`` SDK semantics)."""
    source = os.environ if environ is None else environ

    explicit = mqtt_topic_prefix
    if explicit is None:
        explicit = source.get("CYBERWAVE_MQTT_TOPIC_PREFIX")
    if explicit is not None and str(explicit).strip():
        return str(explicit).strip()

    env_value = environment
    if env_value is None:
        env_value = source.get("CYBERWAVE_ENVIRONMENT", "")
    env_value = str(env_value or "").strip()
    if env_value and env_value.lower() != "production":
        return env_value
    return ""


def mqtt_port_from_env(default: int = 8883) -> int:
    port_raw = get_first_env("CYBERWAVE_MQTT_PORT")
    if not port_raw:
        return default
    try:
        return int(port_raw)
    except ValueError:
        return default


def resolve_mqtt_use_tls(
    *,
    port: int | None = None,
    use_tls_raw: str | None = None,
) -> bool:
    """Resolve MQTT TLS flag (edge-core + Python SDK semantics)."""
    raw = use_tls_raw
    if raw is None:
        raw = get_first_env("CYBERWAVE_MQTT_USE_TLS", "CYBERWAVE_MQTT_TLS")
    parsed = parse_bool_env(raw)
    if parsed is not None:
        return parsed
    effective_port = port if port is not None else mqtt_port_from_env()
    return effective_port == 8883


def ensure_mqtt_tls_env() -> None:
    """Set ``CYBERWAVE_MQTT_USE_TLS`` when unset (entrypoint + driver startup)."""
    if get_first_env("CYBERWAVE_MQTT_USE_TLS", "CYBERWAVE_MQTT_TLS"):
        return
    port = mqtt_port_from_env()
    os.environ["CYBERWAVE_MQTT_USE_TLS"] = "true" if port == 8883 else "false"


# Public STUN fallback when nothing is configured.
DEFAULT_PUBLIC_STUN = "stun:stun.l.google.com:19302"


def resolve_ice_servers(
    environ: Mapping[str, str] | None = None,
) -> list[dict[str, Any]] | None:
    """WebRTC ICE servers from env, or ``None`` to defer to the SDK default relay.

    Per-deployment override: ``CYBERWAVE_WEBRTC_STUN_URL`` / ``_TURN_URL`` (+
    ``_TURN_USERNAME`` / ``_TURN_CREDENTIAL``) — no creds baked into this image. When
    nothing is configured, return ``None`` so ``BaseVideoStreamer`` uses its
    ``DEFAULT_TURN_SERVERS`` (the ``turn.cyberwave.com`` STUN+TURN relay), like the
    so101 / camera-driver edge nodes — this is what makes the cloud/WAN path connect.
    With ``force_turn=false`` that relay is only a fallback, not forced.
    """
    source = os.environ if environ is None else environ
    stun = _strip(source.get("CYBERWAVE_WEBRTC_STUN_URL"))
    turn_url = _strip(source.get("CYBERWAVE_WEBRTC_TURN_URL"))
    if not stun and not turn_url:
        return None

    servers: list[dict[str, Any]] = [{"urls": [stun or DEFAULT_PUBLIC_STUN]}]
    if turn_url:
        turn: dict[str, Any] = {"urls": [turn_url]}
        username = _strip(source.get("CYBERWAVE_WEBRTC_TURN_USERNAME"))
        credential = _strip(source.get("CYBERWAVE_WEBRTC_TURN_CREDENTIAL"))
        if username:
            turn["username"] = username
        if credential:
            turn["credential"] = credential
        servers.append(turn)
    return servers


def has_turn_server(ice_servers: list[dict[str, Any]]) -> bool:
    """True if any entry advertises a turn:/turns: URL."""
    for server in ice_servers:
        urls = server.get("urls", [])
        if isinstance(urls, str):
            urls = [urls]
        if any(str(u).startswith(("turn:", "turns:")) for u in urls):
            return True
    return False


@dataclass(frozen=True)
class EdgeDriverEnv:
    """CYBERWAVE_* variables forwarded from edge-core into the driver container."""

    environment: str = ""
    environment_uuid: str = ""
    edge_log_level: str = ""
    worker_log_level: str = ""
    base_url: str = ""
    mqtt_host: str = ""
    mqtt_port: str = ""
    mqtt_use_tls: bool = False
    api_key: str = ""
    twin_uuid: str = ""
    twin_json_file: str = ""
    twin_uuids: list[str] = field(default_factory=list)
    child_twin_uuids: list[str] = field(default_factory=list)
    data_backend: str = ""
    zenoh_connect: str = ""
    zenoh_shared_memory: str = ""
    config_dir: str = ""
    metadata_video_device: str = ""
    edge_video_device_map: str = ""

    @property
    def mqtt_port_int(self) -> Optional[int]:
        if not self.mqtt_port:
            return None
        try:
            return int(self.mqtt_port)
        except ValueError:
            return None

    @property
    def debug_logs_enabled(self) -> bool:
        return self.edge_log_level.strip().lower() == "debug"

    def load_twin_json(self) -> Optional[dict[str, Any]]:
        if not self.twin_json_file:
            return None
        path = Path(self.twin_json_file)
        if not path.is_file():
            return None
        try:
            with path.open(encoding="utf-8") as handle:
                data = json.load(handle)
            return data if isinstance(data, dict) else None
        except (OSError, json.JSONDecodeError):
            return None


def load_edge_driver_env() -> EdgeDriverEnv:
    """Read standard edge-core driver env vars from the process environment."""
    ensure_mqtt_tls_env()
    return edge_driver_env_from_environ(os.environ)


def edge_driver_env_from_environ(environ: Mapping[str, str]) -> EdgeDriverEnv:
    """Build ``EdgeDriverEnv`` from an explicit env mapping (for tests and tooling)."""
    mqtt_host = (
        get_first_env("CYBERWAVE_MQTT_HOST", "CYBERWAVE_MQTT_BROKER", environ=environ)
        or ""
    )
    mqtt_port = _strip(environ.get("CYBERWAVE_MQTT_PORT"))
    port_int = None
    if mqtt_port:
        try:
            port_int = int(mqtt_port)
        except ValueError:
            port_int = None
    return EdgeDriverEnv(
        environment=_strip(environ.get("CYBERWAVE_ENVIRONMENT")),
        environment_uuid=_strip(environ.get("CYBERWAVE_ENVIRONMENT_UUID")),
        edge_log_level=_strip(environ.get("CYBERWAVE_EDGE_LOG_LEVEL")),
        worker_log_level=_strip(environ.get("CYBERWAVE_WORKER_LOG_LEVEL")),
        base_url=_strip(environ.get("CYBERWAVE_BASE_URL")),
        mqtt_host=mqtt_host,
        mqtt_port=mqtt_port,
        mqtt_use_tls=resolve_mqtt_use_tls(
            port=port_int,
            use_tls_raw=environ.get("CYBERWAVE_MQTT_USE_TLS")
            or environ.get("CYBERWAVE_MQTT_TLS"),
        ),
        api_key=_strip(environ.get("CYBERWAVE_API_KEY"))
        or _strip(environ.get("CYBERWAVE_TOKEN")),
        twin_uuid=_strip(environ.get("CYBERWAVE_TWIN_UUID")),
        twin_json_file=_strip(environ.get("CYBERWAVE_TWIN_JSON_FILE")),
        twin_uuids=_parse_csv_uuids(environ.get("CYBERWAVE_TWIN_UUIDS", "")),
        child_twin_uuids=_parse_csv_uuids(
            environ.get("CYBERWAVE_CHILD_TWIN_UUIDS", "")
        ),
        data_backend=_strip(environ.get("CYBERWAVE_DATA_BACKEND")),
        zenoh_connect=_strip(environ.get("ZENOH_CONNECT")),
        zenoh_shared_memory=_strip(environ.get("ZENOH_SHARED_MEMORY")),
        config_dir=_strip(environ.get("CYBERWAVE_EDGE_CONFIG_DIR")),
        metadata_video_device=_strip(environ.get("CYBERWAVE_METADATA_VIDEO_DEVICE")),
        edge_video_device_map=_strip(environ.get("CYBERWAVE_EDGE_VIDEO_DEVICE_MAP")),
    )


# Mirrors cyberwave.config.DEFAULT_MQTT_HOST (kept local so this module needs no SDK import).
DEFAULT_MQTT_HOST = "mqtt.cyberwave.com"


def resolve_broker_host(param_host: Any, env: EdgeDriverEnv) -> tuple[str, str]:
    """Resolve MQTT broker host: ``CYBERWAVE_MQTT_HOST`` > ``broker.host`` param
    > the SDK default ``mqtt.cyberwave.com``. Like the so101 node, we let the SDK
    default apply instead of failing, so the driver connects when edge-core
    forwards only ``CYBERWAVE_API_KEY`` + ``CYBERWAVE_TWIN_UUID``.
    """
    if env.mqtt_host:
        return env.mqtt_host, "CYBERWAVE_MQTT_HOST"
    param = _strip(str(param_host)) if param_host is not None else ""
    if param:
        return param, "broker.host parameter"
    return DEFAULT_MQTT_HOST, "cyberwave SDK default"


def resolve_broker_port(param_port: Any, env: EdgeDriverEnv) -> tuple[int, str]:
    """Resolve MQTT broker port; ``CYBERWAVE_MQTT_PORT`` wins over params.yaml."""
    if env.mqtt_port_int is not None:
        return env.mqtt_port_int, "CYBERWAVE_MQTT_PORT"
    if param_port is not None:
        try:
            return int(param_port), "broker.port parameter"
        except (TypeError, ValueError):
            pass
    return 8883, "default"


@dataclass(frozen=True)
class ResolvedBrokerSettings:
    """Effective MQTT endpoint the driver dials, with provenance for logging."""

    host: str
    host_source: str
    port: int
    port_source: str
    use_tls: bool


def resolve_broker_settings(
    env: EdgeDriverEnv,
    *,
    host_param: Any = None,
    port_param: Any = None,
) -> ResolvedBrokerSettings:
    """Resolve the effective MQTT endpoint (host/port/TLS) from edge-core env +
    params.yaml in one place, applying the precedence documented on
    :func:`resolve_broker_host` and :func:`resolve_broker_port`.
    """
    host, host_source = resolve_broker_host(host_param, env)
    port, port_source = resolve_broker_port(port_param, env)
    return ResolvedBrokerSettings(
        host=host,
        host_source=host_source,
        port=port,
        port_source=port_source,
        use_tls=env.mqtt_use_tls,
    )


def apply_resolved_mqtt_to_environ(
    host: str,
    port: int,
    *,
    api_key: str = "",
    use_tls: bool | None = None,
    target: MutableMapping[str, str] | None = None,
) -> None:
    """Publish resolved broker settings into ``os.environ`` for the Cyberwave SDK."""
    dest = os.environ if target is None else target
    stripped_host = _strip(host)
    if stripped_host:
        dest["CYBERWAVE_MQTT_HOST"] = stripped_host
    dest["CYBERWAVE_MQTT_PORT"] = str(port)
    if use_tls is not None:
        dest["CYBERWAVE_MQTT_USE_TLS"] = "true" if use_tls else "false"
    if api_key:
        dest.setdefault("CYBERWAVE_API_KEY", api_key)


def log_edge_driver_env(logger: Any, env: EdgeDriverEnv) -> None:
    """Log edge driver env visibility for operators (secrets masked)."""
    twin_json_status = "not set"
    if env.twin_json_file:
        path = Path(env.twin_json_file)
        if path.is_file():
            twin_json_status = f"file ok ({path})"
        else:
            twin_json_status = f"missing ({path})"

    twin_uuids_summary = ",".join(env.twin_uuids) if env.twin_uuids else "(not set)"
    child_summary = (
        ",".join(env.child_twin_uuids) if env.child_twin_uuids else "(not set)"
    )
    topic_prefix = resolve_mqtt_topic_prefix(environment=env.environment)
    topic_family = (
        f"{topic_prefix}cyberwave/twin/<uuid>/…"
        if topic_prefix
        else "cyberwave/twin/<uuid>/…"
    )

    lines = [
        "--- Edge driver environment (from edge-core) ---",
        f"CYBERWAVE_ENVIRONMENT={env.environment or '(not set)'}",
        f"MQTT topic family={topic_family}",
        f"CYBERWAVE_ENVIRONMENT_UUID={env.environment_uuid or '(not set)'}",
        f"CYBERWAVE_EDGE_LOG_LEVEL={env.edge_log_level or '(not set)'}",
        f"CYBERWAVE_WORKER_LOG_LEVEL={env.worker_log_level or '(not set)'}",
        f"CYBERWAVE_BASE_URL={env.base_url or '(not set)'}",
        f"CYBERWAVE_MQTT_HOST={env.mqtt_host or '(not set)'}",
        f"CYBERWAVE_MQTT_PORT={env.mqtt_port or '(not set)'}",
        f"CYBERWAVE_MQTT_USE_TLS={str(env.mqtt_use_tls).lower()}",
        f"CYBERWAVE_API_KEY={_mask_secret(env.api_key)}",
        f"CYBERWAVE_TWIN_UUID={env.twin_uuid or '(not set)'}",
        f"CYBERWAVE_TWIN_JSON_FILE={twin_json_status}",
        f"CYBERWAVE_TWIN_UUIDS={twin_uuids_summary}",
        f"CYBERWAVE_CHILD_TWIN_UUIDS={child_summary}",
        f"CYBERWAVE_DATA_BACKEND={env.data_backend or '(not set)'}",
        f"ZENOH_CONNECT={env.zenoh_connect or '(not set)'}",
        f"ZENOH_SHARED_MEMORY={env.zenoh_shared_memory or '(not set)'}",
        "-----------------------------------------------",
    ]
    for line in lines:
        logger.info(line)

    missing_required: list[str] = []
    if not env.api_key:
        missing_required.append("CYBERWAVE_API_KEY")
    if not env.twin_uuid:
        missing_required.append("CYBERWAVE_TWIN_UUID")
    if not env.mqtt_host:
        missing_required.append("CYBERWAVE_MQTT_HOST")
    if missing_required:
        logger.warning(
            "Missing required edge driver env: " + ", ".join(missing_required)
        )

    if env.mqtt_port_int == 1883 and env.mqtt_use_tls:
        logger.warning(
            "CYBERWAVE_MQTT_USE_TLS=true with CYBERWAVE_MQTT_PORT=1883 — "
            "plain MQTT port; connection may fail unless the broker expects TLS"
        )
    elif env.mqtt_port_int == 8883 and not env.mqtt_use_tls:
        logger.warning(
            "CYBERWAVE_MQTT_USE_TLS=false with CYBERWAVE_MQTT_PORT=8883 — "
            "TLS port without TLS; connection may fail"
        )


# ---------------------------------------------------------------------------
# Twin/asset/environment/sensor config from the edge-core JSON files. This module
# is the ONE place that reads them; ROS/hardware config stays in the mapping YAML.


def normalize_sensor_type(raw: Any) -> str:
    """Normalize a raw sensor type: camera/rgb_camera/rgbd/color->rgb, depth_camera->depth, lidar->lidar_3d."""
    value = str(raw or "").strip().lower()
    if value in {"rgb", "camera", "rgb_camera", "rgbd", "color"}:
        return "rgb"
    if value in {"depth", "depth_camera"}:
        return "depth"
    if value == "lidar":
        return "lidar_3d"
    return value


@dataclass(frozen=True)
class SensorInfo:
    """One normalized sensor from the twin/asset schema."""

    id: str
    name: str
    type: str
    parent_link: Optional[str]
    parameters: dict[str, Any]
    raw: dict[str, Any]

    @property
    def is_rgb(self) -> bool:
        return self.type == "rgb"


@dataclass(frozen=True)
class CameraSelection:
    """The chosen RGB camera sensor plus its resolved runtime bindings."""

    sensor: Optional[SensorInfo]
    frame_id: Optional[str]  # = sensor.parent_link (twin JSON), e.g. "camera_link"
    video_device: Optional[str]
    video_device_source: str
    edge_config: dict[str, Any]

    @property
    def sensor_id(self) -> Optional[str]:
        return self.sensor.id if self.sensor else None


@dataclass(frozen=True)
class TwinConfig:
    """Immutable, cached snapshot of everything read from the JSON files + env."""

    env: EdgeDriverEnv
    twin: dict[str, Any]
    asset: dict[str, Any]
    metadata: dict[str, Any]
    edge_configs: dict[str, Any]
    edge_fingerprint: Optional[str]
    sensors: tuple[SensorInfo, ...]
    sensors_by_id: dict[str, SensorInfo]
    camera: CameraSelection
    sensors_devices: dict[str, str]
    environment_uuid: Optional[str]
    environment_twin_uuids: tuple[str, ...]
    edge_record: dict[str, Any]
    cameras_config: dict[str, Any]
    fingerprint: Optional[str]
    registry_id: Optional[str]
    twin_uuid: Optional[str]
    source_files: dict[str, str]

    def sensor(self, sensor_id: str) -> Optional[SensorInfo]:
        return self.sensors_by_id.get(sensor_id)


def read_json_file(path: Path) -> tuple[Optional[dict[str, Any]], str]:
    """Read a JSON object; never raise. Returns (data|None, "ok"|"missing"|"unreadable")."""
    if not path.is_file():
        return None, "missing"
    try:
        with path.open(encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, json.JSONDecodeError, ValueError):
        return None, "unreadable"
    if not isinstance(data, dict):
        return None, "unreadable"
    return data, "ok"


def resolve_config_dir(environ: Mapping[str, str]) -> Path:
    """CONFIG_DIR precedence: CYBERWAVE_EDGE_CONFIG_DIR -> /app/.cyberwave -> twin-JSON sibling dir -> ~/.cyberwave."""
    explicit = _strip(environ.get("CYBERWAVE_EDGE_CONFIG_DIR"))
    if explicit:
        p = Path(explicit).expanduser()
        if p.is_dir():
            return p

    container = Path("/app/.cyberwave")
    if container.is_dir():
        return container

    twin_json = _strip(environ.get("CYBERWAVE_TWIN_JSON_FILE"))
    if twin_json:
        parent = Path(twin_json).expanduser().parent
        if (parent / _EDGE_JSON).is_file() or (parent / _ENVIRONMENT_JSON).is_file():
            return parent

    return Path("~/.cyberwave").expanduser()


def resolve_twin_json_path(environ: Mapping[str, str], config_dir: Path) -> Optional[Path]:
    """Twin JSON path: CYBERWAVE_TWIN_JSON_FILE else {config_dir}/{twin_uuid}.json."""
    explicit = _strip(environ.get("CYBERWAVE_TWIN_JSON_FILE"))
    if explicit:
        return Path(explicit).expanduser()
    twin_uuid = _strip(environ.get("CYBERWAVE_TWIN_UUID"))
    if twin_uuid:
        candidate = config_dir / f"{twin_uuid}.json"
        return candidate
    return None


def _sensor_from_raw(raw: dict[str, Any]) -> Optional[SensorInfo]:
    if not isinstance(raw, dict):
        return None
    params = raw.get("parameters") if isinstance(raw.get("parameters"), dict) else {}
    sensor_id = raw.get("id") or params.get("id") or raw.get("name")
    if sensor_id is None:
        return None
    name = raw.get("name") or sensor_id
    return SensorInfo(
        id=str(sensor_id),
        name=str(name),
        type=normalize_sensor_type(raw.get("type")),
        parent_link=raw.get("parent_link"),
        parameters=dict(params),
        raw=dict(raw),
    )


def _sensor_lists(twin: Mapping[str, Any], asset: Mapping[str, Any]) -> list[list[Any]]:
    """Sensor source locations in authoritative precedence order (§0.2)."""

    def at(obj: Mapping[str, Any], *keys: str) -> list[Any]:
        cur: Any = obj
        for key in keys:
            if not isinstance(cur, Mapping):
                return []
            cur = cur.get(key)
        return cur if isinstance(cur, list) else []

    return [
        at(twin, "capabilities", "sensors"),
        at(asset, "capabilities", "sensors"),
        at(twin, "universal_schema", "sensors"),
        at(asset, "universal_schema", "sensors"),
        at(twin, "metadata", "_production_capabilities", "sensors"),
        at(asset, "metadata", "_production_capabilities", "sensors"),
    ]


def extract_sensors(
    twin: Mapping[str, Any], asset: Mapping[str, Any]
) -> tuple[SensorInfo, ...]:
    """Walk the authoritative sensor sources, dedupe by id (first wins)."""
    out: list[SensorInfo] = []
    seen: set[str] = set()
    for source in _sensor_lists(twin, asset):
        for raw in source:
            info = _sensor_from_raw(raw)
            if info is None or info.id in seen:
                continue
            seen.add(info.id)
            out.append(info)
    return tuple(out)


def resolve_edge_configs(
    metadata: Mapping[str, Any], fingerprint: Optional[str]
) -> dict[str, Any]:
    """Resolve metadata.edge_configs to a flat dict (handles legacy fingerprint-keyed shape); absent -> {}."""
    ec = metadata.get("edge_configs")
    if not isinstance(ec, Mapping) or not ec:
        return {}
    if fingerprint and fingerprint in ec and isinstance(ec[fingerprint], Mapping):
        return dict(ec[fingerprint])
    return dict(ec)


def resolve_cameras_config(
    edge_record: Mapping[str, Any], cameras_json: Optional[Mapping[str, Any]]
) -> dict[str, Any]:
    """Prefer edge.json's metadata.cameras over cameras.json (mirrors edge-core _read_cameras_config)."""
    meta = edge_record.get("metadata") if isinstance(edge_record, Mapping) else None
    if isinstance(meta, Mapping):
        cameras = meta.get("cameras")
        if isinstance(cameras, Mapping) and cameras:
            return dict(cameras)
    if isinstance(cameras_json, Mapping) and cameras_json:
        return dict(cameras_json)
    return {}


def resolve_video_device_from_cameras(
    cameras_config: Mapping[str, Any], twin_uuid: Optional[str]
) -> Optional[str]:
    """Resolve /dev/videoN: twin_to_device[uuid] (else selected_device) is a device index -> devices[index].primary_path."""
    if not isinstance(cameras_config, Mapping) or not cameras_config:
        return None
    ttd = cameras_config.get("twin_to_device")
    index: Any = None
    if isinstance(ttd, Mapping) and twin_uuid and twin_uuid in ttd:
        index = ttd.get(twin_uuid)
    if index is None:
        index = cameras_config.get("selected_device")
    if index is None:
        return None
    devices = cameras_config.get("devices")
    if isinstance(devices, list):
        for dev in devices:
            if isinstance(dev, Mapping) and dev.get("index") == index:
                primary = dev.get("primary_path")
                if primary:
                    return str(primary)
                paths = dev.get("paths")
                if isinstance(paths, list) and paths:
                    return str(paths[0])
    try:
        return f"/dev/video{int(index)}"
    except (TypeError, ValueError):
        return None


def resolve_camera_selection(
    sensors: tuple[SensorInfo, ...],
    *,
    env: EdgeDriverEnv,
    cameras_config: Mapping[str, Any],
    sensors_devices: Mapping[str, str],
    edge_configs: Mapping[str, Any],
    twin_uuid: Optional[str],
) -> CameraSelection:
    """First RGB sensor -> frame_id (parent_link) + video device
    (CYBERWAVE_METADATA_VIDEO_DEVICE > sensors_devices > cameras block)."""
    sensor = next((s for s in sensors if s.is_rgb), None)
    frame_id = sensor.parent_link if sensor else None

    camera_config = edge_configs.get("camera_config") if isinstance(edge_configs, Mapping) else None
    camera_config = dict(camera_config) if isinstance(camera_config, Mapping) else {}

    device: Optional[str] = None
    source = "unresolved"
    if env.metadata_video_device:
        device, source = env.metadata_video_device, "CYBERWAVE_METADATA_VIDEO_DEVICE"
    elif sensor and isinstance(sensors_devices, Mapping) and sensors_devices.get(sensor.id):
        device, source = str(sensors_devices[sensor.id]), "metadata.sensors_devices"
    else:
        resolved = resolve_video_device_from_cameras(cameras_config, twin_uuid)
        if resolved:
            device, source = resolved, "cameras_config"

    return CameraSelection(
        sensor=sensor,
        frame_id=frame_id,
        video_device=device,
        video_device_source=source,
        edge_config=camera_config,
    )


def _effective_environ(
    environ: Mapping[str, str], credentials_envs: Mapping[str, str]
) -> dict[str, str]:
    """Overlay credentials.json envs UNDER the process env (process env wins)."""
    merged: dict[str, str] = {}
    for key, value in credentials_envs.items():
        if isinstance(key, str) and isinstance(value, str):
            merged[key] = value
    for key, value in environ.items():
        if isinstance(value, str):
            merged[key] = value
    return merged


def build_twin_config_via_sdk(env: EdgeDriverEnv) -> Optional[dict[str, Any]]:
    """Fallback: fetch twin (+ nested asset) via the SDK when the JSON file is absent.

    Lazy import; requires CYBERWAVE_API_KEY + CYBERWAVE_TWIN_UUID. Returns None if unusable.
    """
    if not (env.api_key and env.twin_uuid):
        return None
    try:
        from cyberwave import Cyberwave
    except Exception:
        return None
    try:
        kwargs: dict[str, Any] = {"api_key": env.api_key}
        if env.base_url:
            kwargs["base_url"] = env.base_url
        client = Cyberwave(**kwargs)
        twin_obj = client.twins.get_raw(env.twin_uuid)
        twin_dict = twin_obj.to_dict() if hasattr(twin_obj, "to_dict") else dict(twin_obj)
        asset_uuid = twin_dict.get("asset_uuid") or twin_dict.get("asset_id")
        if asset_uuid and "asset" not in twin_dict:
            try:
                asset_obj = client.assets.get(asset_uuid)
                twin_dict["asset"] = (
                    asset_obj.to_dict() if hasattr(asset_obj, "to_dict") else dict(asset_obj)
                )
            except Exception:
                pass
        return twin_dict if isinstance(twin_dict, dict) else None
    except Exception:
        return None


def build_twin_config(
    environ: Optional[Mapping[str, str]] = None,
    *,
    config_dir: Optional[Path] = None,
) -> TwinConfig:
    """Read all edge-core JSON files + env once, project to a TwinConfig; never mutates the files."""
    src = dict(os.environ) if environ is None else dict(environ)
    cfg_dir = config_dir if config_dir is not None else resolve_config_dir(src)

    source_files: dict[str, str] = {}

    credentials, source_files[_CREDENTIALS_JSON] = read_json_file(cfg_dir / _CREDENTIALS_JSON)
    cred_envs = credentials.get("envs") if isinstance(credentials, dict) else None
    effective_env = _effective_environ(src, cred_envs if isinstance(cred_envs, dict) else {})
    env = edge_driver_env_from_environ(effective_env)

    edge_record, source_files[_EDGE_JSON] = read_json_file(cfg_dir / _EDGE_JSON)
    environment, source_files[_ENVIRONMENT_JSON] = read_json_file(cfg_dir / _ENVIRONMENT_JSON)
    cameras_json, source_files[_CAMERAS_JSON] = read_json_file(cfg_dir / _CAMERAS_JSON)
    fingerprint_file, source_files[_FINGERPRINT_JSON] = read_json_file(cfg_dir / _FINGERPRINT_JSON)

    twin_path = resolve_twin_json_path(effective_env, cfg_dir)
    twin_full: Optional[dict[str, Any]] = None
    if twin_path is not None:
        twin_full, source_files["twin.json"] = read_json_file(twin_path)
    else:
        source_files["twin.json"] = "missing"

    # Fallback to the SDK when the JSON file is absent.
    if not twin_full:
        sdk_twin = build_twin_config_via_sdk(env)
        if sdk_twin is not None:
            twin_full = sdk_twin
            source_files["twin.json"] = "sdk"

    twin_full = twin_full or {}
    twin = dict(twin_full)  # copy; never mutate the on-disk file (edge-core syncs it back)
    asset = twin.pop("asset", {})
    asset = dict(asset) if isinstance(asset, dict) else {}
    metadata = twin.get("metadata") if isinstance(twin.get("metadata"), dict) else {}

    edge_fingerprint = metadata.get("edge_fingerprint") if metadata else None
    fingerprint = fingerprint_file.get("fingerprint") if isinstance(fingerprint_file, dict) else None

    sensors = extract_sensors(twin, asset)
    sensors_by_id = {s.id: s for s in sensors}
    sensors_devices_raw = metadata.get("sensors_devices") if metadata else None
    sensors_devices = (
        {str(k): str(v) for k, v in sensors_devices_raw.items()}
        if isinstance(sensors_devices_raw, dict)
        else {}
    )

    edge_record = edge_record or {}
    cameras_config = resolve_cameras_config(edge_record, cameras_json)
    edge_configs = resolve_edge_configs(metadata, edge_fingerprint)

    twin_uuid = twin.get("uuid") or (_strip(effective_env.get("CYBERWAVE_TWIN_UUID")) or None)

    camera = resolve_camera_selection(
        sensors,
        env=env,
        cameras_config=cameras_config,
        sensors_devices=sensors_devices,
        edge_configs=edge_configs,
        twin_uuid=twin_uuid,
    )

    # environment.json wins for the environment UUID, else env, else twin.
    environment = environment or {}
    environment_uuid = (
        (environment.get("uuid") if isinstance(environment, dict) else None)
        or (env.environment_uuid or None)
        or twin.get("environment_uuid")
    )
    env_twin_uuids_raw = environment.get("twin_uuids") if isinstance(environment, dict) else None
    if isinstance(env_twin_uuids_raw, list) and env_twin_uuids_raw:
        environment_twin_uuids = tuple(str(u) for u in env_twin_uuids_raw)
    else:
        environment_twin_uuids = tuple(env.twin_uuids)

    registry_id = asset.get("registry_id") or (metadata.get("registry_id") if metadata else None)

    return TwinConfig(
        env=env,
        twin=twin,
        asset=asset,
        metadata=dict(metadata),
        edge_configs=edge_configs,
        edge_fingerprint=edge_fingerprint,
        sensors=sensors,
        sensors_by_id=sensors_by_id,
        camera=camera,
        sensors_devices=sensors_devices,
        environment_uuid=environment_uuid,
        environment_twin_uuids=environment_twin_uuids,
        edge_record=edge_record,
        cameras_config=cameras_config,
        fingerprint=fingerprint,
        registry_id=registry_id,
        twin_uuid=twin_uuid,
        source_files=source_files,
    )


# Cache keyed on (twin_json_path, config_dir, mtime_ns): repeat calls free, rebuilds on rewrite.
_TWIN_CONFIG_CACHE: dict[tuple[str, str, int], TwinConfig] = {}


def load_twin_config() -> TwinConfig:
    """Read the real process environment + JSON files, memoized on file mtime."""
    ensure_mqtt_tls_env()
    environ = os.environ
    cfg_dir = resolve_config_dir(environ)
    twin_path = resolve_twin_json_path(environ, cfg_dir)
    mtime = 0
    if twin_path is not None:
        try:
            mtime = twin_path.stat().st_mtime_ns
        except OSError:
            mtime = 0
    key = (str(twin_path or ""), str(cfg_dir), mtime)
    cached = _TWIN_CONFIG_CACHE.get(key)
    if cached is not None:
        return cached
    cfg = build_twin_config(environ, config_dir=cfg_dir)
    _TWIN_CONFIG_CACHE[key] = cfg
    return cfg


def log_twin_config(logger: Any, cfg: TwinConfig) -> None:
    """Log the resolved twin/sensor/camera config (anti-discard guard: every read value is surfaced)."""
    log_edge_driver_env(logger, cfg.env)

    files = ", ".join(f"{name}={status}" for name, status in sorted(cfg.source_files.items()))
    lines = [
        "--- Twin config (from edge-core JSON files) ---",
        f"config_dir source files: {files}",
        f"twin_uuid={cfg.twin_uuid or '(not set)'}",
        f"registry_id={cfg.registry_id or '(not set)'}",
        f"environment_uuid={cfg.environment_uuid or '(not set)'}",
        f"environment_twin_uuids={','.join(cfg.environment_twin_uuids) or '(not set)'}",
        (
            f"edge_fingerprint={cfg.edge_fingerprint or '(not set)'}"
            f" (fingerprint.json={cfg.fingerprint or '(not set)'})"
        ),
        f"edge_configs_keys={sorted(cfg.edge_configs.keys()) or '(empty)'}",
        f"sensors_devices={cfg.sensors_devices or '(empty)'}",
        f"sensors={[f'{s.id}:{s.type}:{s.parent_link}' for s in cfg.sensors] or '(none)'}",
    ]
    cam = cfg.camera
    if cam.sensor is not None:
        lines.append(
            f"camera sensor={cam.sensor.id} type={cam.sensor.type} "
            f"frame_id={cam.frame_id or '(none)'} "
            f"video_device={cam.video_device or '(unresolved)'} "
            f"(source={cam.video_device_source})"
        )
    else:
        lines.append("camera sensor=(none resolved) — WebRTC recording disabled")
    lines.append("-----------------------------------------------")
    for line in lines:
        logger.info(line)

    if cfg.edge_fingerprint and cfg.fingerprint and cfg.edge_fingerprint != cfg.fingerprint:
        logger.warning(
            "twin.metadata.edge_fingerprint (%s) != fingerprint.json (%s)",
            cfg.edge_fingerprint,
            cfg.fingerprint,
        )
    if (
        cfg.environment_uuid
        and cfg.twin.get("environment_uuid")
        and cfg.environment_uuid != cfg.twin.get("environment_uuid")
    ):
        logger.warning(
            "environment.json uuid (%s) != twin.environment_uuid (%s)",
            cfg.environment_uuid,
            cfg.twin.get("environment_uuid"),
        )
