"""Per-robot ROS topic namespacing, derived from the twin UUID (``ugv_beast_<first6>``)."""

from __future__ import annotations

import re

# ROS 2 name tokens allow only [A-Za-z0-9_] and must not start with a digit.
_ROS_NAME_INVALID = re.compile(r"[^A-Za-z0-9_]")

# Prefix guarantees a ROS-valid leading letter even when the UUID starts with a digit.
_DEFAULT_NAMESPACE_PREFIX = "ugv_beast"


def _sanitize_ros_namespace(value: str) -> str:
    """Coerce *value* into a token-valid ROS 2 namespace (or '' for none)."""
    raw = str(value or "").strip().strip("/")
    if not raw:
        return ""
    segments: list[str] = []
    for seg in raw.split("/"):
        cleaned = _ROS_NAME_INVALID.sub("_", seg.strip())
        if not cleaned:
            continue
        if cleaned[0].isdigit():
            cleaned = f"_{cleaned}"
        segments.append(cleaned)
    return "/".join(segments)


def derive_robot_namespace(
    twin_uuid: str | None,
    *,
    prefix: str = _DEFAULT_NAMESPACE_PREFIX,
) -> str:
    """Derive a per-robot ROS 2 namespace (``{prefix}_{first6}``) from a twin UUID.

    Keep this formula in sync with master_beast.launch.py, which derives the same
    namespace for the hardware nodes so the bridge and hardware share one graph.
    """
    raw = str(twin_uuid or "").replace("-", "").strip().lower()
    if not raw:
        return ""
    return _sanitize_ros_namespace(f"{prefix}_{raw[:6]}")


def resolve_ros_namespace(
    *,
    configured_namespace: str = "",
    twin_uuid: str | None = None,
) -> str:
    """Resolve the active ROS namespace: configured override wins, else derived from *twin_uuid*."""
    configured = _sanitize_ros_namespace(configured_namespace)
    if configured:
        return configured
    return derive_robot_namespace(twin_uuid)


def to_relative_topic(topic_name: str) -> str:
    """Return the RELATIVE form of a topic name (strip a leading ``/``) so rcl applies the node namespace."""
    return str(topic_name or "").strip().lstrip("/")


def resolve_ros_topic(topic_name: str, namespace: str = "") -> str:
    """Return a fully-qualified ABSOLUTE ROS topic in ``namespace`` (legacy cross-namespace path; prefer :func:`to_relative_topic`)."""
    topic = str(topic_name or "").strip()
    if not topic:
        return topic
    base_topic = topic.lstrip("/")
    ns = str(namespace or "").strip().strip("/")
    if not ns:
        return f"/{base_topic}"
    if base_topic == ns or base_topic.startswith(f"{ns}/"):
        return f"/{base_topic}"
    return f"/{ns}/{base_topic}"
