"""Helpers for per-robot ROS topic namespacing.

Multiple UGV Beasts can share one ROS 2 graph, so each robot scopes its nodes
and topics under a namespace derived from its twin UUID:

    ugv_beast_<first 6 hex chars of the twin uuid>     e.g. ugv_beast_27dca7

The ``ugv_beast_`` prefix is essential: a raw UUID is **not** a valid ROS 2 name
(it contains '-' and may start with a digit), so it can never be used directly.
The prefix guarantees a valid leading letter, and the 6-char hex suffix keeps it
unique-enough across a fleet while staying readable.

IMPORTANT — both sides of the ROS graph must use the *same* namespace or topics
won't connect: ``master_beast.launch.py`` namespaces the hardware nodes, and
``mqtt_bridge`` prefixes its topics via :func:`resolve_ros_topic`. Keep
:func:`derive_robot_namespace` in sync with the derivation in that launch file.

With no twin UUID (and no explicit ``ros_namespace`` override) the result is ''
→ global topics (single-robot / dev).
"""

from __future__ import annotations

import re

# ROS 2 name tokens may contain only [A-Za-z0-9_] and must not start with a digit.
_ROS_NAME_INVALID = re.compile(r"[^A-Za-z0-9_]")

# Prefix applied to the twin-derived namespace; guarantees a ROS-valid leading
# letter even when the UUID's first characters are digits.
_DEFAULT_NAMESPACE_PREFIX = "ugv_beast"


def _sanitize_ros_namespace(value: str) -> str:
    """Coerce *value* into a token-valid ROS 2 namespace (or '' for none).

    Each '/'-separated segment is sanitized independently: disallowed characters
    become '_', and a leading digit is prefixed with '_'. An empty result means
    "no namespace" (global topics).
    """
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
    """Derive a per-robot ROS 2 namespace from a twin UUID.

    Pattern: ``{prefix}_{first 6 hex chars of the uuid}`` — e.g. twin
    ``27dca72f-6e17-...`` -> ``ugv_beast_27dca7``. Returns '' when *twin_uuid* is
    empty. The result is sanitized to a valid ROS 2 name.

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
    """Resolve the active ROS namespace.

    An explicitly-configured ``ros_namespace`` wins (sanitized to a valid ROS
    name); otherwise one is derived from *twin_uuid* as ``ugv_beast_<first6>``.
    Empty result → global ROS topics.
    """
    configured = _sanitize_ros_namespace(configured_namespace)
    if configured:
        return configured
    return derive_robot_namespace(twin_uuid)


def resolve_ros_topic(topic_name: str, namespace: str = "") -> str:
    """Return a fully-qualified ROS topic in the active namespace."""
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
