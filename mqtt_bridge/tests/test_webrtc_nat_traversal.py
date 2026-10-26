"""Guard that the shipped WebRTC config defaults force_turn to false (normal ICE, not relay-only)."""

from __future__ import annotations

import ast
import re
from pathlib import Path
from typing import Any

import pytest
import yaml

COMPONENT_ROOT = Path(__file__).resolve().parents[2]
PARAMS_YAML = COMPONENT_ROOT / "config" / "params.yaml"
NODE_PY = COMPONENT_ROOT / "mqtt_bridge" / "mqtt_bridge_node.py"


def _find_force_turn(obj: Any) -> list[Any]:
    """Collect every ``force_turn`` value anywhere in the parsed params tree."""
    found: list[Any] = []
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key == "force_turn":
                found.append(value)
            found.extend(_find_force_turn(value))
    elif isinstance(obj, list):
        for item in obj:
            found.extend(_find_force_turn(item))
    return found


def test_params_yaml_default_is_not_relay_only() -> None:
    """The shipped params.yaml must default force_turn to false (normal ICE)."""
    assert PARAMS_YAML.exists(), f"missing {PARAMS_YAML}"
    data = yaml.safe_load(PARAMS_YAML.read_text())

    values = _find_force_turn(data)
    assert values, (
        "config/params.yaml no longer declares webrtc.force_turn — the "
        "NAT-traversal default is unguarded. Re-add it as `force_turn: false`."
    )
    offenders = [v for v in values if v is not False]
    assert not offenders, (
        "config/params.yaml ships force_turn as relay-only "
        f"({offenders}). That forces ALL WebRTC media through TURN and breaks "
        "NAT traversal whenever the TURN is unreachable (e.g. edge + backend on "
        "the same LAN). The safe default is `force_turn: false` — relay-only "
        "must be an explicit per-deployment opt-in, never the shipped default."
    )


def test_node_declares_force_turn_defaulting_to_false() -> None:
    """The node's declare_parameter default for force_turn must also be False (code-side fallback)."""
    src = NODE_PY.read_text()
    tree = ast.parse(src)

    defaults: list[Any] = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and func.attr == "declare_parameter"):
            continue
        if not node.args:
            continue
        name = node.args[0]
        if isinstance(name, ast.Constant) and name.value == "webrtc.force_turn":
            # declare_parameter("webrtc.force_turn", <default>)
            default = node.args[1] if len(node.args) > 1 else None
            defaults.append(default.value if isinstance(default, ast.Constant) else default)

    assert defaults, (
        "mqtt_bridge_node.py no longer declares webrtc.force_turn — the code-side "
        "NAT-traversal default is unguarded."
    )
    offenders = [d for d in defaults if d is not False]
    assert not offenders, (
        "mqtt_bridge_node.py declares webrtc.force_turn with a non-False default "
        f"({offenders}); relay-only must never be the code default. Use "
        'self.declare_parameter("webrtc.force_turn", False).'
    )


def test_force_turn_comment_documents_the_flag() -> None:
    """Guard the operator-facing doc so the flag stays explained, not silent."""
    text = PARAMS_YAML.read_text()
    block = re.search(r"force_turn", text)
    assert block, "force_turn key missing from params.yaml"
    assert re.search(r"relay[- ]only|RELAY-ONLY", text, re.IGNORECASE), (
        "params.yaml no longer documents that force_turn=true is relay-only — "
        "keep the comment so operators understand the NAT-traversal trade-off."
    )


def test_node_bakes_no_turn_credentials() -> None:
    """ICE/TURN config must come from env (resolve_ice_servers), never be baked in.

    The old static turn.cyberwave.com + cyberwave-user/cyberwave-admin creds must
    never reappear in the node; they belong in per-deployment env vars.
    """
    src = NODE_PY.read_text()
    for forbidden in ("cyberwave-user", "cyberwave-admin", "turn.cyberwave.com"):
        assert forbidden not in src, (
            f"mqtt_bridge_node.py bakes in {forbidden!r}; source ICE servers from "
            "env via resolve_ice_servers() (CYBERWAVE_WEBRTC_STUN_URL / _TURN_URL) "
            "instead of hardcoding them."
        )
    assert "resolve_ice_servers" in src, (
        "the node no longer builds ICE servers via resolve_ice_servers() — ICE "
        "config must be env-sourced, not hardcoded."
    )
