"""Guard that Dockerfiles and in-image scripts bake no per-deployment endpoint/credential/twin env vars."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

COMPONENT_ROOT = Path(__file__).resolve().parents[2]

# Per-deployment values that must be passed at runtime, never baked into the image.
FORBIDDEN_ENV_VARS = (
    "CYBERWAVE_MQTT_HOST",
    "CYBERWAVE_MQTT_BROKER",
    "CYBERWAVE_MQTT_PORT",
    "CYBERWAVE_MQTT_USE_TLS",
    "CYBERWAVE_MQTT_TLS",
    "CYBERWAVE_BASE_URL",
    "CYBERWAVE_API_KEY",
    "CYBERWAVE_TOKEN",
    "CYBERWAVE_TWIN_UUID",
    "CYBERWAVE_TWIN_UUIDS",
    "CYBERWAVE_CHILD_TWIN_UUIDS",
    "CYBERWAVE_ENVIRONMENT_UUID",
)

# Active slim build + any archived legacy Dockerfiles.
DOCKERFILES = [
    COMPONENT_ROOT / "docker-conf" / "Dockerfile",
    *sorted((COMPONENT_ROOT / "docker-conf" / "legacy").glob("Dockerfile*")),
]
IN_IMAGE_SCRIPTS = sorted((COMPONENT_ROOT / "scripts" / "ugv_beast").glob("*.sh"))

_VARS_ALT = "|".join(re.escape(v) for v in FORBIDDEN_ENV_VARS)
# Dockerfile `ENV VAR=...` or `ENV VAR ...`
_ENV_RE = re.compile(rf"^\s*ENV\s+({_VARS_ALT})\b")
# Shell `export VAR=<literal>` or `VAR=<literal>` where RHS is a hardcoded value
# (not a "$VAR"/"${VAR}" passthrough and not empty).
_EXPORT_RE = re.compile(rf"^\s*(?:export\s+)?({_VARS_ALT})\s*=\s*(['\"]?)([^'\"\n]*)\2\s*$")


@pytest.mark.parametrize("dockerfile", DOCKERFILES, ids=lambda p: p.name)
def test_dockerfiles_bake_no_endpoint_env(dockerfile: Path) -> None:
    if not dockerfile.exists():
        pytest.skip(f"{dockerfile.name} not present")
    offenders = [
        f"{dockerfile.name}:{i + 1}: {line.strip()}"
        for i, line in enumerate(dockerfile.read_text().splitlines())
        if _ENV_RE.match(line)
    ]
    assert not offenders, (
        "Dockerfile bakes a per-deployment endpoint/credential env var into the "
        "image (must be passed at runtime instead):\n" + "\n".join(offenders)
    )


@pytest.mark.parametrize("script", IN_IMAGE_SCRIPTS, ids=lambda p: p.name)
def test_in_image_scripts_hardcode_no_endpoint_env(script: Path) -> None:
    offenders: list[str] = []
    for i, line in enumerate(script.read_text().splitlines()):
        m = _EXPORT_RE.match(line)
        if not m:
            continue
        rhs = m.group(3).strip()
        # allow passthrough (export VAR="$VAR") and unset/empty
        if rhs and not rhs.startswith("$"):
            offenders.append(f"{script.name}:{i + 1}: {line.strip()}")
    assert not offenders, (
        "In-image script hardcodes a per-deployment endpoint/credential env var "
        "(must be passed at runtime instead):\n" + "\n".join(offenders)
    )
