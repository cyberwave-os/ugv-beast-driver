"""Guard: docker-conf/Dockerfile must stay lean + multi-stage (ros-base builder →
ros-core runtime), never the heavy pinned vendor base. Regressed more than once."""

from __future__ import annotations

import re
from pathlib import Path

DOCKERFILE = Path(__file__).resolve().parents[2] / "docker-conf" / "Dockerfile"

_STAGE_RE = re.compile(r"^FROM\s+\S+\s+AS\s+(\S+)", re.MULTILINE)


def test_dockerfile_present() -> None:
    assert DOCKERFILE.is_file(), f"missing {DOCKERFILE}"


def test_dockerfile_is_multistage() -> None:
    """At least two named stages (builder + runtime)."""
    stages = _STAGE_RE.findall(DOCKERFILE.read_text())
    assert len(stages) >= 2, (
        f"docker-conf/Dockerfile must be multi-stage (found stages: {stages}). "
        "It regressed to a single-stage build before — keep the ros-base builder "
        "→ ros-core runtime split so the toolchain never ships in the runtime."
    )


def test_dockerfile_uses_lean_bases_not_vendor() -> None:
    """Builder on ros-base, runtime on ros-core, and NOT the heavy vendor base."""
    text = DOCKERFILE.read_text()
    assert "dudulrx0601" not in text, (
        "docker-conf/Dockerfile uses the pinned, unmaintained vendor base "
        "dudulrx0601/ugv_rpi_ros_humble. Use the lean ros-base builder → "
        "ros-core runtime instead (upstream, toolchain-free runtime)."
    )
    assert re.search(r"^FROM\s+ros:\S*ros-base\s+AS\s+\S+", text, re.MULTILINE), (
        "expected a builder stage: FROM ros:<distro>-ros-base AS <stage>"
    )
    assert re.search(r"^FROM\s+ros:\S*ros-core\s+AS\s+runtime", text, re.MULTILINE), (
        "expected the runtime stage: FROM ros:<distro>-ros-core AS runtime"
    )
