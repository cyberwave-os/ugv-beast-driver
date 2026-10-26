"""Canonical twin UUID resolution for the UGV bridge (env → edge_config → mapping)."""

from __future__ import annotations

import os
import re
from typing import Any, Optional

_UUID_RE = re.compile(
    r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$",
    re.IGNORECASE,
)


def is_valid_uuid(value: str) -> bool:
    """True if *value* is a canonical 8-4-4-4-12 UUID string."""
    return bool(_UUID_RE.match(str(value or "").strip()))


def first_uuid_env(value: str) -> Optional[str]:
    """Return the first valid UUID from a comma-separated env value, else None."""
    for part in str(value or "").split(","):
        part = part.strip()
        if part and is_valid_uuid(part):
            return part
    return None


class TwinResolver:
    """Resolve the twin UUID from env → edge_config → mapping, with caching."""

    def __init__(
        self,
        edge_config: Optional[Any] = None,
        mapping: Optional[Any] = None,
        logger: Optional[Any] = None,
    ) -> None:
        self._edge_config = edge_config
        self._mapping = mapping
        self._logger = logger
        self._warned_missing = False
        self._cached_uuid: Optional[str] = None

    def set_mapping(self, mapping: Any) -> None:
        self._mapping = mapping
        self._cached_uuid = None

    def resolve(self, use_cache: bool = True) -> Optional[str]:
        """Return the canonical twin UUID (or None), highest-priority source first."""
        if use_cache and self._cached_uuid:
            return self._cached_uuid

        env_uuid = first_uuid_env(os.environ.get("CYBERWAVE_TWIN_UUID", ""))
        if env_uuid:
            self._cached_uuid = env_uuid
            return env_uuid

        if self._edge_config is not None:
            edge_uuid = getattr(self._edge_config, "twin_uuid", None)
            if edge_uuid and is_valid_uuid(str(edge_uuid)):
                self._cached_uuid = str(edge_uuid)
                return self._cached_uuid

        if self._mapping is not None:
            mapping_uuid = getattr(self._mapping, "twin_uuid", None)
            if mapping_uuid and is_valid_uuid(str(mapping_uuid)):
                self._cached_uuid = str(mapping_uuid)
                return self._cached_uuid

        if not self._warned_missing and self._logger is not None:
            self._warned_missing = True
            self._logger.warning(
                "twin_uuid could not be resolved (CYBERWAVE_TWIN_UUID unset, no "
                "edge-core/mapping value). MQTT topics containing {twin_uuid} will "
                "not be published — export CYBERWAVE_TWIN_UUID=<uuid>."
            )
        return None
