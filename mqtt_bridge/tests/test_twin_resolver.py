"""Tests for the canonical twin UUID resolver (SDK-primary)."""

from __future__ import annotations

import pytest

from mqtt_bridge.twin_resolver import (
    TwinResolver,
    first_uuid_env,
    is_valid_uuid,
)

UUID_A = "8cc6f516-3730-4ae0-b038-a0e1f0e86de3"
UUID_B = "00000000-0000-0000-0000-000000000001"


class _Obj:
    def __init__(self, twin_uuid=None):
        self.twin_uuid = twin_uuid


def test_is_valid_uuid():
    assert is_valid_uuid(UUID_A)
    assert not is_valid_uuid("not-a-uuid")
    assert not is_valid_uuid("")
    assert not is_valid_uuid("ugv_beast_8cc6f5")


def test_first_uuid_env_picks_first_valid():
    assert first_uuid_env(f"junk, {UUID_A}, {UUID_B}") == UUID_A
    assert first_uuid_env("") is None
    assert first_uuid_env("junk,also-junk") is None


def test_env_wins_over_edge_and_mapping(monkeypatch):
    monkeypatch.setenv("CYBERWAVE_TWIN_UUID", UUID_A)
    r = TwinResolver(edge_config=_Obj(UUID_B), mapping=_Obj(UUID_B))
    assert r.resolve() == UUID_A


def test_edge_config_wins_over_mapping_when_env_unset(monkeypatch):
    monkeypatch.delenv("CYBERWAVE_TWIN_UUID", raising=False)
    r = TwinResolver(edge_config=_Obj(UUID_A), mapping=_Obj(UUID_B))
    assert r.resolve() == UUID_A


def test_mapping_fallback_when_env_and_edge_unset(monkeypatch):
    monkeypatch.delenv("CYBERWAVE_TWIN_UUID", raising=False)
    r = TwinResolver(edge_config=_Obj(None), mapping=_Obj(UUID_B))
    assert r.resolve() == UUID_B


def test_invalid_values_are_ignored(monkeypatch):
    monkeypatch.setenv("CYBERWAVE_TWIN_UUID", "not-a-uuid")
    r = TwinResolver(edge_config=_Obj("also-bad"), mapping=_Obj(UUID_B))
    assert r.resolve() == UUID_B  # falls through to the only valid source


def test_returns_none_and_warns_once_when_nothing_resolves(monkeypatch):
    monkeypatch.delenv("CYBERWAVE_TWIN_UUID", raising=False)

    class _Logger:
        def __init__(self):
            self.warnings = 0

        def warning(self, *_a, **_k):
            self.warnings += 1

    log = _Logger()
    r = TwinResolver(edge_config=_Obj(None), mapping=_Obj(None), logger=log)
    assert r.resolve() is None
    assert r.resolve() is None
    assert log.warnings == 1  # warned once, not per call


def test_cache_and_set_mapping_clears_it(monkeypatch):
    monkeypatch.delenv("CYBERWAVE_TWIN_UUID", raising=False)
    r = TwinResolver(mapping=_Obj(UUID_A))
    assert r.resolve() == UUID_A
    r.set_mapping(_Obj(UUID_B))  # clears cache
    assert r.resolve() == UUID_B
