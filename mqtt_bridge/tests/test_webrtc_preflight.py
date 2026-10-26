"""Step 7 gate (offline): STUN/TURN preflight parsing + failure handling."""

from __future__ import annotations

import socket
import struct

from mqtt_bridge.plugins import webrtc_preflight as pf


class _FakeSock:
    def __init__(self, response: bytes = None, raise_exc: Exception = None):
        self._response = response
        self._raise = raise_exc
        self.sent = []

    def settimeout(self, t):
        pass

    def sendto(self, data, addr):
        self.sent.append((data, addr))
        self._last_txid = data[8:20]

    def recvfrom(self, n):
        if self._raise:
            raise self._raise
        return self._response, ("1.2.3.4", 3478)

    def close(self):
        pass


def _success_response(txid: bytes) -> bytes:
    # STUN Binding success header + XOR-MAPPED-ADDRESS attribute.
    header = struct.pack(">HHI", 0x0101, 12, pf._STUN_MAGIC_COOKIE) + txid
    # attr: type 0x0020, len 8, family 0x01, xor-port, xor-ip
    xport = struct.pack(">H", 1234 ^ (pf._STUN_MAGIC_COOKIE >> 16))
    xip = struct.pack(">I", 0x01020304 ^ pf._STUN_MAGIC_COOKIE)
    attr = struct.pack(">HH", 0x0020, 8) + b"\x00\x01" + xport + xip
    return header + attr


def test_stun_success_parses_mapped_address(monkeypatch):
    holder = {}

    def fake_socket(*a, **k):
        s = _FakeSock()
        # response txid must match the request's; capture it via sendto
        orig_sendto = s.sendto

        def sendto(data, addr):
            orig_sendto(data, addr)
            s._response = _success_response(data[8:20])

        s.sendto = sendto
        holder["s"] = s
        return s

    monkeypatch.setattr(socket, "socket", fake_socket)
    result = pf.stun_binding_check("turn.example.com", 3478, timeout=1.0)
    assert result["reachable"] is True
    assert result["binding_success"] is True
    assert result["mapped"] == "1.2.3.4:1234"


def test_stun_timeout_reports_unreachable(monkeypatch):
    monkeypatch.setattr(
        socket, "socket", lambda *a, **k: _FakeSock(raise_exc=socket.timeout("timed out"))
    )
    result = pf.stun_binding_check("10.255.255.1", 3478, timeout=0.5)
    assert result["reachable"] is False
    assert "error" in result


def test_preflight_picks_turn_url(monkeypatch):
    seen = {}

    def fake_check(host, port, timeout):
        seen["host"], seen["port"] = host, port
        return {"reachable": True}

    monkeypatch.setattr(pf, "stun_binding_check", fake_check)
    servers = [
        {"urls": ["stun:stun.l.google.com:19302"]},
        {"urls": "turn:turn.cyberwave.com:3478", "username": "u", "credential": "c"},
    ]
    out = pf.preflight_turn(servers, timeout=2.0)
    assert out["reachable"] is True
    # First STUN/TURN url is used.
    assert seen["host"] == "stun.l.google.com" and seen["port"] == 19302


def test_preflight_no_url():
    assert pf.preflight_turn([], timeout=1.0)["reachable"] is False
