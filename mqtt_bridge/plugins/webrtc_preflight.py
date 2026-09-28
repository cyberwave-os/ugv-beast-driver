"""WebRTC NAT/TURN reachability preflight.

Relay-only ICE (force_turn) only needs OUTBOUND reach to the TURN server (works
through Docker bridge NAT). If blocked, ICE has no candidates and the stream
silently fails — so send a real STUN Binding request (TURN answers STUN on the
same port) to log reachability at startup. Pure stdlib, no aiortc.
"""

from __future__ import annotations

import logging
import os
import socket
import struct
from typing import Any, Dict

logger = logging.getLogger(__name__)

_STUN_BINDING_REQUEST = 0x0001
_STUN_BINDING_SUCCESS = 0x0101
_STUN_MAGIC_COOKIE = 0x2112A442


def stun_binding_check(host: str, port: int = 3478, timeout: float = 3.0) -> Dict[str, Any]:
    """STUN Binding request to host:port (UDP). Returns {"reachable": bool, ...};
    on success includes the server-reflexive "mapped" address (proves NAT round-trip)."""
    txid = os.urandom(12)
    request = struct.pack(">HHI", _STUN_BINDING_REQUEST, 0, _STUN_MAGIC_COOKIE) + txid

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(request, (host, port))
        data, _addr = sock.recvfrom(2048)
    except (socket.timeout, socket.gaierror, OSError) as exc:
        return {"reachable": False, "host": host, "port": port, "error": str(exc)}
    finally:
        sock.close()

    if len(data) < 20:
        return {"reachable": False, "host": host, "port": port, "error": "short response"}
    msg_type, _length, magic = struct.unpack(">HHI", data[:8])
    if data[8:20] != txid:
        return {"reachable": False, "host": host, "port": port, "error": "txid mismatch"}
    return {
        "reachable": True,
        "host": host,
        "port": port,
        "binding_success": msg_type == _STUN_BINDING_SUCCESS,
        "mapped": _parse_xor_mapped_address(data, magic),
    }


def _parse_xor_mapped_address(data: bytes, magic: int):
    """Best-effort parse of the XOR-MAPPED-ADDRESS attribute (type 0x0020)."""
    try:
        offset = 20  # after the 20-byte header
        while offset + 4 <= len(data):
            attr_type, attr_len = struct.unpack(">HH", data[offset : offset + 4])
            value = data[offset + 4 : offset + 4 + attr_len]
            if attr_type == 0x0020 and len(value) >= 8:
                port = struct.unpack(">H", value[2:4])[0] ^ (magic >> 16)
                ip_int = struct.unpack(">I", value[4:8])[0] ^ magic
                ip = ".".join(str((ip_int >> s) & 0xFF) for s in (24, 16, 8, 0))
                return f"{ip}:{port}"
            offset += 4 + attr_len + ((4 - attr_len % 4) % 4)
    except Exception:
        pass
    return None


def preflight_turn(turn_servers, timeout: float = 3.0) -> Dict[str, Any]:
    """Check the first resolvable turn:/stun: URL in an aiortc-style server list."""
    for server in turn_servers or []:
        urls = server.get("urls") if isinstance(server, dict) else None
        if isinstance(urls, str):
            urls = [urls]
        for url in urls or []:
            if url.startswith(("turn:", "stun:")):
                hostport = url.split(":", 1)[1]
                host = hostport.split(":")[0].split("?")[0]
                parts = hostport.replace("?", ":").split(":")
                port = 3478
                if len(parts) > 1 and parts[1].isdigit():
                    port = int(parts[1])
                result = stun_binding_check(host, port, timeout)
                result["url"] = url
                return result
    return {"reachable": False, "error": "no turn/stun url found"}
