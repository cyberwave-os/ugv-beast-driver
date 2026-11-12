"""Adapter around CyberwaveMQTTClient to provide a small publish/subscribe API
compatible with the existing mqtt_bridge node.

This adapter is optional — if the Cyberwave SDK is not available the node
continues to use paho-mqtt directly.
"""

from __future__ import annotations

import logging
import json
import asyncio
import os
from pathlib import Path
from types import SimpleNamespace
import threading
import time
import inspect
from typing import Callable, Optional, Any, Dict
from .logger_shim import LoggerShim

import importlib.metadata as ilmd
from importlib.metadata import version, PackageNotFoundError

# Prefer the public, high-level factory `Cyberwave` and do not depend on the
# lower-level `CyberwaveMQTTClient` implementation. If the SDK isn't
# available the adapter should raise early so callers can fall back.
try:
    from cyberwave import Cyberwave

    # Import source types with fallbacks for SDK version compatibility
    try:
        from cyberwave import SOURCE_TYPE_EDGE
    except ImportError:
        SOURCE_TYPE_EDGE = "edge"
    try:
        from cyberwave import SOURCE_TYPE_TELE
    except ImportError:
        SOURCE_TYPE_TELE = "tele"
    try:
        from cyberwave import SOURCE_TYPE_EDIT
    except ImportError:
        SOURCE_TYPE_EDIT = "edit"
    try:
        from cyberwave import SOURCE_TYPE_SIM
    except ImportError:
        SOURCE_TYPE_SIM = "sim"
    try:
        from cyberwave import SOURCE_TYPE_SIM_TELE
    except ImportError:
        SOURCE_TYPE_SIM_TELE = "sim_tele"
    # These may not exist in older SDK versions (< 0.3.24)
    try:
        from cyberwave import SOURCE_TYPE_EDGE_LEADER
    except ImportError:
        SOURCE_TYPE_EDGE_LEADER = "edge_leader"
    try:
        from cyberwave import SOURCE_TYPE_EDGE_FOLLOWER
    except ImportError:
        SOURCE_TYPE_EDGE_FOLLOWER = "edge_follower"
except ImportError:  # pragma: no cover - optional dependency
    Cyberwave = None  # type: ignore
    SOURCE_TYPE_EDGE = "edge"
    SOURCE_TYPE_TELE = "tele"
    SOURCE_TYPE_EDIT = "edit"
    SOURCE_TYPE_SIM = "sim"
    SOURCE_TYPE_SIM_TELE = "sim_tele"
    SOURCE_TYPE_EDGE_LEADER = "edge_leader"
    SOURCE_TYPE_EDGE_FOLLOWER = "edge_follower"


class CyberwaveAdapter:
    def __init__(
        self,
        broker: str = "mqtt.cyberwave.com",
        port: int = 8883,  # TLS auto-enabled for port 8883
        api_token: Optional[str] = None,
        base_url: Optional[str] = None,
        mqtt_use_tls: Optional[bool] = None,
        topic_prefix: str = "",
        auto_connect: bool = True,
        logger: Optional[Any] = None,
        loop: Optional[asyncio.AbstractEventLoop] = None,
    ) -> None:
        # Use the provided logger (ROS node logger) when available so logs
        # are visible in the ROS2 logging pipeline; otherwise fall back to
        # a module-level logger. The Cyberwave SDK uses the standard
        # logging.Logger API (logger.info(msg, *args)), but rclpy's
        # RcutilsLogger.info does not accept formatting args. Wrap the
        # provided logger with a small shim that accepts (msg, *args)
        # and forwards a single formatted string to the underlying logger.
        orig_logger = logger or logging.getLogger(__name__)

        # Use a lightweight logger shim to adapt formatting-style calls to the
        # underlying logger implementation (for example rclpy's RcutilsLogger
        # which doesn't accept formatting args). The implementation lives in
        # `mqtt_bridge.logger_shim.LoggerShim` to keep this module focused on
        # MQTT adapter logic.
        self._logger = LoggerShim(orig_logger)
        self._loop = loop

        try:
            v = version("Cyberwave")
            self._logger.info(f"Cyberwave package version: {v}")
        except PackageNotFoundError:
            v = None  # not installed
            self._logger.info("Cyberwave package version: unknown")

        # The adapter now relies solely on the high-level Cyberwave factory
        # (`from cyberwave import Cyberwave`). If that factory isn't present
        # we fail early so callers (the node) can fall back to paho-mqtt.
        if Cyberwave is None:
            raise RuntimeError("Cyberwave SDK is not installed")

        # Initialize the high-level factory. The SDK requires a valid API token.
        # The token should be provided via the api_token parameter (from ROS params
        # or CYBERWAVE_API_KEY / CYBERWAVE_TOKEN environment variable).
        try:
            if not api_token:
                raise RuntimeError(
                    "Cyberwave SDK requires an API token. Set 'cyberwave_token' in params.yaml "
                    "or export CYBERWAVE_API_KEY environment variable."
                )

            self._logger.info(
                f"Initializing Cyberwave client for broker {broker}:{port}"
            )
            self._logger.info(f"Using API token: {api_token[:8]}...")
            self._logger.info(f"Using topic_prefix: '{topic_prefix or ''}'")

            effective_tls = (
                mqtt_use_tls if mqtt_use_tls is not None else int(port) == 8883
            )
            effective_base_url = base_url or os.getenv("CYBERWAVE_BASE_URL")

            # Initialize the Cyberwave SDK client with topic_prefix
            # The SDK will create an MQTT client internally and manage the connection
            cw = Cyberwave(
                api_key=api_token,
                base_url=effective_base_url,
                mqtt_host=broker,
                mqtt_port=port,
                mqtt_use_tls=effective_tls,
                topic_prefix=topic_prefix or "",
            )

            self._logger.info(f"Cyberwave client initialized successfully")
            self._logger.info(f"SDK mqtt.topic_prefix: '{cw.mqtt.topic_prefix}'")

            # store factory instance for passthroughs and keep topic prefix
            self._cw = cw
            self.topic_prefix = topic_prefix or ""

            # Try to initiate connection via the factory if it exposes a mqtt
            # object with connect(). This is best-effort; the SDK may manage
            # its own connection lifecycle.
            try:
                if hasattr(cw, "mqtt") and callable(getattr(cw.mqtt, "connect", None)):
                    self._logger.info("Calling cw.mqtt.connect()...")
                    cw.mqtt.connect()
                    self._logger.info("cw.mqtt.connect() completed")

                    # Wait for connection to establish (up to 5 seconds)
                    self._logger.info("Waiting for MQTT connection to establish...")
                    for i in range(50):  # 50 * 0.1s = 5 seconds max
                        if hasattr(cw.mqtt, "_client") and hasattr(
                            cw.mqtt._client, "connected"
                        ):
                            if cw.mqtt._client.connected:
                                self._logger.info(
                                    f"MQTT connection established after {i * 0.1:.1f}s"
                                )
                                break
                        time.sleep(0.1)
                    else:
                        self._logger.warning(
                            "MQTT connection not established after 5 seconds"
                        )

                elif hasattr(cw, "connect") and callable(getattr(cw, "connect", None)):
                    self._logger.info("Calling cw.connect()...")
                    cw.connect()
                    self._logger.info("cw.connect() completed")
                else:
                    self._logger.info(
                        "No explicit connect() method found, SDK may auto-connect"
                    )
            except Exception as e:
                self._logger.debug(
                    f"Cyberwave factory connect() raised during init: {e}"
                )

            # factory exposes .mqtt (preferred) or .mqtt_client
            client_obj = getattr(cw, "mqtt", None) or getattr(cw, "mqtt_client", None)
            if client_obj is None:
                raise RuntimeError("Cyberwave factory did not expose an MQTT client")

            # log client type and any connection-related attributes to aid
            # debugging when connections fail
            try:
                for attr in ("connected", "is_connected", "connection", "state"):
                    if hasattr(client_obj, attr):
                        try:
                            self._logger.info(
                                f"client.{attr} = {getattr(client_obj, attr)}"
                            )
                        except Exception:
                            self._logger.info(f"client has attribute {attr}")
            except Exception:
                pass

            # Prefer to use the underlying concrete MQTT client implementation
            # if the factory's client wrapper exposes it (common names are
            # 'client', '_client', 'mqtt_client' or 'mqttc'). This lets the
            # adapter call the real client's publish/subscribe API directly
            # (for example the paho client) which is generally stable.
            core_client = None
            for candidate in (
                "_client",
                "client",
                "mqtt_client",
                "mqttc",
                "_mqtt_client",
            ):
                try:
                    c = getattr(client_obj, candidate, None)
                except Exception:
                    c = None
                if c is not None and hasattr(c, "subscribe") and hasattr(c, "publish"):
                    core_client = c
                    break

            # If we didn't find a nested core, but the client_obj itself
            # implements publish/subscribe, just use it.
            if core_client is None:
                if hasattr(client_obj, "subscribe") and hasattr(client_obj, "publish"):
                    core_client = client_obj
                else:
                    # Last resort: leave the original wrapper and hope it
                    # forwards calls appropriately.
                    core_client = client_obj

            self._mqtt_client = core_client

            # Store reference to the SDK's high-level mqtt object for direct use
            # by components that need SDK-native methods (like BaseVideoStreamer)
            self._sdk_mqtt = getattr(cw, "mqtt", None)

            # pending subscriptions queued while client is not connected
            self._pending_subs = []  # list of (topic, handler, qos)
            # lock protecting access to _pending_subs
            self._pending_lock = threading.Lock()
            # background thread that flushes pending subscriptions when connected
            try:
                self._sub_flush_thread = threading.Thread(
                    target=self._sub_flush_loop, daemon=True
                )
                self._sub_flush_thread.start()
            except Exception:
                self._logger.debug("Could not start subscription flush thread")
        except Exception:
            self._logger.exception("Failed to initialize Cyberwave factory client")
            # re-raise so callers know initialization failed and can fallback
            raise

    @property
    def sdk_mqtt(self):
        """Return the SDK's native MQTT client object.

        This provides direct access to the Cyberwave SDK's mqtt interface,
        which has specialized methods like publish_webrtc_message(),
        subscribe_webrtc_messages(), etc. Use this for components that need
        SDK-native functionality (e.g., BaseVideoStreamer).
        """
        return getattr(self, "_sdk_mqtt", None)

    # DEAD CODE (disabled): cyberwave_factory — defined but never referenced anywhere.
    # @property
    # def cyberwave_factory(self):
    #     """Return the Cyberwave SDK factory instance.

    #     This provides access to cw.twin(), cw.video_stream(), etc.
    #     """
    #     return getattr(self, "_cw", None)

    @property
    def connected(self) -> bool:
        # Adapter should be tolerant to different underlying MQTT client
        # implementations. Check a few common attribute/method names used
        # by MQTT clients for connection state.
        if not getattr(self, "_mqtt_client", None):
            return False
        client = self._mqtt_client
        # common boolean attribute
        val = getattr(client, "connected", None)
        if isinstance(val, bool):
            return val
        # common boolean or callable is_connected
        is_conn = getattr(client, "is_connected", None)
        if callable(is_conn):
            try:
                return bool(is_conn())
            except Exception:
                pass
        if isinstance(is_conn, bool):
            return is_conn
        # some clients expose a 'state' or 'connection' attribute
        for attr in ("state", "connection"):
            v = getattr(client, attr, None)
            if isinstance(v, bool):
                return v
        return False

    # DEAD CODE (disabled): _discover_core_client — defined but never referenced anywhere.
    # def _discover_core_client(self, client_obj: Any) -> Any:
    #     """Find a concrete MQTT client under a factory wrapper.

    #     Many SDK factory wrappers expose an inner paho client under various
    #     attribute names. Return the first nested client that implements the
    #     basic publish/subscribe API, otherwise return the provided object.
    #     """
    #     for candidate in ("_client", "client", "mqtt_client", "mqttc", "_mqtt_client"):
    #         try:
    #             c = getattr(client_obj, candidate, None)
    #         except Exception:
    #             c = None
    #         if c is not None and hasattr(c, "subscribe") and hasattr(c, "publish"):
    #             self._logger.debug(
    #                 "Using underlying client via client_obj.%s", candidate
    #             )
    #             return c
    #     # Fallback: prefer client_obj if it itself has publish/subscribe
    #     if hasattr(client_obj, "subscribe") and hasattr(client_obj, "publish"):
    #         return client_obj
    #     return client_obj

    def _make_handler(self, on_message: Optional[Callable], topic: str) -> Callable:
        """Return a handler that normalizes SDK callback args into a
        SimpleNamespace(topic, payload) object and invokes on_message.

        Pulled out of the subscribe body so it can be unit-tested and reused
        from other places (for example during flush).
        """

        def handler(*args) -> None:
            # SDK may call handler(data) or handler(topic, data)
            try:
                if len(args) == 1:
                    topic_arg = topic
                    data = args[0]
                else:
                    topic_arg = args[0]
                    data = args[1]

                # data may be a dict or string; convert to bytes payload like paho
                if isinstance(data, (dict, list)):
                    b = json.dumps(data).encode("utf-8")
                elif isinstance(data, bytes):
                    b = data
                else:
                    b = str(data).encode("utf-8")
            except Exception:
                try:
                    b = str(args[-1]).encode("utf-8")
                except Exception:
                    b = b""

            fake_msg = SimpleNamespace(topic=topic_arg, payload=b)
            if on_message:
                try:
                    # Determine data for the callback
                    try:
                        data = json.loads(b.decode("utf-8"))
                    except Exception:
                        data = b.decode("utf-8")

                    # Helper to call the callback correctly
                    def invoke(cb, *args):
                        if inspect.iscoroutinefunction(cb):
                            # We need to run this in an event loop.
                            loop = getattr(self, "_loop", None)
                            if loop and loop.is_running():
                                asyncio.run_coroutine_threadsafe(cb(*args), loop)
                            else:
                                # Fallback: try to find any running loop
                                try:
                                    asyncio.run_coroutine_threadsafe(
                                        cb(*args), asyncio.get_event_loop()
                                    )
                                except Exception:
                                    pass
                        else:
                            cb(*args)

                    # Invoke the callback
                    try:
                        try:
                            sig = inspect.signature(on_message)
                            params = list(sig.parameters.values())
                            effective_count = len(params)
                        except Exception:
                            effective_count = -1

                        if effective_count >= 3:
                            invoke(on_message, topic_arg, data, fake_msg)
                        elif effective_count == 1:
                            invoke(on_message, data)
                        else:
                            try:
                                invoke(on_message, topic_arg, data, fake_msg)
                            except TypeError:
                                try:
                                    invoke(on_message, data)
                                except TypeError:
                                    invoke(on_message, fake_msg)
                    except Exception as e:
                        raise
                except Exception:
                    self._logger.exception(
                        f"Error in on_message handler for topic '{topic_arg}'. Payload snippet: {b[:100]!r}"
                    )

        return handler

    def publish(self, topic: str, payload, qos: int = 0) -> Any:
        """Publish a payload using the Cyberwave client.

        The SDK accepts dict or arbitrary payloads; preserve behaviour from
        the node: pass strings/bytes through, and dicts will be JSON-encoded
        by the SDK.
        """
        try:
            # If the payload is already a string or bytes, it means it's already
            # encoded for the wire. In this case, bypass the high-level SDK
            # validation which can be noisy for custom topics (like joint updates
            # used for odometry).
            if isinstance(payload, (str, bytes, bytearray)):
                return self._mqtt_client.publish(topic, payload, qos=qos)

            # SDK's publish expects either dict or string-like. Prefer
            # calling the high-level factory's mqtt.publish if available so
            # we don't reimplement payload handling for non-string types.
            try:
                if (
                    getattr(self, "_cw", None) is not None
                    and hasattr(self._cw, "mqtt")
                    and callable(getattr(self._cw.mqtt, "publish", None))
                ):
                    return self._cw.mqtt.publish(topic, payload, qos=qos)
            except Exception:
                # fall back to underlying client
                pass

            return self._mqtt_client.publish(topic, payload, qos=qos)
        except Exception as e:
            self._logger.exception("CyberwaveAdapter.publish failed: %s", e)
            raise

    def subscribe(
        self, topic: str, on_message: Optional[Callable] = None, qos: int = 1
    ) -> Any:
        """Subscribe and translate SDK handler signature into node callback.

        The SDK will call handlers with `data` (already JSON-decoded or string).
        We translate that into a simple object with `.topic` and `.payload` so
        existing node logic can be reused.
        """
        # Build an internal handler that normalizes the SDK's handler
        # signature into a SimpleNamespace(topic, payload) object.
        handler = self._make_handler(on_message, topic)

        # If client is not connected yet, queue the subscription to avoid
        # noisy 'not connected' warnings from the SDK. The background flush
        # thread will attempt subscriptions once connected. Store the
        # internal wrapper `handler` so the SDK receives a callable that
        # converts (topic,data) into a fake_msg with .topic/.payload.
        try:
            if not self.connected:
                try:
                    with self._pending_lock:
                        self._pending_subs.append((topic, handler, qos))
                except Exception:
                    # if the lock or append fails, fall back to unsynchronized append
                    self._pending_subs.append((topic, handler, qos))
                self._logger.debug("Queued subscription to %s until connected", topic)
                return None
        except Exception:
            # If connected check fails, proceed to attempt subscribe
            pass

        # Try preferred high-level SDK subscribe first (factory.mqtt.subscribe)
        try:
            if (
                getattr(self, "_cw", None) is not None
                and hasattr(self._cw, "mqtt")
                and callable(getattr(self._cw.mqtt, "subscribe", None))
            ):
                try:
                    # Always pass our internal wrapper `handler` which converts
                    # the SDK's handler signature into the node-friendly
                    # SimpleNamespace(topic, payload) object.
                    return self._cw.mqtt.subscribe(topic, handler, qos=qos)
                except TypeError:
                    # Try alternate signature without qos
                    return self._cw.mqtt.subscribe(topic, handler)

            # Fallback to underlying mqtt client (paho-like). For paho we
            # can attach a per-topic callback via message_callback_add if
            # available, then call subscribe(topic, qos).
            try:
                add_cb = getattr(self._mqtt_client, "message_callback_add", None)
                if callable(add_cb) and on_message is not None:
                    try:
                        add_cb(
                            topic,
                            lambda client, userdata, msg: handler(
                                msg.topic, msg.payload
                            ),
                        )
                    except Exception:
                        # ignore callback attachment failure
                        pass
                # subscribe (paho returns a (result, mid) tuple)
                try:
                    return self._mqtt_client.subscribe(topic, qos)
                except TypeError:
                    return self._mqtt_client.subscribe(topic)
            except Exception:
                # If subscribe failed entirely, propagate
                raise
        except Exception:
            self._logger.exception("CyberwaveAdapter.subscribe failed for %s", topic)
            raise

    def _sub_flush_loop(self) -> None:
        """Background thread that flushes queued subscriptions once connected."""
        while True:
            try:
                # copy and clear pending subscriptions under lock to avoid
                # races with subscribe(). We may attempt to flush only when
                # we have pending entries and the client is connected.
                pending = []
                try:
                    with self._pending_lock:
                        if self._pending_subs:
                            pending = list(self._pending_subs)
                            self._pending_subs.clear()
                except Exception:
                    # if locking fails for any reason, attempt a non-locked copy
                    pending = list(self._pending_subs)

                if pending and self.connected:
                    for topic, handler, qos in pending:
                        try:
                            self._logger.debug(
                                "Flushing queued subscription to %s", topic
                            )
                            if (
                                getattr(self, "_cw", None) is not None
                                and hasattr(self._cw, "mqtt")
                                and callable(getattr(self._cw.mqtt, "subscribe", None))
                            ):
                                try:
                                    self._cw.mqtt.subscribe(topic, handler, qos=qos)
                                except Exception:
                                    try:
                                        # try attach callback for underlying client
                                        add_cb = getattr(
                                            self._mqtt_client,
                                            "message_callback_add",
                                            None,
                                        )
                                        if callable(add_cb):
                                            try:
                                                add_cb(
                                                    topic,
                                                    lambda client,
                                                    userdata,
                                                    msg: handler(
                                                        msg.topic, msg.payload
                                                    ),
                                                )
                                            except Exception:
                                                pass
                                        self._mqtt_client.subscribe(topic, qos)
                                    except Exception:
                                        self._logger.debug(
                                            "Failed to subscribe to %s during flush",
                                            topic,
                                        )
                            else:
                                try:
                                    add_cb = getattr(
                                        self._mqtt_client, "message_callback_add", None
                                    )
                                    if callable(add_cb):
                                        try:
                                            add_cb(
                                                topic,
                                                lambda client, userdata, msg: handler(
                                                    msg.topic, msg.payload
                                                ),
                                            )
                                        except Exception:
                                            pass
                                    self._mqtt_client.subscribe(topic, qos)
                                except Exception:
                                    self._logger.debug(
                                        "Failed to subscribe to %s during flush", topic
                                    )
                        except Exception:
                            self._logger.exception(
                                "Error flushing subscription %s", topic
                            )
            except Exception:
                self._logger.debug("Subscription flush loop encountered an error")
            time.sleep(0.1)

    # High-level passthroughs that prefer SDK implementations when present
    def update_joint_state(
        self,
        twin_uuid: str,
        joint_name: str,
        position: Optional[float] = None,
        velocity: Optional[float] = None,
        effort: Optional[float] = None,
        source_type: Optional[str] = None,
    ) -> Any:
        """
        Prefer the SDK's update_joint_state when available, otherwise publish JSON.

        Args:
            twin_uuid: UUID of the twin
            joint_name: Name of the joint
            position: Joint position
            velocity: Joint velocity
            effort: Joint effort
            source_type: Source type (defaults to SOURCE_TYPE_EDGE for edge devices)
        """
        # Default to SOURCE_TYPE_EDGE for edge devices
        if source_type is None:
            source_type = SOURCE_TYPE_EDGE

        try:
            if (
                getattr(self, "_cw", None) is not None
                and hasattr(self._cw, "mqtt")
                and callable(getattr(self._cw.mqtt, "update_joint_state", None))
            ):
                return self._cw.mqtt.update_joint_state(
                    twin_uuid,
                    joint_name,
                    position=position,
                    velocity=velocity,
                    effort=effort,
                    source_type=source_type,
                )
        except Exception:
            self._logger.debug("SDK update_joint_state failed, falling back to publish")

        # Fallback: construct payload and publish to standard update topic
        joint_state = {}
        if position is not None:
            joint_state["position"] = position
        if velocity is not None:
            joint_state["velocity"] = velocity
        if effort is not None:
            joint_state["effort"] = effort

        topic = f"{self.topic_prefix}cyberwave/joint/{twin_uuid}/update"
        message = {
            "source_type": source_type,
            "type": "joint_state",
            "joint_name": joint_name,
            "joint_state": joint_state,
            "timestamp": time.time(),
        }
        return self.publish(topic, message)

    def update_joints_state(
        self,
        twin_uuid: str,
        joint_positions: Dict[str, float],
        source_type: Optional[str] = None,
    ) -> Any:
        """
        Prefer the SDK's update_joints_state when available, otherwise publish a flat dict.

        Args:
            twin_uuid: UUID of the twin
            joint_positions: Dict mapping joint names to positions
            source_type: Source type (defaults to SOURCE_TYPE_EDGE)
        """
        if source_type is None:
            source_type = SOURCE_TYPE_EDGE

        try:
            if (
                getattr(self, "_cw", None) is not None
                and hasattr(self._cw, "mqtt")
                and callable(getattr(self._cw.mqtt, "update_joints_state", None))
            ):
                return self._cw.mqtt.update_joints_state(
                    twin_uuid, joint_positions, source_type=source_type
                )
        except Exception:
            self._logger.debug(
                "SDK update_joints_state failed, falling back to publish"
            )

        # Fallback: construct payload and publish to standard update topic
        topic = f"{self.topic_prefix}cyberwave/joint/{twin_uuid}/update"
        message = {
            "source_type": source_type,
            **joint_positions,
            "timestamp": time.time(),
        }
        return self.publish(topic, message)

    def subscribe_twin_joint_states(
        self, twin_uuid: str, on_update: Optional[Callable] = None
    ) -> Any:
        topic = f"{getattr(self, 'topic_prefix', '')}cyberwave/joint/{twin_uuid}/update"
        return self.subscribe(topic, on_update)

    def subscribe_twin(
        self, twin_uuid: str, on_update: Optional[Callable] = None
    ) -> Any:
        topic = f"{getattr(self, 'topic_prefix', '')}cyberwave/twin/{twin_uuid}/+"
        return self.subscribe(topic, on_update)

    def publish_position(
        self,
        twin_uuid: str,
        position: Dict[str, float],
        rotation: Optional[Dict[str, float]] = None,
        source_type: Optional[str] = None,
    ) -> Any:
        """
        Publish twin position and optionally rotation (matches EdgeNode.publish_position() pattern).
        This is the recommended method following the Cyberwave SDK pattern.

        Args:
            twin_uuid: UUID of the twin
            position: Dict with keys 'x', 'y', 'z' (floats)
            rotation: Optional dict with keys 'w', 'x', 'y', 'z' (quaternion components as floats)
            source_type: Source type (defaults to SOURCE_TYPE_EDGE)
        """
        if source_type is None:
            source_type = SOURCE_TYPE_EDGE

        # Publish position
        try:
            if (
                getattr(self, "_cw", None) is not None
                and hasattr(self._cw, "mqtt")
                and callable(getattr(self._cw.mqtt, "update_twin_position", None))
            ):
                self._cw.mqtt.update_twin_position(twin_uuid, position)
            else:
                # Fallback: publish to position topic
                self._logger.debug(
                    f"SDK update_twin_position not available, using fallback publish"
                )
                topic = f"{self.topic_prefix}cyberwave/twin/{twin_uuid}/position"
                message = {
                    **position,
                    "timestamp": time.time(),
                    "source_type": source_type,
                }
                self.publish(topic, message)
        except Exception as e:
            self._logger.warning(
                f"publish_position failed for position: {e}", exc_info=True
            )

        # Publish rotation if provided
        if rotation is not None:
            try:
                if (
                    getattr(self, "_cw", None) is not None
                    and hasattr(self._cw, "mqtt")
                    and callable(getattr(self._cw.mqtt, "update_twin_rotation", None))
                ):
                    self._cw.mqtt.update_twin_rotation(twin_uuid, rotation)
                else:
                    # Fallback: publish to rotation topic
                    self._logger.debug(
                        f"SDK update_twin_rotation not available, using fallback publish"
                    )
                    topic = f"{self.topic_prefix}cyberwave/twin/{twin_uuid}/rotation"
                    message = {
                        **rotation,
                        "timestamp": time.time(),
                        "source_type": source_type,
                    }
                    self.publish(topic, message)
            except Exception as e:
                self._logger.warning(
                    f"publish_position failed for rotation: {e}", exc_info=True
                )

    def update_twin_position(self, twin_uuid: str, position: Dict[str, float]) -> Any:
        """
        Update twin position (x, y, z) using SDK when available.
        Prefer publish_position() for combined position+rotation updates.

        Args:
            twin_uuid: UUID of the twin
            position: Dict with keys 'x', 'y', 'z' (floats)
        """
        try:
            if (
                getattr(self, "_cw", None) is not None
                and hasattr(self._cw, "mqtt")
                and callable(getattr(self._cw.mqtt, "update_twin_position", None))
            ):
                return self._cw.mqtt.update_twin_position(twin_uuid, position)
        except Exception:
            self._logger.debug(
                "SDK update_twin_position failed, falling back to publish"
            )

        # Fallback: publish to position topic
        topic = f"{self.topic_prefix}cyberwave/twin/{twin_uuid}/position"
        message = {
            **position,
            "timestamp": time.time(),
        }
        return self.publish(topic, message)

    def update_twin_rotation(self, twin_uuid: str, rotation: Dict[str, float]) -> Any:
        """
        Update twin rotation (quaternion: w, x, y, z) using SDK when available.
        Prefer publish_position() for combined position+rotation updates.

        Args:
            twin_uuid: UUID of the twin
            rotation: Dict with keys 'w', 'x', 'y', 'z' (quaternion components as floats)
        """
        try:
            if (
                getattr(self, "_cw", None) is not None
                and hasattr(self._cw, "mqtt")
                and callable(getattr(self._cw.mqtt, "update_twin_rotation", None))
            ):
                return self._cw.mqtt.update_twin_rotation(twin_uuid, rotation)
        except Exception:
            self._logger.debug(
                "SDK update_twin_rotation failed, falling back to publish"
            )

        # Fallback: publish to rotation topic
        topic = f"{self.topic_prefix}cyberwave/twin/{twin_uuid}/rotation"
        message = {
            **rotation,
            "timestamp": time.time(),
        }
        return self.publish(topic, message)

    def twin(self, twin_uuid: str) -> Any:
        """Returns a Cyberwave SDK Twin object for high-level operations like streaming."""
        if self._cw:
            return self._cw.twin(twin_uuid)
        return None

    def disconnect(self) -> Any:
        try:
            return self._mqtt_client.disconnect()
        except Exception:
            self._logger.exception("Error disconnecting Cyberwave client")
