from __future__ import annotations

import json
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from rclpy.node import Node
from rclpy.publisher import Publisher


class CommandHandler(ABC):
    """Base class for MQTT command handlers used by the UGV plugin."""

    def __init__(self, node: Node):
        self.node = node
        self.logger = node.get_logger()
        self._publishers: Dict[str, Publisher] = {}
        self._mqtt_adapter: Any = None
        self._command_topic: Optional[str] = None
        self._setup_publishers()

    def set_mqtt_context(self, adapter: Any, command_topic: str):
        self._mqtt_adapter = adapter
        self._command_topic = command_topic

    def _get_status_topic(self) -> Optional[str]:
        try:
            twin_uuid = None
            if hasattr(self.node, "_mapping") and self.node._mapping:
                twin_uuid = getattr(self.node._mapping, "twin_uuid", None)
            if not twin_uuid:
                return None
            prefix = getattr(self.node, "ros_prefix", "")
            return f"{prefix}cyberwave/twin/{twin_uuid}/{self.get_command_name()}/status"
        except Exception:
            return None

    def _resolve_ros_topic(self, topic: str) -> str:
        resolver = getattr(self.node, "resolve_ros_topic", None)
        if callable(resolver):
            try:
                return resolver(topic)
            except Exception:
                pass
        return topic

    def create_ros_publisher(
        self, msg_type: type, topic: str, qos: int
    ) -> Publisher:
        resolved_topic = self._resolve_ros_topic(topic)
        return self.node.create_publisher(msg_type, resolved_topic, qos)

    def publish_response(self, response_data: Dict[str, Any]) -> bool:
        if not self._mqtt_adapter:
            self.logger.warning("Cannot publish response: MQTT adapter not set")
            return False
        status_topic = self._get_status_topic()
        if not status_topic:
            self.logger.warning("Cannot publish response: twin_uuid not available")
            return False
        try:
            payload = {
                "command": self.get_command_name(),
                "type": "response",
                "source_type": "edge",
                "timestamp": self.node.get_clock().now().nanoseconds / 1e9,
                "data": response_data,
            }
            self._mqtt_adapter.publish(status_topic, json.dumps(payload))
            self.logger.debug(f"Published response to {status_topic}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish response: {e}")
            return False

    def publish_simple_response(self, response_data: Dict[str, Any]) -> bool:
        if not self._mqtt_adapter:
            self.logger.warning("Cannot publish simple response: MQTT adapter not set")
            return False
        status_topic = self._get_status_topic()
        if not status_topic:
            self.logger.warning(
                "Cannot publish simple response: twin_uuid not available"
            )
            return False
        try:
            response_data["source_type"] = "edge"
            response_data["timestamp"] = self.node.get_clock().now().nanoseconds / 1e9
            self._mqtt_adapter.publish(status_topic, json.dumps(response_data))
            self.logger.debug(f"Published simple response to {status_topic}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to publish simple response: {e}")
            return False

    @abstractmethod
    def _setup_publishers(self) -> None:
        pass

    @abstractmethod
    def handle(self, data: Dict[str, Any]) -> bool:
        pass

    @abstractmethod
    def get_command_name(self) -> str:
        pass

    def validate_data(self, data: Dict[str, Any], required_fields: list) -> bool:
        missing = [field for field in required_fields if field not in data]
        if missing:
            self.logger.warning(
                f"Command '{self.get_command_name()}' missing fields: {missing}"
            )
            return False
        return True
