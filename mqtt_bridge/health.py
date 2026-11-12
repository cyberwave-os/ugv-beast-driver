import time
import json
import os
from typing import Any, Optional
from rclpy.node import Node

class HealthPublisher:
    def __init__(self, node: Node):
        self.node = node
        self._start_time = time.time()
        self._last_health_log_time = 0.0

    def publish_health_status(self):
        """
        Periodic health status update for the frontend.
        """
        # Global upstream kill-switch
        if getattr(self.node, '_disable_all_upstream', False):
            return

        if not hasattr(self.node, '_mapping') or not self.node._mapping or not self.node._mapping.twin_uuid:
            return

        try:
            mapping = self.node._mapping
            twin_uuid = mapping.twin_uuid
            
            # Resolve edge_id dynamically:
            # 1. Environment variable (MQTT_EDGE_ID)
            # 2. Mapping configuration (robot_id)
            # 3. Fallback to generic ID
            edge_id = os.getenv('MQTT_EDGE_ID') or \
                      getattr(mapping, 'robot_id', None) or \
                      "generic_robot_edge"

            prefix = getattr(self.node, 'ros_prefix', '')
            topic = f"{prefix}cyberwave/twin/{twin_uuid}/edge_health"
            
            # Build minimal health payload compatible with frontend UseEdgeHealth options
            payload = {
                "type": "edge_health",
                "timestamp": time.time(),
                "edge_id": edge_id,
                "twin_uuid": twin_uuid,
                "uptime_seconds": time.time() - self._start_time,
                "streams": {},
                "stream_count": 0,
                "healthy_streams": 0,
            }
            
            # If a streamer is active, add its info
            if hasattr(self.node, '_ros_streamer') and self.node._ros_streamer is not None:
                streamer = self.node._ros_streamer
                # Go2 uses "go2_camera" as ID, we use the image topic or a friendly name
                stream_id = getattr(streamer, 'image_topic', 'ugv_camera')
                
                # Calculate FPS if possible
                fps = getattr(streamer, 'fps', 0.0)
                frames_sent = 0
                if hasattr(streamer, 'streamer') and hasattr(streamer.streamer, 'frame_count'):
                    frames_sent = streamer.streamer.frame_count
                
                payload["streams"][stream_id] = {
                    "camera_id": stream_id,
                    "connection_state": "connected" if getattr(streamer, '_answer_received', False) else "connecting",
                    "ice_connection_state": "connected", # Simplified
                    "frames_sent": frames_sent,
                    "fps": round(fps, 2),
                    "is_healthy": True,
                    "is_stale": False
                }
                payload["stream_count"] = 1
                payload["healthy_streams"] = 1
            
            # Use the adapter if available (Cyberwave SDK), otherwise fall back to direct client
            if hasattr(self.node, '_mqtt_adapter') and self.node._mqtt_adapter:
                self.node._mqtt_adapter.publish(topic, payload)
            elif hasattr(self.node, '_mqtt_client') and self.node._mqtt_client:
                import json as json_module
                self.node._mqtt_client.publish(topic, json_module.dumps(payload))
            else:
                self.node.get_logger().error("No MQTT client available for health publishing")
                return
            
            # Log periodically to avoid spam (every 60s)
            now = time.time()
            if now - self._last_health_log_time > 60:
                self.node.get_logger().info(f"Published periodic health status to {topic}")
                self._last_health_log_time = now
                
        except Exception as e:
            self.node.get_logger().error(f"Error publishing health status: {e}")
