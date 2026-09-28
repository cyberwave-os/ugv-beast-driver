from __future__ import annotations

import math
import time
import typing
import json

from rclpy.action import ActionClient
from rclpy.node import Node

try:
    from ..cyberwave_mqtt_adapter import (
        SOURCE_TYPE_EDGE,
        SOURCE_TYPE_TELE,
        SOURCE_TYPE_EDIT,
        SOURCE_TYPE_SIM,
        SOURCE_TYPE_SIM_TELE,
    )
except Exception:
    SOURCE_TYPE_EDGE = 'edge'
    SOURCE_TYPE_TELE = 'tele'
    SOURCE_TYPE_EDIT = 'edit'
    SOURCE_TYPE_SIM = 'sim'
    SOURCE_TYPE_SIM_TELE = 'sim_tele'


class NavigationBridge:
    def __init__(self, node: Node):
        self.node = node
        self._init_navigation()

    def _init_navigation(self) -> None:
        self.node.declare_parameter('navigation.frame_id', 'map')
        self.node.declare_parameter('navigation.source_type', SOURCE_TYPE_EDGE)
        self.node.declare_parameter('navigation.navigate_to_pose_action', 'navigate_to_pose')
        self.node.declare_parameter('navigation.follow_path_action', 'follow_path')
        self.node.declare_parameter('navigation.pose_topic', '')
        self.node.declare_parameter('navigation.pose_topic_type', 'odom')

        self._nav_goal_handles = {}
        self._nav_last_goals = {}
        self._nav_active_action_ids = {}
        self._nav_clients = {}
        self._nav_types = {}
        self._nav2_available = None
        self._nav_pose_last_publish_time = 0.0
        self._nav_frame_id = self.node.get_parameter('navigation.frame_id').value or 'map'
        self._nav_status_source_type = (
            self.node.get_parameter('navigation.source_type').value or SOURCE_TYPE_EDGE
        )
        self._nav_action_names = {
            'goto': self.node.get_parameter('navigation.navigate_to_pose_action').value or 'navigate_to_pose',
            'path': self.node.get_parameter('navigation.follow_path_action').value or 'follow_path',
        }

        self._setup_navigation_pose_subscription()
        self._register_navigation_callbacks()

    def _register_navigation_callbacks(self) -> None:
        mapping = getattr(self.node, '_mapping', None)
        if mapping is not None and hasattr(mapping, 'twin_uuid') and mapping.twin_uuid:
            twin_uuid = mapping.twin_uuid
            ros_prefix = getattr(self.node, 'ros_prefix', '')
            nav_command_topic = f'{ros_prefix}cyberwave/twin/{twin_uuid}/navigate/command'
            
            # Register with the node's MQTT callback system
            if not hasattr(self.node, '_mqtt_callbacks'):
                self.node._mqtt_callbacks = {}
            
            self.node._mqtt_callbacks.setdefault(nav_command_topic, []).append(
                self._on_navigation_command
            )
            self.node.get_logger().info(
                f"Registered navigation callback for topic: {nav_command_topic} (ros_prefix='{ros_prefix}')"
            )

    def _is_navigation_command_topic(self, topic: str) -> bool:
        return self._extract_nav_command_twin_uuid(topic) is not None

    def _extract_nav_command_twin_uuid(self, topic: str) -> typing.Optional[str]:
        marker = "cyberwave/twin/"
        suffix = "/navigate/command"
        idx = topic.find(marker)
        if idx == -1 or not topic.endswith(suffix):
            return None
        start = idx + len(marker)
        if start >= len(topic) - len(suffix):
            return None
        return topic[start : -len(suffix)]

    def _on_navigation_command(self, topic: str, payload, mqtt_msg) -> None:
        data = payload
        if isinstance(payload, str):
            try:
                data = json.loads(payload)
            except Exception:
                data = payload
        twin_uuid = self._extract_nav_command_twin_uuid(topic)
        if not twin_uuid:
            return
        self._handle_navigation_command(twin_uuid, data)

    def _handle_navigation_command(self, twin_uuid: str, data: typing.Any) -> None:
        if not isinstance(data, dict):
            self.node.get_logger().warning("Navigation command payload is not JSON")
            return

        source_type = data.get("source_type")
        if source_type and source_type != SOURCE_TYPE_TELE:
            self.node.get_logger().debug(
                f"Ignoring navigation command with source_type='{source_type}'"
            )
            return

        action_id = str(data.get("action_id") or f"nav-{int(time.time() * 1000)}")
        command = str(data.get("command") or "").strip().lower()
        frame_id = None
        metadata = data.get("metadata")
        if isinstance(metadata, dict):
            frame_id = metadata.get("frame_id")
        frame_id = frame_id or data.get("frame_id") or self._nav_frame_id

        if command == "goto":
            position = data.get("position")
            pos = self._normalize_position(position)
            if not pos:
                self._publish_nav_status(
                    twin_uuid, action_id, "failed", "Navigation goto requires position"
                )
                return
            goal = self._build_nav_goal(
                "goto",
                position=pos,
                rotation=data.get("rotation"),
                yaw=data.get("yaw"),
                frame_id=frame_id,
            )
            if goal is None:
                self._publish_nav_status(
                    twin_uuid, action_id, "failed", "Failed to build navigation goal"
                )
                return
            self._send_nav_goal(twin_uuid, action_id, "goto", goal)
            return

        if command == "path":
            waypoints = data.get("waypoints")
            if not isinstance(waypoints, list) or not waypoints:
                self._publish_nav_status(
                    twin_uuid, action_id, "failed", "Navigation path requires waypoints"
                )
                return
            goal = self._build_nav_goal(
                "path",
                waypoints=waypoints,
                rotation=data.get("rotation"),
                yaw=data.get("yaw"),
                frame_id=frame_id,
            )
            if goal is None:
                self._publish_nav_status(
                    twin_uuid, action_id, "failed", "Failed to build navigation path"
                )
                return
            self._send_nav_goal(twin_uuid, action_id, "path", goal)
            return

        if command in {"stop", "pause"}:
            reason = "Navigation stopped" if command == "stop" else "Navigation paused"
            self._cancel_nav_goal(twin_uuid, action_id, reason, keep_last=(command == "pause"))
            self._publish_nav_status(twin_uuid, action_id, "completed", reason)
            return

        if command == "resume":
            last_goal = self._nav_last_goals.get(twin_uuid)
            if not last_goal:
                self._publish_nav_status(
                    twin_uuid, action_id, "failed", "No navigation goal to resume"
                )
                return
            self._send_nav_goal(
                twin_uuid,
                action_id,
                last_goal.get("type"),
                last_goal.get("goal"),
                reuse_last=True,
            )
            return

        self._publish_nav_status(
            twin_uuid, action_id, "failed", f"Unsupported navigation command: {command}"
        )

    def _ensure_nav2_clients(self) -> bool:
        if self._nav2_available is False:
            return False
        if self._nav2_available is True:
            return True
        try:
            from nav2_msgs.action import NavigateToPose, FollowPath
            from geometry_msgs.msg import PoseStamped
            from nav_msgs.msg import Path
            from action_msgs.msg import GoalStatus
        except Exception as exc:
            self.node.get_logger().warning(f"Nav2 messages unavailable: {exc}")
            self._nav2_available = False
            return False

        self._nav_types = {
            "NavigateToPose": NavigateToPose,
            "FollowPath": FollowPath,
            "PoseStamped": PoseStamped,
            "Path": Path,
            "GoalStatus": GoalStatus,
        }
        self._nav_clients["goto"] = ActionClient(
            self.node, NavigateToPose, self._nav_action_names["goto"]
        )
        self._nav_clients["path"] = ActionClient(
            self.node, FollowPath, self._nav_action_names["path"]
        )
        self._nav2_available = True
        return True

    def _build_nav_goal(
        self,
        goal_type: str,
        *,
        position: typing.Optional[typing.Dict[str, float]] = None,
        waypoints: typing.Optional[typing.List[typing.Any]] = None,
        rotation: typing.Any = None,
        yaw: typing.Any = None,
        frame_id: typing.Optional[str] = None,
    ) -> typing.Optional[typing.Any]:
        if not self._ensure_nav2_clients():
            return None
        PoseStamped = self._nav_types.get("PoseStamped")
        Path = self._nav_types.get("Path")
        if PoseStamped is None or Path is None:
            return None

        frame = frame_id or self._nav_frame_id

        if goal_type == "goto":
            if not position:
                return None
            pose = self._build_pose_stamped(
                position, rotation=rotation, yaw=yaw, frame_id=frame
            )
            NavigateToPose = self._nav_types.get("NavigateToPose")
            if NavigateToPose is None:
                return None
            goal = NavigateToPose.Goal()
            goal.pose = pose
            return goal

        if goal_type == "path":
            if not waypoints:
                return None
            path_msg = Path()
            path_msg.header.frame_id = frame
            path_msg.header.stamp = self.node.get_clock().now().to_msg()
            for waypoint in waypoints:
                pos = self._extract_waypoint_position(waypoint)
                if not pos:
                    continue
                waypoint_rotation = None
                waypoint_yaw = None
                if isinstance(waypoint, dict):
                    waypoint_rotation = waypoint.get("rotation")
                    waypoint_yaw = waypoint.get("yaw")
                pose = self._build_pose_stamped(
                    pos,
                    rotation=waypoint_rotation or rotation,
                    yaw=waypoint_yaw if waypoint_yaw is not None else yaw,
                    frame_id=frame,
                )
                path_msg.poses.append(pose)
            FollowPath = self._nav_types.get("FollowPath")
            if FollowPath is None:
                return None
            goal = FollowPath.Goal()
            goal.path = path_msg
            return goal

        return None

    def _send_nav_goal(
        self,
        twin_uuid: str,
        action_id: str,
        goal_type: str,
        goal: typing.Any,
        *,
        reuse_last: bool = False,
    ) -> None:
        if not self._ensure_nav2_clients():
            self._publish_nav_status(
                twin_uuid, action_id, "failed", "Nav2 action server unavailable"
            )
            return
        client = self._nav_clients.get(goal_type)
        if client is None:
            self._publish_nav_status(
                twin_uuid, action_id, "failed", "Navigation action client unavailable"
            )
            return
        if not client.wait_for_server(timeout_sec=2.0):
            self._publish_nav_status(
                twin_uuid, action_id, "failed", "Navigation action server not ready"
            )
            return

        if not reuse_last:
            self._nav_last_goals[twin_uuid] = {"type": goal_type, "goal": goal}
        self._nav_active_action_ids[twin_uuid] = action_id

        send_future = client.send_goal_async(goal)
        send_future.add_done_callback(
            lambda future: self._on_nav_goal_response(
                twin_uuid, action_id, goal_type, future
            )
        )

    def _on_nav_goal_response(
        self, twin_uuid: str, action_id: str, goal_type: str, future
    ) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_nav_status(
                twin_uuid, action_id, "failed", f"Navigation goal error: {exc}"
            )
            return

        if not getattr(goal_handle, "accepted", False):
            self._publish_nav_status(
                twin_uuid, action_id, "failed", "Navigation goal rejected"
            )
            return

        self._nav_goal_handles[twin_uuid] = goal_handle
        self._publish_nav_status(twin_uuid, action_id, "running", "Navigation running")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._on_nav_result(twin_uuid, action_id, fut)
        )

    def _on_nav_result(self, twin_uuid: str, action_id: str, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._publish_nav_status(
                twin_uuid, action_id, "failed", f"Navigation failed: {exc}"
            )
            return

        GoalStatus = self._nav_types.get("GoalStatus")
        status_code = getattr(result, "status", None)
        status = "failed"
        message = None
        if GoalStatus is not None:
            if status_code == GoalStatus.STATUS_SUCCEEDED:
                status = "completed"
            elif status_code == GoalStatus.STATUS_ABORTED:
                status = "failed"
            elif status_code == GoalStatus.STATUS_CANCELED:
                status = "cancelled"
            else:
                status = "failed"
                message = f"Navigation ended with status {status_code}"
        self._publish_nav_status(twin_uuid, action_id, status, message)
        if self._nav_active_action_ids.get(twin_uuid) == action_id:
            self._nav_active_action_ids.pop(twin_uuid, None)

    def _cancel_nav_goal(
        self,
        twin_uuid: str,
        action_id: str,
        reason: str,
        *,
        keep_last: bool = False,
    ) -> None:
        goal_handle = self._nav_goal_handles.get(twin_uuid)
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.node.get_logger().warning(f"Failed to cancel navigation goal: {exc}")
        active_action_id = self._nav_active_action_ids.get(twin_uuid)
        if active_action_id:
            self._publish_nav_status(twin_uuid, active_action_id, "cancelled", reason)
            self._nav_active_action_ids.pop(twin_uuid, None)
        if not keep_last:
            self._nav_last_goals.pop(twin_uuid, None)

    def _publish_nav_status(
        self,
        twin_uuid: str,
        action_id: str,
        status: str,
        message: typing.Optional[str] = None,
        progress: typing.Optional[float] = None,
    ) -> None:
        ros_prefix = getattr(self.node, 'ros_prefix', '')
        topic = f"{ros_prefix}cyberwave/twin/{twin_uuid}/navigate/status"
        payload = {
            "action_id": action_id,
            "status": status,
            "message": message,
            "progress": progress,
            "source_type": self._nav_status_source_type,
            "timestamp": time.time(),
        }
        payload = {k: v for k, v in payload.items() if v is not None}
        try:
            self.node.publish(topic, payload)
        except Exception as exc:
            self.node.get_logger().warning(f"Failed to publish nav status: {exc}")

    def _build_pose_stamped(
        self,
        position: typing.Dict[str, float],
        *,
        rotation: typing.Any = None,
        yaw: typing.Any = None,
        frame_id: typing.Optional[str] = None,
    ) -> typing.Any:
        PoseStamped = self._nav_types.get("PoseStamped")
        pose = PoseStamped()
        pose.header.frame_id = frame_id or self._nav_frame_id
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = position["x"]
        pose.pose.position.y = position["y"]
        pose.pose.position.z = position["z"]
        qw, qx, qy, qz = self._normalize_orientation(rotation=rotation, yaw=yaw)
        pose.pose.orientation.w = qw
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        return pose

    def _normalize_orientation(
        self, *, rotation: typing.Any = None, yaw: typing.Any = None
    ) -> typing.Tuple[float, float, float, float]:
        if rotation is not None:
            if isinstance(rotation, (list, tuple)) and len(rotation) == 4:
                values = [float(v) for v in rotation]
                return (values[0], values[1], values[2], values[3])
            if isinstance(rotation, dict) and all(k in rotation for k in ("w", "x", "y", "z")):
                return (
                    float(rotation.get("w")),
                    float(rotation.get("x")),
                    float(rotation.get("y")),
                    float(rotation.get("z")),
                )
        if yaw is not None:
            return self._yaw_to_quaternion(float(yaw))
        return (1.0, 0.0, 0.0, 0.0)

    def _yaw_to_quaternion(self, yaw: float) -> typing.Tuple[float, float, float, float]:
        half = yaw * 0.5
        return (math.cos(half), 0.0, 0.0, math.sin(half))

    def _normalize_position(self, position: typing.Any) -> typing.Optional[typing.Dict[str, float]]:
        if position is None:
            return None
        if isinstance(position, (list, tuple)) and len(position) >= 3:
            return {"x": float(position[0]), "y": float(position[1]), "z": float(position[2])}
        if isinstance(position, dict):
            if "position" in position:
                return self._normalize_position(position.get("position"))
            if all(k in position for k in ("x", "y", "z")):
                return {
                    "x": float(position.get("x")),
                    "y": float(position.get("y")),
                    "z": float(position.get("z")),
                }
        return None

    def _extract_waypoint_position(
        self, waypoint: typing.Any
    ) -> typing.Optional[typing.Dict[str, float]]:
        if isinstance(waypoint, dict) and "position" in waypoint:
            return self._normalize_position(waypoint.get("position"))
        return self._normalize_position(waypoint)

    def _setup_navigation_pose_subscription(self) -> None:
        pose_topic = self.node.get_parameter('navigation.pose_topic').value
        if not pose_topic:
            return
        mapping = getattr(self.node, '_mapping', None)
        twin_uuid = getattr(mapping, 'twin_uuid', None) if mapping is not None else None
        if not twin_uuid:
            self.node.get_logger().warning(
                "navigation.pose_topic set but mapping has no twin_uuid; skipping pose bridge"
            )
            return

        pose_type = (self.node.get_parameter('navigation.pose_topic_type').value or 'odom').strip().lower()
        msg_cls = None
        callback = None
        try:
            if pose_type in ("odom", "odometry"):
                from nav_msgs.msg import Odometry
                msg_cls = Odometry
                callback = self._handle_nav_pose_odometry
            elif pose_type in ("pose_with_covariance", "pose_with_covariance_stamped", "amcl_pose"):
                from geometry_msgs.msg import PoseWithCovarianceStamped
                msg_cls = PoseWithCovarianceStamped
                callback = self._handle_nav_pose_with_covariance
            elif pose_type in ("pose_stamped", "pose"):
                from geometry_msgs.msg import PoseStamped
                msg_cls = PoseStamped
                callback = self._handle_nav_pose_stamped
        except Exception as exc:
            self.node.get_logger().warning(f"Navigation pose bridge unavailable: {exc}")
            return

        if msg_cls is None or callback is None:
            self.node.get_logger().warning(
                f"Unsupported navigation.pose_topic_type '{pose_type}', skipping pose bridge"
            )
            return

        self._nav_pose_twin_uuid = twin_uuid
        self._nav_pose_source_type = self._nav_status_source_type
        # Resolve through the per-robot ROS namespace so the pose subscription
        # matches the namespaced hardware topic in a fleet deployment.
        resolve = getattr(self.node, "resolve_ros_topic", None)
        pose_topic = resolve(pose_topic) if callable(resolve) else pose_topic
        self._nav_pose_topic = pose_topic
        self._nav_pose_subscription = self.node.create_subscription(
            msg_cls, pose_topic, callback, 10
        )
        ros_prefix = getattr(self.node, 'ros_prefix', '')
        self.node.get_logger().info(
            f"Navigation pose bridge: {pose_topic} ({msg_cls.__name__}) -> "
            f"{ros_prefix}cyberwave/twin/{twin_uuid}/position"
        )

    def _publish_nav_pose_from_pose(self, pose) -> None:
        twin_uuid = getattr(self, "_nav_pose_twin_uuid", None)
        if not twin_uuid:
            return
        
        ros2mqtt_rate_interval = getattr(self.node, '_ros2mqtt_rate_interval', 1.0)
        if ros2mqtt_rate_interval > 0:
            now = time.time()
            if now - self._nav_pose_last_publish_time < ros2mqtt_rate_interval:
                return
            self._nav_pose_last_publish_time = now

        # Standard Cyberwave SDK Pose format:
        # position: [x, y, z]
        # rotation: {w, x, y, z} or roll/pitch/yaw
        
        position = [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
        ]
        rotation = {
            "w": float(pose.orientation.w),
            "x": float(pose.orientation.x),
            "y": float(pose.orientation.y),
            "z": float(pose.orientation.z),
        }
        timestamp = time.time()
        source_type = self._nav_pose_source_type
        
        # Standard Cyberwave position/rotation topics
        position_payload = {
            "source_type": source_type,
            "position": position,
            "ts": timestamp,
        }
        rotation_payload = {
            "source_type": source_type,
            "rotation": rotation,
            "ts": timestamp,
        }
        
        # Also provide a consolidated "update" payload for efficiency (Go2 style)
        update_payload = {
            "source_type": source_type,
            "type": "update",
            "position": {"x": position[0], "y": position[1], "z": position[2]},
            "rotation": rotation,
            "ts": timestamp
        }

        ros_prefix = getattr(self.node, 'ros_prefix', '')
        try:
            # Publish to individual topics for SDK compatibility
            self.node.publish(
                f"{ros_prefix}cyberwave/twin/{twin_uuid}/position",
                position_payload,
            )
            self.node.publish(
                f"{ros_prefix}cyberwave/twin/{twin_uuid}/rotation",
                rotation_payload,
            )
            # Publish to consolidated update topic for frontend efficiency
            self.node.publish(
                f"{ros_prefix}cyberwave/pose/{twin_uuid}/update",
                update_payload,
            )
        except Exception as exc:
            self.node.get_logger().warning(f"Failed to publish navigation pose: {exc}")

    def _handle_nav_pose_odometry(self, msg) -> None:
        try:
            self._publish_nav_pose_from_pose(msg.pose.pose)
        except Exception as exc:
            self.node.get_logger().warning(f"Failed to handle odometry pose: {exc}")

    def _handle_nav_pose_with_covariance(self, msg) -> None:
        try:
            self._publish_nav_pose_from_pose(msg.pose.pose)
        except Exception as exc:
            self.node.get_logger().warning(f"Failed to handle pose covariance: {exc}")

    def _handle_nav_pose_stamped(self, msg) -> None:
        try:
            self._publish_nav_pose_from_pose(msg.pose)
        except Exception as exc:
            self.node.get_logger().warning(f"Failed to handle pose stamped: {exc}")
