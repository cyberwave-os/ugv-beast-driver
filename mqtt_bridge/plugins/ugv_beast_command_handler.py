"""
Command Handler Framework for MQTT Bridge Command Router

This module provides a scalable command routing system that maps MQTT commands
to ROS topics without using if-else chains. New commands can be added by:
1. Creating a handler class (for complex logic)
2. Using @register_command decorator
3. Or adding to YAML config (for simple mappings)

Design Pattern: Command Pattern + Registry Pattern
"""

import base64
import time
import math
from typing import Any, Callable, Dict, Optional

from rclpy.node import Node

# Import common ROS message types
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cyberwave_edge_common.actuation.timed_stop import TimedStop
from cyberwave_edge_common.actuation.ugv_velocity import (
    LocomotionVelocityCommandError,
    UGVVelocityLimits,
    lateral_velocity_requested,
    load_ugv_velocity_limits,
    normalize_ugv_velocity_command,
)
from cyberwave_edge_common.locomotion_contracts import build_locomotion_velocity_command
from cyberwave_edge_common.ros.twist_mapper import publish_twist

from .command_handler_base import CommandHandler


_MOVEMENT_ACTUATIONS = frozenset(
    {"move_forward", "move_backward", "turn_left", "turn_right"}
)
_MOVEMENT_OPPOSITES = {
    "move_forward": "move_backward",
    "move_backward": "move_forward",
    "turn_left": "turn_right",
    "turn_right": "turn_left",
}


# Import OpenCV for image encoding
try:
    import cv2

    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False


class UGVBeastActuationHandler(CommandHandler):
    """
    UGV Beast handler for actuation-based commands from frontend.

    This handler maps simple actuation strings (e.g., "move_forward", "turn_left")
    to appropriate ROS messages, enabling universal command-based control for
    ANY robot using "control_mode": "command".

    Supported actuations:
    - Locomotion: move_forward, move_backward, turn_left, turn_right, stop
      Combined diagonals (W+D, W+A, S+D, S+A) merge active keys into one twist.
      Movement watchdog auto-stops after 500ms with no teleop commands.
    - Camera: camera_up, camera_down, camera_left, camera_right
    - Lights: chassis_light_toggle, camera_light_toggle, led_toggle
    - Utilities: take_photo, battery_check
    - Robot-specific: sit_down, stand_up, obstacle_avoidance_toggle

    Configuration:
    - Linear speed: Default 0.5 m/s (can be configured via ROS params)
    - Angular speed: Default 0.8 rad/s (can be configured via ROS params)
    """

    def __init__(self, node: Node):
        # UGV velocity limits are runtime configuration, with conservative
        # defaults for the stock tracked rover.
        self._velocity_limits: UGVVelocityLimits = load_ugv_velocity_limits()
        self._linear_speed = self._velocity_limits.default_linear_speed
        self._angular_speed = self._velocity_limits.default_angular_speed
        self._camera_step = 0.1  # radians per key press
        self._locomotion_stop = TimedStop(lambda: self._send_twist(0.0, 0.0))

        # Track light states for toggles
        self._chassis_light_on = False
        self._camera_light_on = False
        self._led_on = False

        # Camera servo current position
        self._camera_pan = 0.0
        self._camera_tilt = 0.0

        # Movement watchdog: auto-stop if no commands received
        self._last_movement_command_time = 0.0
        self._movement_timeout = 0.5  # 500ms (2x typical 100ms teleop send rate)
        self._watchdog_timer = None

        # Active movement keys for combined diagonal teleop (W+D, W+A, S+D, S+A)
        self._active_movements = {
            "move_forward": False,
            "move_backward": False,
            "turn_left": False,
            "turn_right": False,
        }

        super().__init__(node)
        self.logger.info(
            f"UGVBeastActuationHandler initialized: "
            f"linear_speed={self._linear_speed} m/s, "
            f"angular_speed={self._angular_speed} rad/s, "
            f"movement_timeout={self._movement_timeout}s"
        )
        self._start_movement_watchdog()
    
        self._actuation_dispatch = self._build_actuation_dispatch()

    def get_command_name(self) -> str:
        return "actuation"

    def _setup_publishers(self) -> None:
        """Create publishers for all possible actuation types."""
        # Movement
        self._publishers["cmd_vel"] = self.create_ros_publisher(Twist, "/cmd_vel", 10)

        # Lights (UGV Beast specific, but safe to create)
        self._publishers["led_ctrl"] = self.create_ros_publisher(
            Float32MultiArray, "/ugv/led_ctrl", 10
        )

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle generic actuation command.

        Expected data formats (from frontend):

        1. Simple commands (keyboard controller):
        {
            "command": "move_forward",
            "timestamp": 1706547890.123,
            "source_type": "tele"
        }

        2. Complex commands (camera/lights):
        {
            "command": "camera_light_toggle",
            "data": {"state": true},
            "timestamp": 1706547890.123,
            "source_type": "tele"
        }

        3. Explicit analog velocity (top-level ``velocity_command``):
        {
            "command": "move_forward",
            "source_type": "tele",
            "timestamp": 1706547890.123,
            "velocity_command": {
                "linear_x": 0.6,
                "linear_y": 0.0,
                "angular_z": 0.4,
                "duration_ms": 300
            }
        }
        """
        try:
            # Extract the actuation command
            actuation = data.get("command")
            if not actuation:
                self.logger.warning("Actuation handler requires 'command' field")
                return False

            self.logger.info(f"Processing actuation: {actuation}")

            # Analog velocity path. A message may carry an explicit
            # ``velocity_command`` object (linear_x/angular_z/duration_ms, with or
            # without the full locomotion contract). When present it is
            # authoritative: honor the explicit velocities regardless of the
            # discrete ``command`` string (which may still be a keyboard actuation
            # like "move_forward"). Backend locomotion policy execution and analog
            # teleop both use this shape; older callers instead set
            # command == "velocity_command"/"locomotion_velocity" with the same
            # top-level object. Both routes converge on the locomotion handler.
            if isinstance(data.get("velocity_command"), dict) or actuation in {
                "locomotion_velocity",
                "velocity_command",
            }:
                return self._send_locomotion_velocity(data)

            # Otherwise it is a discrete command-mode actuation; per-command data
            # (camera/lights/video params) lives under ``data``.
            return self._process_actuation(actuation, data.get("data", {}))

        except Exception as e:
            self.logger.error(f"Failed to handle actuation: {e}")
            return False

    def _process_actuation(self, actuation: str, data: Dict[str, Any] = None) -> bool:
        """Map actuation string to appropriate ROS message."""
        data = data or {}

        if actuation in _MOVEMENT_ACTUATIONS:
            self._last_movement_command_time = time.time()
            self._active_movements[actuation] = True
            self._active_movements[_MOVEMENT_OPPOSITES[actuation]] = False
            self._publish_combined_twist()
            return True

        if actuation == "stop":
            self._last_movement_command_time = 0.0
            for key in self._active_movements:
                self._active_movements[key] = False
            return self._stop_motion()

        handler = self._actuation_dispatch.get(actuation)
        if handler is None:
            self.logger.warning(
                f"Unknown actuation: {actuation}. "
                f"Add mapping in UGVBeastActuationHandler._build_actuation_dispatch()"
            )
            self.publish_response(
                {"status": "error", "message": f"Unknown actuation: {actuation}"}
            )
            return False
        return handler(data)

    def _handle_start_video(self, data: Dict[str, Any]) -> bool:
        try:
            if hasattr(self.node, "start_camera_stream"):
                kwargs = CameraCommandHandler._stream_kwargs(data)
                self.logger.info(f"Starting video stream ({kwargs or 'defaults'})")
                self.node.start_camera_stream(**kwargs)
                self.publish_simple_response({"status": "ok", "type": "video_started"})
                return True

            self.logger.error("start_video: node.start_camera_stream() not available")
            self.publish_simple_response(
                {"status": "error", "message": "Video streaming not supported"}
            )
            return False
        except Exception as e:
            self.logger.error(f"Failed to start video stream: {e}")
            self.publish_simple_response({"status": "error", "message": str(e)})
            return False

    def _handle_stop_video(self) -> bool:
        try:
            if hasattr(self.node, "stop_camera_stream"):
                self.logger.info("Stopping video stream")
                self.node.stop_camera_stream()
                self.publish_simple_response({"status": "ok", "type": "video_stopped"})
                return True

            self.logger.error("stop_video: node.stop_camera_stream() not available")
            self.publish_simple_response(
                {"status": "error", "message": "Video streaming not supported"}
            )
            return False
        except Exception as e:
            self.logger.error(f"Failed to stop video stream: {e}")
            self.publish_simple_response({"status": "error", "message": str(e)})
            return False

    def _build_actuation_dispatch(self) -> Dict[str, Callable[[Dict[str, Any]], bool]]:
        return {
            "locomotion_velocity": self._send_locomotion_velocity,
            "velocity_command": self._send_locomotion_velocity,
            "camera_up": lambda _d: self._send_camera_servo(
                tilt_delta=self._camera_step
            ),
            "camera_down": lambda _d: self._send_camera_servo(
                tilt_delta=-self._camera_step
            ),
            "camera_left": lambda _d: self._send_camera_servo(
                pan_delta=-self._camera_step
            ),
            "camera_right": lambda _d: self._send_camera_servo(
                pan_delta=self._camera_step
            ),
            "camera_default": lambda _d: self._send_camera_servo_reset(),
            "chassis_light_toggle": lambda _d: self._toggle_chassis_light(),
            "camera_light_toggle": lambda _d: self._toggle_camera_light(),
            "led_toggle": lambda _d: self._toggle_led_pair(),
            "take_photo": lambda _d: self._dispatch_to_registry("take_photo"),
            "battery_check": lambda _d: self._dispatch_to_registry("battery_check"),
            "sit_down": lambda _d: self._not_implemented("sit_down"),
            "stand_up": lambda _d: self._not_implemented("stand_up"),
            "obstacle_avoidance_toggle": lambda _d: self._not_implemented(
                "obstacle_avoidance_toggle"
            ),
            "start_video": self._handle_start_video,
            "stop_video": lambda _d: self._handle_stop_video(),
        }

    def _stop_motion(self) -> bool:
        self._locomotion_stop.cancel()
        return self._send_twist(0.0, 0.0)

    def _legacy_velocity_command(
        self,
        *,
        linear_x: float = 0.0,
        angular_z: float = 0.0,
    ) -> Dict[str, Any]:
        return {
            "velocity_command": build_locomotion_velocity_command(
                linear_x=linear_x,
                angular_z=angular_z,
                duration_ms=500,
                gait="walk",
                origin="teleop",
            ).to_payload()
        }

    def _toggle_chassis_light(self) -> bool:
        self._chassis_light_on = not self._chassis_light_on
        return self._send_led_ctrl(chassis=255 if self._chassis_light_on else 0)

    def _toggle_camera_light(self) -> bool:
        self._camera_light_on = not self._camera_light_on
        return self._send_led_ctrl(camera=255 if self._camera_light_on else 0)

    def _toggle_led_pair(self) -> bool:
        self._led_on = not self._led_on
        value = 255 if self._led_on else 0
        return self._send_led_ctrl(chassis=value, camera=value)

    def _dispatch_to_registry(self, command_name: str) -> bool:
        if hasattr(self.node, "_command_registry"):
            return self.node._command_registry.handle_command(command_name, {})
        self.logger.warning(f"{command_name}: command registry not available")
        return False

    def _not_implemented(self, actuation: str) -> bool:
        self.logger.info(f"{actuation} actuation (no ROS topic mapped yet)")
        self.publish_response({"status": "not_implemented", "actuation": actuation})
        return True

    def _send_twist(self, linear_x: float, angular_z: float) -> bool:
        """Send a Twist message for locomotion."""
        try:
            publish_twist(self._publishers["cmd_vel"], Twist, linear_x, angular_z)
            self.logger.debug(
                f"Published Twist: linear.x={linear_x}, angular.z={angular_z}"
            )

            # Send confirmation
            self.publish_response(
                {"status": "success", "linear_x": linear_x, "angular_z": angular_z}
            )
            return True

        except Exception as e:
            self.logger.error(f"Failed to send Twist: {e}")
            return False

    def _send_locomotion_velocity(self, data: Dict[str, Any]) -> bool:
        """Map Cyberwave locomotion.velocity_command.v1 to UGV /cmd_vel."""
        if not isinstance(data, dict):
            self.logger.warning("locomotion_velocity requires object data")
            return False

        try:
            velocity_command = normalize_ugv_velocity_command(
                data,
                self._velocity_limits,
            )
        except LocomotionVelocityCommandError as e:
            self.logger.warning(f"Ignoring invalid locomotion_velocity command: {e}")
            return False

        if velocity_command.is_stop:
            return self._stop_motion()

        self._locomotion_stop.cancel()
        if lateral_velocity_requested(data):
            self.logger.debug("Ignoring lateral velocity for tracked UGV Beast")
        sent = self._send_twist(
            velocity_command.linear_x,
            velocity_command.angular_z,
        )
        if sent:
            self._locomotion_stop.schedule_ms(velocity_command.duration_ms)
        return sent

    def _send_camera_servo(
        self, pan_delta: float = 0.0, tilt_delta: float = 0.0
    ) -> bool:
        """Send camera servo command via camera_servo handler."""
        try:
            # Delegate to CameraServoHandler via command registry
            if hasattr(self.node, "_command_registry"):
                servo_data = {}
                if pan_delta != 0.0:
                    servo_data["pan_delta"] = pan_delta
                if tilt_delta != 0.0:
                    servo_data["tilt_delta"] = tilt_delta

                return self.node._command_registry.handle_command(
                    "camera_servo", servo_data
                )

            self.logger.warning("camera_servo: command registry not available")
            return False

        except Exception as e:
            self.logger.error(f"Failed to send camera servo: {e}")
            return False

    def _send_camera_servo_reset(self) -> bool:
        """Reset camera servo to default position (0.0, 0.0)."""
        try:
            if hasattr(self.node, "_command_registry"):
                return self.node._command_registry.handle_command(
                    "camera_servo", {"pan": 0.0, "tilt": 0.0}
                )
            self.logger.warning("camera_servo: command registry not available")
            return False
        except Exception as e:
            self.logger.error(f"Failed to reset camera servo: {e}")
            return False

    def _send_led_ctrl(
        self, chassis: Optional[float] = None, camera: Optional[float] = None
    ) -> bool:
        """Send LED control command via lights handler."""
        try:
            # Delegate to LightsHandler via command registry
            if hasattr(self.node, "_command_registry"):
                led_data = {}
                if chassis is not None:
                    led_data["chassis_light"] = chassis
                if camera is not None:
                    led_data["camera_light"] = camera

                return self.node._command_registry.handle_command("lights", led_data)

            self.logger.warning("lights: command registry not available")
            return False

        except Exception as e:
            self.logger.error(f"Failed to send LED control: {e}")
            return False

    def _start_movement_watchdog(self) -> None:
        """Auto-stop if no movement commands arrive within the timeout window."""

        def watchdog_callback() -> None:
            if self._last_movement_command_time <= 0:
                return
            elapsed = time.time() - self._last_movement_command_time
            if elapsed > self._movement_timeout:
                self.logger.info(
                    f"Movement watchdog: no commands for {elapsed:.2f}s, sending STOP"
                )
                for key in self._active_movements:
                    self._active_movements[key] = False
                self._stop_motion()
                self._last_movement_command_time = 0.0

        self._watchdog_timer = self.node.create_timer(0.1, watchdog_callback)
        self.logger.info("Movement watchdog timer started (checks every 100ms)")

    def _publish_combined_twist(self) -> None:
        """Publish one twist from all currently active movement keys."""
        linear_x = 0.0
        angular_z = 0.0
        if self._active_movements["move_forward"]:
            linear_x += self._linear_speed
        if self._active_movements["move_backward"]:
            linear_x -= self._linear_speed
        if self._active_movements["turn_left"]:
            angular_z += self._angular_speed
        if self._active_movements["turn_right"]:
            angular_z -= self._angular_speed

        self.logger.debug(
            f"Combined twist: linear_x={linear_x:.2f}, angular_z={angular_z:.2f} "
            f"(active: {[k for k, v in self._active_movements.items() if v]})"
        )
        # Keyboard hold-to-drive: watchdog stops on MQTT gap; do not arm TimedStop.
        self._locomotion_stop.cancel()
        return self._send_twist(linear_x, angular_z)


class LightsHandler(CommandHandler):
    """
    Handler for UGV Beast LED/headlight control commands.

    This is the ONLY handler for light control (led_ctrl is deprecated).

    Controls:
    - IO4: Chassis headlight (near OKA camera)
    - IO5: Camera pan-tilt headlight (USB camera)

    Values: 0 = off, 255 = on (PWM for dimming)

    Status published to: {prefix}cyberwave/twin/{twin_uuid}/lights/status
    """

    def __init__(self, node: Node):
        super().__init__(node)
        # Track current LED states
        self._chassis_light_value = 0.0  # IO4
        self._camera_light_value = 0.0  # IO5

    def get_command_name(self) -> str:
        return "lights"

    def _setup_publishers(self) -> None:
        self._publishers["led_ctrl"] = self.create_ros_publisher(
            Float32MultiArray, "/ugv/led_ctrl", 10
        )

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle lights command with PWM support.

        Supported formats:

        1. PWM format (recommended):
           {"pwm": 128}  // 0-255, applies to both lights

        2. ROS-like format:
           {"data": [255, 255]}

        3. Array helper format:
           {"leds": [255, 255]}
           - leds[0] = IO4 (chassis headlight)
           - leds[1] = IO5 (camera headlight)

        4. Named IO format:
           {"io4": 255, "io5": 0}
           {"chassis_light": 255, "camera_light": 0}

        5. Convenience format:
           {"all": 255}  // Turn all lights on/off

        Values: 0 = off, 255 = on (or 0-255 for dimming if supported)
        """
        try:
            msg = Float32MultiArray()

            # Format 1: PWM format (most common from frontend)
            if "pwm" in data:
                pwm_value = data.get("pwm", 255)
                # Validate PWM range
                if not 0 <= pwm_value <= 255:
                    self.logger.warning(
                        f"PWM value {pwm_value} out of range [0-255], clamping"
                    )
                    pwm_value = max(0, min(255, pwm_value))
                msg.data = [float(pwm_value), float(pwm_value)]

            # Format 2 & 3: Array formats
            elif "data" in data:
                msg.data = [float(v) for v in data["data"]]
            elif "leds" in data:
                msg.data = [float(v) for v in data["leds"]]

            # Format 4: Named IO format
            elif "io4" in data or "io5" in data:
                io4 = float(data.get("io4", self._chassis_light_value))
                io5 = float(data.get("io5", self._camera_light_value))
                msg.data = [io4, io5]

            # Format 5: Named descriptive format
            elif "chassis_light" in data or "camera_light" in data:
                chassis = float(data.get("chassis_light", self._chassis_light_value))
                camera = float(data.get("camera_light", self._camera_light_value))
                msg.data = [chassis, camera]

            # Format 6: Convenience - all lights same value
            elif "all" in data:
                value = float(data["all"])
                msg.data = [value, value]

            else:
                self.logger.warning(
                    "lights command requires one of: 'pwm', 'data', 'leds', 'io4/io5', "
                    "'chassis_light/camera_light', or 'all'"
                )
                return False

            # Ensure we have exactly 2 elements as expected by UGV Beast driver
            if len(msg.data) < 2:
                msg.data.extend([0.0] * (2 - len(msg.data)))

            # Track current LED states
            self._chassis_light_value = msg.data[0]
            self._camera_light_value = msg.data[1]

            self._publishers["led_ctrl"].publish(msg)

            # Send response to dedicated status topic
            self.publish_response(
                {
                    "status": "success",
                    "io4": msg.data[0],
                    "io5": msg.data[1],
                    "chassis_light": "on" if msg.data[0] > 0 else "off",
                    "camera_light": "on" if msg.data[1] > 0 else "off",
                    "chassis_light_value": int(self._chassis_light_value),
                    "camera_light_value": int(self._camera_light_value),
                }
            )
            return True

        except Exception as e:
            self.logger.error(f"Failed to handle lights: {e}")
            return False


class EmergencyStopHandler(CommandHandler):
    """Handler for emergency stop commands."""

    def get_command_name(self) -> str:
        return "estop"

    def _setup_publishers(self) -> None:
        self._publishers["estop"] = self.create_ros_publisher(
            Bool, "/emergency_stop", 10
        )

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle emergency stop command.

        Expected data format:
        {
            "activate": true
        }
        """
        try:
            if not self.validate_data(data, ["activate"]):
                return False

            msg = Bool()
            msg.data = bool(data["activate"])

            self._publishers["estop"].publish(msg)
            self.logger.warn(
                f"Emergency stop {'ACTIVATED' if msg.data else 'DEACTIVATED'}"
            )
            return True

        except Exception as e:
            self.logger.error(f"Failed to handle estop: {e}")
            return False


class OledCtrlHandler(CommandHandler):
    """Handler for UGV Beast OLED display commands."""

    def get_command_name(self) -> str:
        return "oled_ctrl"

    def _setup_publishers(self) -> None:
        self._publishers["oled_ctrl"] = self.create_ros_publisher(
            String, "/ugv/oled_ctrl", 10
        )

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle OLED control command.

        Expected data format:
        {"text": "Hello"} or {"data": "Hello"} or just a string "Hello"
        """
        try:
            msg = String()
            if isinstance(data, str):
                msg.data = data
            elif "text" in data:
                msg.data = str(data["text"])
            elif "data" in data:
                msg.data = str(data["data"])
            else:
                self.logger.warning("oled_ctrl requires 'text' or 'data' field")
                return False

            self._publishers["oled_ctrl"].publish(msg)

            # Send confirmation response
            self.publish_response({"status": "displayed", "text": msg.data})
            return True

        except Exception as e:
            self.logger.error(f"Failed to handle oled_ctrl: {e}")
            return False


class JointTrajectoryHandler(CommandHandler):
    """Handler for sending joint trajectories (individual wheel/joint control)."""

    def get_command_name(self) -> str:
        return "trajectory"

    def _setup_publishers(self) -> None:
        self._publishers["trajectory"] = self.create_ros_publisher(
            JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10
        )

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle trajectory command.

        Expected data format:
        {
            "joint_names": ["left_up_wheel_link_joint", ...],
            "points": [{
                "positions": [0.0, ...],
                "velocities": [1.0, ...],
                "time_from_start": {"sec": 1, "nanosec": 0}
            }]
        }
        """
        try:
            if not self.validate_data(data, ["joint_names", "points"]):
                return False

            msg = JointTrajectory()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.joint_names = data["joint_names"]

            for pt_data in data["points"]:
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in pt_data.get("positions", [])]
                point.velocities = [float(v) for v in pt_data.get("velocities", [])]

                tfs = pt_data.get("time_from_start", {})
                point.time_from_start.sec = int(tfs.get("sec", 0))
                point.time_from_start.nanosec = int(tfs.get("nanosec", 0))

                msg.points.append(point)

            self._publishers["trajectory"].publish(msg)

            self.publish_response(
                {
                    "status": "success",
                    "joints": msg.joint_names,
                    "points_count": len(msg.points),
                }
            )
            return True

        except Exception as e:
            self.logger.error(f"Failed to handle trajectory: {e}")
            return False


class GripperHandler(CommandHandler):
    """Handler for gripper control commands."""

    def get_command_name(self) -> str:
        return "gripper"

    def _setup_publishers(self) -> None:
        # Using String for simplicity - can be upgraded to service call
        self._publishers["gripper"] = self.create_ros_publisher(
            String, "/gripper/command", 10
        )

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle gripper command.

        Expected data format:
        {
            "action": "grip"  // or "release", "reset"
        }
        """
        try:
            if not self.validate_data(data, ["action"]):
                return False

            action = data["action"]
            if action not in ["grip", "release", "reset"]:
                self.logger.warning(f"Invalid gripper action: {action}")
                return False

            msg = String()
            msg.data = action

            self._publishers["gripper"].publish(msg)

            # Send confirmation response
            self.publish_response({"status": "executed", "action": action})
            return True

        except Exception as e:
            self.logger.error(f"Failed to handle gripper: {e}")
            return False


class CameraServoHandler(CommandHandler):
    """
    Handler for UGV Beast pan-tilt camera servo control with smooth interpolation.

    Controls the pan-tilt mechanism via joint trajectory commands.
    Pan joint: pt_base_link_to_pt_link1 (horizontal rotation)
    Tilt joint: pt_link1_to_pt_link2 (vertical movement)

    After each servo movement the handler publishes the final position directly to
    MQTT (via the Cyberwave adapter) so the digital twin reflects the new state
    immediately, overriding any stale values published by the joint_state_publisher
    which only knows about default (zero) positions for the servo joints.
    """

    _PAN_JOINT = "pt_base_link_to_pt_link1"
    _TILT_JOINT = "pt_link1_to_pt_link2"

    def __init__(self, node: Node):
        super().__init__(node)
        # Track current servo positions
        self._pan_position = 0.0  # radians
        self._tilt_position = 0.0  # radians

        # Servo limits (in radians)
        self._pan_min = -3.14159  # -180 degrees
        self._pan_max = 3.14159  # +180 degrees
        self._tilt_min = -0.785  # -45 degrees
        self._tilt_max = 1.57  # +90 degrees

        # Interpolation settings - optimised for fast & smooth movement
        self._interpolation_steps = 4  # Number of intermediate steps
        self._interpolation_interval = 0.02  # seconds between steps (20 ms)
        self._interpolation_timer = None
        # Snapshot of the start position when an interpolation begins
        self._interp_start_pan = 0.0
        self._interp_start_tilt = 0.0
        self._target_pan = 0.0
        self._target_tilt = 0.0
        self._current_step = 0

        self.logger.info("CameraServoHandler initialized with smooth interpolation")

    def get_command_name(self) -> str:
        return "camera_servo"

    def _setup_publishers(self) -> None:
        # UGV Beast uses /ugv/joint_states for servo control, not trajectory controller
        self._publishers["joint_states"] = self.create_ros_publisher(
            JointState, "/ugv/joint_states", 10
        )

    def _interpolate_step(self) -> None:
        """Execute one step of the smooth interpolation towards the target position."""
        if self._current_step >= self._interpolation_steps:
            # Interpolation complete – snap to exact target and update persistent state
            self._pan_position = self._target_pan
            self._tilt_position = self._target_tilt
            if self._interpolation_timer is not None:
                self._interpolation_timer.cancel()
                self._interpolation_timer = None
            # Publish final position to MQTT so the digital twin stays in sync
            self._publish_servo_positions_to_mqtt()
            return

        # Progress goes from 0.0 → 1.0 over _interpolation_steps ticks.
        # We start at step=1 so progress ranges (1/N … N/N).
        # Interpolate from the *snapshot* start position captured when the command
        # arrived, not from the continuously-mutating _pan_position, to avoid the
        # progressive-error bug where each tick's delta compounds.
        progress = self._current_step / self._interpolation_steps

        # Ease-in-out smoothstep for smooth acceleration/deceleration
        progress = progress * progress * (3.0 - 2.0 * progress)

        self._pan_position = (
            self._interp_start_pan
            + (self._target_pan - self._interp_start_pan) * progress
        )
        self._tilt_position = (
            self._interp_start_tilt
            + (self._target_tilt - self._interp_start_tilt) * progress
        )

        # Publish intermediate position to ROS (drives the hardware)
        self._publish_ros_position()

        self._current_step += 1

    def _publish_ros_position(self) -> None:
        """Publish current servo position to the ROS hardware topic."""
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = (
            "tele"  # CRITICAL: must be 'tele' for ugv_integrated_driver to accept
        )
        msg.name = [self._PAN_JOINT, self._TILT_JOINT]
        msg.position = [self._pan_position, self._tilt_position]
        msg.velocity = []
        msg.effort = []
        self._publishers["joint_states"].publish(msg)

    def _publish_servo_positions_to_mqtt(self) -> None:
        """Publish the final servo positions directly to MQTT.

        The joint_state_publisher broadcasts default (zero) positions for the
        pan-tilt joints because the hardware does not report servo feedback.
        Those messages arrive at the MQTT bridge and overwrite the positions in
        the digital twin.  By publishing the authoritative servo positions to
        MQTT here we ensure the twin reflects the actual commanded state.
        """
        try:
            import time

            adapter = getattr(self.node, "_mqtt_adapter", None)
            mapping = getattr(self.node, "_mapping", None)
            if adapter is None or mapping is None:
                return

            twin_uuid = getattr(mapping, "twin_uuid", None)
            if not twin_uuid:
                return

            prefix = getattr(self.node, "ros_prefix", "")
            topic = f"{prefix}cyberwave/joint/{twin_uuid}/update"

            payload = {
                "source_type": "edge",
                "positions": {
                    self._PAN_JOINT: self._pan_position,
                    self._TILT_JOINT: self._tilt_position,
                },
                "ts": time.time(),
            }
            import json

            adapter.publish(topic, json.dumps(payload))
            self.logger.debug(
                f"Camera servo MQTT update: pan={self._pan_position:.3f}, tilt={self._tilt_position:.3f}"
            )
        except Exception as e:
            self.logger.warning(f"Failed to publish servo positions to MQTT: {e}")

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle camera servo command with smooth interpolation.

        Expected data format:
        {
            "pan_delta": 0.1,    # Optional: change in pan (radians)
            "tilt_delta": -0.1,  # Optional: change in tilt (radians)
            "pan": 0.5,          # Optional: absolute pan position (radians)
            "tilt": 0.3          # Optional: absolute tilt position (radians)
        }
        """
        try:
            # Cancel any ongoing interpolation and snap to its current position
            if self._interpolation_timer is not None:
                self._interpolation_timer.cancel()
                self._interpolation_timer = None

            # Calculate target position
            pan_delta = data.get("pan_delta", 0.0)
            tilt_delta = data.get("tilt_delta", 0.0)

            if "pan" in data:
                self._target_pan = float(data["pan"])
            else:
                self._target_pan = self._pan_position + float(pan_delta)

            if "tilt" in data:
                self._target_tilt = float(data["tilt"])
            else:
                self._target_tilt = self._tilt_position + float(tilt_delta)

            # Clamp target to physical limits
            self._target_pan = max(self._pan_min, min(self._pan_max, self._target_pan))
            self._target_tilt = max(
                self._tilt_min, min(self._tilt_max, self._target_tilt)
            )

            # Snapshot start position for correct linear interpolation
            self._interp_start_pan = self._pan_position
            self._interp_start_tilt = self._tilt_position

            # Start interpolation from step 1 (step 0 = current position, already sent)
            self._current_step = 1

            # Create timer for interpolation steps
            self._interpolation_timer = self.node.create_timer(
                self._interpolation_interval, self._interpolate_step
            )

            self.logger.debug(
                f"Camera servo: target pan={self._target_pan:.3f} rad, tilt={self._target_tilt:.3f} rad"
            )

            import math

            pan_deg = math.degrees(self._target_pan)
            tilt_deg = math.degrees(self._target_tilt)

            self.publish_response(
                {
                    "status": "success",
                    "pan": self._target_pan,
                    "tilt": self._target_tilt,
                    "pan_degrees": round(pan_deg, 1),
                    "tilt_degrees": round(tilt_deg, 1),
                    "pan_radians": round(self._target_pan, 3),
                    "tilt_radians": round(self._target_tilt, 3),
                }
            )
            return True

        except Exception as e:
            self.logger.error(f"Failed to handle camera servo: {e}")
            return False


class BatteryCheckHandler(CommandHandler):
    """
    Handler for explicit battery level check.

    Publishes to the same topic as automatic battery updates:
    cyberwave/twin/{twin_uuid}/battery/status
    """

    def get_command_name(self) -> str:
        return "battery_check"

    def _setup_publishers(self) -> None:
        pass

    def get_status_topic(self) -> Optional[str]:
        """
        Override to publish to battery/status instead of battery_check/status.
        This ensures command responses use the same topic as automatic telemetry.
        """
        try:
            mapping = getattr(self.node, "_mapping", None)
            if mapping and hasattr(mapping, "twin_uuid"):
                twin_uuid = mapping.twin_uuid
                prefix = getattr(self.node, "ros_prefix", "")
                # Publish to battery/status (same as automatic updates)
                return f"{prefix}cyberwave/twin/{twin_uuid}/battery/status"
        except Exception:
            return None

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle battery check command.

        Publishes battery status to: {prefix}cyberwave/twin/{twin_uuid}/battery/status
        Same format as automatic updates.
        """
        try:
            import time

            self.logger.info("Battery check command received")

            # Try to get battery data from the node's cache
            battery_msg = None
            if hasattr(self.node, "_last_battery_msg") and self.node._last_battery_msg:
                battery_msg = self.node._last_battery_msg
                self.logger.info(
                    f"Found cached battery message: {type(battery_msg).__name__}"
                )
            else:
                self.logger.warning("No battery message cached yet")

            if battery_msg:
                # Extract battery info
                # Check if it's a BatteryState or Float32 message
                if hasattr(battery_msg, "voltage"):
                    # BatteryState message
                    voltage = float(battery_msg.voltage)
                    percentage = (
                        float(battery_msg.percentage)
                        if hasattr(battery_msg, "percentage")
                        else None
                    )
                    self.logger.info(
                        f"BatteryState: voltage={voltage}V, percentage={percentage}"
                    )
                elif hasattr(battery_msg, "data"):
                    # Float32 message - calculate percentage
                    voltage = float(battery_msg.data)
                    percentage = (voltage - 9.0) / (12.6 - 9.0)
                    percentage = float(max(0.0, min(1.0, percentage)))
                    self.logger.info(
                        f"Float32: voltage={voltage}V, calculated percentage={percentage}"
                    )
                else:
                    voltage = None
                    percentage = None
                    self.logger.error("Unknown battery message format")

                if voltage is not None:
                    # Get the status topic (battery/status)
                    status_topic = self.get_status_topic()
                    if status_topic:
                        # Publish directly to the topic (not through publish_response)
                        # to use the same format as automatic updates
                        payload = {
                            "source_type": "edge",
                            "voltage": voltage,
                            "percentage": percentage if percentage is not None else 0.0,
                            "timestamp": time.time(),
                        }

                        self.logger.info(
                            f"Publishing battery status to {status_topic}: {payload}"
                        )

                        # Use the adapter or client to publish
                        if (
                            hasattr(self.node, "_mqtt_adapter")
                            and self.node._mqtt_adapter
                        ):
                            self.node._mqtt_adapter.publish(status_topic, payload)
                            self.logger.info("Published via adapter")
                        elif (
                            hasattr(self.node, "_mqtt_client")
                            and self.node._mqtt_client
                        ):
                            import json

                            self.node._mqtt_client.publish(
                                status_topic, json.dumps(payload)
                            )
                            self.logger.info("Published via paho client")
                        else:
                            self.logger.error("No MQTT client available")

                        return True
                    else:
                        self.logger.error("Could not determine status topic")

                # Fallback if we can't extract voltage
                self.logger.error("Could not extract voltage from battery message")
                self.publish_response(
                    {
                        "status": "error",
                        "message": "Could not extract voltage from battery message",
                    }
                )
            else:
                self.logger.warning(
                    "Battery data not available yet - no cached message"
                )
                self.publish_response(
                    {"status": "error", "message": "Battery data not available yet"}
                )
            return True
        except Exception as e:
            self.logger.error(f"Failed to handle battery check: {e}")
            return False


class TakePhotoHandler(CommandHandler):
    """Handler for taking a photo/snapshot from the camera."""

    def get_command_name(self) -> str:
        return "take_photo"

    def _setup_publishers(self) -> None:
        pass

    def handle(self, data: Dict[str, Any]) -> bool:
        """Handle take_photo command."""
        try:
            if not CV2_AVAILABLE:
                self.publish_response(
                    {
                        "status": "error",
                        "command": "take_photo",
                        "message": "OpenCV not available for image encoding",
                    }
                )
                return False
            
            # Try to get the latest frame from the camera streamer (BGR for JPEG).
            frame = None
            if hasattr(self.node, '_ros_streamer') and self.node._ros_streamer:
                if hasattr(self.node._ros_streamer, 'streamer') and self.node._ros_streamer.streamer:
                    streamer = self.node._ros_streamer.streamer
                    if hasattr(streamer, 'get_latest_frame_bgr'):
                        frame = streamer.get_latest_frame_bgr()
                    elif hasattr(streamer, 'latest_frame'):
                        frame = streamer.latest_frame
            
            if frame is None:
                self.publish_response(
                    {
                        "status": "error",
                        "command": "take_photo",
                        "message": "No camera frame available",
                    }
                )
                return False

            # Encode frame to JPEG
            success, encoded_image = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85]
            )

            if not success:
                self.publish_response(
                    {
                        "status": "error",
                        "command": "take_photo",
                        "message": "Failed to encode image",
                    }
                )
                return False

            # Convert to base64
            image_base64 = base64.b64encode(encoded_image.tobytes()).decode("utf-8")

            # Get twin UUID for topic
            twin_uuid = None
            if hasattr(self.node, "_mapping") and self.node._mapping:
                twin_uuid = getattr(self.node._mapping, "twin_uuid", None)

            if twin_uuid:
                prefix = getattr(self.node, "ros_prefix", "")

                # Publish the captured image to a dedicated photo topic
                photo_topic = f"{prefix}cyberwave/twin/{twin_uuid}/camera/photo"
                photo_payload = {
                    "source_type": "edge",
                    "timestamp": self.node.get_clock().now().nanoseconds / 1e9,
                    "image": image_base64,
                    "format": "jpeg",
                    "width": frame.shape[1],
                    "height": frame.shape[0],
                }
                # Use node.publish() which properly serializes dict to JSON
                # and handles both CyberwaveAdapter and paho-mqtt fallback
                self.node.publish(photo_topic, photo_payload)
                self.logger.info(
                    f"Published photo to {photo_topic} ({len(image_base64)} bytes)"
                )

            # Send success response
            self.publish_response(
                {
                    "status": "success",
                    "command": "take_photo",
                    "message": "Photo captured successfully",
                    "image_size": len(image_base64),
                }
            )
            return True

        except Exception as e:
            self.logger.error(f"Failed to handle take_photo: {e}")
            self.publish_response(
                {
                    "status": "error",
                    "command": "take_photo",
                    "message": f"Exception: {str(e)}",
                }
            )
            return False


class CameraCommandHandler(CommandHandler):
    """Handles frontend video streaming commands."""

    def __init__(self, node: Node, command_name: str):
        super().__init__(node)
        self._command_name = command_name

    def get_command_name(self) -> str:
        return self._command_name

    def _setup_publishers(self) -> None:
        pass

    @staticmethod
    def _stream_kwargs(data: Any) -> Dict[str, Any]:
        """Extract optional {recording,format/pixel_format,width,height,fps} from data."""
        if not isinstance(data, dict):
            return {}
        kwargs: Dict[str, Any] = {}
        if "recording" in data:
            kwargs["recording"] = bool(data.get("recording"))
        fmt = data.get("pixel_format") or data.get("format")
        if fmt:
            kwargs["pixel_format"] = str(fmt)
        for src, dst in (("width", "width"), ("image_width", "width"),
                         ("height", "height"), ("image_height", "height")):
            if data.get(src) is not None:
                kwargs[dst] = int(data[src])
        if data.get("fps") is not None:
            kwargs["fps"] = int(data["fps"])
        return kwargs

    def handle(self, data: Any) -> bool:
        # Commands from FE: "start_video", "stop_video", "set_camera_format"
        if self._command_name == "start_video":
            # Pass through any requested format/resolution/fps/recording so the
            # frontend can select a stream profile (previously dropped).
            self.node.start_camera_stream(**self._stream_kwargs(data))
            self.publish_response({"status": "ok", "type": "video_started"})
        elif self._command_name == "stop_video":
            self.node.stop_camera_stream()
            self.publish_response({"status": "ok", "type": "video_stopped"})
        elif self._command_name == "set_camera_format":
            if not hasattr(self.node, "set_camera_format"):
                self.publish_response(
                    {"status": "error", "message": "set_camera_format not supported"}
                )
                return False
            d = data if isinstance(data, dict) else {}
            result = self.node.set_camera_format(
                pixel_format=d.get("pixel_format") or d.get("format"),
                width=d.get("width") or d.get("image_width"),
                height=d.get("height") or d.get("image_height"),
                fps=d.get("fps"),
            )
            # Node returns a structured dict (applied config or validation error).
            self.publish_response(result if isinstance(result, dict) else {"status": "ok"})
            return bool(isinstance(result, dict) and result.get("status") != "error")
        return True


class StatusQueryHandler(CommandHandler):
    """Handler for querying robot status (IMU, Battery, Odom)."""

    def get_command_name(self) -> str:
        return "get_status"

    def _setup_publishers(self) -> None:
        # No publishers needed, we just read from the node's state
        pass

    def handle(self, data: Dict[str, Any]) -> bool:
        """
        Handle status query command.

        Expected data format:
        {
            "target": "battery"  // or "imu", "odom", "joint_states", "all"
        }
        """
        try:
            target = data.get("target", "all")

            # Map simple target names to actual ROS topics
            topic_map = {
                "battery": "/ugv/battery_status",
                "imu": "/ugv/imu",
                "odom": "/odom",
                "joint_states": "/ugv/joint_states",
                "cmd_vel": "/cmd_vel",
                "led_ctrl": "/ugv/led_ctrl",
                "oled_ctrl": "/ugv/oled_ctrl",
            }

            # Get the cache from the node
            cache = getattr(self.node, "_ros_state_cache", {})

            if target == "all":
                response_data = {}
                for name, topic in topic_map.items():
                    msg = cache.get(topic)
                    if msg:
                        response_data[name] = self._serialize_msg(msg)
                self.publish_response({"status": "success", "data": response_data})
            elif target in topic_map:
                topic = topic_map[target]
                msg = cache.get(topic)
                if msg:
                    self.publish_response(
                        {
                            "status": "success",
                            "target": target,
                            "data": self._serialize_msg(msg),
                        }
                    )
                else:
                    self.publish_response(
                        {
                            "status": "error",
                            "message": f"No data cached for {target} ({topic})",
                        }
                    )
            else:
                self.publish_response(
                    {
                        "status": "error",
                        "message": f"Unknown status target: {target}. Available: {list(topic_map.keys())}",
                    }
                )

            return True

        except Exception as e:
            self.logger.error(f"Failed to handle status query: {e}")
            return False

    def _serialize_msg(self, msg) -> Any:
        """Helper to serialize ROS message to dict."""
        try:
            # Try to use the node's encoder if available (it handles JointState and battery status specially)
            if hasattr(self.node, "_encode_msg_for_mqtt"):
                import json

                # Determine context topic if possible
                context_topic = None
                if (
                    "battery" in str(type(msg)).lower()
                    or "float" in str(type(msg)).lower()
                ):
                    # For battery status, we need to pass a topic containing 'status/battery' to trigger special encoding
                    # if the type is Float32.
                    context_topic = "cyberwave/twin/unknown/status/battery"

                encoded = self.node._encode_msg_for_mqtt(
                    msg, type(msg), mqtt_topic=context_topic
                )
                return json.loads(encoded)

            # Fallback to direct conversion
            from rosidl_runtime_py.convert import message_to_ordereddict

            return message_to_ordereddict(msg)
        except Exception:
            return str(msg)


class CommandRegistry:
    """
    Registry for command handlers.

    Maintains a mapping of command names to handler instances.
    Provides methods to register handlers and route commands.
    """

    def __init__(self, node: Node):
        """
        Initialize the command registry.

        Args:
            node: ROS2 node for creating handlers
        """
        self.node = node
        self.logger = node.get_logger()
        self._handlers: Dict[str, CommandHandler] = {}
        self._mqtt_adapter: Any = None
        self._command_topic: Optional[str] = None
        self._register_default_handlers()

    def set_mqtt_context(self, adapter: Any, command_topic: str):
        """Pass MQTT context to all registered handlers."""
        self._mqtt_adapter = adapter
        self._command_topic = command_topic
        for handler in self._handlers.values():
            handler.set_mqtt_context(adapter, command_topic)

    def _register_default_handlers(self) -> None:
        """Register all default command handlers."""
        # NOTE: cmd_vel and led_ctrl handlers are deprecated
        # Use 'actuation' for movement and 'lights' for LED control
        default_handlers = [
            UGVBeastActuationHandler,  # Handles movement commands (move_forward, turn_left, stop, etc.)
            LightsHandler,  # Handles lights command (replaces led_ctrl)
            EmergencyStopHandler,
            OledCtrlHandler,
            JointTrajectoryHandler,
            GripperHandler,
            StatusQueryHandler,
            BatteryCheckHandler,
            TakePhotoHandler,
            CameraServoHandler,
        ]

        for handler_class in default_handlers:
            self.register_handler(handler_class)

        # Register specialized camera handlers for start/stop to match FE button
        self.register_handler_instance(CameraCommandHandler(self.node, "start_video"))
        self.register_handler_instance(CameraCommandHandler(self.node, "stop_video"))
        # Runtime format/resolution/fps selection (managed relaunch in the node).
        self.register_handler_instance(CameraCommandHandler(self.node, "set_camera_format"))

    def register_handler(self, handler_class: type) -> None:
        """
        Register a command handler class.

        Args:
            handler_class: CommandHandler subclass to register
        """
        try:
            handler = handler_class(self.node)
            command_name = handler.get_command_name()
            self._handlers[command_name] = handler
            self.logger.info(f"Registered command handler: {command_name}")
        except Exception as e:
            self.logger.error(
                f"Failed to register handler {handler_class.__name__}: {e}"
            )

    def register_handler_instance(self, handler: CommandHandler) -> None:
        """
        Register an already-instantiated command handler.

        Args:
            handler: CommandHandler instance to register
        """
        command_name = handler.get_command_name()
        self._handlers[command_name] = handler
        self.logger.info(f"Registered command handler: {command_name}")

    def handle_command(self, command: str, data: Dict[str, Any]) -> bool:
        """
        Route a command to the appropriate handler.

        Args:
            command: Command name/type
            data: Command data dictionary

        Returns:
            True if command was handled successfully, False otherwise
        """
        if command not in self._handlers:
            self.logger.warning(
                f"No handler registered for command: {command}. "
                f"Available: {list(self._handlers.keys())}"
            )
            return False

        handler = self._handlers[command]
        return handler.handle(data)

    def get_registered_commands(self) -> list:
        """Get list of all registered command names."""
        return list(self._handlers.keys())

    def unregister_handler(self, command_name: str) -> bool:
        """
        Unregister a command handler.

        Args:
            command_name: Name of command to unregister

        Returns:
            True if handler was found and removed, False otherwise
        """
        if command_name in self._handlers:
            del self._handlers[command_name]
            self.logger.info(f"Unregistered command handler: {command_name}")
            return True
        return False


# Decorator for easy handler registration
def command_handler(command_name: str):
    """
    Decorator for registering command handlers.

    Usage:
        @command_handler("my_command")
        class MyCommandHandler(CommandHandler):
            ...
    """

    def decorator(cls):
        cls._command_name = command_name
        return cls

    return decorator
