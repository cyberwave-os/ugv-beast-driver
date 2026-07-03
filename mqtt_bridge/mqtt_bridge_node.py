"""Simple ROS2 <-> MQTT bridge node.

This node implements a small, configurable bridge between ROS2 topics and an
MQTT broker. Configuration is taken from ROS parameters (typically via a
YAML file such as `config/params.yaml`). It supports two primary mapping
directions:

- ros2 -> mqtt (``ros2mqtt``): subscribe to ROS topics and publish to MQTT.
- mqtt -> ros2 (``mqtt2ros``): subscribe to MQTT topics and publish ROS msgs.

Design notes / behavior highlights:
- This project can optionally use the Cyberwave Python SDK via a lightweight
    adapter (``CyberwaveAdapter``). When the adapter is present and configured
    some mappings may use SDK helper methods (for example ``update_joint_state``)
    instead of raw topic publishing. This is used for richer JointState
    integration and to reuse SDK rate-limiting / semantics.
- Mapping files can contain per-robot information (including ``twin_uuid`` and
    joint name maps). When present the bridge will remap JointState messages to
    the SDK expected payload shape.
- MQTT callbacks normally run on the MQTT client's background thread. Simple
    publishing into rclpy from that thread is allowed here. If you perform heavy
    processing in callback handlers consider delegating work to the ROS executor.

Limitations / scope:
- The implementation aims to be compact and easy to extend. It focuses on
    common message types (String, Int32, Float32, JointState, and simple arrays)
    and provides mapping helpers for JointState payloads.
"""

from __future__ import annotations

import logging
import math
import asyncio
import threading

# Default: Silence verbose third-party libraries
# This will be overridden in __init__ if debug_logs parameter is enabled
for lib in ["aiortc", "aioice", "google", "asyncio"]:
    logging.getLogger(lib).setLevel(logging.WARNING)
import time
import typing
import json
import os
import yaml
import sys
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import (
    String,
    Int32,
    Float32,
    UInt32MultiArray,
    Float32MultiArray,
    Float64MultiArray,
)
from sensor_msgs.msg import JointState, Imu, BatteryState, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, SetBool

try:
    from ur_msgs.srv import SetIO
except ImportError:
    SetIO = None

from rosidl_runtime_py.convert import message_to_ordereddict

from .mapping import Mapping
from .ros_topic_namespace import resolve_ros_namespace, resolve_ros_topic as _resolve_ros_topic
from .plugins.ros_camera import ROSCameraStreamer
from .plugins.camera_device_manager import (
    CameraDeviceManager,
    FormatValidationError,
    decide_recovery,
    RECOVERY_NONE,
    RECOVERY_RESTART,
    RECOVERY_RECONFIGURE,
    RECOVERY_RERESOLVE,
)
from .plugins.navigation_bridge import NavigationBridge
from .health import HealthPublisher
from .telemetry import TelemetryProcessor
from .plugins.internal_odometry import InternalOdometry
import asyncio
import importlib

import paho.mqtt.client as mqtt

try:
    from .cyberwave_mqtt_adapter import (
        CyberwaveAdapter,
        SOURCE_TYPE_EDGE,
        SOURCE_TYPE_TELE,
        SOURCE_TYPE_EDIT,
        SOURCE_TYPE_SIM,
        SOURCE_TYPE_SIM_TELE,
        SOURCE_TYPE_EDGE_LEADER,
        SOURCE_TYPE_EDGE_FOLLOWER,
    )
except Exception:
    CyberwaveAdapter = None
    SOURCE_TYPE_EDGE = "edge"
    SOURCE_TYPE_TELE = "tele"
    SOURCE_TYPE_EDIT = "edit"
    SOURCE_TYPE_SIM = "sim"
    SOURCE_TYPE_SIM_TELE = "sim_tele"
    SOURCE_TYPE_EDGE_LEADER = "edge_leader"
    SOURCE_TYPE_EDGE_FOLLOWER = "edge_follower"


def _paho_on_connect(client, userdata, flags, rc) -> None:
    node = userdata
    # call instance method
    try:
        node._handle_mqtt_connect(rc)
    except Exception as e:
        node.get_logger().error(f"Exception in on_connect handler: {e}")


def _paho_on_message(client, userdata, msg) -> None:
    node = userdata
    try:
        node._handle_mqtt_message(msg)
    except Exception as e:
        node.get_logger().error(f"Exception in on_message handler: {e}")


def _strip_topic_prefix(value: typing.Any) -> str:
    """Strip whitespace and trailing ``/`` for ``{prefix}cyberwave/...`` topics."""
    if not value:
        return ""
    return str(value).strip().rstrip("/")


class MQTTBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mqtt_bridge_node")
        self._start_time = time.time()

        from .edge_driver_env import (
            apply_resolved_mqtt_to_environ,
            load_edge_driver_env,
            log_edge_driver_env,
            resolve_broker_host,
            resolve_broker_port,
            resolve_mqtt_topic_prefix,
        )

        self._edge_env = load_edge_driver_env()
        log_edge_driver_env(self.get_logger(), self._edge_env)
        self._child_twin_uuids = list(self._edge_env.child_twin_uuids)
        twin_json = self._edge_env.load_twin_json()
        if twin_json is not None:
            self.get_logger().info(
                "Loaded twin metadata from CYBERWAVE_TWIN_JSON_FILE "
                f"({self._edge_env.twin_json_file})"
            )
        elif self._edge_env.twin_json_file:
            self.get_logger().warning(
                f"CYBERWAVE_TWIN_JSON_FILE set but not readable: "
                f"{self._edge_env.twin_json_file}"
            )

        # Configuration is loaded from params.yaml via ROS parameter server
        # Environment variables can still override via os.getenv() fallbacks below

        # declare parameters with sensible defaults
        # declare scalar broker settings (ROS parameter server flattens YAML
        # mappings into dotted parameter names when loaded) and avoid
        # declaring dict defaults which rclpy does not accept.
        mqtt_broker_env = self._edge_env.mqtt_host or None

        # rclpy's declare_parameter API warns when a parameter is declared
        # without an explicit default value. Ensure we always pass a concrete
        # default (empty string) so callers can still override via params or
        # environment variables without triggering deprecation warnings.
        self.declare_parameter(
            "broker.host", mqtt_broker_env if mqtt_broker_env is not None else ""
        )
        _default_broker_port = (
            self._edge_env.mqtt_port_int if self._edge_env.mqtt_port_int else 1883
        )
        self.declare_parameter("broker.port", _default_broker_port)
        self.declare_parameter("broker.use_paho_direct", False)
        self.declare_parameter("broker.username", "")
        self.declare_parameter("broker.password", "")
        self.declare_parameter("cyberwave_token", "")
        self.declare_parameter("topic_prefix", "")
        self.declare_parameter("ros_namespace", "")

        # Get values from parameters (which are loaded from params.yaml)
        p_token = self.get_parameter("cyberwave_token").value

        # Fallback to edge-core env, then ROS params
        token = p_token or self._edge_env.api_key

        # Debug logging parameter - enables verbose logging for aiortc, aioice, etc.
        self.declare_parameter("debug_logs", self._edge_env.debug_logs_enabled)
        debug_logs_enabled = self.get_parameter("debug_logs").value

        # Configure logging levels based on debug_logs parameter
        # Always enable cyberwave SDK logging at INFO level to see WebRTC signaling
        logging.getLogger("cyberwave").setLevel(logging.INFO)
        logging.getLogger("cyberwave.camera").setLevel(logging.INFO)
        logging.getLogger("cyberwave.sensor").setLevel(logging.INFO)

        # Configure a handler to ensure SDK logs reach stdout
        sdk_handler = logging.StreamHandler()
        sdk_handler.setLevel(logging.INFO)
        sdk_handler.setFormatter(
            logging.Formatter("[%(name)s] %(levelname)s: %(message)s")
        )
        for sdk_logger_name in ["cyberwave", "cyberwave.camera", "cyberwave.sensor"]:
            sdk_logger = logging.getLogger(sdk_logger_name)
            if not sdk_logger.handlers:
                sdk_logger.addHandler(sdk_handler)

        if debug_logs_enabled:
            self.get_logger().info(
                "Debug logs ENABLED - showing aiortc, aioice, google, asyncio, cyberwave debug messages"
            )
            for lib in ["aiortc", "aioice", "google", "asyncio", "cyberwave"]:
                logging.getLogger(lib).setLevel(logging.DEBUG)
        else:
            self.get_logger().info(
                "Debug logs disabled - aiortc, aioice, google, asyncio set to WARNING level (cyberwave at INFO)"
            )
            # Ensure they're set to WARNING (they should already be from module init)
            for lib in ["aiortc", "aioice", "google", "asyncio"]:
                logging.getLogger(lib).setLevel(logging.WARNING)

        # Rate limiting for upstream (ROS -> MQTT) publishing
        # Default upstream publishing frequency to Cyberwave cloud (in Hz)
        DEFAULT_UPSTREAM_PUBLISH_RATE_HZ = 1.0

        # Read from environment variable (in Hz) or use default
        rate_limit_env = os.getenv("MQTT_PUBLISH_RATE_LIMIT")
        default_rate_hz = (
            float(rate_limit_env)
            if rate_limit_env
            else DEFAULT_UPSTREAM_PUBLISH_RATE_HZ
        )
        self.declare_parameter("ros2mqtt_rate_limit", default_rate_hz)

        # Flag to disable upstream joint state updates from edge
        self.declare_parameter("disable_edge_joint_updates", False)
        self._disable_edge_joint_updates = self.get_parameter(
            "disable_edge_joint_updates"
        ).value

        # Global kill-switch for all upstream MQTT traffic (ROS -> MQTT)
        self.declare_parameter("disable_all_upstream", False)
        self._disable_all_upstream = self.get_parameter("disable_all_upstream").value

        # MQTT QoS for command subscriptions (default: 1 for reliability with low latency)
        self.declare_parameter("mqtt_command_qos", 1)
        self._mqtt_command_qos = self.get_parameter("mqtt_command_qos").value

        # Broker endpoint: CYBERWAVE_MQTT_* from edge-core wins over params.yaml
        # (e.g. params broker.port: 8883 must not override CYBERWAVE_MQTT_PORT=1883).
        host_param = self.get_parameter("broker.host").value
        port_param = self.get_parameter("broker.port").value
        username_param = self.get_parameter("broker.username").value
        password_param = self.get_parameter("broker.password").value
        host, host_source = resolve_broker_host(host_param, self._edge_env)
        port, port_source = resolve_broker_port(port_param, self._edge_env)
        if self._edge_env.mqtt_port and port_param is not None:
            try:
                yaml_port = int(port_param)
            except (TypeError, ValueError):
                yaml_port = None
            if yaml_port is not None and yaml_port != port:
                self.get_logger().info(
                    f"broker.port from params.yaml ({yaml_port}) overridden by "
                    f"CYBERWAVE_MQTT_PORT={port}"
                )
        if self._edge_env.mqtt_host and host_param and str(host_param).strip() != host:
            self.get_logger().info(
                f"broker.host from params.yaml ({host_param!r}) overridden by "
                f"CYBERWAVE_MQTT_HOST={host!r}"
            )
        self.get_logger().info(
            f"MQTT broker endpoint: {host}:{port} "
            f"(host from {host_source}, port from {port_source})"
        )

        mqtt_use_tls = self._edge_env.mqtt_use_tls
        apply_resolved_mqtt_to_environ(
            host,
            port,
            api_key=token or self._edge_env.api_key,
            use_tls=mqtt_use_tls,
        )

        # Store broker credentials
        username = username_param if username_param else None
        password = password_param if password_param else None

        # 'bridge' is a potentially complex mapping. ROS2 parameter system doesn't
        # handle complex nested dicts well, so we load directly from params.yaml
        bridge = {}
        try:
            # Find the package share directory for mqtt_bridge
            # Use multiple methods to be robust
            pkg_share = None
            try:
                from ament_index_python.packages import (
                    get_package_share_directory as gpsd,
                )

                pkg_share = gpsd("mqtt_bridge")
            except Exception:
                # Fallback: try to find it relative to this file
                import pathlib

                current_file = pathlib.Path(__file__).resolve()
                # Go up to find the install directory
                for parent in current_file.parents:
                    potential_share = parent / "share" / "mqtt_bridge"
                    if potential_share.exists():
                        pkg_share = str(potential_share)
                        break

            if pkg_share:
                cfg_path = os.path.join(pkg_share, "config", "params.yaml")
                if os.path.exists(cfg_path):
                    with open(cfg_path, "r") as f:
                        raw = yaml.safe_load(f)

                    # params.yaml nests under '/mqtt_bridge_node/ros__parameters'
                    if (
                        "/mqtt_bridge_node" in raw
                        and "ros__parameters" in raw["/mqtt_bridge_node"]
                    ):
                        ros_params = raw["/mqtt_bridge_node"]["ros__parameters"]
                        bridge = ros_params.get("bridge", {}) or {}

                        ros2mqtt_topics = bridge.get("ros2mqtt", {}).get(
                            "ros_topics", []
                        )
                        mqtt2ros_topics = bridge.get("mqtt2ros", {}).get(
                            "mqtt_topics", []
                        )

                        self.get_logger().info(
                            f"Loaded bridge config from {cfg_path}: "
                            f"{len(ros2mqtt_topics)} ROS->MQTT topics, "
                            f"{len(mqtt2ros_topics)} MQTT->ROS topics"
                        )
                    else:
                        self.get_logger().warning(
                            f"No ros__parameters found in {cfg_path}"
                        )
                else:
                    self.get_logger().error(f"Config file not found: {cfg_path}")
            else:
                self.get_logger().error(
                    "Could not find mqtt_bridge package share directory"
                )
        except Exception as e:
            import traceback

            self.get_logger().error(
                f"Failed to load bridge config: {e}\n{traceback.format_exc()}"
            )
            bridge = {}

        # storage for runtime mappings
        # mqtt_topic -> (rclpy publisher, msg_class)
        self._mqtt2ros_pubs = {}
        # ros topic -> mqtt topic (for subscribers' callbacks)
        # and store the msg class for the subscription so encoder can be chosen
        self._ros2mqtt_map = {}
        # optional SDK method name per ROS topic (e.g. 'update_joint_state')
        self._ros2mqtt_sdk_method = {}
        # optional SDK method name per MQTT topic (e.g. 'subscribe_twin_joint_states')
        self._mqtt2ros_sdk_method = {}
        self._ros2mqtt_msgtypes = {}
        # track ros topic -> message class used to avoid creating
        # incompatible duplicate publishers
        self._ros_topic_msgtype = {}
        # Store last command for continuous republishing (needed for position controllers)
        self._last_position_command = None
        self._last_position_command_msg_cls = None
        self._position_command_publisher = None
        self._position_command_timer = None

        # Battery check interval (removed periodic)
        self._last_battery_msg = None
        self._battery_update_timer = None  # Timer for periodic battery updates
        # Accumulated joint states for building full trajectories from single-joint updates
        self._accumulated_joint_states = {}
        self._joint_state_initialized = False

        # --- Internal Odometry State (Plugin) ---
        self._internal_odom = None
        # -------------------------------
        # mqtt topic -> list of python callbacks(callable(topic,payload,mqtt_msg))
        self._mqtt_callbacks = {}
        # Rate limiting: track last publish time per ROS topic (for ros2mqtt direction)
        self._last_publish_time = {}  # ros_topic -> timestamp
        self._ros2mqtt_custom_intervals = {}  # per-topic custom intervals (e.g. for battery)

        # Read rate limit parameter (Hz frequency)
        try:
            self._ros2mqtt_rate_hz = float(
                self.get_parameter("ros2mqtt_rate_limit").value
            )
            if self._ros2mqtt_rate_hz > 0:
                interval_seconds = 1.0 / self._ros2mqtt_rate_hz
                self.get_logger().info(
                    f"ROS->MQTT rate limiting enabled: {self._ros2mqtt_rate_hz:.2f} Hz "
                    f"({interval_seconds:.2f}s between publishes)"
                )
                # Store the interval in seconds for actual rate limiting logic
                self._ros2mqtt_rate_interval = interval_seconds
            else:
                self.get_logger().info("ROS->MQTT rate limiting disabled (set to 0 Hz)")
                self._ros2mqtt_rate_interval = 0.0
        except Exception:
            self._ros2mqtt_rate_hz = (
                DEFAULT_UPSTREAM_PUBLISH_RATE_HZ  # default fallback
            )
            self._ros2mqtt_rate_interval = 1.0 / DEFAULT_UPSTREAM_PUBLISH_RATE_HZ
            self.get_logger().warn(
                f"Failed to read ros2mqtt_rate_limit parameter, using default: "
                f"{self._ros2mqtt_rate_hz:.2f} Hz ({self._ros2mqtt_rate_interval:.3f}s)"
            )

        # remember last published joint arrays per ROS topic so we can
        # preserve older values when updates contain NaNs for some joints
        # key: ros_topic -> list[float]
        self._last_joint_values: typing.Dict[str, typing.List[float]] = {}
        # Also keep last joint arrays keyed by twin_uuid for stability
        # across topic naming/publisher changes.
        self._last_joint_values_by_twin: typing.Dict[str, typing.List[float]] = {}

        # Cache for latest ROS messages keyed by topic name.
        # Used by command handlers for status queries.
        self._ros_state_cache: typing.Dict[str, typing.Any] = {}

        # Initialize background asyncio loop for WebRTC and other async tasks.
        # Initialize early so it can be passed to components like the MQTT adapter.
        self._async_loop = asyncio.new_event_loop()

        def _run_async_loop(loop):
            asyncio.set_event_loop(loop)
            try:
                loop.run_forever()
            except Exception:
                pass

        self._async_loop_thread = threading.Thread(
            target=_run_async_loop, args=(self._async_loop,), daemon=True
        )
        self._async_loop_thread.start()

        # Initialize health publisher
        self._health_publisher = HealthPublisher(self)
        self._health_timer = self.create_timer(
            5.0, self._health_publisher.publish_health_status
        )

        # Camera watchdog - only enable if usb_cam package is available
        self._last_camera_check_time = 0
        self._last_image_time = (
            time.time()
        )  # Initialize with current time for grace period
        self._camera_watchdog_timer = None
        self._image_sub_for_watchdog = None

        # Check if usb_cam package is available before enabling watchdog
        try:
            from ament_index_python.packages import get_package_share_directory

            get_package_share_directory("usb_cam")
            self._usb_cam_available = True
            self._camera_watchdog_timer = self.create_timer(
                5.0, self._check_camera_status
            )
            self.get_logger().info("Camera watchdog enabled (usb_cam package found)")
        except Exception:
            self._usb_cam_available = False
            self.get_logger().info(
                "Camera watchdog disabled (usb_cam package not installed)"
            )
        # Note: We no longer create a separate subscription here.
        # The ROSVideoStreamTrack updates self._last_image_time directly.

        # helper: map simple type strings to message classes we support here
        self._type_map = {
            "std_msgs/String": String,
            "String": String,
            "std_msgs/Int32": Int32,
            "Int32": Int32,
            "std_msgs/Float32": Float32,
            "Float32": Float32,
            "std_msgs/UInt32MultiArray": UInt32MultiArray,
            "UInt32MultiArray": UInt32MultiArray,
            "std_msgs/Float32MultiArray": Float32MultiArray,
            "Float32MultiArray": Float32MultiArray,
            "std_msgs/Float64MultiArray": Float64MultiArray,
            "Float64MultiArray": Float64MultiArray,
            # support JointState messages (used by /joint_state)
            "sensor_msgs/JointState": JointState,
            "sensor_msgs/msg/JointState": JointState,
            "JointState": JointState,
            "sensor_msgs/Imu": Imu,
            "sensor_msgs/msg/Imu": Imu,
            "Imu": Imu,
            "sensor_msgs/BatteryState": BatteryState,
            "sensor_msgs/msg/BatteryState": BatteryState,
            "BatteryState": BatteryState,
            "nav_msgs/Odometry": Odometry,
            "nav_msgs/msg/Odometry": Odometry,
            "Odometry": Odometry,
            "geometry_msgs/Twist": Twist,
            "geometry_msgs/msg/Twist": Twist,
            "Twist": Twist,
            "trajectory_msgs/JointTrajectory": JointTrajectory,
            "trajectory_msgs/msg/JointTrajectory": JointTrajectory,
            "JointTrajectory": JointTrajectory,
        }
        # Optionally use the Cyberwave SDK adapter if available and requested
        self.declare_parameter("broker.use_cyberwave", True)
        use_cw = self.get_parameter("broker.use_cyberwave").value

        # Check if direct paho-mqtt connection is requested (bypasses Cyberwave SDK)
        use_paho_direct = self.get_parameter("broker.use_paho_direct").value
        if use_paho_direct:
            self.get_logger().warning(
                "use_paho_direct=True: Forcing paho-mqtt direct connection (bypassing Cyberwave SDK)"
            )
            use_cw = False
        # Camera runtime overrides (config/params.yaml `camera:` block). These let
        # an operator pin pixel_format/resolution/fps without editing the per-robot
        # mapping; they override the mapping's camera defaults at startup. Runtime
        # adaptation may still LOWER fps/resolution under load. Sentinels ("" / 0)
        # mean "not overridden — use the mapping value".
        self.declare_parameter("camera.pixel_format", "")
        self.declare_parameter("camera.image_width", 0)
        self.declare_parameter("camera.image_height", 0)
        self.declare_parameter("camera.fps", 0)

        # Note: image_topic is now in the robot mapping file under camera.image_topic
        self.declare_parameter("webrtc.auto_start", False)
        self.declare_parameter("webrtc.auto_start_delay_sec", 10.0)
        self.declare_parameter("webrtc.auto_start_retry_sec", 5.0)
        self.declare_parameter("webrtc.fps", 15.0)
        self.declare_parameter("webrtc.force_turn", False)

        # Log connection attempt
        self.get_logger().info(f"Connecting to MQTT broker {host}:{port}...")
        self.get_logger().debug(
            f"Broker configuration: host={host}, port={port}, user={username}, use_cyberwave={use_cw}"
        )

        self._set_topic_prefixes(resolve_mqtt_topic_prefix())

        self._mqtt_adapter = None
        if use_cw and CyberwaveAdapter is not None:
            if token:
                try:
                    self.get_logger().info("=" * 60)
                    self.get_logger().info("Initializing Cyberwave SDK MQTT Connection")
                    self.get_logger().info("=" * 60)
                    self._mqtt_adapter = CyberwaveAdapter(
                        broker=host,
                        port=port,
                        api_token=token,
                        base_url=self._edge_env.base_url or None,
                        mqtt_use_tls=mqtt_use_tls,
                        topic_prefix=self.topic_prefix,
                        auto_connect=True,
                        logger=self.get_logger(),
                        loop=self._async_loop,
                    )
                    self.get_logger().info(
                        "✓ Cyberwave SDK adapter initialized successfully"
                    )
                    self.get_logger().info(
                        "  Connection method: Cyberwave SDK (PRIMARY)"
                    )
                    self.get_logger().info("=" * 60)
                except Exception as e:
                    self._mqtt_adapter = None
                    self.get_logger().error(
                        f"Failed to initialize Cyberwave adapter with token: {e}"
                    )
                    self.get_logger().warning("Will fall back to paho-mqtt client")
            else:
                self.get_logger().error(
                    "Cyberwave API token not found! 'use_cyberwave' is enabled but no "
                    "token was provided via parameters or CYBERWAVE_API_KEY / "
                    "CYBERWAVE_TOKEN. Video streaming will be unavailable."
                )

        # create paho MQTT client and callbacks (fallback)
        self._mqtt_client = mqtt.Client()
        self._mqtt_client.user_data_set(self)
        self._mqtt_client.on_connect = _paho_on_connect
        self._mqtt_client.on_message = _paho_on_message

        if username and password:
            self._mqtt_client.username_pw_set(username, password)
            self.get_logger().info(
                f"MQTT authentication configured for user: {username}"
            )

        # Keep paho client in sync (created after _set_topic_prefixes above).
        self._mqtt_client.topic_prefix = self.topic_prefix

        # Mapping support (per-robot json_by_name mappings)
        # declare mapping-related params
        self.declare_parameter("robot_id", "default")
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("mapping_reload_on_change", False)
        # If true, treat missing twin_uuid in mapping as an error
        self.declare_parameter("mapping_require_digital_twin", True)

        self._internal_odom = None
        self._navigation_bridge = None
        self._telemetry_processor = None

        # Mapping support (per-robot json_by_name mappings)
        try:
            self._load_mapping()

            # Log Twin UUID from mapping after it's loaded
            if self._mapping and getattr(self._mapping, "twin_uuid", None):
                self.get_logger().info(f"--- Robot Mapping Loaded ---")
                self.get_logger().info(
                    f"ROBOT_ID: {self.get_parameter('robot_id').value}"
                )
                self.get_logger().info(f"TWIN_UUID: {self._mapping.twin_uuid}")
                self.get_logger().info(f"-----------------------------")
        except Exception as e:
            self.get_logger().warning(f"Could not load mapping: {e}")

        # params.yaml camera overrides merged onto the mapping (one place so the
        # manager, streamer, and track agree).
        self._apply_camera_param_overrides()

        # managed_by_bridge: bridge owns the usb_cam process (runtime format
        # changes, no respawn EBUSY race); else the static launch node is used.
        self._camera_manager = None
        try:
            camera_cfg = self._mapping.raw.get("camera", {}) if self._mapping else {}
            if camera_cfg.get("managed_by_bridge"):
                # Launch usb_cam under the SAME namespace the bridge subscribes in,
                # so /<ns>/image_raw matches (else the track receives 0 frames).
                self._camera_manager = CameraDeviceManager(
                    camera_cfg,
                    log=self.get_logger(),
                    namespace=self._resolve_ros_namespace(),
                )
                pf = camera_cfg.get("pixel_format", "mjpeg2rgb")
                w = int(camera_cfg.get("image_width", 1920))
                h = int(camera_cfg.get("image_height", 1080))
                fps = int(camera_cfg.get("capture_fps", camera_cfg.get("fps", 30)))
                try:
                    self._camera_manager.start(pf, w, h, fps)
                    self.get_logger().info(
                        f"CameraDeviceManager started usb_cam: {pf} {w}x{h}@{fps} "
                        f"on {self._camera_manager.video_device}"
                    )
                except FormatValidationError as exc:
                    self.get_logger().error(f"Initial camera format invalid: {exc}")
                except Exception as exc:
                    self.get_logger().error(f"Failed to start managed camera: {exc}")
        except Exception as exc:
            self.get_logger().warning(f"Camera manager init skipped: {exc}")

        # Initialize camera streamer proactively to pre-cache frames
        self._ros_streamer = None
        self._webrtc_start_future = None  # Track ongoing WebRTC start operation
        try:
            twin_uuid = getattr(self._mapping, "twin_uuid", None)
            if twin_uuid:
                from cyberwave.utils import TimeReference

                self.get_logger().info(
                    "Pre-initializing ROSCameraStreamer to cache frames..."
                )

                # Configure STUN/TURN servers for NAT traversal
                ice_servers = [
                    {"urls": ["stun:stun.l.google.com:19302"]},
                    {"urls": ["stun:turn.cyberwave.com:3478"]},
                    {
                        "urls": ["turn:turn.cyberwave.com:3478"],
                        "username": "cyberwave-user",
                        "credential": "cyberwave-admin",
                    },
                ]

                # Check if force_turn is enabled to bypass NAT/firewall issues
                force_turn = bool(self.get_parameter("webrtc.force_turn").value)
                if force_turn:
                    self.get_logger().info(
                        "WebRTC force_turn ENABLED - all media will be relayed through TURN server"
                    )
                    # Relay-only ICE needs outbound reach to TURN; a blocked path
                    # is the usual silent no-media failure in Docker, so surface it.
                    try:
                        from .plugins.webrtc_preflight import preflight_turn

                        pf = preflight_turn(ice_servers, timeout=3.0)
                        if pf.get("reachable"):
                            self.get_logger().info(
                                f"TURN/STUN preflight OK via {pf.get('url')} "
                                f"(mapped={pf.get('mapped')})"
                            )
                        else:
                            self.get_logger().error(
                                f"TURN/STUN preflight FAILED ({pf.get('error')}). "
                                "Relay-only WebRTC may not connect — check container "
                                "outbound UDP to the TURN server."
                            )
                    except Exception as exc:
                        self.get_logger().debug(f"TURN preflight skipped: {exc}")

                # Use SDK's native mqtt client for the streamer if available.
                # The BaseVideoStreamer from the SDK expects the SDK's mqtt object
                # which has methods like publish_webrtc_message(), subscribe_webrtc_messages()
                # that are used internally for WebRTC signaling.
                mqtt_client = None
                if self._mqtt_adapter is not None:
                    # Get the SDK's native mqtt object from the adapter
                    mqtt_client = getattr(self._mqtt_adapter, "sdk_mqtt", None)
                    if mqtt_client is not None:
                        self.get_logger().info(
                            "Using SDK's native MQTT client for WebRTC streaming"
                        )
                    else:
                        # Fall back to adapter itself (which implements publish/subscribe)
                        mqtt_client = self._mqtt_adapter
                        self.get_logger().info(
                            "Using CyberwaveAdapter for WebRTC streaming"
                        )

                if mqtt_client is None:
                    self.get_logger().warning(
                        "No MQTT client available for WebRTC streaming"
                    )
                else:
                    # Store SDK Twin reference for high-level streaming control
                    # The Twin API (start_streaming/stop_streaming) is preferred over
                    # run_with_auto_reconnect() to avoid "Unknown command type" warnings
                    try:
                        if self._mqtt_adapter:
                            self._sdk_twin = self._mqtt_adapter.twin(twin_uuid)
                            if self._sdk_twin:
                                self.get_logger().info(
                                    f"SDK Twin object initialized for {twin_uuid}"
                                )
                            else:
                                self.get_logger().info(
                                    f"SDK Twin object is None for {twin_uuid} (may initialize later)"
                                )
                    except Exception as e:
                        self.get_logger().info(
                            f"SDK Twin not available yet: {e} (this is normal, will use custom streamer)"
                        )
                        self._sdk_twin = None

                    self._ros_streamer = ROSCameraStreamer(
                        node=self,
                        force_relay=force_turn,
                        client=mqtt_client,
                        twin_uuid=twin_uuid,
                        fps=self.get_parameter("webrtc.fps").value,
                        time_reference=TimeReference(),
                        turn_servers=ice_servers,
                    )
                    # Initialize the track immediately to start /image_raw subscription
                    self._ros_streamer.initialize_track()
                    self.get_logger().info(
                        "ROSCameraStreamer pre-initialized and track subscribed."
                    )
        except Exception as e:
            self.get_logger().warning(
                f"Failed to pre-initialize ROSCameraStreamer: {e}"
            )

        # Initialize telemetry processor
        self._telemetry_processor = TelemetryProcessor(self, self._internal_odom)

        # Initialize navigation bridge plugin
        self._navigation_bridge = NavigationBridge(self)

        # Initialize command router registry if specified in mapping
        self._command_registry = None
        self._command_registry_init_attempts = 0
        self._try_init_command_registry()

        # Service to trigger a mapping reload at runtime (hot-reload).
        # Create a node-scoped service name so callers can call
        # /<node_name>/reload_mapping which is consistent with the node
        # namespace. Avoid creating the legacy global '/reload_mapping'.
        try:
            ns = self.get_namespace() or ""
            if not ns.startswith("/"):
                ns = f"/{ns}"
            ns = ns.rstrip("/")

            # Reload Mapping Service
            fq_reload_name = f"{ns}/{self.get_name()}/reload_mapping"
            self._reload_srv = self.create_service(
                Trigger, fq_reload_name, self._on_reload_mapping
            )

            # Start Camera Service
            fq_start_video_name = f"{ns}/{self.get_name()}/start_video"
            self._start_video_srv = self.create_service(
                Trigger, fq_start_video_name, self._on_start_video
            )

            # Stop Camera Service
            fq_stop_video_name = f"{ns}/{self.get_name()}/stop_video"
            self._stop_video_srv = self.create_service(
                Trigger, fq_stop_video_name, self._on_stop_video
            )

            self.get_logger().info(
                f"WebRTC Video Services initialized: {fq_start_video_name}, {fq_stop_video_name}"
            )

        except Exception as e:
            self._reload_srv = None
            self._start_video_srv = None
            self._stop_video_srv = None
            self.get_logger().warning(f"Could not create ROS services: {e}")

        # Optional early WebRTC start to have the edge ready before frontend connects
        self._auto_start_timer = None
        try:
            auto_start = bool(self.get_parameter("webrtc.auto_start").value)
            auto_start_delay = float(
                self.get_parameter("webrtc.auto_start_delay_sec").value
            )
            auto_start_retry = float(
                self.get_parameter("webrtc.auto_start_retry_sec").value
            )
        except Exception:
            auto_start = False
            auto_start_delay = 10.0
            auto_start_retry = 5.0

        if auto_start:
            delay = max(0.0, auto_start_delay)
            self._auto_start_retry_sec = max(1.0, auto_start_retry)
            self.get_logger().info(
                f"WebRTC auto-start enabled, scheduling start in {delay:.1f}s"
            )
            self._auto_start_timer = self.create_timer(
                delay, self._maybe_auto_start_webrtc
            )

        # Optionally start a background watcher thread that auto-reloads the
        # mapping file if it changes on disk. Controlled by mapping_reload_on_change param.
        try:
            mapping_reload_on_change = bool(
                self.get_parameter("mapping_reload_on_change").value
            )
        except Exception:
            mapping_reload_on_change = False

        self._mapping_watcher_stop = None
        self._mapping_watcher_thread = None
        if mapping_reload_on_change and getattr(self, "_mapping", None) is not None:
            try:
                self._mapping_watcher_stop = threading.Event()
                self._mapping_watcher_thread = threading.Thread(
                    target=self._mapping_watcher_loop, daemon=True
                )
                self._mapping_watcher_thread.start()
                self.get_logger().info("Started mapping watcher thread")
            except Exception as e:
                self.get_logger().warning(f"Could not start mapping watcher: {e}")

        # parse ros2mqtt mappings (ROS -> MQTT)
        ros2mqtt = bridge.get("ros2mqtt", {})
        self.get_logger().info(f"Loaded bridge config: {bridge}")
        ros_topics = ros2mqtt.get("ros_topics", []) or []
        for ros_topic in ros_topics:
            # Support two params layouts:
            # 1) per-topic mappings as direct keys under ros2mqtt: ros2mqtt[ros_topic]
            # 2) nested under ros2mqtt['topics'][ros_topic]
            mapping = ros2mqtt.get(ros_topic, {}) or {}
            if not mapping:
                mapping = (ros2mqtt.get("topics", {}) or {}).get(ros_topic, {}) or {}
            mqtt_topic_template = mapping.get("mqtt_topic", ros_topic.lstrip("/"))
            mqtt_topic = mqtt_topic_template

            if (
                "{twin_uuid}" in mqtt_topic
                and self._mapping is not None
                and hasattr(self._mapping, "twin_uuid")
                and self._mapping.twin_uuid
            ):
                mqtt_topic = mqtt_topic.replace("{twin_uuid}", self._mapping.twin_uuid)

            if (
                hasattr(self, "ros_prefix")
                and self.ros_prefix
                and not mqtt_topic.startswith(self.ros_prefix)
            ):
                mqtt_topic = f"{self.ros_prefix}{mqtt_topic}"

            type_str = mapping.get("type")
            msg_cls = self._resolve_msg_class(type_str)
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
            )

            resolved_ros_topic = self.resolve_ros_topic(ros_topic)
            self.get_logger().info(
                f"Bridge ROS -> MQTT: {resolved_ros_topic} -> {mqtt_topic}"
            )
            sub = self.create_subscription(
                msg_cls,
                resolved_ros_topic,
                self._make_ros_cb(resolved_ros_topic, mqtt_topic, msg_cls),
                qos,
            )
            self._ros2mqtt_map[resolved_ros_topic] = mqtt_topic
            # capture optional sdk_method hint from params.yaml so the
            # bridge can call SDK helpers (e.g., update_joint_state)
            sdk_method = (
                mapping.get("sdk_method") if isinstance(mapping, dict) else None
            )
            if sdk_method:
                self._ros2mqtt_sdk_method[resolved_ros_topic] = sdk_method
            self._ros2mqtt_msgtypes[resolved_ros_topic] = msg_cls

            # capture optional custom rate limit or interval for this topic
            custom_rate = mapping.get("rate") or mapping.get("rate_limit")
            custom_interval = mapping.get("interval")
            if custom_rate is not None:
                self._ros2mqtt_custom_intervals[resolved_ros_topic] = 1.0 / float(
                    custom_rate
                )
            elif custom_interval is not None:
                self._ros2mqtt_custom_intervals[resolved_ros_topic] = float(
                    custom_interval
                )

        # parse mqtt2ros mappings (MQTT -> ROS)
        mqtt2ros = bridge.get("mqtt2ros", {})
        # Support either an explicit list of mqtt_topics or per-topic mappings
        # under mqtt2ros['topics']. To be robust we subscribe to the union of
        # both sources so entries defined in either place are handled.
        mqtt_topics = mqtt2ros.get("mqtt_topics", []) or []
        per_topic_keys = list((mqtt2ros.get("topics", {}) or {}).keys())
        # maintain original order as much as possible: explicit list first
        # then any additional keys from topics mapping
        topics_to_subscribe = list(mqtt_topics) + [
            t for t in per_topic_keys if t not in mqtt_topics
        ]
        for mqtt_topic_template in topics_to_subscribe:
            # Clean up template string
            mqtt_topic_template = mqtt_topic_template.strip()
            mqtt_topic = mqtt_topic_template

            # Replace {twin_uuid} template placeholder with actual twin_uuid from mapping
            if (
                "{twin_uuid}" in mqtt_topic
                and self._mapping is not None
                and hasattr(self._mapping, "twin_uuid")
                and self._mapping.twin_uuid
            ):
                twin_uuid = self._mapping.twin_uuid
                mqtt_topic = mqtt_topic.replace("{twin_uuid}", twin_uuid)

            # Apply ros_prefix to MQTT topic (consistent with upstream topics)
            if hasattr(self, "ros_prefix") and self.ros_prefix:
                if not mqtt_topic.startswith(self.ros_prefix):
                    mqtt_topic = f"{self.ros_prefix}{mqtt_topic}"

            # Support both flat and nested 'topics' layouts for mqtt2ros as well
            mapping = mqtt2ros.get(mqtt_topic_template, {}) or {}
            if not mapping:
                mapping = (mqtt2ros.get("topics", {}) or {}).get(
                    mqtt_topic_template, {}
                ) or {}

            if mqtt_topic.endswith("/command"):
                if mqtt_topic not in self._mqtt_callbacks:
                    self._mqtt_callbacks[mqtt_topic] = []
                continue

            # Handle list of mappings (multiple ROS targets for one MQTT topic)
            pubs = []
            sdk_methods = []

            if isinstance(mapping, list):
                # List of mappings: each item defines a separate ROS topic target
                self.get_logger().info(
                    f"Processing multi-target MQTT->ROS bridge: {mqtt_topic} -> {len(mapping)} targets"
                )
                for map_entry in mapping:
                    if not isinstance(map_entry, dict):
                        continue
                    ros_topic = map_entry.get(
                        "ros_topic",
                        mqtt_topic if mqtt_topic.startswith("/") else f"/{mqtt_topic}",
                    )
                    type_str = map_entry.get("type")
                    msg_cls = self._resolve_msg_class(type_str)
                    sdk_method = map_entry.get("sdk_method")
                    if sdk_method:
                        sdk_methods.append(sdk_method)

                    # Process this mapping entry (ros_topic should be a single string in multi-target configs)
                    rt_name = self.resolve_ros_topic(ros_topic)
                    self.get_logger().info(f"  Target: {rt_name} ({msg_cls.__name__})")
                    existing = self._ros_topic_msgtype.get(rt_name)
                    if existing is not None and existing is not msg_cls:
                        self.get_logger().error(
                            f"Topic {rt_name} already has publisher with type {existing.__name__}; skipping incompatible mapping from {mqtt_topic} ({msg_cls.__name__})"
                        )
                        continue
                    try:
                        pub = self.create_publisher(msg_cls, rt_name, 10)
                        pubs.append((pub, msg_cls, rt_name))
                        self._ros_topic_msgtype[rt_name] = msg_cls
                    except Exception as e:
                        self.get_logger().error(
                            f"Failed to create publisher for {rt_name} ({msg_cls.__name__}): {e} - skipping mapping from {mqtt_topic}"
                        )
                        continue
            else:
                # Single mapping dict - extract fields
                ros_topic = mapping.get(
                    "ros_topic",
                    mqtt_topic if mqtt_topic.startswith("/") else f"/{mqtt_topic}",
                )
                type_str = mapping.get("type")
                msg_cls = self._resolve_msg_class(type_str)
                sdk_method = mapping.get("sdk_method")
                if sdk_method:
                    sdk_methods.append(sdk_method)

                # ros_topic can be a single string or a list of topics
                if isinstance(ros_topic, list):
                    self.get_logger().info(
                        f"Creating MQTT->ROS bridge: {mqtt_topic} -> {ros_topic} ({msg_cls.__name__})"
                    )
                    for rt in ros_topic:
                        rt_name = self.resolve_ros_topic(rt)
                        existing = self._ros_topic_msgtype.get(rt_name)
                        if existing is not None and existing is not msg_cls:
                            self.get_logger().error(
                                f"Topic {rt_name} already has publisher with type {existing.__name__}; skipping incompatible mapping from {mqtt_topic} ({msg_cls.__name__})"
                            )
                            continue
                        try:
                            pub = self.create_publisher(msg_cls, rt_name, 10)
                        except Exception as e:
                            self.get_logger().error(
                                f"Failed to create publisher for {rt_name} ({msg_cls.__name__}): {e} - skipping mapping from {mqtt_topic}"
                            )
                            continue
                        pubs.append((pub, msg_cls, rt_name))
                        self._ros_topic_msgtype[rt_name] = msg_cls
                else:
                    # Handle single topic case
                    rt_name = self.resolve_ros_topic(ros_topic)
                    self.get_logger().info(
                        f"Creating MQTT->ROS bridge: {mqtt_topic} -> {rt_name} ({msg_cls.__name__})"
                    )
                    existing = self._ros_topic_msgtype.get(rt_name)
                    if existing is not None and existing is not msg_cls:
                        self.get_logger().error(
                            f"Topic {rt_name} already has publisher with type {existing.__name__}; skipping incompatible mapping from {mqtt_topic} ({msg_cls.__name__})"
                        )
                    else:
                        try:
                            pub = self.create_publisher(msg_cls, rt_name, 10)
                            pubs.append((pub, msg_cls, rt_name))
                            self._ros_topic_msgtype[rt_name] = msg_cls
                        except Exception as e:
                            self.get_logger().error(
                                f"Failed to create publisher for {rt_name} ({msg_cls.__name__}): {e} - skipping mapping from {mqtt_topic}"
                            )

            if pubs:
                self._mqtt2ros_pubs[mqtt_topic] = pubs
            # store sdk_method(s) if any — keep single string or list for backward compatibility
            if sdk_methods:
                sdk_methods = list(dict.fromkeys(sdk_methods))
                self._mqtt2ros_sdk_method[mqtt_topic] = (
                    sdk_methods[0] if len(sdk_methods) == 1 else sdk_methods
                )

        # Store host and port for logging
        self._mqtt_host = host
        self._mqtt_port = port

        # Subscribe to /joint_states to initialize accumulated state with current robot position
        self._joint_states_sub = self.create_subscription(
            JointState,
            self.resolve_ros_topic("/joint_states"),
            self._on_joint_states,
            10,
        )

        # Initialize IO / Tool client if specified in mapping
        self._robot_io_client = None
        self._io_config = {}
        if self._mapping is not None and self._mapping.io_configuration:
            self._io_config = self._mapping.io_configuration
            for tool_name, config in self._io_config.items():
                if config.get("enabled") and "service_name" in config:
                    srv_name = self.resolve_ros_topic(config["service_name"])
                    srv_type_str = config.get("service_type", "std_srvs/srv/Trigger")

                    # Dynamically resolve service type if possible
                    # For simplicity, we check specifically for SetIO since it's common
                    if srv_type_str == "ur_msgs/srv/SetIO" and SetIO is not None:
                        self._robot_io_client = self.create_client(SetIO, srv_name)
                        self.get_logger().info(
                            f"Initialized IO client for {tool_name} on {srv_name}"
                        )

        # Subscribe to joint updates for the specific twin_uuid
        if (
            self._mapping is not None
            and hasattr(self._mapping, "twin_uuid")
            and self._mapping.twin_uuid
        ):
            twin_uuid = self._mapping.twin_uuid
            joint_update_topic = f"{self.ros_prefix}cyberwave/joint/{twin_uuid}/update"

            # Check if we should register the tool control callback
            if self._io_config.get("tool0", {}).get("enabled"):
                self._mqtt_callbacks.setdefault(joint_update_topic, []).append(
                    self._on_tool_control_update
                )
                self.get_logger().info(
                    f"Registered tool control callback for {joint_update_topic}"
                )
        else:
            self.get_logger().warning(
                "No twin_uuid available - tool0 joint control will not be available. "
                "Set 'twin_uuid' in the mapping file or export CYBERWAVE_TWIN_UUID."
            )

        # try connect
        try:
            if self._mqtt_adapter is None:
                # Fallback to paho MQTT client
                self.get_logger().info("=" * 60)
                self.get_logger().info(
                    "Establishing MQTT Connection: paho-mqtt (FALLBACK)"
                )
                self.get_logger().info("=" * 60)
                self._mqtt_client.connect(host, port)
                self._mqtt_client.loop_start()
                self.get_logger().info("✓ Connected using paho-mqtt client")
            else:
                # Using Cyberwave adapter (already connected via auto_connect)
                self.get_logger().info("=" * 60)
                self.get_logger().info(
                    "Establishing MQTT Connection: Cyberwave SDK (PRIMARY)"
                )
                self.get_logger().info("=" * 60)
                adapter = self._mqtt_adapter
                self._wait_for_adapter_connection(adapter)
                self.get_logger().info("✓ Cyberwave SDK MQTT connection established")
                self.get_logger().info("=" * 60)

                # Subscribe to all configured MQTT->ROS topics
                for topic in list(self._mqtt2ros_pubs.keys()):
                    try:
                        self._subscribe_to_mqtt_topic(adapter, topic)
                    except Exception as e:
                        self.get_logger().error(
                            f"Failed to subscribe (adapter) to {topic}: {e}"
                        )

                # Subscribe to general callback topics (e.g., ping)
                for topic in list(self._mqtt_callbacks.keys()):
                    try:
                        qos = self._mqtt_command_qos if "/command" in topic else 0
                        adapter.subscribe(
                            topic, on_message=self._handle_mqtt_message, qos=qos
                        )
                        self.get_logger().info(
                            f"Subscribed (adapter) to MQTT topic '{topic}' (QoS {qos})"
                        )
                    except Exception as e:
                        self.get_logger().error(
                            f"Failed to subscribe (adapter) to {topic}: {e}"
                        )
        except Exception as e:
            self.get_logger().error(f"Could not connect to MQTT broker: {e}")

        # subscribe to configured MQTT topics once connected (on_connect will re-subscribe)
        # Ping handling is intentionally omitted here (no _on_ping callback).

    def _set_topic_prefixes(self, prefix: str) -> None:
        """Set ``topic_prefix`` and ``ros_prefix`` together for ``{prefix}cyberwave/...``."""
        self.topic_prefix = prefix
        self.ros_prefix = self.topic_prefix
        example_topic = f"{self.topic_prefix}cyberwave/twin/<uuid>/command"
        self.get_logger().info(
            f"MQTT topic prefix: '{self.topic_prefix}' "
            f"(ros_prefix='{self.ros_prefix}', example: {example_topic})"
        )

    def _resolve_ros_namespace(self) -> str:
        """Resolve the per-robot ROS namespace.

        Honors an explicit ``ros_namespace`` param (set by
        master_beast.launch.py); otherwise derives ``ugv_beast_<first6 of twin
        uuid>`` so multiple UGV Beasts can share a ROS 2 graph without colliding.

        The fallback derives from the ``CYBERWAVE_TWIN_UUID`` environment
        variable — the SAME source master_beast.launch.py uses for the hardware
        nodes — so the bridge and hardware always land on identical topics even
        if the launch did not pass ``ros_namespace`` explicitly. Empty → global.
        """
        try:
            configured = str(self.get_parameter("ros_namespace").value or "")
        except Exception:
            configured = ""
        # Mirror master_beast.launch.py precedence exactly so the bridge and the
        # hardware nodes always agree, whether or not the launch passed the param:
        #   explicit ros_namespace param > CYBERWAVE_ROS_NAMESPACE env >
        #   derived from CYBERWAVE_TWIN_UUID env.
        override = os.getenv("CYBERWAVE_ROS_NAMESPACE", "").strip().strip("/")
        twin_uuid = os.getenv("CYBERWAVE_TWIN_UUID", "")
        return resolve_ros_namespace(
            configured_namespace=configured or override,
            twin_uuid=twin_uuid or None,
        )

    def resolve_ros_topic(self, topic_name: str) -> str:
        """Return a fully-qualified ROS topic in the active namespace."""
        return _resolve_ros_topic(topic_name, self._resolve_ros_namespace())

    def _get_virtual_joints(self) -> typing.Set[str]:
        """Return a set of joint names that are used for virtual IO/control."""
        if not hasattr(self, "_io_config") or not self._io_config:
            return set()
        return {
            config.get("joint_name")
            for config in self._io_config.values()
            if config.get("joint_name")
        }

    def _resolve_msg_class(self, type_str: typing.Optional[str]) -> typing.Any:
        if not type_str:
            return String
        # accept both 'pkg/Type' and 'Type'
        return self._type_map.get(type_str, String)

    def _wait_for_adapter_connection(self, adapter, timeout: float = 5.0) -> None:
        """Wait for adapter to report connected, with timeout."""
        self.get_logger().info(
            f"Waiting for Cyberwave SDK connection (timeout: {timeout}s)..."
        )
        poll_interval = 0.1
        waited = 0.0
        while not getattr(adapter, "connected", False) and waited < timeout:
            time.sleep(poll_interval)
            waited += poll_interval

        if getattr(adapter, "connected", False):
            self.get_logger().info(f"✓ Connection confirmed after {waited:.2f}s")
        else:
            self.get_logger().warning(
                f"Connection not confirmed after {timeout}s (may still be connecting)"
            )

    def _get_or_create_publisher(
        self, topic: str, msg_cls: typing.Any
    ) -> typing.Optional[typing.Any]:
        """Get existing publisher or create a new one for the given topic and message type.

        Returns the publisher instance or None on failure.
        """
        topic = self.resolve_ros_topic(topic)
        existing = self._ros_topic_msgtype.get(topic)

        if existing is not None and existing is not msg_cls:
            self.get_logger().error(
                f"Cannot create publisher for {topic}: existing publisher with different type {existing.__name__}"
            )
            return None

        if existing is None:
            try:
                pub = self.create_publisher(msg_cls, topic, 10)
                self._ros_topic_msgtype[topic] = msg_cls
                return pub
            except Exception as e:
                self.get_logger().error(f"Failed to create publisher for {topic}: {e}")
                return None
        else:
            # Find existing publisher in mqtt2ros_pubs
            for pubs in self._mqtt2ros_pubs.values():
                for p, pc, name in pubs:
                    if name == topic and pc is msg_cls:
                        return p
            return None

    def _create_joint_command_callback(
        self, twin_uuid: str, cmd_pub, cmd_topic: str, cmd_msg_cls: typing.Any
    ) -> typing.Callable[[typing.Any], None]:
        """Create a callback that converts SDK joint state updates into the
        appropriate ROS message class expected by the publisher.

        The SDK update payloads are often simple per-joint numeric updates.
        This factory will build either a Float64MultiArray (default) or a
        JointTrajectory (single-point) when required by the publisher.
        """

        def _on_update(fake_msg):
            payload = getattr(fake_msg, "payload", b"")

            # normalize payload to text
            if isinstance(payload, (bytes, bytearray)):
                try:
                    payload = payload.decode("utf-8", errors="replace")
                except Exception:
                    payload = str(payload)
            else:
                payload = str(payload)

            try:
                data = json.loads(payload)
            except Exception:
                self.get_logger().debug("on_update: payload not JSON, ignoring")
                return

            source_type = data.get("source_type") if isinstance(data, dict) else None
            if source_type != SOURCE_TYPE_TELE:
                self.get_logger().info(
                    f"Ignoring joint update from {source_type} (only '{SOURCE_TYPE_TELE}' allowed)"
                )
                return

            jstate = data.get("joint_state") if isinstance(data, dict) else None
            if not isinstance(jstate, dict):
                return

            # extract joint name if present (SDK payloads often include it)
            jname_mqtt = data.get("joint_name") if isinstance(data, dict) else None
            # normalize joint name to string
            if jname_mqtt is not None and not isinstance(jname_mqtt, str):
                try:
                    jname_mqtt = str(jname_mqtt)
                    self.get_logger().debug(
                        f"Normalized joint_name to string: {jname_mqtt}"
                    )
                except Exception:
                    pass

            def _first_numeric(v):
                if v is None:
                    return None
                if isinstance(v, (list, tuple)) and v:
                    v = v[0]
                try:
                    return float(str(v))
                except Exception:
                    return None

            val = next(
                (
                    _first_numeric(jstate.get(k))
                    for k in ("position", "velocity", "effort")
                ),
                None,
            )
            if val is None:
                return

            mapping = getattr(self, "_mapping", None)
            # If we have a mapping with joint_names, maintain a per-twin array
            if mapping is not None and getattr(mapping, "joint_names", None):
                try:
                    n = len(mapping.joint_names)
                    # get or init stored array for this twin
                    arr = self._last_joint_values_by_twin.get(twin_uuid)
                    if not isinstance(arr, list) or len(arr) != n:
                        # Try to initialize from accumulated state if available
                        initial_state = self._accumulated_joint_states.get(
                            "initial_joint_state"
                        )
                        if initial_state and len(initial_state) == n:
                            arr = list(initial_state)
                        else:
                            arr = [0.0] * n

                    # find index for this mqtt joint name (fallback to 0)
                    idx = None
                    if jname_mqtt:
                        ros_name = mapping.mqtt_to_name.get(jname_mqtt) or jname_mqtt
                        try:
                            idx = list(mapping.joint_names).index(ros_name)
                        except ValueError:
                            idx = None
                    if idx is None:
                        idx = 0

                    # apply reverse transform (mqtt->ros) so mapping.transform
                    # options such as 'invert' are respected when publishing
                    try:
                        ros_name_for_idx = mapping.joint_names[idx]
                        rev = mapping.reverse_transforms.get(
                            ros_name_for_idx, lambda x: x
                        )
                        transformed = rev(val)
                        # rev may already return float; coerce defensively
                        new_val = float(transformed)
                    except Exception:
                        try:
                            new_val = float(val)
                        except Exception:
                            # cannot coerce value; skip
                            return

                    # only update stored value when it actually differs
                    old_val = None
                    try:
                        old_val = float(arr[idx])
                    except Exception:
                        old_val = None
                    if old_val is None or old_val != new_val:
                        arr[idx] = new_val
                        # save a copy
                        self._last_joint_values_by_twin[twin_uuid] = list(arr)

                    # Determine message type based on provided class
                    msg_type = cmd_msg_cls

                    if msg_type is JointTrajectory:
                        ros_msg = JointTrajectory()
                        virtual_joints = self._get_virtual_joints()
                        joint_names_filtered = [
                            jn for jn in mapping.joint_names if jn not in virtual_joints
                        ]
                        positions_filtered = [
                            arr[i]
                            for i, jn in enumerate(mapping.joint_names)
                            if jn not in virtual_joints
                        ]

                        ros_msg.joint_names = joint_names_filtered
                        # Calculate safe trajectory time based on distance and velocity limits
                        trajectory_time = self._calculate_trajectory_time(
                            positions_filtered, joint_names_filtered
                        )
                        point = JointTrajectoryPoint()
                        point.positions = positions_filtered
                        # set velocities to empty list
                        point.velocities = []
                        point.accelerations = []
                        point.time_from_start.sec = int(trajectory_time)
                        point.time_from_start.nanosec = int(
                            (trajectory_time - int(trajectory_time)) * 1e9
                        )
                        ros_msg.points = [point]
                        ros_msg.header.stamp = self.get_clock().now().to_msg()
                        ros_msg.header.frame_id = (
                            str(source_type) if source_type else ""
                        )
                    elif msg_type is JointState:
                        ros_msg = JointState()
                        ros_msg.header.stamp = self.get_clock().now().to_msg()
                        ros_msg.header.frame_id = (
                            str(source_type) if source_type else ""
                        )
                        ros_msg.name = list(mapping.joint_names)
                        ros_msg.position = list(arr)
                    else:
                        ros_msg = Float64MultiArray()
                        ros_msg.data = list(arr)
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to build joint array for twin {twin_uuid}: {e}"
                    )
                    return
            else:
                # no mapping available: single-element message
                data_list = [val]
                if cmd_msg_cls is Float64MultiArray:
                    ros_msg = Float64MultiArray()
                    ros_msg.data = data_list
                elif cmd_msg_cls is JointTrajectory:
                    jt = JointTrajectory()
                    jt.joint_names = []
                    pt = JointTrajectoryPoint()
                    pt.positions = data_list
                    # Explicitly set velocities to empty list
                    pt.velocities = []
                    pt.accelerations = []
                    jt.points = [pt]
                    ros_msg = jt
                else:
                    ros_msg = Float64MultiArray()
                    ros_msg.data = data_list

            if cmd_pub:
                try:
                    cmd_pub.publish(ros_msg)
                except Exception as e:
                    self.get_logger().error(
                        f"Failed to publish twin {twin_uuid} update to {cmd_topic}: {e}"
                    )
            else:
                self.get_logger().warning(
                    f"No publisher available to forward twin {twin_uuid} update to {cmd_topic}"
                )

        return _on_update

    def _subscribe_with_sdk_method(self, adapter, topic: str, sdk_method: str) -> bool:
        """Subscribe using SDK passthrough method. Returns True if successful."""
        twin_uuid = getattr(self._mapping, "twin_uuid", None) if self._mapping else None
        if not twin_uuid:
            return False

        method = getattr(adapter, sdk_method)

        # Special handling for subscribe_twin_joint_states
        if sdk_method == "subscribe_twin_joint_states":
            return self._subscribe_twin_joint_states(method, twin_uuid, topic)

        # Generic SDK method subscription
        try:
            method(twin_uuid, on_update=self._handle_mqtt_message)
        except TypeError:
            method(twin_uuid, self._handle_mqtt_message)

        self.get_logger().info(
            f"Subscribed (adapter.{sdk_method}) to twin '{twin_uuid}' for MQTT topic '{topic}'"
        )
        return True

    def _subscribe_twin_joint_states(
        self, method, twin_uuid: str, mqtt_topic: str
    ) -> bool:
        """Handle subscribe_twin_joint_states special case.

        Infer the ROS message type from the existing mqtt2ros mapping. If the
        mapping for the MQTT topic exists the associated ROS topic and message
        class are used. If no mapping exists we fall back to Float64MultiArray
        but log the fallback.
        """
        # Determine ROS topic and message class from configured mappings
        pubs = self._mqtt2ros_pubs.get(mqtt_topic, [])
        ros_topic = self.resolve_ros_topic("/joint/commands")
        msg_cls = None
        if pubs:
            # Prefer a publisher that targets the canonical joint commands topic
            for _, pc, name in pubs:
                if name == ros_topic:
                    msg_cls = pc
                    break
            # otherwise use the first mapping's message class
            if msg_cls is None:
                _, msg_cls, name = pubs[0]
                ros_topic = name

        if msg_cls is None:
            # No mapping found — fall back to previous behaviour but warn
            self.get_logger().warning(
                f"No mqtt2ros mapping found for {mqtt_topic}; defaulting to Float64MultiArray for {ros_topic}"
            )
            msg_cls = Float64MultiArray

        pubs = self._mqtt2ros_pubs.get(mqtt_topic, [])
        if not pubs:
            self.get_logger().warning(
                f"No publishers configured for {mqtt_topic}; cannot use subscribe_twin_joint_states"
            )
            return False

        # Build per-publisher callbacks and compose a single adapter callback
        callbacks = []
        for pub, pc, rt_name in pubs:
            cb = self._create_joint_command_callback(twin_uuid, pub, rt_name, pc)
            callbacks.append(cb)

        def adapter_on_update(fake_msg):
            # forward the adapter update to each per-pub callback; exceptions per
            # callback are isolated so one failing target doesn't stop others.
            for cb in callbacks:
                try:
                    cb(fake_msg)
                except Exception as e:
                    self.get_logger().error(
                        f"Adapter forwarding callback failed for {mqtt_topic}: {e}"
                    )

        try:
            method(twin_uuid, on_update=adapter_on_update)
        except TypeError:
            method(twin_uuid, adapter_on_update)

        self.get_logger().info(
            f"Subscribed adapter.subscribe_twin_joint_states -> {', '.join([rt for (_, _, rt) in pubs])}"
        )
        return True

    def _subscribe_to_mqtt_topic(self, adapter, topic: str) -> None:
        """Subscribe to a single MQTT topic, using SDK method if configured."""
        sdk_method = self._mqtt2ros_sdk_method.get(topic)

        # Try SDK method(s) first if configured. We accept either a single
        # method name or a list of methods and attempt them in order.
        methods_to_try = []
        if isinstance(sdk_method, (list, tuple)):
            methods_to_try = list(sdk_method)
        elif isinstance(sdk_method, str):
            methods_to_try = [sdk_method]

        for m in methods_to_try:
            if hasattr(adapter, m):
                try:
                    self._subscribe_with_sdk_method(adapter, topic, m)
                    # Continue to also subscribe to the plain MQTT topic pattern
                    # in case messages arrive via regular MQTT publish (not via SDK)
                except Exception as e:
                    self.get_logger().warning(
                        f"Adapter passthrough {m} failed for {topic}: {e}; falling back to plain subscribe"
                    )

        # Simplified: always subscribe using the topic without a leading slash
        try:
            normalized = topic.lstrip("/")
            qos = self._mqtt_command_qos if "/command" in normalized else 0
            adapter.subscribe(normalized, on_message=self._handle_mqtt_message, qos=qos)
            self.get_logger().info(
                f"Subscribed (adapter) to MQTT topic '{normalized}' (QoS {qos})"
            )
        except Exception as e:
            self.get_logger().warning(
                f"Adapter subscribe for '{normalized}' failed: {e}"
            )

    def _try_init_command_registry(self) -> bool:
        """Attempt to initialise the command registry from the mapping.

        Returns True if the registry is ready (already initialised or
        successfully created), False otherwise. Safe to call repeatedly;
        logs verbose tracebacks for the first few failures, then rate-limits.
        """
        _MAX_VERBOSE_ATTEMPTS = 5

        if self._command_registry is not None:
            return True

        if self._mapping is None or not self._mapping.command_registry:
            if self._command_registry_init_attempts == 0:
                self.get_logger().info("No command registry specified in mapping")
            return False

        self._command_registry_init_attempts += 1

        try:
            registry_path = self._mapping.command_registry
            module_path, class_name = registry_path.rsplit(".", 1)
            module = importlib.import_module(module_path)
            registry_class = getattr(module, class_name)

            self._command_registry = registry_class(self)

            cmd_topic_template = "cyberwave/twin/{twin_uuid}/command"
            cmd_topic = cmd_topic_template
            if self._mapping is not None and getattr(
                self._mapping, "twin_uuid", None
            ):
                cmd_topic = cmd_topic_template.replace(
                    "{twin_uuid}", self._mapping.twin_uuid
                )

            if hasattr(self, "ros_prefix") and self.ros_prefix:
                if not cmd_topic.startswith(self.ros_prefix):
                    cmd_topic = f"{self.ros_prefix}{cmd_topic}"

            self._command_registry.set_mqtt_context(
                self._mqtt_adapter or self._mqtt_client, cmd_topic
            )

            if cmd_topic not in self._mqtt_callbacks:
                self._mqtt_callbacks[cmd_topic] = []

            registered = self._command_registry.get_registered_commands()
            attempt = self._command_registry_init_attempts
            self.get_logger().info(
                f"Command router '{class_name}' initialized with "
                f"{len(registered)} handlers (attempt {attempt})"
            )
            return True
        except Exception as e:
            import time
            import traceback

            attempt = self._command_registry_init_attempts
            if attempt <= _MAX_VERBOSE_ATTEMPTS:
                self.get_logger().error(
                    f"Could not initialize command registry from "
                    f"{self._mapping.command_registry} (attempt {attempt}/"
                    f"{_MAX_VERBOSE_ATTEMPTS}): {e}\n{traceback.format_exc()}"
                )
                if attempt == _MAX_VERBOSE_ATTEMPTS:
                    self.get_logger().error(
                        "Command registry init still failing after "
                        f"{_MAX_VERBOSE_ATTEMPTS} attempts. Will keep retrying "
                        "on incoming commands with reduced logging. Check that "
                        "the command_registry class path in the mapping YAML is "
                        "correct and all dependencies are installed."
                    )
            else:
                now = time.time()
                last_log = getattr(self, "_last_registry_retry_log_ts", 0.0)
                if now - last_log >= 60.0:
                    self._last_registry_retry_log_ts = now
                    self.get_logger().warning(
                        "Command registry init still failing "
                        f"(attempt {attempt}): {e}"
                    )
            return False

    def _load_mapping(self) -> None:
        # Prefer explicit mapping_file param, otherwise robot_id -> default file
        mapping_file = self.get_parameter("mapping_file").value or ""
        robot_id = self.get_parameter("robot_id").value or ""
        pkg_share = get_package_share_directory("mqtt_bridge")
        mappings_dir = os.path.join(pkg_share, "config", "mappings")

        if mapping_file:
            path = mapping_file
        elif robot_id:
            path = os.path.join(mappings_dir, f"{robot_id}.yaml")
        else:
            path = os.path.join(mappings_dir, "default.yaml")

        # Make relative paths relative to package share
        if not os.path.isabs(path):
            path = (
                os.path.join(mappings_dir, path)
                if not path.startswith(mappings_dir)
                else path
            )

        try:
            self._mapping = Mapping(path)
            self.get_logger().info(f"Loaded mapping from {path}")

            # Log camera settings for verification
            camera_cfg = self._mapping.raw.get("camera", {})
            if camera_cfg:
                self.get_logger().info(
                    f"Camera Config: {camera_cfg.get('image_width')}x{camera_cfg.get('image_height')} "
                    f"@{camera_cfg.get('fps')}fps, format={camera_cfg.get('format')}, "
                    f"encoding={camera_cfg.get('encoding')}"
                )
            else:
                self.get_logger().warn("No camera configuration found in mapping!")

            # Initialize internal odometry plugin if enabled
            if self._mapping.internal_odometry.get("enabled"):
                self._internal_odom = InternalOdometry(self._mapping.internal_odometry)
                self.get_logger().info("Initialized internal odometry plugin")
            else:
                self._internal_odom = None

            if self._edge_env.twin_uuid:
                self._mapping.twin_uuid = self._edge_env.twin_uuid
            elif os.getenv("CYBERWAVE_TWIN_UUID"):
                self._mapping.twin_uuid = os.getenv("CYBERWAVE_TWIN_UUID")
                self.get_logger().info(
                    f"twin_uuid set from CYBERWAVE_TWIN_UUID env var: "
                    f"{self._mapping.twin_uuid}"
                )

            try:
                require_uuid = bool(
                    self.get_parameter("mapping_require_digital_twin").value
                )
            except Exception:
                require_uuid = False
            uuid = getattr(self._mapping, "twin_uuid", None)
            if not uuid and require_uuid:
                raise ValueError(
                    f"Mapping {path} missing 'twin_uuid'. "
                    "Set it in the mapping YAML or export CYBERWAVE_TWIN_UUID."
                )
        except Exception as e:
            self._mapping = None
            raise

    def _on_reload_mapping(self, request, response) -> typing.Any:
        try:
            if self._mapping is None:
                response.success = False
                response.message = "No mapping loaded"
                return response
            self._mapping.reload()
            response.success = True
            response.message = f"Reloaded mapping from {self._mapping.path}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"Failed to reload mapping: {e}")
        return response

    def _on_start_video(self, request, response) -> typing.Any:
        """ROS service callback to start camera streaming."""
        try:
            self.start_camera_stream()
            response.success = True
            response.message = "Camera stream start command issued"
        except Exception as e:
            self.get_logger().error(
                f"Failed to handle start camera stream service call: {e}"
            )
            response.success = False
            response.message = f"Failed to start camera stream: {str(e)}"
        return response

    def _on_stop_video(self, request, response) -> typing.Any:
        """ROS service callback to stop camera streaming."""
        try:
            self.stop_camera_stream()
            response.success = True
            response.message = "Camera stream stop command issued"
        except Exception as e:
            self.get_logger().error(
                f"Failed to handle stop camera stream service call: {e}"
            )
            response.success = False
            response.message = f"Failed to stop camera stream: {str(e)}"
        return response

    def _mapping_watcher_loop(self) -> None:
        # Watch mapping file mtime and auto-reload when it changes.
        mapping = getattr(self, "_mapping", None)
        if mapping is None:
            return
        path = mapping.path
        try:
            last_mtime = os.path.getmtime(path)
        except Exception:
            last_mtime = None

        ev = getattr(self, "_mapping_watcher_stop", None)
        while ev is None or not ev.is_set():
            try:
                m = os.path.getmtime(path)
                if last_mtime is None or m != last_mtime:
                    last_mtime = m
                    try:
                        mapping.reload()
                        self.get_logger().info(f"Auto-reloaded mapping from {path}")
                    except Exception as e:
                        self.get_logger().error(f"Auto-reload mapping failed: {e}")
            except Exception:
                # ignore transient IO errors
                pass
            # wait with timeout so we can exit quickly on stop
            if ev is None:
                time.sleep(1.0)
            else:
                ev.wait(1.0)

    def _make_ros_cb(
        self, ros_topic: str, mqtt_topic: str, msg_cls: typing.Any
    ) -> typing.Callable[[typing.Any], None]:
        """ROS subscriber callback forwarding to MQTT."""

        def cb(msg):
            try:
                # Global upstream kill-switch
                if getattr(self, "_disable_all_upstream", False):
                    return

                payload = None
                self._ros_state_cache[ros_topic] = msg

                # Update accumulated joint states from feedback if this is a JointState message
                if msg_cls is JointState or msg_cls.__name__ == "JointState":
                    if self._telemetry_processor:
                        self._telemetry_processor.process_joint_states(msg)

                if msg_cls is BatteryState or msg_cls.__name__ == "BatteryState":
                    self._last_battery_msg = msg
                    self.get_logger().debug(
                        f"Cached battery message: voltage={msg.voltage}V"
                    )
                elif (
                    msg_cls is Float32 or msg_cls.__name__ == "Float32"
                ) and "battery" in mqtt_topic:
                    self._last_battery_msg = msg
                    self.get_logger().debug(
                        f"Cached battery message from Float32: {msg.data}V"
                    )

                # Rate limiting
                interval = self._ros2mqtt_custom_intervals.get(
                    ros_topic, self._ros2mqtt_rate_interval
                )
                if interval > 0:
                    current_time = time.time()
                    last_time = self._last_publish_time.get(ros_topic, 0.0)
                    if current_time - last_time < interval:
                        return
                    self._last_publish_time[ros_topic] = current_time

                adapter = getattr(self, "_mqtt_adapter", None)
                sdk_method = (
                    self._ros2mqtt_sdk_method.get(ros_topic)
                    if hasattr(self, "_ros2mqtt_sdk_method")
                    else None
                )
                mapping = getattr(self, "_mapping", None)

                # Upstream filtering based on mapping mode
                if mapping is not None:
                    topic_type = (
                        "joint"
                        if "cyberwave/joint/" in mqtt_topic
                        else "pose"
                        if "cyberwave/pose/" in mqtt_topic
                        else None
                    )
                    if topic_type and not mapping.should_publish_topic(topic_type):
                        return

                is_odom = (
                    msg_cls is Odometry
                    or msg_cls.__name__ == "Odometry"
                    or "Odometry" in str(msg_cls)
                )

                # Fallback to internal odom if ROS /odom not available and internal odom is enabled
                if (
                    "cyberwave/pose/" in mqtt_topic
                    and "/update" in mqtt_topic
                    and payload is None
                    and mapping is not None
                    and self._internal_odom
                ):
                    try:
                        pose = self._internal_odom.get_pose()
                        iqz = math.sin(pose["theta"] / 2.0)
                        iqw = math.cos(pose["theta"] / 2.0)
                        payload = json.dumps(
                            {
                                "source_type": SOURCE_TYPE_EDGE,
                                "type": "update",
                                "position": {"x": pose["x"], "y": pose["y"], "z": 0.0},
                                "rotation": {"w": iqw, "x": 0.0, "y": 0.0, "z": iqz},
                                "ts": time.time(),
                                "method": "internal_dead_reckoning",
                            }
                        )
                    except Exception:
                        pass

                if is_odom and (
                    "position" in mqtt_topic
                    or "rotation" in mqtt_topic
                    or "update" in mqtt_topic
                ):
                    try:
                        raw_pos = msg.pose.pose.position
                        raw_quat = msg.pose.pose.orientation
                        if "update" in mqtt_topic:
                            payload_obj = {
                                "source_type": SOURCE_TYPE_EDGE,
                                "type": "update",
                                "position": {
                                    "x": raw_pos.x,
                                    "y": raw_pos.y,
                                    "z": raw_pos.z,
                                },
                                "rotation": {
                                    "w": raw_quat.w,
                                    "x": raw_quat.x,
                                    "y": raw_quat.y,
                                    "z": raw_quat.z,
                                },
                                "ts": time.time(),
                            }
                        elif "/position" in mqtt_topic:
                            payload_obj = {
                                "source_type": SOURCE_TYPE_EDGE,
                                "position": {
                                    "x": raw_pos.x,
                                    "y": raw_pos.y,
                                    "z": raw_pos.z,
                                },
                                "ts": time.time(),
                            }
                        else:  # /rotation
                            payload_obj = {
                                "source_type": SOURCE_TYPE_EDGE,
                                "rotation": {
                                    "w": raw_quat.w,
                                    "x": raw_quat.x,
                                    "y": raw_quat.y,
                                    "z": raw_quat.z,
                                },
                                "ts": time.time(),
                            }
                        payload = json.dumps(payload_obj)
                    except Exception:
                        payload = self._encode_msg_for_mqtt(
                            msg, msg_cls, mqtt_topic=mqtt_topic
                        )

                elif (
                    (msg_cls is JointState or msg_cls.__name__ == "JointState")
                    and adapter is not None
                    and sdk_method == "update_joint_state"
                    and getattr(self, "_mapping", None) is not None
                    and getattr(self._mapping, "twin_uuid", None)
                ):
                    if getattr(self, "_disable_edge_joint_updates", False):
                        return
                    try:
                        mapping = self._mapping
                        twin = mapping.twin_uuid
                        name_to_idx = {n: i for i, n in enumerate(msg.name or [])}

                        # Retrieve servo positions tracked by CameraServoHandler so we
                        # can override the stale zero values published by joint_state_publisher
                        # for the pan-tilt joints (the hardware does not report them back).
                        servo_overrides: dict = {}
                        try:
                            registry = getattr(self, "_command_registry", None)
                            if registry is not None:
                                servo_handler = registry._handlers.get("camera_servo")
                                if servo_handler is not None:
                                    servo_overrides = {
                                        servo_handler._PAN_JOINT: servo_handler._pan_position,
                                        servo_handler._TILT_JOINT: servo_handler._tilt_position,
                                    }
                        except Exception:
                            pass

                        for ros_name, mqtt_name in mapping.name_to_mqtt.items():
                            idx = name_to_idx.get(ros_name)
                            if idx is None:
                                continue
                            # Use servo override position when available to avoid the
                            # joint_state_publisher resetting pan/tilt to 0.0 in the twin.
                            if ros_name in servo_overrides:
                                pos = servo_overrides[ros_name]
                            else:
                                pos = (
                                    float(msg.position[idx])
                                    if msg.position and idx < len(msg.position)
                                    else None
                                )
                            vel = (
                                float(msg.velocity[idx])
                                if msg.velocity and idx < len(msg.velocity)
                                else None
                            )
                            eff = (
                                float(msg.effort[idx])
                                if msg.effort and idx < len(msg.effort)
                                else None
                            )
                            adapter.update_joint_state(
                                twin, mqtt_name, position=pos, velocity=vel, effort=eff
                            )
                        return
                    except Exception:
                        self.get_logger().debug(
                            f"SDK update_joint_state failed for {ros_topic}"
                        )

                # Handle Odometry messages with SDK position/rotation methods
                # Following the Cyberwave EdgeNode pattern: publish_position(twin_uuid, position, rotation)
                elif (
                    (msg_cls is Odometry or msg_cls.__name__ == "Odometry")
                    and adapter is not None
                    and sdk_method == "update_twin_pose"
                    and getattr(self, "_mapping", None) is not None
                    and getattr(self._mapping, "twin_uuid", None)
                ):
                    try:
                        twin = self._mapping.twin_uuid
                        # Extract position
                        position = {
                            "x": float(msg.pose.pose.position.x),
                            "y": float(msg.pose.pose.position.y),
                            "z": float(msg.pose.pose.position.z),
                        }
                        # Extract rotation (quaternion)
                        rotation = {
                            "w": float(msg.pose.pose.orientation.w),
                            "x": float(msg.pose.pose.orientation.x),
                            "y": float(msg.pose.pose.orientation.y),
                            "z": float(msg.pose.pose.orientation.z),
                        }
                        # Publish both position and rotation using SDK pattern
                        # This matches the EdgeNode.publish_position() pattern from the example
                        adapter.publish_position(
                            twin_uuid=twin,
                            position=position,
                            rotation=rotation,
                            source_type=SOURCE_TYPE_EDGE,
                        )
                        return
                    except Exception as e:
                        self.get_logger().warning(
                            f"SDK publish_position failed for {ros_topic}: {e}",
                            exc_info=True,
                        )
                        # Fall through to standard encoding

                if payload is None:
                    payload = self._encode_msg_for_mqtt(
                        msg, msg_cls, mqtt_topic=mqtt_topic
                    )

                if isinstance(payload, str):
                    if payload == "{}":  # Skip empty payloads from disabled updates
                        return
                    # Verbose logging removed for performance
                    # if any(x in mqtt_topic for x in ['odom_raw', 'imu', 'battery', 'voltage']):
                    #     self.get_logger().info(f"MQTT PUBLISH: {ros_topic} → {mqtt_topic} (payload={payload[:80]}...)")
                    if adapter is not None:
                        try:
                            adapter.publish(mqtt_topic, payload)
                        except Exception as e:
                            self.get_logger().error(f"Adapter publish failed: {e}")
                    else:
                        self._mqtt_client.publish(mqtt_topic, payload)
            except Exception as e:
                self.get_logger().error(
                    f"Failed to publish to MQTT topic {mqtt_topic}: {e}"
                )

        return cb

    def _encode_msg_for_mqtt(
        self, msg, msg_cls, mqtt_topic: str = None
    ) -> typing.Union[str, bytes]:
        # Convert ROS message to an MQTT payload (bytes or string)
        try:
            # Special case for battery status when using Float32
            if (
                (msg_cls is Float32 or msg_cls.__name__ == "Float32")
                and mqtt_topic
                and "battery" in mqtt_topic
            ):
                try:
                    voltage_val = float(msg.data)
                    # Default 3S LiPo range: 9.0V to 12.6V
                    percentage = (voltage_val - 9.0) / (12.6 - 9.0)
                    return json.dumps(
                        {
                            "source_type": SOURCE_TYPE_EDGE,
                            "voltage": voltage_val,
                            "percentage": float(max(0.0, min(1.0, percentage))),
                            "timestamp": time.time(),
                        }
                    )
                except Exception:
                    pass

            # If we have a mapping and this is a JointState, use the mapping
            mapping = getattr(self, "_mapping", None)
            if (
                msg_cls is JointState or msg_cls.__name__ == "JointState"
            ) and mapping is not None:
                if getattr(self, "_disable_edge_joint_updates", False):
                    return json.dumps({})  # Return empty or skip
                try:
                    # Standard Cyberwave SDK Joint State format:
                    # { "type": "joint_state", "joint_name": "...", "joint_state": { "position": ... } }
                    # or consolidated mapping
                    payload_obj = mapping.remap_ros_to_mqtt(msg)

                    # Override pan-tilt servo positions with values tracked by
                    # CameraServoHandler.  The joint_state_publisher resets them to 0
                    # because the hardware doesn't report servo feedback positions.
                    try:
                        registry = getattr(self, "_command_registry", None)
                        if registry is not None:
                            servo_handler = registry._handlers.get("camera_servo")
                            if servo_handler is not None and isinstance(payload_obj, dict):
                                positions = payload_obj.get("positions", {})
                                if isinstance(positions, dict):
                                    pan_mqtt = mapping.name_to_mqtt.get(servo_handler._PAN_JOINT)
                                    tilt_mqtt = mapping.name_to_mqtt.get(servo_handler._TILT_JOINT)
                                    if pan_mqtt:
                                        positions[pan_mqtt] = servo_handler._pan_position
                                    if tilt_mqtt:
                                        positions[tilt_mqtt] = servo_handler._tilt_position
                    except Exception:
                        pass

                    # Ensure source_type and ts are present (Go2 style)
                    if isinstance(payload_obj, dict):
                        payload_obj["source_type"] = SOURCE_TYPE_EDGE
                        if "ts" not in payload_obj:
                            payload_obj["ts"] = time.time()

                    return json.dumps(payload_obj)
                except Exception as e:
                    self.get_logger().error(f"Mapping ros->mqtt failed: {e}")
                    # fallthrough to fallback encoders

            if msg_cls is String:
                return msg.data
            if msg_cls is Int32 or msg_cls is Float32:
                return str(msg.data)
            if msg_cls is UInt32MultiArray or msg_cls is Float32MultiArray:
                # convert to JSON array
                return json.dumps(list(msg.data))

            # Generic ROS message to dict converter for other types (Imu, BatteryState, Odometry, etc.)
            try:
                payload_dict = message_to_ordereddict(msg)

                # Add source_type for upstream traffic from edge device
                if "source_type" not in payload_dict:
                    payload_dict["source_type"] = SOURCE_TYPE_EDGE

                # Add timestamp for convenience
                if (
                    "timestamp" not in payload_dict
                    and hasattr(msg, "header")
                    and hasattr(msg.header, "stamp")
                ):
                    payload_dict["ts"] = (
                        float(msg.header.stamp.sec)
                        + float(msg.header.stamp.nanosec) * 1e-9
                    )
                return json.dumps(payload_dict)
            except Exception as e:
                self.get_logger().debug(f"Generic encoder failed: {e}")
        except Exception:
            pass
        # fallback: try to stringify
        try:
            return str(msg)
        except Exception:
            return ""

    def _calculate_trajectory_time(
        self, target_positions: typing.List[float], joint_names: typing.List[str]
    ) -> float:
        """
        Calculate safe trajectory time based on distance to target and velocity/acceleration limits.

        Args:
            target_positions: List of target joint positions (radians)
            joint_names: List of joint names corresponding to positions

        Returns:
            Safe trajectory time in seconds
        """
        mapping = getattr(self, "_mapping", None)
        if not mapping or not hasattr(mapping, "robot_constants"):
            # Fallback: use conservative default
            return 2.0

        constants = mapping.robot_constants
        max_velocities = constants.get("max_velocities", {})
        max_accelerations = constants.get("max_accelerations", {})
        min_time = constants.get("min_trajectory_time", 0.5)
        safety_factor = constants.get("time_safety_factor", 2.0)

        # Get current robot positions
        current_positions = {}
        if (
            hasattr(self, "_accumulated_joint_states")
            and "initial_joint_state" in self._accumulated_joint_states
        ):
            initial_state = self._accumulated_joint_states["initial_joint_state"]
            for idx, joint_name in enumerate(mapping.joint_names):
                if idx < len(initial_state):
                    current_positions[joint_name] = initial_state[idx]

        # Calculate required time for each joint
        max_time = min_time

        for idx, joint_name in enumerate(joint_names):
            if idx >= len(target_positions):
                continue

            current_pos = current_positions.get(joint_name, 0.0)
            target_pos = target_positions[idx]
            distance = abs(target_pos - current_pos)

            if distance < 0.001:  # Negligible movement
                continue

            # Get velocity and acceleration limits for this joint
            max_vel = max_velocities.get(joint_name, max_velocities.get("default", 1.0))
            max_accel = max_accelerations.get(
                joint_name, max_accelerations.get("default", 3.0)
            )

            # Calculate time based on velocity limit (distance / velocity)
            time_by_velocity = distance / max_vel if max_vel > 0 else min_time

            # Calculate time based on acceleration limit (sqrt(2 * distance / acceleration))
            # For triangular velocity profile: time = 2 * sqrt(distance / acceleration)
            time_by_accel = (
                2.0 * math.sqrt(distance / max_accel) if max_accel > 0 else min_time
            )

            # Use the maximum (most conservative) time
            joint_time = max(time_by_velocity, time_by_accel, min_time)
            max_time = max(max_time, joint_time)

        # Apply safety factor
        safe_time = max_time * safety_factor

        # Ensure minimum time for safety
        safe_time = max(safe_time, min_time)

        return safe_time

    def _create_smooth_trajectory_points(
        self,
        start_positions: typing.List[float],
        target_positions: typing.List[float],
        total_time: float,
        num_points: int = 3,
    ) -> typing.List[JointTrajectoryPoint]:
        """
        Create smooth trajectory points with intermediate waypoints for smoother motion.

        Args:
            start_positions: Starting joint positions
            target_positions: Target joint positions
            total_time: Total trajectory time in seconds
            num_points: Number of trajectory points (including start and end)

        Returns:
            List of JointTrajectoryPoint messages
        """
        points = []

        for i in range(num_points):
            # Normalized time [0, 1] - first point at t=0 (current), last at t=1 (target)
            t = i / (num_points - 1) if num_points > 1 else 1.0
            point_time = total_time * t

            # Linear interpolation between start and target
            interpolated_positions = []
            for j in range(len(target_positions)):
                start_pos = (
                    start_positions[j]
                    if j < len(start_positions)
                    else target_positions[j]
                )
                target_pos = target_positions[j]
                interp_pos = start_pos + (target_pos - start_pos) * t
                interpolated_positions.append(interp_pos)

            point = JointTrajectoryPoint()
            point.positions = interpolated_positions
            point.velocities = []
            point.accelerations = []
            # time_from_start is relative to trajectory start (header.stamp)
            point.time_from_start.sec = int(point_time)
            point.time_from_start.nanosec = int((point_time - int(point_time)) * 1e9)
            points.append(point)

        return points

    def _republish_position_command(self) -> None:
        """Continuously republish the last position/trajectory command."""
        if (
            self._last_position_command is not None
            and self._position_command_publisher is not None
        ):
            try:
                # Update timestamp for trajectory messages
                if hasattr(self._last_position_command, "header"):
                    self._last_position_command.header.stamp = (
                        self.get_clock().now().to_msg()
                    )
                self._position_command_publisher.publish(self._last_position_command)
            except Exception as e:
                self.get_logger().error(f"Failed to republish command: {e}")

    def _on_joint_states(self, msg: JointState) -> None:
        """
        Callback to initialize and continuously update current robot position from /joint_states.
        Delegates to telemetry module.
        """
        if self._telemetry_processor:
            self._telemetry_processor.process_joint_states(msg)

    def _handle_mqtt_connect(self, rc: int) -> None:
        if rc == 0:
            self.get_logger().info(
                f"Connected to MQTT broker {getattr(self, '_mqtt_host', '?')}:{getattr(self, '_mqtt_port', '?')}"
            )
            # (re)subscribe to topics that we want to forward to ROS
            for topic in self._mqtt2ros_pubs.keys():
                try:
                    self._mqtt_client.subscribe(topic)
                    self.get_logger().info(f"Subscribed to MQTT topic '{topic}'")
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to {topic}: {e}")
            # also (re)subscribe to any general callback topics (e.g., ping)
            for topic in list(self._mqtt_callbacks.keys()):
                try:
                    self._mqtt_client.subscribe(topic)
                    self.get_logger().info(f"Subscribed to MQTT topic '{topic}'")
                except Exception as e:
                    self.get_logger().error(f"Failed to subscribe to {topic}: {e}")

            # Publish initial battery status after connection
            self._publish_battery_status()

            # Create timer for periodic battery updates (every 60 seconds)
            if (
                not hasattr(self, "_battery_update_timer")
                or self._battery_update_timer is None
            ):
                self._battery_update_timer = self.create_timer(
                    60.0, self._publish_battery_status
                )
                self.get_logger().info(
                    "Started periodic battery status updates (60s interval)"
                )
        else:
            self.get_logger().error(f"MQTT connect returned error code {rc}")

    def _publish_battery_status(self) -> None:
        """Publish current battery status to MQTT."""
        try:
            # Check if we have battery data
            if self._last_battery_msg is None:
                self.get_logger().debug(
                    "No battery data available yet for periodic update"
                )
                return

            # Get twin UUID
            if not hasattr(self, "_mapping") or self._mapping is None:
                self.get_logger().debug(
                    "No mapping available for battery status publishing"
                )
                return

            twin_uuid = getattr(self._mapping, "twin_uuid", None)
            if not twin_uuid:
                self.get_logger().debug(
                    "No twin_uuid available for battery status publishing"
                )
                return

            # Prepare topic
            topic = f"{self.ros_prefix}cyberwave/twin/{twin_uuid}/status/battery"

            # Encode battery message for MQTT
            payload_str = self._encode_msg_for_mqtt(
                self._last_battery_msg, type(self._last_battery_msg), mqtt_topic=topic
            )

            # Parse the JSON string to dict for adapter, or use as-is for paho client
            if self._mqtt_adapter:
                # Adapter expects dict
                try:
                    payload_dict = (
                        json.loads(payload_str)
                        if isinstance(payload_str, str)
                        else payload_str
                    )
                    self._mqtt_adapter.publish(topic, payload_dict)
                    self.get_logger().info(
                        f"Published periodic battery status to {topic}: voltage={payload_dict.get('voltage', 'N/A')}V, percentage={payload_dict.get('percentage', 'N/A')}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Failed to parse battery payload: {e}")
            else:
                # Paho client expects string
                self._mqtt_client.publish(topic, payload_str)
                self.get_logger().info(f"Published periodic battery status to {topic}")

        except Exception as e:
            self.get_logger().error(f"Failed to publish battery status: {e}")

    def _handle_mqtt_message(self, topic: str, payload=None, mqtt_msg=None) -> None:
        """Main entry point for all incoming MQTT messages.

        Routes messages based on topic to either standard ROS publishers or
        specialized command handlers (CommandRouter).
        """
        # Robust argument extraction: handles both Paho (1 arg) and Bridge (3 args) signatures
        if mqtt_msg is None:
            if hasattr(topic, "topic"):  # Paho style: _handle_mqtt_message(msg)
                mqtt_msg = topic
                topic = mqtt_msg.topic
                payload_bytes = mqtt_msg.payload
            else:  # Bridge style: _handle_mqtt_message(topic, payload)
                payload_bytes = payload
        else:  # Full 3-arg style
            topic = mqtt_msg.topic
            payload_bytes = mqtt_msg.payload

        try:
            payload = payload_bytes.decode("utf-8")
        except Exception:
            payload = str(payload_bytes)

        try:
            data = json.loads(payload)
        except Exception:
            data = payload

        source_type = None
        if isinstance(data, dict):
            source_type = data.get("source_type")

        # Log downstream message if it comes from tele (debug level to reduce overhead)
        if source_type == SOURCE_TYPE_TELE:
            self.get_logger().debug(
                f"Received downstream message from TELE: topic={topic}, content={payload}"
            )

        # Reduced logging for command topics (only log at debug level for performance)
        if "/command" in topic:
            self.get_logger().debug(
                f"Command topic: {topic}, source_type: {source_type}, payload: {payload[:200]}"
            )

        if "webrtc-" in topic:
            self.get_logger().debug(
                f"Signaling topic: {topic}, source_type: {source_type}, payload: {payload[:100]}..."
            )

        is_signaling = "webrtc-" in topic

        if (
            source_type == SOURCE_TYPE_EDGE
            and not is_signaling
            and topic not in self._mqtt_callbacks
        ):
            return

        # CRITICAL FILTER: Commands must come from frontend/simulator, NEVER from edge
        # This prevents command-feedback loops where edge devices echo commands back
        if topic.endswith("/command"):
            if source_type != SOURCE_TYPE_TELE:
                self.get_logger().debug(
                    f"🚫 Filtered command from {source_type} (allowed: {SOURCE_TYPE_TELE})"
                )
                return

        # COMMAND ROUTER: Handle command messages from multiple endpoints
        # Supported endpoints:
        #   - cyberwave/twin/{twin_uuid}/command (general commands)
        #   - cyberwave/twin/{twin_uuid}/motion/command (motion/actuation commands)
        #   - cyberwave/twin/{twin_uuid}/navigate/command (navigation - handled separately)
        # Format: {"command": "cmd_vel", "data": {...}}
        if topic.endswith("/command") and isinstance(data, dict):
            command = data.get("command")
            raw_command_data = data.get("data")
            command_data = (
                raw_command_data if isinstance(raw_command_data, dict) else {}
            )
            if source_type and isinstance(command_data, dict):
                command_data["_source_type"] = source_type

            # Enforce source_type="tele" for video and camera servo commands as requested
            if command in ["start_video", "stop_video", "camera_servo"]:
                if source_type != SOURCE_TYPE_TELE:
                    self.get_logger().warning(
                        f"Ignoring {command} from non-tele source: {source_type} (expected: {SOURCE_TYPE_TELE})"
                    )
                    return
                else:
                    self.get_logger().info(
                        f"Accepted {command} command from {source_type}"
                    )

            # Lazy-init: if registry failed at startup, retry now
            if command and self._command_registry is None:
                self._try_init_command_registry()

            if command and self._command_registry is not None:
                try:
                    # List of actuation commands that should be routed to the 'actuation' handler
                    actuation_commands = [
                        "move_forward",
                        "move_backward",
                        "turn_left",
                        "turn_right",
                        "stop",
                        "locomotion_velocity",
                        "velocity_command",
                        "camera_up",
                        "camera_down",
                        "camera_left",
                        "camera_right",
                        "camera_default",
                        "chassis_light_toggle",
                        "camera_light_toggle",
                        "led_toggle",
                        "take_photo",
                        "battery_check",
                        "sit_down",
                        "stand_up",
                        "obstacle_avoidance_toggle",
                        "start_video",
                        "stop_video",
                    ]

                    # Route actuation commands to the 'actuation' handler with full
                    # payload. A message carrying an explicit top-level
                    # ``velocity_command`` is also an actuation (analog velocity),
                    # even when its discrete ``command`` is not in the static list.
                    if command in actuation_commands or isinstance(
                        data.get("velocity_command"), dict
                    ):
                        # Pass the full original data (including 'command' field) to actuation handler
                        success = self._command_registry.handle_command(
                            "actuation", data
                        )
                    else:
                        # Other commands use the split format (command name + data only)
                        success = self._command_registry.handle_command(
                            command, command_data
                        )

                    if success:
                        return  # Command handled, no further processing needed
                    else:
                        self.get_logger().warning(
                            f"Command '{command}' from topic '{topic}' could not be handled"
                        )
                except Exception as e:
                    self.get_logger().error(f"Error handling command '{command}': {e}")
                return  # Always return for command topics, even if handler failed
            elif command:
                self.get_logger().warning(
                    f"Received command '{command}' but command registry not initialized"
                )
                return
            else:
                self.get_logger().warning(
                    f"Received message on command topic but missing 'command' field: {topic}"
                )
                return

        # Check for exact topic match OR wildcard pattern match
        matched_patterns = []
        if topic in self._mqtt2ros_pubs:
            matched_patterns.append(topic)
        else:
            # Check wildcard patterns (+, #)
            for pattern in self._mqtt2ros_pubs.keys():
                if "+" in pattern or "#" in pattern:
                    if self._match_mqtt_pattern(pattern, topic):
                        matched_patterns.append(pattern)

        # Call registered callbacks FIRST (e.g., tool0 control) even if there are matched patterns
        callback_handled = False
        handlers = self._mqtt_callbacks.get(topic, [])
        for h in handlers:
            try:
                # SDK-compatible callback dispatch: check signature to decide how to call
                import inspect

                try:
                    sig = inspect.signature(h)
                    params = list(sig.parameters.values())
                    effective_count = len(params)
                except Exception:
                    effective_count = -1

                if effective_count == 1:
                    # SDK style: on_message(data)
                    h(data)
                elif effective_count == 2:
                    # Alternative style: on_message(topic, data)
                    h(topic, data)
                else:
                    # Node style or unknown: on_message(topic, data, mqtt_msg)
                    try:
                        h(topic, data, mqtt_msg)
                    except TypeError:
                        # Fallback for SDK nested functions where signature might fail
                        try:
                            h(data)
                        except TypeError:
                            h(topic, data, mqtt_msg)

                callback_handled = True
            except Exception as e:
                self.get_logger().error(f"Error in MQTT callback for {topic}: {e}")

        # Process matched patterns for trajectory/command publishing
        if matched_patterns:
            for pattern in matched_patterns:
                entries = self._mqtt2ros_pubs[pattern]
                for pub, msg_cls, ros_topic in entries:
                    try:
                        # STRICT FILTER: For joint updates (actuation), only allow messages from "tele"
                        if (
                            msg_cls is JointState
                            or msg_cls is JointTrajectory
                            or "/joint_states" in ros_topic
                        ):
                            if source_type != SOURCE_TYPE_TELE:
                                self.get_logger().info(
                                    f"Ignoring joint update for {ros_topic} from {source_type} "
                                    f"(only '{SOURCE_TYPE_TELE}' allowed for actuation)"
                                )
                                continue

                        ros_msg = self._decode_payload_to_msg(payload, msg_cls)

                        # Pass source_type metadata through frame_id so the driver can filter
                        if hasattr(ros_msg, "header") and source_type:
                            ros_msg.header.frame_id = str(source_type)

                        # Validate JointTrajectory messages before publishing
                        if msg_cls is JointTrajectory and isinstance(
                            ros_msg, JointTrajectory
                        ):
                            if not ros_msg.joint_names or not ros_msg.points:
                                self.get_logger().warning(
                                    f"Empty trajectory created for {ros_topic}: "
                                    f"joint_names={len(ros_msg.joint_names) if ros_msg.joint_names else 0}, "
                                    f"points={len(ros_msg.points) if ros_msg.points else 0}"
                                )
                                continue
                            if ros_msg.points and len(
                                ros_msg.points[0].positions
                            ) != len(ros_msg.joint_names):
                                self.get_logger().error(
                                    f"Invalid trajectory: {len(ros_msg.joint_names)} joint names but "
                                    f"{len(ros_msg.points[0].positions)} positions - skipping publish"
                                )
                                continue
                            # Log trajectory info - show last point's time (actual trajectory duration)
                            last_point_time = 0.0
                            if ros_msg.points:
                                last_point = ros_msg.points[-1]
                                last_point_time = (
                                    float(last_point.time_from_start.sec)
                                    + float(last_point.time_from_start.nanosec) * 1e-9
                                )

                            self.get_logger().info(
                                f"Trajectory: {len(ros_msg.joint_names)} joints, {len(ros_msg.points)} points, {last_point_time:.2f}s"
                            )

                        pub.publish(ros_msg)
                    except Exception as e:
                        self.get_logger().error(
                            f"Failed to publish ROS message for {topic} -> {ros_topic}: {e}"
                        )

        # Also check for wildcard callback patterns
        # This ensures callbacks work for both exact matches and wildcards
        for pattern, wildcard_handlers in self._mqtt_callbacks.items():
            if pattern == topic:
                continue  # Already handled above
            if "+" in pattern or "#" in pattern:
                if self._match_mqtt_pattern(pattern, topic):
                    for h in wildcard_handlers:
                        try:
                            h(topic, data, mqtt_msg)
                            callback_handled = True
                        except Exception as e:
                            self.get_logger().error(
                                f"Error in MQTT wildcard callback for {pattern}: {e}"
                            )

        # Log if no patterns matched and no callbacks handled
        if not matched_patterns and not callback_handled:
            pass

    def _decode_payload_to_msg(self, payload: str, msg_cls) -> typing.Any:
        if msg_cls is String:
            return String(data=payload)
        mapping = getattr(self, "_mapping", None)
        if msg_cls is JointState:
            try:
                data = json.loads(payload) if isinstance(payload, str) else payload
            except Exception:
                data = None

            if (
                isinstance(data, dict)
                and data.get("type") == "joint_state"
                and "joint_name" in data
                and "joint_state" in data
            ):
                try:
                    jname_mqtt = data.get("joint_name")
                    jstate = data.get("joint_state") or {}
                    if mapping is not None:
                        ros_name = mapping.mqtt_to_name.get(jname_mqtt) or jname_mqtt
                        js = JointState()
                        js.header.stamp = self.get_clock().now().to_msg()
                        js.name = list(mapping.joint_names)
                        n = len(js.name)
                        js.position, js.velocity, js.effort = (
                            [float("nan")] * n,
                            [float("nan")] * n,
                            [float("nan")] * n,
                        )
                        try:
                            idx = js.name.index(ros_name)
                        except ValueError:
                            idx = None
                        rev = mapping.reverse_transforms.get(ros_name, lambda x: x)
                        if idx is not None:
                            if "position" in jstate:
                                js.position[idx] = float(rev(jstate["position"]))
                            if "velocity" in jstate:
                                js.velocity[idx] = float(rev(jstate["velocity"]))
                            if "effort" in jstate:
                                js.effort[idx] = float(rev(jstate["effort"]))
                        return js
                    else:
                        js = JointState(name=[jname_mqtt])
                        js.header.stamp = self.get_clock().now().to_msg()
                        pos = jstate.get("position")
                        js.position = (
                            [float(pos)] if pos is not None else [float("nan")]
                        )
                        return js
                except Exception:
                    pass

            if mapping is not None:
                try:
                    return mapping.remap_mqtt_to_ros(payload)
                except Exception:
                    pass
        if msg_cls is Int32:
            return Int32(data=int(payload))
        if msg_cls is Float32:
            return Float32(data=float(payload))
        if msg_cls is UInt32MultiArray:
            m = UInt32MultiArray()
            try:
                m.data = [int(x) for x in json.loads(payload)]
            except Exception:
                m.data = []
            return m
        if msg_cls is Float32MultiArray:
            m = Float32MultiArray()
            try:
                m.data = [float(x) for x in json.loads(payload)]
            except Exception:
                m.data = []
            return m
        if msg_cls is Float64MultiArray:
            m = Float64MultiArray()
            try:
                data = json.loads(payload) if isinstance(payload, str) else payload
                if isinstance(data, dict) and mapping is not None:
                    m.data = [
                        float(data.get(mapping.name_to_mqtt.get(n, n), 0.0))
                        for n in mapping.joint_names
                    ]
                elif isinstance(data, list):
                    m.data = [float(x) for x in data]
            except Exception:
                m.data = []
            return m
        if msg_cls is JointTrajectory:
            m = JointTrajectory()
            try:
                data = json.loads(payload) if isinstance(payload, str) else payload
                if (
                    isinstance(data, dict)
                    and "joint_name" in data
                    and "joint_state" in data
                ):
                    if mapping is not None:
                        jname_mqtt, jstate = (
                            data.get("joint_name"),
                            data.get("joint_state", {}),
                        )
                        pos_val = jstate.get("position")
                        if jname_mqtt is not None and pos_val is not None:
                            ros_name = mapping.mqtt_to_name.get(jname_mqtt, jname_mqtt)
                            if ros_name in self._get_virtual_joints():
                                m.header.stamp = self.get_clock().now().to_msg()
                                return m
                            state_key = "trajectory_accumulated_state"
                            if not hasattr(self, "_accumulated_joint_states"):
                                self._accumulated_joint_states = {}
                            if state_key not in self._accumulated_joint_states:
                                self._accumulated_joint_states[state_key] = [0.0] * len(
                                    mapping.joint_names
                                )
                            try:
                                idx = list(mapping.joint_names).index(ros_name)
                                self._accumulated_joint_states[state_key][idx] = float(
                                    mapping.reverse_transforms.get(
                                        ros_name, lambda x: x
                                    )(pos_val)
                                )
                            except Exception:
                                pass
                        if not self._joint_state_initialized:
                            m.header.stamp = self.get_clock().now().to_msg()
                            return m
                        initial_state = self._accumulated_joint_states.get(
                            "initial_joint_state"
                        )
                        virtual_joints = self._get_virtual_joints()
                        joint_names_filtered = [
                            jn for jn in mapping.joint_names if jn not in virtual_joints
                        ]
                        positions_filtered, start_positions = [], []
                        for jn in joint_names_filtered:
                            idx = list(mapping.joint_names).index(jn)
                            target_val = self._accumulated_joint_states[state_key][idx]
                            positions_filtered.append(target_val)
                            start_positions.append(
                                initial_state[idx]
                                if initial_state and idx < len(initial_state)
                                else target_val
                            )
                        m.joint_names = joint_names_filtered
                        traj_time = self._calculate_trajectory_time(
                            positions_filtered, joint_names_filtered
                        )
                        m.points = self._create_smooth_trajectory_points(
                            start_positions, positions_filtered, traj_time, num_points=3
                        )
                        m.header.stamp = self.get_clock().now().to_msg()
                        return m
                if isinstance(data, dict) and mapping is not None:
                    virtual_joints = self._get_virtual_joints()
                    joint_names_filtered = [
                        jn for jn in mapping.joint_names if jn not in virtual_joints
                    ]
                    positions, start_positions = [], []
                    initial_state = self._accumulated_joint_states.get(
                        "initial_joint_state"
                    )
                    for jn in joint_names_filtered:
                        idx = list(mapping.joint_names).index(jn)
                        mqtt_n = mapping.name_to_mqtt.get(jn, jn)
                        val = (
                            float(
                                mapping.reverse_transforms.get(jn, lambda x: x)(
                                    data[mqtt_n]
                                )
                            )
                            if mqtt_n in data
                            else (
                                initial_state[idx]
                                if initial_state and idx < len(initial_state)
                                else 0.0
                            )
                        )
                        positions.append(val)
                        start_positions.append(
                            initial_state[idx]
                            if initial_state and idx < len(initial_state)
                            else val
                        )
                    if not self._joint_state_initialized:
                        m.header.stamp = self.get_clock().now().to_msg()
                        return m
                    m.joint_names = joint_names_filtered
                    traj_time = self._calculate_trajectory_time(
                        positions, joint_names_filtered
                    )
                    m.points = self._create_smooth_trajectory_points(
                        start_positions, positions, traj_time, num_points=3
                    )
                    m.header.stamp = self.get_clock().now().to_msg()
                    return m
                if isinstance(data, list):
                    if mapping is not None:
                        virtual_joints = self._get_virtual_joints()
                        m.joint_names = [
                            jn for jn in mapping.joint_names if jn not in virtual_joints
                        ]
                        positions = [float(x) for x in data[: len(m.joint_names)]]
                    else:
                        positions = [float(x) for x in data]
                    traj_time = self._calculate_trajectory_time(
                        positions, m.joint_names if mapping else []
                    )
                    p = JointTrajectoryPoint(
                        positions=positions, velocities=[], accelerations=[]
                    )
                    p.time_from_start.sec = int(traj_time)
                    p.time_from_start.nanosec = int((traj_time - int(traj_time)) * 1e9)
                    m.points = [p]
                    m.header.stamp = self.get_clock().now().to_msg()
                    return m
            except Exception:
                pass
            return m
        return String(data=payload)
        # fallback: return String with raw payload
        m = String()
        m.data = payload
        return m

    def _match_mqtt_pattern(self, pattern: str, topic: str) -> bool:
        """Match MQTT pattern (+ and #) against a concrete topic.

        This mirrors the simple matching used in the Cyberwave SDK adapter.
        """
        # Convert MQTT pattern to regex
        import re

        pattern_escaped = re.escape(pattern)
        pattern_escaped = pattern_escaped.replace(r"\+", r"[^/]+")
        if pattern_escaped.endswith(r"\#"):
            pattern_escaped = pattern_escaped[:-2] + r".*"
        elif r"\#" in pattern_escaped:
            return False
        return bool(re.match(f"^{pattern_escaped}$", topic))

    def publish(self, topic: str, message, qos: int = 0) -> typing.Any:
        """Publish a message to MQTT using simple serialization rules.

        - dict or list -> JSON
        - bytes -> sent as-is
        - other -> str()

        Returns paho-mqtt's publish result object.
        """
        # Global upstream kill-switch
        if getattr(self, "_disable_all_upstream", False):
            # Allow WebRTC signaling to pass through even if upstream is disabled
            # as it's required for the video stream to establish.
            if "webrtc" not in topic:
                return None

        # TOPIC MAPPING FIX (2026-01-18):
        # The Rust Media Service expects specialized topics for signaling.
        # If we see a WebRTC message on the consolidated topic, route it to the specific one.
        final_topic = topic
        if topic.endswith("/webrtc") and isinstance(message, dict):
            msg_type = message.get("type")
            if msg_type == "offer":
                final_topic = topic + "-offer"
                self.get_logger().info(f"Rerouting signaling: {topic} -> {final_topic}")
            elif msg_type == "answer":
                final_topic = topic + "-answer"
                self.get_logger().info(f"Rerouting signaling: {topic} -> {final_topic}")
            elif msg_type == "candidate":
                final_topic = topic + "-candidate"
                self.get_logger().info(f"Rerouting signaling: {topic} -> {final_topic}")

        # Debug log for WebRTC signaling and commands
        if "webrtc" in final_topic or "command" in final_topic:
            self.get_logger().info(
                f"MQTT OUTGOING: topic={final_topic}, content={str(message)[:100]}..."
            )

        try:
            if isinstance(message, (dict, list)):
                payload = json.dumps(message)
            elif isinstance(message, (bytes, bytearray)):
                payload = message
            else:
                payload = str(message)
            # Use adapter if present, otherwise paho publish
            adapter = getattr(self, "_mqtt_adapter", None)
            if adapter is not None:
                res = adapter.publish(final_topic, payload, qos=qos)
            else:
                # paho publish is thread-safe for simple usage
                res = self._mqtt_client.publish(final_topic, payload, qos=qos)
            return res
        except Exception as e:
            self.get_logger().error(
                f"Failed to publish to MQTT topic {final_topic}: {e}"
            )
            raise

    def subscribe(
        self, topic: str, on_message: typing.Callable = None, qos: int = 0
    ) -> None:
        """SDK-compatible subscribe method.

        Registers a callback and ensures the underlying MQTT client is subscribed.
        """
        self.get_logger().info(
            f"Subscribed to topic: {topic} (on_message: {on_message is not None})"
        )
        if on_message:
            # Register in our internal dispatch map. We do this even if an adapter
            # is used because the node's _handle_mqtt_message relies on this map.
            if topic not in self._mqtt_callbacks:
                self._mqtt_callbacks[topic] = []
            if on_message not in self._mqtt_callbacks[topic]:
                self._mqtt_callbacks[topic].append(on_message)

        # Perform actual MQTT subscription
        if self._mqtt_adapter is not None:
            # The adapter handles its own callback registration and dispatching
            # via CyberwaveAdapter._make_handler which is SDK-compatible.
            self._mqtt_adapter.subscribe(topic, on_message=on_message)
            return

        self._mqtt_client.subscribe(topic, qos=qos)

    def ping(self, resource_uuid: str):
        """Send a simple ping JSON message for a resource_uuid to the broker.

        The message is published to: <topic_prefix>cyberwave/ping/<resource_uuid>/request
        Payload is a small JSON object with a timestamp and identifier.
        """
        topic = f"{self.topic_prefix}cyberwave/ping/{resource_uuid}/request"
        payload = {
            "type": "ping",
            "resource_uuid": resource_uuid,
            "from": self.get_name(),
            "ts": time.time(),
        }
        try:
            self.publish(topic, payload)
            self.get_logger().info(f"Sent ping for {resource_uuid} to {topic}")
        except Exception as e:
            self.get_logger().error(f"Failed to send ping for {resource_uuid}: {e}")

    def subscribe_pong(
        self, resource_uuid: str, on_pong: typing.Optional[typing.Callable] = None
    ) -> None:
        """Subscribe to pong responses for a resource_uuid.

        Calls self.subscribe with the standard pong topic path.
        """
        topic = f"{self.topic_prefix}cyberwave/pong/{resource_uuid}/response"
        self.subscribe(topic, on_pong)

    def _on_ping(self, *args) -> None:
        # stop mapping watcher (if running) then stop mqtt loop and disconnect cleanly
        # Ping handling removed: responses are not handled in-node. If you
        # need automatic ping->pong behavior, implement it externally or add
        # a lightweight handler here.
        pass

    def _on_tool_control_update(self, topic: str, payload, mqtt_msg) -> None:
        """Handle Tool Digital Output control via position updates from MQTT."""
        try:
            if self._robot_io_client is None or not self._io_config:
                return

            # Parse payload
            data = {}
            if isinstance(payload, str):
                try:
                    data = json.loads(payload)
                except Exception:
                    return
            elif isinstance(payload, dict):
                data = payload
            else:
                return

            source_type = data.get("source_type") if isinstance(data, dict) else None
            if source_type != SOURCE_TYPE_TELE:
                self.get_logger().info(
                    f"Ignoring tool control update from {source_type} (only '{SOURCE_TYPE_TELE}' allowed)"
                )
                return

            # Iterate through all configured tools
            for tool_name, config in self._io_config.items():
                if not config.get("enabled"):
                    continue

                target_joint = config.get("joint_name")
                if not target_joint:
                    continue

                # Extract position for this joint
                position = None
                # Try single-joint format: {"joint_name": "...", "joint_state": {"position": 1.0}}
                if data.get("joint_name") == target_joint:
                    joint_state = data.get("joint_state")
                    if isinstance(joint_state, dict):
                        position = joint_state.get("position")
                # Try multi-joint format: {"joint_name": position, ...}
                elif target_joint in data:
                    position = data.get(target_joint)

                if position is None:
                    continue

                try:
                    position_value = float(position)
                except (ValueError, TypeError):
                    continue

                # Map position to ON/OFF state based on threshold
                threshold = config.get("on_threshold", 0.0)
                on_value = config.get("on_value", 1.0)
                off_value = config.get("off_value", 0.0)

                if position_value >= threshold:
                    state = on_value
                    state_str = "ON"
                else:
                    state = off_value
                    state_str = "OFF"

                # Call service
                if self._robot_io_client.wait_for_service(timeout_sec=1.0):
                    request = SetIO.Request()
                    # Default to function=1, pin=0 for UR tool0 if not specified
                    request.fun = config.get("function", 1)
                    request.pin = config.get("pin", 0)
                    request.state = float(state)

                    self.get_logger().info(
                        f"Tool '{tool_name}': {state_str} (pos={position_value:.3f})"
                    )

                    future = self._robot_io_client.call_async(request)
                    future.add_done_callback(
                        lambda f,
                        tn=tool_name,
                        s=state,
                        p=position_value: self._tool_io_response_callback(f, tn, s, p)
                    )

        except Exception as e:
            self.get_logger().error(f"Failed to process tool update: {e}")

    def _tool_io_response_callback(
        self, future, tool_name: str, requested_state: float, position: float
    ) -> None:
        """Handle response from tool I/O service call."""
        try:
            response = future.result()
            if response.success:
                state_str = "on" if requested_state > 0.5 else "off"
                status_payload = {
                    "tool_name": tool_name,
                    "position": position,
                    "state": state_str,
                    "success": True,
                    "timestamp": time.time(),
                }

                # Publish status back to MQTT
                if self._mapping and self._mapping.twin_uuid:
                    prefix = getattr(self, "ros_prefix", "")
                    topic = (
                        f"{prefix}cyberwave/twin/{self._mapping.twin_uuid}/status/io"
                    )
                    self._mqtt_client.publish(topic, json.dumps(status_payload))
            else:
                self.get_logger().error(
                    f"IO service call failed for tool '{tool_name}'"
                )
        except Exception as e:
            self.get_logger().error(f"Error in tool IO callback: {e}")

    def _maybe_auto_start_webrtc(self) -> None:
        """Auto-start WebRTC camera stream once prerequisites are met.

        Checks that the ROSCameraStreamer, MQTT adapter, and twin_uuid are
        all available, then proactively calls start_camera_stream().  If any
        prerequisite is missing, a retry is scheduled.  If the WebRTC
        handshake fails (e.g. no backend listening), start_camera_stream's
        built-in retry mechanism will reschedule automatically.
        """
        if self._auto_start_timer is not None:
            try:
                self._auto_start_timer.cancel()
            except Exception:
                pass
            self._auto_start_timer = None

        # Check if WebRTC infrastructure is ready (ROSCameraStreamer)
        # Note: SDK Twin object is optional - we use ROSCameraStreamer which extends
        # BaseVideoStreamer directly and handles WebRTC signaling via MQTT
        if not hasattr(self, "_ros_streamer") or self._ros_streamer is None:
            self.get_logger().warning(
                "WebRTC auto-start skipped: ROSCameraStreamer not initialized"
            )
            self._schedule_auto_start_retry()
            return

        # Check if WebRTC peer connection is already active
        if getattr(self, "_ros_streamer", None) is not None:
            existing_pc = getattr(self._ros_streamer, "pc", None)
            if existing_pc and getattr(
                existing_pc, "connectionState", "closed"
            ) not in ["closed", "failed", None]:
                self.get_logger().info(
                    "WebRTC auto-start skipped: peer connection already active"
                )
                return

        if self._mqtt_adapter is None:
            self.get_logger().warning(
                "WebRTC auto-start skipped: Cyberwave adapter not initialized"
            )
            self._schedule_auto_start_retry()
            return

        if not getattr(self._mqtt_adapter, "connected", False):
            self.get_logger().warning(
                "WebRTC auto-start skipped: MQTT adapter not connected yet"
            )
            self._schedule_auto_start_retry()
            return

        twin_uuid = getattr(self._mapping, "twin_uuid", None)
        if not twin_uuid:
            self.get_logger().warning(
                "WebRTC auto-start skipped: twin_uuid missing in mapping"
            )
            self._schedule_auto_start_retry()
            return

        if not hasattr(self, "_ros_streamer") or self._ros_streamer is None:
            self.get_logger().warning(
                "WebRTC auto-start skipped: ROSCameraStreamer not initialized"
            )
            self._schedule_auto_start_retry()
            return

        try:
            self.get_logger().info(
                f"WebRTC prerequisites met — auto-starting camera stream for twin {twin_uuid}"
            )
            self.start_camera_stream()
        except Exception as e:
            self.get_logger().error(f"WebRTC auto-start failed: {e}")
            self._schedule_auto_start_retry()

    def _start_webrtc_with_auto_reconnect(self) -> None:
        """Start WebRTC streaming using the SDK's run_with_auto_reconnect().

        This uses the SDK's built-in functionality for:
        - Subscribing to MQTT commands (start_video, stop_video)
        - Starting WebRTC only when commanded (not proactively)
        - Auto-reconnection on WebRTC disconnect
        - Connection monitoring

        IMPORTANT: The SDK's run_with_auto_reconnect() handles the entire WebRTC lifecycle.
        It will wait for start_video commands from the cloud/frontend before establishing
        WebRTC connections. Do NOT call start() manually as it causes timeouts when no
        backend is listening.
        """
        if not hasattr(self, "_ros_streamer") or self._ros_streamer is None:
            self.get_logger().error("Cannot start: ROSCameraStreamer not initialized")
            return

        # Create a stop event that can be used to gracefully stop the streamer
        if not hasattr(self, "_webrtc_stop_event"):
            self._webrtc_stop_event = asyncio.Event()
        else:
            self._webrtc_stop_event.clear()

        # Enable auto-reconnect on the streamer
        self._ros_streamer.auto_reconnect = True

        # Define command callback to log command responses
        def on_command_response(status: str, message: str):
            self.get_logger().info(f"WebRTC command response: {status} - {message}")

        # Wrap the coroutine to capture exceptions and manage lifecycle
        async def _run_with_auto_reconnect():
            try:
                self.get_logger().info("SDK run_with_auto_reconnect() starting...")
                self._webrtc_auto_reconnect_running = True

                # Ensure track is initialized before starting
                if self._ros_streamer.streamer is None:
                    self._ros_streamer.initialize_track()

                # Wait for frames to be ready
                self.get_logger().info(
                    "Waiting for camera frames before starting WebRTC..."
                )
                frame_ready = await asyncio.get_event_loop().run_in_executor(
                    None, self._ros_streamer.streamer.wait_for_frames, 10.0
                )

                if frame_ready:
                    self.get_logger().info(
                        f"Camera frames ready! ({self._ros_streamer.streamer._frames_received} cached frames)"
                    )
                else:
                    self.get_logger().warning(
                        "No camera frames after 10s wait. Continuing anyway..."
                    )

                # Run the SDK's auto-reconnect loop which handles:
                # 1. Subscribe to MQTT command topics (start_video, stop_video)
                # 2. Wait for start_video command from cloud/frontend
                # 3. Establish WebRTC connection when commanded
                # 4. Monitor connection and auto-reconnect on disconnect
                #
                # NOTE: This does NOT proactively start WebRTC. It waits for commands.
                # This prevents timeout errors when no backend/frontend is ready to answer.
                self.get_logger().info(
                    "Starting SDK auto-reconnect loop (will wait for start_video commands)..."
                )

                await self._ros_streamer.run_with_auto_reconnect(
                    stop_event=self._webrtc_stop_event,
                    command_callback=on_command_response,
                )

                self.get_logger().info("SDK run_with_auto_reconnect() completed")
            except Exception as e:
                self.get_logger().error(f"SDK run_with_auto_reconnect() failed: {e}")
                import traceback

                self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            finally:
                self._webrtc_auto_reconnect_running = False

        # Run the coroutine in the async loop
        future = asyncio.run_coroutine_threadsafe(
            _run_with_auto_reconnect(), self._async_loop
        )

        # Store the future so we can check its status later
        self._webrtc_auto_reconnect_future = future

        self.get_logger().info(
            "WebRTC auto-reconnect loop started (waiting for start_video commands)"
        )

    def _schedule_auto_start_retry(self) -> None:
        """Reschedule auto-start if prerequisites are not ready yet."""
        retry_sec = getattr(self, "_auto_start_retry_sec", 5.0)
        self._auto_start_timer = self.create_timer(
            retry_sec, self._maybe_auto_start_webrtc
        )
        self.get_logger().info(f"WebRTC auto-start retry scheduled in {retry_sec:.1f}s")

    def _schedule_webrtc_retry(self, delay_sec: float = 30.0) -> None:
        """Schedule a retry of WebRTC stream start after a failure (e.g., signaling timeout)."""
        # Reset the streamer's internal state so it can try again
        if hasattr(self, "_ros_streamer") and self._ros_streamer is not None:
            try:
                # Reset SDK state flags for a fresh attempt
                self._ros_streamer._answer_received = False
                self._ros_streamer._answer_data = None
                # Close existing peer connection if any
                if hasattr(self._ros_streamer, "pc") and self._ros_streamer.pc:
                    asyncio.run_coroutine_threadsafe(
                        self._ros_streamer.pc.close(), self._async_loop
                    )
                    self._ros_streamer.pc = None
            except Exception as e:
                self.get_logger().debug(f"Error resetting streamer state: {e}")

        # Cancel existing retry timer if any
        if (
            hasattr(self, "_webrtc_retry_timer")
            and self._webrtc_retry_timer is not None
        ):
            try:
                self._webrtc_retry_timer.cancel()
            except Exception:
                pass

        self._webrtc_retry_timer = self.create_timer(
            delay_sec, self._webrtc_retry_callback
        )
        self.get_logger().info(f"WebRTC stream retry scheduled in {delay_sec:.1f}s")

    def _webrtc_retry_callback(self) -> None:
        """Callback for WebRTC retry timer."""
        if (
            hasattr(self, "_webrtc_retry_timer")
            and self._webrtc_retry_timer is not None
        ):
            try:
                self._webrtc_retry_timer.cancel()
            except Exception:
                pass
            self._webrtc_retry_timer = None

        self.get_logger().info("Retrying WebRTC stream start...")
        try:
            self.start_camera_stream()
        except Exception as e:
            self.get_logger().error(f"WebRTC retry failed: {e}")
            # Schedule another retry
            self._schedule_webrtc_retry(60.0)  # Back off to 60s on repeated failures

    def reset_internal_odometry(self) -> None:
        """Resets internal dead-reckoning variables to zero."""
        self._internal_pose_x = 0.0
        self._internal_pose_y = 0.0
        self._internal_pose_theta = 0.0
        self._last_left_pos = None
        self._last_right_pos = None
        self.get_logger().info("Internal odometry has been reset to (0,0,0)")

    def _apply_camera_param_overrides(self) -> None:
        """Merge params.yaml `camera:` overrides onto the mapping (sentinels ""/0 = skip)."""
        if not (hasattr(self, "_mapping") and self._mapping):
            return
        try:
            cam = self._mapping.raw.setdefault("camera", {})
        except Exception:
            return

        def _p(name):
            try:
                return self.get_parameter(name).value
            except Exception:
                return None

        changed: Dict[str, Any] = {}
        pf = _p("camera.pixel_format")
        if pf:
            cam["pixel_format"] = pf
            changed["pixel_format"] = pf
        w = _p("camera.image_width")
        if w and int(w) > 0:
            cam["image_width"] = int(w)
            changed["image_width"] = int(w)
        h = _p("camera.image_height")
        if h and int(h) > 0:
            cam["image_height"] = int(h)
            changed["image_height"] = int(h)
        fps = _p("camera.fps")
        if fps and int(fps) > 0:
            cam["capture_fps"] = int(fps)
            cam["stream_fps"] = min(int(cam.get("stream_fps", fps) or fps), int(fps))
            changed["fps"] = int(fps)
        if changed:
            self.get_logger().info(
                f"Camera params.yaml overrides applied over mapping: {changed}"
            )

    def start_camera_stream(
        self,
        recording: bool = True,
        fps: Optional[int] = None,
        pixel_format: Optional[str] = None,
        width: Optional[int] = None,
        height: Optional[int] = None,
    ) -> None:
        """Start the WebRTC stream via ROSCameraStreamer (SDK BaseVideoStreamer).

        Edge is the WebRTC offerer. If pixel_format/width/height are given and
        differ from the current managed-camera format, the camera is reconfigured
        (managed relaunch) first.
        """
        import inspect

        caller = "unknown"
        try:
            stack = inspect.stack()
            if len(stack) > 1:
                caller = stack[1].function
        except Exception:
            pass

        # Requested a specific format and we own the camera -> switch first
        # (set_camera_format restarts the stream), then return.
        if (
            self._camera_manager is not None
            and (pixel_format or width or height)
        ):
            result = self.set_camera_format(
                pixel_format=pixel_format, width=width, height=height, fps=fps,
                recording=recording,
            )
            if isinstance(result, dict) and result.get("status") == "error":
                self.get_logger().error(
                    f"start_camera_stream: requested format rejected: {result.get('message')}"
                )
            return

        # When run_with_auto_reconnect() is active, the SDK handles start_video commands
        # internally. We should NOT also call start() ourselves as it causes duplicate
        # WebRTC offers and connection confusion.
        if getattr(self, "_webrtc_auto_reconnect_running", False):
            self.get_logger().info(
                f"start_camera_stream called by {caller}, but auto-reconnect is running. "
                "SDK will handle start_video commands internally - skipping duplicate start."
            )
            return

        # Resolve FPS from parameter, mapping, or default
        if fps is None:
            try:
                # Try parameter first
                fps = self.get_parameter("webrtc.fps").value
            except Exception:
                try:
                    # Try mapping
                    fps = self._mapping.raw.get("camera", {}).get("fps", 30)
                except Exception:
                    fps = 30  # Default to 30

        self.get_logger().info(
            f"Starting WebRTC camera stream (recording={recording}, fps={fps})... (called by: {caller})"
        )
        try:
            twin_uuid = getattr(self._mapping, "twin_uuid", None)
            if not twin_uuid:
                self.get_logger().error(
                    "Cannot start camera stream: twin_uuid not found in mapping"
                )
                return

            if not hasattr(self, "_ros_streamer") or self._ros_streamer is None:
                self.get_logger().error(
                    "Cannot start camera stream: ROSCameraStreamer was not pre-initialized. "
                    "Check that twin_uuid is configured in your mapping file."
                )
                return

            # Check if it's already running or starting
            # Check if there's an ongoing start operation
            if (
                hasattr(self, "_webrtc_start_future")
                and self._webrtc_start_future is not None
            ):
                if not self._webrtc_start_future.done():
                    self.get_logger().warning(
                        "WebRTC start already in progress; ignoring duplicate start_video command"
                    )
                    return

            existing_pc = getattr(self._ros_streamer, "pc", None)
            if existing_pc and existing_pc.connectionState not in ["closed", "failed"]:
                self.get_logger().info(
                    "WebRTC streamer already active; skipping restart"
                )
                return

            # Use pre-initialized streamer (it's already caching frames)
            self.get_logger().info("Using pre-initialized camera streamer...")

            # Pass recording preference to streamer
            self._ros_streamer._should_record = recording

            self.get_logger().info(
                f"Triggering async streamer start (Edge as Offerer) for twin {twin_uuid}"
            )

            # Wrap the coroutine to capture exceptions
            # NOTE: self._ros_streamer is ROSCameraStreamer which extends SDK's BaseVideoStreamer
            # Calling .start() uses the SDK's WebRTC signaling methods internally
            # This is the correct approach - we're using SDK functionality, not bypassing it
            async def _start_with_logging():
                try:
                    self.get_logger().info(
                        "SDK BaseVideoStreamer.start() coroutine beginning..."
                    )
                    # This calls SDK's BaseVideoStreamer.start() which handles WebRTC offer/answer
                    await self._ros_streamer.start()
                    self.get_logger().info(
                        "SDK BaseVideoStreamer.start() coroutine completed successfully"
                    )
                except Exception as e:
                    self.get_logger().error(
                        f"SDK BaseVideoStreamer.start() failed: {e}"
                    )
                    import traceback

                    self.get_logger().error(f"Traceback: {traceback.format_exc()}")
                    raise

            future = asyncio.run_coroutine_threadsafe(
                _start_with_logging(), self._async_loop
            )
            self._webrtc_start_future = future  # Track this operation

            # Add a callback to check for errors after completion and schedule retry
            def _on_done(fut):
                try:
                    fut.result()  # This will raise if the coroutine failed
                    # Clear the future reference on success
                    self._webrtc_start_future = None
                except TimeoutError as e:
                    self.get_logger().warning(f"WebRTC signaling timed out: {e}")
                    self.get_logger().info(
                        "Backend may not be running. Will retry WebRTC auto-start in 30s..."
                    )
                    self._webrtc_start_future = None  # Clear the future
                    # Schedule a retry - the auto-start mechanism will handle this
                    self._schedule_webrtc_retry(30.0)
                except Exception as e:
                    self.get_logger().error(f"Async streamer start failed: {e}")
                    self._webrtc_start_future = None  # Clear the future
                    # Schedule retry for other failures too
                    self._schedule_webrtc_retry(30.0)

            future.add_done_callback(_on_done)

            self.get_logger().info(
                f"Camera stream object initialized for twin {twin_uuid}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start camera stream: {e}")

    def stop_camera_stream(self) -> None:
        """Stops the camera stream using SDK's BaseVideoStreamer.stop() method.

        NOTE: self._ros_streamer extends SDK's BaseVideoStreamer, so calling
        .stop() uses the SDK's cleanup and connection termination logic.
        """
        if hasattr(self, "_ros_streamer") and self._ros_streamer is not None:
            try:
                # Use SDK's BaseVideoStreamer.stop() method which properly handles cleanup
                self.get_logger().info(
                    "Calling SDK BaseVideoStreamer.stop() to close WebRTC stream..."
                )
                asyncio.run_coroutine_threadsafe(
                    self._ros_streamer.stop(), self._async_loop
                )

                self.get_logger().info(
                    "Camera WebRTC stream stopped (track remains active for pre-caching)"
                )

                # Notify frontend (ROS service / direct stop paths without actuation handler)
                twin_uuid = getattr(self._mapping, "twin_uuid", None)
                if twin_uuid:
                    topic = f"{self.topic_prefix}cyberwave/twin/{twin_uuid}/command"
                    payload = {
                        "command": "stop_video",
                        "type": "response",
                        "source_type": "edge",
                        "status": "ok",
                        "data": {"status": "ok", "type": "video_stopped"},
                    }
                    self.publish(topic, payload)
                    self.get_logger().info(
                        f"Sent video_stopped notification to {topic}"
                    )
            except Exception as e:
                self.get_logger().error(f"Error stopping camera stream: {e}")

    def set_camera_format(
        self,
        pixel_format: Optional[str] = None,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
        recording: bool = True,
    ) -> Dict[str, Any]:
        """Switch capture format at runtime: validate -> stop stream -> reconfigure
        usb_cam -> restart. Returns {"status": "ok"|"error", "applied": {...}}.
        """
        mgr = self._camera_manager
        if mgr is None:
            return {"status": "error", "message": "camera not managed by bridge"}

        # Fill unspecified fields from the current/default config.
        cam_cfg = self._mapping.raw.get("camera", {}) if self._mapping else {}
        cur = mgr.current or (
            cam_cfg.get("pixel_format", "mjpeg2rgb"),
            int(cam_cfg.get("image_width", 1920)),
            int(cam_cfg.get("image_height", 1080)),
            int(cam_cfg.get("capture_fps", 30)),
        )
        pf = pixel_format or cur[0]
        w = int(width or cur[1])
        h = int(height or cur[2])
        cap_fps = int(fps or cur[3])

        try:
            mgr.validate(pf, w, h, cap_fps)
        except FormatValidationError as exc:
            return {"status": "error", "message": str(exc)}

        was_streaming = (
            getattr(self, "_ros_streamer", None) is not None
            and getattr(self._ros_streamer, "pc", None) is not None
        )
        self.get_logger().info(
            f"set_camera_format -> {pf} {w}x{h}@{cap_fps} (was_streaming={was_streaming})"
        )

        # Stop stream (frees the track + /image_raw subscription) before reconfig.
        if was_streaming:
            try:
                fut = asyncio.run_coroutine_threadsafe(
                    self._ros_streamer.stop(), self._async_loop
                )
                fut.result(timeout=10.0)
            except Exception as exc:
                self.get_logger().warning(f"Error stopping stream before reformat: {exc}")

        try:
            mgr.reconfigure(pf, w, h, cap_fps)   # kill -> reap -> device-free -> relaunch
        except FormatValidationError as exc:
            return {"status": "error", "message": str(exc)}
        except Exception as exc:
            self.get_logger().error(f"Camera reconfigure failed: {exc}")
            return {"status": "error", "message": f"reconfigure failed: {exc}"}

        # Streamer rebuilds a fresh track on next start() -> just restart if it was up.
        applied = {"pixel_format": pf, "width": w, "height": h, "fps": cap_fps}
        if was_streaming:
            self.start_camera_stream(recording=recording, fps=fps)
        return {"status": "ok", "applied": applied}

    async def _set_stop_event(self) -> None:
        """Helper to set the stop event from the async loop."""
        if hasattr(self, "_webrtc_stop_event"):
            self._webrtc_stop_event.set()

    def _watchdog_image_callback(self, msg: Image) -> None:
        """Callback for the camera watchdog to track the last received image time."""
        self._last_image_time = time.time()

    def _check_camera_status(self) -> None:
        """Periodic check of the camera availability and settings."""
        now = time.time()

        # 1. Resolve the device path. For the bridge-managed camera, use the
        # manager's resolved path (the mapping value may be "auto").
        managed = self._camera_manager is not None
        if managed:
            video_device = self._camera_manager.video_device
        else:
            video_device = "/dev/video0"
            if hasattr(self, "_mapping") and self._mapping:
                video_device = self._mapping.raw.get("camera", {}).get(
                    "video_device", video_device
                )

        device_exists = os.path.exists(video_device)

        # 2. Are we receiving images?
        is_receiving = (now - self._last_image_time) < 5.0
        silent_secs = now - self._last_image_time

        # 3. Is the usb_cam node in the graph? (managed: also trust the process)
        node_names = self.get_node_names()
        usb_cam_running = any("usb_cam" in name for name in node_names)
        if managed:
            usb_cam_running = usb_cam_running or self._camera_manager.is_running()

        streaming = (
            getattr(self, "_ros_streamer", None) is not None
            and getattr(self._ros_streamer, "streamer", None) is not None
            and getattr(self._ros_streamer, "pc", None) is not None
        )

        # 4. Decide + execute recovery (managed camera only); else just log.
        action = decide_recovery(
            managed=managed,
            device_exists=device_exists,
            usb_cam_running=usb_cam_running,
            receiving=is_receiving,
            streaming=streaming,
            silent_secs=silent_secs,
        )

        if action != RECOVERY_NONE:
            self._recover_camera(action, video_device)
            return

        if not is_receiving and not managed:
            self.get_logger().warn(
                f"CAMERA WATCHDOG: {video_device} exists but NO IMAGES on /image_raw (unmanaged)"
            )
        elif is_receiving and now - self._last_camera_check_time > 60.0:
            self.get_logger().info(
                f"CAMERA WATCHDOG: Camera OK ({video_device} active, streaming at /image_raw)"
            )
            track = (
                getattr(self._ros_streamer, "streamer", None)
                if getattr(self, "_ros_streamer", None)
                else None
            )
            if track is not None:
                tf, ts = track.get_quality() if hasattr(track, "get_quality") else (track.fps, 1.0)
                self.get_logger().info(
                    f"CAMERA STATS: {track.actual_width}x{track.actual_height} "
                    f"send_fps={tf} scale={ts} encoding={track.encoding}"
                )
            self._last_camera_check_time = now

    def _default_camera_format(self):
        cam = self._mapping.raw.get("camera", {}) if self._mapping else {}
        return (
            cam.get("pixel_format", "mjpeg2rgb"),
            int(cam.get("image_width", 1920)),
            int(cam.get("image_height", 1080)),
            int(cam.get("capture_fps", cam.get("fps", 30))),
        )

    def _recover_camera(self, action: str, device: str) -> None:
        """Execute a watchdog recovery action against the managed camera."""
        mgr = self._camera_manager
        if mgr is None:
            return
        try:
            if action == RECOVERY_RERESOLVE:
                newdev = mgr.reresolve_device()
                self.get_logger().error(
                    f"CAMERA WATCHDOG: device {device} missing; re-resolved to {newdev}"
                )
                if os.path.exists(newdev):
                    mgr.start(*(mgr.current or self._default_camera_format()))
            elif action == RECOVERY_RESTART:
                self.get_logger().error(
                    "CAMERA WATCHDOG: usb_cam not running; restarting managed camera"
                )
                mgr.start(*(mgr.current or self._default_camera_format()))
            elif action == RECOVERY_RECONFIGURE:
                self.get_logger().error(
                    "CAMERA WATCHDOG: no frames for >10s; reconfiguring camera (clean device cycle)"
                )
                mgr.reconfigure(*(mgr.current or self._default_camera_format()))
        except Exception as exc:
            self.get_logger().error(f"CAMERA WATCHDOG recovery ({action}) failed: {exc}")

    def destroy_node(self) -> None:
        try:
            if hasattr(self, "_ros_streamer") and self._ros_streamer:
                # Fully stop the streamer including subscriptions
                asyncio.run_coroutine_threadsafe(
                    self._ros_streamer.stop(), self._async_loop
                )
            loop = getattr(self, "_async_loop", None)
            if loop:
                loop.call_soon_threadsafe(loop.stop)
            th = getattr(self, "_async_loop_thread", None)
            if th:
                th.join(timeout=1.0)
            ev = getattr(self, "_mapping_watcher_stop", None)
            if ev:
                ev.set()
            th = getattr(self, "_mapping_watcher_thread", None)
            if th:
                th.join(timeout=1.0)
        except Exception:
            pass

        try:
            adapter = getattr(self, "_mqtt_adapter", None)
            if adapter:
                adapter.disconnect()
            else:
                self._mqtt_client.loop_stop()
                self._mqtt_client.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MQTTBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        import traceback

        logging.getLogger("mqtt_bridge_node").error(
            f"Node crashed: {e}\n{traceback.format_exc()}"
        )
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
