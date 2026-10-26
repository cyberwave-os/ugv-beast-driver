#!/usr/bin/env python3
"""UGV Beast bring-up node: bridges the STM32 base board (UART/JSON) to ROS 2.

ROS-free logic (unit conversions, command shaping, protocol parsing) lives in
``ugv_driver_core`` so it can be unit-tested without rclpy/serial; this module
is intentionally limited to ROS communication and hardware I/O.
"""

import json
import queue
import subprocess
import threading

import serial

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Header, Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState

from ugv_bringup import ugv_driver_core as core


def _reliable_qos(depth: int) -> QoSProfile:
    """KEEP_LAST + RELIABLE profile (identical wire behaviour to a bare int depth).

    TODO(CYB): the high-rate sensor topics (imu/data_raw, imu/mag, odom/odom_raw)
    would ideally use ``rclpy.qos.qos_profile_sensor_data`` (BEST_EFFORT). That is
    deferred because the current subscribers (upstream ugv_base_node, mqtt_bridge)
    advertise RELIABLE; flipping unilaterally would silently drop delivery. Named
    profiles here replace the previous magic integer depths.
    """
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


# Helper class for reading newline-delimited frames from a serial port.
class ReadLine:
    def __init__(self, s, lock=None):
        self.buf = bytearray()
        self.s = s
        self._lock = lock or threading.Lock()

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            try:
                i = max(1, min(512, self.s.in_waiting))
                data = self.s.read(i)
                i = data.find(b"\n")
                if i >= 0:
                    r = self.buf + data[:i + 1]
                    self.buf[0:] = data[i + 1:]
                    return r
                else:
                    self.buf.extend(data)
            except Exception:
                return b""

    def clear_buffer(self):
        # Guarded: a buffer flush must not interleave with a concurrent write.
        with self._lock:
            self.s.reset_input_buffer()


# Manages UART communication with the base board and the outbound command queue.
class BaseController:
    def __init__(self, uart_dev_set, baud_set, logger):
        self._logger = logger  # ROS logger injected by the node (single logging system)
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)
        self._serial_lock = threading.Lock()  # guards writes / input-buffer flush
        self.rl = ReadLine(self.ser, self._serial_lock)
        self.command_queue = queue.Queue()
        self._stop = threading.Event()
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)
        self.command_thread.start()
        self.data_buffer = None
        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0,
                          "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0,
                          "odl": 0, "odr": 0, "v": 0}

    # Read one telemetry frame; returns a dict or None (never raises).
    def feedback_data(self):
        try:
            line_bytes = self.rl.readline()
            if not line_bytes:
                return None
            parsed = core.parse_base_frame(line_bytes)
            if parsed is None:
                # Malformed/garbage line (serial noise) — drop and resync.
                self.rl.clear_buffer()
                return None
            self.data_buffer = parsed
            self.base_data = parsed
            return self.base_data
        except Exception as e:
            self._logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()
            return None

    def send_command(self, data):
        self.command_queue.put(data)

    # Background thread: drain the queue and write commands as JSON over UART.
    def process_commands(self):
        while not self._stop.is_set():
            try:
                data = self.command_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if data is None:  # shutdown sentinel
                break
            with self._serial_lock:
                try:
                    self.ser.write((json.dumps(data) + '\n').encode("utf-8"))
                except Exception as e:
                    self._logger.error(f"[base_ctrl.process_commands] serial write failed: {e}")

    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

    # Stop the command thread and close the port (called on node shutdown).
    def stop(self):
        self._stop.set()
        self.command_queue.put(None)  # unblock the queue.get
        self.command_thread.join(timeout=1.0)
        try:
            if self.ser.is_open:
                self.ser.close()
        except Exception:
            pass


# ROS node: publishes base-board sensor data and forwards control commands.
class UgvBringupNode(Node):
    def __init__(self):
        super().__init__('ugv_bringup')

        # --- Parameters (defaults preserve the previous hardcoded behaviour) ---
        self.declare_parameter('serial_port', core.detect_default_serial_port())
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('feedback_period_s', 0.05)
        self.declare_parameter('imu_frame_id', 'base_imu_link')
        self.declare_parameter('low_battery_volts', 9.0)
        self.declare_parameter('alert_interval_s', 30.0)
        self.declare_parameter(
            'low_battery_wav',
            '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav')
        self.declare_parameter('alsa_device', 'plughw:3,0')

        self._imu_frame_id = self.get_parameter('imu_frame_id').value
        self._low_battery_volts = self.get_parameter('low_battery_volts').value
        self._low_battery_wav = self.get_parameter('low_battery_wav').value
        self._alsa_device = self.get_parameter('alsa_device').value
        self._alert_interval_ns = int(self.get_parameter('alert_interval_s').value * 1e9)
        self._last_alert_ns = None
        self._alert_proc = None

        # Publishers (named QoS profiles instead of magic integer depths)
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", _reliable_qos(100))
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", _reliable_qos(100))
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", _reliable_qos(100))
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", _reliable_qos(50))

        # Subscribers for control commands (default callback group)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, _reliable_qos(10))
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, _reliable_qos(10))
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, _reliable_qos(10))

        # Initialize the base controller with the configured UART port / baud.
        self.base_controller = BaseController(
            self.get_parameter('serial_port').value,
            self.get_parameter('baud_rate').value,
            self.get_logger(),
        )
        # Feedback loop does a (potentially blocking) serial read, so it runs in
        # its OWN mutually-exclusive callback group. With a MultiThreadedExecutor
        # this keeps cmd_vel and the other control callbacks responsive instead
        # of being serialized behind the serial read.
        self._io_cb_group = MutuallyExclusiveCallbackGroup()
        self.feedback_timer = self.create_timer(
            self.get_parameter('feedback_period_s').value,
            self.feedback_loop,
            callback_group=self._io_cb_group,
        )

    # Forward velocity commands to the base board.
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = core.apply_turn_in_place_deadband(linear_velocity, msg.angular.z)
        data = {'T': '13', 'X': linear_velocity, 'Z': angular_velocity}
        self.base_controller.send_command(data)

    # Forward pan/tilt joint commands (teleoperation source only).
    def joint_states_callback(self, msg):
        # NOTE: 'tele' is the system-wide source_type convention carried (today)
        # in header.frame_id; see mqtt_bridge/mapping.py. Migrating it to a
        # dedicated message field is a tracked cross-repo change (deferred).
        source_type = msg.header.frame_id
        if source_type != 'tele':
            return

        name = msg.name
        position = msg.position
        try:
            x_rad = position[name.index('pt_base_link_to_pt_link1')]
            y_rad = position[name.index('pt_link1_to_pt_link2')]
            x_degree, y_degree = core.joint_rad_to_servo_degrees(x_rad, y_rad)
            joint_data = {'T': 134, 'X': x_degree, 'Y': y_degree, "SX": 600, "SY": 600}
            # Throttled: this fires on every teleop joint update (high frequency).
            self.get_logger().info(f"[PT_SERVO] Sending to UART: {joint_data}",
                                   throttle_duration_sec=2.0)
            self.base_controller.send_command(joint_data)
        except (ValueError, IndexError) as e:
            self.get_logger().warning(f"[PT_SERVO] Failed to extract joint positions: {e}")

    # Forward LED control commands.
    def led_ctrl_callback(self, msg):
        if len(msg.data) >= 2:
            led_ctrl_data = {'T': 132, "IO4": msg.data[0], "IO5": msg.data[1]}
            self.base_controller.send_command(led_ctrl_data)

    # Read sensor feedback and publish it to ROS topics.
    def feedback_loop(self):
        data = self.base_controller.feedback_data()
        if data and isinstance(data, dict) and data.get("T") == 1001:
            self.publish_imu_data_raw()
            self.publish_imu_mag()
            self.publish_odom_raw()
            self.publish_voltage()

    def publish_imu_data_raw(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame_id
        d = self.base_controller.base_data
        msg.linear_acceleration.x = core.accel_to_mps2(d["ax"])
        msg.linear_acceleration.y = core.accel_to_mps2(d["ay"])
        msg.linear_acceleration.z = core.accel_to_mps2(d["az"])
        msg.angular_velocity.x = core.gyro_to_rad_s(d["gx"])
        msg.angular_velocity.y = core.gyro_to_rad_s(d["gy"])
        msg.angular_velocity.z = core.gyro_to_rad_s(d["gz"])
        self.imu_data_raw_publisher_.publish(msg)

    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._imu_frame_id
        d = self.base_controller.base_data
        # Some hardware has no magnetometer; publish zeros if data is missing.
        try:
            msg.magnetic_field.x = core.mag_to_field(d.get("mx", 0))
            msg.magnetic_field.y = core.mag_to_field(d.get("my", 0))
            msg.magnetic_field.z = core.mag_to_field(d.get("mz", 0))
        except (KeyError, TypeError, ValueError):
            msg.magnetic_field.x = 0.0
            msg.magnetic_field.y = 0.0
            msg.magnetic_field.z = 0.0
        self.imu_mag_publisher_.publish(msg)

    def publish_odom_raw(self):
        d = self.base_controller.base_data
        array = [core.odom_counts_to_m(d["odl"]), core.odom_counts_to_m(d["odr"])]
        msg = Float32MultiArray(data=array)
        self.odom_publisher_.publish(msg)

    def publish_voltage(self):
        d = self.base_controller.base_data
        voltage_value = core.raw_to_volts(d["v"])
        msg = Float32()
        msg.data = voltage_value
        self.voltage_publisher_.publish(msg)

        if core.is_low_battery(voltage_value, threshold=self._low_battery_volts):
            self._maybe_alert_low_battery()

    # Fire-and-forget, rate-limited low-battery chime. MUST NOT block: this runs
    # on the executor, so a blocking subprocess.run + time.sleep (the old code)
    # would freeze cmd_vel handling and let the robot keep driving.
    def _maybe_alert_low_battery(self):
        now_ns = self.get_clock().now().nanoseconds
        if not core.alert_due(now_ns, self._last_alert_ns, self._alert_interval_ns):
            return
        # Skip if the previous chime is still playing.
        if self._alert_proc is not None and self._alert_proc.poll() is None:
            return
        self._last_alert_ns = now_ns
        self.get_logger().warning('Low battery detected — playing alert',
                                  throttle_duration_sec=30.0)
        try:
            self._alert_proc = subprocess.Popen(
                ['aplay', '-D', self._alsa_device, self._low_battery_wav],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as e:
            self.get_logger().warning(f'Low-battery alert could not start: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = UgvBringupNode()
    # MultiThreadedExecutor so the blocking serial read in feedback_loop (its own
    # callback group) cannot starve the control callbacks. Henki prefers a single
    # threaded executor in general; this node genuinely needs concurrency.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.base_controller.stop()  # stop command thread + close serial port
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
