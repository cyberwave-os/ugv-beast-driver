"""ROS-free core logic for the UGV Beast driver.

Separated from the ROS node (``ugv_integrated_driver.py``) so the unit
conversions, command shaping and protocol parsing can be unit-tested without
``rclpy`` or a serial device. Henki best practice: *"Separate application logic
into its own class or library, keeping ROS 2 nodes focused solely on
communication."*

Every conversion here reproduces the exact arithmetic the node used before the
refactor, so published values are unchanged (the only deliberate numeric change
is using ``math.pi`` via ``math.radians``/``math.degrees`` instead of the
hard-coded ``3.1415926`` literal — a <1e-7 difference).
"""

from __future__ import annotations

import json
import math
import os

# --- Hardware / wire-protocol constants (were magic numbers in the node) ---
# NOTE: GRAVITY_MPS2 stays 9.8 (not 9.80665) to keep IMU output bit-identical
# to the pre-refactor behaviour; changing it would shift values seen by
# downstream consumers (upstream ugv_base_node / imu filters).
GRAVITY_MPS2 = 9.8
ACCEL_LSB_PER_G = 8192.0          # raw accelerometer counts per 1 g
GYRO_LSB_PER_DEG_S = 16.4         # raw gyro counts per deg/s
MAG_SCALE = 0.15                  # raw magnetometer counts -> field units
ODOM_COUNTS_PER_M = 100.0         # raw wheel-odometry counts per metre
VOLTAGE_SCALE = 100.0             # raw battery counts per volt
DEFAULT_MIN_TURN_RATE = 0.2       # rad/s floor applied when turning in place
DEFAULT_MIN_VALID_VOLTAGE = 0.1   # below this the reading is treated as noise

# NVIDIA Jetson images ship this file; its presence distinguishes the Jetson
# carrier (ttyTHS1) from the Raspberry Pi (ttyAMA0). Replaces an os.walk("/")
# over the entire filesystem that the node ran at import time.
JETSON_MARKER = "/etc/nv_tegra_release"
JETSON_SERIAL_PORT = "/dev/ttyTHS1"
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"


def detect_default_serial_port(jetson_marker: str = JETSON_MARKER) -> str:
    """Return the UART device for this platform via a single O(1) stat."""
    return JETSON_SERIAL_PORT if os.path.exists(jetson_marker) else DEFAULT_SERIAL_PORT


def apply_turn_in_place_deadband(
    linear: float, angular: float, min_rate: float = DEFAULT_MIN_TURN_RATE
) -> float:
    """Floor a tiny rotation command when driving in place.

    When ``linear == 0`` and ``0 < |angular| < min_rate`` the base will not
    actually turn, so bump the magnitude up to ``min_rate`` (sign preserved).
    Returns the adjusted angular rate; all other cases pass through unchanged.
    """
    if linear == 0.0 and angular != 0.0 and abs(angular) < min_rate:
        return math.copysign(min_rate, angular)
    return angular


def accel_to_mps2(raw: float) -> float:
    """Raw accelerometer count -> m/s^2 (was ``9.8 * raw / 8192``)."""
    return GRAVITY_MPS2 * float(raw) / ACCEL_LSB_PER_G


def gyro_to_rad_s(raw: float) -> float:
    """Raw gyro count -> rad/s (was ``pi * raw / (16.4 * 180)``)."""
    return math.radians(float(raw) / GYRO_LSB_PER_DEG_S)


def mag_to_field(raw: float) -> float:
    """Raw magnetometer count -> field units (was ``raw * 0.15``)."""
    return float(raw) * MAG_SCALE


def odom_counts_to_m(raw: float) -> float:
    """Raw wheel-odometry count -> metres (was ``raw / 100``)."""
    return float(raw) / ODOM_COUNTS_PER_M


def raw_to_volts(raw: float) -> float:
    """Raw battery count -> volts (was ``raw / 100``)."""
    return float(raw) / VOLTAGE_SCALE


def joint_rad_to_servo_degrees(x_rad: float, y_rad: float) -> tuple[int, int]:
    """Pan/tilt radians -> integer degrees (the STM32 JSON parser wants ints)."""
    return int(math.degrees(x_rad)), int(math.degrees(y_rad))


def is_low_battery(
    voltage: float,
    threshold: float,
    min_valid: float = DEFAULT_MIN_VALID_VOLTAGE,
) -> bool:
    """True when a *valid* reading is under ``threshold`` (ignores 0/noise)."""
    return min_valid < voltage < threshold


def alert_due(now_ns: int, last_alert_ns: int | None, interval_ns: int) -> bool:
    """Rate-limit decision: True if ``interval_ns`` has elapsed (or first time).

    Used to throttle the low-battery audio alert without blocking the executor
    (the old code did ``subprocess.run`` + ``time.sleep(5)`` on the timer thread).
    """
    if last_alert_ns is None:
        return True
    return (now_ns - last_alert_ns) >= interval_ns


def parse_base_frame(line: str | bytes) -> dict | None:
    """Parse one JSON telemetry line into a dict, or ``None`` (never raises)."""
    try:
        if isinstance(line, (bytes, bytearray)):
            line = line.decode("utf-8")
        obj = json.loads(line)
    except (ValueError, TypeError, UnicodeDecodeError):
        return None
    return obj if isinstance(obj, dict) else None
