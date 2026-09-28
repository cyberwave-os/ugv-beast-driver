"""Unit tests for the ROS-free driver core (no rclpy / no serial device).

Run (host venv or any python with pytest):
    PYTHONPATH=../ugv_bringup pytest test/test_ugv_driver_core.py
"""

import math

import pytest

import ugv_driver_core as core


# --- detect_default_serial_port ------------------------------------------
def test_serial_port_jetson(tmp_path):
    marker = tmp_path / "nv_tegra_release"
    marker.write_text("R35\n")
    assert core.detect_default_serial_port(str(marker)) == core.JETSON_SERIAL_PORT


def test_serial_port_pi(tmp_path):
    missing = tmp_path / "nope"
    assert core.detect_default_serial_port(str(missing)) == core.DEFAULT_SERIAL_PORT


# --- apply_turn_in_place_deadband (matches the original branch logic) -----
@pytest.mark.parametrize(
    "linear, angular, expected",
    [
        (0.0, 0.0, 0.0),        # no rotation -> untouched
        (0.0, 0.1, 0.2),        # tiny +rotation in place -> floored
        (0.0, -0.1, -0.2),      # tiny -rotation in place -> floored
        (0.0, 0.2, 0.2),        # already at floor -> unchanged
        (0.0, 0.5, 0.5),        # above floor -> unchanged
        (1.0, 0.05, 0.05),      # moving forward -> deadband NOT applied
    ],
)
def test_deadband(linear, angular, expected):
    assert core.apply_turn_in_place_deadband(linear, angular) == pytest.approx(expected)


# --- conversions reproduce the original arithmetic ------------------------
def test_accel_matches_original():
    assert core.accel_to_mps2(8192) == pytest.approx(9.8)
    assert core.accel_to_mps2(0) == 0.0


def test_gyro_matches_original():
    # original: 3.1415926 * raw / (16.4 * 180)
    raw = 1640
    original = 3.1415926 * raw / (16.4 * 180)
    assert core.gyro_to_rad_s(raw) == pytest.approx(original, abs=1e-6)


def test_mag_and_odom_and_voltage():
    assert core.mag_to_field(10) == pytest.approx(1.5)
    assert core.odom_counts_to_m(250) == pytest.approx(2.5)
    assert core.raw_to_volts(1234) == pytest.approx(12.34)


def test_joint_rad_to_servo_degrees_matches_original():
    x_rad, y_rad = 0.5, -1.0
    x_deg, y_deg = core.joint_rad_to_servo_degrees(x_rad, y_rad)
    assert (x_deg, y_deg) == (int((180 * x_rad) / 3.1415926), int((180 * y_rad) / 3.1415926))
    assert isinstance(x_deg, int) and isinstance(y_deg, int)


# --- is_low_battery (ignores 0/noise, matches `0.1 < v < 9` original) ------
@pytest.mark.parametrize(
    "voltage, expected",
    [(0.0, False), (0.05, False), (8.5, True), (9.0, False), (12.0, False)],
)
def test_is_low_battery(voltage, expected):
    assert core.is_low_battery(voltage, threshold=9.0) is expected


# --- alert_due throttling --------------------------------------------------
def test_alert_due_first_time():
    assert core.alert_due(now_ns=1000, last_alert_ns=None, interval_ns=500) is True


def test_alert_due_within_interval():
    assert core.alert_due(now_ns=1400, last_alert_ns=1000, interval_ns=500) is False


def test_alert_due_after_interval():
    assert core.alert_due(now_ns=1500, last_alert_ns=1000, interval_ns=500) is True
    assert core.alert_due(now_ns=1600, last_alert_ns=1000, interval_ns=500) is True


# --- parse_base_frame never raises ---------------------------------------
def test_parse_base_frame_ok():
    assert core.parse_base_frame('{"T": 1001, "v": 1200}') == {"T": 1001, "v": 1200}
    assert core.parse_base_frame(b'{"T": 1001}') == {"T": 1001}


@pytest.mark.parametrize("bad", ["", "not json", "[1,2,3]", b"\xff\xfe", "123"])
def test_parse_base_frame_bad_returns_none(bad):
    assert core.parse_base_frame(bad) is None
