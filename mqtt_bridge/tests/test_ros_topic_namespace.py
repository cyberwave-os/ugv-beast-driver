"""Regression tests for twin-scoped ROS topic namespacing (#2874)."""

from __future__ import annotations

import pytest

from mqtt_bridge.ros_topic_namespace import resolve_ros_namespace, resolve_ros_topic

TWIN_UUID = "00000000-0000-0000-0000-000000000001"


class TestResolveRosNamespace:
    def test_prefers_configured_namespace(self) -> None:
        assert (
            resolve_ros_namespace(
                configured_namespace="/custom/ns",
                twin_uuid=TWIN_UUID,
            )
            == "custom/ns"
        )

    def test_derives_namespace_from_twin_uuid(self) -> None:
        # ugv_beast_ + first 6 hex chars of the uuid; the prefix guarantees a
        # ROS-valid leading letter (a raw UUID is an invalid ROS name).
        assert resolve_ros_namespace(twin_uuid=TWIN_UUID) == "ugv_beast_000000"
        assert resolve_ros_namespace(twin_uuid="27dca72f-6e17-4cb4") == "ugv_beast_27dca7"

    def test_configured_namespace_wins_over_twin_uuid(self) -> None:
        assert (
            resolve_ros_namespace(configured_namespace="custom_ns", twin_uuid=TWIN_UUID)
            == "custom_ns"
        )

    def test_returns_empty_when_unconfigured(self) -> None:
        assert resolve_ros_namespace() == ""

    def test_sanitizes_invalid_configured_namespace(self) -> None:
        # Hyphens -> '_'; a leading digit is prefixed so the token is ROS-valid.
        assert resolve_ros_namespace(configured_namespace="12-ab/c-d") == "_12_ab/c_d"


class TestResolveRosTopic:
    @pytest.mark.parametrize(
        ("topic", "namespace", "expected"),
        [
            ("/cmd_vel", "", "/cmd_vel"),
            ("/cmd_vel", TWIN_UUID, f"/{TWIN_UUID}/cmd_vel"),
            (f"/{TWIN_UUID}/cmd_vel", TWIN_UUID, f"/{TWIN_UUID}/cmd_vel"),
            (f"{TWIN_UUID}/joint_states", TWIN_UUID, f"/{TWIN_UUID}/joint_states"),
            ("cmd_vel", TWIN_UUID, f"/{TWIN_UUID}/cmd_vel"),
            ("", TWIN_UUID, ""),
            ("   ", TWIN_UUID, ""),
        ],
    )
    def test_namespace_resolution(
        self, topic: str, namespace: str, expected: str
    ) -> None:
        assert resolve_ros_topic(topic, namespace) == expected

    def test_idempotent_for_already_namespaced_topics(self) -> None:
        namespaced = f"/{TWIN_UUID}/cmd_vel"
        assert resolve_ros_topic(namespaced, TWIN_UUID) == namespaced
