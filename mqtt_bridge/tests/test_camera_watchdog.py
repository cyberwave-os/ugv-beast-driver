"""Step 6 gate: watchdog recovery decision (decide_recovery)."""

from __future__ import annotations

from mqtt_bridge.plugins.camera_device_manager import (
    decide_recovery,
    RECOVERY_NONE,
    RECOVERY_RESTART,
    RECOVERY_RECONFIGURE,
    RECOVERY_RERESOLVE,
)


def test_unmanaged_camera_never_auto_recovers():
    assert (
        decide_recovery(
            managed=False, device_exists=False, usb_cam_running=False,
            receiving=False, streaming=True, silent_secs=999,
        )
        == RECOVERY_NONE
    )


def test_device_missing_triggers_reresolve():
    # USB re-enumerated / unplugged -> re-resolve the /dev/video* path.
    assert (
        decide_recovery(
            managed=True, device_exists=False, usb_cam_running=True,
            receiving=False, streaming=True, silent_secs=12,
        )
        == RECOVERY_RERESOLVE
    )


def test_usb_cam_process_gone_triggers_restart():
    assert (
        decide_recovery(
            managed=True, device_exists=True, usb_cam_running=False,
            receiving=False, streaming=False, silent_secs=3,
        )
        == RECOVERY_RESTART
    )


def test_silent_while_streaming_triggers_reconfigure():
    assert (
        decide_recovery(
            managed=True, device_exists=True, usb_cam_running=True,
            receiving=False, streaming=True, silent_secs=12,
        )
        == RECOVERY_RECONFIGURE
    )


def test_brief_silence_does_not_reconfigure():
    # Under the threshold -> no action (avoid thrashing the camera).
    assert (
        decide_recovery(
            managed=True, device_exists=True, usb_cam_running=True,
            receiving=False, streaming=True, silent_secs=4,
        )
        == RECOVERY_NONE
    )


def test_healthy_camera_no_action():
    assert (
        decide_recovery(
            managed=True, device_exists=True, usb_cam_running=True,
            receiving=True, streaming=True, silent_secs=0,
        )
        == RECOVERY_NONE
    )


def test_not_streaming_idle_camera_no_reconfigure():
    # Not streaming and no frames is normal (nobody is pulling) -> leave it.
    assert (
        decide_recovery(
            managed=True, device_exists=True, usb_cam_running=True,
            receiving=False, streaming=False, silent_secs=30,
        )
        == RECOVERY_NONE
    )
