"""Step 4 gate: CPU adaptation controller (ladder, fps_floor 15, hysteresis)."""

from __future__ import annotations

import yaml
import pytest

from mqtt_bridge.plugins.camera_adaptation import (
    AdaptationConfig,
    AdaptationController,
    clamp_ladder,
    validate_ladder,
)

# The real ladder from the robot mapping.
LADDER = [
    (1920, 1080, 30),
    (1920, 1080, 15),
    (1280, 720, 15),
    (800, 600, 15),
    (640, 480, 15),
]


class FakeClock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_controller(cooldown=10.0):
    cfg = AdaptationConfig(
        cpu_high=85, cpu_low=55, fps_floor=15, cooldown_s=cooldown, ema_alpha=1.0, ladder=LADDER
    )
    applied = []
    clock = FakeClock()
    ctrl = AdaptationController(cfg, apply_fn=applied.append, clock=clock)
    return ctrl, applied, clock


def test_validate_ladder_enforces_fps_floor_rule():
    validate_ladder(LADDER, 15)  # ok
    with pytest.raises(ValueError):
        validate_ladder([(1920, 1080, 30), (1280, 720, 10)], 15)  # below floor
    with pytest.raises(ValueError):
        validate_ladder([(640, 480, 15), (1280, 720, 15)], 15)  # dimension grows at floor
    with pytest.raises(ValueError):
        validate_ladder([(1280, 720, 15), (1280, 720, 30)], 15)  # fps increases


def test_high_cpu_drops_fps_first_then_dimension_never_below_floor():
    ctrl, applied, clock = make_controller()
    # Sustained high CPU -> step down every sample until the worst rung.
    for _ in range(len(LADDER) + 3):
        ctrl.update_cpu(99.0)
    # Reached the bottom rung and stayed there.
    assert ctrl.current_rung == (640, 480, 15)
    # First step reduced fps (30->15) at SAME dimension; later steps shrank dims.
    assert applied[0] == (1920, 1080, 15)
    assert applied[1] == (1280, 720, 15)
    # The fps floor (15) is never violated anywhere in the walk.
    assert all(fps >= 15 for (_w, _h, fps) in applied)


def test_low_cpu_recovers_up_with_cooldown_hysteresis():
    ctrl, applied, clock = make_controller(cooldown=10.0)
    for _ in range(len(LADDER)):
        ctrl.update_cpu(99.0)  # drive to bottom
    assert ctrl.index == len(LADDER) - 1
    applied.clear()

    # Low CPU but BEFORE cooldown elapses -> no step up (anti-flap).
    ctrl.update_cpu(10.0)
    assert applied == []

    # After cooldown, low CPU steps up one rung at a time.
    clock.advance(11.0)
    ctrl.update_cpu(10.0)
    assert applied[-1] == (800, 600, 15)
    clock.advance(11.0)
    ctrl.update_cpu(10.0)
    assert applied[-1] == (1280, 720, 15)


def test_midband_cpu_holds_steady():
    ctrl, applied, _ = make_controller()
    for _ in range(5):
        ctrl.update_cpu(70.0)  # between cpu_low(55) and cpu_high(85)
    assert ctrl.index == 0
    assert applied == []


def test_config_loads_from_real_mapping():
    cam = yaml.safe_load(
        open("config/mappings/robot_ugv_beast_v1.yaml")
    )["camera"]
    cfg = AdaptationConfig.from_mapping(cam)
    assert cfg.fps_floor == 15
    validate_ladder(cfg.ladder, cfg.fps_floor)  # real config obeys the rule
    # Mapping default capture is now 1280x720@30 -> ladder clamps its top to that.
    assert cfg.ladder[0] == (1280, 720, 30)
    # No rung exceeds the capture format, and the 15 fps floor is respected.
    assert all(w <= 1280 and h <= 720 and fps >= 15 for (w, h, fps) in cfg.ladder)


def test_clamp_ladder_bounds_and_dedupes():
    clamped = clamp_ladder(LADDER, max_w=1280, max_h=720, max_fps=30, fps_floor=15)
    # Nothing exceeds the capture ceiling...
    assert all(w <= 1280 and h <= 720 and 15 <= fps <= 30 for (w, h, fps) in clamped)
    # ...top rung is the capture format, and consecutive dups are collapsed.
    assert clamped[0] == (1280, 720, 30)
    assert len(clamped) == len(set(clamped))
    validate_ladder(clamped, 15)


def test_clamp_to_15fps_capture_keeps_floor():
    clamped = clamp_ladder(LADDER, max_w=1280, max_h=720, max_fps=15, fps_floor=15)
    assert all(fps == 15 for (_w, _h, fps) in clamped)  # never below floor, capped at 15
    assert clamped[0] == (1280, 720, 15)


def test_params_override_then_clamp_matches_node_merge():
    # Mirror the node merge: params.yaml camera overrides mapping, fps -> capture_fps.
    mapping_cam = yaml.safe_load(
        open("config/mappings/robot_ugv_beast_v1.yaml")
    )["camera"]
    params_cam = yaml.safe_load(open("config/params.yaml"))[
        "/mqtt_bridge_node"
    ]["ros__parameters"]["camera"]

    merged = dict(mapping_cam)
    merged["pixel_format"] = params_cam["pixel_format"]
    merged["image_width"] = params_cam["image_width"]
    merged["image_height"] = params_cam["image_height"]
    merged["capture_fps"] = params_cam["fps"]

    cfg = AdaptationConfig.from_mapping(merged)
    # The params.yaml default (1280x720@30) becomes the adaptation ceiling.
    assert cfg.ladder[0] == (1280, 720, 30)
    assert all(w <= 1280 and h <= 720 and fps >= 15 for (w, h, fps) in cfg.ladder)
    validate_ladder(cfg.ladder, cfg.fps_floor)
