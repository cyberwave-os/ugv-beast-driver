"""Guard: the UGV always streams with adaptive fps + dimension, never unbounded.

`from_mapping` must always yield a valid, bounded ladder (even with no `adaptation`
block), and the deployed mapping must keep adaptation enabled and consistent
(floor <= stream_fps <= capture_fps)."""

from __future__ import annotations

from pathlib import Path

import yaml

from mqtt_bridge.plugins.camera_adaptation import (
    AdaptationConfig,
    default_ladder,
    validate_ladder,
)

COMPONENT_ROOT = Path(__file__).resolve().parents[2]
MAPPING = COMPONENT_ROOT / "config" / "mappings" / "robot_ugv_beast_v1.yaml"


def test_default_ladder_is_valid_and_bounded() -> None:
    ladder = default_ladder(1280, 720, 30, fps_floor=15)
    assert ladder, "default ladder must not be empty"
    validate_ladder(ladder, 15)  # drop-fps-then-shrink rule holds
    assert ladder[0] == (1280, 720, 30), "top rung is the capture format"
    assert all(w <= 1280 and h <= 720 and fps >= 15 for (w, h, fps) in ladder)
    # Actually shrinks: the worst rung is smaller than the best.
    tw, th, _ = ladder[0]
    bw, bh, _ = ladder[-1]
    assert bw * bh < tw * th
    # Even dimensions (H.264/yuv420p requirement).
    assert all(w % 2 == 0 and h % 2 == 0 for (w, h, _f) in ladder)


def test_default_ladder_valid_for_odd_and_unknown_capture() -> None:
    # Unknown capture (0s) falls back to sane defaults; odd dims stay even/valid.
    for args in [(0, 0, 0), (1281, 721, 15), (640, 480, 15)]:
        ladder = default_ladder(*args, fps_floor=15)
        assert ladder
        validate_ladder(ladder, 15)
        assert all(w % 2 == 0 and h % 2 == 0 for (w, h, _f) in ladder)


def test_from_mapping_without_adaptation_still_yields_ladder() -> None:
    """The whole point: no `adaptation` block -> still adaptive, not unbounded."""
    cfg = AdaptationConfig.from_mapping(
        {"image_width": 1280, "image_height": 720, "capture_fps": 30}
    )
    assert cfg.ladder, "from_mapping must synthesize a ladder when none is configured"
    validate_ladder(cfg.ladder, cfg.fps_floor)
    assert cfg.ladder[0] == (1280, 720, 30)


def test_from_mapping_empty_config_yields_ladder() -> None:
    cfg = AdaptationConfig.from_mapping({})
    assert cfg.ladder
    validate_ladder(cfg.ladder, cfg.fps_floor)


def test_deployed_mapping_enables_bounded_adaptation() -> None:
    """The shipped mapping must keep the stream adaptive + bounded (regression guard)."""
    cam = yaml.safe_load(MAPPING.read_text())["camera"]
    ad = cam.get("adaptation", {})
    assert ad.get("enabled") is True, "adaptation must stay enabled in the mapping"

    cfg = AdaptationConfig.from_mapping(cam)
    assert cfg.ladder
    validate_ladder(cfg.ladder, cfg.fps_floor)

    capture_fps = int(cam.get("capture_fps", cam.get("fps", 0)))
    stream_fps = int(cam.get("stream_fps", capture_fps))
    assert cfg.fps_floor <= stream_fps <= capture_fps, (
        f"stream_fps={stream_fps} must sit between fps_floor={cfg.fps_floor} and "
        f"capture_fps={capture_fps} — otherwise the base send rate is unbounded"
    )
    # No rung exceeds the capture format; floor is respected everywhere.
    assert all(fps >= cfg.fps_floor for (_w, _h, fps) in cfg.ladder)
