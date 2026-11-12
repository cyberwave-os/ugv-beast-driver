"""CPU-driven adaptation: walk a quality ladder by smoothed CPU load.

Rule: drop fps to fps_floor FIRST, then only shrink dimension — never below
fps_floor. Pure logic (CPU sample injected) so it's host-testable; the node owns
the psutil sampling loop and applies each rung via a callback.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

Rung = Tuple[int, int, int]  # (width, height, fps)


@dataclass
class AdaptationConfig:
    cpu_high: float = 85.0
    cpu_low: float = 55.0
    fps_floor: int = 15
    check_period_s: float = 2.0
    cooldown_s: float = 10.0
    ema_alpha: float = 0.5
    ladder: Optional[List[Rung]] = None

    @classmethod
    def from_mapping(cls, camera_cfg: dict) -> "AdaptationConfig":
        camera_cfg = camera_cfg or {}
        ad = camera_cfg.get("adaptation", {}) or {}
        fps_floor = int(ad.get("fps_floor", 15))
        cap_w = int(camera_cfg.get("image_width", 0) or 0)
        cap_h = int(camera_cfg.get("image_height", 0) or 0)
        cap_fps = int(camera_cfg.get("capture_fps", camera_cfg.get("fps", 0)) or 0)
        ladder = [tuple(r) for r in ad.get("ladder", [])] or None
        # Clamp the ladder to the active capture format (can't send larger/faster
        # than captured), so the same ladder adapts to overrides / set_camera_format.
        if ladder:
            ladder = clamp_ladder(ladder, cap_w, cap_h, cap_fps, fps_floor)
        else:
            # No ladder configured — synthesize one so adaptation still runs.
            ladder = default_ladder(cap_w, cap_h, cap_fps, fps_floor)
        return cls(
            cpu_high=float(ad.get("cpu_high", 85.0)),
            cpu_low=float(ad.get("cpu_low", 55.0)),
            fps_floor=fps_floor,
            check_period_s=float(ad.get("check_period_s", 2.0)),
            cooldown_s=float(ad.get("cooldown_s", 10.0)),
            ladder=ladder,
        )


def default_ladder(
    cap_w: int, cap_h: int, cap_fps: int, fps_floor: int = 15
) -> List[Rung]:
    """Synthesize a best->worst ladder from the capture format (drop fps to the floor,
    then shrink dimension) when the mapping provides none. 0 falls back to defaults."""
    cap_w = int(cap_w or 1280)
    cap_h = int(cap_h or 720)
    cap_fps = int(cap_fps or 30)
    fps_floor = int(fps_floor or 15)
    hi_fps = max(fps_floor, cap_fps)

    def even(v: float) -> int:
        i = int(v)
        return max(2, i - (i % 2))  # H.264/yuv420p need even dimensions

    fw, fh = even(cap_w), even(cap_h)
    rungs = [
        (fw, fh, hi_fps),                                        # full quality
        (fw, fh, fps_floor),                                     # fps -> floor
        (even(cap_w * 3 // 4), even(cap_h * 3 // 4), fps_floor),  # 0.75x dimension
        (even(cap_w // 2), even(cap_h // 2), fps_floor),          # 0.5x dimension
    ]
    # clamp_ladder collapses duplicates (e.g. when cap_fps == fps_floor).
    return clamp_ladder(rungs, cap_w, cap_h, cap_fps, fps_floor)


def clamp_ladder(
    ladder: List[Rung], max_w: int, max_h: int, max_fps: int, fps_floor: int
) -> List[Rung]:
    """Clamp rungs to the capture ceiling (0 = no limit): fps to [fps_floor, max_fps],
    w/h to <= max_w/max_h; collapse resulting duplicates."""
    out: List[Rung] = []
    for (w, h, fps) in ladder:
        if max_fps:
            fps = min(fps, max_fps)
        fps = max(fps, fps_floor)
        if max_w:
            w = min(w, max_w)
        if max_h:
            h = min(h, max_h)
        rung = (w, h, fps)
        if not out or out[-1] != rung:
            out.append(rung)
    return out


def validate_ladder(ladder: List[Rung], fps_floor: int) -> None:
    """Enforce the rule: no rung below fps_floor, fps non-increasing, and at
    constant fps the area non-increasing. Raises ValueError otherwise."""
    if not ladder:
        raise ValueError("adaptation ladder is empty")
    for (w, h, fps) in ladder:
        if fps < fps_floor:
            raise ValueError(f"ladder rung {(w, h, fps)} is below fps_floor {fps_floor}")
    # Monotonic non-increasing 'cost': fps must not increase as we descend, and
    # once fps has reached the floor, area must be strictly non-increasing.
    for i in range(1, len(ladder)):
        pw, ph, pfps = ladder[i - 1]
        w, h, fps = ladder[i]
        if fps > pfps:
            raise ValueError(f"ladder fps increases at rung {i}: {ladder}")
        if pfps == fps and (w * h) > (pw * ph):
            raise ValueError(f"ladder dimension grows at constant fps at rung {i}: {ladder}")


class AdaptationController:
    """Stateful ladder walker driven by injected CPU samples + a monotonic clock."""

    def __init__(
        self,
        config: AdaptationConfig,
        apply_fn: Callable[[Rung], None],
        clock: Callable[[], float],
    ) -> None:
        if config.ladder:
            validate_ladder(config.ladder, config.fps_floor)
        self.cfg = config
        self._apply = apply_fn
        self._clock = clock
        self.index = 0  # 0 == best quality
        self._cpu_ema: Optional[float] = None
        self._last_step_at: float = clock()

    @property
    def ladder(self) -> List[Rung]:
        return self.cfg.ladder or []

    @property
    def current_rung(self) -> Rung:
        return self.ladder[self.index]

    def update_cpu(self, cpu_percent: float) -> Optional[Rung]:
        """Feed one CPU sample; maybe step the ladder. Returns the new rung if changed.

        ema > cpu_high -> step DOWN immediately; ema < cpu_low -> step UP, but only
        after cooldown_s since the last step (anti-flap).
        """
        a = self.cfg.ema_alpha
        self._cpu_ema = cpu_percent if self._cpu_ema is None else a * cpu_percent + (1 - a) * self._cpu_ema
        now = self._clock()

        changed: Optional[Rung] = None
        if self._cpu_ema > self.cfg.cpu_high and self.index < len(self.ladder) - 1:
            self.index += 1
            self._last_step_at = now
            changed = self.current_rung
        elif self._cpu_ema < self.cfg.cpu_low and self.index > 0:
            if (now - self._last_step_at) >= self.cfg.cooldown_s:
                self.index -= 1
                self._last_step_at = now
                changed = self.current_rung

        if changed is not None:
            self._apply(changed)
        return changed

    @property
    def cpu_ema(self) -> Optional[float]:
        return self._cpu_ema
