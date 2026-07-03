#!/usr/bin/env python3
"""Benchmark aiortc's H.264 encoder on THIS hardware and recommend a default
capture resolution/fps (the Pi 5 has no HW encoder, so this is hardware-specific).

Usage (in the driver container or any env with aiortc + this package):
    python3 benchmark_camera.py [--fps-target 30] [--headroom 1.5]
Prints sustained encode fps per resolution and recommends the largest that holds
the target with headroom. Keep the box idle for an accurate result.
"""

from __future__ import annotations

import argparse
import fractions
import time

import numpy as np

try:
    from aiortc.codecs import get_encoder
    from aiortc.rtcrtpparameters import RTCRtpCodecParameters
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"aiortc is required to benchmark the encoder: {exc}")

try:
    from mqtt_bridge.plugins import camera_frame as cf
except Exception:  # allow running standalone next to the module
    import os
    import sys

    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "mqtt_bridge", "plugins"))
    import camera_frame as cf  # type: ignore

RESOLUTIONS = [(640, 480), (800, 600), (1280, 720), (1920, 1080)]
_CODEC = RTCRtpCodecParameters(mimeType="video/H264", clockRate=90000, payloadType=109)


def bench(width: int, height: int, n: int = 150, pool: int = 12) -> float:
    enc = get_encoder(_CODEC)
    frames = [
        cf.encode_yuv420p(
            np.random.randint(0, 255, size=(height, width, 3), dtype=np.uint8), "bgr8", 1.0
        )
        for _ in range(pool)
    ]
    for i in range(3):  # warm up
        f = frames[i % pool]
        f.pts, f.time_base = i, fractions.Fraction(1, 90000)
        enc.encode(f, force_keyframe=(i == 0))
    t0 = time.time()
    for i in range(n):
        f = frames[i % pool]
        f.pts, f.time_base = (i + 10) * 3000, fractions.Fraction(1, 90000)
        enc.encode(f, force_keyframe=False)
    return n / (time.time() - t0)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--fps-target", type=int, default=30)
    ap.add_argument("--headroom", type=float, default=1.5,
                    help="require encode capacity >= fps_target * headroom to call it 'fits'")
    args = ap.parse_args()

    print(f"Target {args.fps_target} fps, headroom x{args.headroom} "
          f"(need >= {args.fps_target * args.headroom:.0f} fps encode capacity)\n")
    print(f"{'resolution':>12} | {'encode fps':>10} | {'fits target?':>12}")
    best = None
    for (w, h) in RESOLUTIONS:
        fps = bench(w, h)
        fits = fps >= args.fps_target * args.headroom
        if fits:
            best = (w, h)
        print(f"{w}x{h:<6} | {fps:10.1f} | {'yes' if fits else 'no':>12}")

    print()
    if best:
        print(f"RECOMMENDED default: MJPG {best[0]}x{best[1]} @ {args.fps_target} fps")
        print("Set in config/params.yaml under `camera:` "
              f"(image_width={best[0]}, image_height={best[1]}, fps={args.fps_target}).")
    else:
        print(f"No resolution holds {args.fps_target} fps with the requested headroom; "
              "lower --fps-target (e.g. 15) or pick the smallest resolution.")


if __name__ == "__main__":
    main()
