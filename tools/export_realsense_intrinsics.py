#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

import pyrealsense2 as rs


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Export RealSense color intrinsics to an OpenCV YAML file."
    )
    parser.add_argument("--output", required=True, help="Output OpenCV YAML path")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    profile = None
    try:
        profile = pipeline.start(config)
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_profile.get_intrinsics()

        camera_matrix = np.array(
            [[intr.fx, 0.0, intr.ppx], [0.0, intr.fy, intr.ppy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        dist_coeffs = np.array(intr.coeffs[:5], dtype=np.float64).reshape(-1, 1)

        fs = cv2.FileStorage(str(output_path), cv2.FileStorage_WRITE)
        if not fs.isOpened():
            raise RuntimeError(f"Failed to open output file for write: {output_path}")
        fs.write("camera_matrix", camera_matrix)
        fs.write("dist_coeffs", dist_coeffs)
        fs.write("image_width", int(intr.width))
        fs.write("image_height", int(intr.height))
        fs.write("fps", int(args.fps))
        fs.write("model", str(intr.model))
        fs.release()

        print(output_path)
        print(
            f"fx={intr.fx:.6f} fy={intr.fy:.6f} cx={intr.ppx:.6f} cy={intr.ppy:.6f} "
            f"size={intr.width}x{intr.height}"
        )
        return 0
    finally:
        if profile is not None:
            pipeline.stop()


if __name__ == "__main__":
    raise SystemExit(main())
