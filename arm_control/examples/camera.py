"""Live RealSense preview for reading intrinsics and clicking RGB-D points."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs
import yaml


def _bootstrap_arm_imports() -> None:
    """Make the arm package importable from either current or future layout."""
    current = Path(__file__).resolve()
    for candidate in (current.parent, *current.parents):
        if (candidate / "README.md").exists() and (candidate / "repo_paths.py").exists():
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)
            from repo_paths import bootstrap_arm_imports

            bootstrap_arm_imports(current)
            return
    raise RuntimeError(f"Unable to locate repository root from: {current}")


if __package__ in (None, ""):
    _bootstrap_arm_imports()

from arm_control.paths import config_path  # noqa: E402


def _intrinsics_from_profile(profile: rs.pipeline_profile) -> dict[str, Any]:
    """Extract color intrinsics from the active RealSense profile."""
    color_stream = profile.get_stream(rs.stream.color)
    color_intr = color_stream.as_video_stream_profile().get_intrinsics()
    return {
        "width": int(color_intr.width),
        "height": int(color_intr.height),
        "fx": float(color_intr.fx),
        "fy": float(color_intr.fy),
        "cx": float(color_intr.ppx),
        "cy": float(color_intr.ppy),
        "distortion_model": str(color_intr.model),
        "coeffs": [float(value) for value in color_intr.coeffs],
    }


def _print_intrinsics(intrinsics: dict[str, Any]) -> None:
    """Pretty-print the camera intrinsics."""
    print("=" * 50)
    print("【相机内参】")
    print(f"宽: {intrinsics['width']}  高: {intrinsics['height']}")
    print(f"fx: {intrinsics['fx']:.2f}")
    print(f"fy: {intrinsics['fy']:.2f}")
    print(f"cx: {intrinsics['cx']:.2f}")
    print(f"cy: {intrinsics['cy']:.2f}")
    print("畸变模型:", intrinsics["distortion_model"])
    print("畸变系数:", intrinsics["coeffs"])
    print("=" * 50)


def _write_intrinsics_to_config(config_path: Path, intrinsics: dict[str, Any]) -> None:
    """Persist the captured intrinsics into calibration.yaml."""
    with config_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}
    raw["camera_intrinsics"] = intrinsics
    with config_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(raw, stream, sort_keys=False, allow_unicode=True)
    print(f"已写入内参到: {config_path}")


def main() -> int:
    """Run the live RGB-D preview window."""
    parser = argparse.ArgumentParser(description="Preview RealSense RGB-D frames and optionally save intrinsics.")
    parser.add_argument(
        "--config",
        type=Path,
        default=config_path("calibration.yaml"),
        help="Path to calibration.yaml",
    )
    parser.add_argument("--width", type=int, default=640, help="Stream width.")
    parser.add_argument("--height", type=int, default=480, help="Stream height.")
    parser.add_argument("--fps", type=int, default=30, help="Stream FPS.")
    parser.add_argument("--write-config", action="store_true", help="Write the current intrinsics into calibration.yaml.")
    args = parser.parse_args()

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    profile = pipeline.start(config)

    intrinsics = _intrinsics_from_profile(profile)
    _print_intrinsics(intrinsics)
    if args.write_config:
        _write_intrinsics_to_config(args.config, intrinsics)

    align = rs.align(rs.stream.color)

    def mouse_click(event: int, x: int, y: int, _flags: int, param: tuple[rs.depth_frame, np.ndarray]) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        depth_frame, color_image = param
        depth = depth_frame.get_distance(x, y)
        depth_mm = int(depth * 1000)
        b, g, r = color_image[y, x]
        print(f"点击 ({x:3d},{y:3d}) | 深度 {depth_mm:4d} mm | RGB({r:3d},{g:3d},{b:3d})")

    cv2.namedWindow("Aligned RGB-D", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            show_img = np.hstack((color_image, depth_colormap))

            cv2.setMouseCallback("Aligned RGB-D", mouse_click, (depth_frame, color_image))
            cv2.imshow("Aligned RGB-D", show_img)

            if cv2.waitKey(1) & 0xFF == 27:
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


