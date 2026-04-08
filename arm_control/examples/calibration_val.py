"""Quick validation script for pixel+depth to robot-base conversion."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


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

from arm_control.core.calibration import CalibrationManager  # noqa: E402
from arm_control.paths import config_path  # noqa: E402


def main() -> int:
    """Validate one pixel/depth sample against the configured calibration."""
    parser = argparse.ArgumentParser(description="Validate camera intrinsics/extrinsics from calibration.yaml.")
    parser.add_argument(
        "--config",
        type=Path,
        default=config_path("calibration.yaml"),
        help="Path to calibration.yaml",
    )
    parser.add_argument("--u", type=float, default=307.0, help="Pixel x coordinate.")
    parser.add_argument("--v", type=float, default=276.0, help="Pixel y coordinate.")
    parser.add_argument("--depth", type=float, default=0.79, help="Depth in meters.")
    args = parser.parse_args()

    calibration = CalibrationManager.from_yaml(args.config)
    intrinsics = calibration.get_camera_intrinsics()

    print("外参矩阵 camera_to_robot_base:")
    print(calibration.get_camera_to_robot_base())
    print()
    print("相机内参:")
    print(intrinsics.to_dict())

    point_cam = calibration.deproject_pixel_to_camera(args.u, args.v, args.depth)
    point_base = calibration.transform_point_to_robot_base(point_cam)

    print()
    print(f"像素点: ({args.u:.2f}, {args.v:.2f}) depth={args.depth:.4f} m")
    print("相机坐标系下的点:", [point_cam.x, point_cam.y, point_cam.z])
    print("机械臂 base 坐标系下的点:", [point_base.x, point_base.y, point_base.z])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


