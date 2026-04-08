"""Click a point in a live RGB-D stream and move the real robot above it.

Left click picks a point in the aligned color image. The script reads depth from
the aligned depth frame, deprojects the pixel into camera coordinates, converts
the point into the robot base frame using calibration, keeps the current TCP
orientation, adds a configurable Z lift, and then executes the motion through
the shared pipeline/robot-controller stack.
"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from pprint import pformat

import cv2
import numpy as np
import pyrealsense2 as rs


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

from arm_control.core.pipeline import TeleopPipeline  # noqa: E402
from arm_control.core.pose_types import Pose6D, Quaternion, Vector3  # noqa: E402
from arm_control.core.transform_utils import rotation_matrix_to_quaternion  # noqa: E402
from arm_control.paths import PROJECT_ROOT  # noqa: E402

logger = logging.getLogger(__name__)


def _normalize(vector: np.ndarray) -> np.ndarray:
    """Return a normalized copy of a 3D vector."""
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        raise ValueError("Cannot normalize a near-zero vector.")
    return vector / norm


def _look_at_quaternion(
    eye_position: Vector3,
    target_position: Vector3,
    world_up: np.ndarray | None = None,
) -> Quaternion:
    """Return an orientation whose local +Z axis points from eye to target."""
    eye = eye_position.to_numpy()
    target = target_position.to_numpy()
    local_z = _normalize(target - eye)

    up = np.array([0.0, 0.0, 1.0], dtype=np.float64) if world_up is None else _normalize(world_up)
    if abs(float(np.dot(local_z, up))) > 0.99:
        up = np.array([0.0, 1.0, 0.0], dtype=np.float64)

    local_x = _normalize(np.cross(up, local_z))
    local_y = _normalize(np.cross(local_z, local_x))

    rotation = np.column_stack((local_x, local_y, local_z))
    return rotation_matrix_to_quaternion(rotation)


def _build_target_pose_from_click(
    pipeline: TeleopPipeline,
    pixel_x: int,
    pixel_y: int,
    depth_m: float,
    lift_z_m: float,
) -> Pose6D:
    """Convert a clicked pixel into a base-frame target pose using look-at orientation."""
    calibration = pipeline.calibration_manager

    point_cam = calibration.deproject_pixel_to_camera(pixel_x, pixel_y, depth_m)
    point_base = calibration.transform_point_to_robot_base(point_cam)
    eye_position = Vector3(
        x=float(point_base.x),
        y=float(point_base.y),
        z=float(point_base.z + float(lift_z_m)),
    )
    orientation = _look_at_quaternion(
        eye_position=eye_position,
        target_position=point_base,
    )
    return Pose6D(
        position=eye_position,
        orientation=orientation,
        frame_id=calibration.robot_base_frame,
    )


def main() -> int:
    """Run the interactive click-to-move test tool."""
    parser = argparse.ArgumentParser(description="Click a live RGB-D point and move the real robot toward it.")
    parser.add_argument(
        "--project-root",
        type=Path,
        default=PROJECT_ROOT,
        help="Path to the arm_control project root.",
    )
    parser.add_argument("--width", type=int, default=640, help="Camera stream width.")
    parser.add_argument("--height", type=int, default=480, help="Camera stream height.")
    parser.add_argument("--fps", type=int, default=30, help="Camera stream FPS.")
    parser.add_argument(
        "--lift-z",
        type=float,
        default=0.05,
        help="Additional base-frame Z lift in meters so the robot approaches above the clicked point.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually move the robot. Without this flag the script only dry-runs the plan.",
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s | %(levelname)s | %(name)s | %(message)s")

    pipeline = TeleopPipeline.from_yaml(args.project_root)
    controller = pipeline.ensure_real_controller_ready()
    logger.info("Real robot current pose: %s", pipeline._format_pose_for_log(controller.get_current_pose()))

    camera_pipeline = rs.pipeline()
    camera_config = rs.config()
    camera_config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    camera_config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    camera_pipeline.start(camera_config)
    align = rs.align(rs.stream.color)

    window_name = "Click To RealMan"
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)

    state: dict[str, object] = {
        "depth_frame": None,
        "color_image": None,
    }

    def on_click(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        depth_frame = state.get("depth_frame")
        color_image = state.get("color_image")
        if depth_frame is None or color_image is None:
            return

        depth_m = float(depth_frame.get_distance(x, y))
        if depth_m <= 0.0:
            logger.warning("Ignoring click (%s, %s) because no valid depth was available.", x, y)
            return

        b, g, r = color_image[y, x]
        logger.info(
            "Clicked pixel=(%s, %s) depth=%.4f m rgb=(%s, %s, %s)",
            x,
            y,
            depth_m,
            int(r),
            int(g),
            int(b),
        )

        target_pose = _build_target_pose_from_click(
            pipeline=pipeline,
            pixel_x=x,
            pixel_y=y,
            depth_m=depth_m,
            lift_z_m=args.lift_z,
        )
        logger.info("Base-frame target: %s", pipeline._format_pose_for_log(target_pose))

        if args.execute:
            result = pipeline.execute_pose_base(target_pose, mode="real")
        else:
            current_joints = controller.get_current_joints()
            ik_result = pipeline.solve_ik(target_pose, seed_state=current_joints)
            pose_safety, joint_safety = pipeline.check_execution_safety(target_pose, ik_result.joint_state)
            result = {
                "success": bool(ik_result.success and pose_safety.ok and joint_safety.ok),
                "mode": "real-dry-run",
                "pose_cam": None,
                "pose_base_target": target_pose,
                "ik_success": ik_result.success,
                "q_target": ik_result.joint_state,
                "actual_ee_pose": None,
                "position_error": None,
                "orientation_error": None,
                "message": "Dry run only. Re-run with --execute to move the real robot.",
                "pose_safety_ok": pose_safety.ok,
                "pose_safety_reasons": pose_safety.reasons,
                "joint_safety_ok": joint_safety.ok,
                "joint_safety_reasons": joint_safety.reasons,
            }

        logger.info("Click result:\n%s", pformat(result, sort_dicts=False))

    cv2.setMouseCallback(window_name, on_click)

    try:
        while True:
            frames = camera_pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            show_img = np.hstack((color_image, depth_colormap))

            state["depth_frame"] = depth_frame
            state["color_image"] = color_image

            cv2.imshow(window_name, show_img)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
    finally:
        camera_pipeline.stop()
        cv2.destroyAllWindows()
        try:
            controller.disconnect()
        except Exception as exc:  # pragma: no cover - best-effort shutdown
            logger.warning("Disconnect reported an error: %s", exc)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


