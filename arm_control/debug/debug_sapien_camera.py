"""Launch a minimal SAPIEN scene and validate fixed-camera extrinsics.

Coordinate conventions:
- ``T_ab`` maps points from frame ``b`` into frame ``a``.
- ``T_world_base`` is the robot base pose in the world frame.
- ``T_world_cam`` is the fixed camera pose in the world frame.
- ``T_base_cam = inv(T_world_base) @ T_world_cam`` maps camera-frame points into
  the robot base frame.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np


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

from arm_control.core.pose_types import Pose6D
from arm_control.core.transform_utils import matrix_to_pose, quaternion_to_rotation_matrix
from arm_control.paths import config_path
from arm_control.sim.sapien_controller import SapienArmController
from arm_control.sim.sapien_visualizer import SapienVisualizer


def default_robot_config_path() -> Path:
    """Return the default robot config path used by the debug camera script."""
    return config_path("robot.yaml")


def default_calibration_path() -> Path:
    """Return the default calibration config path used by the debug camera script."""
    return config_path("calibration.yaml")


def format_pose(pose: Pose6D) -> str:
    """Format a pose as a compact single-line string."""
    return (
        f"frame={pose.frame_id} "
        f"pos=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) "
        f"quat_xyzw=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
        f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f})"
    )


def np_array_string(values: object) -> str:
    """Format a NumPy-compatible array with compact fixed precision."""
    return np.array2string(np.asarray(values), precision=4, suppress_small=True)


def print_pose_with_rotation(label: str, pose: Pose6D) -> None:
    """Print a pose, its translation, and its rotation matrix."""
    rotation = quaternion_to_rotation_matrix(pose.orientation)
    print(f"{label}_pose: {format_pose(pose)}")
    print(f"{label}_translation: {np_array_string(pose.position.to_numpy())}")
    print(f"{label}_rotation_matrix:\n{np_array_string(rotation)}")


def poses_are_close(lhs: Pose6D, rhs: Pose6D, atol: float = 1e-6) -> bool:
    """Return whether two poses are numerically almost identical."""
    lhs_rot = quaternion_to_rotation_matrix(lhs.orientation)
    rhs_rot = quaternion_to_rotation_matrix(rhs.orientation)
    return bool(
        np.allclose(lhs.position.to_numpy(), rhs.position.to_numpy(), atol=atol)
        and np.allclose(lhs_rot, rhs_rot, atol=atol)
    )


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the camera extrinsics debug script."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=default_robot_config_path(),
        help="Path to the arm project robot.yaml.",
    )
    parser.add_argument(
        "--calibration-config",
        type=Path,
        default=default_calibration_path(),
        help="Path to the arm project calibration.yaml.",
    )
    parser.add_argument(
        "--scene-name",
        type=str,
        default="single_arm_camera_debug",
        help="Logical name stored in the simulator metadata.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run without opening a viewer window. Useful for smoke testing imports and loading.",
    )
    parser.add_argument(
        "--steps",
        type=int,
        default=240,
        help="Number of simulation steps to execute in headless mode.",
    )
    parser.add_argument(
        "--view-mode",
        type=str,
        choices=["viewer", "camera"],
        default="camera",
        help="Choose whether the interactive window follows the free viewer pose or the fixed scene camera pose.",
    )
    parser.add_argument(
        "--show-labels",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Whether to display 3D labels next to the visualized coordinate frames.",
    )
    return parser.parse_args()


def main() -> None:
    """Create the debug scene, load the robot, print camera extrinsics, and optionally hold the viewer open."""
    args = parse_args()
    controller = SapienArmController.from_yaml(
        path=args.robot_config,
        calibration_path=args.calibration_config,
    )
    controller.connect()
    controller.load_scene(args.scene_name)
    controller.load_robot()

    world_pose = Pose6D.identity(frame_id="world")
    base_pose = controller.get_base_pose_world()
    camera_pose = controller.get_camera_pose_world()
    T_base_cam = controller.get_T_base_cam()
    base_camera_pose = matrix_to_pose(T_base_cam, frame_id="base")

    print_pose_with_rotation("world", world_pose)
    print_pose_with_rotation("base", base_pose)
    print_pose_with_rotation("camera", camera_pose)
    print(f"T_world_base:\n{np_array_string(controller.get_T_world_base())}")
    print(f"T_world_cam:\n{np_array_string(controller.get_T_world_cam())}")
    print("T_base_cam = inv(T_world_base) @ T_world_cam")
    print(f"T_base_cam:\n{np_array_string(T_base_cam)}")
    print_pose_with_rotation("camera_in_base", base_camera_pose)
    if poses_are_close(world_pose, base_pose):
        print("note: world frame and base frame are coincident in this scene, so they visually overlap.")

    if args.headless:
        controller.step_simulation(num_steps=args.steps)
        controller.update_render()
        controller.disconnect()
        return

    visualizer = SapienVisualizer(controller=controller)
    if args.view_mode == "camera":
        visualizer.set_camera_mode()
        print("view_mode: camera")
        print("note: in camera mode you are looking through the fixed camera, so the camera frame itself may be hard to see.")
    else:
        visualizer.set_viewer_mode()
        print("view_mode: viewer")

    visualizer.draw_frame(
        world_pose,
        axis_length=0.20,
        name="world_frame",
        label="WORLD" if args.show_labels else "",
    )
    visualizer.draw_frame(
        base_pose,
        axis_length=0.15,
        name="base_frame",
        label="BASE" if args.show_labels else "",
    )
    visualizer.draw_frame(
        camera_pose,
        axis_length=0.15,
        name="camera_frame",
        label="CAMERA" if args.show_labels else "",
    )
    visualizer.spin()


if __name__ == "__main__":
    main()


