"""Launch a minimal SAPIEN scene for a single robot arm and print state once."""

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
from arm_control.core.transform_utils import quaternion_to_rotation_matrix
from arm_control.paths import config_path
from arm_control.sim.sapien_controller import SapienArmController
from arm_control.sim.sapien_visualizer import SapienVisualizer


def default_robot_config_path() -> Path:
    """Return the default robot config path used by the SAPIEN debug scene."""
    return config_path("robot.yaml")


def format_pose(pose: Pose6D) -> str:
    """Format a pose for single-line debug logging."""
    return (
        f"frame={pose.frame_id} "
        f"pos=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) "
        f"quat_xyzw=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, "
        f"{pose.orientation.z:.4f}, {pose.orientation.w:.4f})"
    )


def pose_translation_string(pose: Pose6D) -> str:
    """Format the pose translation vector for terminal output."""
    translation = pose.position.to_numpy()
    return np_array_string(translation)


def pose_rotation_matrix_string(pose: Pose6D) -> str:
    """Format the pose rotation matrix for terminal output.

    The matrix columns are the child frame's local ``x/y/z`` axes expressed in the
    parent frame. In this script, parent is the SAPIEN world frame.
    """
    rotation = quaternion_to_rotation_matrix(pose.orientation)
    return np_array_string(rotation)


def np_array_string(values: object) -> str:
    """Format a NumPy-compatible array with compact fixed precision."""

    return np.array2string(np.asarray(values), precision=4, suppress_small=True)


def print_frame_pose(label: str, pose: Pose6D) -> None:
    """Print translation and rotation information for a named frame pose."""
    print(f"{label}_pose: {format_pose(pose)}")
    print(f"{label}_translation: {pose_translation_string(pose)}")
    print(f"{label}_rotation_matrix:\n{pose_rotation_matrix_string(pose)}")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments for the debug SAPIEN script."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=default_robot_config_path(),
        help="Path to the arm project robot.yaml.",
    )
    parser.add_argument(
        "--scene-name",
        type=str,
        default="single_arm_debug",
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
        "--show-labels",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Whether to display 3D labels next to the visualized coordinate frames.",
    )
    return parser.parse_args()


def main() -> None:
    """Create the debug scene, load the robot, print state, and optionally hold the viewer open."""
    args = parse_args()
    controller = SapienArmController.from_yaml(args.robot_config)
    controller.connect()
    controller.load_scene(args.scene_name)
    controller.load_robot()

    world_pose = Pose6D.identity(frame_id="world")
    state = controller.get_robot_state()
    print_frame_pose("world", world_pose)
    print_frame_pose("base", state.base_pose)
    print_frame_pose("ee", state.ee_pose)
    print(f"joint_names: {state.joint_state.names}")
    print(f"qpos:        {state.joint_state.positions}")

    if args.headless:
        controller.step_simulation(num_steps=args.steps)
        controller.update_render()
        controller.disconnect()
        return

    visualizer = SapienVisualizer(controller=controller)
    visualizer.draw_frame(
        world_pose,
        axis_length=0.20,
        name="world_frame",
        label="WORLD" if args.show_labels else "",
    )
    visualizer.draw_frame(
        state.base_pose,
        axis_length=0.15,
        name="base_frame",
        label="BASE" if args.show_labels else "",
    )
    visualizer.draw_frame(
        state.ee_pose,
        axis_length=0.12,
        name="ee_frame",
        label="EE" if args.show_labels else "",
    )
    visualizer.spin()


if __name__ == "__main__":
    main()


