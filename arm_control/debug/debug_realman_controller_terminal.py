"""Terminal-oriented RealMan controller test entrypoint.

This script is meant for manual bring-up on the real robot. Motion-producing
commands require ``--execute`` so that users can inspect the parsed target first.
"""

from __future__ import annotations

import argparse
import logging
import math
import sys
from pathlib import Path


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

from arm_control.core.pose_types import Pose6D, Quaternion, Vector3  # noqa: E402
from arm_control.core.robot_controller import (  # noqa: E402
    RealManController,
    RealManControllerError,
)
from arm_control.core.transform_utils import euler_rpy_to_quaternion  # noqa: E402
from arm_control.paths import config_path  # noqa: E402

logger = logging.getLogger(__name__)


def _build_pose_from_xyzrpy(values: list[float], frame_id: str) -> Pose6D:
    """Build a project pose from ``[x, y, z, rx, ry, rz]`` in meters/radians."""
    if len(values) != 6:
        raise ValueError("Expected 6 values: x y z rx ry rz")
    return Pose6D(
        position=Vector3(x=float(values[0]), y=float(values[1]), z=float(values[2])),
        orientation=euler_rpy_to_quaternion([float(values[3]), float(values[4]), float(values[5])]),
        frame_id=frame_id,
    )


def _print_status(controller: RealManController) -> None:
    """Print current robot state in both SDK-friendly and project-friendly units."""
    pose_xyzrpy = controller.get_current_pose_xyzrpy()
    joints_rad = controller.get_current_joints()
    joints_deg = [math.degrees(value) for value in joints_rad.positions]
    logger.info("Current pose xyzrpy (m/rad): %s", [round(value, 6) for value in pose_xyzrpy])
    logger.info("Current joints (rad): %s", [round(value, 6) for value in joints_rad.positions])
    logger.info("Current joints (deg): %s", [round(value, 6) for value in joints_deg])


def _require_execute(args: argparse.Namespace) -> None:
    """Raise a clear error if a motion command is requested without --execute."""
    if not bool(args.execute):
        raise RealManControllerError(
            "This command would move the real robot. Re-run with --execute after checking the target."
        )


def main() -> int:
    """Run one terminal-driven RealMan controller test command."""
    parser = argparse.ArgumentParser(description="Terminal test entrypoint for RealManController.")
    parser.add_argument(
        "--config",
        type=Path,
        default=config_path("robot.yaml"),
        help="Path to the arm project robot.yaml",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually execute motion commands on the real robot.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="Connect and print the current joint state and pose.")
    subparsers.add_parser("stop", help="Send an immediate stop command.")

    safe_lift = subparsers.add_parser("safe-lift", help="Raise current TCP by a small delta in +Z.")
    safe_lift.add_argument("--delta-z", type=float, default=0.01, help="Lift distance in meters.")
    safe_lift.add_argument("--speed", type=int, default=None, help="Override motion speed percentage.")

    pose_linear = subparsers.add_parser("pose-linear", help="Move linearly to a target xyzrpy pose.")
    pose_linear.add_argument(
        "--pose",
        type=float,
        nargs=6,
        required=True,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="Target pose as x y z rx ry rz in meters/radians.",
    )
    pose_linear.add_argument("--speed", type=int, default=None, help="Override motion speed percentage.")

    pose_joint = subparsers.add_parser("pose-joint", help="Move in joint space to a target xyzrpy pose.")
    pose_joint.add_argument(
        "--pose",
        type=float,
        nargs=6,
        required=True,
        metavar=("X", "Y", "Z", "RX", "RY", "RZ"),
        help="Target pose as x y z rx ry rz in meters/radians.",
    )
    pose_joint.add_argument("--speed", type=int, default=None, help="Override motion speed percentage.")

    joints = subparsers.add_parser("joints", help="Move directly to a target joint vector in degrees.")
    joints.add_argument(
        "--degrees",
        type=float,
        nargs="+",
        required=True,
        help="Target joint angles in degrees, ordered by configured controlled joints.",
    )
    joints.add_argument("--speed", type=int, default=None, help="Override motion speed percentage.")

    subparsers.add_parser("home", help="Move to configured realman.home_joints_deg.")
    subparsers.add_parser("top-view", help="Move to configured realman.top_view_joints_deg.")

    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    controller = RealManController.from_robot_yaml(args.config)
    try:
        controller.connect()
        _print_status(controller)

        if args.command == "status":
            return 0

        if args.command == "stop":
            controller.stop()
            logger.info("Stop command sent.")
            return 0

        if args.command == "safe-lift":
            target = controller.get_current_pose()
            target.position.z += float(args.delta_z)
            logger.info("Safe-lift target pose xyzrpy (m/rad): %s", controller._pose6d_to_rm_pose(target))
            _require_execute(args)
            controller.move_to_pose_linear(target, v=args.speed)
            logger.info("Safe-lift executed.")
            return 0

        if args.command == "pose-linear":
            target = _build_pose_from_xyzrpy(args.pose, frame_id=controller.config.current_pose_frame_id)
            logger.info("Linear target pose xyzrpy (m/rad): %s", controller._pose6d_to_rm_pose(target))
            _require_execute(args)
            controller.move_to_pose_linear(target, v=args.speed)
            logger.info("Linear pose command executed.")
            return 0

        if args.command == "pose-joint":
            target = _build_pose_from_xyzrpy(args.pose, frame_id=controller.config.current_pose_frame_id)
            logger.info("Joint-space pose target xyzrpy (m/rad): %s", controller._pose6d_to_rm_pose(target))
            _require_execute(args)
            controller.move_to_pose_joint(target, v=args.speed)
            logger.info("Joint-space pose command executed.")
            return 0

        if args.command == "joints":
            joint_radians = [math.radians(float(value)) for value in args.degrees]
            logger.info("Target joints (deg): %s", [round(float(value), 6) for value in args.degrees])
            logger.info("Target joints (rad): %s", [round(float(value), 6) for value in joint_radians])
            _require_execute(args)
            controller.move_to_joints(joint_radians, v=args.speed)
            logger.info("Joint command executed.")
            return 0

        if args.command == "home":
            logger.info("Configured home joints (deg): %s", controller.config.home_joints_deg)
            _require_execute(args)
            controller.go_home()
            logger.info("Home motion executed.")
            return 0

        if args.command == "top-view":
            logger.info("Configured top-view joints (deg): %s", controller.config.top_view_joints_deg)
            _require_execute(args)
            controller.move_to_top_view()
            logger.info("Top-view motion executed.")
            return 0

        raise RealManControllerError(f"Unsupported command: {args.command}")
    except RealManControllerError as exc:
        logger.error("RealMan terminal test failed: %s", exc)
        return 1
    finally:
        try:
            controller.disconnect()
        except RealManControllerError as exc:
            logger.warning("Disconnect reported an error: %s", exc)


if __name__ == "__main__":
    raise SystemExit(main())


