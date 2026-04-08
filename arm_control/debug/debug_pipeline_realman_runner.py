"""Pipeline-driven real-robot runner for single camera poses or trajectories.

Input conventions:
- Single pose:
  {"pose_cam": {"position": [x, y, z], "euler": [r, p, y]}}
- Trajectory:
  {"trajectory_cam": [{"position": [...], "euler": [...], "t": 0.0}, ...]}

All position units are meters and all Euler angles are radians.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from pprint import pformat
from typing import Any


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
from arm_control.core.pose_types import JointState, Pose6D, Vector3  # noqa: E402
from arm_control.core.transform_utils import euler_rpy_to_quaternion  # noqa: E402
from arm_control.paths import PROJECT_ROOT  # noqa: E402

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class TimedCameraPose:
    """One camera-frame pose with an optional relative execution time."""

    pose: Pose6D
    t: float | None = None


def _pose_from_position_euler(position: list[float], euler: list[float], frame_id: str) -> Pose6D:
    """Build a camera-frame pose from API-style position and Euler vectors."""
    return Pose6D(
        position=Vector3(x=float(position[0]), y=float(position[1]), z=float(position[2])),
        orientation=euler_rpy_to_quaternion([float(euler[0]), float(euler[1]), float(euler[2])]),
        frame_id=frame_id,
    )


def _default_single_pose() -> dict[str, Any]:
    """Return a conservative built-in single pose example."""
    return {
        "pose_cam": {
            "position": [-0.60, 0.00, -0.20],
            "euler": [2.80, -0.50, -2.67],
        }
    }


def _default_trajectory() -> dict[str, Any]:
    """Return a small built-in trajectory example."""
    return {
        "trajectory_cam": [
            {"position": [-0.60, 0.00, -0.20], "euler": [2.80, -0.50, -2.67], "t": 0.0},
            {"position": [-0.58, 0.01, -0.18], "euler": [2.78, -0.48, -2.65], "t": 0.8},
            {"position": [-0.56, 0.02, -0.16], "euler": [2.76, -0.46, -2.63], "t": 1.6},
        ]
    }


def _load_input_payload(args: argparse.Namespace) -> dict[str, Any]:
    """Load single-pose or trajectory input JSON from file, inline string, or built-in example."""
    if args.input_file is not None:
        return json.loads(args.input_file.read_text(encoding="utf-8"))
    if args.input_json is not None:
        return json.loads(args.input_json)
    return _default_trajectory() if args.kind == "trajectory" else _default_single_pose()


def _parse_payload(payload: dict[str, Any], frame_id: str) -> tuple[str, list[TimedCameraPose]]:
    """Parse the accepted input payload into a normalized timed-pose sequence."""
    if "pose_cam" in payload:
        pose_raw = payload["pose_cam"]
        return "single", [
            TimedCameraPose(
                pose=_pose_from_position_euler(
                    position=list(pose_raw["position"]),
                    euler=list(pose_raw["euler"]),
                    frame_id=frame_id,
                ),
                t=0.0,
            )
        ]

    if "trajectory_cam" in payload:
        points: list[TimedCameraPose] = []
        for raw_point in payload["trajectory_cam"]:
            points.append(
                TimedCameraPose(
                    pose=_pose_from_position_euler(
                        position=list(raw_point["position"]),
                        euler=list(raw_point["euler"]),
                        frame_id=frame_id,
                    ),
                    t=float(raw_point["t"]) if "t" in raw_point else None,
                )
            )
        return "trajectory", points

    raise ValueError("Input JSON must contain either 'pose_cam' or 'trajectory_cam'.")


def _dry_run_pose(pipeline: TeleopPipeline, pose_cam: Pose6D, seed_state: JointState | None) -> dict[str, Any]:
    """Run transform, IK, and safety without commanding the real robot."""
    pose_base_target = pipeline.transform_pose_cam_to_base(pose_cam)
    ik_result = pipeline.solve_ik(pose_base_target=pose_base_target, seed_state=seed_state)
    pose_safety, joint_safety = pipeline.check_execution_safety(pose_base_target, ik_result.joint_state)
    return {
        "success": bool(ik_result.success and pose_safety.ok and joint_safety.ok),
        "mode": "real-dry-run",
        "pose_cam": pose_cam,
        "pose_base_target": pose_base_target,
        "ik_success": ik_result.success,
        "q_target": ik_result.joint_state,
        "actual_ee_pose": None,
        "position_error": None,
        "orientation_error": None,
        "message": "Dry run only. No command sent to the real robot.",
        "pose_safety_ok": pose_safety.ok,
        "pose_safety_reasons": pose_safety.reasons,
        "joint_safety_ok": joint_safety.ok,
        "joint_safety_reasons": joint_safety.reasons,
    }


def _sleep_for_trajectory_point(
    previous: TimedCameraPose | None,
    current: TimedCameraPose,
    fallback_sleep_s: float,
) -> None:
    """Sleep between trajectory points using point timestamps when available."""
    if previous is None:
        return
    if previous.t is not None and current.t is not None:
        delta = max(0.0, float(current.t) - float(previous.t))
    else:
        delta = max(0.0, fallback_sleep_s)
    if delta > 0.0:
        time.sleep(delta)


def main() -> int:
    """Run a single camera pose or a camera-frame trajectory through the real-robot pipeline."""
    parser = argparse.ArgumentParser(description="Pipeline-based real-robot runner for single poses or trajectories.")
    parser.add_argument(
        "--project-root",
        type=Path,
        default=PROJECT_ROOT,
        help="Path to the arm_control project root.",
    )
    parser.add_argument(
        "--kind",
        choices=("single", "trajectory"),
        default="single",
        help="Fallback example type when no JSON input is provided.",
    )
    parser.add_argument(
        "--input-file",
        type=Path,
        default=None,
        help="Path to a JSON file containing either pose_cam or trajectory_cam.",
    )
    parser.add_argument(
        "--input-json",
        type=str,
        default=None,
        help="Inline JSON string containing either pose_cam or trajectory_cam.",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually execute the commands on the real robot. Without this flag, only dry-run planning is performed.",
    )
    parser.add_argument(
        "--sleep-s",
        type=float,
        default=0.5,
        help="Fallback sleep between trajectory points when no point timestamps are provided.",
    )
    parser.add_argument(
        "--continue-on-fail",
        action="store_true",
        help="Continue executing the remaining trajectory points after a failed step.",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )

    payload = _load_input_payload(args)
    pipeline = TeleopPipeline.from_yaml(args.project_root)
    input_kind, points = _parse_payload(payload, frame_id="camera_color_optical_frame")
    logger.info("Loaded %s input with %d point(s). execute=%s", input_kind, len(points), bool(args.execute))

    previous_point: TimedCameraPose | None = None
    previous_q_target: JointState | None = None
    overall_success = True

    for index, point in enumerate(points):
        logger.info("Processing point %d/%d", index + 1, len(points))
        if args.execute:
            result = pipeline.execute_pose_cam(point.pose, mode="real", seed_state=previous_q_target)
        else:
            result = _dry_run_pose(pipeline, point.pose, seed_state=previous_q_target)

        logger.info("Step result %d:\n%s", index + 1, pformat(result, sort_dicts=False))

        q_target = result.get("q_target")
        if isinstance(q_target, JointState):
            previous_q_target = q_target

        if not bool(result.get("success")):
            overall_success = False
            if not args.continue_on_fail:
                logger.warning("Stopping sequence after failed point %d.", index + 1)
                break

        _sleep_for_trajectory_point(previous_point, point, args.sleep_s)
        previous_point = point

    return 0 if overall_success else 1


if __name__ == "__main__":
    raise SystemExit(main())


