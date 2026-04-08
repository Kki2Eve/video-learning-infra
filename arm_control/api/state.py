"""Shared backend state for the arm_control FastAPI app."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from threading import Lock
import time
from typing import Any

import numpy as np
import yaml

from arm_control.core.calibration import CalibrationManager
from arm_control.core.pose_types import JointState, Pose6D, Quaternion, Vector3
from arm_control.core.ik_solver import IKResult, IKSolver
from arm_control.core.safety_guard import SafetyGuard
from arm_control.core.transform_utils import euler_rpy_to_quaternion, quaternion_to_rotation_matrix
from arm_control.paths import PROJECT_ROOT, REPO_ROOT
from arm_control.sim.sapien_controller import SapienArmController


@dataclass(slots=True)
class TimedTargetPose:
    """One base-frame target pose together with its trajectory timestamp."""

    pose: Pose6D
    t: float


@dataclass(slots=True)
class TrajectoryExecutionState:
    """Execution and playback state shared by the backend demo."""

    target_pose_base: Pose6D | None = None
    target_trajectory_base: list[TimedTargetPose] = field(default_factory=list)
    actual_ee_trajectory: list[TimedTargetPose] = field(default_factory=list)
    playback_status: str = "paused"
    current_step_index: int | None = None
    last_executed_step_index: int | None = None
    last_step_started_at_s: float | None = None
    reject_count: int = 0
    fail_count: int = 0
    last_ik_result: IKResult | None = None
    last_reject_reasons: list[str] = field(default_factory=list)
    last_error: str | None = None


@dataclass(slots=True)
class BackendState:
    """Shared backend resources reused across FastAPI requests."""

    repo_root: Path
    project_root: Path
    robot_config_path: Path
    calibration_path: Path
    workspace_path: Path
    calibration_manager: CalibrationManager
    ik_solver: IKSolver
    safety_guard: SafetyGuard
    workspace_min_position: np.ndarray
    workspace_max_position: np.ndarray
    controlled_joint_names: list[str]
    initial_qpos: list[float]
    hand_to_tool_euler_rpy: tuple[float, float, float]
    sim_scene_name: str = "api_backend_scene"
    _sim_controller: SapienArmController | None = None
    _sim_error: str | None = None
    _execution: TrajectoryExecutionState = field(default_factory=TrajectoryExecutionState)
    _target_lock: Lock = field(default_factory=Lock, repr=False)

    @classmethod
    def create(cls, project_root: Path | None = None) -> "BackendState":
        """Create backend state from the project configuration files."""
        if project_root is None:
            project_root = PROJECT_ROOT
        repo_root = REPO_ROOT
        robot_config_path = project_root / "config" / "robot.yaml"
        calibration_path = project_root / "config" / "calibration.yaml"
        workspace_path = project_root / "config" / "workspace.yaml"

        with robot_config_path.open("r", encoding="utf-8") as stream:
            robot_raw: dict[str, Any] = yaml.safe_load(stream)
        with workspace_path.open("r", encoding="utf-8") as stream:
            workspace_raw: dict[str, Any] = yaml.safe_load(stream)

        return cls(
            repo_root=repo_root,
            project_root=project_root,
            robot_config_path=robot_config_path,
            calibration_path=calibration_path,
            workspace_path=workspace_path,
            calibration_manager=CalibrationManager.from_yaml(calibration_path),
            ik_solver=IKSolver.from_yaml(robot_config_path),
            safety_guard=SafetyGuard.from_yaml(workspace_path=workspace_path, robot_path=robot_config_path),
            workspace_min_position=np.array(workspace_raw["workspace"]["min_position"], dtype=np.float64),
            workspace_max_position=np.array(workspace_raw["workspace"]["max_position"], dtype=np.float64),
            controlled_joint_names=[str(name) for name in robot_raw["robot"]["controlled_joints"]],
            initial_qpos=[float(value) for value in robot_raw.get("sim", {}).get("initial_qpos", [])],
            hand_to_tool_euler_rpy=tuple(
                float(value) for value in robot_raw.get("mapping", {}).get("hand_to_tool_euler_rpy", [0.0, 0.0, 0.0])
            ),
        )

    def ensure_sim_controller(self) -> SapienArmController | None:
        """Create and initialize the SAPIEN controller on first use."""
        if self._sim_controller is not None:
            return self._sim_controller
        if self._sim_error is not None:
            return None

        try:
            controller = SapienArmController.from_yaml(
                path=self.robot_config_path,
                calibration_path=self.calibration_path,
            )
            controller.connect()
            controller.load_scene(self.sim_scene_name)
            controller.load_robot()
        except Exception as exc:  # pragma: no cover - depends on local SAPIEN runtime.
            self._sim_error = str(exc)
            return None

        self._sim_controller = controller
        return controller

    @property
    def sim_error(self) -> str | None:
        """Return the last simulation initialization error, if any."""
        return self._sim_error

    def hand_to_tool_rotation_offset(self) -> np.ndarray:
        """Return the current hand-to-tool rotation offset matrix."""
        quaternion = euler_rpy_to_quaternion(self.hand_to_tool_euler_rpy)
        return quaternion_to_rotation_matrix(quaternion)

    def get_robot_urdf_url(self) -> str | None:
        """Return a browser-loadable URL for the configured robot URDF when possible."""
        try:
            relative_path = self.ik_solver_configured_urdf_relative_path()
        except ValueError:
            return None
        return f"/{relative_path.as_posix()}"

    def ik_solver_configured_urdf_relative_path(self) -> Path:
        """Return the URDF path relative to the repo root for web serving."""
        with self.robot_config_path.open("r", encoding="utf-8") as stream:
            raw: dict[str, Any] = yaml.safe_load(stream)
        urdf_path = Path(str(raw["robot"]["urdf_path"]))
        if urdf_path.is_absolute():
            try:
                return urdf_path.relative_to(self.repo_root)
            except ValueError as exc:
                raise ValueError("Configured URDF path is outside the repository root.") from exc
        return urdf_path

    def set_target_pose_base(self, pose: Pose6D | None) -> Pose6D | None:
        """Store one base-frame target pose and clear any active target trajectory."""
        with self._target_lock:
            self._execution = TrajectoryExecutionState(
                target_pose_base=self._clone_pose(pose),
                target_trajectory_base=(
                    [TimedTargetPose(pose=self._clone_pose(pose), t=0.0)]
                    if pose is not None
                    else []
                ),
                playback_status="paused",
                current_step_index=0 if pose is not None else None,
            )
            self._reset_robot_to_initial_qpos()
            return self._clone_pose(self._execution.target_pose_base)

    def set_target_trajectory_base(self, trajectory: list[TimedTargetPose]) -> tuple[Pose6D | None, int, int | None]:
        """Store a base-frame target trajectory and activate time-based indexing."""
        with self._target_lock:
            copied_trajectory = [
                TimedTargetPose(pose=self._clone_pose(point.pose), t=float(point.t))
                for point in trajectory
            ]
            target_pose = self._clone_pose(copied_trajectory[0].pose) if copied_trajectory else None
            self._execution = TrajectoryExecutionState(
                target_pose_base=target_pose,
                target_trajectory_base=copied_trajectory,
                playback_status="paused",
                current_step_index=0 if copied_trajectory else None,
            )
            self._reset_robot_to_initial_qpos()
            return (
                self._clone_pose(target_pose),
                len(copied_trajectory),
                0 if copied_trajectory else None,
            )

    def play_trajectory_execution(self) -> tuple[str, int | None, int]:
        """Enter playing mode for the currently loaded target trajectory."""
        with self._target_lock:
            if not self._execution.target_trajectory_base:
                return self._execution.playback_status, self._execution.current_step_index, 0
            if self._execution.current_step_index is None:
                self._execution.current_step_index = 0
            self._execution.playback_status = "playing"
            self._execution.last_step_started_at_s = (
                self._execution.last_step_started_at_s or time.monotonic()
            )
            return self._execution.playback_status, self._execution.current_step_index, len(self._execution.target_trajectory_base)

    def pause_trajectory_execution(self) -> tuple[str, int | None, int]:
        """Pause trajectory playback without clearing the current target."""
        with self._target_lock:
            self._execution.playback_status = "paused"
            return self._execution.playback_status, self._execution.current_step_index, len(self._execution.target_trajectory_base)

    def reset_trajectory_execution(self) -> tuple[str, int | None, int]:
        """Reset execution to the first trajectory sample and clear counters/history."""
        with self._target_lock:
            if self._execution.target_trajectory_base:
                self._execution.target_pose_base = self._clone_pose(self._execution.target_trajectory_base[0].pose)
                self._execution.current_step_index = 0
            else:
                self._execution.target_pose_base = None
                self._execution.current_step_index = None
            self._execution.last_executed_step_index = None
            self._execution.last_step_started_at_s = None
            self._execution.actual_ee_trajectory = []
            self._execution.reject_count = 0
            self._execution.fail_count = 0
            self._execution.last_reject_reasons = []
            self._execution.last_error = None
            self._execution.playback_status = "paused"
            self._execution.last_ik_result = None
            self._reset_robot_to_initial_qpos()
            return self._execution.playback_status, self._execution.current_step_index, len(self._execution.target_trajectory_base)

    def step_trajectory_execution(self) -> tuple[str, int | None, int]:
        """Advance one trajectory step in paused mode."""
        with self._target_lock:
            self._execution.playback_status = "paused"
            if not self._execution.target_trajectory_base:
                return self._execution.playback_status, self._execution.current_step_index, 0
            if self._execution.current_step_index is None:
                self._execution.current_step_index = 0
            elif self._execution.last_executed_step_index == self._execution.current_step_index:
                self._execution.current_step_index = min(
                    self._execution.current_step_index + 1,
                    len(self._execution.target_trajectory_base) - 1,
                )
            self._execution.last_executed_step_index = None
            self._execution.target_pose_base = self._clone_pose(
                self._execution.target_trajectory_base[self._execution.current_step_index].pose
            )
            return self._execution.playback_status, self._execution.current_step_index, len(self._execution.target_trajectory_base)

    def execute_trajectory_if_needed(self) -> None:
        """Advance playback timing, run absolute IK, and command the SAPIEN robot."""
        controller = self.ensure_sim_controller()
        if controller is None:
            return

        with self._target_lock:
            if not self._execution.target_trajectory_base:
                return
            self._advance_playback_clock_locked()
            current_index = self._execution.current_step_index
            if current_index is None:
                return
            self._execution.target_pose_base = self._clone_pose(self._execution.target_trajectory_base[current_index].pose)
            if self._execution.last_executed_step_index == current_index:
                return
            target_pose = self._clone_pose(self._execution.target_pose_base)

        if target_pose is None:
            return

        safe_report = self.safety_guard.validate_pose(target_pose)
        safe_pose = safe_report.clamped_pose or target_pose
        seed_state = controller.get_joint_state()
        ik_result = self.ik_solver.solve(target_pose=safe_pose, seed_state=seed_state)
        joint_report = self.safety_guard.validate_joint_state(ik_result.joint_state)

        controller.step_simulation(1)
        if safe_report.reasons:
            with self._target_lock:
                self._execution.reject_count += 1
                self._execution.last_reject_reasons = list(safe_report.reasons)
        if not ik_result.success:
            with self._target_lock:
                self._execution.fail_count += 1
                self._execution.last_ik_result = ik_result
                self._execution.last_error = ik_result.diagnostics.get("error") if isinstance(ik_result.diagnostics, dict) else None
                self._execution.last_executed_step_index = self._execution.current_step_index
            controller.update_render()
            self._append_actual_ee_pose_locked(controller.get_end_effector_pose(), current_index)
            return

        if not joint_report.ok:
            with self._target_lock:
                self._execution.reject_count += 1
                self._execution.last_reject_reasons = list(joint_report.reasons)
                self._execution.last_ik_result = ik_result
                self._execution.last_executed_step_index = self._execution.current_step_index
            controller.update_render()
            self._append_actual_ee_pose_locked(controller.get_end_effector_pose(), current_index)
            return

        controller.send_joint_command(ik_result.joint_state)
        controller.step_simulation(4)
        controller.update_render()
        with self._target_lock:
            self._execution.last_ik_result = ik_result
            self._execution.last_reject_reasons = []
            self._execution.last_error = None
            self._execution.last_executed_step_index = self._execution.current_step_index
        self._append_actual_ee_pose_locked(controller.get_end_effector_pose(), current_index)

    def get_execution_snapshot(self) -> dict[str, Any]:
        """Return a detached snapshot of the current trajectory execution state."""
        with self._target_lock:
            return {
                "target_pose_base": self._clone_pose(self._execution.target_pose_base),
                "target_trajectory_base": [
                    TimedTargetPose(pose=self._clone_pose(point.pose), t=point.t)
                    for point in self._execution.target_trajectory_base
                ],
                "actual_ee_trajectory": [
                    TimedTargetPose(pose=self._clone_pose(point.pose), t=point.t)
                    for point in self._execution.actual_ee_trajectory
                ],
                "playback_status": self._execution.playback_status,
                "current_step_index": self._execution.current_step_index,
                "trajectory_length": len(self._execution.target_trajectory_base),
                "reject_count": self._execution.reject_count,
                "fail_count": self._execution.fail_count,
                "last_ik_result": self._execution.last_ik_result,
                "last_reject_reasons": list(self._execution.last_reject_reasons),
                "last_error": self._execution.last_error,
            }

    def _advance_playback_clock_locked(self) -> None:
        """Advance the current target index according to trajectory timestamps while playing."""
        if self._execution.playback_status != "playing":
            return
        if self._execution.current_step_index is None:
            self._execution.current_step_index = 0
        if self._execution.current_step_index >= len(self._execution.target_trajectory_base) - 1:
            self._execution.playback_status = "paused"
            return

        now = time.monotonic()
        if self._execution.last_step_started_at_s is None:
            self._execution.last_step_started_at_s = now
            return

        current_index = self._execution.current_step_index
        current_t = self._execution.target_trajectory_base[current_index].t
        next_t = self._execution.target_trajectory_base[current_index + 1].t
        required_dt = max(next_t - current_t, 0.05)
        if self._execution.last_executed_step_index == current_index and now - self._execution.last_step_started_at_s >= required_dt:
            self._execution.current_step_index = current_index + 1
            self._execution.last_step_started_at_s = now
            self._execution.target_pose_base = self._clone_pose(
                self._execution.target_trajectory_base[self._execution.current_step_index].pose
            )

    def _append_actual_ee_pose_locked(self, pose: Pose6D, current_index: int | None) -> None:
        """Record the latest actual end-effector pose into the execution history."""
        with self._target_lock:
            timestamp = (
                self._execution.target_trajectory_base[current_index].t
                if current_index is not None and 0 <= current_index < len(self._execution.target_trajectory_base)
                else float(len(self._execution.actual_ee_trajectory))
            )
            self._execution.actual_ee_trajectory.append(
                TimedTargetPose(pose=self._clone_pose(pose), t=float(timestamp))
            )
            self._execution.actual_ee_trajectory = self._execution.actual_ee_trajectory[-500:]

    def _reset_robot_to_initial_qpos(self) -> None:
        """Reset the simulator robot to the configured initial joint state when available."""
        controller = self.ensure_sim_controller()
        if controller is None or not self.initial_qpos:
            return
        initial_state = JointState(names=self.controlled_joint_names, positions=list(self.initial_qpos))
        controller.send_joint_command(initial_state)
        controller.step_simulation(2)
        controller.update_render()

    @staticmethod
    def _clone_pose(pose: Pose6D | None) -> Pose6D | None:
        """Create a detached copy of a project pose for safe shared-state reuse."""
        if pose is None:
            return None
        return Pose6D(
            position=Vector3(x=pose.position.x, y=pose.position.y, z=pose.position.z),
            orientation=Quaternion(
                x=pose.orientation.x,
                y=pose.orientation.y,
                z=pose.orientation.z,
                w=pose.orientation.w,
            ),
            frame_id=pose.frame_id,
            timestamp_s=pose.timestamp_s,
        )

