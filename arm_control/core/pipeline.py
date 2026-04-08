"""End-to-end teleoperation pipeline orchestration."""

from __future__ import annotations

from dataclasses import dataclass
import logging
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from arm_control.core.calibration import CalibrationManager
from arm_control.core.ik_solver import IKResult, IKSolver
from arm_control.core.pose_filter import PoseFilter, build_pose_filter
from arm_control.core.pose_mapper import PoseMapper
from arm_control.core.pose_types import JointState, Pose6D
from arm_control.core.robot_controller import RealManController
from arm_control.core.safety_guard import SafetyGuard, SafetyReport
from arm_control.core.transform_utils import (
    compose_transform,
    euler_rpy_to_quaternion,
    invert_transform,
    matrix_to_pose,
    pose_to_matrix,
    quaternion_to_rotation_matrix,
)
from arm_control.sim.sapien_controller import SapienArmController
from arm_control.sim.sim_controller import SimController

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class PipelineInput:
    """One teleoperation sample entering the backend pipeline."""

    pose_cam: Pose6D
    seed_state: JointState | None = None


@dataclass(slots=True)
class PipelineOutput:
    """Structured output of one pipeline step."""

    pose_base: Pose6D
    mapped_pose: Pose6D
    filtered_pose: Pose6D
    pre_ik_safety: SafetyReport
    ik_result: IKResult
    post_ik_safety: SafetyReport


class TeleopPipeline:
    """Compose calibration, mapping, filtering, safety checks, and IK."""

    def __init__(
        self,
        calibration: CalibrationManager,
        mapper: PoseMapper,
        pose_filter: PoseFilter,
        safety_guard: SafetyGuard,
        ik_solver: IKSolver,
        sim_controller: SimController | None = None,
        real_controller: RealManController | None = None,
        orientation_offset_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        """Initialize the teleoperation pipeline with swappable dependencies."""
        self._calibration = calibration
        self._mapper = mapper
        self._pose_filter = pose_filter
        self._safety_guard = safety_guard
        self._ik_solver = ik_solver
        self._sim_controller = sim_controller
        self._real_controller = real_controller
        self._orientation_offset_rpy = tuple(float(value) for value in orientation_offset_rpy)

    @classmethod
    def from_yaml(cls, project_root: str | Path) -> "TeleopPipeline":
        """Build the default pipeline and execution backends from project config."""
        root = Path(project_root).resolve()
        robot_path = root / "config" / "robot.yaml"
        calibration_path = root / "config" / "calibration.yaml"
        workspace_path = root / "config" / "workspace.yaml"

        with robot_path.open("r", encoding="utf-8") as stream:
            robot_raw: dict[str, Any] = yaml.safe_load(stream)

        return cls(
            calibration=CalibrationManager.from_yaml(calibration_path),
            mapper=PoseMapper.from_yaml(robot_path),
            pose_filter=build_pose_filter(robot_path),
            safety_guard=SafetyGuard.from_yaml(workspace_path=workspace_path, robot_path=robot_path),
            ik_solver=IKSolver.from_yaml(robot_path),
            sim_controller=SapienArmController.from_yaml(path=robot_path, calibration_path=calibration_path),
            real_controller=RealManController.from_robot_yaml(robot_path),
            orientation_offset_rpy=tuple(
                float(value)
                for value in robot_raw.get("mapping", {}).get("hand_to_tool_euler_rpy", [0.0, 0.0, 0.0])
            ),
        )

    def step(self, command: PipelineInput) -> PipelineOutput:
        """Run one teleoperation step from camera-frame pose to IK result."""
        pose_base = self._calibration.transform_pose_to_robot_base(command.pose_cam)
        mapped_pose = self._mapper.map_pose(pose_base)
        filtered_pose = self._pose_filter.filter(mapped_pose)
        pre_ik_safety = self._safety_guard.validate_pose(filtered_pose)
        safe_pose = pre_ik_safety.clamped_pose or filtered_pose
        ik_result = self._ik_solver.solve(target_pose=safe_pose, seed_state=command.seed_state)
        post_ik_safety = self._safety_guard.validate_joint_state(ik_result.joint_state)
        return PipelineOutput(
            pose_base=pose_base,
            mapped_pose=mapped_pose,
            filtered_pose=safe_pose,
            pre_ik_safety=pre_ik_safety,
            ik_result=ik_result,
            post_ik_safety=post_ik_safety,
        )

    def transform_pose_cam_to_base(self, pose_cam: Pose6D) -> Pose6D:
        """Transform a camera-frame pose into the execution target pose in base frame."""
        pose_base = self._calibration.transform_pose_to_robot_base(pose_cam)
        mapped_pose = self._mapper.map_pose(pose_base)
        pose_base_target = self._apply_orientation_offset(mapped_pose)
        logger.info("Input pose_cam: %s", self._format_pose_for_log(pose_cam))
        logger.info("Transformed pose_base_target: %s", self._format_pose_for_log(pose_base_target))
        return pose_base_target

    def solve_ik(self, pose_base_target: Pose6D, seed_state: JointState | None = None) -> IKResult:
        """Solve IK for one base-frame target pose."""
        result = self._ik_solver.solve(target_pose=pose_base_target, seed_state=seed_state)
        logger.info(
            "IK result: success=%s solver=%s q_target=%s diagnostics=%s",
            result.success,
            result.solver_name,
            [round(position, 6) for position in result.joint_state.positions],
            result.diagnostics,
        )
        return result

    def check_execution_safety(self, pose_base_target: Pose6D, q_target: JointState) -> tuple[SafetyReport, SafetyReport]:
        """Run Cartesian and joint-space safety checks for the target pose and joint solution."""
        pose_report = self._safety_guard.validate_pose(pose_base_target)
        joint_report = self._safety_guard.validate_joint_state(q_target)
        logger.info(
            "Safety reports: pose_ok=%s pose_reasons=%s joint_ok=%s joint_reasons=%s",
            pose_report.ok,
            pose_report.reasons,
            joint_report.ok,
            joint_report.reasons,
        )
        return pose_report, joint_report

    def execute_target_pose(self, pose_base_target: Pose6D, q_target: JointState, mode: str) -> tuple[Pose6D | None, str]:
        """Execute the IK solution either in simulation or on the real arm."""
        del pose_base_target
        normalized_mode = mode.strip().lower()
        logger.info("Execution mode: %s", normalized_mode)

        if normalized_mode == "sim":
            controller = self._ensure_sim_controller_ready()
            controller.send_joint_command(q_target)
            if hasattr(controller, "step_simulation"):
                controller.step_simulation(1)
            if hasattr(controller, "update_render"):
                controller.update_render()
            actual_pose = self._get_sim_actual_ee_pose_base(controller)
            logger.info("Actual sim ee pose: %s", self._format_pose_for_log(actual_pose))
            return actual_pose, "Executed target pose in SAPIEN simulation."

        if normalized_mode == "real":
            controller = self._ensure_real_controller_ready()
            controller.move_to_joints(q_target.positions)
            actual_pose = controller.get_current_pose()
            logger.info("Actual real ee pose: %s", self._format_pose_for_log(actual_pose))
            return actual_pose, "Executed target pose on the RealMan robot."

        raise ValueError(f"Unsupported execution mode: {mode}")

    def execute_pose_cam(
        self,
        pose_cam: Pose6D,
        mode: str = "sim",
        seed_state: JointState | None = None,
    ) -> dict[str, Any]:
        """Execute a single camera-frame target pose through the unified pipeline."""
        mode_normalized = mode.strip().lower()
        result: dict[str, Any] = {
            "success": False,
            "mode": mode_normalized,
            "pose_cam": pose_cam,
            "pose_base_target": None,
            "ik_success": False,
            "q_target": None,
            "actual_ee_pose": None,
            "position_error": None,
            "orientation_error": None,
            "message": "",
        }

        pose_base_target = self.transform_pose_cam_to_base(pose_cam)
        result = self.execute_pose_base(
            pose_base_target=pose_base_target,
            mode=mode_normalized,
            seed_state=seed_state,
        )
        result["pose_cam"] = pose_cam
        result["pose_base_target"] = pose_base_target
        return result

    def execute_pose_base(
        self,
        pose_base_target: Pose6D,
        mode: str = "real",
        seed_state: JointState | None = None,
    ) -> dict[str, Any]:
        """Execute a single robot-base-frame target pose through IK and the selected backend."""
        mode_normalized = mode.strip().lower()
        result: dict[str, Any] = {
            "success": False,
            "mode": mode_normalized,
            "pose_cam": None,
            "pose_base_target": pose_base_target,
            "ik_success": False,
            "q_target": None,
            "actual_ee_pose": None,
            "position_error": None,
            "orientation_error": None,
            "message": "",
        }

        try:
            ik_result = self.solve_ik(pose_base_target=pose_base_target, seed_state=seed_state)
            result["ik_success"] = ik_result.success
            result["q_target"] = ik_result.joint_state
            if not ik_result.success:
                message = f"IK failed: {ik_result.diagnostics}"
                logger.warning(message)
                result["message"] = message
                return result

            pose_safety, joint_safety = self.check_execution_safety(pose_base_target, ik_result.joint_state)
            if not pose_safety.ok:
                message = f"Pose rejected by safety guard: {'; '.join(pose_safety.reasons) or 'unknown pose safety failure'}"
                logger.warning(message)
                result["message"] = message
                return result
            if not joint_safety.ok:
                message = f"Joint target rejected by safety guard: {'; '.join(joint_safety.reasons) or 'unknown joint safety failure'}"
                logger.warning(message)
                result["message"] = message
                return result

            actual_pose, execution_message = self.execute_target_pose(
                pose_base_target=pose_base_target,
                q_target=ik_result.joint_state,
                mode=mode_normalized,
            )
            result["actual_ee_pose"] = actual_pose
            result["position_error"] = self._compute_position_error(
                pose_base_target=pose_base_target,
                actual_ee_pose=actual_pose,
            )
            result["orientation_error"] = self._compute_orientation_error(
                pose_base_target=pose_base_target,
                actual_ee_pose=actual_pose,
            )
            result["success"] = actual_pose is not None
            result["message"] = execution_message
            logger.info(
                "Execution result: success=%s position_error=%s orientation_error=%s",
                result["success"],
                result["position_error"],
                result["orientation_error"],
            )
            return result
        except Exception as exc:
            message = f"Pipeline execution failed: {exc}"
            logger.exception(message)
            result["message"] = message
            return result

    def _apply_orientation_offset(self, pose: Pose6D) -> Pose6D:
        """Apply the configured orientation offset from mapping config."""
        if np.allclose(np.asarray(self._orientation_offset_rpy, dtype=np.float64), np.zeros(3, dtype=np.float64)):
            return pose

        pose_matrix = pose_to_matrix(pose)
        rotation_offset = quaternion_to_rotation_matrix(euler_rpy_to_quaternion(self._orientation_offset_rpy))
        offset_matrix = np.eye(4, dtype=np.float64)
        offset_matrix[:3, :3] = rotation_offset
        transformed = compose_transform(pose_matrix, offset_matrix)
        offset_pose = matrix_to_pose(transformed, frame_id=pose.frame_id)
        offset_pose.timestamp_s = pose.timestamp_s
        return offset_pose

    def _ensure_sim_controller_ready(self) -> SapienArmController:
        """Connect and initialize the SAPIEN controller when needed."""
        if self._sim_controller is None:
            raise RuntimeError("Simulation mode requested but no sim controller is configured.")
        controller = self._sim_controller
        controller.connect()
        if hasattr(controller, "load_scene"):
            controller.load_scene("pipeline_single_pose")
        if hasattr(controller, "load_robot"):
            try:
                controller.robot
            except Exception:
                controller.load_robot()
        return controller  # type: ignore[return-value]

    def _ensure_real_controller_ready(self) -> RealManController:
        """Connect the RealMan controller when needed."""
        if self._real_controller is None:
            raise RuntimeError("Real mode requested but no RealMan controller is configured.")
        if not self._real_controller.is_connected:
            self._real_controller.connect()
        return self._real_controller

    def ensure_real_controller_ready(self) -> RealManController:
        """Return a connected RealMan controller instance."""
        return self._ensure_real_controller_ready()

    @property
    def calibration_manager(self) -> CalibrationManager:
        """Expose the loaded calibration manager."""
        return self._calibration

    def _get_sim_actual_ee_pose_base(self, controller: SapienArmController) -> Pose6D:
        """Return the current SAPIEN end-effector pose expressed in the robot base frame."""
        actual_ee_world = controller.get_end_effector_pose()
        T_base_world = invert_transform(controller.get_T_world_base())
        actual_ee_base = matrix_to_pose(
            T_base_world @ pose_to_matrix(actual_ee_world),
            frame_id=self._calibration.robot_base_frame,
        )
        actual_ee_base.timestamp_s = actual_ee_world.timestamp_s
        return actual_ee_base

    @staticmethod
    def _compute_position_error(pose_base_target: Pose6D, actual_ee_pose: Pose6D | None) -> float | None:
        """Return Euclidean position error in meters when the frames match."""
        if actual_ee_pose is None or actual_ee_pose.frame_id != pose_base_target.frame_id:
            return None
        delta = pose_base_target.position.to_numpy() - actual_ee_pose.position.to_numpy()
        return float(np.linalg.norm(delta))

    @staticmethod
    def _compute_orientation_error(pose_base_target: Pose6D, actual_ee_pose: Pose6D | None) -> float | None:
        """Return angular orientation error in radians when the frames match."""
        if actual_ee_pose is None or actual_ee_pose.frame_id != pose_base_target.frame_id:
            return None
        target_rotation = quaternion_to_rotation_matrix(pose_base_target.orientation)
        actual_rotation = quaternion_to_rotation_matrix(actual_ee_pose.orientation)
        delta = target_rotation.T @ actual_rotation
        trace_value = np.clip((np.trace(delta) - 1.0) * 0.5, -1.0, 1.0)
        return float(np.arccos(trace_value))

    @staticmethod
    def _format_pose_for_log(pose: Pose6D) -> str:
        """Return a concise string for per-step pipeline logging."""
        quat = pose.orientation.normalized()
        return (
            f"frame={pose.frame_id} "
            f"position=[{pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}] "
            f"quaternion=[{quat.x:.4f}, {quat.y:.4f}, {quat.z:.4f}, {quat.w:.4f}]"
        )

