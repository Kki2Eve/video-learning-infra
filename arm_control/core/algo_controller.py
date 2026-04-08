"""Compatibility wrapper around the canonical Realman API package.

This module preserves the original ``AlgoController`` interface while sourcing
all backend implementations from the single surviving ``robotic_arm_package``.
It supports the algorithm API exposed either through:

1. ``robotic_arm_package.rm_robot_interface``
2. ``robotic_arm_package.robotic_arm``
"""

from __future__ import annotations

import logging
from typing import Any

logger = logging.getLogger(__name__)

_USING_LEGACY_BINDINGS = False

try:
    from robotic_arm_package.rm_robot_interface import (  # type: ignore[import-not-found]
        Algo,
        rm_api_version,
        rm_force_type_e,
        rm_frame_t,
        rm_inverse_kinematics_params_t,
        rm_robot_arm_model_e,
    )
except ImportError:
    from robotic_arm_package.robotic_arm import (  # type: ignore[import-not-found]
        Algo,
        Arm,
        Euler,
        FRAME,
        FRAME_NAME,
        Pos,
        Pose,
        Quat,
        RobotType,
        SensorType,
    )
else:
    _USING_LEGACY_BINDINGS = True


if _USING_LEGACY_BINDINGS:
    ARM_MODELS_CONFIG = {
        "RM_65": [rm_robot_arm_model_e.RM_MODEL_RM_65_E, rm_force_type_e.RM_MODEL_RM_B_E, [0.0] * 6],
        "RM_75": [rm_robot_arm_model_e.RM_MODEL_RM_75_E, rm_force_type_e.RM_MODEL_RM_B_E, [0.0] * 7],
        "RML_63": [rm_robot_arm_model_e.RM_MODEL_RM_63_II_E, rm_force_type_e.RM_MODEL_RM_B_E, [0.0] * 6],
        "ECO_65": [rm_robot_arm_model_e.RM_MODEL_ECO_65_E, rm_force_type_e.RM_MODEL_RM_B_E, [0.0] * 6],
        "GEN_72": [rm_robot_arm_model_e.RM_MODEL_GEN_72_E, rm_force_type_e.RM_MODEL_RM_B_E, [0.0] * 7],
        "ECO_63": [rm_robot_arm_model_e.RM_MODEL_ECO_63_E, rm_force_type_e.RM_MODEL_RM_B_E, [0.0] * 6],
    }
else:
    # The vendored library uses a 7-element joint buffer for inverse kinematics
    # even on 6-axis arms, so we keep one trailing redundant slot.
    ARM_MODELS_CONFIG = {
        "RM_65": [RobotType.RM65, SensorType.B, [0.0] * 7],
        "RM_75": [RobotType.RM75, SensorType.B, [0.0] * 7],
        "RML_63": [RobotType.RML63II, SensorType.B, [0.0] * 7],
        "ECO_65": [RobotType.ECO65, SensorType.B, [0.0] * 7],
        "GEN_72": [RobotType.GEN72, SensorType.B, [0.0] * 7],
        "ECO_63": [RobotType.ECO63, SensorType.B, [0.0] * 7],
    }


class AlgoController:
    """Algorithm controller used by the teleop backend absolute IK path."""

    def __init__(self, arm_model_name: str = "RM_65", config: dict[str, Any] | None = None) -> None:
        """Initialize the controller and the selected algorithm backend."""
        self.config = config or {}
        if arm_model_name not in ARM_MODELS_CONFIG:
            logger.warning("Unsupported arm model %s, falling back to RM_65.", arm_model_name)
            arm_model_name = "RM_65"

        model_config = ARM_MODELS_CONFIG[arm_model_name]
        self.arm_model = model_config[0]
        self.force_type = model_config[1]
        self.default_joints = list(model_config[2])

        if _USING_LEGACY_BINDINGS:
            self.robot = Algo(self.arm_model, self.force_type)
            if self.robot.handle.id == -1:
                raise RuntimeError("Unable to initialize legacy Realman algorithm engine.")
        else:
            Algo.pDll = Arm.pDll
            Algo.Algo_Init_Sys_Data(self.arm_model, self.force_type)
            try:
                Algo.Algo_Set_Redundant_Parameter_Traversal_Mode(False)
            except Exception:
                logger.debug("Traversal mode setup is unavailable in the vendored backend.", exc_info=True)
            self.robot = Algo

        self._initialize_frames()
        self.last_joint_angles = self.default_joints.copy()

    def _initialize_frames(self) -> None:
        """Initialize install angle, workframe, and toolframe from config."""
        install_angle = self.config.get("install_angle", [0.0, 0.0, 0.0])
        self.set_angle(*install_angle)

        workframe = self.config.get("workframe", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.set_workframe(workframe)

        toolframe = self.config.get("toolframe", {})
        pose = toolframe.get("pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        payload = toolframe.get("payload", 0.0)
        center_of_mass = toolframe.get("center_of_mass", [0.0, 0.0, 0.0])
        self.set_toolframe(pose, payload, *center_of_mass)

    def set_angle(self, x: float, y: float, z: float) -> None:
        """Set the robot install angle in degrees."""
        if _USING_LEGACY_BINDINGS:
            self.robot.rm_algo_set_angle(x, y, z)
        else:
            self.robot.Algo_Set_Angle(float(x), float(y), float(z))

    def set_workframe(self, pose: list[float] | tuple[float, ...]) -> None:
        """Set the workframe pose ``[x, y, z, rx, ry, rz]``."""
        if _USING_LEGACY_BINDINGS:
            frame = rm_frame_t(pose=pose)
            self.robot.rm_algo_set_workframe(frame)
            return

        frame = FRAME()
        frame.frame_name = FRAME_NAME(b"work")
        frame.pose = Pose(position=Pos(*pose[:3]), euler=Euler(*pose[3:]))
        frame.payload = 0.0
        frame.x = 0.0
        frame.y = 0.0
        frame.z = 0.0
        self.robot.Algo_Set_WorkFrame(frame)

    def set_toolframe(
        self,
        pose: list[float] | tuple[float, ...],
        payload: float,
        x: float,
        y: float,
        z: float,
    ) -> None:
        """Set the toolframe pose and payload parameters."""
        if _USING_LEGACY_BINDINGS:
            frame = rm_frame_t(None, pose, payload, x, y, z)
            self.robot.rm_algo_set_toolframe(frame)
            return

        frame = FRAME()
        frame.frame_name = FRAME_NAME(b"tool")
        frame.pose = Pose(position=Pos(*pose[:3]), euler=Euler(*pose[3:]))
        frame.payload = float(payload)
        frame.x = float(x)
        frame.y = float(y)
        frame.z = float(z)
        self.robot.Algo_Set_ToolFrame(frame)

    def forward_kinematics(self, joint_angles: list[float] | None = None, use_euler: bool = True) -> list[float]:
        """Run forward kinematics and return a 6D pose."""
        if joint_angles is None:
            joint_angles = self.last_joint_angles
        if _USING_LEGACY_BINDINGS:
            flag = 1 if use_euler else 0
            return self.robot.rm_algo_forward_kinematics(joint_angles, flag)
        return self.robot.Algo_Forward_Kinematics(self._normalize_joint_buffer(joint_angles))

    def inverse_kinematics(
        self,
        target_pose: list[float],
        initial_joints: list[float] | None = None,
        use_euler: bool = True,
    ) -> tuple[bool, list[float] | None]:
        """Run inverse kinematics and return ``(success, joint_angles_deg)``."""
        if initial_joints is None:
            initial_joints = self.last_joint_angles
        initial_joints = self._normalize_joint_buffer(initial_joints)

        if _USING_LEGACY_BINDINGS:
            flag = 1 if use_euler else 0
            params = rm_inverse_kinematics_params_t(initial_joints, target_pose, flag)
            result = self.robot.rm_algo_inverse_kinematics(params)
            success = result[0] == 0
            if success:
                self.last_joint_angles = list(result[1])
                return True, list(result[1])
            logger.warning("Legacy IK failed with error code: %s", result[0])
            return False, None

        tag, joint_angles = self.robot.Algo_Inverse_Kinematics(
            initial_joints,
            target_pose,
            1 if use_euler else 0,
        )
        success = tag == 0
        if success:
            self.last_joint_angles = list(joint_angles)
            return True, list(joint_angles)
        logger.warning("Vendored IK failed with error code: %s", tag)
        return False, None

    def euler_to_quaternion(self, euler_angles: list[float]) -> list[float]:
        """Convert Euler angles to quaternion ``[w, x, y, z]``."""
        if _USING_LEGACY_BINDINGS:
            return self.robot.rm_algo_euler2quaternion(euler_angles)
        quat = self.robot.Algo_Euler2Quaternion(Euler(*euler_angles))
        return [quat.w, quat.x, quat.y, quat.z]

    def quaternion_to_euler(self, quaternion: list[float]) -> list[float]:
        """Convert quaternion ``[w, x, y, z]`` to Euler angles."""
        if _USING_LEGACY_BINDINGS:
            return self.robot.rm_algo_quaternion2euler(quaternion)
        euler = self.robot.Algo_Quaternion2Euler(Quat(*quaternion))
        return [euler.rx, euler.ry, euler.rz]

    def get_api_version(self) -> str:
        """Return a short backend identifier."""
        if _USING_LEGACY_BINDINGS:
            return str(rm_api_version())
        return "robotic_arm_package.robotic_arm"

    def _normalize_joint_buffer(self, joint_angles: list[float]) -> list[float]:
        """Pad or trim joint vectors to match the backend solver buffer size."""
        solver_dof = len(self.default_joints)
        normalized = list(float(value) for value in joint_angles[:solver_dof])
        if len(normalized) < solver_dof:
            normalized.extend([0.0] * (solver_dof - len(normalized)))
        return normalized
