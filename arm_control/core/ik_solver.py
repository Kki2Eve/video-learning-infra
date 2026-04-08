"""Inverse kinematics interfaces and placeholder backends."""

from __future__ import annotations

from dataclasses import dataclass, field
import importlib
import importlib.util
from pathlib import Path
from typing import Any, Protocol

import numpy as np
import yaml

from arm_control.core.pose_types import JointState, Pose6D
from arm_control.core.transform_utils import quaternion_to_euler_rpy


@dataclass(slots=True)
class IKRequest:
    """Input data passed to the IK backend."""

    target_pose: Pose6D
    seed_state: JointState | None = None
    timeout_s: float = 0.02


@dataclass(slots=True)
class IKResult:
    """Result returned by the IK solver layer."""

    success: bool
    joint_state: JointState
    solver_name: str
    diagnostics: dict[str, Any] = field(default_factory=dict)


class IKBackend(Protocol):
    """Protocol for pluggable IK backends."""

    def solve(self, request: IKRequest) -> IKResult:
        """Solve IK for a target pose."""


class StubIKBackend:
    """Reference backend that returns the seed state or a zero vector."""

    def __init__(self, joint_names: list[str]) -> None:
        """Initialize the stub backend with the controlled joint names."""
        self._joint_names = joint_names

    def solve(self, request: IKRequest) -> IKResult:
        """Return a deterministic placeholder joint state for integration testing."""
        if request.seed_state is not None:
            joint_state = request.seed_state
        else:
            joint_state = JointState(names=self._joint_names, positions=[0.0] * len(self._joint_names))
        return IKResult(
            success=True,
            joint_state=joint_state,
            solver_name=self.__class__.__name__,
            diagnostics={"note": "Stub backend does not compute real IK."},
        )


class RealmanIKBackend:
    """IK backend that delegates to the existing AlgoController when available."""

    def __init__(
        self,
        joint_names: list[str],
        arm_model_name: str = "RM_65",
        algo_config: dict[str, Any] | None = None,
        joints_in_degrees: bool = True,
    ) -> None:
        """Initialize the Realman backend with optional algorithm configuration."""
        self._joint_names = joint_names
        self._arm_model_name = arm_model_name
        self._algo_config = algo_config or {}
        self._joints_in_degrees = joints_in_degrees
        self._controller: Any | None = None
        self._availability_error: str | None = None

    def solve(self, request: IKRequest) -> IKResult:
        """Solve IK through the Realman algorithm package, or return a clear fallback failure."""
        controller = self._ensure_controller()
        if controller is None:
            fallback_state = request.seed_state or JointState(
                names=self._joint_names,
                positions=[0.0] * len(self._joint_names),
            )
            return IKResult(
                success=False,
                joint_state=fallback_state,
                solver_name=self.__class__.__name__,
                diagnostics={
                    "error": self._availability_error or "Realman IK backend is unavailable.",
                    "fallback": "No joint command sent.",
                },
            )

        target_pose = self._pose6d_to_realman_pose(request.target_pose)
        initial_joints = request.seed_state.positions if request.seed_state is not None else None
        if initial_joints is not None and self._joints_in_degrees:
            initial_joints = np.rad2deg(np.asarray(initial_joints, dtype=np.float64)).tolist()
        if initial_joints is not None and hasattr(controller, "default_joints"):
            solver_dof = len(getattr(controller, "default_joints"))
            initial_joints = list(initial_joints[:solver_dof])
            if len(initial_joints) < solver_dof:
                initial_joints.extend([0.0] * (solver_dof - len(initial_joints)))

        success, joint_angles = controller.inverse_kinematics(
            target_pose=target_pose,
            initial_joints=initial_joints,
            use_euler=True,
        )
        if not success or joint_angles is None:
            fallback_state = request.seed_state or JointState(
                names=self._joint_names,
                positions=[0.0] * len(self._joint_names),
            )
            return IKResult(
                success=False,
                joint_state=fallback_state,
                solver_name=self.__class__.__name__,
                diagnostics={"target_pose": target_pose, "error": "Realman IK reported failure."},
            )

        solved = np.asarray(joint_angles[: len(self._joint_names)], dtype=np.float64)
        if self._joints_in_degrees:
            solved = np.deg2rad(solved)
        return IKResult(
            success=True,
            joint_state=JointState(names=self._joint_names, positions=solved.tolist()),
            solver_name=self.__class__.__name__,
            diagnostics={"target_pose": target_pose},
        )

    def _ensure_controller(self) -> Any | None:
        """Lazily create the AlgoController only when the external package is available."""
        if self._controller is not None:
            return self._controller
        if self._availability_error is not None:
            return None

        if (
            importlib.util.find_spec("robotic_arm_package.rm_robot_interface") is None
            and importlib.util.find_spec("robotic_arm_package.robotic_arm") is None
        ):
            self._availability_error = (
                "Neither robotic_arm_package.rm_robot_interface nor robotic_arm_package.robotic_arm is installed."
            )
            return None

        try:
            module = importlib.import_module("arm_control.core.algo_controller")
            controller_cls = getattr(module, "AlgoController")
            self._controller = controller_cls(self._arm_model_name, self._algo_config)
            return self._controller
        except SystemExit as exc:
            self._availability_error = f"AlgoController exited during import: {exc}."
        except Exception as exc:  # pragma: no cover - depends on external runtime.
            self._availability_error = str(exc)
        return None

    @staticmethod
    def _pose6d_to_realman_pose(pose: Pose6D) -> list[float]:
        """Convert a project pose into the Realman algorithm pose format."""
        euler_rpy = quaternion_to_euler_rpy(pose.orientation)
        return [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(euler_rpy[0]),
            float(euler_rpy[1]),
            float(euler_rpy[2]),
        ]


class IKSolver:
    """High-level IK wrapper that isolates backend selection from the pipeline."""

    def __init__(self, backend: IKBackend, default_timeout_s: float = 0.02) -> None:
        """Initialize the solver with a pluggable backend."""
        self._backend = backend
        self._default_timeout_s = default_timeout_s

    @classmethod
    def from_yaml(cls, path: str | Path) -> "IKSolver":
        """Construct an IK solver from a robot YAML file."""
        with Path(path).open("r", encoding="utf-8") as stream:
            raw: dict[str, Any] = yaml.safe_load(stream)

        joint_names = [str(name) for name in raw["robot"]["controlled_joints"]]
        backend_name = str(raw["ik"]["backend"]).strip().lower()
        backend: IKBackend
        if backend_name == "stub":
            backend = StubIKBackend(joint_names=joint_names)
        elif backend_name == "realman":
            backend = RealmanIKBackend(
                joint_names=joint_names,
                arm_model_name=str(raw["ik"].get("arm_model", "RM_65")),
                algo_config=dict(raw["ik"].get("algo_config", {})),
                joints_in_degrees=bool(raw["ik"].get("joints_in_degrees", True)),
            )
        else:
            backend = StubIKBackend(joint_names=joint_names)
        return cls(backend=backend, default_timeout_s=float(raw["ik"]["default_timeout_s"]))

    def solve(self, target_pose: Pose6D, seed_state: JointState | None = None) -> IKResult:
        """Solve IK for the provided target pose."""
        request = IKRequest(target_pose=target_pose, seed_state=seed_state, timeout_s=self._default_timeout_s)
        return self._backend.solve(request)

