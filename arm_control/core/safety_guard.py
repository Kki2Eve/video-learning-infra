"""Safety constraints for workspace and joint-space validation."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from arm_control.core.pose_types import JointState, Pose6D, Vector3


@dataclass(slots=True)
class WorkspaceBounds:
    """Cartesian workspace bounds in the robot base frame."""

    min_position: Vector3
    max_position: Vector3
    frame_id: str


@dataclass(slots=True)
class SafetyReport:
    """Safety evaluation result for a teleoperation step."""

    ok: bool
    reasons: list[str] = field(default_factory=list)
    clamped_pose: Pose6D | None = None


class SafetyGuard:
    """Apply lightweight workspace and joint-limit checks before commanding the arm."""

    def __init__(
        self,
        workspace: WorkspaceBounds,
        joint_limits: dict[str, tuple[float, float]],
        table_height_m: float | None = None,
    ) -> None:
        """Initialize the guard with workspace and joint-space limits."""
        self._workspace = workspace
        self._joint_limits = joint_limits
        self._table_height_m = float(table_height_m) if table_height_m is not None else workspace.min_position.z

    @classmethod
    def from_yaml(cls, workspace_path: str | Path, robot_path: str | Path) -> "SafetyGuard":
        """Load workspace and joint limits from YAML files."""
        with Path(workspace_path).open("r", encoding="utf-8") as stream:
            workspace_raw: dict[str, Any] = yaml.safe_load(stream)
        with Path(robot_path).open("r", encoding="utf-8") as stream:
            robot_raw: dict[str, Any] = yaml.safe_load(stream)

        workspace = WorkspaceBounds(
            min_position=Vector3.from_iterable(workspace_raw["workspace"]["min_position"]),
            max_position=Vector3.from_iterable(workspace_raw["workspace"]["max_position"]),
            frame_id=str(workspace_raw["workspace"]["frame"]),
        )
        joint_limits = {
            str(name): (float(limit[0]), float(limit[1]))
            for name, limit in robot_raw["robot"]["joint_limits"].items()
        }
        table_height_m = float(workspace_raw.get("safety", {}).get("table_height_m", workspace.min_position.z))
        return cls(workspace=workspace, joint_limits=joint_limits, table_height_m=table_height_m)

    def clamp_pose(self, pose: Pose6D) -> Pose6D:
        """Clamp a pose position into the configured workspace bounds."""
        min_xyz = self._workspace.min_position.to_numpy()
        max_xyz = self._workspace.max_position.to_numpy()
        clamped = pose.position.to_numpy().clip(min_xyz, max_xyz)
        clamped[2] = max(clamped[2], self._table_height_m)
        return Pose6D(
            position=Vector3.from_iterable(clamped.tolist()),
            orientation=pose.orientation,
            frame_id=pose.frame_id,
            timestamp_s=pose.timestamp_s,
        )

    def validate_pose(self, pose: Pose6D) -> SafetyReport:
        """Check whether the target pose lies within the allowed Cartesian workspace."""
        clamped_pose = self.clamp_pose(pose)
        original = pose.position.to_numpy()
        clamped = clamped_pose.position.to_numpy()
        is_same_position = np.allclose(clamped, original)
        reasons: list[str] = []
        if original[2] < self._table_height_m:
            reasons.append(f"Target pose was raised to stay above the table height {self._table_height_m:.3f} m.")
        if not is_same_position:
            reasons.append("Target pose was clamped to workspace bounds.")
        return SafetyReport(ok=is_same_position, reasons=reasons, clamped_pose=clamped_pose)

    def validate_joint_state(self, joint_state: JointState) -> SafetyReport:
        """Check whether joint positions lie inside configured joint limits."""
        reasons: list[str] = []
        for name, position in zip(joint_state.names, joint_state.positions, strict=False):
            lower, upper = self._joint_limits.get(name, (-float("inf"), float("inf")))
            if not lower <= position <= upper:
                reasons.append(f"Joint {name} violated limit [{lower}, {upper}].")
        return SafetyReport(ok=not reasons, reasons=reasons, clamped_pose=None)

    @property
    def table_height_m(self) -> float:
        """Return the current minimum allowed table height in the robot base frame."""
        return self._table_height_m

