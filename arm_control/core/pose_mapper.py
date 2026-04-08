"""Map teleoperation targets from calibrated poses into robot command poses."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from arm_control.core.pose_types import Pose6D, Vector3


@dataclass(slots=True)
class MappingConfig:
    """Configuration for pose scaling and end-effector offsets."""

    position_scale: float
    tool_offset_xyz: Vector3
    target_frame: str


class PoseMapper:
    """Map a robot-base pose into the target pose expected by downstream control."""

    def __init__(self, config: MappingConfig) -> None:
        """Initialize the mapper with static scaling and offset parameters."""
        self._config = config

    @classmethod
    def from_yaml(cls, path: str | Path) -> "PoseMapper":
        """Load mapping parameters from a robot YAML file."""
        with Path(path).open("r", encoding="utf-8") as stream:
            raw: dict[str, Any] = yaml.safe_load(stream)
        mapping = raw["mapping"]
        config = MappingConfig(
            position_scale=float(mapping["position_scale"]),
            tool_offset_xyz=Vector3.from_iterable(mapping["tool_offset_xyz"]),
            target_frame=str(mapping["target_frame"]),
        )
        return cls(config=config)

    def map_pose(self, pose_base: Pose6D) -> Pose6D:
        """Apply lightweight scaling and Cartesian offsets to a base-frame pose."""
        scaled = pose_base.position.to_numpy() * self._config.position_scale
        offset = self._config.tool_offset_xyz.to_numpy()
        return Pose6D(
            position=Vector3.from_iterable((scaled + offset).tolist()),
            orientation=pose_base.orientation,
            frame_id=self._config.target_frame,
            timestamp_s=pose_base.timestamp_s,
        )


