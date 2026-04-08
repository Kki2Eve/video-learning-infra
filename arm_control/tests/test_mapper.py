"""Tests for the pose mapper layer."""

from __future__ import annotations

import pytest

from arm_control.core.pose_mapper import MappingConfig, PoseMapper
from arm_control.core.pose_types import Pose6D, Quaternion, Vector3


def test_pose_mapper_applies_scale_and_offset() -> None:
    """Mapper should scale the position and add the configured offset."""
    mapper = PoseMapper(
        config=MappingConfig(
            position_scale=2.0,
            tool_offset_xyz=Vector3(x=0.1, y=0.0, z=-0.1),
            target_frame="robot_base",
        )
    )
    pose = Pose6D(
        position=Vector3(x=0.2, y=0.3, z=0.4),
        orientation=Quaternion.identity(),
        frame_id="robot_base",
    )
    mapped = mapper.map_pose(pose)
    assert mapped.frame_id == "robot_base"
    assert mapped.position.x == pytest.approx(0.5)
    assert mapped.position.z == pytest.approx(0.7)

