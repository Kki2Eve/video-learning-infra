"""Tests for transform and pose conversion utilities."""

from __future__ import annotations

import numpy as np

from arm_control.core.pose_types import Pose6D, Quaternion, Vector3
from arm_control.core.transform_utils import matrix_to_pose, pose_to_matrix, transform_pose


def test_pose_matrix_roundtrip_preserves_translation() -> None:
    """Pose-to-matrix roundtrip should preserve the position component."""
    pose = Pose6D(
        position=Vector3(x=0.1, y=-0.2, z=0.3),
        orientation=Quaternion.identity(),
        frame_id="camera",
    )
    matrix = pose_to_matrix(pose)
    restored = matrix_to_pose(matrix, frame_id="camera")
    assert np.allclose(restored.position.to_numpy(), pose.position.to_numpy())


def test_transform_pose_updates_frame() -> None:
    """Applying a homogeneous transform should update the frame label."""
    pose = Pose6D.identity(frame_id="camera")
    transform = np.eye(4)
    transform[0, 3] = 0.5
    transformed = transform_pose(pose=pose, transform=transform, target_frame="base")
    assert transformed.frame_id == "base"
    assert np.isclose(transformed.position.x, 0.5)


