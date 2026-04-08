"""Tests for the reference pose filter implementation."""

from __future__ import annotations

import numpy as np

from arm_control.core.pose_filter import ExponentialSmoothingConfig, ExponentialSmoothingFilter
from arm_control.core.pose_types import Pose6D, Quaternion, Vector3


def test_exponential_filter_blends_positions() -> None:
    """Filter should move toward the latest sample according to alpha."""
    pose_a = Pose6D(
        position=Vector3(x=0.0, y=0.0, z=0.0),
        orientation=Quaternion.identity(),
        frame_id="base",
    )
    pose_b = Pose6D(
        position=Vector3(x=1.0, y=0.0, z=0.0),
        orientation=Quaternion.identity(),
        frame_id="base",
    )
    pose_filter = ExponentialSmoothingFilter(config=ExponentialSmoothingConfig(alpha=0.5))
    pose_filter.filter(pose_a)
    filtered = pose_filter.filter(pose_b)
    assert np.isclose(filtered.position.x, 0.5)


