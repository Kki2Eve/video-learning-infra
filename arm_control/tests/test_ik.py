"""Tests for the pluggable IK solver skeleton."""

from __future__ import annotations

from arm_control.core.ik_solver import IKSolver, StubIKBackend
from arm_control.core.pose_types import JointState, Pose6D


def test_stub_ik_solver_returns_seed_state() -> None:
    """Stub solver should return the supplied seed state unchanged."""
    solver = IKSolver(backend=StubIKBackend(joint_names=["j1", "j2"]))
    seed = JointState(names=["j1", "j2"], positions=[0.1, -0.2])
    result = solver.solve(target_pose=Pose6D.identity(frame_id="base"), seed_state=seed)
    assert result.success is True
    assert result.joint_state.positions == [0.1, -0.2]


