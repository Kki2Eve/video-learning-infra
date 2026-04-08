"""Helpers for visualizing teleoperation pipeline state in the simulator loop."""

from __future__ import annotations

from arm_control.core.pipeline import PipelineOutput
from arm_control.core.pose_types import Pose6D


class PipelineVisualizer:
    """Reference visualizer that formats pipeline state for logging or debugging."""

    def format_pose(self, pose: Pose6D) -> str:
        """Return a compact human-readable representation of a pose."""
        return (
            f"frame={pose.frame_id} "
            f"pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}) "
            f"quat=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, "
            f"{pose.orientation.z:.3f}, {pose.orientation.w:.3f})"
        )

    def summarize_step(self, output: PipelineOutput) -> str:
        """Summarize one teleoperation pipeline step for console output."""
        return (
            f"filtered_pose=[{self.format_pose(output.filtered_pose)}] "
            f"ik_success={output.ik_result.success} "
            f"post_ik_safe={output.post_ik_safety.ok}"
        )


