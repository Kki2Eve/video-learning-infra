"""Pose filtering interfaces and lightweight reference implementations."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Protocol

import numpy as np
import yaml

from arm_control.core.pose_types import Pose6D, Quaternion, Vector3


class PoseFilter(Protocol):
    """Protocol for swappable target pose filters."""

    def reset(self) -> None:
        """Clear the filter state."""

    def filter(self, pose: Pose6D, dt_s: float | None = None) -> Pose6D:
        """Filter one incoming pose sample."""


class NoOpPoseFilter:
    """Pass poses through unchanged."""

    def reset(self) -> None:
        """Clear internal state for interface parity."""

    def filter(self, pose: Pose6D, dt_s: float | None = None) -> Pose6D:
        """Return the input pose unchanged."""
        return pose


@dataclass(slots=True)
class ExponentialSmoothingConfig:
    """Configuration for first-order pose smoothing."""

    alpha: float


class ExponentialSmoothingFilter:
    """Simple reference filter for smoothing Cartesian teleoperation targets."""

    def __init__(self, config: ExponentialSmoothingConfig) -> None:
        """Initialize the filter with a fixed smoothing factor."""
        self._config = config
        self._last_pose: Pose6D | None = None

    @classmethod
    def from_yaml(cls, path: str | Path) -> "ExponentialSmoothingFilter":
        """Load filter parameters from a robot YAML file."""
        with Path(path).open("r", encoding="utf-8") as stream:
            raw: dict[str, Any] = yaml.safe_load(stream)
        config = ExponentialSmoothingConfig(alpha=float(raw["filter"]["alpha"]))
        return cls(config=config)

    def reset(self) -> None:
        """Clear the last filtered sample."""
        self._last_pose = None

    def filter(self, pose: Pose6D, dt_s: float | None = None) -> Pose6D:
        """Blend the input pose with the previous pose using linear interpolation."""
        del dt_s
        if self._last_pose is None:
            self._last_pose = pose
            return pose

        alpha = self._config.alpha
        position = alpha * pose.position.to_numpy() + (1.0 - alpha) * self._last_pose.position.to_numpy()
        orientation = alpha * pose.orientation.to_numpy() + (1.0 - alpha) * self._last_pose.orientation.to_numpy()
        filtered = Pose6D(
            position=Vector3.from_iterable(position.tolist()),
            orientation=Quaternion(
                x=float(orientation[0]),
                y=float(orientation[1]),
                z=float(orientation[2]),
                w=float(orientation[3]),
            ).normalized(),
            frame_id=pose.frame_id,
            timestamp_s=pose.timestamp_s,
        )
        self._last_pose = filtered
        return filtered


def build_pose_filter(path: str | Path) -> PoseFilter:
    """Build the configured pose filter from a robot YAML file."""
    with Path(path).open("r", encoding="utf-8") as stream:
        raw: dict[str, Any] = yaml.safe_load(stream)
    filter_type = str(raw["filter"]["type"]).strip().lower()
    if filter_type == "exponential_smoothing":
        return ExponentialSmoothingFilter.from_yaml(path)
    return NoOpPoseFilter()


