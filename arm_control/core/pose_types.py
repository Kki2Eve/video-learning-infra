"""Shared pose and robot state types for teleoperation."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass(slots=True)
class Vector3:
    """Three-dimensional vector expressed in meters."""

    x: float
    y: float
    z: float

    def to_numpy(self) -> NDArray[np.float64]:
        """Return the vector as a NumPy array with shape ``(3,)``."""
        return np.array([self.x, self.y, self.z], dtype=np.float64)

    @classmethod
    def from_iterable(cls, values: tuple[float, float, float] | list[float]) -> "Vector3":
        """Build a vector from three numeric values."""
        x, y, z = values
        return cls(x=float(x), y=float(y), z=float(z))


@dataclass(slots=True)
class Quaternion:
    """Unit quaternion in ``(x, y, z, w)`` ordering."""

    x: float
    y: float
    z: float
    w: float

    def to_numpy(self) -> NDArray[np.float64]:
        """Return the quaternion as a NumPy array with shape ``(4,)``."""
        return np.array([self.x, self.y, self.z, self.w], dtype=np.float64)

    def normalized(self) -> "Quaternion":
        """Return a normalized copy of the quaternion."""
        coeffs = self.to_numpy()
        norm = np.linalg.norm(coeffs)
        if norm == 0.0:
            return Quaternion.identity()
        normalized = coeffs / norm
        return Quaternion(
            x=float(normalized[0]),
            y=float(normalized[1]),
            z=float(normalized[2]),
            w=float(normalized[3]),
        )

    @staticmethod
    def identity() -> "Quaternion":
        """Return the identity rotation quaternion."""
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)


@dataclass(slots=True)
class Pose6D:
    """Rigid pose containing a position and an orientation."""

    position: Vector3
    orientation: Quaternion
    frame_id: str
    timestamp_s: float | None = None

    @staticmethod
    def identity(frame_id: str) -> "Pose6D":
        """Return an identity pose in the given frame."""
        return Pose6D(
            position=Vector3(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion.identity(),
            frame_id=frame_id,
        )


@dataclass(slots=True)
class JointState:
    """Robot joint state used by the IK solver and simulator bridge."""

    names: list[str]
    positions: list[float]
    velocities: list[float] = field(default_factory=list)

