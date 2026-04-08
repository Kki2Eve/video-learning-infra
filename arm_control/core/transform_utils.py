"""Utilities for rigid-body transforms and pose conversions."""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from arm_control.core.pose_types import Pose6D, Quaternion, Vector3


def quaternion_to_rotation_matrix(quaternion: Quaternion) -> NDArray[np.float64]:
    """Convert a quaternion into a 3x3 rotation matrix."""
    x, y, z, w = quaternion.normalized().to_numpy()
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def rotation_matrix_to_quaternion(rotation: NDArray[np.float64]) -> Quaternion:
    """Convert a 3x3 rotation matrix into a quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = np.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return Quaternion(x=float(x), y=float(y), z=float(z), w=float(w)).normalized()


def euler_rpy_to_quaternion(euler_rpy: tuple[float, float, float] | list[float]) -> Quaternion:
    """Convert Euler angles in roll-pitch-yaw order into a quaternion.

    The Euler convention is intrinsic ``XYZ`` / roll-pitch-yaw in radians,
    provided as ``[roll, pitch, yaw]``.
    """
    roll, pitch, yaw = [float(value) for value in euler_rpy]
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    ).normalized()


def quaternion_to_euler_rpy(quaternion: Quaternion) -> NDArray[np.float64]:
    """Convert a quaternion into Euler angles in roll-pitch-yaw order."""
    x, y, z, w = quaternion.normalized().to_numpy()

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = np.sign(sinp) * (np.pi / 2.0)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float64)


def pose_to_matrix(pose: Pose6D) -> NDArray[np.float64]:
    """Convert a pose into a homogeneous 4x4 transform matrix."""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = quaternion_to_rotation_matrix(pose.orientation)
    transform[:3, 3] = pose.position.to_numpy()
    return transform


def matrix_to_pose(matrix: NDArray[np.float64], frame_id: str) -> Pose6D:
    """Convert a homogeneous 4x4 transform matrix into a pose."""
    return Pose6D(
        position=Vector3.from_iterable(matrix[:3, 3].tolist()),
        orientation=rotation_matrix_to_quaternion(matrix[:3, :3]),
        frame_id=frame_id,
    )


def invert_transform(matrix: NDArray[np.float64]) -> NDArray[np.float64]:
    """Return the inverse of a homogeneous 4x4 transform matrix."""
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def compose_transform(lhs: NDArray[np.float64], rhs: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compose two homogeneous 4x4 transform matrices."""
    return lhs @ rhs


def transform_pose(
    pose: Pose6D,
    transform: NDArray[np.float64],
    target_frame: str,
) -> Pose6D:
    """Apply a transform matrix to a pose and relabel it with the target frame."""
    result = compose_transform(transform, pose_to_matrix(pose))
    transformed = matrix_to_pose(result, frame_id=target_frame)
    transformed.timestamp_s = pose.timestamp_s
    return transformed

