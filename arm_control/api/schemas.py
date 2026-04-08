"""Pydantic schemas for the arm_control FastAPI backend.

Conventions:
- ``T_ab`` means a transform that maps points from frame ``b`` into frame ``a``.
- Euler angles use roll-pitch-yaw order, i.e. ``[roll, pitch, yaw]`` in radians.
- Quaternions in this project use ``(x, y, z, w)`` ordering when they appear.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class PoseInputModel(BaseModel):
    """Pose input represented by position and Euler angles."""

    position: list[float] = Field(
        ...,
        min_length=3,
        max_length=3,
        description="Cartesian position [x, y, z] in meters.",
    )
    euler: list[float] = Field(
        ...,
        min_length=3,
        max_length=3,
        description="Euler angles [roll, pitch, yaw] in radians.",
    )


class PoseOutputModel(BaseModel):
    """Pose output represented by position, Euler angles, and a homogeneous matrix."""

    position: list[float] = Field(..., description="Cartesian position [x, y, z] in meters.")
    euler: list[float] = Field(..., description="Euler angles [roll, pitch, yaw] in radians.")
    matrix: list[list[float]] = Field(
        ...,
        description="Homogeneous transform matrix ``T_ab`` with row-major nested lists.",
    )


class TrajectoryPointInputModel(PoseInputModel):
    """One pose sample in a camera-frame trajectory."""

    t: float = Field(..., description="Trajectory timestamp in seconds.")


class TrajectoryPointOutputModel(PoseOutputModel):
    """One pose sample in a base-frame trajectory."""

    t: float = Field(..., description="Trajectory timestamp in seconds.")


class HealthResponse(BaseModel):
    """Basic service health response."""

    status: str
    sim_available: bool
    sim_error: str | None = None


class CalibrationResponse(BaseModel):
    """Current calibration response."""

    T_base_cam: list[list[float]]
    R_offset_hand_to_tool: list[list[float]]


class TransformPoseRequest(BaseModel):
    """Transform a camera-frame pose into the robot base frame."""

    pose_cam: PoseInputModel


class TransformPoseResponse(BaseModel):
    """Base-frame pose response."""

    pose_base: PoseOutputModel


class TransformTrajectoryRequest(BaseModel):
    """Transform a full camera-frame trajectory into the robot base frame."""

    trajectory_cam: list[TrajectoryPointInputModel]


class TransformTrajectoryResponse(BaseModel):
    """Base-frame trajectory response."""

    trajectory_base: list[TrajectoryPointOutputModel]


class SetTargetPoseBaseRequest(BaseModel):
    """Set one target pose expressed in the robot base frame."""

    target_pose_base: PoseInputModel


class SetTargetPoseBaseResponse(BaseModel):
    """Echo the stored base-frame target pose."""

    target_pose_base: PoseOutputModel | None


class SetTargetTrajectoryBaseRequest(BaseModel):
    """Set one target trajectory expressed in the robot base frame."""

    target_trajectory_base: list[TrajectoryPointInputModel]


class SetTargetTrajectoryBaseResponse(BaseModel):
    """Echo the stored base-frame target trajectory."""

    target_pose_base: PoseOutputModel | None
    trajectory_length: int
    current_step_index: int | None = None


class SimPlaybackControlRequest(BaseModel):
    """Playback control command for the simulator trajectory executor."""

    action: str = Field(
        ...,
        description="One of: play, pause, reset, step.",
    )


class SimPlaybackControlResponse(BaseModel):
    """Acknowledgement after updating simulator trajectory playback state."""

    playback_status: str
    current_step_index: int | None = None
    trajectory_length: int = 0


class SamplePoseCamResponse(BaseModel):
    """Random sample pose in the camera frame."""

    pose_cam: PoseOutputModel


class SimStateResponse(BaseModel):
    """Current simulator state exposed to the web frontend."""

    sim_connected: bool
    joint_names: list[str]
    qpos: list[float]
    ik_success: bool | None = None
    playback_status: str = "paused"
    target_pose_base: PoseOutputModel | None
    target_trajectory_base: list[TrajectoryPointOutputModel] = Field(
        default_factory=list,
        description="Target trajectory samples in the robot base frame.",
    )
    actual_ee_pose: PoseOutputModel | None
    actual_ee_trajectory: list[TrajectoryPointOutputModel] = Field(
        default_factory=list,
        description="Actual end-effector trajectory samples in the SAPIEN world frame.",
    )
    ee_pose: PoseOutputModel | None
    base_pose: PoseOutputModel | None
    camera_pose: PoseOutputModel | None
    T_base_cam: list[list[float]]
    robot_urdf_url: str | None = None
    workspace_min_position: list[float]
    workspace_max_position: list[float]
    current_step_index: int | None = None
    trajectory_length: int = 0
    reject_count: int = 0
    fail_count: int = 0
    last_reject_reasons: list[str] = Field(default_factory=list)
    error: str | None = None

