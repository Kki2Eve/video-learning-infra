"""Thin service layer for the arm_control FastAPI backend."""

from __future__ import annotations

from typing import Any

import numpy as np

from arm_control.api import schemas
from arm_control.api.state import BackendState, TimedTargetPose
from arm_control.core.pose_types import JointState, Pose6D, Vector3
from arm_control.core.transform_utils import (
    euler_rpy_to_quaternion,
    invert_transform,
    pose_to_matrix,
    quaternion_to_euler_rpy,
    transform_pose,
)


def pose_from_position_euler(position: list[float], euler: list[float], frame_id: str) -> Pose6D:
    """Build a project ``Pose6D`` from API-friendly position and Euler arrays."""
    return Pose6D(
        position=Vector3.from_iterable(position),
        orientation=euler_rpy_to_quaternion(euler),
        frame_id=frame_id,
    )


def pose_to_output_model(pose: Pose6D) -> schemas.PoseOutputModel:
    """Convert a project pose into the API response schema."""
    matrix = pose_to_matrix(pose)
    euler = quaternion_to_euler_rpy(pose.orientation)
    return schemas.PoseOutputModel(
        position=[float(value) for value in pose.position.to_numpy().tolist()],
        euler=[float(value) for value in euler.tolist()],
        matrix=matrix_to_nested_list(matrix),
    )


def pose_to_trajectory_point_output_model(
    pose: Pose6D,
    timestamp_s: float,
) -> schemas.TrajectoryPointOutputModel:
    """Convert a project pose into one trajectory sample response object."""
    pose_output = pose_to_output_model(pose)
    return schemas.TrajectoryPointOutputModel(
        position=pose_output.position,
        euler=pose_output.euler,
        matrix=pose_output.matrix,
        t=float(timestamp_s),
    )


def matrix_to_nested_list(matrix: np.ndarray) -> list[list[float]]:
    """Convert a NumPy matrix into JSON-serializable nested floats."""
    return [[float(value) for value in row] for row in matrix.tolist()]


def get_health_response(state: BackendState) -> schemas.HealthResponse:
    """Return backend health, including simulation availability."""
    sim_available = state.ensure_sim_controller() is not None
    return schemas.HealthResponse(
        status="ok",
        sim_available=sim_available,
        sim_error=state.sim_error,
    )


def get_calibration_response(state: BackendState) -> schemas.CalibrationResponse:
    """Return the currently configured calibration values."""
    T_base_cam = state.calibration_manager.get_camera_to_robot_base()
    R_offset_hand_to_tool = state.hand_to_tool_rotation_offset()
    return schemas.CalibrationResponse(
        T_base_cam=matrix_to_nested_list(T_base_cam),
        R_offset_hand_to_tool=matrix_to_nested_list(R_offset_hand_to_tool),
    )


def transform_pose_to_base(
    state: BackendState,
    request: schemas.TransformPoseRequest,
) -> schemas.TransformPoseResponse:
    """Transform a camera-frame pose into the robot base frame."""
    pose_cam = pose_from_position_euler(
        position=request.pose_cam.position,
        euler=request.pose_cam.euler,
        frame_id=state.calibration_manager.camera_frame,
    )
    pose_base = state.calibration_manager.transform_pose_to_robot_base(pose_cam)
    return schemas.TransformPoseResponse(pose_base=pose_to_output_model(pose_base))


def transform_trajectory_to_base(
    state: BackendState,
    request: schemas.TransformTrajectoryRequest,
) -> schemas.TransformTrajectoryResponse:
    """Transform a camera-frame trajectory into the robot base frame."""
    trajectory_base: list[schemas.TrajectoryPointOutputModel] = []
    for point in request.trajectory_cam:
        pose_cam = pose_from_position_euler(
            position=point.position,
            euler=point.euler,
            frame_id=state.calibration_manager.camera_frame,
        )
        pose_cam.timestamp_s = float(point.t)
        pose_base = state.calibration_manager.transform_pose_to_robot_base(pose_cam)
        trajectory_base.append(
            pose_to_trajectory_point_output_model(
                pose=pose_base,
                timestamp_s=point.t,
            )
        )
    return schemas.TransformTrajectoryResponse(trajectory_base=trajectory_base)


def set_target_pose_base_response(
    state: BackendState,
    request: schemas.SetTargetPoseBaseRequest,
) -> schemas.SetTargetPoseBaseResponse:
    """Store one base-frame target pose for downstream simulator execution."""
    target_pose_base = pose_from_position_euler(
        position=request.target_pose_base.position,
        euler=request.target_pose_base.euler,
        frame_id=state.calibration_manager.robot_base_frame,
    )
    stored_pose = state.set_target_pose_base(target_pose_base)
    return schemas.SetTargetPoseBaseResponse(
        target_pose_base=pose_to_output_model(stored_pose) if stored_pose is not None else None,
    )


def set_target_trajectory_base_response(
    state: BackendState,
    request: schemas.SetTargetTrajectoryBaseRequest,
) -> schemas.SetTargetTrajectoryBaseResponse:
    """Store one base-frame target trajectory for downstream simulator execution."""
    target_trajectory = [
        TimedTargetPose(
            pose=pose_from_position_euler(
                position=point.position,
                euler=point.euler,
                frame_id=state.calibration_manager.robot_base_frame,
            ),
            t=float(point.t),
        )
        for point in request.target_trajectory_base
    ]
    for point, timed_target in zip(request.target_trajectory_base, target_trajectory, strict=False):
        timed_target.pose.timestamp_s = float(point.t)

    target_pose_base, trajectory_length, current_step_index = state.set_target_trajectory_base(target_trajectory)
    return schemas.SetTargetTrajectoryBaseResponse(
        target_pose_base=pose_to_output_model(target_pose_base) if target_pose_base is not None else None,
        trajectory_length=trajectory_length,
        current_step_index=current_step_index,
    )


def control_sim_playback_response(
    state: BackendState,
    request: schemas.SimPlaybackControlRequest,
) -> schemas.SimPlaybackControlResponse:
    """Update simulator trajectory playback mode."""
    action = request.action.strip().lower()
    if action == "play":
        playback_status, current_step_index, trajectory_length = state.play_trajectory_execution()
    elif action == "pause":
        playback_status, current_step_index, trajectory_length = state.pause_trajectory_execution()
    elif action == "reset":
        playback_status, current_step_index, trajectory_length = state.reset_trajectory_execution()
    elif action == "step":
        playback_status, current_step_index, trajectory_length = state.step_trajectory_execution()
    else:
        raise ValueError(f"Unsupported playback action: {request.action}")

    return schemas.SimPlaybackControlResponse(
        playback_status=playback_status,
        current_step_index=current_step_index,
        trajectory_length=trajectory_length,
    )


def sample_pose_cam_response(state: BackendState) -> schemas.SamplePoseCamResponse:
    """Sample an IK-reachable camera-frame pose using a Realman-friendly base-pose prior."""
    rng = np.random.default_rng()
    nominal_position = np.array([-0.30, 0.0, 0.35], dtype=np.float64)
    nominal_euler = np.array([2.8, -0.5, -2.67], dtype=np.float64)
    position_sigma = np.array([0.07, 0.10, 0.06], dtype=np.float64)
    euler_sigma = np.array([0.20, 0.18, 0.25], dtype=np.float64)

    seed_state = (
        JointState(names=state.controlled_joint_names, positions=list(state.initial_qpos))
        if state.initial_qpos
        else None
    )

    pose_base: Pose6D | None = None
    for _ in range(64):
        position_base = nominal_position + rng.normal(0.0, position_sigma)
        position_base = np.clip(position_base, state.workspace_min_position + 0.04, state.workspace_max_position - 0.04)
        euler_base = nominal_euler + rng.normal(0.0, euler_sigma)
        candidate = pose_from_position_euler(
            position=position_base.tolist(),
            euler=euler_base.tolist(),
            frame_id=state.calibration_manager.robot_base_frame,
        )
        safe_report = state.safety_guard.validate_pose(candidate)
        safe_pose = safe_report.clamped_pose or candidate
        ik_result = state.ik_solver.solve(target_pose=safe_pose, seed_state=seed_state)
        if ik_result.success:
            pose_base = safe_pose
            break

    if pose_base is None:
        margin = np.array([0.05, 0.05, 0.05], dtype=np.float64)
        lower = state.workspace_min_position + margin
        upper = state.workspace_max_position - margin
        position_base = rng.uniform(lower, upper)
        euler_base = rng.uniform(
            nominal_euler - np.array([0.25, 0.25, 0.35], dtype=np.float64),
            nominal_euler + np.array([0.25, 0.25, 0.35], dtype=np.float64),
        )
        pose_base = pose_from_position_euler(
            position=position_base.tolist(),
            euler=euler_base.tolist(),
            frame_id=state.calibration_manager.robot_base_frame,
        )

    T_cam_base = invert_transform(state.calibration_manager.get_camera_to_robot_base())
    pose_cam = transform_pose(
        pose=pose_base,
        transform=T_cam_base,
        target_frame=state.calibration_manager.camera_frame,
    )
    return schemas.SamplePoseCamResponse(pose_cam=pose_to_output_model(pose_cam))


def get_sim_state_response(state: BackendState) -> schemas.SimStateResponse:
    """Return the current simulator state using the existing SAPIEN controller."""
    state.execute_trajectory_if_needed()
    snapshot = state.get_execution_snapshot()
    target_pose_base = snapshot["target_pose_base"]
    target_trajectory_base = snapshot["target_trajectory_base"]
    actual_ee_trajectory = snapshot["actual_ee_trajectory"]
    current_step_index = snapshot["current_step_index"]
    trajectory_length = snapshot["trajectory_length"]
    playback_status = snapshot["playback_status"]
    last_ik_result = snapshot["last_ik_result"]
    ik_success = last_ik_result.success if last_ik_result is not None else None
    controller = state.ensure_sim_controller()
    if controller is None:
        return schemas.SimStateResponse(
            sim_connected=False,
            joint_names=[],
            qpos=[],
            ik_success=ik_success,
            playback_status=playback_status,
            target_pose_base=pose_to_output_model(target_pose_base) if target_pose_base is not None else None,
            target_trajectory_base=[
                pose_to_trajectory_point_output_model(point.pose, point.t)
                for point in target_trajectory_base
            ],
            actual_ee_pose=None,
            actual_ee_trajectory=[
                pose_to_trajectory_point_output_model(point.pose, point.t)
                for point in actual_ee_trajectory
            ],
            ee_pose=None,
            base_pose=None,
            camera_pose=None,
            T_base_cam=matrix_to_nested_list(state.calibration_manager.get_camera_to_robot_base()),
            robot_urdf_url=state.get_robot_urdf_url(),
            workspace_min_position=[float(value) for value in state.workspace_min_position.tolist()],
            workspace_max_position=[float(value) for value in state.workspace_max_position.tolist()],
            current_step_index=current_step_index,
            trajectory_length=trajectory_length,
            reject_count=snapshot["reject_count"],
            fail_count=snapshot["fail_count"],
            last_reject_reasons=snapshot["last_reject_reasons"],
            error=state.sim_error or snapshot["last_error"],
        )

    controller.step_simulation(1)
    controller.update_render()
    joint_state = controller.get_joint_state()
    base_pose = controller.get_base_pose_world()
    actual_ee_pose = controller.get_end_effector_pose()
    camera_pose = controller.get_camera_pose_world()
    actual_ee_pose_output = pose_to_output_model(actual_ee_pose)
    return schemas.SimStateResponse(
        sim_connected=True,
        joint_names=joint_state.names,
        qpos=[float(value) for value in joint_state.positions],
        ik_success=ik_success,
        playback_status=playback_status,
        target_pose_base=pose_to_output_model(target_pose_base) if target_pose_base is not None else None,
        target_trajectory_base=[
            pose_to_trajectory_point_output_model(point.pose, point.t)
            for point in target_trajectory_base
        ],
        actual_ee_pose=actual_ee_pose_output,
        actual_ee_trajectory=[
            pose_to_trajectory_point_output_model(point.pose, point.t)
            for point in actual_ee_trajectory
        ],
        ee_pose=actual_ee_pose_output,
        base_pose=pose_to_output_model(base_pose),
        camera_pose=pose_to_output_model(camera_pose),
        T_base_cam=matrix_to_nested_list(controller.get_T_base_cam()),
        robot_urdf_url=state.get_robot_urdf_url(),
        workspace_min_position=[float(value) for value in state.workspace_min_position.tolist()],
        workspace_max_position=[float(value) for value in state.workspace_max_position.tolist()],
        current_step_index=current_step_index,
        trajectory_length=trajectory_length,
        reject_count=snapshot["reject_count"],
        fail_count=snapshot["fail_count"],
        last_reject_reasons=snapshot["last_reject_reasons"],
        error=snapshot["last_error"],
    )

