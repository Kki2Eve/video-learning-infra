export type Matrix4x4 = number[][];
export type DebugMode = "single" | "trajectory";

export interface PoseInput {
  position: [number, number, number];
  euler: [number, number, number];
}

export interface PoseOutput {
  position: [number, number, number];
  euler: [number, number, number];
  matrix: Matrix4x4;
}

export interface TrajectoryPointInput extends PoseInput {
  t: number;
}

export interface TrajectoryPointOutput extends PoseOutput {
  t: number;
}

export interface CalibrationResponse {
  T_base_cam: Matrix4x4;
  R_offset_hand_to_tool: number[][];
}

export interface TransformPoseRequest {
  pose_cam: PoseInput;
}

export interface TransformPoseResponse {
  pose_base: PoseOutput;
}

export interface TransformTrajectoryRequest {
  trajectory_cam: TrajectoryPointInput[];
}

export interface TransformTrajectoryResponse {
  trajectory_base: TrajectoryPointOutput[];
}

export interface SetTargetPoseBaseRequest {
  target_pose_base: PoseInput;
}

export interface SetTargetPoseBaseResponse {
  target_pose_base: PoseOutput | null;
}

export interface SetTargetTrajectoryBaseRequest {
  target_trajectory_base: TrajectoryPointInput[];
}

export interface SetTargetTrajectoryBaseResponse {
  target_pose_base: PoseOutput | null;
  trajectory_length: number;
  current_step_index: number | null;
}

export type PlaybackAction = "play" | "pause" | "reset" | "step";

export interface SimPlaybackControlRequest {
  action: PlaybackAction;
}

export interface SimPlaybackControlResponse {
  playback_status: string;
  current_step_index: number | null;
  trajectory_length: number;
}

export interface SamplePoseCamResponse {
  pose_cam: PoseOutput;
}

export interface SimStateResponse {
  sim_connected: boolean;
  joint_names: string[];
  qpos: number[];
  ik_success: boolean | null;
  playback_status: string;
  target_pose_base: PoseOutput | null;
  target_trajectory_base: TrajectoryPointOutput[];
  actual_ee_pose: PoseOutput | null;
  actual_ee_trajectory: TrajectoryPointOutput[];
  ee_pose: PoseOutput | null;
  base_pose: PoseOutput | null;
  camera_pose: PoseOutput | null;
  T_base_cam: Matrix4x4;
  robot_urdf_url: string | null;
  workspace_min_position: [number, number, number];
  workspace_max_position: [number, number, number];
  current_step_index: number | null;
  trajectory_length: number;
  reject_count: number;
  fail_count: number;
  last_reject_reasons: string[];
  error: string | null;
}
