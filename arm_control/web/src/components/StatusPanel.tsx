import type {
  CalibrationResponse,
  DebugMode,
  PoseInput,
  PoseOutput,
  SimStateResponse,
  TrajectoryPointInput,
  TrajectoryPointOutput,
} from "../types/api";

interface StatusPanelProps {
  mode: DebugMode;
  poseCam: PoseInput;
  poseBase: PoseOutput | null;
  trajectoryCam: TrajectoryPointInput[];
  trajectoryBase: TrajectoryPointOutput[];
  currentTrajectoryIndex: number;
  calibration: CalibrationResponse | null;
  simState: SimStateResponse | null;
  error: string | null;
}

function formatArray(values: number[]) {
  return `[${values.map((value) => value.toFixed(3)).join(", ")}]`;
}

function formatMatrix(matrix: number[][]) {
  return matrix
    .map((row) => `[${row.map((value) => value.toFixed(3)).join(", ")}]`)
    .join("\n");
}

function multiplyMatrices(left: number[][], right: number[][]) {
  return Array.from({ length: 4 }, (_, row) =>
    Array.from({ length: 4 }, (_, column) =>
      left[row][0] * right[0][column]
      + left[row][1] * right[1][column]
      + left[row][2] * right[2][column]
      + left[row][3] * right[3][column]
    ),
  );
}

function posePositionFromMatrix(matrix: number[][]) {
  return [matrix[0][3], matrix[1][3], matrix[2][3]];
}

function computePositionError(targetWorld: number[][], actualWorld: number[][]) {
  const target = posePositionFromMatrix(targetWorld);
  const actual = posePositionFromMatrix(actualWorld);
  const dx = target[0] - actual[0];
  const dy = target[1] - actual[1];
  const dz = target[2] - actual[2];
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function clamp(value: number, min: number, max: number) {
  return Math.max(min, Math.min(value, max));
}

function computeOrientationErrorDeg(targetWorld: number[][], actualWorld: number[][]) {
  const trace =
    targetWorld[0][0] * actualWorld[0][0]
    + targetWorld[1][0] * actualWorld[1][0]
    + targetWorld[2][0] * actualWorld[2][0]
    + targetWorld[0][1] * actualWorld[0][1]
    + targetWorld[1][1] * actualWorld[1][1]
    + targetWorld[2][1] * actualWorld[2][1]
    + targetWorld[0][2] * actualWorld[0][2]
    + targetWorld[1][2] * actualWorld[1][2]
    + targetWorld[2][2] * actualWorld[2][2];
  const cosTheta = clamp((trace - 1.0) * 0.5, -1.0, 1.0);
  return (Math.acos(cosTheta) * 180.0) / Math.PI;
}

export function StatusPanel({
  mode,
  poseCam,
  poseBase,
  trajectoryCam,
  trajectoryBase,
  currentTrajectoryIndex,
  calibration,
  simState,
  error,
}: StatusPanelProps) {
  const targetPoseBase = simState?.target_pose_base ?? poseBase;
  const actualEePose = simState?.actual_ee_pose ?? simState?.ee_pose ?? null;
  const basePose = simState?.base_pose ?? null;
  const cameraPose = simState?.camera_pose ?? null;
  const simTargetTrajectory = simState?.target_trajectory_base ?? [];
  const simActualTrajectory = simState?.actual_ee_trajectory ?? [];

  const targetWorldMatrix = targetPoseBase && basePose
    ? multiplyMatrices(basePose.matrix, targetPoseBase.matrix)
    : targetPoseBase?.matrix ?? null;
  const actualWorldMatrix = actualEePose?.matrix ?? null;
  const positionError = targetWorldMatrix && actualWorldMatrix
    ? computePositionError(targetWorldMatrix, actualWorldMatrix)
    : null;
  const orientationErrorDeg = targetWorldMatrix && actualWorldMatrix
    ? computeOrientationErrorDeg(targetWorldMatrix, actualWorldMatrix)
    : null;

  return (
    <>
      {error ? <div className="error-banner">{error}</div> : null}
      <div className="status-block">
        <div className="status-label">mode</div>
        <pre>{mode === "single" ? "Single Pose Mode" : "Trajectory Mode"}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">ik status</div>
        <pre>
          {simState?.ik_success === true
            ? "success"
            : simState?.ik_success === false
              ? "fail"
              : "waiting..."}
        </pre>
      </div>
      <div className="status-block">
        <div className="status-label">playback status</div>
        <pre>{simState?.playback_status ?? "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">pose_cam.position</div>
        <pre>{formatArray(poseCam.position)}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">pose_cam.euler</div>
        <pre>{formatArray(poseCam.euler)}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">pose_base.position</div>
        <pre>{targetPoseBase ? formatArray(targetPoseBase.position) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">pose_base.euler</div>
        <pre>{targetPoseBase ? formatArray(targetPoseBase.euler) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">T_base_cam</div>
        <pre>{simState ? formatMatrix(simState.T_base_cam) : calibration ? formatMatrix(calibration.T_base_cam) : "loading..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">trajectory length</div>
        <pre>{`${trajectoryCam.length} cam samples / ${trajectoryBase.length} base samples / ${simTargetTrajectory.length} backend target samples / ${simActualTrajectory.length} actual samples`}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">current sample index</div>
        <pre>{simState?.current_step_index ?? (trajectoryCam.length > 0 ? `${currentTrajectoryIndex}` : "n/a")}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">reject / fail count</div>
        <pre>{`${simState?.reject_count ?? 0} / ${simState?.fail_count ?? 0}`}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">actual_ee.position</div>
        <pre>{actualEePose ? formatArray(actualEePose.position) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">actual_ee.euler</div>
        <pre>{actualEePose ? formatArray(actualEePose.euler) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">base_pose.position</div>
        <pre>{basePose ? formatArray(basePose.position) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">camera_pose.position</div>
        <pre>{cameraPose ? formatArray(cameraPose.position) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">position error (m)</div>
        <pre>{positionError !== null ? positionError.toFixed(4) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">orientation error (deg)</div>
        <pre>{orientationErrorDeg !== null ? orientationErrorDeg.toFixed(2) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">current qpos</div>
        <pre>{simState ? formatArray(simState.qpos) : "waiting..."}</pre>
      </div>
      <div className="status-block">
        <div className="status-label">last reject reasons</div>
        <pre>{simState?.last_reject_reasons?.length ? simState.last_reject_reasons.join("\n") : "none"}</pre>
      </div>
    </>
  );
}
