import { useCallback, useEffect, useState } from "react";
import type {
  CalibrationResponse,
  PlaybackAction,
  PoseInput,
  SamplePoseCamResponse,
  SimPlaybackControlResponse,
  SetTargetPoseBaseResponse,
  SetTargetTrajectoryBaseResponse,
  SimStateResponse,
  TrajectoryPointInput,
  TransformTrajectoryResponse,
  TransformPoseResponse,
} from "../types/api";

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL ?? "";

async function readJson<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(`${API_BASE_URL}${path}`, {
    headers: {
      "Content-Type": "application/json",
    },
    ...init,
  });
  if (!response.ok) {
    throw new Error(`Request failed: ${response.status} ${response.statusText}`);
  }
  return (await response.json()) as T;
}

export function useBackend() {
  const [calibration, setCalibration] = useState<CalibrationResponse | null>(null);
  const [poseBase, setPoseBase] = useState<TransformPoseResponse["pose_base"] | null>(null);
  const [trajectoryBase, setTrajectoryBase] = useState<TransformTrajectoryResponse["trajectory_base"]>([]);
  const [simState, setSimState] = useState<SimStateResponse | null>(null);
  const [loadingCalibration, setLoadingCalibration] = useState(false);
  const [loadingTransform, setLoadingTransform] = useState(false);
  const [loadingTrajectory, setLoadingTrajectory] = useState(false);
  const [loadingSimState, setLoadingSimState] = useState(false);
  const [randomizing, setRandomizing] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const loadCalibration = useCallback(async () => {
    setLoadingCalibration(true);
    try {
      const payload = await readJson<CalibrationResponse>("/api/calibration");
      setCalibration(payload);
      setError(null);
      return payload;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown calibration error.";
      setError(message);
      throw backendError;
    } finally {
      setLoadingCalibration(false);
    }
  }, []);

  const transformPose = useCallback(async (poseCam: PoseInput) => {
    setLoadingTransform(true);
    try {
      const payload = await readJson<TransformPoseResponse>("/api/transform_pose", {
        method: "POST",
        body: JSON.stringify({ pose_cam: poseCam }),
      });
      setPoseBase(payload.pose_base);
      setError(null);
      return payload.pose_base;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown transform error.";
      setError(message);
      throw backendError;
    } finally {
      setLoadingTransform(false);
    }
  }, []);

  const transformTrajectory = useCallback(async (trajectoryCam: TrajectoryPointInput[]) => {
    setLoadingTrajectory(true);
    try {
      const payload = await readJson<TransformTrajectoryResponse>("/api/transform_trajectory", {
        method: "POST",
        body: JSON.stringify({ trajectory_cam: trajectoryCam }),
      });
      setTrajectoryBase(payload.trajectory_base);
      setError(null);
      return payload.trajectory_base;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown trajectory transform error.";
      setError(message);
      throw backendError;
    } finally {
      setLoadingTrajectory(false);
    }
  }, []);

  const samplePoseCam = useCallback(async () => {
    setRandomizing(true);
    try {
      const payload = await readJson<SamplePoseCamResponse>("/api/sample_pose_cam", {
        method: "POST",
      });
      setError(null);
      return payload.pose_cam;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown sample error.";
      setError(message);
      throw backendError;
    } finally {
      setRandomizing(false);
    }
  }, []);

  const loadSimState = useCallback(async () => {
    setLoadingSimState(true);
    try {
      const payload = await readJson<SimStateResponse>("/api/sim/state");
      setSimState(payload);
      setError(payload.error ?? null);
      return payload;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown sim-state error.";
      setError(message);
      throw backendError;
    } finally {
      setLoadingSimState(false);
    }
  }, []);

  const setTargetPoseBase = useCallback(async (poseBaseInput: PoseInput) => {
    try {
      const payload = await readJson<SetTargetPoseBaseResponse>("/api/sim/set_target_pose_base", {
        method: "POST",
        body: JSON.stringify({ target_pose_base: poseBaseInput }),
      });
      await loadSimState();
      setError(null);
      return payload.target_pose_base;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown set-target-pose error.";
      setError(message);
      throw backendError;
    }
  }, [loadSimState]);

  const setTargetTrajectoryBase = useCallback(async (trajectoryBaseInput: TrajectoryPointInput[]) => {
    try {
      const payload = await readJson<SetTargetTrajectoryBaseResponse>("/api/sim/set_target_trajectory_base", {
        method: "POST",
        body: JSON.stringify({ target_trajectory_base: trajectoryBaseInput }),
      });
      await loadSimState();
      setError(null);
      return payload;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown set-target-trajectory error.";
      setError(message);
      throw backendError;
    }
  }, [loadSimState]);

  const controlSimPlayback = useCallback(async (action: PlaybackAction) => {
    try {
      const payload = await readJson<SimPlaybackControlResponse>("/api/sim/playback", {
        method: "POST",
        body: JSON.stringify({ action }),
      });
      await loadSimState();
      setError(null);
      return payload;
    } catch (backendError) {
      const message = backendError instanceof Error ? backendError.message : "Unknown playback control error.";
      setError(message);
      throw backendError;
    }
  }, [loadSimState]);

  useEffect(() => {
    void loadCalibration();
  }, [loadCalibration]);

  useEffect(() => {
    void loadSimState();
    const intervalId = window.setInterval(() => {
      void loadSimState();
    }, 300);
    return () => window.clearInterval(intervalId);
  }, [loadSimState]);

  const clearTrajectoryBase = useCallback(() => {
    setTrajectoryBase([]);
  }, []);

  return {
    calibration,
    poseBase,
    trajectoryBase,
    simState,
    loadingCalibration,
    loadingTransform,
    loadingTrajectory,
    loadingSimState,
    randomizing,
    error,
    loadCalibration,
    loadSimState,
    transformPose,
    transformTrajectory,
    samplePoseCam,
    setTargetPoseBase,
    setTargetTrajectoryBase,
    controlSimPlayback,
    clearTrajectoryBase,
  };
}
