import { useMemo, useState } from "react";
import type { PoseInput, PoseOutput } from "../types/api";

export interface PoseControlsState {
  tx: number;
  ty: number;
  tz: number;
  roll: number;
  pitch: number;
  yaw: number;
}

const DEFAULT_POSE: PoseControlsState = {
  tx: 0,
  ty: 0,
  tz: 0,
  roll: 0,
  pitch: 0,
  yaw: 0,
};

export function usePoseControls() {
  const [controls, setControls] = useState<PoseControlsState>(DEFAULT_POSE);

  const poseCam = useMemo<PoseInput>(
    () => ({
      position: [controls.tx, controls.ty, controls.tz],
      euler: [controls.roll, controls.pitch, controls.yaw],
    }),
    [controls],
  );

  const setControlValue = (key: keyof PoseControlsState, value: number) => {
    setControls((current) => ({
      ...current,
      [key]: value,
    }));
  };

  const resetPose = () => {
    setControls(DEFAULT_POSE);
  };

  const applyPose = (pose: PoseInput | PoseOutput) => {
    setControls({
      tx: pose.position[0],
      ty: pose.position[1],
      tz: pose.position[2],
      roll: pose.euler[0],
      pitch: pose.euler[1],
      yaw: pose.euler[2],
    });
  };

  return {
    controls,
    poseCam,
    setControlValue,
    resetPose,
    applyPose,
  };
}
