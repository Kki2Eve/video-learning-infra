import { useEffect, useMemo, useState } from "react";
import { ControlPanel } from "./components/ControlPanel";
import { SceneView } from "./components/SceneView";
import { useBackend } from "./hooks/useBackend";
import { usePoseControls } from "./hooks/usePoseControls";
import type { ChangeEvent } from "react";
import type { DebugMode, PoseInput, PoseOutput, TrajectoryPointInput, TrajectoryPointOutput } from "./types/api";

function clampIndex(index: number, length: number) {
  if (length <= 0) {
    return 0;
  }
  return Math.max(0, Math.min(index, length - 1));
}

function clamp(value: number, min: number, max: number) {
  return Math.max(min, Math.min(value, max));
}

function roundTo(value: number, digits: number) {
  return Number(value.toFixed(digits));
}

function randomBetween(min: number, max: number) {
  return min + Math.random() * (max - min);
}

function buildRandomTrajectory(centerPoseCam: PoseInput): TrajectoryPointInput[] {
  const sampleCount = Math.floor(randomBetween(8, 15));
  const trajectory: TrajectoryPointInput[] = [];
  let position: [number, number, number] = [
    roundTo(centerPoseCam.position[0] + randomBetween(-0.03, 0.03), 3),
    roundTo(centerPoseCam.position[1] + randomBetween(-0.03, 0.03), 3),
    roundTo(centerPoseCam.position[2] + randomBetween(-0.03, 0.03), 3),
  ];
  let euler: [number, number, number] = [
    roundTo(centerPoseCam.euler[0] + randomBetween(-0.08, 0.08), 3),
    roundTo(centerPoseCam.euler[1] + randomBetween(-0.08, 0.08), 3),
    roundTo(centerPoseCam.euler[2] + randomBetween(-0.10, 0.10), 3),
  ];

  for (let index = 0; index < sampleCount; index += 1) {
    if (index > 0) {
      position = [
        roundTo(clamp(position[0] + randomBetween(-0.025, 0.025), -1.0, 1.0), 3),
        roundTo(clamp(position[1] + randomBetween(-0.025, 0.025), -1.0, 1.0), 3),
        roundTo(clamp(position[2] + randomBetween(-0.02, 0.03), -1.0, 1.5), 3),
      ];
      euler = [
        roundTo(clamp(euler[0] + randomBetween(-0.06, 0.06), -3.14, 3.14), 3),
        roundTo(clamp(euler[1] + randomBetween(-0.06, 0.06), -3.14, 3.14), 3),
        roundTo(clamp(euler[2] + randomBetween(-0.08, 0.08), -3.14, 3.14), 3),
      ];
    }

    trajectory.push({
      position,
      euler,
      t: roundTo(index * 0.1, 3),
    });
  }

  return trajectory;
}

function poseToInput(pose: PoseInput | PoseOutput): PoseInput {
  return {
    position: pose.position,
    euler: pose.euler,
  };
}

function trajectoryPointToInput(point: TrajectoryPointInput | TrajectoryPointOutput): TrajectoryPointInput {
  return {
    position: point.position,
    euler: point.euler,
    t: point.t,
  };
}

export default function App() {
  const { controls, poseCam, setControlValue, resetPose, applyPose } = usePoseControls();
  const [mode, setMode] = useState<DebugMode>("single");
  const [trajectoryCam, setTrajectoryCam] = useState<TrajectoryPointInput[]>([]);
  const [currentTrajectoryIndex, setCurrentTrajectoryIndex] = useState(0);
  const [showReferenceFrames, setShowReferenceFrames] = useState(true);
  const [showRobot, setShowRobot] = useState(true);
  const [showTargetFrame, setShowTargetFrame] = useState(true);
  const [showActualEeFrame, setShowActualEeFrame] = useState(true);
  const [showTrajectories, setShowTrajectories] = useState(true);
  const [showWorkspaceBox, setShowWorkspaceBox] = useState(true);
  const {
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
    transformPose,
    transformTrajectory,
    samplePoseCam,
    setTargetPoseBase,
    setTargetTrajectoryBase,
    controlSimPlayback,
    clearTrajectoryBase,
  } = useBackend();

  const selectedTrajectoryPoseCam = useMemo(
    () => trajectoryCam[clampIndex(currentTrajectoryIndex, trajectoryCam.length)],
    [currentTrajectoryIndex, trajectoryCam],
  );

  useEffect(() => {
    if (mode !== "single") {
      return;
    }
    const timer = window.setTimeout(() => {
      void (async () => {
        try {
          const transformedPose = await transformPose(poseCam);
          await setTargetPoseBase(poseToInput(transformedPose));
        } catch {
          return;
        }
      })();
    }, 120);
    return () => window.clearTimeout(timer);
  }, [mode, poseCam, setTargetPoseBase, transformPose]);

  useEffect(() => {
    if (mode !== "trajectory") {
      return;
    }
    if (trajectoryCam.length === 0) {
      clearTrajectoryBase();
      void setTargetTrajectoryBase([]).catch(() => undefined);
      return;
    }
    const timer = window.setTimeout(() => {
      void (async () => {
        try {
          const transformedTrajectory = await transformTrajectory(trajectoryCam);
          await setTargetTrajectoryBase(transformedTrajectory.map(trajectoryPointToInput));
        } catch {
          return;
        }
      })();
    }, 120);
    return () => window.clearTimeout(timer);
  }, [clearTrajectoryBase, mode, setTargetTrajectoryBase, trajectoryCam, transformTrajectory]);

  useEffect(() => {
    setCurrentTrajectoryIndex((current) => clampIndex(current, trajectoryCam.length));
  }, [trajectoryCam.length]);

  const handleRandomPose = async () => {
    const randomPose = await samplePoseCam();
    applyPose(randomPose);
  };

  const handleResetPose = () => {
    resetPose();
  };

  const handleAppendTrajectoryPoint = () => {
    setMode("trajectory");
    setTrajectoryCam((current) => {
      const nextPoint: TrajectoryPointInput = {
        position: poseCam.position,
        euler: poseCam.euler,
        t: current.length === 0 ? 0.0 : Number((current[current.length - 1].t + 0.1).toFixed(3)),
      };
      const next = [...current, nextPoint];
      setCurrentTrajectoryIndex(next.length - 1);
      return next;
    });
  };

  const handleClearTrajectory = () => {
    setTrajectoryCam([]);
    setCurrentTrajectoryIndex(0);
    clearTrajectoryBase();
  };

  const handleRandomTrajectory = async () => {
    setMode("trajectory");
    const randomPose = await samplePoseCam();
    applyPose(randomPose);
    const nextTrajectory = buildRandomTrajectory({
      position: randomPose.position,
      euler: randomPose.euler,
    });
    setTrajectoryCam(nextTrajectory);
    setCurrentTrajectoryIndex(0);
  };

  const handleImportTrajectory = async (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) {
      return;
    }
    const text = await file.text();
    const parsed = JSON.parse(text) as TrajectoryPointInput[];
    const normalized = parsed.map((point) => ({
      position: point.position,
      euler: point.euler,
      t: point.t,
    }));
    setMode("trajectory");
    setTrajectoryCam(normalized);
    setCurrentTrajectoryIndex(0);
    event.target.value = "";
  };

  const displayedPoseCam = mode === "trajectory" && selectedTrajectoryPoseCam
    ? {
        position: selectedTrajectoryPoseCam.position,
        euler: selectedTrajectoryPoseCam.euler,
      }
    : poseCam;

  const playbackIndex = simState?.current_step_index ?? null;
  const activeTrajectoryIndex = mode === "trajectory"
    ? (playbackIndex ?? clampIndex(currentTrajectoryIndex, trajectoryCam.length))
    : clampIndex(currentTrajectoryIndex, trajectoryCam.length);

  const displayedPoseBase = mode === "trajectory" && trajectoryBase.length > 0
    ? trajectoryBase[clampIndex(activeTrajectoryIndex, trajectoryBase.length)]
    : poseBase;

  const targetPoseBase = simState?.target_pose_base ?? displayedPoseBase;
  const targetTrajectoryBase = simState?.target_trajectory_base?.length
    ? simState.target_trajectory_base
    : trajectoryBase;

  const handlePlaybackAction = (action: "play" | "pause" | "reset" | "step") => {
    void controlSimPlayback(action);
  };

  return (
    <div className="app-shell">
      <ControlPanel
        mode={mode}
        controls={controls}
        poseCam={displayedPoseCam}
        poseBase={displayedPoseBase}
        trajectoryCam={trajectoryCam}
        trajectoryBase={targetTrajectoryBase}
        currentTrajectoryIndex={clampIndex(activeTrajectoryIndex, trajectoryCam.length)}
        calibration={calibration}
        simState={simState}
        error={error}
        loadingCalibration={loadingCalibration}
        loadingTransform={loadingTransform}
        loadingTrajectory={loadingTrajectory}
        loadingSimState={loadingSimState}
        randomizing={randomizing}
        showReferenceFrames={showReferenceFrames}
        showRobot={showRobot}
        showTargetFrame={showTargetFrame}
        showActualEeFrame={showActualEeFrame}
        showTrajectories={showTrajectories}
        showWorkspaceBox={showWorkspaceBox}
        onModeChange={setMode}
        onSliderChange={setControlValue}
        onResetPose={handleResetPose}
        onRandomPose={() => {
          void handleRandomPose();
        }}
        onImportTrajectory={(event) => {
          void handleImportTrajectory(event);
        }}
        onAppendTrajectoryPoint={handleAppendTrajectoryPoint}
        onClearTrajectory={handleClearTrajectory}
        onRandomTrajectory={() => {
          void handleRandomTrajectory();
        }}
        onCurrentTrajectoryIndexChange={setCurrentTrajectoryIndex}
        onPlaybackPlay={() => handlePlaybackAction("play")}
        onPlaybackPause={() => handlePlaybackAction("pause")}
        onPlaybackReset={() => handlePlaybackAction("reset")}
        onPlaybackStep={() => handlePlaybackAction("step")}
        onToggleShowReferenceFrames={() => setShowReferenceFrames((current) => !current)}
        onToggleShowRobot={() => setShowRobot((current) => !current)}
        onToggleShowTargetFrame={() => setShowTargetFrame((current) => !current)}
        onToggleShowActualEeFrame={() => setShowActualEeFrame((current) => !current)}
        onToggleShowTrajectories={() => setShowTrajectories((current) => !current)}
        onToggleShowWorkspaceBox={() => setShowWorkspaceBox((current) => !current)}
      />
      <SceneView
        mode={mode}
        calibration={calibration}
        poseBase={targetPoseBase}
        simState={simState}
        trajectoryBase={targetTrajectoryBase}
        actualEeTrajectory={simState?.actual_ee_trajectory ?? []}
        currentTrajectoryIndex={clampIndex(activeTrajectoryIndex, trajectoryCam.length)}
        showReferenceFrames={showReferenceFrames}
        showRobot={showRobot}
        showTargetFrame={showTargetFrame}
        showActualEeFrame={showActualEeFrame}
        showTrajectories={showTrajectories}
        showWorkspaceBox={showWorkspaceBox}
      />
    </div>
  );
}
