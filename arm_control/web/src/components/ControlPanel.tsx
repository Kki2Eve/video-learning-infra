import { useState } from "react";
import type { ChangeEvent, ReactNode } from "react";
import type { PoseControlsState } from "../hooks/usePoseControls";
import type {
  CalibrationResponse,
  DebugMode,
  PoseInput,
  PoseOutput,
  SimStateResponse,
  TrajectoryPointInput,
  TrajectoryPointOutput,
} from "../types/api";
import { PoseSliders } from "./PoseSliders";
import { StatusPanel } from "./StatusPanel";

interface ControlPanelProps {
  mode: DebugMode;
  controls: PoseControlsState;
  poseCam: PoseInput;
  poseBase: PoseOutput | null;
  trajectoryCam: TrajectoryPointInput[];
  trajectoryBase: TrajectoryPointOutput[];
  currentTrajectoryIndex: number;
  calibration: CalibrationResponse | null;
  simState: SimStateResponse | null;
  error: string | null;
  loadingCalibration: boolean;
  loadingTransform: boolean;
  loadingTrajectory: boolean;
  loadingSimState: boolean;
  randomizing: boolean;
  showReferenceFrames: boolean;
  showRobot: boolean;
  showTargetFrame: boolean;
  showActualEeFrame: boolean;
  showTrajectories: boolean;
  showWorkspaceBox: boolean;
  onModeChange: (mode: DebugMode) => void;
  onSliderChange: (key: keyof PoseControlsState, value: number) => void;
  onResetPose: () => void;
  onRandomPose: () => void;
  onImportTrajectory: (event: ChangeEvent<HTMLInputElement>) => void;
  onAppendTrajectoryPoint: () => void;
  onClearTrajectory: () => void;
  onRandomTrajectory: () => void;
  onCurrentTrajectoryIndexChange: (index: number) => void;
  onPlaybackPlay: () => void;
  onPlaybackPause: () => void;
  onPlaybackReset: () => void;
  onPlaybackStep: () => void;
  onToggleShowReferenceFrames: () => void;
  onToggleShowRobot: () => void;
  onToggleShowTargetFrame: () => void;
  onToggleShowActualEeFrame: () => void;
  onToggleShowTrajectories: () => void;
  onToggleShowWorkspaceBox: () => void;
}

interface CollapsibleSectionProps {
  title: string;
  defaultOpen?: boolean;
  children: ReactNode;
}

function CollapsibleSection({
  title,
  defaultOpen = true,
  children,
}: CollapsibleSectionProps) {
  const [open, setOpen] = useState(defaultOpen);

  return (
    <section className="panel-section">
      <button
        type="button"
        className="section-toggle"
        onClick={() => setOpen((current) => !current)}
        aria-expanded={open}
      >
        <span className="section-title">{title}</span>
        <span className={`section-chevron ${open ? "section-chevron-open" : ""}`}>▾</span>
      </button>
      {open ? <div className="section-body">{children}</div> : null}
    </section>
  );
}

export function ControlPanel({
  mode,
  controls,
  poseCam,
  poseBase,
  trajectoryCam,
  trajectoryBase,
  currentTrajectoryIndex,
  calibration,
  simState,
  error,
  loadingCalibration,
  loadingTransform,
  loadingTrajectory,
  loadingSimState,
  randomizing,
  showReferenceFrames,
  showRobot,
  showTargetFrame,
  showActualEeFrame,
  showTrajectories,
  showWorkspaceBox,
  onModeChange,
  onSliderChange,
  onResetPose,
  onRandomPose,
  onImportTrajectory,
  onAppendTrajectoryPoint,
  onClearTrajectory,
  onRandomTrajectory,
  onCurrentTrajectoryIndexChange,
  onPlaybackPlay,
  onPlaybackPause,
  onPlaybackReset,
  onPlaybackStep,
  onToggleShowReferenceFrames,
  onToggleShowRobot,
  onToggleShowTargetFrame,
  onToggleShowActualEeFrame,
  onToggleShowTrajectories,
  onToggleShowWorkspaceBox,
}: ControlPanelProps) {
  const actionsTitle = mode === "single" ? "Single Pose Actions" : "Trajectory Actions";

  return (
    <aside className="control-panel">
      <div className="panel-header">
        <div>
          <h1>Web Pose Debugger</h1>
          <p>Debug camera-frame to robot-base pose transforms.</p>
        </div>
        <div className="panel-actions">
          <button type="button" className={mode === "single" ? "mode-active" : ""} onClick={() => onModeChange("single")}>
            Single Pose Mode
          </button>
          <button
            type="button"
            className={mode === "trajectory" ? "mode-active" : ""}
            onClick={() => onModeChange("trajectory")}
          >
            Trajectory Mode
          </button>
        </div>
      </div>

      <div className="backend-meta">
        <span>{loadingCalibration ? "Loading calibration..." : "Calibration ready"}</span>
        <span>{loadingTransform ? "Transforming pose..." : "Transform idle"}</span>
        <span>{loadingTrajectory ? "Transforming trajectory..." : "Trajectory idle"}</span>
        <span>{loadingSimState ? "Polling sim..." : simState?.sim_connected ? "Sim connected" : "Sim disconnected"}</span>
      </div>

      <CollapsibleSection title={actionsTitle}>
        {mode === "single" ? (
          <div className="button-stack">
            <button type="button" onClick={onResetPose}>
              Reset Pose
            </button>
            <button type="button" onClick={onRandomPose} disabled={randomizing}>
              {randomizing ? "Sampling..." : "Random Pose"}
            </button>
          </div>
        ) : (
          <>
            <div className="button-stack">
              <label className="file-button">
                <span>Import Trajectory JSON</span>
                <input type="file" accept="application/json" onChange={onImportTrajectory} />
              </label>
              <button type="button" onClick={onAppendTrajectoryPoint}>
                Append Current Pose
              </button>
              <button type="button" onClick={onRandomTrajectory}>
                Random Trajectory
              </button>
              <button type="button" onClick={onClearTrajectory}>
                Clear Trajectory
              </button>
            </div>
            <div className="button-stack button-stack-inline">
              <button type="button" onClick={onPlaybackPlay} disabled={trajectoryBase.length === 0}>
                Play
              </button>
              <button type="button" onClick={onPlaybackPause} disabled={trajectoryBase.length === 0}>
                Pause
              </button>
              <button type="button" onClick={onPlaybackReset} disabled={trajectoryBase.length === 0}>
                Reset
              </button>
              <button type="button" onClick={onPlaybackStep} disabled={trajectoryBase.length === 0}>
                Step
              </button>
            </div>
            <div className="toggle-list">
              <label className="toggle-row">
                <input type="checkbox" checked={showReferenceFrames} onChange={onToggleShowReferenceFrames} />
                <span>Show world/base/camera</span>
              </label>
              <label className="toggle-row">
                <input type="checkbox" checked={showRobot} onChange={onToggleShowRobot} />
                <span>Show robot</span>
              </label>
              <label className="toggle-row">
                <input type="checkbox" checked={showTargetFrame} onChange={onToggleShowTargetFrame} />
                <span>Show target frame</span>
              </label>
              <label className="toggle-row">
                <input type="checkbox" checked={showActualEeFrame} onChange={onToggleShowActualEeFrame} />
                <span>Show actual ee frame</span>
              </label>
              <label className="toggle-row">
                <input type="checkbox" checked={showTrajectories} onChange={onToggleShowTrajectories} />
                <span>Show trajectories</span>
              </label>
              <label className="toggle-row">
                <input type="checkbox" checked={showWorkspaceBox} onChange={onToggleShowWorkspaceBox} />
                <span>Show workspace box</span>
              </label>
            </div>
            <div className="trajectory-index">
              <div className="slider-header">
                <span>Current sample index</span>
                <span>{trajectoryCam.length > 0 ? currentTrajectoryIndex : "n/a"}</span>
              </div>
              <input
                type="range"
                min={0}
                max={Math.max(trajectoryCam.length - 1, 0)}
                step={1}
                value={trajectoryCam.length > 0 ? currentTrajectoryIndex : 0}
                disabled={trajectoryCam.length === 0}
                onChange={(event) => onCurrentTrajectoryIndexChange(Number(event.target.value))}
              />
            </div>
          </>
        )}
        {mode === "single" ? (
          <div className="toggle-list">
            <label className="toggle-row">
              <input type="checkbox" checked={showReferenceFrames} onChange={onToggleShowReferenceFrames} />
              <span>Show world/base/camera</span>
            </label>
            <label className="toggle-row">
              <input type="checkbox" checked={showRobot} onChange={onToggleShowRobot} />
              <span>Show robot</span>
            </label>
            <label className="toggle-row">
              <input type="checkbox" checked={showTargetFrame} onChange={onToggleShowTargetFrame} />
              <span>Show target frame</span>
            </label>
            <label className="toggle-row">
              <input type="checkbox" checked={showActualEeFrame} onChange={onToggleShowActualEeFrame} />
              <span>Show actual ee frame</span>
            </label>
            <label className="toggle-row">
              <input type="checkbox" checked={showTrajectories} onChange={onToggleShowTrajectories} />
              <span>Show trajectories</span>
            </label>
            <label className="toggle-row">
              <input type="checkbox" checked={showWorkspaceBox} onChange={onToggleShowWorkspaceBox} />
              <span>Show workspace box</span>
            </label>
          </div>
        ) : null}
      </CollapsibleSection>

      <CollapsibleSection title="Pose Controls" defaultOpen={false}>
        <PoseSliders controls={controls} onChange={onSliderChange} />
      </CollapsibleSection>

      <CollapsibleSection title="Status" defaultOpen={false}>
        <StatusPanel
          mode={mode}
          poseCam={poseCam}
          poseBase={poseBase}
          trajectoryCam={trajectoryCam}
          trajectoryBase={trajectoryBase}
          currentTrajectoryIndex={currentTrajectoryIndex}
          calibration={calibration}
          simState={simState}
          error={error}
        />
      </CollapsibleSection>
    </aside>
  );
}
