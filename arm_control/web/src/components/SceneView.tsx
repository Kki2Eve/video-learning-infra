import { Canvas } from "@react-three/fiber";
import { Line, OrbitControls } from "@react-three/drei";
import { useMemo } from "react";
import type {
  CalibrationResponse,
  DebugMode,
  PoseOutput,
  SimStateResponse,
  TrajectoryPointOutput,
} from "../types/api";
import { FrameAxes } from "./FrameAxes";
import { RobotView } from "./RobotView";
import { TrajectoryView } from "./TrajectoryView";

const IDENTITY_MATRIX = [
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1],
];

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

function transformPoint(matrix: number[][], point: [number, number, number]) {
  const [x, y, z] = point;
  return [
    matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
    matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
    matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
  ] as [number, number, number];
}

function buildWorkspaceEdges(minPosition: [number, number, number], maxPosition: [number, number, number]) {
  const [minX, minY, minZ] = minPosition;
  const [maxX, maxY, maxZ] = maxPosition;
  const corners: [number, number, number][] = [
    [minX, minY, minZ],
    [maxX, minY, minZ],
    [maxX, maxY, minZ],
    [minX, maxY, minZ],
    [minX, minY, maxZ],
    [maxX, minY, maxZ],
    [maxX, maxY, maxZ],
    [minX, maxY, maxZ],
  ];
  const edges: [number, number][] = [
    [0, 1], [1, 2], [2, 3], [3, 0],
    [4, 5], [5, 6], [6, 7], [7, 4],
    [0, 4], [1, 5], [2, 6], [3, 7],
  ];
  return edges.map(([start, end]) => [corners[start], corners[end]] as [[number, number, number], [number, number, number]]);
}

interface SceneViewProps {
  mode: DebugMode;
  calibration: CalibrationResponse | null;
  poseBase: PoseOutput | null;
  simState: SimStateResponse | null;
  trajectoryBase: TrajectoryPointOutput[];
  actualEeTrajectory: TrajectoryPointOutput[];
  currentTrajectoryIndex: number;
  showReferenceFrames: boolean;
  showRobot: boolean;
  showTargetFrame: boolean;
  showActualEeFrame: boolean;
  showTrajectories: boolean;
  showWorkspaceBox: boolean;
}

export function SceneView({
  mode,
  calibration,
  poseBase,
  simState,
  trajectoryBase,
  actualEeTrajectory,
  currentTrajectoryIndex,
  showReferenceFrames,
  showRobot,
  showTargetFrame,
  showActualEeFrame,
  showTrajectories,
  showWorkspaceBox,
}: SceneViewProps) {
  const worldBaseMatrix = simState?.base_pose?.matrix ?? IDENTITY_MATRIX;
  const cameraBaseMatrix = simState?.T_base_cam ?? calibration?.T_base_cam ?? IDENTITY_MATRIX;
  const cameraMatrix = simState?.camera_pose?.matrix ?? multiplyMatrices(worldBaseMatrix, cameraBaseMatrix);
  const cameraLookSegment = useMemo(
    () => [
      transformPoint(cameraMatrix, [0, 0, 0]),
      transformPoint(cameraMatrix, [0.26, 0, 0]),
    ] as [[number, number, number], [number, number, number]],
    [cameraMatrix],
  );
  const targetBaseLocalMatrix =
    mode === "trajectory"
      ? trajectoryBase[currentTrajectoryIndex]?.matrix ?? poseBase?.matrix ?? IDENTITY_MATRIX
      : poseBase?.matrix ?? IDENTITY_MATRIX;
  const targetMatrix = multiplyMatrices(worldBaseMatrix, targetBaseLocalMatrix);
  const actualEeMatrix = simState?.actual_ee_pose?.matrix ?? simState?.ee_pose?.matrix ?? null;
  const workspaceEdges = useMemo(
    () =>
      buildWorkspaceEdges(
        simState?.workspace_min_position ?? [-0.6, -0.6, 0.0],
        simState?.workspace_max_position ?? [0.6, 0.6, 1.2],
      ).map(([start, end]) => [
        transformPoint(worldBaseMatrix, start),
        transformPoint(worldBaseMatrix, end),
      ] as [[number, number, number], [number, number, number]]),
    [simState?.workspace_max_position, simState?.workspace_min_position, worldBaseMatrix],
  );

  const frameLegend = useMemo(
    () => [
      ...(showReferenceFrames
        ? [
            { name: "world frame", detail: "fixed identity frame", tone: "world" },
            { name: "base frame", detail: "actual T_world_base from SAPIEN", tone: "base" },
            { name: "camera frame", detail: "actual T_world_cam from SAPIEN", tone: "camera" },
            { name: "camera look", detail: "camera local +x viewing direction", tone: "look" },
          ]
        : []),
      ...(showTargetFrame ? [{ name: "target frame", detail: "current target pose expressed in world", tone: "target" }] : []),
      ...(showActualEeFrame ? [{ name: "actual ee frame", detail: "current end-effector pose from SAPIEN", tone: "actual" }] : []),
      ...(showRobot ? [{ name: "robot", detail: "frontend geometry driven by qpos", tone: "robot" }] : []),
    ],
    [showActualEeFrame, showReferenceFrames, showRobot, showTargetFrame],
  );

  return (
    <section className="scene-shell">
      <div className="scene-toolbar">
        {frameLegend.map((entry) => (
          <div key={entry.name} className={`legend-pill legend-${entry.tone}`}>
            <span>{entry.name}</span>
            <small>{entry.detail}</small>
          </div>
        ))}
      </div>
      <div className="scene-canvas">
        <Canvas
          camera={{ position: [2.2, -2.0, 1.8], fov: 45 }}
          onCreated={({ camera }) => {
            camera.up.set(0, 0, 1);
            camera.lookAt(0, 0, 0.25);
          }}
        >
          <color attach="background" args={["#0b1117"]} />
          <ambientLight intensity={0.9} />
          <directionalLight position={[3, 4, 5]} intensity={1.4} />
          <gridHelper args={[4, 20, "#3a526b", "#1a2c3b"]} rotation={[Math.PI / 2, 0, 0]} position={[0, 0, 0]} />
          <mesh position={[0, 0, -0.001]}>
            <planeGeometry args={[4, 4]} />
            <meshStandardMaterial color="#11202d" transparent opacity={0.35} />
          </mesh>

          {showWorkspaceBox
            ? workspaceEdges.map((edge, index) => (
                <Line key={`workspace-${index}`} points={edge} color="#5d7da0" lineWidth={1.4} />
              ))
            : null}

          <RobotView
            urdfUrl={simState?.robot_urdf_url ?? null}
            jointNames={simState?.joint_names ?? []}
            qpos={simState?.qpos ?? []}
            baseMatrix={worldBaseMatrix}
            visible={showRobot}
          />
          {showReferenceFrames ? <FrameAxes matrix={IDENTITY_MATRIX} axisLength={0.6} /> : null}
          {showReferenceFrames ? <FrameAxes matrix={worldBaseMatrix} axisLength={0.38} /> : null}
          {showReferenceFrames ? <FrameAxes matrix={cameraMatrix} axisLength={0.28} /> : null}
          {showReferenceFrames ? <Line points={cameraLookSegment} color="#ffd54f" lineWidth={2.8} /> : null}
          {showReferenceFrames ? (
            <mesh position={cameraLookSegment[1]}>
              <sphereGeometry args={[0.022, 16, 16]} />
              <meshStandardMaterial color="#ffe082" emissive="#b68b00" emissiveIntensity={0.55} />
            </mesh>
          ) : null}
          {showTargetFrame ? <FrameAxes matrix={targetMatrix} axisLength={0.34} /> : null}
          {showActualEeFrame && actualEeMatrix ? <FrameAxes matrix={actualEeMatrix} axisLength={0.32} /> : null}
          <TrajectoryView
            trajectoryBase={trajectoryBase}
            actualEeTrajectory={actualEeTrajectory}
            TWorldBase={worldBaseMatrix}
            visible={showTrajectories}
            currentIndex={currentTrajectoryIndex}
          />

          <OrbitControls enableDamping target={[0, 0, 0.25]} />
        </Canvas>
      </div>
    </section>
  );
}
