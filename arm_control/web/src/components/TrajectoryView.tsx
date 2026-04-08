import { Line } from "@react-three/drei";
import { useMemo } from "react";
import type { TrajectoryPointOutput } from "../types/api";

interface TrajectoryViewProps {
  trajectoryBase: TrajectoryPointOutput[];
  actualEeTrajectory: TrajectoryPointOutput[];
  TWorldBase: number[][];
  visible: boolean;
  currentIndex: number;
}

function pointPosition(point: { position: [number, number, number] }) {
  return [point.position[0], point.position[1], point.position[2]] as [number, number, number];
}

function transformPoint(matrix: number[][], point: [number, number, number]) {
  const [x, y, z] = point;
  return [
    matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3],
    matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3],
    matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3],
  ] as [number, number, number];
}

export function TrajectoryView({
  trajectoryBase,
  actualEeTrajectory,
  TWorldBase,
  visible,
  currentIndex,
}: TrajectoryViewProps) {
  if (!visible) {
    return null;
  }

  const basePoints = useMemo(
    () => trajectoryBase.map((point) => transformPoint(TWorldBase, pointPosition(point))),
    [TWorldBase, trajectoryBase],
  );
  const actualPoints = useMemo(() => actualEeTrajectory.map(pointPosition), [actualEeTrajectory]);
  const highlightedBasePoint = trajectoryBase[currentIndex];
  const highlightedActualPoint = actualEeTrajectory[actualEeTrajectory.length - 1];

  return (
    <>
      {basePoints.length > 1 ? (
        <Line points={basePoints} color="#6be0b1" lineWidth={2.6} />
      ) : null}
      {actualPoints.length > 1 ? (
        <Line points={actualPoints} color="#ff5d9e" lineWidth={2.4} />
      ) : null}

      {highlightedBasePoint ? (
        <mesh position={transformPoint(TWorldBase, pointPosition(highlightedBasePoint))}>
          <sphereGeometry args={[0.03, 18, 18]} />
          <meshStandardMaterial color="#85f0c7" emissive="#2d8f6a" emissiveIntensity={0.5} />
        </mesh>
      ) : null}

      {highlightedActualPoint ? (
        <mesh position={pointPosition(highlightedActualPoint)}>
          <sphereGeometry args={[0.03, 18, 18]} />
          <meshStandardMaterial color="#ff83b5" emissive="#b51d66" emissiveIntensity={0.55} />
        </mesh>
      ) : null}
    </>
  );
}
