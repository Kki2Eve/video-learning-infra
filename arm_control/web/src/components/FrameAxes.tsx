import { useMemo } from "react";
import { AxesHelper, Matrix4 } from "three";

interface FrameAxesProps {
  matrix: number[][];
  axisLength: number;
}

export function FrameAxes({ matrix, axisLength }: FrameAxesProps) {
  const transform = useMemo(() => {
    const next = new Matrix4();
    next.set(
      matrix[0][0], matrix[0][1], matrix[0][2], matrix[0][3],
      matrix[1][0], matrix[1][1], matrix[1][2], matrix[1][3],
      matrix[2][0], matrix[2][1], matrix[2][2], matrix[2][3],
      matrix[3][0], matrix[3][1], matrix[3][2], matrix[3][3],
    );
    return next;
  }, [matrix]);
  const helper = useMemo(() => new AxesHelper(axisLength), [axisLength]);

  return <primitive object={helper} matrix={transform} matrixAutoUpdate={false} />;
}
