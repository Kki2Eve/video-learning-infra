import { useEffect, useMemo, useState } from "react";
import type { GroupProps } from "@react-three/fiber";
import { Matrix4, Quaternion, Vector3 } from "three";
import type { BufferGeometry } from "three";
import { STLLoader } from "three/examples/jsm/loaders/STLLoader.js";
import type { UrdfJoint, UrdfLink, UrdfModel, UrdfVisual } from "../utils/urdf";
import { computeRobotSkeleton, loadUrdfModel, matrixFromJointMotion, matrixFromOrigin } from "../utils/urdf";

const API_BASE_URL = import.meta.env.VITE_API_BASE_URL ?? "";

interface RobotViewProps extends GroupProps {
  urdfUrl: string | null;
  jointNames: string[];
  qpos: number[];
  baseMatrix?: number[][];
  visible?: boolean;
}

interface LinkNodeProps {
  linkName: string;
  model: UrdfModel;
  qposByJoint: Record<string, number>;
  meshGeometries: Map<string, BufferGeometry>;
}

function resolveResourceUrl(resourceUrl: string | null) {
  if (!resourceUrl) {
    return null;
  }
  if (resourceUrl.startsWith("http://") || resourceUrl.startsWith("https://")) {
    return resourceUrl;
  }
  if (resourceUrl.startsWith("/") && API_BASE_URL) {
    return `${API_BASE_URL}${resourceUrl}`;
  }
  try {
    return new URL(resourceUrl, window.location.origin).toString();
  } catch {
    return resourceUrl;
  }
}

function matrixFromArray(matrixElements?: number[][]) {
  const matrix = new Matrix4();
  if (!matrixElements) {
    return matrix.identity();
  }
  matrix.set(
    matrixElements[0][0], matrixElements[0][1], matrixElements[0][2], matrixElements[0][3],
    matrixElements[1][0], matrixElements[1][1], matrixElements[1][2], matrixElements[1][3],
    matrixElements[2][0], matrixElements[2][1], matrixElements[2][2], matrixElements[2][3],
    matrixElements[3][0], matrixElements[3][1], matrixElements[3][2], matrixElements[3][3],
  );
  return matrix;
}

function decomposeMatrix(matrix: Matrix4) {
  const position = new Vector3();
  const quaternion = new Quaternion();
  const scale = new Vector3();
  matrix.decompose(position, quaternion, scale);
  return {
    position: [position.x, position.y, position.z] as [number, number, number],
    quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w] as [number, number, number, number],
    scale: [scale.x, scale.y, scale.z] as [number, number, number],
  };
}

function VisualMesh({
  visual,
  geometry,
}: {
  visual: UrdfVisual;
  geometry: BufferGeometry | undefined;
}) {
  const transform = useMemo(
    () => decomposeMatrix(matrixFromOrigin(visual.originXyz, visual.originRpy)),
    [visual.originRpy, visual.originXyz],
  );

  if (!geometry) {
    return null;
  }

  return (
    <mesh
      geometry={geometry}
      position={transform.position}
      quaternion={transform.quaternion}
      scale={visual.scale}
      castShadow
      receiveShadow
    >
      <meshStandardMaterial color="#cfd7e3" metalness={0.18} roughness={0.58} />
    </mesh>
  );
}

function LinkVisuals({
  link,
  meshGeometries,
}: {
  link: UrdfLink | undefined;
  meshGeometries: Map<string, BufferGeometry>;
}) {
  if (!link) {
    return null;
  }

  return (
    <>
      {link.visuals.map((visual, index) => (
        <VisualMesh
          key={`${link.name}-visual-${index}`}
          visual={visual}
          geometry={visual.meshUrl ? meshGeometries.get(visual.meshUrl) : undefined}
        />
      ))}
    </>
  );
}

function JointNode({
  joint,
  model,
  qposByJoint,
  meshGeometries,
}: {
  joint: UrdfJoint;
  model: UrdfModel;
  qposByJoint: Record<string, number>;
  meshGeometries: Map<string, BufferGeometry>;
}) {
  const jointOrigin = useMemo(
    () => decomposeMatrix(matrixFromOrigin(joint.originXyz, joint.originRpy)),
    [joint.originRpy, joint.originXyz],
  );
  const jointMotion = useMemo(
    () => decomposeMatrix(matrixFromJointMotion(joint, qposByJoint[joint.name] ?? 0.0)),
    [joint, qposByJoint],
  );
  const childLink = model.links.get(joint.child);
  const childJoints = model.childJointsByLink.get(joint.child) ?? [];

  return (
    <group position={jointOrigin.position} quaternion={jointOrigin.quaternion}>
      <group position={jointMotion.position} quaternion={jointMotion.quaternion}>
        <LinkVisuals link={childLink} meshGeometries={meshGeometries} />
        {childJoints.map((childJoint) => (
          <JointNode
            key={childJoint.name}
            joint={childJoint}
            model={model}
            qposByJoint={qposByJoint}
            meshGeometries={meshGeometries}
          />
        ))}
      </group>
    </group>
  );
}

function loadMeshGeometries(meshUrls: string[]) {
  const loader = new STLLoader();
  return Promise.all(
    meshUrls.map(
      (meshUrl) =>
        new Promise<[string, BufferGeometry]>((resolve, reject) => {
          loader.load(
            meshUrl,
            (geometry) => {
              geometry.computeVertexNormals();
              resolve([meshUrl, geometry]);
            },
            undefined,
            (error) => reject(error),
          );
        }),
    ),
  );
}

export function RobotView({
  urdfUrl,
  jointNames,
  qpos,
  baseMatrix,
  visible = true,
  ...groupProps
}: RobotViewProps) {
  const [model, setModel] = useState<UrdfModel | null>(null);
  const [meshGeometries, setMeshGeometries] = useState<Map<string, BufferGeometry>>(new Map());
  const [loadError, setLoadError] = useState<string | null>(null);
  const resolvedUrdfUrl = useMemo(() => resolveResourceUrl(urdfUrl), [urdfUrl]);

  useEffect(() => {
    if (!resolvedUrdfUrl) {
      setModel(null);
      setMeshGeometries(new Map());
      setLoadError(null);
      return;
    }

    let cancelled = false;
    void (async () => {
      try {
        const nextModel = await loadUrdfModel(resolvedUrdfUrl);
        const meshUrls = Array.from(
          new Set(
            Array.from(nextModel.links.values())
              .flatMap((link) => link.visuals)
              .map((visual) => visual.meshUrl)
              .filter((meshUrl): meshUrl is string => Boolean(meshUrl)),
          ),
        );
        const loadedGeometries = await loadMeshGeometries(meshUrls);
        if (!cancelled) {
          setModel(nextModel);
          setMeshGeometries(new Map(loadedGeometries));
          setLoadError(null);
        }
      } catch (error) {
        if (!cancelled) {
          setModel(null);
          setMeshGeometries(new Map());
          const message = error instanceof Error ? error.message : "Unknown URDF load error.";
          setLoadError(message);
          console.error("RobotView failed to load robot model:", message, resolvedUrdfUrl);
        }
      }
    })();

    return () => {
      cancelled = true;
    };
  }, [resolvedUrdfUrl]);

  const qposByJoint = useMemo(
    () =>
      Object.fromEntries(
        jointNames.map((jointName, index) => [jointName, qpos[index] ?? 0.0]),
      ) as Record<string, number>,
    [jointNames, qpos],
  );
  const baseTransform = useMemo(() => decomposeMatrix(matrixFromArray(baseMatrix)), [baseMatrix]);
  const fallbackSkeleton = useMemo(
    () => (model ? computeRobotSkeleton(model, qposByJoint, baseMatrix) : null),
    [baseMatrix, model, qposByJoint],
  );
  const rootLink = model?.links.get(model.rootLink);
  const rootJoints = model?.childJointsByLink.get(model.rootLink) ?? [];

  if (!visible) {
    return null;
  }

  if (!model) {
    if (loadError && baseMatrix) {
      const fallbackPosition: [number, number, number] = [
        baseMatrix[0][3],
        baseMatrix[1][3],
        baseMatrix[2][3],
      ];
      return (
        <group {...groupProps}>
          <mesh position={fallbackPosition}>
            <boxGeometry args={[0.08, 0.08, 0.08]} />
            <meshStandardMaterial color="#ff6f6f" emissive="#842f2f" emissiveIntensity={0.55} />
          </mesh>
        </group>
      );
    }
    return null;
  }

  const hasMeshGeometry = meshGeometries.size > 0;

  return (
    <group
      {...groupProps}
      position={baseTransform.position}
      quaternion={baseTransform.quaternion}
      scale={baseTransform.scale}
    >
      {hasMeshGeometry ? <LinkVisuals link={rootLink} meshGeometries={meshGeometries} /> : null}
      {rootJoints.map((joint) => (
        <JointNode
          key={joint.name}
          joint={joint}
          model={model}
          qposByJoint={qposByJoint}
          meshGeometries={meshGeometries}
        />
      ))}

      {!hasMeshGeometry && fallbackSkeleton ? (
        <>
          {fallbackSkeleton.segments.map((segment) => (
            <group key={segment.jointName}>
              <mesh position={segment.end}>
                <sphereGeometry args={[0.022, 14, 14]} />
                <meshStandardMaterial color="#d8e2ef" emissive="#516377" emissiveIntensity={0.35} />
              </mesh>
            </group>
          ))}
        </>
      ) : null}
    </group>
  );
}
