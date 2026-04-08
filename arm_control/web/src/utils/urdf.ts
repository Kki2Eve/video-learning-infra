import { Matrix4, Quaternion, Vector3 } from "three";

export interface UrdfVisual {
  originXyz: [number, number, number];
  originRpy: [number, number, number];
  meshFilename: string | null;
  meshUrl: string | null;
  scale: [number, number, number];
}

export interface UrdfLink {
  name: string;
  visuals: UrdfVisual[];
}

export interface UrdfJoint {
  name: string;
  type: string;
  parent: string;
  child: string;
  originXyz: [number, number, number];
  originRpy: [number, number, number];
  axis: [number, number, number];
}

export interface UrdfModel {
  name: string;
  rootLink: string;
  links: Map<string, UrdfLink>;
  joints: UrdfJoint[];
  childJointsByLink: Map<string, UrdfJoint[]>;
}

export interface RobotSegment {
  jointName: string;
  parentLink: string;
  childLink: string;
  start: [number, number, number];
  end: [number, number, number];
}

export interface RobotSkeleton {
  segments: RobotSegment[];
  jointPositions: [number, number, number][];
}

function parseVector3(value: string | null, fallback: [number, number, number] = [0, 0, 0]) {
  if (!value) {
    return fallback;
  }
  const parts = value.trim().split(/\s+/).map((entry) => Number(entry));
  if (parts.length !== 3 || parts.some((entry) => Number.isNaN(entry))) {
    return fallback;
  }
  return [parts[0], parts[1], parts[2]] as [number, number, number];
}

export function matrixFromOrigin(xyz: [number, number, number], rpy: [number, number, number]) {
  const [roll, pitch, yaw] = rpy;
  const translation = new Matrix4().makeTranslation(xyz[0], xyz[1], xyz[2]);
  const rotationX = new Matrix4().makeRotationX(roll);
  const rotationY = new Matrix4().makeRotationY(pitch);
  const rotationZ = new Matrix4().makeRotationZ(yaw);
  // URDF origin rpy follows roll-pitch-yaw, which corresponds to Rz(yaw) * Ry(pitch) * Rx(roll).
  const rotation = rotationZ.multiply(rotationY).multiply(rotationX);
  return translation.multiply(rotation);
}

export function matrixFromJointMotion(joint: UrdfJoint, qpos: number) {
  if (joint.type === "prismatic") {
    const axis = new Vector3(joint.axis[0], joint.axis[1], joint.axis[2]).normalize();
    return new Matrix4().makeTranslation(axis.x * qpos, axis.y * qpos, axis.z * qpos);
  }
  if (joint.type === "revolute" || joint.type === "continuous") {
    const axis = new Vector3(joint.axis[0], joint.axis[1], joint.axis[2]).normalize();
    const quaternion = new Quaternion().setFromAxisAngle(axis, qpos);
    return new Matrix4().makeRotationFromQuaternion(quaternion);
  }
  return new Matrix4().identity();
}

function positionFromMatrix(matrix: Matrix4) {
  const position = new Vector3();
  position.setFromMatrixPosition(matrix);
  return [position.x, position.y, position.z] as [number, number, number];
}

function resolveRelativeUrl(pathValue: string, baseUrl: string) {
  try {
    const absoluteBaseUrl = new URL(baseUrl, window.location.origin).toString();
    return new URL(pathValue, absoluteBaseUrl).toString();
  } catch {
    return null;
  }
}

function parseLink(linkElement: Element, urdfUrl: string): UrdfLink {
  const visuals = Array.from(linkElement.querySelectorAll(":scope > visual")).map<UrdfVisual>((visualElement) => {
    const origin = visualElement.querySelector("origin");
    const mesh = visualElement.querySelector("geometry > mesh");
    const meshFilename = mesh?.getAttribute("filename") ?? null;
    return {
      originXyz: parseVector3(origin?.getAttribute("xyz")),
      originRpy: parseVector3(origin?.getAttribute("rpy")),
      meshFilename,
      meshUrl: meshFilename ? resolveRelativeUrl(meshFilename, urdfUrl) : null,
      scale: parseVector3(mesh?.getAttribute("scale"), [1, 1, 1]),
    };
  });
  return {
    name: linkElement.getAttribute("name") ?? "unnamed_link",
    visuals,
  };
}

export async function loadUrdfModel(urdfUrl: string): Promise<UrdfModel> {
  const response = await fetch(urdfUrl);
  if (!response.ok) {
    throw new Error(`Failed to load URDF: ${response.status} ${response.statusText}`);
  }
  const text = await response.text();
  const document = new DOMParser().parseFromString(text, "application/xml");
  const robotElement = document.querySelector("robot");
  if (!robotElement) {
    throw new Error("URDF parse error: <robot> element not found.");
  }

  const links = new Map<string, UrdfLink>();
  for (const linkElement of Array.from(document.querySelectorAll("link"))) {
    const link = parseLink(linkElement, urdfUrl);
    links.set(link.name, link);
  }

  const joints = Array.from(document.querySelectorAll("joint")).map<UrdfJoint>((jointElement) => {
    const origin = jointElement.querySelector("origin");
    const parent = jointElement.querySelector("parent");
    const child = jointElement.querySelector("child");
    const axis = jointElement.querySelector("axis");
    return {
      name: jointElement.getAttribute("name") ?? "unnamed_joint",
      type: jointElement.getAttribute("type") ?? "fixed",
      parent: parent?.getAttribute("link") ?? "",
      child: child?.getAttribute("link") ?? "",
      originXyz: parseVector3(origin?.getAttribute("xyz")),
      originRpy: parseVector3(origin?.getAttribute("rpy")),
      axis: parseVector3(axis?.getAttribute("xyz"), [0, 0, 1]),
    };
  });

  const childLinks = new Set(joints.map((joint) => joint.child));
  const parentLinks = new Set(joints.map((joint) => joint.parent));
  const rootLink = Array.from(parentLinks).find((link) => !childLinks.has(link)) ?? "base_link";
  const childJointsByLink = new Map<string, UrdfJoint[]>();
  for (const joint of joints) {
    const siblings = childJointsByLink.get(joint.parent) ?? [];
    siblings.push(joint);
    childJointsByLink.set(joint.parent, siblings);
  }

  return {
    name: robotElement.getAttribute("name") ?? "robot",
    rootLink,
    links,
    joints,
    childJointsByLink,
  };
}

export function computeRobotSkeleton(
  model: UrdfModel,
  qposByJoint: Record<string, number>,
  baseMatrixElements?: number[][],
): RobotSkeleton {
  const baseMatrix = new Matrix4();
  if (baseMatrixElements) {
    baseMatrix.set(
      baseMatrixElements[0][0], baseMatrixElements[0][1], baseMatrixElements[0][2], baseMatrixElements[0][3],
      baseMatrixElements[1][0], baseMatrixElements[1][1], baseMatrixElements[1][2], baseMatrixElements[1][3],
      baseMatrixElements[2][0], baseMatrixElements[2][1], baseMatrixElements[2][2], baseMatrixElements[2][3],
      baseMatrixElements[3][0], baseMatrixElements[3][1], baseMatrixElements[3][2], baseMatrixElements[3][3],
    );
  } else {
    baseMatrix.identity();
  }

  const segments: RobotSegment[] = [];
  const jointPositions: [number, number, number][] = [positionFromMatrix(baseMatrix)];

  const traverse = (parentLink: string, parentMatrix: Matrix4) => {
    const joints = model.childJointsByLink.get(parentLink) ?? [];
    for (const joint of joints) {
      const originMatrix = matrixFromOrigin(joint.originXyz, joint.originRpy);
      const motionMatrix = matrixFromJointMotion(joint, qposByJoint[joint.name] ?? 0.0);
      const childMatrix = parentMatrix.clone().multiply(originMatrix).multiply(motionMatrix);
      const start = positionFromMatrix(parentMatrix);
      const end = positionFromMatrix(childMatrix);

      segments.push({
        jointName: joint.name,
        parentLink: joint.parent,
        childLink: joint.child,
        start,
        end,
      });
      jointPositions.push(end);
      traverse(joint.child, childMatrix);
    }
  };

  traverse(model.rootLink, baseMatrix);
  return { segments, jointPositions };
}
