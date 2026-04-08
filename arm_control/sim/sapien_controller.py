"""Minimal SAPIEN single-arm scene controller.

Coordinate conventions:
- ``T_ab`` means a homogeneous transform that maps points from frame ``b`` into
  frame ``a``.
- Poses returned by this controller follow ``T_world_child``. They describe the
  child frame pose expressed in the SAPIEN world frame.
- The base-link pose is ``T_world_base``.
- The end-effector pose is ``T_world_ee``.
- The fixed scene camera pose is ``T_world_cam``.
- Therefore ``T_base_cam = inv(T_world_base) @ T_world_cam``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

try:
    import sapien
    from sapien.asset import create_dome_envmap
except ImportError as exc:  # pragma: no cover - depends on local runtime.
    sapien = None  # type: ignore[assignment]
    create_dome_envmap = None  # type: ignore[assignment]
    _SAPIEN_IMPORT_ERROR: ImportError | None = exc
else:
    _SAPIEN_IMPORT_ERROR = None

from arm_control.core.pose_types import JointState, Pose6D, Quaternion, Vector3
from arm_control.core.transform_utils import invert_transform, pose_to_matrix, rotation_matrix_to_quaternion
from arm_control.paths import find_repo_root
from arm_control.sim.sim_controller import SimController, SimulationState


@dataclass(slots=True)
class SapienCameraConfig:
    """Fixed scene camera configuration in the SAPIEN world frame."""

    width: int = 1280
    height: int = 720
    fovy_rad: float = 1.0
    near: float = 0.05
    far: float = 20.0
    frame_name: str = "camera"
    position_xyz: tuple[float, float, float] = (1.6, -1.2, 1.1)
    target_xyz: tuple[float, float, float] | None = (0.0, 0.0, 0.4)
    up_xyz: tuple[float, float, float] | None = (0.0, 0.0, 1.0)
    quaternion_wxyz: tuple[float, float, float, float] = (
        0.3647052,
        0.1159169,
        0.2798481,
        0.8804762,
    )


@dataclass(slots=True)
class SapienSceneConfig:
    """Renderable scene configuration for the minimal debug environment."""

    timestep_s: float = 1.0 / 240.0
    ground_altitude_m: float = 0.0
    camera: SapienCameraConfig = field(default_factory=SapienCameraConfig)


@dataclass(slots=True)
class SapienRobotConfig:
    """Robot description loaded from ``config/robot.yaml``."""

    name: str
    urdf_path: Path
    base_link: str
    ee_link: str
    controlled_joints: list[str]
    initial_qpos: list[float]
    fix_root_link: bool = True
    disable_gravity: bool = True


@dataclass(slots=True)
class SapienRobotState:
    """Snapshot of the loaded robot state in the world frame."""

    base_pose: Pose6D
    ee_pose: Pose6D
    joint_state: JointState


class SapienArmController(SimController):
    """Own a minimal SAPIEN scene and a single URDF robot articulation."""

    def __init__(
        self,
        robot_config: SapienRobotConfig,
        scene_config: SapienSceneConfig | None = None,
    ) -> None:
        """Initialize the controller with pre-parsed robot and scene configuration."""
        self._robot_config = robot_config
        self._scene_config = scene_config or SapienSceneConfig()
        self._state = SimulationState(connected=False)

        self._scene: sapien.Scene | None = None
        self._camera: Any | None = None
        self._robot: Any | None = None
        self._base_link: Any | None = None
        self._ee_link: Any | None = None

    @classmethod
    def from_yaml(
        cls,
        path: str | Path,
        calibration_path: str | Path | None = None,
    ) -> "SapienArmController":
        """Build a controller from robot and optional calibration YAML files."""
        config_path = Path(path).resolve()
        teleop_root = config_path.parent.parent
        repo_root = find_repo_root(config_path)
        if calibration_path is None:
            calibration_path = teleop_root / "config" / "calibration.yaml"
        calibration_path = Path(calibration_path).resolve()

        with config_path.open("r", encoding="utf-8") as stream:
            raw: dict[str, Any] = yaml.safe_load(stream)
        with calibration_path.open("r", encoding="utf-8") as stream:
            calibration_raw: dict[str, Any] = yaml.safe_load(stream)

        robot_raw = raw["robot"]
        sim_raw = raw.get("sim", {})
        sim_camera_raw = calibration_raw.get("sim_camera", {}).get("pose_world", {})
        robot_camera_raw = sim_raw.get("camera", {})
        urdf_path = cls._resolve_path(
            path_value=str(robot_raw["urdf_path"]),
            repo_root=repo_root,
            teleop_root=teleop_root,
        )
        controlled_joints = [str(name) for name in robot_raw["controlled_joints"]]
        robot_config = SapienRobotConfig(
            name=str(robot_raw["name"]),
            urdf_path=urdf_path,
            base_link=str(robot_raw.get("base_link", "base_link")),
            ee_link=str(robot_raw.get("ee_link", "")),
            controlled_joints=controlled_joints,
            initial_qpos=[float(value) for value in sim_raw.get("initial_qpos", [0.0] * len(controlled_joints))],
            fix_root_link=bool(sim_raw.get("fix_root_link", True)),
            disable_gravity=bool(sim_raw.get("disable_gravity", True)),
        )
        scene_config = SapienSceneConfig(
            timestep_s=float(sim_raw.get("timestep_s", 1.0 / 240.0)),
            camera=SapienCameraConfig(
                width=int(robot_camera_raw.get("width", 1280)),
                height=int(robot_camera_raw.get("height", 720)),
                fovy_rad=float(robot_camera_raw.get("fovy_rad", 1.0)),
                near=float(robot_camera_raw.get("near", 0.05)),
                far=float(robot_camera_raw.get("far", 20.0)),
                frame_name=str(calibration_raw.get("camera_frame", "camera")),
                position_xyz=tuple(float(v) for v in sim_camera_raw.get("position_xyz", [1.6, -1.2, 1.1])),
                target_xyz=(
                    tuple(float(v) for v in sim_camera_raw["target_xyz"])
                    if "target_xyz" in sim_camera_raw
                    else None
                ),
                up_xyz=(
                    tuple(float(v) for v in sim_camera_raw["up_xyz"])
                    if "up_xyz" in sim_camera_raw
                    else None
                ),
                quaternion_wxyz=tuple(
                    float(v) for v in sim_camera_raw.get("quaternion_wxyz", [0.3647052, 0.1159169, 0.2798481, 0.8804762])
                ),
            ),
        )
        return cls(robot_config=robot_config, scene_config=scene_config)

    @staticmethod
    def _resolve_path(path_value: str, repo_root: Path, teleop_root: Path) -> Path:
        """Resolve a possibly relative path from the repo or teleop project root."""
        candidate = Path(path_value)
        if candidate.is_absolute():
            return candidate
        repo_candidate = repo_root / candidate
        if repo_candidate.exists():
            return repo_candidate
        return teleop_root / candidate

    def connect(self) -> None:
        """Create the SAPIEN scene, ground, lights, and debug camera."""
        self._require_sapien()
        if self._scene is not None:
            self._state.connected = True
            return

        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")

        scene = sapien.Scene()
        scene.set_timestep(self._scene_config.timestep_s)
        self._scene = scene
        self._add_ground()
        self._add_lights()
        self._camera = self._create_camera()

        self._state.connected = True
        self._state.metadata["scene_timestep_s"] = self._scene_config.timestep_s

    def disconnect(self) -> None:
        """Release the robot and scene handles owned by the controller."""
        self._robot = None
        self._base_link = None
        self._ee_link = None
        self._camera = None
        self._scene = None
        self._state.connected = False

    def load_scene(self, scene_name: str) -> None:
        """Record the active scene name and ensure the base scene exists."""
        if self._scene is None:
            self.connect()
        self._state.metadata["scene_name"] = scene_name

    def load_robot(self) -> None:
        """Load the configured URDF articulation and resolve base/EE links."""
        self._require_sapien()
        scene = self.scene
        loader = scene.create_urdf_loader()
        loader.load_multiple_collisions_from_file = True
        robot_builder = loader.load_file_as_articulation_builder(str(self._robot_config.urdf_path))
        if robot_builder is None:
            raise RuntimeError(f"Failed to parse URDF into articulation builder: {self._robot_config.urdf_path}")
        robot = robot_builder.build(fix_root_link=self._robot_config.fix_root_link)

        for link in robot.get_links():
            if hasattr(link, "disable_gravity"):
                link.disable_gravity = self._robot_config.disable_gravity

        robot.set_qpos(np.zeros(robot.dof, dtype=np.float64))
        if self._robot_config.initial_qpos:
            self._set_initial_qpos(robot)

        self._robot = robot
        self._base_link = self._resolve_base_link()
        self._ee_link = self._resolve_end_effector_link()
        self._state.metadata["robot_name"] = self._robot_config.name
        self._state.metadata["urdf_path"] = str(self._robot_config.urdf_path)
        self._state.metadata["base_link"] = self._link_name(self._base_link)
        self._state.metadata["ee_link"] = self._link_name(self._ee_link)
        self._state.metadata["fix_root_link"] = self._robot_config.fix_root_link
        self._state.metadata["disable_gravity"] = self._robot_config.disable_gravity

    def send_joint_command(self, joint_state: JointState) -> None:
        """Apply a joint vector directly to the loaded articulation."""
        robot = self.robot
        active_joint_names = self.get_active_joint_names()
        qpos = robot.get_qpos().copy()
        name_to_index = {name: index for index, name in enumerate(active_joint_names)}
        for name, position in zip(joint_state.names, joint_state.positions, strict=False):
            if name in name_to_index:
                qpos[name_to_index[name]] = position
        robot.set_qpos(qpos)
        self._state.last_command = joint_state

    def get_state(self) -> SimulationState:
        """Return generic simulator state metadata for higher-level code."""
        return SimulationState(
            connected=self._state.connected,
            last_command=self._state.last_command,
            metadata=dict(self._state.metadata),
        )

    @property
    def scene(self) -> sapien.Scene:
        """Return the owned SAPIEN scene."""
        if self._scene is None:
            raise RuntimeError("SAPIEN scene has not been created. Call connect() first.")
        return self._scene

    @property
    def camera(self) -> Any:
        """Return the fixed camera entity stored in the scene."""
        if self._camera is None:
            raise RuntimeError("Fixed scene camera has not been created. Call connect() first.")
        return self._camera

    @property
    def robot(self) -> Any:
        """Return the loaded robot articulation."""
        if self._robot is None:
            raise RuntimeError("Robot has not been loaded. Call load_robot() first.")
        return self._robot

    def get_active_joint_names(self) -> list[str]:
        """Return the active joint names in SAPIEN articulation order."""
        return [self._joint_name(joint) for joint in self.robot.get_active_joints()]

    def get_base_pose(self) -> Pose6D:
        """Return the world-frame pose of the configured base link."""
        return self._sapien_pose_to_pose6d(self._link_pose(self._base_link), frame_id="world")

    def get_base_pose_world(self) -> Pose6D:
        """Return ``T_world_base`` as a pose in world coordinates."""
        return self.get_base_pose()

    def get_camera_pose_world(self) -> Pose6D:
        """Return ``T_world_cam`` as a pose in world coordinates."""
        return self._sapien_pose_to_pose6d(self._camera_pose(), frame_id="world")

    def get_end_effector_pose(self) -> Pose6D:
        """Return the world-frame pose of the configured end-effector link."""
        return self._sapien_pose_to_pose6d(self._link_pose(self._ee_link), frame_id="world")

    def get_T_world_base(self) -> np.ndarray:
        """Return ``T_world_base``, which maps base-frame points into the world frame."""
        return pose_to_matrix(self.get_base_pose_world())

    def get_T_world_cam(self) -> np.ndarray:
        """Return ``T_world_cam``, which maps camera-frame points into the world frame."""
        return pose_to_matrix(self.get_camera_pose_world())

    def get_T_base_cam(self) -> np.ndarray:
        """Return ``T_base_cam`` using ``inv(T_world_base) @ T_world_cam``.

        This matrix maps points expressed in the camera frame into the robot base
        frame.
        """
        return invert_transform(self.get_T_world_base()) @ self.get_T_world_cam()

    def get_joint_state(self) -> JointState:
        """Return the configured arm joint positions in controller order."""
        active_joint_names = self.get_active_joint_names()
        active_qpos = self.robot.get_qpos()
        name_to_position = {
            name: float(active_qpos[index])
            for index, name in enumerate(active_joint_names)
        }
        joint_names = [name for name in self._robot_config.controlled_joints if name in name_to_position]
        return JointState(
            names=joint_names,
            positions=[name_to_position[name] for name in joint_names],
        )

    def get_robot_state(self) -> SapienRobotState:
        """Return base pose, end-effector pose, and active-joint state together."""
        return SapienRobotState(
            base_pose=self.get_base_pose(),
            ee_pose=self.get_end_effector_pose(),
            joint_state=self.get_joint_state(),
        )

    def step_simulation(self, num_steps: int = 1) -> None:
        """Advance the physics scene by the requested number of fixed timesteps."""
        scene = self.scene
        for _ in range(num_steps):
            scene.step()

    def update_render(self) -> None:
        """Refresh the render state after physics stepping or robot updates."""
        self.scene.update_render()

    def _add_ground(self) -> None:
        """Add a simple ground plane to the scene."""
        render_material = sapien.render.RenderMaterial()
        render_material.base_color = np.array([0.22, 0.24, 0.28, 1.0])
        render_material.metallic = 0.0
        render_material.roughness = 0.85
        render_material.specular = 0.35
        self.scene.add_ground(
            self._scene_config.ground_altitude_m,
            render_material=render_material,
            render_half_size=[10, 10],
        )

    def _add_lights(self) -> None:
        """Add minimal ambient, dome, and key lighting for arm inspection."""
        scene = self.scene
        scene.set_environment_map(
            create_dome_envmap(
                sky_color=[0.18, 0.20, 0.22],
                ground_color=[0.18, 0.18, 0.18],
            )
        )
        scene.set_ambient_light(np.array([0.35, 0.35, 0.35]))
        scene.add_directional_light(np.array([1.0, -1.0, -1.2]), np.array([2.6, 2.6, 2.6]), shadow=True)
        scene.add_directional_light(np.array([-0.3, 0.5, -1.0]), np.array([1.2, 1.2, 1.2]), shadow=False)
        scene.add_point_light(np.array([1.5, -1.0, 1.8]), np.array([1.4, 1.4, 1.4]), shadow=False)

    def _create_camera(self) -> Any:
        """Create a fixed camera in the world frame.

        The camera pose is configured either by:
        1. ``position_xyz + target_xyz + up_xyz`` for intuitive angle adjustment, or
        2. ``position_xyz + quaternion_wxyz`` as a lower-level fallback.
        """
        config = self._scene_config.camera
        camera = self.scene.add_camera(
            name=config.frame_name,
            width=config.width,
            height=config.height,
            fovy=config.fovy_rad,
            near=config.near,
            far=config.far,
        )
        camera_pose = self._camera_pose_from_config(config)
        camera.set_local_pose(camera_pose)
        return camera

    def _set_initial_qpos(self, robot: Any) -> None:
        """Apply configured joint positions even if the articulation has extra active joints."""
        active_joint_names = [self._joint_name(joint) for joint in robot.get_active_joints()]
        qpos = robot.get_qpos().copy()
        name_to_index = {name: index for index, name in enumerate(active_joint_names)}
        for name, position in zip(
            self._robot_config.controlled_joints,
            self._robot_config.initial_qpos,
            strict=False,
        ):
            if name in name_to_index:
                qpos[name_to_index[name]] = position
        robot.set_qpos(qpos)

    @staticmethod
    def _camera_pose_from_config(config: SapienCameraConfig) -> sapien.Pose:
        """Build the fixed camera pose from either look-at parameters or a quaternion.

        When ``target_xyz`` is provided, we construct a right-handed camera frame
        whose local ``+x`` axis points from the camera position toward the target,
        and whose local ``+z`` axis is as aligned as possible with ``up_xyz``.
        This makes camera angle adjustment much easier than editing quaternions by hand.
        """
        if config.target_xyz is not None:
            quaternion_wxyz = SapienArmController._look_at_quaternion_wxyz(
                position_xyz=config.position_xyz,
                target_xyz=config.target_xyz,
                up_xyz=config.up_xyz or (0.0, 0.0, 1.0),
            )
        else:
            quaternion_wxyz = config.quaternion_wxyz
        return sapien.Pose(list(config.position_xyz), list(quaternion_wxyz))

    @staticmethod
    def _look_at_quaternion_wxyz(
        position_xyz: tuple[float, float, float],
        target_xyz: tuple[float, float, float],
        up_xyz: tuple[float, float, float],
    ) -> tuple[float, float, float, float]:
        """Compute a camera quaternion from a look-at target and an up vector."""
        position = np.asarray(position_xyz, dtype=np.float64)
        target = np.asarray(target_xyz, dtype=np.float64)
        up = np.asarray(up_xyz, dtype=np.float64)

        x_axis = target - position
        x_axis /= np.linalg.norm(x_axis)

        z_axis = up - np.dot(up, x_axis) * x_axis
        z_axis_norm = np.linalg.norm(z_axis)
        if z_axis_norm < 1e-8:
            raise ValueError("Camera up vector is parallel to the viewing direction.")
        z_axis /= z_axis_norm

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rotation = np.column_stack([x_axis, y_axis, z_axis])
        quaternion = rotation_matrix_to_quaternion(rotation)
        return (quaternion.w, quaternion.x, quaternion.y, quaternion.z)

    @staticmethod
    def _require_sapien() -> None:
        """Raise a clear error if SAPIEN is not installed in the active Python environment."""
        if _SAPIEN_IMPORT_ERROR is not None:
            raise RuntimeError(
                "SAPIEN is not installed in the active Python environment. "
                "Install the dependency from arm_control/requirements.txt or the repo pyproject first."
            ) from _SAPIEN_IMPORT_ERROR

    def _resolve_base_link(self) -> Any:
        """Resolve the configured base link, falling back to ``base_link`` or the first link."""
        configured = self._find_link_by_name(self._robot_config.base_link)
        if configured is not None:
            return configured
        default_base = self._find_link_by_name("base_link")
        if default_base is not None:
            return default_base
        return self.robot.get_links()[0]

    def _resolve_end_effector_link(self) -> Any:
        """Resolve the configured end-effector link with safe single-arm fallbacks."""
        configured = self._find_link_by_name(self._robot_config.ee_link)
        if configured is not None:
            return configured

        candidate_names = [
            f"link_{len(self._robot_config.controlled_joints)}",
            "tool0",
            "flange",
            "ee_link",
        ]
        for candidate in candidate_names:
            link = self._find_link_by_name(candidate)
            if link is not None:
                return link
        return self.robot.get_links()[-1]

    def _find_link_by_name(self, link_name: str) -> Any | None:
        """Return a link by name if it exists in the loaded articulation."""
        if not link_name:
            return None
        for link in self.robot.get_links():
            if self._link_name(link) == link_name:
                return link
        return None

    @staticmethod
    def _link_name(link: Any) -> str:
        """Return the string name of a SAPIEN link-like object."""
        if hasattr(link, "get_name"):
            return str(link.get_name())
        return str(getattr(link, "name"))

    @staticmethod
    def _joint_name(joint: Any) -> str:
        """Return the string name of a SAPIEN joint-like object."""
        if hasattr(joint, "get_name"):
            return str(joint.get_name())
        return str(getattr(joint, "name"))

    @staticmethod
    def _link_pose(link: Any) -> sapien.Pose:
        """Return the world pose of a link-like object."""
        if hasattr(link, "get_pose"):
            return link.get_pose()
        return link.pose

    @staticmethod
    def _sapien_pose_to_pose6d(pose: sapien.Pose, frame_id: str) -> Pose6D:
        """Convert a SAPIEN pose into the project's ``Pose6D`` type.

        The returned pose follows the convention ``T_parent_child``. In this debug
        scene, parent is always the SAPIEN world frame, so the pose describes the
        child frame expressed in world coordinates.
        """
        position = np.asarray(pose.p, dtype=np.float64)
        quaternion_wxyz = np.asarray(pose.q, dtype=np.float64)
        return Pose6D(
            position=Vector3.from_iterable(position.tolist()),
            orientation=Quaternion(
                x=float(quaternion_wxyz[1]),
                y=float(quaternion_wxyz[2]),
                z=float(quaternion_wxyz[3]),
                w=float(quaternion_wxyz[0]),
            ).normalized(),
            frame_id=frame_id,
        )

    def _camera_pose(self) -> sapien.Pose:
        """Return the fixed camera pose as ``T_world_cam``."""
        if hasattr(self.camera, "get_local_pose"):
            return self.camera.get_local_pose()
        return self.camera.pose

    @staticmethod
    def pose6d_to_sapien_pose(pose: Pose6D) -> sapien.Pose:
        """Convert ``Pose6D`` into a SAPIEN pose using the same ``T_parent_child`` convention.

        ``pose`` is interpreted as the child frame pose expressed in its parent
        frame. When used by the debug visualizer, parent is the SAPIEN world frame.
        """
        quaternion = pose.orientation.normalized()
        return sapien.Pose(
            list(pose.position.to_numpy()),
            [quaternion.w, quaternion.x, quaternion.y, quaternion.z],
        )

