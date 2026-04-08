"""RealMan hardware controller wrapper for the teleoperation backend.

This module keeps the project's internal pose/joint types while matching the
existing RealMan SDK style used in the example scripts.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from robotic_arm_package.rm_robot_interface import RoboticArm, rm_thread_mode_e
from arm_control.core.pose_types import JointState, Pose6D, Vector3
from arm_control.core.transform_utils import euler_rpy_to_quaternion, quaternion_to_euler_rpy

logger = logging.getLogger(__name__)


class RealManControllerError(RuntimeError):
    """Raised when the RealMan SDK returns an error or the controller is misused."""


@dataclass(slots=True)
class RealManControllerConfig:
    """Configuration used to initialize a :class:`RealManController`."""

    ip: str
    port: int
    work_frame: str = "World"
    self_collision_enable: bool = True
    default_speed: int = 100
    default_block: int = 1
    default_blend_radius: int = 0
    default_connect: int = 0
    default_joint_move_t: int = 0
    current_pose_frame_id: str = "World"
    sdk_log_level: int = 3
    controlled_joints: list[str] = field(default_factory=list)
    home_joints_deg: list[float] | None = None
    top_view_joints_deg: list[float] | None = None

    @classmethod
    def from_mapping(cls, config: Mapping[str, Any]) -> "RealManControllerConfig":
        """Build controller config from the full robot config or a RealMan subsection."""
        realman_section = config.get("realman") if "realman" in config else config
        if not isinstance(realman_section, Mapping):
            raise RealManControllerError("Expected a mapping for RealMan configuration.")

        robot_section = config.get("robot", {}) if "robot" in config else {}
        controlled_joints = list(robot_section.get("controlled_joints", []))

        if "ip" not in realman_section or "port" not in realman_section:
            raise RealManControllerError("RealMan configuration must define both 'ip' and 'port'.")

        return cls(
            ip=str(realman_section["ip"]),
            port=int(realman_section["port"]),
            work_frame=str(realman_section.get("work_frame", "World")),
            self_collision_enable=bool(realman_section.get("self_collision_enable", True)),
            default_speed=int(realman_section.get("default_speed", 20)),
            default_block=int(realman_section.get("default_block", 1)),
            default_blend_radius=int(realman_section.get("default_blend_radius", 0)),
            default_connect=int(realman_section.get("default_connect", 0)),
            default_joint_move_t=int(realman_section.get("default_joint_move_t", 0)),
            current_pose_frame_id=str(realman_section.get("current_pose_frame_id", realman_section.get("work_frame", "World"))),
            sdk_log_level=int(realman_section.get("sdk_log_level", 3)),
            controlled_joints=controlled_joints,
            home_joints_deg=_optional_float_list(realman_section.get("home_joints_deg")),
            top_view_joints_deg=_optional_float_list(realman_section.get("top_view_joints_deg")),
        )


def load_robot_yaml_config(config_path: str | Path) -> dict[str, Any]:
    """Load the project robot YAML file into a plain dictionary."""
    path = Path(config_path).expanduser().resolve()
    with path.open("r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    if not isinstance(loaded, dict):
        raise RealManControllerError(f"Robot config file must contain a YAML mapping: {path}")
    return loaded


class RealManController:
    """Thin RealMan SDK wrapper using project-internal pose and joint types.

    Internal conventions:
    - Cartesian position is expressed in meters.
    - Orientation is expressed in radians.
    - Project-internal poses are represented as :class:`Pose6D` with quaternion
      orientation in ``(x, y, z, w)`` order.
    - Before sending a pose to the RealMan SDK it is converted into
      ``[x, y, z, rx, ry, rz]`` in meters and radians.
    - Joint states are exposed by this wrapper in radians, while the SDK joint
      motion API ``rm_movej`` expects degrees. The conversion happens here.
    """

    def __init__(self, config: Mapping[str, Any] | RealManControllerConfig) -> None:
        """Create a controller instance without connecting to hardware yet."""
        self.config = config if isinstance(config, RealManControllerConfig) else RealManControllerConfig.from_mapping(config)
        self._arm: RoboticArm | None = None
        self._handle_id: int | None = None

    @classmethod
    def from_robot_yaml(cls, config_path: str | Path) -> "RealManController":
        """Create a controller from ``config/robot.yaml``."""
        return cls(load_robot_yaml_config(config_path))

    @property
    def is_connected(self) -> bool:
        """Return whether the hardware connection has been established."""
        return self._arm is not None and self._handle_id is not None

    def connect(self) -> None:
        """Connect to the RealMan controller and apply default runtime settings."""
        if self.is_connected:
            logger.info("RealMan controller already connected to %s:%s.", self.config.ip, self.config.port)
            return

        logger.info("Connecting to RealMan controller at %s:%s...", self.config.ip, self.config.port)
        arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
        handle = arm.rm_create_robot_arm(self.config.ip, self.config.port, level=self.config.sdk_log_level)
        handle_id = int(getattr(handle, "id", -1))
        if handle_id < 0:
            raise RealManControllerError(
                f"Failed to connect to RealMan controller at {self.config.ip}:{self.config.port}."
            )

        self._arm = arm
        self._handle_id = handle_id
        logger.info("RealMan controller connected with handle id=%s.", handle_id)

        try:
            self._check_sdk_status(
                self._arm.rm_change_work_frame(self.config.work_frame),
                f"change work frame to {self.config.work_frame}",
            )
            self._check_sdk_status(
                self._arm.rm_set_self_collision_enable(self.config.self_collision_enable),
                f"set self collision enable to {self.config.self_collision_enable}",
            )
        except Exception:
            try:
                self.disconnect()
            finally:
                raise

    def disconnect(self) -> None:
        """Disconnect from the RealMan controller."""
        if self._arm is None:
            return

        logger.info("Disconnecting RealMan controller handle id=%s...", self._handle_id)
        status = self._arm.rm_delete_robot_arm()
        self._arm = None
        self._handle_id = None
        if status != 0:
            raise RealManControllerError(f"Failed to disconnect RealMan controller: {self._format_sdk_error(status)}")
        logger.info("RealMan controller disconnected.")

    def get_current_pose(self) -> Pose6D:
        """Return the current end-effector pose as a project-internal ``Pose6D``."""
        arm = self._require_connected()
        status, state = arm.rm_get_current_arm_state()
        self._check_sdk_status(status, "read current arm state")
        pose_values = state["pose"]
        return self._rm_pose_to_pose6d(pose_values, frame_id=self.config.current_pose_frame_id)

    def get_current_pose_xyzrpy(self) -> list[float]:
        """Return the current end-effector pose as ``[x, y, z, rx, ry, rz]``.

        This matches the common RealMan SDK convention:
        - position is in meters
        - orientation is Euler roll-pitch-yaw in radians

        The method is intentionally provided in addition to ``get_current_pose()``
        so that the project can keep using ``Pose6D`` internally without breaking
        existing pipeline and simulator code.
        """
        arm = self._require_connected()
        status, state = arm.rm_get_current_arm_state()
        self._check_sdk_status(status, "read current arm state")
        pose_values = state["pose"]
        if len(pose_values) != 6:
            raise RealManControllerError(f"Expected 6 pose values from SDK, got {len(pose_values)}.")
        return [float(value) for value in pose_values]

    def get_current_joints(self) -> JointState:
        """Return the current joint positions as radians in a ``JointState``."""
        arm = self._require_connected()
        status, joints_deg = arm.rm_get_joint_degree()
        self._check_sdk_status(status, "read current joint degrees")
        positions = [math.radians(float(angle_deg)) for angle_deg in joints_deg]
        names = self._joint_names_for_count(len(positions))
        return JointState(names=names, positions=positions)

    def move_to_pose_linear(
        self,
        pose: Pose6D,
        v: int | None = None,
        r: int | None = None,
        connect: int | None = None,
        block: int | None = None,
    ) -> int:
        """Move linearly in Cartesian space using ``rm_movel``.

        The input pose is first converted to the RealMan SDK pose list
        ``[x, y, z, rx, ry, rz]`` using meters and radians.
        """
        arm = self._require_connected()
        status = arm.rm_movel(
            self._pose6d_to_rm_pose(pose),
            int(self.config.default_speed if v is None else v),
            int(self.config.default_blend_radius if r is None else r),
            int(self.config.default_connect if connect is None else connect),
            int(self.config.default_block if block is None else block),
        )
        self._check_sdk_status(status, "execute rm_movel")
        return status

    def move_to_pose_joint(
        self,
        pose: Pose6D,
        v: int | None = None,
        r: int | None = None,
        connect: int | None = None,
        block: int | None = None,
    ) -> int:
        """Move in joint space to a Cartesian target using ``rm_movej_p``."""
        arm = self._require_connected()
        status = arm.rm_movej_p(
            self._pose6d_to_rm_pose(pose),
            int(self.config.default_speed if v is None else v),
            int(self.config.default_blend_radius if r is None else r),
            int(self.config.default_connect if connect is None else connect),
            int(self.config.default_block if block is None else block),
        )
        self._check_sdk_status(status, "execute rm_movej_p")
        return status

    def move_to_joints(
        self,
        joints: Sequence[float],
        v: int | None = None,
        t: int | None = None,
        connect: int | None = None,
        block: int | None = None,
    ) -> int:
        """Move to joint targets using ``rm_movej``.

        Args:
            joints: Joint angles in radians. They are converted to degrees before
                calling the RealMan SDK.
            v: Speed percentage passed to the SDK.
            t: Compatibility parameter matching the existing example code. It is
                forwarded to the third ``rm_movej`` argument.
            connect: Trajectory connection flag.
            block: Blocking flag.
        """
        arm = self._require_connected()
        joint_degrees = [math.degrees(float(joint)) for joint in joints]
        status = arm.rm_movej(
            joint_degrees,
            int(self.config.default_speed if v is None else v),
            int(self.config.default_joint_move_t if t is None else t),
            int(self.config.default_connect if connect is None else connect),
            int(self.config.default_block if block is None else block),
        )
        self._check_sdk_status(status, "execute rm_movej")
        return status

    def stop(self) -> int:
        """Stop the current RealMan motion."""
        arm = self._require_connected()
        status = arm.rm_set_arm_stop()
        self._check_sdk_status(status, "stop arm motion")
        return status

    def go_home(
        self,
        v: int | None = None,
        t: int | None = None,
        connect: int | None = None,
        block: int | None = None,
    ) -> int:
        """Move to the configured home joint set, if available."""
        if not self.config.home_joints_deg:
            raise RealManControllerError("No 'home_joints_deg' configured under the 'realman' section.")
        return self.move_to_joints(
            [math.radians(value) for value in self.config.home_joints_deg],
            v=v,
            t=t,
            connect=connect,
            block=block,
        )

    def move_to_top_view(
        self,
        v: int | None = None,
        t: int | None = None,
        connect: int | None = None,
        block: int | None = None,
    ) -> int:
        """Move to the configured top-view joint set, if available."""
        if not self.config.top_view_joints_deg:
            raise RealManControllerError("No 'top_view_joints_deg' configured under the 'realman' section.")
        return self.move_to_joints(
            [math.radians(value) for value in self.config.top_view_joints_deg],
            v=v,
            t=t,
            connect=connect,
            block=block,
        )

    def _require_connected(self) -> RoboticArm:
        """Return the connected SDK object or raise a clear error."""
        if self._arm is None:
            raise RealManControllerError("RealMan controller is not connected. Call connect() first.")
        return self._arm

    def _check_sdk_status(self, status: int, action: str) -> None:
        """Raise a controller error when the SDK reports failure."""
        if status != 0:
            raise RealManControllerError(f"Failed to {action}: {self._format_sdk_error(status)}")

    def _joint_names_for_count(self, count: int) -> list[str]:
        """Return configured joint names or a stable fallback list."""
        configured = self.config.controlled_joints[:count]
        if len(configured) == count:
            return configured
        return [f"joint_{index + 1}" for index in range(count)]

    def _pose6d_to_rm_pose(self, pose: Pose6D) -> list[float]:
        """Convert project pose into SDK pose ``[x, y, z, rx, ry, rz]``."""
        euler = quaternion_to_euler_rpy(pose.orientation)
        return [
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            float(euler[0]),
            float(euler[1]),
            float(euler[2]),
        ]

    def _rm_pose_to_pose6d(self, pose_values: Sequence[float], frame_id: str) -> Pose6D:
        """Convert SDK pose list ``[x, y, z, rx, ry, rz]`` into ``Pose6D``."""
        if len(pose_values) != 6:
            raise RealManControllerError(f"Expected 6 pose values from SDK, got {len(pose_values)}.")
        return Pose6D(
            position=Vector3(
                x=float(pose_values[0]),
                y=float(pose_values[1]),
                z=float(pose_values[2]),
            ),
            orientation=euler_rpy_to_quaternion(
                [float(pose_values[3]), float(pose_values[4]), float(pose_values[5])]
            ),
            frame_id=frame_id,
        )

    @staticmethod
    def _format_sdk_error(status: int) -> str:
        """Return a short human-readable description for common SDK status codes."""
        descriptions = {
            0: "success",
            1: "controller returned failure or the arm is in an invalid state",
            -1: "data send failure",
            -2: "data receive failure or controller timeout",
            -3: "response parse failure",
            -4: "arrival device validation failure",
            -5: "single-thread timeout waiting for completion",
        }
        description = descriptions.get(status, "unknown SDK error")
        return f"status={status} ({description})"


def _optional_float_list(values: Any) -> list[float] | None:
    """Convert an optional iterable of numbers into a plain float list."""
    if values is None:
        return None
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)):
        raise RealManControllerError("Expected a numeric list in RealMan configuration.")
    return [float(value) for value in values]

