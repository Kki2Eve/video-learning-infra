"""Calibration loading and camera-to-robot pose conversion."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import NDArray

from arm_control.core.pose_types import Pose6D, Vector3
from arm_control.core.transform_utils import transform_pose


@dataclass(slots=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics used for deprojection."""

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion_model: str = "unknown"
    coeffs: tuple[float, ...] = ()

    @classmethod
    def from_mapping(cls, raw: dict[str, Any]) -> "CameraIntrinsics":
        """Build intrinsics from a YAML mapping."""
        return cls(
            width=int(raw["width"]),
            height=int(raw["height"]),
            fx=float(raw["fx"]),
            fy=float(raw["fy"]),
            cx=float(raw["cx"]),
            cy=float(raw["cy"]),
            distortion_model=str(raw.get("distortion_model", "unknown")),
            coeffs=tuple(float(value) for value in raw.get("coeffs", [])),
        )

    def to_dict(self) -> dict[str, Any]:
        """Return a YAML-friendly representation."""
        return {
            "width": int(self.width),
            "height": int(self.height),
            "fx": float(self.fx),
            "fy": float(self.fy),
            "cx": float(self.cx),
            "cy": float(self.cy),
            "distortion_model": self.distortion_model,
            "coeffs": [float(value) for value in self.coeffs],
        }


@dataclass(slots=True)
class CalibrationConfig:
    """Extrinsic calibration from a camera frame to the robot base frame."""

    camera_frame: str
    robot_base_frame: str
    camera_to_robot_base: NDArray[np.float64]
    camera_intrinsics: CameraIntrinsics | None = None


class CalibrationManager:
    """Own the calibration transform and expose frame conversion helpers."""

    def __init__(self, config: CalibrationConfig) -> None:
        """Initialize the manager from an already-parsed config."""
        self._config = config

    @classmethod
    def from_yaml(cls, path: str | Path) -> "CalibrationManager":
        """Load calibration parameters from a YAML file."""
        with Path(path).open("r", encoding="utf-8") as stream:
            raw: dict[str, Any] = yaml.safe_load(stream)
        matrix = np.array(raw["camera_to_robot_base"], dtype=np.float64)
        config = CalibrationConfig(
            camera_frame=str(raw["camera_frame"]),
            robot_base_frame=str(raw["robot_base_frame"]),
            camera_to_robot_base=matrix,
            camera_intrinsics=(
                CameraIntrinsics.from_mapping(raw["camera_intrinsics"])
                if "camera_intrinsics" in raw and raw["camera_intrinsics"] is not None
                else None
            ),
        )
        return cls(config=config)

    def get_camera_to_robot_base(self) -> NDArray[np.float64]:
        """Return the 4x4 extrinsic transform from camera frame to robot base."""
        return self._config.camera_to_robot_base.copy()

    def get_camera_intrinsics(self) -> CameraIntrinsics:
        """Return the configured camera intrinsics."""
        if self._config.camera_intrinsics is None:
            raise ValueError("Camera intrinsics are not configured in calibration.yaml.")
        return self._config.camera_intrinsics

    def deproject_pixel_to_camera(self, u: float, v: float, depth_m: float) -> Vector3:
        """Project a pixel with depth into the camera coordinate frame."""
        intrinsics = self.get_camera_intrinsics()
        depth = float(depth_m)
        x = (float(u) - intrinsics.cx) * depth / intrinsics.fx
        y = (float(v) - intrinsics.cy) * depth / intrinsics.fy
        return Vector3(x=x, y=y, z=depth)

    def transform_point_to_robot_base(self, point_cam: Vector3) -> Vector3:
        """Transform a 3D point from the camera frame into robot base coordinates."""
        point_h = np.array([point_cam.x, point_cam.y, point_cam.z, 1.0], dtype=np.float64)
        point_base = self._config.camera_to_robot_base @ point_h
        return Vector3.from_iterable(point_base[:3].tolist())

    def transform_pose_to_robot_base(self, pose_cam: Pose6D) -> Pose6D:
        """Transform a pose from the camera frame into the robot base frame."""
        return transform_pose(
            pose=pose_cam,
            transform=self._config.camera_to_robot_base,
            target_frame=self._config.robot_base_frame,
        )

    @property
    def camera_frame(self) -> str:
        """Return the configured camera frame name."""
        return self._config.camera_frame

    @property
    def robot_base_frame(self) -> str:
        """Return the configured robot base frame name."""
        return self._config.robot_base_frame

