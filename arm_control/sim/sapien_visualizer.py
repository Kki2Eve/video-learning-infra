"""Viewer wrapper for the minimal SAPIEN single-arm debug scene.

Coordinate conventions:
- The viewer camera pose is expressed in the SAPIEN world frame.
- The world frame is the same frame returned by ``SapienArmController`` for base
  and end-effector poses.
- Frame poses passed to ``draw_frame`` follow ``T_parent_child``. In this file we
  use them as ``T_world_frame``: the child frame pose expressed in the world frame.
- The red, green, and blue axes represent the positive local ``+x``, ``+y``, and
  ``+z`` directions of that child frame after applying the pose rotation.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from arm_control.core.pose_types import Pose6D, Vector3
from arm_control.core.transform_utils import rotation_matrix_to_quaternion
from arm_control.sim.sapien_controller import SapienArmController, SapienCameraConfig

try:
    import sapien
    from sapien.utils import Viewer
except ImportError as exc:  # pragma: no cover - depends on local runtime.
    sapien = None  # type: ignore[assignment]
    Viewer = None  # type: ignore[assignment]
    _SAPIEN_VIEWER_IMPORT_ERROR: ImportError | None = exc
else:
    _SAPIEN_VIEWER_IMPORT_ERROR = None


class SapienVisualizer:
    """Encapsulate the SAPIEN viewer lifecycle for the debug robot scene."""

    def __init__(
        self,
        controller: SapienArmController,
        initial_view_pose: Pose6D | None = None,
    ) -> None:
        """Create a viewer and attach it to the controller's scene.

        The interactive viewer pose is intentionally independent from the fixed
        scene camera pose so that the fixed camera frame remains visible in the
        scene during debugging.
        """
        self._require_viewer()
        self._controller = controller
        self._frame_actors: list[Any] = []
        self._label_actors: list[Any] = []
        self._viewer = Viewer()
        self._viewer.set_scene(controller.scene)
        if initial_view_pose is None:
            initial_view_pose = self.default_debug_view_pose(controller._scene_config.camera)
        self._viewer.set_camera_pose(controller.pose6d_to_sapien_pose(initial_view_pose))
        self._viewer.control_window.show_origin_frame = False
        self._viewer.control_window.toggle_camera_lines(False)
        self._viewer.control_window.move_speed = 0.05

    @property
    def viewer(self) -> Viewer:
        """Return the underlying SAPIEN viewer instance."""
        return self._viewer

    def set_viewer_mode(self) -> None:
        """Move the interactive viewer to a third-person debugging pose."""
        pose = self.default_debug_view_pose(self._controller._scene_config.camera)
        self._viewer.set_camera_pose(self._controller.pose6d_to_sapien_pose(pose))

    def set_camera_mode(self) -> None:
        """Move the interactive viewer onto the fixed scene camera pose."""
        self._viewer.set_camera_pose(self._controller._camera_pose())

    def render_once(self) -> None:
        """Update the render state and draw one viewer frame."""
        self._controller.update_render()
        self._viewer.render()

    def draw_frame(
        self,
        pose: Pose6D,
        axis_length: float = 0.1,
        name: str = "",
        label: str = "",
    ) -> Any:
        """Draw a coordinate frame as three colored local axes.

        ``pose`` is interpreted as ``T_world_frame``. The resulting actor's local
        axes align with the frame being visualized, so the red axis points along
        the frame's local ``+x``, green along local ``+y``, and blue along local
        ``+z`` after the pose rotation is applied in SAPIEN.
        """
        thickness = max(axis_length * 0.04, 0.0025)
        builder = self._controller.scene.create_actor_builder()
        builder.add_box_visual(
            sapien.Pose([axis_length * 0.5, 0.0, 0.0]),
            half_size=np.array([axis_length * 0.5, thickness, thickness]),
            material=self._make_axis_material(color_rgba=[1.0, 0.0, 0.0, 1.0]),
        )
        builder.add_box_visual(
            sapien.Pose([0.0, axis_length * 0.5, 0.0]),
            half_size=np.array([thickness, axis_length * 0.5, thickness]),
            material=self._make_axis_material(color_rgba=[0.0, 1.0, 0.0, 1.0]),
        )
        builder.add_box_visual(
            sapien.Pose([0.0, 0.0, axis_length * 0.5]),
            half_size=np.array([thickness, thickness, axis_length * 0.5]),
            material=self._make_axis_material(color_rgba=[0.0, 0.0, 1.0, 1.0]),
        )
        actor = builder.build_kinematic(name=name or "frame")
        actor.set_pose(self._controller.pose6d_to_sapien_pose(pose))
        self._frame_actors.append(actor)
        if label:
            label_actor = self._build_label_actor(
                pose=pose,
                text=label,
                axis_length=axis_length,
                actor_name=f"{name or 'frame'}_label",
            )
            self._label_actors.append(label_actor)
        return actor

    @staticmethod
    def default_debug_view_pose(camera_config: SapienCameraConfig) -> Pose6D:
        """Return a viewer pose that looks at the robot without sitting on the fixed camera.

        The fixed scene camera may be used for calibration or extrinsics checking,
        so the interactive viewer is moved farther back and slightly upward to keep
        the world, base, and camera frames all in view.
        """
        target = np.asarray(camera_config.target_xyz or (0.0, 0.0, 0.4), dtype=np.float64)
        position = np.asarray(camera_config.position_xyz, dtype=np.float64) + np.array([0.8, -0.8, 0.5])
        up = np.asarray(camera_config.up_xyz or (0.0, 0.0, 1.0), dtype=np.float64)
        return SapienVisualizer.look_at_pose(
            position_xyz=Vector3.from_iterable(position.tolist()),
            target_xyz=Vector3.from_iterable(target.tolist()),
            up_xyz=Vector3.from_iterable(up.tolist()),
        )

    @staticmethod
    def look_at_pose(position_xyz: Vector3, target_xyz: Vector3, up_xyz: Vector3) -> Pose6D:
        """Build a world-frame pose whose local ``+x`` axis looks at the target point."""
        position = position_xyz.to_numpy()
        target = target_xyz.to_numpy()
        up = up_xyz.to_numpy()

        x_axis = target - position
        x_axis /= np.linalg.norm(x_axis)

        z_axis = up - np.dot(up, x_axis) * x_axis
        z_axis /= np.linalg.norm(z_axis)

        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

        rotation = np.column_stack([x_axis, y_axis, z_axis])
        quaternion = rotation_matrix_to_quaternion(rotation)
        return Pose6D(
            position=position_xyz,
            orientation=quaternion,
            frame_id="world",
        )

    def spin(self, physics_steps_per_frame: int = 1) -> None:
        """Keep rendering the window while stepping the scene at a fixed rate."""
        while not self._viewer.closed:
            self._controller.step_simulation(num_steps=physics_steps_per_frame)
            self.render_once()

    def _build_label_actor(
        self,
        pose: Pose6D,
        text: str,
        axis_length: float,
        actor_name: str,
    ) -> Any:
        """Build a simple 3D label near the frame origin.

        Labels are drawn as block letters in the local frame so they move with the
        corresponding coordinate system. This keeps the implementation independent
        from any text-overlay feature of the SAPIEN viewer.
        """
        pixel = axis_length * 0.06
        thickness = max(axis_length * 0.01, 0.0015)
        spacing = pixel * 0.35
        glyph_color = [1.0, 1.0, 1.0, 1.0]
        baseline_z = axis_length * 1.15
        offset_y = axis_length * 0.18

        builder = self._controller.scene.create_actor_builder()
        cursor_x = 0.0
        for character in text.upper():
            pattern = self._glyph_pattern(character)
            if pattern is None:
                cursor_x += pixel * 4.0
                continue
            rows = len(pattern)
            cols = len(pattern[0])
            for row_index, row in enumerate(pattern):
                for col_index, cell in enumerate(row):
                    if cell != "1":
                        continue
                    local_x = cursor_x + col_index * pixel
                    local_z = baseline_z + (rows - 1 - row_index) * pixel
                    builder.add_box_visual(
                        sapien.Pose([local_x, offset_y, local_z]),
                        half_size=np.array([pixel * 0.45, thickness, pixel * 0.45]),
                        material=self._make_axis_material(color_rgba=glyph_color),
                    )
            cursor_x += cols * pixel + spacing

        actor = builder.build_kinematic(name=actor_name)
        actor.set_pose(self._controller.pose6d_to_sapien_pose(pose))
        return actor

    @staticmethod
    def _glyph_pattern(character: str) -> list[str] | None:
        """Return a compact 5x5 block-letter pattern for a supported character."""
        glyphs: dict[str, list[str]] = {
            "A": ["01110", "10001", "11111", "10001", "10001"],
            "B": ["11110", "10001", "11110", "10001", "11110"],
            "C": ["01111", "10000", "10000", "10000", "01111"],
            "D": ["11110", "10001", "10001", "10001", "11110"],
            "E": ["11111", "10000", "11110", "10000", "11111"],
            "L": ["10000", "10000", "10000", "10000", "11111"],
            "M": ["10001", "11011", "10101", "10001", "10001"],
            "O": ["01110", "10001", "10001", "10001", "01110"],
            "R": ["11110", "10001", "11110", "10100", "10010"],
            "S": ["01111", "10000", "01110", "00001", "11110"],
            "W": ["10001", "10001", "10101", "11011", "10001"],
        }
        return glyphs.get(character)

    @staticmethod
    def _make_axis_material(color_rgba: list[float]) -> Any:
        """Create a simple lit material for frame-axis rendering."""
        material = sapien.render.RenderMaterial()
        material.base_color = np.array(color_rgba, dtype=np.float32)
        material.metallic = 0.0
        material.roughness = 0.2
        material.specular = 0.6
        return material

    @staticmethod
    def _require_viewer() -> None:
        """Raise a clear error if the SAPIEN viewer dependency is unavailable."""
        if _SAPIEN_VIEWER_IMPORT_ERROR is not None:
            raise RuntimeError(
                "SAPIEN viewer components are not installed in the active Python environment."
            ) from _SAPIEN_VIEWER_IMPORT_ERROR

