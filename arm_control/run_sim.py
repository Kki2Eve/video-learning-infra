"""Entry point for the single-arm teleoperation simulator backend skeleton."""

from __future__ import annotations

from pathlib import Path
import sys


def _bootstrap_arm_imports() -> None:
    """Expose repo-level helpers before importing the arm package directly."""
    current = Path(__file__).resolve()
    for candidate in (current.parent, *current.parents):
        if (candidate / "README.md").exists() and (candidate / "repo_paths.py").exists():
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)
            from repo_paths import bootstrap_arm_imports

            bootstrap_arm_imports(current)
            return
    raise RuntimeError(f"Unable to locate repository root from: {current}")


if __package__ in (None, ""):
    _bootstrap_arm_imports()

from arm_control.core.calibration import CalibrationManager
from arm_control.core.ik_solver import IKSolver
from arm_control.core.pipeline import PipelineInput, TeleopPipeline
from arm_control.core.pose_filter import build_pose_filter
from arm_control.core.pose_mapper import PoseMapper
from arm_control.core.pose_types import Pose6D, Quaternion, Vector3
from arm_control.core.safety_guard import SafetyGuard
from arm_control.paths import config_path
from arm_control.sim.sim_controller import DummySimController
from arm_control.sim.test_scenes import default_scene
from arm_control.sim.visualizer import PipelineVisualizer


def build_pipeline() -> TeleopPipeline:
    """Build the default teleoperation pipeline from YAML configuration."""
    calibration = CalibrationManager.from_yaml(config_path("calibration.yaml"))
    mapper = PoseMapper.from_yaml(config_path("robot.yaml"))
    pose_filter = build_pose_filter(config_path("robot.yaml"))
    safety_guard = SafetyGuard.from_yaml(
        workspace_path=config_path("workspace.yaml"),
        robot_path=config_path("robot.yaml"),
    )
    ik_solver = IKSolver.from_yaml(config_path("robot.yaml"))
    return TeleopPipeline(
        calibration=calibration,
        mapper=mapper,
        pose_filter=pose_filter,
        safety_guard=safety_guard,
        ik_solver=ik_solver,
    )


def example_input_pose() -> Pose6D:
    """Return a sample camera-frame pose for simulator bring-up."""
    return Pose6D(
        position=Vector3(x=0.20, y=-0.10, z=0.55),
        orientation=Quaternion.identity(),
        frame_id="camera_color_optical_frame",
    )


def main() -> None:
    """Run one skeleton teleoperation step and forward the command to the simulator."""
    pipeline = build_pipeline()
    sim = DummySimController()
    visualizer = PipelineVisualizer()

    sim.connect()
    sim.load_scene(default_scene().name)
    output = pipeline.step(PipelineInput(pose_cam=example_input_pose()))
    sim.send_joint_command(output.ik_result.joint_state)
    print(visualizer.summarize_step(output))
    sim.disconnect()


if __name__ == "__main__":
    main()


