"""FastAPI application for the arm_control backend."""

from __future__ import annotations

from fastapi import FastAPI, HTTPException, Request
from fastapi.staticfiles import StaticFiles

from arm_control.api import schemas, services
from arm_control.api.state import BackendState


def create_app() -> FastAPI:
    """Create the FastAPI application and attach shared backend state."""
    app = FastAPI(title="arm_control API", version="0.1.0")
    app.state.backend = BackendState.create()
    assets_dir = app.state.backend.repo_root / "assets"
    if assets_dir.exists():
        app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")

    @app.get("/api/health", response_model=schemas.HealthResponse)
    def health(request: Request) -> schemas.HealthResponse:
        """Return service health information."""
        return services.get_health_response(request.app.state.backend)

    @app.get("/api/calibration", response_model=schemas.CalibrationResponse)
    def calibration(request: Request) -> schemas.CalibrationResponse:
        """Return the current camera-to-base transform and hand-to-tool rotation offset."""
        return services.get_calibration_response(request.app.state.backend)

    @app.post("/api/transform_pose", response_model=schemas.TransformPoseResponse)
    def transform_pose(
        payload: schemas.TransformPoseRequest,
        request: Request,
    ) -> schemas.TransformPoseResponse:
        """Transform a camera-frame pose into the robot base frame."""
        return services.transform_pose_to_base(request.app.state.backend, payload)

    @app.post("/api/transform_trajectory", response_model=schemas.TransformTrajectoryResponse)
    def transform_trajectory(
        payload: schemas.TransformTrajectoryRequest,
        request: Request,
    ) -> schemas.TransformTrajectoryResponse:
        """Transform a camera-frame trajectory into the robot base frame."""
        return services.transform_trajectory_to_base(request.app.state.backend, payload)

    @app.post("/api/sample_pose_cam", response_model=schemas.SamplePoseCamResponse)
    def sample_pose_cam(request: Request) -> schemas.SamplePoseCamResponse:
        """Return one random legal camera-frame pose sample."""
        return services.sample_pose_cam_response(request.app.state.backend)

    @app.post("/api/sim/set_target_pose_base", response_model=schemas.SetTargetPoseBaseResponse)
    def set_target_pose_base(
        payload: schemas.SetTargetPoseBaseRequest,
        request: Request,
    ) -> schemas.SetTargetPoseBaseResponse:
        """Store one base-frame target pose for simulator-side execution."""
        return services.set_target_pose_base_response(request.app.state.backend, payload)

    @app.post("/api/sim/set_target_trajectory_base", response_model=schemas.SetTargetTrajectoryBaseResponse)
    def set_target_trajectory_base(
        payload: schemas.SetTargetTrajectoryBaseRequest,
        request: Request,
    ) -> schemas.SetTargetTrajectoryBaseResponse:
        """Store one base-frame target trajectory for simulator-side execution."""
        return services.set_target_trajectory_base_response(request.app.state.backend, payload)

    @app.post("/api/sim/playback", response_model=schemas.SimPlaybackControlResponse)
    def control_sim_playback(
        payload: schemas.SimPlaybackControlRequest,
        request: Request,
    ) -> schemas.SimPlaybackControlResponse:
        """Control trajectory playback state for the simulator execution loop."""
        try:
            return services.control_sim_playback_response(request.app.state.backend, payload)
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.get("/api/sim/state", response_model=schemas.SimStateResponse)
    def sim_state(request: Request) -> schemas.SimStateResponse:
        """Return the current simulator joint state, frame poses, and calibration."""
        return services.get_sim_state_response(request.app.state.backend)

    return app


app = create_app()

