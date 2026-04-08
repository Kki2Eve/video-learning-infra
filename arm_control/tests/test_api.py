"""Minimal tests for the arm_control FastAPI backend."""

from __future__ import annotations

import pytest

fastapi = pytest.importorskip("fastapi")
from fastapi.testclient import TestClient

from arm_control.api.app import create_app


def test_health_endpoint_returns_ok() -> None:
    """Health endpoint should return a basic status payload."""
    client = TestClient(create_app())
    response = client.get("/api/health")
    assert response.status_code == 200
    payload = response.json()
    assert payload["status"] == "ok"
    assert "sim_available" in payload


def test_calibration_endpoint_returns_matrices() -> None:
    """Calibration endpoint should expose T_base_cam and R_offset_hand_to_tool."""
    client = TestClient(create_app())
    response = client.get("/api/calibration")
    assert response.status_code == 200
    payload = response.json()
    assert len(payload["T_base_cam"]) == 4
    assert len(payload["R_offset_hand_to_tool"]) == 3


def test_transform_pose_endpoint_returns_pose_base() -> None:
    """Transform endpoint should return a base-frame pose and matrix."""
    client = TestClient(create_app())
    response = client.post(
        "/api/transform_pose",
        json={
            "pose_cam": {
                "position": [0.1, -0.2, 0.3],
                "euler": [0.0, 0.0, 0.0],
            }
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert len(payload["pose_base"]["position"]) == 3
    assert len(payload["pose_base"]["euler"]) == 3
    assert len(payload["pose_base"]["matrix"]) == 4


def test_transform_trajectory_endpoint_returns_trajectory_base() -> None:
    """Trajectory transform endpoint should return one transformed sample per input sample."""
    client = TestClient(create_app())
    response = client.post(
        "/api/transform_trajectory",
        json={
            "trajectory_cam": [
                {
                    "position": [0.1, 0.0, 0.3],
                    "euler": [0.0, 0.0, 0.0],
                    "t": 0.0,
                },
                {
                    "position": [0.2, 0.1, 0.4],
                    "euler": [0.1, -0.1, 0.2],
                    "t": 0.1,
                },
            ]
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert len(payload["trajectory_base"]) == 2
    assert len(payload["trajectory_base"][0]["matrix"]) == 4
    assert payload["trajectory_base"][1]["t"] == 0.1


def test_sample_pose_cam_endpoint_returns_pose() -> None:
    """Sample endpoint should return a valid camera-frame pose payload."""
    client = TestClient(create_app())
    response = client.post("/api/sample_pose_cam")
    assert response.status_code == 200
    payload = response.json()
    assert len(payload["pose_cam"]["position"]) == 3
    assert len(payload["pose_cam"]["matrix"]) == 4


def test_set_target_pose_base_endpoint_stores_target_pose() -> None:
    """Single-pose target endpoint should store a base-frame target for sim polling."""
    client = TestClient(create_app())
    response = client.post(
        "/api/sim/set_target_pose_base",
        json={
            "target_pose_base": {
                "position": [0.2, -0.1, 0.4],
                "euler": [0.0, 0.1, -0.2],
            }
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["target_pose_base"]["position"] == [0.2, -0.1, 0.4]

    state_response = client.get("/api/sim/state")
    assert state_response.status_code == 200
    state_payload = state_response.json()
    assert state_payload["target_pose_base"]["position"] == [0.2, -0.1, 0.4]


def test_set_target_trajectory_base_endpoint_stores_trajectory() -> None:
    """Trajectory target endpoint should report stored length and current step index."""
    client = TestClient(create_app())
    response = client.post(
        "/api/sim/set_target_trajectory_base",
        json={
            "target_trajectory_base": [
                {
                    "position": [0.1, 0.0, 0.3],
                    "euler": [0.0, 0.0, 0.0],
                    "t": 0.0,
                },
                {
                    "position": [0.15, 0.05, 0.35],
                    "euler": [0.1, 0.0, 0.2],
                    "t": 0.1,
                },
            ]
        },
    )
    assert response.status_code == 200
    payload = response.json()
    assert payload["trajectory_length"] == 2
    assert payload["current_step_index"] == 0
    assert payload["target_pose_base"]["position"] == [0.1, 0.0, 0.3]


def test_sim_playback_endpoint_controls_execution_state() -> None:
    """Playback endpoint should accept play, pause, reset, and step commands."""
    client = TestClient(create_app())
    client.post(
        "/api/sim/set_target_trajectory_base",
        json={
            "target_trajectory_base": [
                {
                    "position": [0.1, 0.0, 0.3],
                    "euler": [0.0, 0.0, 0.0],
                    "t": 0.0,
                },
                {
                    "position": [0.15, 0.05, 0.35],
                    "euler": [0.1, 0.0, 0.2],
                    "t": 0.1,
                },
            ]
        },
    )

    play_response = client.post("/api/sim/playback", json={"action": "play"})
    assert play_response.status_code == 200
    assert play_response.json()["playback_status"] == "playing"

    pause_response = client.post("/api/sim/playback", json={"action": "pause"})
    assert pause_response.status_code == 200
    assert pause_response.json()["playback_status"] == "paused"

    step_response = client.post("/api/sim/playback", json={"action": "step"})
    assert step_response.status_code == 200
    assert step_response.json()["trajectory_length"] == 2

    reset_response = client.post("/api/sim/playback", json={"action": "reset"})
    assert reset_response.status_code == 200
    assert reset_response.json()["current_step_index"] == 0


def test_sim_state_endpoint_returns_expected_shape() -> None:
    """Simulator state endpoint should return either live state or a clear fallback."""
    client = TestClient(create_app())
    response = client.get("/api/sim/state")
    assert response.status_code == 200
    payload = response.json()
    assert "sim_connected" in payload
    assert "T_base_cam" in payload
    assert "target_pose_base" in payload
    assert "target_trajectory_base" in payload
    assert "actual_ee_pose" in payload
    assert "actual_ee_trajectory" in payload
    assert "ik_success" in payload
    assert "playback_status" in payload
    assert "robot_urdf_url" in payload
    assert "workspace_min_position" in payload
    assert "reject_count" in payload
    assert "fail_count" in payload

