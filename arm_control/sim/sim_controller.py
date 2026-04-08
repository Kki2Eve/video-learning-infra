"""Simulator control abstraction for robot-arm teleoperation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol

from arm_control.core.pose_types import JointState


@dataclass(slots=True)
class SimulationState:
    """Snapshot of the simulator-facing robot state."""

    connected: bool
    last_command: JointState | None = None
    metadata: dict[str, Any] = field(default_factory=dict)


class SimController(Protocol):
    """Protocol for simulator adapters that consume joint commands."""

    def connect(self) -> None:
        """Connect to the simulator backend."""

    def disconnect(self) -> None:
        """Disconnect from the simulator backend."""

    def load_scene(self, scene_name: str) -> None:
        """Load or reset a simulator scene."""

    def send_joint_command(self, joint_state: JointState) -> None:
        """Send one joint-space command to the simulator."""

    def get_state(self) -> SimulationState:
        """Return the current simulator bridge state."""


class DummySimController:
    """Minimal simulator adapter used for bring-up and unit testing."""

    def __init__(self) -> None:
        """Initialize an in-memory simulator state."""
        self._state = SimulationState(connected=False)

    def connect(self) -> None:
        """Mark the simulator bridge as connected."""
        self._state.connected = True

    def disconnect(self) -> None:
        """Mark the simulator bridge as disconnected."""
        self._state.connected = False

    def load_scene(self, scene_name: str) -> None:
        """Record the requested scene name."""
        self._state.metadata["scene_name"] = scene_name

    def send_joint_command(self, joint_state: JointState) -> None:
        """Store the last command for later inspection."""
        self._state.last_command = joint_state

    def get_state(self) -> SimulationState:
        """Return a copy-like view of the current in-memory simulator state."""
        return SimulationState(
            connected=self._state.connected,
            last_command=self._state.last_command,
            metadata=dict(self._state.metadata),
        )


