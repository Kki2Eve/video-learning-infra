"""Scene definitions used by simulator adapters during development."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class SceneDefinition:
    """Description of a simulator scene or world preset."""

    name: str
    description: str
    assets: list[str] = field(default_factory=list)


def default_scene() -> SceneDefinition:
    """Return the default bring-up scene for single-arm teleoperation."""
    return SceneDefinition(
        name="empty_workspace",
        description="Robot arm in a collision-light workspace for initial backend integration.",
        assets=[],
    )


def available_scenes() -> list[SceneDefinition]:
    """Return all scene presets exposed by the skeleton project."""
    return [default_scene()]

