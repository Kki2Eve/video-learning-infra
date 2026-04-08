"""Local entry point for the arm_control FastAPI backend."""

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


def main() -> None:
    """Start the FastAPI backend with Uvicorn."""
    try:
        import uvicorn
    except ImportError as exc:  # pragma: no cover - depends on local runtime.
        raise RuntimeError(
            "Uvicorn is not installed in the active Python environment. "
            "Install dependencies from arm_control/requirements.txt first."
        ) from exc

    uvicorn.run(
        "arm_control.api.app:app",
        host="127.0.0.1",
        port=8007,
        reload=False,
    )


if __name__ == "__main__":
    main()


