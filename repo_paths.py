"""Repository path helpers for the current monorepo layout."""

from __future__ import annotations

from pathlib import Path
import sys

_REPO_MARKERS = ("README.md", "repo_paths.py")
REPO_ROOT = Path(__file__).resolve().parent
ARM_CONTROL_ROOT = REPO_ROOT / "arm_control"
HAND_CONTROL_ROOT = REPO_ROOT / "hand_control"
ASSETS_ROOT = REPO_ROOT / "assets"
THIRD_PARTY_ROOT = REPO_ROOT / "third_party"


def ensure_on_sys_path(path: str | Path) -> Path:
    """Insert a path at the front of ``sys.path`` when it is not present yet."""
    resolved = Path(path).resolve()
    resolved_str = str(resolved)
    if resolved_str not in sys.path:
        sys.path.insert(0, resolved_str)
    return resolved


def find_repo_root(start: str | Path) -> Path:
    """Locate the repository root by walking upward from a file or directory."""
    current = Path(start).resolve()
    search_root = current if current.is_dir() else current.parent
    for candidate in (search_root, *search_root.parents):
        if all((candidate / marker).exists() for marker in _REPO_MARKERS):
            return candidate
    raise RuntimeError(f"Unable to locate repository root from: {current}")


def third_party_root(repo_root: str | Path) -> Path:
    """Return the shared third-party directory under the repository root."""
    return Path(repo_root).resolve() / "third_party"


def arm_project_root(repo_root: str | Path) -> Path:
    """Return the current arm project root under the repository root."""
    root = Path(repo_root).resolve()
    candidates = (root / "arm_control",)
    for candidate in candidates:
        if candidate.is_dir():
            return candidate
    raise RuntimeError("Unable to locate arm_control under the repository root.")


def arm_import_root(repo_root: str | Path) -> Path:
    """Return the directory that should be added to ``sys.path`` for arm imports."""
    return Path(repo_root).resolve()


def arm_config_path(filename: str) -> Path:
    """Return a configuration file path under the arm project root."""
    return ARM_CONTROL_ROOT / "config" / filename


def arm_examples_path(filename: str) -> Path:
    """Return an example payload path under the arm project root."""
    return ARM_CONTROL_ROOT / "examples" / filename


def bootstrap_arm_imports(start: str | Path) -> tuple[Path, Path]:
    """Ensure repo, arm import root, and shared third-party paths are importable."""
    repo_root = ensure_on_sys_path(find_repo_root(start))
    ensure_on_sys_path(arm_import_root(repo_root))
    if THIRD_PARTY_ROOT.exists():
        ensure_on_sys_path(THIRD_PARTY_ROOT)
    return repo_root, arm_project_root(repo_root)



