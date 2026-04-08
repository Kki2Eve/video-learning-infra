"""Stable top-level entrypoint for hand_control commands."""

from __future__ import annotations

import argparse
import os
import runpy
import sys
from pathlib import Path


def _repo_root() -> Path:
    """Return the repository root from the script location."""
    return Path(__file__).resolve().parents[2]


def _hand_root() -> Path:
    """Return the hand_control project root."""
    return _repo_root() / "hand_control"


COMMANDS: dict[str, tuple[Path, str]] = {
    "controller": (
        _hand_root() / "example" / "revo2_retargeting" / "brainco_controller.py",
        "启动手侧控制入口，可用于 local/http 模式。",
    ),
    "stream": (
        _hand_root() / "example" / "revo2_retargeting" / "teleop.py",
        "启动手侧检测/重定向数据流入口。",
    ),
}


def _parse_args(argv: list[str]) -> tuple[str, list[str]]:
    """Split the wrapper command from the remaining arguments."""
    parser = argparse.ArgumentParser(
        description="Stable wrapper around hand_control entrypoints.",
    )
    parser.add_argument("command", choices=tuple(COMMANDS))
    parser.add_argument("args", nargs=argparse.REMAINDER)
    namespace = parser.parse_args(argv)
    return str(namespace.command), list(namespace.args)


def main(argv: list[str] | None = None) -> int:
    """Dispatch one stable top-level hand command into the underlying script."""
    repo_root = _repo_root()
    repo_root_str = str(repo_root)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)

    command, forwarded_args = _parse_args(sys.argv[1:] if argv is None else argv)
    target_script, _ = COMMANDS[command]
    previous_cwd = Path.cwd()
    try:
        os.chdir(_hand_root())
        sys.argv = [f"scripts/hand/run.py {command}", *forwarded_args]
        runpy.run_path(str(target_script), run_name="__main__")
    finally:
        os.chdir(previous_cwd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
