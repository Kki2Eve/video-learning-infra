"""Stable top-level entrypoint for arm_control commands."""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path


def _bootstrap_imports() -> None:
    """Make the transitional arm_control package importable from the repo root."""
    repo_root = Path(__file__).resolve().parents[2]
    repo_root_str = str(repo_root)
    if repo_root_str not in sys.path:
        sys.path.insert(0, repo_root_str)

    from repo_paths import bootstrap_arm_imports

    bootstrap_arm_imports(repo_root)


COMMANDS: dict[str, tuple[str, str]] = {
    "api": ("arm_control.run_api", "启动 FastAPI 后端"),
    "sim": ("arm_control.run_sim", "运行最小仿真入口"),
    "camera": ("arm_control.examples.camera", "打开 RealSense 预览与内参工具"),
    "calibration-val": ("arm_control.examples.calibration_val", "验证像素+深度到 base 的换算"),
    "debug-realman-terminal": (
        "arm_control.debug.debug_realman_controller_terminal",
        "真机终端调试入口",
    ),
    "debug-pipeline-realman": (
        "arm_control.debug.debug_pipeline_realman_runner",
        "通过 pipeline 驱动真机或 dry-run",
    ),
    "debug-click-to-realman": (
        "arm_control.debug.debug_click_to_realman",
        "点击 RGB-D 画面并执行真机或 dry-run",
    ),
    "debug-sapien-scene": (
        "arm_control.debug.debug_sapien_scene",
        "打开 SAPIEN 机械臂场景调试",
    ),
    "debug-sapien-camera": (
        "arm_control.debug.debug_sapien_camera",
        "验证 SAPIEN 相机外参与可视化",
    ),
}


def _parse_args(argv: list[str]) -> tuple[str, list[str]]:
    """Split the wrapper command from the remaining arguments."""
    parser = argparse.ArgumentParser(
        description="Stable wrapper around arm_control entrypoints.",
    )
    parser.add_argument("command", choices=tuple(COMMANDS))
    parser.add_argument("args", nargs=argparse.REMAINDER)
    namespace = parser.parse_args(argv)
    return str(namespace.command), list(namespace.args)


def main(argv: list[str] | None = None) -> int:
    """Dispatch one stable top-level arm command into the underlying module."""
    _bootstrap_imports()
    command, forwarded_args = _parse_args(sys.argv[1:] if argv is None else argv)
    module_name, _ = COMMANDS[command]
    module = importlib.import_module(module_name)
    sys.argv = [f"scripts/arm/run.py {command}", *forwarded_args]
    result = module.main()
    return int(result) if isinstance(result, int) else 0


if __name__ == "__main__":
    raise SystemExit(main())


