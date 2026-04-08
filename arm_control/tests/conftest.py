"""Pytest configuration for local arm_control imports."""

from __future__ import annotations

import sys
from pathlib import Path

for candidate in (Path(__file__).resolve().parent, *Path(__file__).resolve().parents):
    if (candidate / "README.md").exists() and (candidate / "repo_paths.py").exists():
        candidate_str = str(candidate)
        if candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)
        break
else:
    raise RuntimeError(f"Unable to locate repository root from: {__file__}")

from repo_paths import bootstrap_arm_imports

bootstrap_arm_imports(__file__)


