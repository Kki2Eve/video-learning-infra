"""Compatibility wrapper around the repo-level path helpers."""

from __future__ import annotations

from repo_paths import (
    ARM_CONTROL_ROOT as PROJECT_ROOT,
    REPO_ROOT,
    THIRD_PARTY_ROOT,
    arm_config_path,
    arm_examples_path,
    find_repo_root,
    ensure_on_sys_path,
)


def ensure_third_party_on_sys_path():
    """Keep the historical arm_control.paths API working for now."""
    if THIRD_PARTY_ROOT.exists():
        ensure_on_sys_path(THIRD_PARTY_ROOT)
    return THIRD_PARTY_ROOT


def config_path(filename: str):
    """Return an arm configuration file path."""
    return arm_config_path(filename)


def examples_path(filename: str):
    """Return an arm example payload path."""
    return arm_examples_path(filename)

