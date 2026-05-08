from __future__ import annotations

from pathlib import Path

from paus_perception import resolve_config_artifact_path, resolve_runtime_data_path


def resolve_ui_calibration_paths(config_path: str | Path, trajectory_path: str, session_root_path: str) -> tuple[Path, Path]:
    return (
        Path(resolve_config_artifact_path(trajectory_path, config_path)),
        Path(resolve_runtime_data_path(session_root_path, config_path)),
    )
