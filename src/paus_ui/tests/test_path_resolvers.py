from __future__ import annotations

import sys
from pathlib import Path

import pytest

PROJECT_ROOT = Path(__file__).resolve().parents[3]
UI_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_ui"
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
for package_root in (UI_PACKAGE_ROOT, PERCEPTION_PACKAGE_ROOT):
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

from paus_ui.path_resolvers import resolve_ui_calibration_paths


def test_ui_calibration_paths_match_backend_runtime_resolvers(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    runtime_dir = tmp_path / "runtime"
    config_path = tmp_path / "install" / "paus_bringup" / "share" / "paus_bringup" / "configs" / "default.yaml"
    monkeypatch.setenv("PAUS_ROBOT_RUNTIME_DIR", str(runtime_dir))

    trajectory_path, session_root_path = resolve_ui_calibration_paths(
        config_path,
        "eye_to_hand_trajectory.yaml",
        "calibration_sessions",
    )

    assert trajectory_path == runtime_dir / "configs" / "eye_to_hand_trajectory.yaml"
    assert session_root_path == runtime_dir / "calibration_sessions"
