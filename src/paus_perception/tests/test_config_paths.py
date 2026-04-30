from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
if str(PERCEPTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_PACKAGE_ROOT))

from paus_perception.config import resolve_config_artifact_path, resolve_runtime_data_path


def test_installed_nested_artifact_paths_resolve_under_runtime_dir(monkeypatch, tmp_path: Path) -> None:
    runtime_dir = tmp_path / "runtime"
    config_path = tmp_path / "install" / "paus_bringup" / "share" / "paus_bringup" / "configs" / "default.yaml"
    monkeypatch.setenv("PAUS_ROBOT_RUNTIME_DIR", str(runtime_dir))

    assert Path(resolve_config_artifact_path("extrinsics.yaml", config_path)) == runtime_dir / "configs" / "extrinsics.yaml"
    assert Path(resolve_config_artifact_path("configs/eye_to_hand.yaml", config_path)) == runtime_dir / "configs" / "eye_to_hand.yaml"
    assert Path(resolve_config_artifact_path("calibration/extrinsics.yaml", config_path)) == runtime_dir / "calibration" / "extrinsics.yaml"
    assert Path(resolve_runtime_data_path("runs", config_path)) == runtime_dir / "runs"
