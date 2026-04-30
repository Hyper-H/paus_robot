from __future__ import annotations

from pathlib import Path

from paus_perception.config import _resolve_config_artifact_path


def test_installed_nested_artifact_paths_resolve_under_runtime_dir(monkeypatch, tmp_path: Path) -> None:
    runtime_dir = tmp_path / "runtime"
    config_path = tmp_path / "install" / "paus_bringup" / "share" / "paus_bringup" / "configs" / "default.yaml"
    monkeypatch.setenv("PAUS_ROBOT_RUNTIME_DIR", str(runtime_dir))

    assert Path(_resolve_config_artifact_path("extrinsics.yaml", config_path)) == runtime_dir / "configs" / "extrinsics.yaml"
    assert Path(_resolve_config_artifact_path("configs/eye_to_hand.yaml", config_path)) == runtime_dir / "configs" / "eye_to_hand.yaml"
    assert Path(_resolve_config_artifact_path("calibration/extrinsics.yaml", config_path)) == runtime_dir / "calibration" / "extrinsics.yaml"
