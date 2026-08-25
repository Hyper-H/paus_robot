from __future__ import annotations

from pathlib import Path

from paus_perception import load_eye_to_hand_solution
from paus_perception.config import resolve_config_artifact_path


CURRENT_EXTRINSICS_PATH = "/mnt/data/projects/paus_robot/calibration/current/extrinsics.yaml"
EXAMPLE_EXTRINSICS_PATH = "src/paus_bringup/configs/extrinsics.yaml"


def resolve_extrinsics_path(config: dict, config_path: str | Path, *, execute_motion: bool) -> tuple[str, dict]:
    calibration_cfg = config.get("calibration", {})
    current_path = Path(str(calibration_cfg.get("output_path", CURRENT_EXTRINSICS_PATH))).expanduser()
    if not current_path.is_absolute():
        current_path = Path(resolve_config_artifact_path(current_path, config_path))
    example_path = Path(str(calibration_cfg.get("example_output_path", EXAMPLE_EXTRINSICS_PATH))).expanduser()
    if not example_path.is_absolute():
        example_path = Path(resolve_config_artifact_path(example_path, config_path))
    if not example_path.exists():
        config_sibling_example = Path(config_path).expanduser().resolve().parent / "extrinsics.yaml"
        if config_sibling_example.exists():
            example_path = config_sibling_example

    selected_path = current_path if current_path.exists() else example_path
    source = "current" if current_path.exists() else "example_fallback"
    status = {
        "extrinsics_current_path": str(current_path),
        "extrinsics_example_path": str(example_path),
        "extrinsics_path": str(selected_path),
        "extrinsics_source": source,
        "extrinsics_exists": selected_path.exists(),
        "extrinsics_artifact_kind": None,
        "extrinsics_dummy": None,
    }
    if not selected_path.exists():
        if execute_motion:
            raise RuntimeError(f"Real robot motion requires extrinsics file: {current_path}")
        status["extrinsics_source"] = "missing"
        return str(selected_path), status

    solution = load_eye_to_hand_solution(selected_path)
    status["extrinsics_artifact_kind"] = solution.artifact_kind
    status["extrinsics_dummy"] = solution.is_dummy
    if execute_motion and (source != "current" or solution.is_dummy or not solution.success):
        raise RuntimeError(
            "Real robot motion requires a non-dummy current extrinsics artifact at "
            f"{current_path}; selected={selected_path}, source={source}, kind={solution.artifact_kind}, success={solution.success}."
        )
    return str(selected_path), status
