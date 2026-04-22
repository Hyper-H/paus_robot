from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if (PACKAGE_ROOT / "setup.py").exists() and str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from paus_perception import load_camera_calibration, load_config, process_image_file, result_to_dict


def _default_config_path() -> str:
    if get_package_share_directory is not None:
        try:
            return str(Path(get_package_share_directory("paus_bringup")) / "configs" / "default.yaml")
        except Exception:
            pass
    return str(PACKAGE_ROOT.parents[1] / "src" / "paus_bringup" / "configs" / "default.yaml")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Detect one ArUco marker from a single image.")
    parser.add_argument("--input", required=True, help="Path to one RGB image.")
    parser.add_argument("--output-dir", required=True, help="Directory to store JSON, visualization, and debug images.")
    parser.add_argument("--config", default=_default_config_path(), help="Path to YAML config.")
    parser.add_argument("--camera-config", default=None, help="Optional path to camera calibration YAML.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    camera_calibration = load_camera_calibration(args.camera_config) if args.camera_config else None
    result = process_image_file(args.input, args.output_dir, config, camera_calibration=camera_calibration)
    print(json.dumps(result_to_dict(result), indent=2))
    return 0 if result.status == "ok" else 1


if __name__ == "__main__":
    raise SystemExit(main())
