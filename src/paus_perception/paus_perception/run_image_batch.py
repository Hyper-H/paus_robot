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

from paus_perception import (
    SUPPORTED_IMAGE_EXTENSIONS,
    build_summary_record,
    load_camera_calibration,
    load_config,
    make_output_subdir_name,
    process_image_file,
    write_summary_csv,
    write_summary_json,
)


def _default_config_path() -> str:
    if get_package_share_directory is not None:
        try:
            return str(Path(get_package_share_directory("paus_bringup")) / "configs" / "default.yaml")
        except Exception:
            pass
    return str(PACKAGE_ROOT.parents[1] / "src" / "paus_bringup" / "configs" / "default.yaml")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run marker detection on a flat image directory.")
    parser.add_argument("--input-dir", required=True, help="Path to a single-level image directory.")
    parser.add_argument("--output-dir", required=True, help="Directory to store summaries and per-image outputs.")
    parser.add_argument("--config", default=_default_config_path(), help="Path to YAML config.")
    parser.add_argument("--camera-config", default=None, help="Optional path to camera calibration YAML.")
    return parser.parse_args()


def iter_image_files(input_dir: Path) -> list[Path]:
    return sorted(
        [
            path
            for path in input_dir.iterdir()
            if path.is_file() and path.suffix.lower() in SUPPORTED_IMAGE_EXTENSIONS
        ],
        key=lambda item: item.name.lower(),
    )


def main() -> int:
    args = parse_args()
    config = load_config(args.config)
    camera_calibration = load_camera_calibration(args.camera_config) if args.camera_config else None
    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir)

    if not input_dir.exists() or not input_dir.is_dir():
        raise FileNotFoundError(f"Input directory does not exist: {input_dir}")

    image_files = iter_image_files(input_dir)
    if not image_files:
        raise FileNotFoundError(f"No supported images found in: {input_dir}")

    output_dir.mkdir(parents=True, exist_ok=True)
    used_names: set[str] = set()
    summary_records: list[dict[str, object]] = []

    for image_path in image_files:
        subdir_name = make_output_subdir_name(image_path, used_names)
        image_output_dir = output_dir / subdir_name
        result = process_image_file(image_path, image_output_dir, config, camera_calibration=camera_calibration)
        summary_records.append(
            build_summary_record(
                image_path,
                result,
                subdir_name,
                camera_yaml_used=str(Path(args.camera_config)) if args.camera_config else "",
            )
        )
        print(json.dumps({"image": image_path.name, "status": result.status, "output_subdir": subdir_name}, ensure_ascii=False))

    write_summary_json(summary_records, output_dir / "summary.json")
    write_summary_csv(summary_records, output_dir / "summary.csv")

    success_count = sum(1 for record in summary_records if record["status"] == "ok")
    print(
        json.dumps(
            {"processed": len(summary_records), "success": success_count, "output_dir": str(output_dir)},
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if success_count > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
