from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if (PACKAGE_ROOT / "setup.py").exists() and str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from paus_perception import calibrate_camera_from_directory, save_camera_calibration


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate camera from a chessboard image directory.")
    parser.add_argument("--input-dir", required=True, help="Directory containing chessboard images.")
    parser.add_argument("--rows", required=True, type=int, help="Chessboard inner-corner rows.")
    parser.add_argument("--cols", required=True, type=int, help="Chessboard inner-corner cols.")
    parser.add_argument("--square-size-m", required=True, type=float, help="Physical square size in meters.")
    parser.add_argument("--output", required=True, help="Output camera YAML path.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    result = calibrate_camera_from_directory(args.input_dir, args.rows, args.cols, args.square_size_m)
    if result.success and result.calibration is not None:
        save_camera_calibration(result.calibration, args.output)
    print(json.dumps(result, default=lambda item: item.__dict__, indent=2))
    return 0 if result.success else 1


if __name__ == "__main__":
    raise SystemExit(main())
