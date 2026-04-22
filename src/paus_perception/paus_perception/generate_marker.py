from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

ARUCO_DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in dir(cv2.aruco)
    if name.startswith("DICT_")
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate one printable ArUco marker image.")
    parser.add_argument("--marker-id", type=int, required=True, help="Marker ID inside the selected dictionary.")
    parser.add_argument("--dictionary", default="DICT_4X4_50", help="OpenCV ArUco dictionary name.")
    parser.add_argument("--size", type=int, default=600, help="Output marker image size in pixels.")
    parser.add_argument("--output", required=True, help="Output image path, for example markers/id7.png.")
    return parser.parse_args()


def get_dictionary(name: str) -> cv2.aruco.Dictionary:
    if name not in ARUCO_DICTIONARIES:
        available = ", ".join(sorted(ARUCO_DICTIONARIES))
        raise ValueError(f"Unsupported dictionary '{name}'. Available: {available}")
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICTIONARIES[name])


def main() -> int:
    args = parse_args()
    dictionary = get_dictionary(args.dictionary)
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, args.marker_id, args.size)
    else:
        marker = np.zeros((args.size, args.size), dtype=np.uint8)
        cv2.aruco.drawMarker(dictionary, args.marker_id, args.size, marker, 1)
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    success = cv2.imwrite(str(output_path), marker)
    if not success:
        raise RuntimeError(f"Failed to write marker image to: {output_path}")
    print(f"Saved marker id={args.marker_id} dictionary={args.dictionary} to {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
