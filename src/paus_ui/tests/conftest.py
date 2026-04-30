from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[3]
for package_name in ("paus_ui", "paus_marker_ros2", "paus_perception", "paus_motion_ros2"):
    package_root = PROJECT_ROOT / "src" / package_name
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
