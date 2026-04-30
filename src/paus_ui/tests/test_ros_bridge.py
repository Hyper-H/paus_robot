from __future__ import annotations

import sys
import tempfile
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
UI_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_ui"
if str(UI_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(UI_PACKAGE_ROOT))

from paus_ui.ros_bridge import UiRosBridge


def test_get_detector_returns_none_for_malformed_camera_yaml() -> None:
    with tempfile.TemporaryDirectory() as temp_dir:
        camera_yaml = Path(temp_dir) / "camera.yaml"
        camera_yaml.write_text("not: [valid", encoding="utf-8")
        bridge = UiRosBridge.__new__(UiRosBridge)
        bridge.camera_config_path = camera_yaml
        bridge._detector = None
        bridge._detector_mtime_ns = None

        assert UiRosBridge._get_detector(bridge) is None
        assert bridge._detector is None
        assert bridge._detector_mtime_ns is None
