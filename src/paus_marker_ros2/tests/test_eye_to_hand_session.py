from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
if str(MARKER_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MARKER_PACKAGE_ROOT))

from paus_marker_ros2.eye_to_hand_calibration_node import EyeToHandCalibrationNode


class EyeToHandSessionTests(unittest.TestCase):
    def _make_node(self, session_root_path: Path) -> EyeToHandCalibrationNode:
        node = EyeToHandCalibrationNode.__new__(EyeToHandCalibrationNode)
        node.session_root_path = session_root_path
        node.sample_log_path_override = None
        node.session_dir = None
        node.sample_log_path = None
        node.report_path = None
        node.run_log_path = None
        node.session_owner = None
        node.samples = []
        node.current_solution = None
        return node

    def test_semi_auto_capture_keeps_existing_semi_auto_session(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            node = self._make_node(Path(temp_dir))

            node._begin_new_semi_auto_session()
            semi_auto_dir = node.session_dir
            node.samples.append(object())
            node.current_solution = object()

            node._ensure_session_started("semi_auto")

            self.assertEqual(node.session_dir, semi_auto_dir)
            self.assertEqual(node.session_owner, "semi_auto")
            self.assertEqual(len(node.samples), 1)
            self.assertIsNotNone(node.current_solution)

    def test_manual_capture_after_semi_auto_run_starts_fresh_session(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            node = self._make_node(Path(temp_dir))

            node._begin_new_semi_auto_session()
            semi_auto_dir = node.session_dir
            node.samples.append(object())
            node.current_solution = object()

            node._ensure_session_started("manual")

            self.assertNotEqual(node.session_dir, semi_auto_dir)
            self.assertEqual(node.session_owner, "manual")
            self.assertEqual(node.samples, [])
            self.assertIsNone(node.current_solution)


if __name__ == "__main__":
    unittest.main()
