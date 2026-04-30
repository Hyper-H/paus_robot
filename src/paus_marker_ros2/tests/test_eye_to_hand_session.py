from __future__ import annotations

import sys
import unittest
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
if str(MARKER_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MARKER_PACKAGE_ROOT))

from paus_marker_ros2.semi_auto_calibration import session_owner_matches


class EyeToHandSessionTests(unittest.TestCase):
    def test_session_owner_matches_for_active_semi_auto_capture(self) -> None:
        self.assertTrue(session_owner_matches("semi_auto", "semi_auto"))

    def test_session_owner_mismatch_for_manual_capture_after_semi_auto(self) -> None:
        self.assertFalse(session_owner_matches("semi_auto", "manual"))
        self.assertFalse(session_owner_matches(None, "manual"))


if __name__ == "__main__":
    unittest.main()
