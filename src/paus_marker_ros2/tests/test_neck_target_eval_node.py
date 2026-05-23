from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

PROJECT_ROOT = Path(__file__).resolve().parents[3]
MARKER_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_marker_ros2"
if str(MARKER_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MARKER_PACKAGE_ROOT))

for mod_name in (
    "rclpy",
    "rclpy.node",
    "rclpy.executors",
    "geometry_msgs",
    "geometry_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
    "ament_index_python",
    "ament_index_python.packages",
):
    if mod_name not in sys.modules:
        sys.modules[mod_name] = mock.MagicMock()

sys.modules["rclpy.node"].Node = object
sys.modules["rclpy.executors"].ExternalShutdownException = RuntimeError

from paus_marker_ros2.neck_target_eval_node import NeckTargetEvalNode


class Logger:
    def warning(self, _message):
        pass


class NeckTargetEvalNodeModuleTests(unittest.TestCase):
    def test_write_latest_status_saves_waiting_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            node = NeckTargetEvalNode.__new__(NeckTargetEvalNode)
            node.logging_enabled = True
            node.status_latest_path = Path(tmpdir) / "status_latest.json"
            node.get_logger = lambda: Logger()
            payload = {"source": "neck_target_eval", "status": "waiting", "reason": "markerless_not_ok"}

            node._write_latest_status(payload)

            saved = json.loads(node.status_latest_path.read_text(encoding="utf-8"))
            self.assertEqual(saved, payload)

    def test_write_latest_status_saves_ok_payload(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            node = NeckTargetEvalNode.__new__(NeckTargetEvalNode)
            node.logging_enabled = True
            node.status_latest_path = Path(tmpdir) / "status_latest.json"
            node.get_logger = lambda: Logger()
            payload = {"source": "neck_target_eval", "status": "ok", "error_norm_mm": 12.3}

            node._write_latest_status(payload)

            saved = json.loads(node.status_latest_path.read_text(encoding="utf-8"))
            self.assertEqual(saved["status"], "ok")
            self.assertEqual(saved["error_norm_mm"], 12.3)

    def test_write_latest_status_is_noop_when_logging_disabled(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            node = NeckTargetEvalNode.__new__(NeckTargetEvalNode)
            node.logging_enabled = False
            node.status_latest_path = Path(tmpdir) / "status_latest.json"
            node.get_logger = lambda: Logger()

            node._write_latest_status({"status": "waiting"})

            self.assertFalse(node.status_latest_path.exists())


if __name__ == "__main__":
    unittest.main()
