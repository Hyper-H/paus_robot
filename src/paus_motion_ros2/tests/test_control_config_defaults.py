from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[3]
PERCEPTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_perception"
if str(PERCEPTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PERCEPTION_PACKAGE_ROOT))
MOTION_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_motion_ros2"
if str(MOTION_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(MOTION_PACKAGE_ROOT))

from paus_perception import load_config
from paus_motion_ros2.fairino_linux_client import FairinoLinuxClient


class ConfigControlDefaultsTests(unittest.TestCase):
    def test_default_control_config_contains_fairino_sdk_fallback_settings(self) -> None:
        config = load_config()
        control = config["control"]
        self.assertEqual(control["linux_fairino_sdk_root"], "/opt/fairino_python_sdk/linux")
        self.assertNotIn("prefer_windows_exec_bridge", control)
        self.assertNotIn("prefer_remote_command_service", control)

    def test_user_config_can_override_fairino_sdk_paths(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            config_path = Path(temp_dir) / "config.yaml"
            config_path.write_text(
                yaml.safe_dump(
                    {
                        "control": {
                            "linux_fairino_sdk_root": "/srv/fairino/linux_sdk",
                            "use_mock_pose": True,
                        }
                    },
                    sort_keys=False,
                    allow_unicode=True,
                ),
                encoding="utf-8",
            )

            config = load_config(config_path)
            control = config["control"]
            self.assertEqual(control["linux_fairino_sdk_root"], "/srv/fairino/linux_sdk")
            self.assertTrue(control["use_mock_pose"])


class FairinoLinuxClientTests(unittest.TestCase):
    def test_normalize_pose_result_accepts_sdk_list_shape(self) -> None:
        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")

        error, pose = client._normalize_pose_result((0, [1, 2, 3, 4, 5, 6]), "GetActualTCPPose")

        self.assertEqual(error, 0)
        self.assertEqual(pose, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_normalize_pose_result_accepts_xmlrpc_flat_shape(self) -> None:
        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")

        error, pose = client._normalize_pose_result((0, 1, 2, 3, 4, 5, 6), "GetActualTCPPose")

        self.assertEqual(error, 0)
        self.assertEqual(pose, [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])

    def test_normalize_pose_result_accepts_joint_position_shape(self) -> None:
        client = FairinoLinuxClient("/tmp/fairino", "192.168.58.2")

        error, joints = client._normalize_pose_result((0, [10, 20, 30, 40, 50, 60]), "GetActualJointPosDegree")

        self.assertEqual(error, 0)
        self.assertEqual(joints, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0])


if __name__ == "__main__":
    unittest.main()
