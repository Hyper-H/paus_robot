from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from ag_repro import load_config


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


if __name__ == "__main__":
    unittest.main()
