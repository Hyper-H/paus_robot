from __future__ import annotations

import unittest
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[3]


class OnlineStackLaunchStaticTests(unittest.TestCase):
    def test_markerless_neck_surface_launch_is_default_disabled(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )

        config = yaml.safe_load(
            (PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml").read_text(encoding="utf-8")
        )

        self.assertIn("enabled", config["neck_surface"])
        self.assertIn('load_config(config_path)', launch_text)
        self.assertIn('enable_neck_surface = bool(neck_cfg.get("enabled", False))', launch_text)
        self.assertIn('if enable_neck_surface:', launch_text)
        self.assertIn('executable="neck_surface_pose_node"', launch_text)

    def test_markerless_neck_surface_enables_depth_bridge(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('camera_bridge_cmd += ["--enable-depth"]', launch_text)

    def test_online_stack_prints_neck_runtime_config(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"event": "online_stack_config"', launch_text)
        self.assertIn('"neck_surface_enabled": enable_neck_surface', launch_text)
        self.assertIn('"camera_bridge_enable_depth": enable_neck_surface', launch_text)

    def test_camera_bridge_defaults_to_paus_robot_conda_python(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"python_exec"', launch_text)
        self.assertIn('default_value="/home/chen_lab/miniconda3/envs/paus_robot/bin/python3"', launch_text)
        self.assertIn('LaunchConfiguration("python_exec").perform(context)', launch_text)
        self.assertNotIn('shutil.which("python3")', launch_text)

    def test_markerless_neck_surface_uses_image_receiver_depth_topic(self) -> None:
        config = yaml.safe_load(
            (PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml").read_text(encoding="utf-8")
        )
        receiver_text = (
            PROJECT_ROOT / "src" / "paus_marker_ros2" / "paus_marker_ros2" / "image_receiver_node.py"
        ).read_text(encoding="utf-8")

        self.assertEqual(config["neck_surface"]["depth_topic"], "/camera/depth_aligned")
        self.assertIn('self.declare_parameter("depth_topic", "/camera/depth_aligned")', receiver_text)


if __name__ == "__main__":
    unittest.main()
