from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[3]
BRINGUP_PACKAGE_ROOT = PROJECT_ROOT / "src" / "paus_bringup"
import sys

if str(BRINGUP_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(BRINGUP_PACKAGE_ROOT))

from paus_bringup.extrinsics import resolve_extrinsics_path


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

    def test_neck_target_eval_launch_is_default_enabled(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )

        config = yaml.safe_load(
            (PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml").read_text(encoding="utf-8")
        )

        self.assertTrue(config["neck_surface"]["eval_enabled"])
        self.assertIn('enable_neck_eval = bool(neck_cfg.get("eval_enabled", False))', launch_text)
        self.assertIn('if enable_neck_eval:', launch_text)
        self.assertIn('executable="neck_target_eval_node"', launch_text)

    def test_online_stack_prints_neck_runtime_config(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('"event": "online_stack_config"', launch_text)
        self.assertIn('"neck_surface_enabled": enable_neck_surface', launch_text)
        self.assertIn('"neck_eval_enabled": enable_neck_eval', launch_text)
        self.assertIn('resolve_extrinsics_path(config, config_path, execute_motion=execute_motion_enabled)', launch_text)
        self.assertIn("**extrinsics_status", launch_text)
        self.assertIn('"targeting_mode": targeting_mode', launch_text)
        self.assertIn('"camera_bridge_enable_depth": enable_neck_surface', launch_text)
        self.assertIn('"run_id": run_id', launch_text)
        self.assertIn('"run_dir": str(neck_session_root)', launch_text)

    def test_markerless_runtime_logs_use_per_launch_session_dir(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )

        self.assertIn('DeclareLaunchArgument(\n                "run_id"', launch_text)
        self.assertIn('datetime.now().strftime("%Y-%m-%d_%H-%M-%S")', launch_text)
        self.assertIn('"frames_dir": str(neck_session_root / "frames")', launch_text)
        self.assertIn('"eval_log_path": str(neck_session_root / "eval.jsonl")', launch_text)
        self.assertIn('"summary_path": str(neck_session_root / "summary.json")', launch_text)
        self.assertIn('"transform_trace_path": str(neck_session_root / "transform_trace.jsonl")', launch_text)
        self.assertIn('"selector_trace_path": str(neck_session_root / "selector_trace.jsonl")', launch_text)
        self.assertIn('"target_lock_trace_path": str(neck_session_root / "target_lock_trace.jsonl")', launch_text)
        self.assertIn('"target_lock_status_latest_path": str(neck_session_root / "target_lock_status_latest.json")', launch_text)
        self.assertIn('"control_trace_path": str(neck_session_root / "control_trace.jsonl")', launch_text)
        self.assertIn('"run_report_path": str(neck_session_root / "run_report.md")', launch_text)
        self.assertIn('"run_dir": str(neck_session_root)', launch_text)
        self.assertIn('"logging_enabled": False', launch_text)
        self.assertIn('"run_dir": str(neck_session_root)', launch_text)
        self.assertIn('executable="target_transform_node"', launch_text)
        self.assertIn('executable="target_selector_node"', launch_text)
        self.assertIn('executable="target_lock_node"', launch_text)
        self.assertIn('executable="fairino_control_node"', launch_text)

    def test_target_selector_defaults_and_topics_are_configured(self) -> None:
        launch_text = (PROJECT_ROOT / "src" / "paus_bringup" / "launch" / "online_stack.launch.py").read_text(
            encoding="utf-8"
        )
        config = yaml.safe_load(
            (PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml").read_text(encoding="utf-8")
        )

        self.assertEqual(config["targeting"]["mode"], "markerless_neck")
        self.assertEqual(config["targeting"]["source_timeout_ms"], 2000)
        self.assertTrue(config["target_lock"]["enabled"])
        self.assertEqual(config["target_lock"]["collect_duration_s"], 4.0)
        self.assertEqual(config["target_lock"]["min_samples"], 3)
        self.assertEqual(config["target_lock"]["max_sample_age_ms"], 2500)
        self.assertEqual(config["target_lock"]["max_sample_jump_mm"], 120.0)
        self.assertEqual(config["target_lock"]["max_position_spread_mm"], 60.0)
        self.assertEqual(config["target_lock"]["drift_warning_mm"], 80.0)
        self.assertEqual(config["target_lock"]["policy"], "lock_once_episode")
        self.assertEqual(config["target_lock"]["stale_after_locked_action"], "warn_only")
        self.assertFalse(config["target_lock"]["allow_relock_during_motion"])
        self.assertIn('markerless_target_pose_topic = "/markerless_neck_target_pose_base"', launch_text)
        self.assertIn('marker_target_pose_topic = "/marker_target_pose_base"', launch_text)
        self.assertIn('selected_target_pose_topic = "/selected_target_pose_base"', launch_text)
        self.assertIn('locked_target_pose_topic = "/locked_target_pose_base"', launch_text)
        self.assertIn('target_lock_status_topic = "/target_lock_status"', launch_text)
        self.assertIn("control_target_pose_topic = locked_target_pose_topic if target_lock_enabled else selected_target_pose_topic", launch_text)
        self.assertIn('"target_pose_topic": markerless_target_pose_topic', launch_text)
        self.assertIn('"target_pose_topic": marker_target_pose_topic', launch_text)
        self.assertIn('"selected_target_pose_topic": selected_target_pose_topic', launch_text)
        self.assertIn('"target_pose_topic": control_target_pose_topic', launch_text)
        self.assertIn('"selector_status_topic": selector_status_topic', launch_text)
        self.assertIn('"target_lock_status_topic": target_lock_status_topic', launch_text)
        self.assertIn('"targeting_mode": targeting_mode', launch_text)
        self.assertIn('"neck_surface_target_mode": neck_cfg.get("target_mode", "refined_surface")', launch_text)
        self.assertIn('DeclareLaunchArgument(\n                "targeting_mode"', launch_text)
        self.assertEqual(config["control"]["motion_strategy"], "staged_patient_left_final_hover")
        self.assertTrue(config["control"]["require_locked_target_before_motion"])
        self.assertTrue(config["control"]["continue_with_last_locked_target_on_source_loss"])
        self.assertFalse(config["control"]["move_to_approach_ready_on_start"])
        self.assertEqual(len(config["control"]["approach_ready_joint_deg"]), 6)
        self.assertEqual(config["control"]["roll_policy"], "base_up_projection")
        self.assertEqual(config["control"]["final_hover_vel"], 5.0)

    def test_markerless_logging_defaults_limit_disk_usage(self) -> None:
        config = yaml.safe_load(
            (PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml").read_text(encoding="utf-8")
        )

        neck_cfg = config["neck_surface"]
        self.assertEqual(neck_cfg["log_every_n_frames"], 1)
        self.assertFalse(neck_cfg["log_only_ok_frames"])
        self.assertEqual(neck_cfg["max_logged_frames"], 200)
        self.assertFalse(neck_cfg["save_depth"])
        self.assertTrue(neck_cfg["save_debug_eval"])
        self.assertTrue(neck_cfg["save_debug_surface"])

    def test_markerless_refined_surface_target_mode_is_default(self) -> None:
        config = yaml.safe_load(
            (PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml").read_text(encoding="utf-8")
        )

        self.assertEqual(config["neck_surface"]["target_mode"], "refined_surface")
        self.assertEqual(config["targeting"]["mode"], "markerless_neck")

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

    def test_extrinsics_resolver_allows_example_only_for_dry_run(self) -> None:
        config_path = PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml"
        with tempfile.TemporaryDirectory() as temp_dir:
            missing_current = Path(temp_dir) / "current" / "extrinsics.yaml"
            config = {
                "calibration": {
                    "output_path": str(missing_current),
                    "example_output_path": str(PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "extrinsics.yaml"),
                }
            }

            selected_path, status = resolve_extrinsics_path(config, str(config_path), execute_motion=False)

            self.assertEqual(selected_path, str(PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "extrinsics.yaml"))
            self.assertEqual(status["extrinsics_source"], "example_fallback")
            self.assertEqual(status["extrinsics_artifact_kind"], "identity_example")
            self.assertTrue(status["extrinsics_dummy"])

    def test_extrinsics_resolver_fails_closed_for_real_motion_without_current(self) -> None:
        config_path = PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml"
        with tempfile.TemporaryDirectory() as temp_dir:
            missing_current = Path(temp_dir) / "current" / "extrinsics.yaml"
            config = {
                "calibration": {
                    "output_path": str(missing_current),
                    "example_output_path": str(PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "extrinsics.yaml"),
                }
            }

            with self.assertRaisesRegex(RuntimeError, "Real robot motion requires"):
                resolve_extrinsics_path(config, str(config_path), execute_motion=True)

    def test_extrinsics_resolver_fails_closed_for_dummy_current(self) -> None:
        config_path = PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "default.yaml"
        example_path = PROJECT_ROOT / "src" / "paus_bringup" / "configs" / "extrinsics.yaml"
        config = {
            "calibration": {
                "output_path": str(example_path),
                "example_output_path": str(example_path),
            }
        }

        with self.assertRaisesRegex(RuntimeError, "non-dummy current extrinsics"):
            resolve_extrinsics_path(config, str(config_path), execute_motion=True)


if __name__ == "__main__":
    unittest.main()
