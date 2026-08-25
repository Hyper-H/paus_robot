from __future__ import annotations

from datetime import datetime
from pathlib import Path
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from paus_perception import load_config


def _resolve_default_config_path(bringup_share: Path) -> Path:
    workspace_root = bringup_share.parents[3]
    source_config = workspace_root / "src" / "paus_bringup" / "configs" / "default.yaml"
    return source_config if source_config.exists() else bringup_share / "configs" / "default.yaml"


def _resolve_run_root(config: dict) -> Path:
    neck_cfg = config.get("neck_surface", {})
    configured = Path(str(neck_cfg.get("logging_output_dir", "/mnt/data/projects/paus_robot/runs/markerless-neck-surface-design/neck_surface")))
    return configured.parent if configured.name == "neck_surface" else configured


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    config_path = LaunchConfiguration("config_path").perform(context)
    execute_motion = LaunchConfiguration("execute_motion").perform(context).strip().lower() == "true"
    run_id = LaunchConfiguration("run_id").perform(context).strip() or f"static_locked_target_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    static_trace_path = LaunchConfiguration("static_trace_path").perform(context).strip()
    static_trace_run_id = LaunchConfiguration("static_trace_run_id").perform(context).strip()
    static_pose_mmdeg = LaunchConfiguration("static_pose_mmdeg").perform(context).strip()
    publish_rate_hz = LaunchConfiguration("publish_rate_hz").perform(context).strip()
    targeting_mode = LaunchConfiguration("targeting_mode").perform(context).strip() or "markerless_neck"

    config = load_config(config_path)
    run_root = _resolve_run_root(config)
    run_dir = run_root / run_id
    if not static_trace_path and static_trace_run_id:
        static_trace_path = str(run_root / static_trace_run_id / "control_trace.jsonl")

    locked_target_pose_topic = "/locked_target_pose_base"
    target_lock_status_topic = "/target_lock_status"
    selector_status_topic = "/target_selector_status"

    static_args = [
        "--locked-target-pose-topic",
        locked_target_pose_topic,
        "--target-lock-status-topic",
        target_lock_status_topic,
        "--selector-status-topic",
        selector_status_topic,
        "--publish-rate-hz",
        publish_rate_hz,
    ]
    if static_pose_mmdeg:
        static_args += ["--pose-mmdeg", static_pose_mmdeg]
    else:
        static_args += ["--trace-path", static_trace_path]

    print(
        json.dumps(
            {
                "event": "static_locked_target_control_config",
                "config_path": config_path,
                "execute_motion": execute_motion,
                "run_id": run_id,
                "run_dir": str(run_dir),
                "static_trace_path": static_trace_path,
                "static_trace_run_id": static_trace_run_id,
                "static_pose_mmdeg": static_pose_mmdeg or None,
                "targeting_mode": targeting_mode,
                "locked_target_pose_topic": locked_target_pose_topic,
                "target_lock_status_topic": target_lock_status_topic,
                "selector_status_topic": selector_status_topic,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )

    return [
        Node(
            package="paus_bringup",
            executable="static_locked_target_publisher",
            name="static_locked_target_publisher",
            output="screen",
            arguments=static_args,
        ),
        Node(
            package="paus_motion_ros2",
            executable="fairino_control_node",
            name="fairino_control_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "execute_motion": execute_motion,
                    "target_pose_topic": locked_target_pose_topic,
                    "selector_status_topic": selector_status_topic,
                    "target_lock_status_topic": target_lock_status_topic,
                    "targeting_mode": targeting_mode,
                    "run_dir": str(run_dir),
                }
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    default_config_path = str(_resolve_default_config_path(bringup_share))
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path", default_value=default_config_path, description="Path to the main YAML config file."),
            DeclareLaunchArgument("execute_motion", default_value="false", description="Whether to allow real robot motion."),
            DeclareLaunchArgument("run_id", default_value="", description="Runtime session directory name."),
            DeclareLaunchArgument("static_trace_path", default_value="", description="control_trace.jsonl path used as the static target source."),
            DeclareLaunchArgument("static_trace_run_id", default_value="", description="Run id whose control_trace.jsonl supplies the static target."),
            DeclareLaunchArgument("static_pose_mmdeg", default_value="", description="Static raw target pose as x,y,z,rx,ry,rz in mm/deg."),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0", description="Static target publish rate."),
            DeclareLaunchArgument("targeting_mode", default_value="markerless_neck", description="Control targeting mode."),
            OpaqueFunction(function=_launch_setup),
        ]
    )
