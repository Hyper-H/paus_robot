from __future__ import annotations

from datetime import datetime
from pathlib import Path
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from paus_bringup.extrinsics import resolve_extrinsics_path
from paus_perception import load_config


def _resolve_default_config_path(bringup_share: Path) -> Path:
    workspace_root = bringup_share.parents[3]
    source_config = workspace_root / "src" / "paus_bringup" / "configs" / "default.yaml"
    return source_config if source_config.exists() else bringup_share / "configs" / "default.yaml"


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    marker_share = Path(get_package_share_directory("paus_marker_ros2"))
    python_exec = LaunchConfiguration("python_exec").perform(context).strip() or "/home/chen_lab/miniconda3/envs/paus_robot/bin/python3"
    camera_bridge_script = marker_share / "scripts" / "camera_bridge.py"

    config_path = LaunchConfiguration("config_path").perform(context)
    camera_ip = LaunchConfiguration("camera_ip").perform(context).strip()
    camera_index = LaunchConfiguration("camera_index").perform(context).strip()
    rgb_camera_count = LaunchConfiguration("rgb_camera_count").perform(context).strip()
    execute_motion = LaunchConfiguration("execute_motion").perform(context).strip().lower()
    execute_motion_enabled = execute_motion == "true"
    run_id = LaunchConfiguration("run_id").perform(context).strip()
    max_execution_stage = LaunchConfiguration("max_execution_stage").perform(context).strip()
    approach_ready_vel = LaunchConfiguration("approach_ready_vel").perform(context).strip()
    pre_approach_vel = LaunchConfiguration("pre_approach_vel").perform(context).strip()
    final_hover_vel = LaunchConfiguration("final_hover_vel").perform(context).strip()
    final_hover_servo_cmd_t_s = LaunchConfiguration("final_hover_servo_cmd_t_s").perform(context).strip()
    final_hover_servo_max_step_mm = LaunchConfiguration("final_hover_servo_max_step_mm").perform(context).strip()
    final_hover_servo_max_step_deg = LaunchConfiguration("final_hover_servo_max_step_deg").perform(context).strip()
    config = load_config(config_path)
    extrinsics_path, extrinsics_status = resolve_extrinsics_path(config, config_path, execute_motion=execute_motion_enabled)
    neck_cfg = config.get("neck_surface", {})
    targeting_cfg = config.get("targeting", {})
    target_lock_cfg = config.get("target_lock", {})
    enable_neck_surface = bool(neck_cfg.get("enabled", False))
    enable_neck_eval = bool(neck_cfg.get("eval_enabled", False))
    targeting_mode = LaunchConfiguration("targeting_mode").perform(context).strip() or str(targeting_cfg.get("mode", "markerless_neck"))
    source_timeout_ms = int(targeting_cfg.get("source_timeout_ms", 500))
    target_lock_enabled = bool(target_lock_cfg.get("enabled", True))
    if not run_id:
        run_id = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    neck_session_root = _resolve_neck_session_root(neck_cfg, run_id)
    markerless_target_pose_topic = "/markerless_neck_target_pose_base"
    marker_target_pose_topic = "/marker_target_pose_base"
    selected_target_pose_topic = "/selected_target_pose_base"
    selector_status_topic = "/target_selector_status"
    locked_target_pose_topic = "/locked_target_pose_base"
    target_lock_status_topic = "/target_lock_status"
    control_target_pose_topic = locked_target_pose_topic if target_lock_enabled else selected_target_pose_topic

    camera_bridge_cmd = [
        python_exec,
        str(camera_bridge_script),
        "--host",
        "127.0.0.1",
        "--port",
        "5001",
        "--rgb-camera-count",
        rgb_camera_count,
    ]
    if camera_ip:
        camera_bridge_cmd += ["--camera-ip", camera_ip]
    if camera_index:
        camera_bridge_cmd += ["--camera-index", camera_index]
    if enable_neck_surface:
        camera_bridge_cmd += ["--enable-depth"]

    print(
        json.dumps(
            {
                "event": "online_stack_config",
                "config_path": config_path,
                "execute_motion": execute_motion_enabled,
                **extrinsics_status,
                "neck_surface_enabled": enable_neck_surface,
                "neck_eval_enabled": enable_neck_eval,
                "neck_surface_target_mode": neck_cfg.get("target_mode", "refined_surface"),
                "targeting_mode": targeting_mode,
                "targeting_source_timeout_ms": source_timeout_ms,
                "camera_bridge_enable_depth": enable_neck_surface,
                "image_topic": neck_cfg.get("image_topic", "/camera/image_bridge"),
                "depth_topic": neck_cfg.get("depth_topic", "/camera/depth_aligned"),
                "camera_info_topic": neck_cfg.get("camera_info_topic", "/camera/camera_info"),
                "markerless_target_pose_topic": markerless_target_pose_topic,
                "marker_target_pose_topic": marker_target_pose_topic,
                "selected_target_pose_topic": selected_target_pose_topic,
                "selector_status_topic": selector_status_topic,
                "target_lock_enabled": target_lock_enabled,
                "locked_target_pose_topic": locked_target_pose_topic,
                "target_lock_status_topic": target_lock_status_topic,
                "control_target_pose_topic": control_target_pose_topic,
                "run_id": run_id,
                "run_dir": str(neck_session_root),
                "frames_dir": str(neck_session_root / "frames"),
                "eval_log_path": str(neck_session_root / "eval.jsonl"),
                "summary_path": str(neck_session_root / "summary.json"),
                "transform_trace_path": str(neck_session_root / "transform_trace.jsonl"),
                "selector_trace_path": str(neck_session_root / "selector_trace.jsonl"),
                "target_lock_trace_path": str(neck_session_root / "target_lock_trace.jsonl"),
                "target_lock_status_latest_path": str(neck_session_root / "target_lock_status_latest.json"),
                "control_trace_path": str(neck_session_root / "control_trace.jsonl"),
                "run_report_path": str(neck_session_root / "run_report.md"),
            },
            ensure_ascii=False,
        ),
        flush=True,
    )

    nodes = [
        ExecuteProcess(
            cmd=camera_bridge_cmd,
            name="camera_bridge_process",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="paus_marker_ros2",
            executable="image_receiver_node",
            name="image_receiver_node",
            output="screen",
        ),
        Node(
            package="paus_marker_ros2",
            executable="marker_pose_node",
            name="marker_pose_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                }
            ],
        ),
        Node(
            package="paus_marker_ros2",
            executable="target_transform_node",
            name="target_transform_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "extrinsics_path": extrinsics_path,
                    "target_pose_topic": marker_target_pose_topic,
                    "run_dir": str(neck_session_root),
                }
            ],
        ),
    ]
    if enable_neck_surface:
        nodes.append(
            Node(
                package="paus_marker_ros2",
                executable="neck_surface_pose_node",
                name="neck_surface_pose_node",
                output="screen",
                parameters=[
                    {
                        "config_path": config_path,
                        "extrinsics_path": extrinsics_path,
                        "execute_motion": execute_motion_enabled,
                        "target_pose_topic": markerless_target_pose_topic,
                        "run_dir": str(neck_session_root),
                    }
                ],
            )
        )
        if enable_neck_eval:
            nodes.append(
                Node(
                    package="paus_marker_ros2",
                    executable="neck_target_eval_node",
                    name="neck_target_eval_node",
                    output="screen",
                    parameters=[
                        {
                            "logging_enabled": False,
                        }
                    ],
                )
            )
    nodes.append(
        Node(
            package="paus_marker_ros2",
            executable="target_selector_node",
            name="target_selector_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "mode": targeting_mode,
                    "source_timeout_ms": source_timeout_ms,
                    "markerless_target_pose_topic": markerless_target_pose_topic,
                    "marker_target_pose_topic": marker_target_pose_topic,
                    "selected_target_pose_topic": selected_target_pose_topic,
                    "status_topic": selector_status_topic,
                    "run_dir": str(neck_session_root),
                }
            ],
        )
    )
    if target_lock_enabled:
        nodes.append(
            Node(
                package="paus_marker_ros2",
                executable="target_lock_node",
                name="target_lock_node",
                output="screen",
                parameters=[
                    {
                        "config_path": config_path,
                        "enabled": target_lock_enabled,
                        "live_target_pose_topic": selected_target_pose_topic,
                        "locked_target_pose_topic": locked_target_pose_topic,
                        "status_topic": target_lock_status_topic,
                        "run_dir": str(neck_session_root),
                    }
                ],
            )
        )
    control_params = {
        "config_path": config_path,
        "execute_motion": execute_motion_enabled,
        "target_pose_topic": control_target_pose_topic,
        "selector_status_topic": selector_status_topic,
        "target_lock_status_topic": target_lock_status_topic,
        "targeting_mode": targeting_mode,
        "run_dir": str(neck_session_root),
    }
    if max_execution_stage:
        control_params["max_execution_stage"] = max_execution_stage
    if approach_ready_vel:
        control_params["approach_ready_vel"] = float(approach_ready_vel)
    if pre_approach_vel:
        control_params["pre_approach_vel"] = float(pre_approach_vel)
    if final_hover_vel:
        control_params["final_hover_vel"] = float(final_hover_vel)
    if final_hover_servo_cmd_t_s:
        control_params["final_hover_servo_cmd_t_s"] = float(final_hover_servo_cmd_t_s)
    if final_hover_servo_max_step_mm:
        control_params["final_hover_servo_max_step_mm"] = float(final_hover_servo_max_step_mm)
    if final_hover_servo_max_step_deg:
        control_params["final_hover_servo_max_step_deg"] = float(final_hover_servo_max_step_deg)

    nodes.append(
        Node(
            package="paus_motion_ros2",
            executable="fairino_control_node",
            name="fairino_control_node",
            output="screen",
            parameters=[control_params],
        )
    )
    return nodes


def _resolve_neck_session_root(neck_cfg: dict, run_id: str) -> Path:
    configured = Path(str(neck_cfg.get("logging_output_dir", "/mnt/data/projects/paus_robot/runs/markerless-neck-surface-design/neck_surface")))
    run_root = configured.parent if configured.name == "neck_surface" else configured
    return run_root / run_id


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory("paus_bringup"))
    default_config_path = str(_resolve_default_config_path(bringup_share))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config_path,
                description="Path to the main YAML config file.",
            ),
            DeclareLaunchArgument(
                "camera_ip",
                default_value="",
                description="Optional camera IP. Leave empty to use the first discovered camera.",
            ),
            DeclareLaunchArgument(
                "camera_index",
                default_value="",
                description="Optional camera index. Leave empty to use auto-selection.",
            ),
            DeclareLaunchArgument(
                "rgb_camera_count",
                default_value="1",
                description="DKAM factory calibration index for RGB intrinsics.",
            ),
            DeclareLaunchArgument(
                "execute_motion",
                default_value="false",
                description="Whether to allow real robot motion. Defaults to dry-run.",
            ),
            DeclareLaunchArgument(
                "run_id",
                default_value="",
                description="Optional runtime session directory name. Defaults to a timestamp.",
            ),
            DeclareLaunchArgument(
                "targeting_mode",
                default_value="",
                description="Target selector mode. Empty uses targeting.mode from config.",
            ),
            DeclareLaunchArgument(
                "max_execution_stage",
                default_value="",
                description="Optional staged motion limit: approach_ready, pre_approach, reorient, or final_hover. Empty uses config.",
            ),
            DeclareLaunchArgument(
                "approach_ready_vel",
                default_value="",
                description="Optional override for approach_ready MoveJ velocity.",
            ),
            DeclareLaunchArgument(
                "pre_approach_vel",
                default_value="",
                description="Optional override for pre_approach/reorient MoveJ velocity.",
            ),
            DeclareLaunchArgument(
                "final_hover_vel",
                default_value="",
                description="Optional override for final_hover velocity when MoveL is used.",
            ),
            DeclareLaunchArgument(
                "final_hover_servo_cmd_t_s",
                default_value="",
                description="Optional ServoCart command period override for final_hover.",
            ),
            DeclareLaunchArgument(
                "final_hover_servo_max_step_mm",
                default_value="",
                description="Optional ServoCart max translation step override for final_hover.",
            ),
            DeclareLaunchArgument(
                "final_hover_servo_max_step_deg",
                default_value="",
                description="Optional ServoCart max rotation step override for final_hover.",
            ),
            DeclareLaunchArgument(
                "python_exec",
                default_value="/home/chen_lab/miniconda3/envs/paus_robot/bin/python3",
                description="Python executable used to run camera_bridge.py. Defaults to the paus_robot Conda env.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
