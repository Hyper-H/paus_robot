"""AG 项目的公共导出接口。"""

# 导入配置加载函数，供脚本和测试统一使用。
from .config import load_config
# 导入标定相关的数据结构和函数。
from .calibration import CameraCalibration, CalibrationResult, calibrate_camera_from_directory, load_camera_calibration, save_camera_calibration
# 导入检测相关的数据结构和函数。
from .detection import (
    MarkerDetection,
    PipelineResult,
    detect_marker,
    export_debug_images,
    render_visualization,
    result_to_dict,
    save_result_json,
)
# 导入流水线处理和批处理相关函数。
from .pipeline import (
    SUPPORTED_IMAGE_EXTENSIONS,
    build_summary_record,
    make_output_subdir_name,
    process_image_array,
    process_image_file,
    write_summary_csv,
    write_summary_json,
)
# 导入桥接协议相关函数。
from .bridge_protocol import pack_frame_packet, recv_frame_packet
# 导入 Linux FAIRINO SDK 适配层。
from .fairino_linux_client import FairinoLinuxClient
# 导入 SDK 相机适配层。
from .sdk_camera import (
    DiscoveredCamera,
    capture_rgb_frame,
    discover_cameras,
    encode_bgr_frame_to_jpeg,
    export_factory_calibration_to_yaml,
    open_camera_runtime,
    read_runtime_calibration,
    resolve_camera_index,
    save_runtime_calibration_to_yaml,
)
# 导入位姿估计与接近目标规划相关的数据结构和函数。
from .pose import ApproachPlan, MarkerPose, build_approach_plan, estimate_marker_pose
# 导入机械臂控制决策工具。
from .control import ControlDecision, build_approach_decision, is_finite_point, point_m_to_mm
# 导入坐标变换与外参读写工具。
from .transforms import (
    EyeToHandCalibrationSolution,
    Transform3D,
    average_transform_matrices,
    invert_transform_matrix,
    load_eye_to_hand_solution,
    make_transform_matrix,
    make_transform_struct,
    quaternion_xyzw_to_rotation_matrix,
    rotation_matrix_to_quaternion_xyzw,
    rpy_deg_to_rotation_matrix,
    save_eye_to_hand_solution,
    split_transform_matrix,
)

# 定义包对外暴露的名字，方便在脚本里直接按需导入。
__all__ = [
    "ApproachPlan",
    "CalibrationResult",
    "CameraCalibration",
    "ControlDecision",
    "EyeToHandCalibrationSolution",
    "MarkerDetection",
    "MarkerPose",
    "PipelineResult",
    "DiscoveredCamera",
    "SUPPORTED_IMAGE_EXTENSIONS",
    "Transform3D",
    "average_transform_matrices",
    "build_approach_decision",
    "build_approach_plan",
    "build_summary_record",
    "calibrate_camera_from_directory",
    "capture_rgb_frame",
    "detect_marker",
    "discover_cameras",
    "encode_bgr_frame_to_jpeg",
    "estimate_marker_pose",
    "export_factory_calibration_to_yaml",
    "export_debug_images",
    "load_camera_calibration",
    "load_eye_to_hand_solution",
    "load_config",
    "make_output_subdir_name",
    "make_transform_matrix",
    "make_transform_struct",
    "open_camera_runtime",
    "pack_frame_packet",
    "point_m_to_mm",
    "process_image_array",
    "process_image_file",
    "quaternion_xyzw_to_rotation_matrix",
    "read_runtime_calibration",
    "recv_frame_packet",
    "resolve_camera_index",
    "render_visualization",
    "result_to_dict",
    "rotation_matrix_to_quaternion_xyzw",
    "rpy_deg_to_rotation_matrix",
    "save_camera_calibration",
    "save_eye_to_hand_solution",
    "save_result_json",
    "save_runtime_calibration_to_yaml",
    "split_transform_matrix",
    "is_finite_point",
    "FairinoLinuxClient",
    "write_summary_csv",
    "write_summary_json",
]
