from __future__ import annotations

# 导入 contextmanager，便于管理相机连接生命周期。
from contextlib import contextmanager
# 导入 dataclass，定义发现到的相机信息结构。
from dataclasses import dataclass
# 导入 Path，便于处理标定输出路径。
from pathlib import Path
# 导入 time，便于桥接层循环取帧时休眠与时间戳记录。
import time
from typing import Any

# 导入 OpenCV，用于 JPEG 编码。
import cv2
# 导入 NumPy，用于图像数组转换。
import numpy as np

# 导入统一的相机标定结果类型。
from .calibration import CameraCalibration, save_camera_calibration


# 保存发现到的相机摘要信息。
@dataclass
class DiscoveredCamera:
    # 在 SDK 中使用的相机索引。
    index: int
    # 相机 IP。
    camera_ip: str


# 保存当前桥接会话中的相机状态。
@dataclass
class CameraRuntime:
    # SDK 使用的相机索引。
    camera_index: int
    # SDK 相机对象句柄。
    camera_obj: Any
    # 当前 RGB 图像宽度。
    rgb_width: int
    # 当前 RGB 图像高度。
    rgb_height: int


# 以懒加载方式导入 DkamSDK，避免在 py310 环境中误导入。
def _require_dkam_sdk() -> Any:
    try:
        import DkamSDK
    except ImportError as exc:
        raise RuntimeError("DkamSDK is unavailable in the current environment.") from exc
    return DkamSDK


# 将 SDK 返回的 bytes IP 转换成可读字符串。
def _decode_camera_ip(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="ignore")
    return str(value)


# 枚举当前局域网中的相机。
def discover_cameras() -> list[DiscoveredCamera]:
    DkamSDK = _require_dkam_sdk()
    camera_count = int(DkamSDK.DiscoverCamera())
    if camera_count > 0:
        DkamSDK.CameraSort(0)
    cameras: list[DiscoveredCamera] = []
    for index in range(camera_count):
        cameras.append(
            DiscoveredCamera(
                index=index,
                camera_ip=_decode_camera_ip(DkamSDK.CameraIP(index)),
            )
        )
    return cameras


# 根据 IP 或索引选择目标相机。
def resolve_camera_index(camera_ip: str | None = None, camera_index: int | None = None) -> int:
    cameras = discover_cameras()
    if camera_index is not None:
        if any(item.index == camera_index for item in cameras):
            return int(camera_index)
        raise RuntimeError(f"Camera index {camera_index} was not found.")
    if camera_ip is not None:
        for camera in cameras:
            if camera.camera_ip == camera_ip:
                return camera.index
        raise RuntimeError(f"Camera IP {camera_ip} was not found.")
    if cameras:
        return cameras[0].index
    raise RuntimeError("No camera discovered in LAN.")


# 读取 RGB 图像尺寸。
def _read_rgb_dimensions(DkamSDK: Any, camera_obj: Any) -> tuple[int, int]:
    if hasattr(DkamSDK, "GetCameraWidth") and hasattr(DkamSDK, "GetCameraHeight"):
        width_buffer = DkamSDK.new_intArray(1)
        height_buffer = DkamSDK.new_intArray(1)
        try:
            width_status = int(DkamSDK.GetCameraWidth(camera_obj, width_buffer, 1))
            height_status = int(DkamSDK.GetCameraHeight(camera_obj, height_buffer, 1))
            if width_status != 0:
                raise RuntimeError(f"GetCameraWidth failed with code {width_status}.")
            if height_status != 0:
                raise RuntimeError(f"GetCameraHeight failed with code {height_status}.")
            width = int(DkamSDK.intArray_getitem(width_buffer, 0))
            height = int(DkamSDK.intArray_getitem(height_buffer, 0))
        finally:
            if hasattr(DkamSDK, "delete_intArray"):
                DkamSDK.delete_intArray(width_buffer)
                DkamSDK.delete_intArray(height_buffer)
        return width, height

    camera_index = int(camera_obj)
    width_address = DkamSDK.GetRegisterAddr(camera_index, b"Width") + 0x100
    height_address = DkamSDK.GetRegisterAddr(camera_index, b"Height") + 0x100
    width_buffer = DkamSDK.new_intArray(1)
    height_buffer = DkamSDK.new_intArray(1)
    try:
        DkamSDK.ReadRegister(camera_index, width_address, width_buffer)
        DkamSDK.ReadRegister(camera_index, height_address, height_buffer)
        width = int(DkamSDK.intArray_getitem(width_buffer, 0))
        height = int(DkamSDK.intArray_getitem(height_buffer, 0))
    finally:
        if hasattr(DkamSDK, "delete_intArray"):
            DkamSDK.delete_intArray(width_buffer)
            DkamSDK.delete_intArray(height_buffer)
    return width, height


# 将 SDK 读取到的厂家参数转换为统一的 camera.yaml 数据结构。
def convert_sdk_calibration_to_camera(
    width: int,
    height: int,
    distortion_coeffs: list[float],
    camera_matrix_values: list[float],
    rotation_matrix: list[float],
    translation_vector: list[float],
    source_path: str = "",
) -> CameraCalibration:
    # 将 9 个内参矩阵元素重排为 3x3 矩阵。
    camera_matrix = [
        camera_matrix_values[0:3],
        camera_matrix_values[3:6],
        camera_matrix_values[6:9],
    ]
    # 返回统一的相机标定结构。
    return CameraCalibration(
        image_width=int(width),
        image_height=int(height),
        camera_matrix=[[float(value) for value in row] for row in camera_matrix],
        dist_coeffs=[float(value) for value in distortion_coeffs],
        reprojection_error=0.0,
        board_rows=0,
        board_cols=0,
        square_size_m=0.0,
        rotation_matrix=[float(value) for value in rotation_matrix] if rotation_matrix else None,
        translation_vector=[float(value) for value in translation_vector] if translation_vector else None,
        source_type="sdk_factory",
        source_path=str(source_path),
    )


# 使用 SDK 读取厂家标定参数。
def read_factory_calibration(DkamSDK: Any, camera_obj: Any, width: int, height: int, camera_count: int = 0) -> CameraCalibration:
    distortion = DkamSDK.new_floatArray(5)
    intrinsic = DkamSDK.new_floatArray(9)
    rotation = DkamSDK.new_floatArray(9)
    translation = DkamSDK.new_floatArray(3)
    try:
        internal_status = int(DkamSDK.GetCamInternelParameter(camera_obj, camera_count, distortion, intrinsic))
        if internal_status != 0:
            raise RuntimeError(f"GetCamInternelParameter failed with code {internal_status}.")
        external_status = int(DkamSDK.GetCamExternelParameter(camera_obj, camera_count, rotation, translation))
        if external_status != 0:
            raise RuntimeError(f"GetCamExternelParameter failed with code {external_status}.")
        distortion_values = [float(DkamSDK.floatArray_getitem(distortion, index)) for index in range(5)]
        intrinsic_values = [float(DkamSDK.floatArray_getitem(intrinsic, index)) for index in range(9)]
        rotation_values = [float(DkamSDK.floatArray_getitem(rotation, index)) for index in range(9)]
        translation_values = [float(DkamSDK.floatArray_getitem(translation, index)) for index in range(3)]
    finally:
        DkamSDK.delete_floatArray(distortion)
        DkamSDK.delete_floatArray(intrinsic)
        DkamSDK.delete_floatArray(rotation)
        DkamSDK.delete_floatArray(translation)
    return convert_sdk_calibration_to_camera(
        width=width,
        height=height,
        distortion_coeffs=distortion_values,
        camera_matrix_values=intrinsic_values,
        rotation_matrix=rotation_values,
        translation_vector=translation_values,
    )


# 基于当前已连接会话读取厂家标定参数。
def read_runtime_calibration(runtime: CameraRuntime, camera_count: int = 0) -> CameraCalibration:
    DkamSDK = _require_dkam_sdk()
    return read_factory_calibration(
        DkamSDK=DkamSDK,
        camera_obj=runtime.camera_obj,
        width=runtime.rgb_width,
        height=runtime.rgb_height,
        camera_count=camera_count,
    )


# 管理相机的创建、连接、开流和关闭流程。
@contextmanager
def open_camera_runtime(camera_ip: str | None = None, camera_index: int | None = None) -> CameraRuntime:
    DkamSDK = _require_dkam_sdk()
    selected_index = resolve_camera_index(camera_ip=camera_ip, camera_index=camera_index)
    camera_obj = DkamSDK.CreateCamera(selected_index)
    connect_status = int(DkamSDK.CameraConnect(camera_obj))
    if connect_status != 0:
        DkamSDK.DestroyCamera(camera_obj)
        raise RuntimeError(f"CameraConnect failed with code {connect_status}.")
    try:
        width, height = _read_rgb_dimensions(DkamSDK, camera_obj)
        trigger_status = int(DkamSDK.SetRGBTriggerMode(camera_obj, 0))
        if trigger_status != 0:
            raise RuntimeError(f"SetRGBTriggerMode failed with code {trigger_status}.")
        stream_status = int(DkamSDK.StreamOn(camera_obj, 2))
        if stream_status != 0:
            raise RuntimeError(f"StreamOn failed with code {stream_status}.")
        acquisition_status = int(DkamSDK.AcquisitionStart(camera_obj))
        if acquisition_status != 0:
            raise RuntimeError(f"AcquisitionStart failed with code {acquisition_status}.")
        yield CameraRuntime(camera_index=selected_index, camera_obj=camera_obj, rgb_width=width, rgb_height=height)
    finally:
        try:
            DkamSDK.StreamOff(camera_obj, 2)
        except Exception:
            pass
        try:
            DkamSDK.CameraDisconnect(camera_obj)
        except Exception:
            pass
        try:
            DkamSDK.DestroyCamera(camera_obj)
        except Exception:
            pass


# 在当前连接的相机上抓取一帧 RGB 图像。
def capture_rgb_frame(runtime: CameraRuntime, timeout_us: int = 3_000_000) -> np.ndarray:
    DkamSDK = _require_dkam_sdk()
    photo_info = DkamSDK.PhotoInfoCSharp()
    pixel_count = runtime.rgb_width * runtime.rgb_height * 3
    rgb_buffer = bytes(pixel_count)
    DkamSDK.FlushBuffer(runtime.camera_obj, 2)
    capture_status = int(DkamSDK.TimeoutCaptureCSharp(runtime.camera_obj, 2, photo_info, rgb_buffer, pixel_count, timeout_us))
    if capture_status != 0:
        raise RuntimeError(f"TimeoutCaptureCSharp failed with code {capture_status}.")
    if hasattr(DkamSDK, "RawdataToRgb888CSharp"):
        DkamSDK.RawdataToRgb888CSharp(runtime.camera_obj, photo_info, rgb_buffer, pixel_count)
    rgb_array = np.frombuffer(rgb_buffer, dtype=np.uint8).reshape((runtime.rgb_height, runtime.rgb_width, 3))
    return cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)


# 启动时从当前相机读取厂家标定参数并写入 camera.yaml。
def export_factory_calibration_to_yaml(
    output_path: str | Path,
    camera_ip: str | None = None,
    camera_index: int | None = None,
    camera_count: int = 0,
) -> CameraCalibration:
    DkamSDK = _require_dkam_sdk()
    with open_camera_runtime(camera_ip=camera_ip, camera_index=camera_index) as runtime:
        calibration = read_factory_calibration(
            DkamSDK=DkamSDK,
            camera_obj=runtime.camera_obj,
            width=runtime.rgb_width,
            height=runtime.rgb_height,
            camera_count=camera_count,
        )
    output_path = Path(output_path)
    calibration.source_path = str(output_path)
    save_camera_calibration(calibration, output_path)
    return calibration


# 将当前会话中的厂家标定参数保存为 camera.yaml。
def save_runtime_calibration_to_yaml(runtime: CameraRuntime, output_path: str | Path, camera_count: int = 0) -> CameraCalibration:
    calibration = read_runtime_calibration(runtime, camera_count=camera_count)
    output_path = Path(output_path)
    calibration.source_path = str(output_path)
    save_camera_calibration(calibration, output_path)
    return calibration


# 将一帧图像编码为 JPEG，供桥接层发送。
def encode_bgr_frame_to_jpeg(image_bgr: np.ndarray, jpeg_quality: int = 90) -> bytes:
    success, encoded = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not success:
        raise RuntimeError("Failed to encode image to JPEG.")
    return encoded.tobytes()
