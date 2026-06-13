from __future__ import annotations

import re
from typing import Any


def unwrap_runtime_text(message: Any) -> str:
    text = "" if message is None else str(message).strip()
    for prefix in ("RuntimeError(", "ValueError(", "Exception("):
        if text.startswith(prefix) and text.endswith(")"):
            inner = text[len(prefix) : -1].strip()
            if (inner.startswith("'") and inner.endswith("'")) or (inner.startswith('"') and inner.endswith('"')):
                return inner[1:-1]
            return inner
    return text


def classify_operator_message(message: Any) -> dict[str, str]:
    raw = "" if message is None else str(message)
    text = unwrap_runtime_text(raw)
    lower = text.lower()
    code = "unknown"
    operator = text or "暂无状态信息"

    if "chessboard_not_found" in lower or "chessboard was not detected" in lower or ("棋盘" in lower and "未检测" in lower):
        code = "chessboard_not_detected"
        operator = "棋盘未检测到，请调整标定板姿态、光照或视野位置。"
    elif "no fresh image" in lower or "fresh image" in lower:
        code = "no_fresh_image"
        operator = "没有收到新图像，请检查相机桥接和图像话题。"
    elif "reprojection error" in lower and "exceeds" in lower:
        code = "reprojection_error_high"
        value = _first_number(text)
        operator = f"重投影误差过大{f'（{value}px）' if value else ''}，建议调整角度或去除该点。"
    elif "board margin" in lower or ("margin" in lower and ("below" in lower or "too" in lower)):
        code = "board_margin_low"
        operator = "棋盘离图像边缘过近，请让棋盘更完整地进入视野。"
    elif "service is unavailable" in lower or "service unavailable" in lower:
        code = "service_unavailable"
        operator = "标定节点服务未连接，请确认 eye_to_hand_calibration_node 已启动。"
    elif "service timed out" in lower or "timed out" in lower:
        code = "service_timeout"
        operator = "服务调用超时，节点可能正在执行长任务或未响应。"
    elif "camera calibration" in lower and ("unavailable" in lower or "missing" in lower):
        code = "camera_calibration_missing"
        operator = "相机内参不可用，请检查 camera.yaml 是否存在。"
    elif "solvepnp" in lower:
        code = "pnp_failed"
        operator = "PnP 求解失败，请检查棋盘角点检测质量和相机内参。"
    elif "semi-auto calibration is already running" in lower:
        code = "run_already_active"
        operator = "半自动标定正在运行中。"
    elif "stop is not supported" in lower:
        code = "stop_not_supported"
        operator = "当前后端不支持立即停止；如需立刻停止运动，请使用实体急停或终端中断。"
    elif raw.startswith("RuntimeError(") or raw.startswith("ValueError("):
        code = "runtime_error"
        operator = text or "运行失败。"

    return {"code": code, "message": operator, "raw": raw, "clean": text}


def operator_message(message: Any) -> str:
    return classify_operator_message(message)["message"]


def _first_number(text: str) -> str | None:
    match = re.search(r"[-+]?\d+(?:\.\d+)?", text)
    if not match:
        return None
    return match.group(0)
