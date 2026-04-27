"""PAUS Robot 运动控制相关的公共导出接口。"""

from .control_logic import ControlDecision, build_approach_decision, compute_normal_alignment_error_deg, is_finite_point, point_m_to_mm
from .fairino_linux_client import FairinoLinuxClient

__all__ = [
    "ControlDecision",
    "FairinoLinuxClient",
    "build_approach_decision",
    "compute_normal_alignment_error_deg",
    "is_finite_point",
    "point_m_to_mm",
]
