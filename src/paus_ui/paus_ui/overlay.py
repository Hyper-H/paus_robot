from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

from paus_perception import CameraCalibration, make_transform_matrix, rotation_matrix_to_rpy_deg

from .operator_messages import classify_operator_message


@dataclass
class BoardOverlayResult:
    detected: bool
    reason: str | None
    reason_code: str | None
    operator_message: str | None
    reprojection_error_px: float | None
    board_margin_px: float | None
    camera_to_board_matrix: np.ndarray | None
    camera_to_board_translation_m: list[float] | None
    camera_to_board_rotation_rpy_deg: list[float] | None
    board_angle_deg: float | None
    corners: np.ndarray | None
    rvec: np.ndarray | None
    tvec: np.ndarray | None

    def to_payload(self, *, image_sequence: int | None = None) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "detected": self.detected,
            "reason": self.reason,
            "reason_code": self.reason_code,
            "operator_message": self.operator_message,
            "image_sequence": image_sequence,
            "reprojection_error_px": self.reprojection_error_px,
            "board_margin_px": self.board_margin_px,
            "camera_to_board_translation_m": self.camera_to_board_translation_m,
            "camera_to_board_rotation_rpy_deg": self.camera_to_board_rotation_rpy_deg,
            "board_angle_deg": self.board_angle_deg,
        }
        if self.camera_to_board_matrix is not None:
            payload["camera_to_board_matrix"] = self.camera_to_board_matrix.tolist()
        return payload


class BoardOverlayDetector:
    def __init__(
        self,
        *,
        board_rows: int,
        board_cols: int,
        square_size_m: float,
        camera_calibration: CameraCalibration,
    ) -> None:
        self.board_rows = int(board_rows)
        self.board_cols = int(board_cols)
        self.square_size_m = float(square_size_m)
        self.camera_calibration = camera_calibration
        self.camera_matrix = np.asarray(camera_calibration.camera_matrix, dtype=np.float64)
        self.dist_coeffs = np.asarray(camera_calibration.dist_coeffs, dtype=np.float64)
        self.object_points = self._build_board_object_points()

    def _build_board_object_points(self) -> np.ndarray:
        object_points = np.zeros((self.board_rows * self.board_cols, 3), np.float32)
        object_points[:, :2] = np.mgrid[0 : self.board_cols, 0 : self.board_rows].T.reshape(-1, 2)
        object_points *= self.square_size_m
        return object_points

    def estimate(self, image_bgr: np.ndarray) -> BoardOverlayResult:
        try:
            return self._estimate_or_raise(image_bgr)
        except Exception as exc:
            info = classify_operator_message(repr(exc))
            return BoardOverlayResult(
                detected=False,
                reason=info["clean"],
                reason_code=info["code"],
                operator_message=info["message"],
                reprojection_error_px=None,
                board_margin_px=None,
                camera_to_board_matrix=None,
                camera_to_board_translation_m=None,
                camera_to_board_rotation_rpy_deg=None,
                board_angle_deg=None,
                corners=None,
                rvec=None,
                tvec=None,
            )

    def _estimate_or_raise(self, image_bgr: np.ndarray) -> BoardOverlayResult:
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (self.board_cols, self.board_rows))
        if not found:
            raise RuntimeError("Chessboard was not detected.")

        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        success, rvec, tvec = cv2.solvePnP(self.object_points, refined, self.camera_matrix, self.dist_coeffs)
        if not success:
            raise RuntimeError("solvePnP failed.")

        projected_points, _ = cv2.projectPoints(self.object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        reprojection_error_px = float(np.sqrt(np.mean(np.square(projected_points.reshape(-1, 2) - refined.reshape(-1, 2)))))
        corner_points = refined.reshape(-1, 2)
        height, width = gray.shape[:2]
        board_margin_px = float(
            min(
                np.min(corner_points[:, 0]),
                np.min(corner_points[:, 1]),
                width - 1.0 - np.max(corner_points[:, 0]),
                height - 1.0 - np.max(corner_points[:, 1]),
            )
        )
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        transform = make_transform_matrix(tvec.reshape(3), rotation_matrix)
        normal = rotation_matrix[:, 2]
        board_angle_deg = float(np.degrees(np.arccos(np.clip(abs(float(normal[2])), 0.0, 1.0))))
        return BoardOverlayResult(
            detected=True,
            reason=None,
            reason_code=None,
            operator_message=None,
            reprojection_error_px=reprojection_error_px,
            board_margin_px=board_margin_px,
            camera_to_board_matrix=transform,
            camera_to_board_translation_m=[float(value) for value in tvec.reshape(3).tolist()],
            camera_to_board_rotation_rpy_deg=rotation_matrix_to_rpy_deg(rotation_matrix),
            board_angle_deg=board_angle_deg,
            corners=refined,
            rvec=rvec,
            tvec=tvec,
        )

    def render(
        self,
        image_bgr: np.ndarray,
        *,
        mode: str = "overlay",
        image_sequence: int | None = None,
        show_axes: bool = True,
    ) -> tuple[np.ndarray, BoardOverlayResult]:
        mode = mode if mode in {"raw", "overlay", "pose"} else "overlay"
        result = self.estimate(image_bgr)
        output = image_bgr.copy()
        if mode == "raw":
            return output, result

        if result.detected and result.corners is not None:
            if mode == "overlay":
                cv2.drawChessboardCorners(output, (self.board_cols, self.board_rows), result.corners, True)
            self._draw_board_outline(output, result.corners)
            if show_axes and result.rvec is not None and result.tvec is not None:
                axis_length = max(self.square_size_m * 3.0, 0.02)
                try:
                    cv2.drawFrameAxes(output, self.camera_matrix, self.dist_coeffs, result.rvec, result.tvec, axis_length)
                    self._draw_axis_labels(output, result.rvec, result.tvec, axis_length)
                except Exception:
                    pass
        else:
            self._draw_failure_hint(output, result)
        self._draw_metrics(output, result, image_sequence=image_sequence, show_axes=show_axes)
        return output, result

    def _draw_axis_labels(self, image_bgr: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, axis_length: float) -> None:
        axis_points = np.array(
            [
                [axis_length, 0.0, 0.0],
                [0.0, axis_length, 0.0],
                [0.0, 0.0, axis_length],
            ],
            dtype=np.float32,
        )
        projected, _ = cv2.projectPoints(axis_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        for label, point, color in zip(
            ("X", "Y", "Z"),
            projected.reshape(-1, 2),
            ((0, 64, 255), (0, 180, 80), (255, 128, 0)),
        ):
            x, y = int(point[0]), int(point[1])
            cv2.putText(image_bgr, label, (x + 6, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

    def _draw_board_outline(self, image_bgr: np.ndarray, corners: np.ndarray) -> None:
        points = corners.reshape(self.board_rows, self.board_cols, 2)
        outline = np.array(
            [
                points[0, 0],
                points[0, -1],
                points[-1, -1],
                points[-1, 0],
            ],
            dtype=np.int32,
        )
        cv2.polylines(image_bgr, [outline], isClosed=True, color=(34, 220, 92), thickness=3)

    def _draw_failure_hint(self, image_bgr: np.ndarray, result: BoardOverlayResult) -> None:
        message = result.operator_message or "Chessboard not detected"
        overlay = image_bgr.copy()
        cv2.rectangle(overlay, (20, image_bgr.shape[0] - 92), (min(image_bgr.shape[1] - 20, 760), image_bgr.shape[0] - 24), (22, 28, 34), -1)
        cv2.addWeighted(overlay, 0.7, image_bgr, 0.3, 0.0, image_bgr)
        cv2.putText(image_bgr, message[:64], (40, image_bgr.shape[0] - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 210, 255), 2, cv2.LINE_AA)

    def _draw_metrics(self, image_bgr: np.ndarray, result: BoardOverlayResult, *, image_sequence: int | None, show_axes: bool) -> None:
        lines = [f"image_sequence: {image_sequence if image_sequence is not None else '-'}"]
        if result.detected:
            t = result.camera_to_board_translation_m or [0.0, 0.0, 0.0]
            lines.extend(
                [
                    f"reprojection_error_px: {result.reprojection_error_px:.3f}",
                    f"board_margin_px: {result.board_margin_px:.1f}",
                    f"T_camera_board: x={t[0]:.3f} y={t[1]:.3f} z={t[2]:.3f} m",
                    f"board_angle_deg: {result.board_angle_deg:.2f}",
                    f"axes: {'on' if show_axes else 'off'}",
                ]
            )
        else:
            lines.append(f"detected: false ({result.reason_code or 'unknown'})")

        line_height = 24
        width = min(max(560, int(image_bgr.shape[1] * 0.42)), image_bgr.shape[1] - 20)
        height = 18 + line_height * len(lines)
        overlay = image_bgr.copy()
        cv2.rectangle(overlay, (12, 12), (12 + width, 12 + height), (15, 22, 33), -1)
        cv2.addWeighted(overlay, 0.72, image_bgr, 0.28, 0.0, image_bgr)
        for index, line in enumerate(lines):
            cv2.putText(
                image_bgr,
                line,
                (26, 42 + line_height * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (238, 247, 246),
                2,
                cv2.LINE_AA,
            )


def make_placeholder_image(message: str, *, width: int = 1280, height: int = 720) -> np.ndarray:
    image = np.full((height, width, 3), 244, dtype=np.uint8)
    cv2.rectangle(image, (0, 0), (width - 1, height - 1), (218, 224, 232), 2)
    cv2.putText(image, message, (48, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (85, 96, 110), 2, cv2.LINE_AA)
    return image


def encode_jpeg(image_bgr: np.ndarray, *, quality: int = 85) -> bytes:
    ok, encoded = cv2.imencode(".jpg", image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    if not ok:
        raise RuntimeError("Failed to encode JPEG.")
    return bytes(encoded)
