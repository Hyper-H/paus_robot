import numpy as np

from paus_ui.overlay import BoardOverlayDetector


class _Calibration:
    camera_matrix = [[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]]
    dist_coeffs = [0.0, 0.0, 0.0, 0.0, 0.0]


def test_overlay_failure_payload_is_operator_friendly() -> None:
    detector = BoardOverlayDetector(board_rows=6, board_cols=9, square_size_m=0.02, camera_calibration=_Calibration())
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    result = detector.estimate(image)

    assert result.detected is False
    assert result.reason_code == "chessboard_not_detected"
    assert result.operator_message
    assert "RuntimeError" not in result.operator_message


def test_raw_render_does_not_draw_metrics() -> None:
    detector = BoardOverlayDetector(board_rows=6, board_cols=9, square_size_m=0.02, camera_calibration=_Calibration())
    image = np.zeros((480, 640, 3), dtype=np.uint8)

    rendered, _ = detector.render(image, mode="raw", image_sequence=7, show_axes=False)

    assert np.array_equal(rendered, image)
