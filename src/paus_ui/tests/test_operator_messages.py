from paus_ui.operator_messages import classify_operator_message, unwrap_runtime_text


def test_operator_message_translates_common_detection_errors() -> None:
    info = classify_operator_message("RuntimeError('Chessboard was not detected in the latest image.')")

    assert info["code"] == "chessboard_not_detected"
    assert "棋盘未检测到" in info["message"]
    assert "RuntimeError" not in info["message"]


def test_operator_message_translates_service_errors() -> None:
    info = classify_operator_message("Service is unavailable: /eye_to_hand/run_semi_auto_calibration")

    assert info["code"] == "service_unavailable"
    assert "标定节点服务未连接" in info["message"]


def test_unwrap_runtime_text_removes_python_wrapper() -> None:
    assert unwrap_runtime_text("RuntimeError('No fresh image arrived within 2.000s.')") == "No fresh image arrived within 2.000s."
