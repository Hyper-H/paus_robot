from __future__ import annotations

import importlib.util
import sys
import threading
import time
from pathlib import Path


CAMERA_BRIDGE_PATH = Path(__file__).parents[1] / "scripts" / "camera_bridge.py"


def _load_camera_bridge_module():
    module_name = "paus_marker_ros2_camera_bridge_test"
    spec = importlib.util.spec_from_file_location(module_name, CAMERA_BRIDGE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load {CAMERA_BRIDGE_PATH}.")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def test_depth_control_enable_waits_for_runtime_and_fresh_frame() -> None:
    module = _load_camera_bridge_module()
    control = module._DepthControl(False)
    result: dict[str, object] = {}

    worker = threading.Thread(target=lambda: result.update(control.request(True, 1.0)))
    worker.start()
    time.sleep(0.02)
    assert control.requested() is True
    assert worker.is_alive()

    control.mark_runtime(True)
    assert worker.is_alive()
    control.mark_depth_frame()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert result["success"] is True
    assert result["active"] is True


def test_depth_control_disable_waits_for_runtime_to_close() -> None:
    module = _load_camera_bridge_module()
    control = module._DepthControl(True)
    control.mark_runtime(True)
    result: dict[str, object] = {}

    worker = threading.Thread(target=lambda: result.update(control.request(False, 1.0)))
    worker.start()
    time.sleep(0.02)
    assert worker.is_alive()

    control.mark_runtime(False)
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert result["success"] is True
    assert result["active"] is False
