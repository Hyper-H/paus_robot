"""Real DDS transport with an explicitly test-only NumPy engine, NOT a thyroid model."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from paus_thyroid_segmentation.imaging import letterbox, restore_mask
from paus_thyroid_segmentation.inference import Prediction
from paus_thyroid_segmentation.node import IMAGE_QOS, STATUS_QOS, SegmentationNode
from paus_thyroid_segmentation.validation import synthetic_pixels


class TestOnlyEngine:
    __test__ = False
    device = "test_only_numpy"

    def __init__(self, delay=0.04):
        self.delay = delay
        self.active = threading.Lock()
        self.calls = 0

    def predict(self, gray):
        assert self.active.acquire(blocking=False), "concurrent model calls"
        try:
            begin = time.perf_counter()
            batch, geometry = letterbox(gray)
            prepared = time.perf_counter()
            time.sleep(self.delay)
            self.calls += 1
            binary = (batch[0, 0] >= 0.5).astype(np.uint8) * 255
            inferred = time.perf_counter()
            mask = restore_mask(binary, geometry)
            area = int(np.count_nonzero(mask))
            return Prediction(mask, area > 0, area, 0.5 if area else 0,
                              (prepared - begin) * 1000, (inferred - prepared) * 1000,
                              (time.perf_counter() - inferred) * 1000)
        finally:
            self.active.release()

    def memory_metrics(self):
        return {"gpu_allocated_mb": 0, "gpu_peak_allocated_mb": 0}


@pytest.fixture
def pipeline():
    rclpy.init()
    engine = TestOnlyEngine()
    node = SegmentationNode(engine=engine, namespace="/thyroid_test",
                            parameter_overrides=[
                                Parameter("input_topic", value="/thyroid_test/input"),
                                Parameter("input_mode", value="image"),
                                Parameter("mask_topic", value="/thyroid_test/mask"),
                                Parameter("overlay_topic", value="/thyroid_test/overlay"),
                                Parameter("status_topic", value="/thyroid_test/status"),
                                Parameter("stale_timeout_sec", value=0.3),
                            ])
    observer = Node("thyroid_test_observer")
    outputs, overlays, statuses = [], [], []
    observer.create_subscription(Image, "/thyroid_test/mask", outputs.append, IMAGE_QOS)
    observer.create_subscription(Image, "/thyroid_test/overlay", overlays.append, IMAGE_QOS)
    observer.create_subscription(DiagnosticArray, "/thyroid_test/status", statuses.append, STATUS_QOS)
    publisher = observer.create_publisher(Image, "/thyroid_test/input", IMAGE_QOS)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)

    def spin_until(predicate, timeout=4):
        stop = time.perf_counter() + timeout
        while not predicate() and time.perf_counter() < stop:
            executor.spin_once(timeout_sec=0.01)
        assert predicate(), "timed out waiting for ROS condition"

    spin_until(lambda: publisher.get_subscription_count() > 0
               and node.mask_publisher.get_subscription_count() > 0
               and node.status_publisher.get_subscription_count() > 0)
    try:
        yield node, observer, publisher, outputs, overlays, statuses, executor, spin_until
    finally:
        node.destroy_node()
        observer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def metrics(statuses):
    return [{entry.key: entry.value for entry in array.status[0].values} for array in statuses]


def save_report(name, statuses, sent=None):
    records = [record for record in metrics(statuses) if float(record.get("end_to_end_ms", -1)) >= 0]
    if not records:
        return
    keys = ("preprocess_ms", "model_ms", "postprocess_ms", "end_to_end_ms")
    report = {
        "engine": "TEST-ONLY NumPy threshold + deliberate 40 ms sleep; NOT trained model",
        "sent": sent, "observed_status_frames": len(records),
        "latest": records[-1],
        "latencies": {key: {
            "mean_ms": float(np.mean([float(r[key]) for r in records])),
            "p95_ms": float(np.percentile([float(r[key]) for r in records], 95)),
        } for key in keys},
    }
    print(json.dumps(report))
    directory = os.environ.get("TG_TEST_REPORT_DIR")
    if directory:
        destination = Path(directory)
        destination.mkdir(parents=True, exist_ok=True)
        (destination / name).write_text(json.dumps(report, indent=2))


def test_synthetic_dds_latest_frame_headers_and_status(pipeline):
    node, observer, publisher, outputs, overlays, statuses, executor, spin_until = pipeline
    bridge = CvBridge()
    sent = {}
    begin = time.perf_counter()
    next_send = begin
    while time.perf_counter() - begin < 2:
        now = time.perf_counter()
        if now >= next_send:
            encoding = ("mono8", "rgb8", "bgr8")[len(sent) % 3]
            image = bridge.cv2_to_imgmsg(synthetic_pixels(179, 113, encoding), encoding=encoding)
            image.header.stamp = observer.get_clock().now().to_msg()
            image.header.frame_id = f"test_{len(sent)}"
            sent[(image.header.stamp.sec, image.header.stamp.nanosec)] = image
            publisher.publish(image)
            next_send += 1 / 100
        executor.spin_once(timeout_sec=0.001)
    spin_until(lambda: len(outputs) >= 10 and node.mailbox.counts()["pending"] == 0)
    last_id = list(sent.values())[-1].header.frame_id
    spin_until(lambda: any(message.header.frame_id == last_id for message in outputs))
    assert node.mailbox.counts()["overwritten"] > 0
    assert len(outputs) < len(sent)
    for message in outputs:
        source = sent[(message.header.stamp.sec, message.header.stamp.nanosec)]
        assert message.header == source.header
        assert (message.width, message.height, message.encoding) == (179, 113, "mono8")
        assert set(np.unique(bridge.imgmsg_to_cv2(message))) <= {0, 255}
    assert overlays and all(message.encoding == "bgr8" for message in overlays)
    records = [record for record in metrics(statuses) if float(record.get("end_to_end_ms", -1)) >= 0]
    assert records and all(record["valid"] == "true" for record in records)
    assert all(float(record["end_to_end_ms"]) >= float(record["model_ms"]) for record in records)
    assert node.engine.calls >= len(outputs)
    save_report("synthetic_transport.json", statuses, len(sent))


def test_invalid_encoding_then_recovery_and_stale(pipeline):
    node, observer, publisher, outputs, overlays, statuses, executor, spin_until = pipeline
    invalid = Image(height=2, width=2, encoding="8UC1", step=2, data=bytes(4))
    invalid.header.frame_id = "bad_encoding"
    publisher.publish(invalid)
    spin_until(lambda: any(array.status[0].level == DiagnosticStatus.ERROR for array in statuses))
    assert not outputs
    error = next(array for array in statuses if array.status[0].level == DiagnosticStatus.ERROR)
    assert error.header.frame_id == "bad_encoding"
    assert metrics([error])[0]["valid"] == "false"
    valid = CvBridge().cv2_to_imgmsg(np.full((31, 17), 255, np.uint8), encoding="mono8")
    valid.header.stamp = observer.get_clock().now().to_msg()
    publisher.publish(valid)
    spin_until(lambda: len(outputs) == 1)
    spin_until(lambda: any(array.status[0].level == DiagnosticStatus.STALE for array in statuses))
    assert any(record.get("valid") == "true" for record in metrics(statuses))


def test_reference_comparison_cli_over_real_dds(pipeline, tmp_path):
    node, observer, publisher, outputs, overlays, statuses, executor, spin_until = pipeline
    gray = np.full((224, 224), 32, np.uint8)
    gray[50:100, 80:130] = 200
    expected = np.zeros((224, 224), np.uint8)
    expected[50:100, 80:130] = 255
    reference = tmp_path / "test_only_reference.npz"
    np.savez(reference, gray=gray, mask=expected)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[1]) + ":" + env.get("PYTHONPATH", "")
    process = subprocess.Popen(
        [sys.executable, "-c", "from paus_thyroid_segmentation.validation import main; main()",
         "compare-ros", "--reference", str(reference),
         "--input-topic", "/thyroid_test/input", "--mask-topic", "/thyroid_test/mask"],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=env)
    try:
        deadline = time.perf_counter() + 12
        while process.poll() is None and time.perf_counter() < deadline:
            executor.spin_once(timeout_sec=0.01)
        assert process.poll() is not None, "comparison process timed out"
        stdout, stderr = process.communicate(timeout=2)
        assert process.returncode == 0, stderr
        assert json.loads(stdout)["ros_mask_equal"] is True
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)


def test_existing_real_rosbag_playback(pipeline):
    bag = os.environ.get("TG_TEST_BAG")
    if not bag:
        pytest.skip("set TG_TEST_BAG and source the UltrasoundFrame overlay for actual bag playback")
    pytest.importorskip("paus_interfaces.msg._ultrasound_frame")
    from paus_thyroid_segmentation.frame_bridge import FrameBridge

    node, observer, publisher, outputs, overlays, statuses, executor, spin_until = pipeline
    frame_bridge = FrameBridge(parameter_overrides=[
        Parameter("frame_topic", value="/thyroid_test/frame"),
        Parameter("image_topic", value="/thyroid_test/input"),
    ])
    executor.add_node(frame_bridge)
    process = subprocess.Popen(
        [sys.executable, "/opt/ros/humble/bin/ros2", "bag", "play", bag,
         "--rate", "2", "--topics", "/ultrasound/us/frame",
         "--remap", "/ultrasound/us/frame:=/thyroid_test/frame"],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    try:
        begin = time.perf_counter()
        while time.perf_counter() - begin < 4:
            executor.spin_once(timeout_sec=0.01)
        spin_until(lambda: len(outputs) >= 10)
        assert process.poll() is None, "bag player exited unexpectedly"
        assert all(message.width == 179 and message.header.frame_id == "ultrasound_image"
                   for message in outputs)
        assert all(message.header.stamp.sec > 0 for message in outputs)
        save_report("real_bag_transport.json", statuses)
    finally:
        process.send_signal(signal.SIGINT)
        try:
            _, error = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            _, error = process.communicate(timeout=5)
        executor.remove_node(frame_bridge)
        frame_bridge.destroy_node()
