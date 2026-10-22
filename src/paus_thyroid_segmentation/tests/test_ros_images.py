import numpy as np
import pytest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from paus_thyroid_segmentation.imaging import Mono16Policy
from paus_thyroid_segmentation.inference import Prediction
from paus_thyroid_segmentation.node import decode_image, output_images


def test_header_dimensions_encodings_and_overlay():
    bridge = CvBridge()
    source = bridge.cv2_to_imgmsg(np.full((101, 179), 45, np.uint8), encoding="mono8")
    source.header.stamp.sec, source.header.stamp.nanosec = 123, 456
    source.header.frame_id = "ultrasound_image"
    gray, bgr = decode_image(bridge, source, Mono16Policy())
    binary = np.zeros_like(gray)
    binary[20:40, 50:70] = 255
    prediction = Prediction(binary, True, 400, 0.8, 1, 2, 3)
    mask, overlay = output_images(bridge, source, prediction, bgr)
    for message in (mask, overlay):
        assert message.header == source.header
        assert (message.height, message.width) == (101, 179)
    assert mask.encoding == "mono8"
    assert overlay.encoding == "bgr8"
    np.testing.assert_array_equal(bridge.imgmsg_to_cv2(mask), binary)
    rendered = bridge.imgmsg_to_cv2(overlay)
    np.testing.assert_array_equal(rendered[0], bgr[0])
    assert tuple(rendered[20, 50]) == (0, 255, 0)
    mask.header.frame_id = "changed"
    assert source.header.frame_id == "ultrasound_image"
    assert output_images(bridge, source, prediction, bgr, False)[1] is None


@pytest.mark.parametrize("bigendian", [False, True])
def test_mono16_byte_order_and_row_padding(bigendian):
    values = np.array([[0, 2048, 4095], [4095, 2048, 0]], dtype=">u2" if bigendian else "<u2")
    rows = np.zeros((2, 8), np.uint8)
    rows[:, :6] = values.view(np.uint8).reshape(2, 6)
    message = Image(height=2, width=3, encoding="mono16", step=8,
                    is_bigendian=bigendian, data=rows.tobytes())
    gray, _ = decode_image(CvBridge(), message, Mono16Policy("fixed_window", 0, 4095))
    np.testing.assert_array_equal(gray, [[0, 128, 255], [255, 128, 0]])


def test_mono8_padding():
    message = Image(height=2, width=2, encoding="mono8", step=3,
                    data=bytes([1, 2, 99, 3, 4, 99]))
    np.testing.assert_array_equal(decode_image(CvBridge(), message, Mono16Policy())[0], [[1, 2], [3, 4]])


def test_bgr_padding_need_not_be_multiple_of_pixel_size():
    message = Image(height=2, width=1, encoding="bgr8", step=4,
                    data=bytes([0, 0, 255, 99, 255, 0, 0, 99]))
    gray, _ = decode_image(CvBridge(), message, Mono16Policy())
    np.testing.assert_array_equal(gray, [[76], [29]])


@pytest.mark.parametrize("encoding,step,data", [
    ("yuv422", 4, bytes(8)), ("mono8", 1, bytes(2)), ("mono8", 2, bytes(3)),
])
def test_reject_malformed_or_unsupported(encoding, step, data):
    with pytest.raises(ValueError):
        decode_image(CvBridge(), Image(height=2, width=2, encoding=encoding, step=step, data=data),
                     Mono16Policy())
