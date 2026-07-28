"""Tests for the offline camera and IMU latency analysis tool."""

import importlib.util
import math
import os
from pathlib import Path

import cv2
import numpy as np
import pytest
from rclpy.serialization import serialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu


os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
TOOLS_DIRECTORY = Path(__file__).resolve().parents[2] / 'tools'
MODULE_PATH = TOOLS_DIRECTORY / 'analyze_imu_camera_bag.py'
SPEC = importlib.util.spec_from_file_location('analyze_imu_camera_bag', MODULE_PATH)
analysis = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(analysis)


def quaternion_for_rotation(axis, angle):
    axis = np.asarray(axis, dtype=np.float64)
    axis /= np.linalg.norm(axis)
    half_angle = angle * 0.5
    xyz = axis * math.sin(half_angle)
    return np.array([xyz[0], xyz[1], xyz[2], math.cos(half_angle)])


def make_feature_image():
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    generator = np.random.default_rng(7)
    for x, y in generator.integers([15, 15], [305, 225], size=(80, 2)):
        cv2.circle(image, (int(x), int(y)), 3, (255, 255, 255), -1)
    return image


def make_topic_metadata(name, message_type):
    return rosbag2_py.TopicMetadata(
        id=0,
        name=name,
        type=message_type,
        serialization_format='cdr',
    )


def write_test_bag(path, include_imu=True):
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    writer.create_topic(
        make_topic_metadata(analysis.IMAGE_TOPIC, analysis.IMAGE_TYPE)
    )
    if include_imu:
        writer.create_topic(
            make_topic_metadata(analysis.IMU_TOPIC, analysis.IMU_TYPE)
        )

    image = CompressedImage()
    image.header.stamp.sec = 10
    image.format = 'jpeg'
    image.data = cv2.imencode('.jpg', make_feature_image())[1].tobytes()
    writer.write(
        analysis.IMAGE_TOPIC,
        serialize_message(image),
        10_000_000_000,
    )
    if include_imu:
        imu = Imu()
        imu.header.stamp.sec = 10
        imu.orientation.w = 1.0
        writer.write(
            analysis.IMU_TOPIC,
            serialize_message(imu),
            10_000_000_000,
        )
    writer.close()


def test_quaternion_angular_velocity_and_xy_magnitude():
    previous = quaternion_for_rotation((1.0, 0.0, 0.0), 0.0)
    current = quaternion_for_rotation((1.0, 0.0, 0.0), 0.2)

    velocity = analysis.quaternion_angular_velocity(previous, current, 0.1)

    assert velocity == pytest.approx((2.0, 0.0, 0.0), abs=1.0e-9)
    assert math.hypot(velocity[0], velocity[1]) == pytest.approx(2.0)


def test_robust_normalize_clips_outliers_and_handles_constant_data():
    values = np.concatenate((np.linspace(0.0, 10.0, 101), [1000.0]))

    normalized = analysis.robust_normalize(values)

    assert np.all((0.0 <= normalized) & (normalized <= 1.0))
    assert normalized[0] == 0.0
    assert normalized[-1] == 1.0
    assert np.array_equal(
        analysis.robust_normalize(np.ones(8)), np.zeros(8)
    )


def test_optical_flow_tracker_reports_translation_magnitude():
    image = make_feature_image()
    transform = np.float32([[1.0, 0.0, 4.0], [0.0, 1.0, 3.0]])
    shifted = cv2.warpAffine(image, transform, (image.shape[1], image.shape[0]))
    tracker = analysis.OpticalFlowTracker()

    _display, first_time, first_motion = tracker.process(1.0, image)
    display, sample_time, motion = tracker.process(1.1, shifted)

    assert first_time is None
    assert first_motion is None
    assert sample_time == pytest.approx(1.05)
    assert motion == pytest.approx(50.0, rel=0.15)
    assert display.shape == image.shape


def test_decode_compressed_image_round_trip():
    source = make_feature_image()
    message = CompressedImage()
    message.format = 'jpeg'
    message.data = cv2.imencode('.jpg', source)[1].tobytes()

    decoded = analysis.decode_compressed_image(message)

    assert decoded.shape == source.shape


def test_estimate_latency_uses_positive_for_image_lag():
    base_times = np.arange(0.0, 8.0, 0.02)
    motion = (
        np.exp(-((base_times - 1.5) / 0.18) ** 2)
        + 0.7 * np.exp(-((base_times - 4.0) / 0.3) ** 2)
        + 0.5 * np.exp(-((base_times - 6.4) / 0.12) ** 2)
    )

    estimate = analysis.estimate_latency(
        base_times + 0.1,
        motion,
        base_times,
        motion,
    )

    assert estimate is not None
    assert estimate.lag_sec == pytest.approx(0.1, abs=0.011)
    assert estimate.coefficient > 0.99


def test_validate_bag_accepts_expected_topics(tmp_path):
    bag_path = tmp_path / 'imu_camera_20260729_120000'
    write_test_bag(bag_path)

    assert analysis.validate_bag(bag_path) == 1


def test_validate_bag_rejects_missing_imu(tmp_path):
    bag_path = tmp_path / 'imu_camera_20260729_120001'
    write_test_bag(bag_path, include_imu=False)

    with pytest.raises(ValueError, match='Required topic is missing'):
        analysis.validate_bag(bag_path)
