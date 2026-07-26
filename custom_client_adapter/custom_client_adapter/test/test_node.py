"""ROS-level tests for direct low-latency telemetry and control."""

import json
import math
import struct

import pytest
import rclpy
from rclpy.time import Time
from rm_mqtt.custom_client_pb2 import CustomByteBlock
from rm_mqtt.custom_client_pb2 import CustomControl
from rm_mqtt.mqtt_transport import MqttTransport
from rm_mqtt.node import RmMqttNode
from rm_mqtt.protocol import calculate_crc16
from std_msgs.msg import String


class RecordingPublisher:
    """Collect messages published by one callback invocation."""

    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_payload() -> bytes:
    """Build one valid red-team IMU packet."""
    data = bytearray(300)
    data[0] = 0x53
    data[1] = 1
    data[2] = 1
    struct.pack_into('<4f', data, 3, 1.0, 0.0, 0.0, 0.0)
    struct.pack_into('<H', data, 41, calculate_crc16(data[:41]))
    return CustomByteBlock(data=bytes(data)).SerializeToString()


def test_missing_client_id_fails_immediately(monkeypatch):
    monkeypatch.setattr(MqttTransport, 'start', lambda self: None)
    rclpy.init()
    try:
        with pytest.raises(ValueError, match='Required ROS parameter mqtt.client_id'):
            RmMqttNode()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def test_callback_publishes_both_topics_with_timestamp_offset(monkeypatch):
    monkeypatch.setattr(MqttTransport, 'start', lambda self: None)
    monkeypatch.setattr(MqttTransport, 'stop', lambda self: None)
    rclpy.init(
        args=[
            '--ros-args',
            '-p',
            'mqtt.client_id:="3"',
            '-p',
            'timestamp_offset_sec:=0.25',
        ]
    )
    node = None
    try:
        node = RmMqttNode()
        imu_publisher = RecordingPublisher()
        color_publisher = RecordingPublisher()
        node._imu_publisher = imu_publisher
        node._color_publisher = color_publisher

        before = node.get_clock().now().nanoseconds
        node._handle_message(make_payload())
        after = node.get_clock().now().nanoseconds

        assert len(imu_publisher.messages) == 1
        assert len(color_publisher.messages) == 1
        stamp = Time.from_msg(
            imu_publisher.messages[0].header.stamp
        ).nanoseconds
        offset_ns = 250_000_000
        assert before + offset_ns <= stamp <= after + offset_ns
        assert imu_publisher.messages[0].header.frame_id == 'imu_link'
        assert color_publisher.messages[0].data is True

        statistics = node._statistics.take_snapshot()
        assert statistics.rx_packets == 1
        assert statistics.rx_errors == 0
        assert statistics.tx_packets == 0
        assert statistics.tx_errors == 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def control_message(
    control=True,
    shoot=False,
    yaw_rad=math.pi / 2.0,
    pitch_rad=-math.pi / 4.0,
):
    return String(data=json.dumps({
        'control': control,
        'shoot': shoot,
        'yaw_rad': yaw_rad,
        'pitch_rad': pitch_rad,
    }))


def test_control_callback_publishes_radians_and_drops_excess_rate(monkeypatch):
    publications = []
    monkeypatch.setattr(MqttTransport, 'start', lambda self: None)
    monkeypatch.setattr(MqttTransport, 'stop', lambda self: None)
    monkeypatch.setattr(
        MqttTransport,
        'publish',
        lambda self, topic, payload: publications.append((topic, payload)),
    )
    rclpy.init(args=['--ros-args', '-p', 'mqtt.client_id:="3"'])
    node = None
    try:
        node = RmMqttNode()
        node._transport._connected = True

        node._handle_control_message(control_message())
        node._handle_control_message(control_message(yaw_rad=0.1))

        assert len(publications) == 1
        topic, payload = publications[0]
        assert topic == 'CustomControl'
        protobuf = CustomControl()
        protobuf.ParseFromString(payload)
        assert len(protobuf.data) == 30
        assert protobuf.data[1] == 1
        yaw, _, _, pitch, _, _ = struct.unpack_from(
            '<6f', protobuf.data, 2
        )
        assert yaw == pytest.approx(math.pi / 2.0)
        assert pitch == pytest.approx(-math.pi / 4.0)

        statistics = node._statistics.take_snapshot()
        assert statistics.tx_packets == 1
        assert statistics.tx_errors == 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_callback_does_nothing_while_disconnected(monkeypatch):
    publications = []
    monkeypatch.setattr(MqttTransport, 'start', lambda self: None)
    monkeypatch.setattr(MqttTransport, 'stop', lambda self: None)
    monkeypatch.setattr(
        MqttTransport,
        'publish',
        lambda self, topic, payload: publications.append((topic, payload)),
    )
    rclpy.init(args=['--ros-args', '-p', 'mqtt.client_id:="3"'])
    node = None
    try:
        node = RmMqttNode()
        node._handle_control_message(control_message())

        assert publications == []
        statistics = node._statistics.take_snapshot()
        assert statistics.tx_packets == 0
        assert statistics.tx_errors == 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_control_callback_counts_invalid_json(monkeypatch):
    monkeypatch.setattr(MqttTransport, 'start', lambda self: None)
    monkeypatch.setattr(MqttTransport, 'stop', lambda self: None)
    rclpy.init(args=['--ros-args', '-p', 'mqtt.client_id:="3"'])
    node = None
    try:
        node = RmMqttNode()
        node._transport._connected = True
        node._handle_control_message(String(data='invalid'))

        statistics = node._statistics.take_snapshot()
        assert statistics.tx_packets == 0
        assert statistics.tx_errors == 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
