"""Tests for the 0x0310 decoder and 0x0311 encoder."""

import json
import math
import struct

from rm_mqtt.custom_client_pb2 import CustomByteBlock
from rm_mqtt.custom_client_pb2 import CustomControl
from rm_mqtt.protocol import AIM_RX_CRC_OFFSET
from rm_mqtt.protocol import AIM_RX_SIZE
from rm_mqtt.protocol import AimControl
from rm_mqtt.protocol import calculate_crc16
from rm_mqtt.protocol import decode_aim_result
from rm_mqtt.protocol import decode_custom_byte_block
from rm_mqtt.protocol import encode_custom_control
from rm_mqtt.protocol import ProtocolError
import pytest


def make_data(
    enemy_color=1,
    quaternion=(1.0, 0.0, 0.0, 0.0),
) -> bytearray:
    """Build one valid 300-byte 0x0310 data block."""
    data = bytearray(300)
    data[0] = 0x53
    data[1] = 1
    data[2] = enemy_color
    struct.pack_into('<4f', data, 3, *quaternion)
    struct.pack_into('<H', data, 39, 1)
    struct.pack_into('<H', data, 41, calculate_crc16(data[:41]))
    return data


def serialize(data: bytes) -> bytes:
    """Wrap raw custom data in the official protobuf message."""
    return CustomByteBlock(data=bytes(data)).SerializeToString()


def refresh_crc(data: bytearray) -> None:
    """Update the inner checksum after changing a covered field."""
    struct.pack_into('<H', data, 41, calculate_crc16(data[:41]))


def test_crc16_known_vector():
    assert calculate_crc16(b'123456789') == 0x6F91


@pytest.mark.parametrize(
    ('enemy_color', 'expected'),
    [(0, False), (1, True)],
)
def test_decode_color_mapping(enemy_color, expected):
    telemetry = decode_custom_byte_block(serialize(make_data(enemy_color)))
    assert telemetry.self_is_red is expected


def test_decode_normalizes_and_reorders_quaternion():
    telemetry = decode_custom_byte_block(
        serialize(make_data(quaternion=(2.0, 2.0, 0.0, 0.0)))
    )
    expected = math.sqrt(0.5)
    assert telemetry.orientation_xyzw == pytest.approx(
        (expected, 0.0, 0.0, expected)
    )


@pytest.mark.parametrize('payload', [b'\x0a\xff', serialize(b'')])
def test_decode_rejects_invalid_protobuf_or_length(payload):
    with pytest.raises(ProtocolError):
        decode_custom_byte_block(payload)


def test_decode_rejects_invalid_head():
    data = make_data()
    data[0] = 0x54
    with pytest.raises(ProtocolError, match='head'):
        decode_custom_byte_block(serialize(data))


def test_decode_rejects_invalid_color():
    data = make_data()
    data[2] = 2
    with pytest.raises(ProtocolError, match='enem_color'):
        decode_custom_byte_block(serialize(data))


def test_decode_rejects_invalid_crc():
    data = make_data()
    data[10] ^= 0x01
    with pytest.raises(ProtocolError, match='CRC16'):
        decode_custom_byte_block(serialize(data))


def test_decode_rejects_nonzero_padding():
    data = make_data()
    data[43] = 1
    with pytest.raises(ProtocolError, match='padding'):
        decode_custom_byte_block(serialize(data))


@pytest.mark.parametrize(
    'quaternion',
    [
        (0.0, 0.0, 0.0, 0.0),
        (math.nan, 0.0, 0.0, 1.0),
    ],
)
def test_decode_rejects_invalid_quaternion(quaternion):
    data = make_data(quaternion=quaternion)
    refresh_crc(data)
    with pytest.raises(ProtocolError, match='quaternion'):
        decode_custom_byte_block(serialize(data))


def parse_control_payload(payload: bytes) -> bytes:
    """Unwrap one official CustomControl protobuf."""
    message = CustomControl()
    message.ParseFromString(payload)
    return message.data


def test_decode_aim_result_reads_required_fields():
    result = decode_aim_result(json.dumps({
        'control': True,
        'shoot': False,
        'yaw_rad': math.pi / 2.0,
        'pitch_rad': -math.pi / 4.0,
        'ignored': 'value',
    }))

    assert result == AimControl(
        control=True,
        shoot=False,
        yaw_rad=pytest.approx(math.pi / 2.0),
        pitch_rad=pytest.approx(-math.pi / 4.0),
    )


@pytest.mark.parametrize(
    'payload',
    [
        'not-json',
        '[]',
        '{"control": 1, "shoot": false, "yaw_rad": 0, "pitch_rad": 0}',
        '{"control": true, "shoot": false, "yaw_rad": "0", "pitch_rad": 0}',
        '{"control": true, "shoot": false, "yaw_rad": NaN, "pitch_rad": 0}',
    ],
)
def test_decode_aim_result_rejects_invalid_json_fields(payload):
    with pytest.raises(ProtocolError):
        decode_aim_result(payload)


@pytest.mark.parametrize(
    ('command', 'allow_fire', 'expected_mode', 'expected_yaw', 'expected_pitch'),
    [
        (AimControl(False, True, 1.0, 2.0), True, 0, 0.0, 0.0),
        (AimControl(True, False, math.pi / 2.0, -math.pi / 4.0),
         False, 1, math.pi / 2.0, -math.pi / 4.0),
        (AimControl(True, True, 0.1, -0.2), False, 1, 0.1, -0.2),
        (AimControl(True, True, 0.1, -0.2), True, 2, 0.1, -0.2),
    ],
)
def test_encode_custom_control_layout(
    command,
    allow_fire,
    expected_mode,
    expected_yaw,
    expected_pitch,
):
    data = parse_control_payload(encode_custom_control(command, allow_fire))

    assert len(data) == 30
    assert data[0] == 0x50
    assert data[1] == expected_mode
    yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc = struct.unpack_from(
        '<6f', data, 2
    )
    assert yaw == pytest.approx(expected_yaw)
    assert pitch == pytest.approx(expected_pitch)
    assert (yaw_vel, yaw_acc, pitch_vel, pitch_acc) == (0.0, 0.0, 0.0, 0.0)
    assert int.from_bytes(data[AIM_RX_CRC_OFFSET:AIM_RX_SIZE], 'little') == (
        calculate_crc16(data[:AIM_RX_CRC_OFFSET])
    )
    assert data[AIM_RX_SIZE:] == b'\x00\x00'


def test_encode_custom_control_rejects_unrepresentable_angle():
    command = AimControl(True, False, 1.0e308, 0.0)
    with pytest.raises(ProtocolError, match='float32'):
        encode_custom_control(command, False)
