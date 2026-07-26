"""Encode and decode RoboMaster custom client messages."""

from dataclasses import dataclass
import json
import math
import struct
from typing import Tuple

from rm_mqtt.custom_client_pb2 import CustomByteBlock
from rm_mqtt.custom_client_pb2 import CustomControl
from google.protobuf.message import DecodeError


CUSTOM_DATA_SIZE = 300
AIM_TX_SIZE = 43
AIM_TX_CRC_OFFSET = 41
AIM_TX_HEAD = 0x53
AIM_RX_SIZE = 28
AIM_RX_CRC_OFFSET = 26
AIM_RX_HEAD = 0x50
CONTROL_DATA_SIZE = 30


class ProtocolError(ValueError):
    """Raised when telemetry or control data is invalid."""


@dataclass(frozen=True)
class AimTelemetry:
    """The subset of Aim_Tx currently exposed through ROS."""

    orientation_xyzw: Tuple[float, float, float, float]
    self_is_red: bool


@dataclass(frozen=True)
class AimControl:
    """Control fields consumed from the upstream ROS result."""

    control: bool
    shoot: bool
    yaw_rad: float
    pitch_rad: float


def calculate_crc16(data: bytes) -> int:
    """Calculate the RoboMaster reflected CRC16 without a final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc


def decode_custom_byte_block(payload: bytes) -> AimTelemetry:
    """Validate a CustomByteBlock MQTT payload and decode its IMU fields."""
    message = CustomByteBlock()
    try:
        message.ParseFromString(payload)
    except DecodeError as exc:
        raise ProtocolError('invalid CustomByteBlock protobuf') from exc

    data = message.data
    if len(data) != CUSTOM_DATA_SIZE:
        raise ProtocolError(
            f'CustomByteBlock data must be {CUSTOM_DATA_SIZE} bytes'
        )
    if data[0] != AIM_TX_HEAD:
        raise ProtocolError('invalid Aim_Tx head')

    enemy_color = data[2]
    if enemy_color not in (0, 1):
        raise ProtocolError('enem_color must be 0 or 1')

    expected_crc = int.from_bytes(data[AIM_TX_CRC_OFFSET:AIM_TX_SIZE], 'little')
    actual_crc = calculate_crc16(data[:AIM_TX_CRC_OFFSET])
    if actual_crc != expected_crc:
        raise ProtocolError('invalid Aim_Tx CRC16')

    if any(data[AIM_TX_SIZE:]):
        raise ProtocolError('0x0310 padding must be zero')

    q_w, q_x, q_y, q_z = struct.unpack_from('<4f', data, 3)
    quaternion = (q_x, q_y, q_z, q_w)
    if not all(math.isfinite(value) for value in quaternion):
        raise ProtocolError('quaternion must be finite')

    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-6:
        raise ProtocolError('quaternion norm is too small')

    orientation = tuple(value / norm for value in quaternion)
    return AimTelemetry(
        orientation_xyzw=orientation,
        self_is_red=bool(enemy_color),
    )


def decode_aim_result(payload: str) -> AimControl:
    """Parse and validate one AUV Client JSON control result."""
    try:
        result = json.loads(payload)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ProtocolError('invalid control JSON') from exc

    if not isinstance(result, dict):
        raise ProtocolError('control JSON must be an object')

    control = result.get('control')
    shoot = result.get('shoot')
    if not isinstance(control, bool) or not isinstance(shoot, bool):
        raise ProtocolError('control and shoot must be boolean')

    yaw_rad = _finite_number(result.get('yaw_rad'), 'yaw_rad')
    pitch_rad = _finite_number(result.get('pitch_rad'), 'pitch_rad')
    return AimControl(
        control=control,
        shoot=shoot,
        yaw_rad=yaw_rad,
        pitch_rad=pitch_rad,
    )


def encode_custom_control(command: AimControl, allow_fire: bool) -> bytes:
    """Encode an Aim_Rx block and wrap it in the CustomControl protobuf."""
    mode = 0
    yaw_radians = 0.0
    pitch_radians = 0.0
    if command.control:
        mode = 2 if command.shoot and allow_fire else 1
        yaw_radians = command.yaw_rad
        pitch_radians = command.pitch_rad
        if not math.isfinite(yaw_radians) or not math.isfinite(pitch_radians):
            raise ProtocolError('control angle is outside float32 range')

    data = bytearray(CONTROL_DATA_SIZE)
    try:
        struct.pack_into(
            '<BB6f',
            data,
            0,
            AIM_RX_HEAD,
            mode,
            yaw_radians,
            0.0,
            0.0,
            pitch_radians,
            0.0,
            0.0,
        )
    except (OverflowError, struct.error) as exc:
        raise ProtocolError('control angle is outside float32 range') from exc

    checksum = calculate_crc16(data[:AIM_RX_CRC_OFFSET])
    struct.pack_into('<H', data, AIM_RX_CRC_OFFSET, checksum)
    return CustomControl(data=bytes(data)).SerializeToString()


def _finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f'{name} must be a number')
    number = float(value)
    if not math.isfinite(number):
        raise ProtocolError(f'{name} must be finite')
    return number
