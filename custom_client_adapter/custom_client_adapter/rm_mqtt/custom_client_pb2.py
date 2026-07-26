# Generated from custom_client.proto. Do not edit.
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder

_sym_db = _symbol_database.Default()

DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(
    b'\n\x13custom_client.proto\x12\x0drm_mqtt.proto"-\n\x0f'
    b'CustomByteBlock\x12\x11\n\x04data\x18\x01 \x01(\x0cH\x00\x88\x01\x01'
    b'B\x07\n\x05_data"+\n\rCustomControl\x12\x11\n\x04data\x18\x01 '
    b'\x01(\x0cH\x00\x88\x01\x01B\x07\n\x05_datab\x06proto3'
)

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(
    DESCRIPTOR, 'rm_mqtt.custom_client_pb2', globals()
)
