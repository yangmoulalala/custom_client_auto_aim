"""ROS 2 bridge for RoboMaster 0x0310 telemetry and 0x0311 control."""

import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rm_mqtt.mqtt_transport import MqttTransport
from rm_mqtt.protocol import decode_aim_result
from rm_mqtt.protocol import decode_custom_byte_block
from rm_mqtt.protocol import encode_custom_control
from rm_mqtt.statistics import PacketStatistics
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import String


class RmMqttNode(Node):
    """Bridge low-latency telemetry and control without application queues."""

    def __init__(self) -> None:
        super().__init__('rm_mqtt')

        host = self.declare_parameter('mqtt.host', '192.168.12.1').value
        port = self.declare_parameter('mqtt.port', 3333).value
        client_id = self.declare_parameter(
            'mqtt.client_id', Parameter.Type.STRING
        ).value
        if client_id is None:
            raise ValueError('Required ROS parameter mqtt.client_id was not provided')
        keepalive_sec = self.declare_parameter('mqtt.keepalive_sec', 10).value
        reconnect_interval_sec = self.declare_parameter(
            'mqtt.reconnect_interval_sec', 1
        ).value
        subscription_topic = self.declare_parameter(
            'mqtt.subscription_topic', 'CustomByteBlock'
        ).value
        self._control_mqtt_topic = self.declare_parameter(
            'mqtt.control_topic', 'CustomControl'
        ).value
        imu_topic = self.declare_parameter(
            'publisher.imu_topic', '/rm_mqtt/imu'
        ).value
        self_is_red_topic = self.declare_parameter(
            'publisher.self_is_red_topic', '/rm_mqtt/self_is_red'
        ).value
        self._frame_id = self.declare_parameter(
            'publisher.frame_id', 'imu_link'
        ).value
        qos_depth = self.declare_parameter('publisher.qos_depth', 1).value
        control_topic = self.declare_parameter(
            'subscriber.control_topic', '/auto_aim/result'
        ).value
        control_qos_depth = self.declare_parameter(
            'subscriber.qos_depth', 1
        ).value
        self._allow_fire = self.declare_parameter(
            'control.allow_fire', False
        ).value
        max_send_rate_hz = self.declare_parameter(
            'control.max_send_rate_hz', 75.0
        ).value
        self._timestamp_offset_sec = self.declare_parameter(
            'timestamp_offset_sec', 0.0
        ).value
        self._statistics_window_sec = self.declare_parameter(
            'statistics.window_sec', 1.0
        ).value

        self._validate_parameters(
            host=host,
            port=port,
            client_id=client_id,
            keepalive_sec=keepalive_sec,
            reconnect_interval_sec=reconnect_interval_sec,
            subscription_topic=subscription_topic,
            control_mqtt_topic=self._control_mqtt_topic,
            imu_topic=imu_topic,
            self_is_red_topic=self_is_red_topic,
            control_topic=control_topic,
            qos_depth=qos_depth,
            control_qos_depth=control_qos_depth,
            allow_fire=self._allow_fire,
            max_send_rate_hz=max_send_rate_hz,
        )

        publisher_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        subscriber_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=control_qos_depth,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._imu_publisher = self.create_publisher(
            Imu, imu_topic, publisher_qos
        )
        self._color_publisher = self.create_publisher(
            Bool, self_is_red_topic, publisher_qos
        )
        self._control_subscription = self.create_subscription(
            String,
            control_topic,
            self._handle_control_message,
            subscriber_qos,
        )

        self._statistics = PacketStatistics()
        self._minimum_control_period_sec = 1.0 / max_send_rate_hz
        self._last_control_publish_time = None
        self._last_statistics_time = time.monotonic()
        self._statistics_timer = self.create_timer(
            self._statistics_window_sec,
            self._log_statistics,
        )
        self._transport = MqttTransport(
            host=host,
            port=port,
            client_id=client_id,
            keepalive_sec=keepalive_sec,
            reconnect_interval_sec=reconnect_interval_sec,
            subscription_topic=subscription_topic,
            message_callback=self._handle_message,
        )
        self._transport.start()

    def destroy_node(self):
        """Stop MQTT callbacks before destroying ROS publishers."""
        if hasattr(self, '_transport'):
            self._transport.stop()
        return super().destroy_node()

    def _validate_parameters(self, **parameters) -> None:
        def require(condition: bool, message: str) -> None:
            if not condition:
                raise ValueError(message)

        require(bool(parameters['host']), 'mqtt.host must not be empty')
        require(1 <= parameters['port'] <= 65535, 'mqtt.port is out of range')
        require(
            str(parameters['client_id']).isdecimal()
            and int(parameters['client_id']) > 0,
            'ROS parameter mqtt.client_id must be a positive integer string',
        )
        require(
            parameters['keepalive_sec'] > 0,
            'mqtt.keepalive_sec must be positive',
        )
        require(
            parameters['reconnect_interval_sec'] > 0,
            'mqtt.reconnect_interval_sec must be positive',
        )
        require(
            bool(parameters['subscription_topic']),
            'mqtt.subscription_topic must not be empty',
        )
        require(
            bool(parameters['control_mqtt_topic']),
            'mqtt.control_topic must not be empty',
        )
        require(
            parameters['subscription_topic']
            != parameters['control_mqtt_topic'],
            'MQTT telemetry and control topics must be different',
        )
        require(bool(parameters['imu_topic']), 'publisher.imu_topic must not be empty')
        require(
            bool(parameters['self_is_red_topic']),
            'publisher.self_is_red_topic must not be empty',
        )
        require(
            parameters['imu_topic'] != parameters['self_is_red_topic'],
            'publisher topics must be different',
        )
        require(
            bool(parameters['control_topic']),
            'subscriber.control_topic must not be empty',
        )
        require(
            parameters['control_topic']
            not in (parameters['imu_topic'], parameters['self_is_red_topic']),
            'ROS telemetry and control topics must be different',
        )
        require(parameters['qos_depth'] > 0, 'publisher.qos_depth must be positive')
        require(
            parameters['control_qos_depth'] > 0,
            'subscriber.qos_depth must be positive',
        )
        require(
            isinstance(parameters['allow_fire'], bool),
            'control.allow_fire must be boolean',
        )
        require(
            math.isfinite(parameters['max_send_rate_hz'])
            and 0.0 < parameters['max_send_rate_hz'] <= 75.0,
            'control.max_send_rate_hz must be within (0, 75]',
        )
        require(
            math.isfinite(self._timestamp_offset_sec)
            and abs(self._timestamp_offset_sec) <= 86400.0,
            'timestamp_offset_sec must be finite and within one day',
        )
        require(
            math.isfinite(self._statistics_window_sec)
            and self._statistics_window_sec > 0.0,
            'statistics.window_sec must be positive and finite',
        )

    def _handle_message(self, payload: bytes) -> None:
        self._statistics.record_rx_packet()
        try:
            telemetry = decode_custom_byte_block(payload)

            imu_message = Imu()
            imu_message.header.stamp = (
                self.get_clock().now()
                + Duration(seconds=self._timestamp_offset_sec)
            ).to_msg()
            imu_message.header.frame_id = self._frame_id
            (
                imu_message.orientation.x,
                imu_message.orientation.y,
                imu_message.orientation.z,
                imu_message.orientation.w,
            ) = telemetry.orientation_xyzw
            imu_message.angular_velocity_covariance[0] = -1.0
            imu_message.linear_acceleration_covariance[0] = -1.0

            color_message = Bool()
            color_message.data = telemetry.self_is_red

            self._imu_publisher.publish(imu_message)
            self._color_publisher.publish(color_message)
        except Exception:
            self._statistics.record_rx_error()

    def _handle_control_message(self, message: String) -> None:
        if not self._transport.connected:
            return

        now = time.monotonic()
        if (
            self._last_control_publish_time is not None
            and now - self._last_control_publish_time
            < self._minimum_control_period_sec
        ):
            return

        try:
            command = decode_aim_result(message.data)
            payload = encode_custom_control(command, self._allow_fire)
            self._transport.publish(self._control_mqtt_topic, payload)
            self._last_control_publish_time = now
            self._statistics.record_tx_packet()
        except Exception:
            self._statistics.record_tx_error()

    def _log_statistics(self) -> None:
        now = time.monotonic()
        elapsed = max(now - self._last_statistics_time, 1.0e-6)
        self._last_statistics_time = now
        snapshot = self._statistics.take_snapshot()

        if not self._transport.connected:
            self.get_logger().warning('MQTT disconnected')
            return

        tx_rate = round(snapshot.tx_packets / elapsed)
        rx_rate = round(snapshot.rx_packets / elapsed)
        self.get_logger().info(
            f'rx={rx_rate}packets/s rx_error={snapshot.rx_errors} '
            f'tx={tx_rate}packets/s tx_error={snapshot.tx_errors}'
        )


def main(args=None) -> None:
    """Run the custom client MQTT node."""
    rclpy.init(args=args)
    node = None
    try:
        node = RmMqttNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
