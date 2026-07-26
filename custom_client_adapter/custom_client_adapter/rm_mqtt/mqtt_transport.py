"""Low-latency MQTT transport with automatic reconnection."""

from typing import Callable

import paho.mqtt.client as mqtt


MessageCallback = Callable[[bytes], None]


class MqttTransport:
    """Own the MQTT loop and expose receive and future transmit paths."""

    def __init__(
        self,
        host: str,
        port: int,
        client_id: str,
        keepalive_sec: int,
        reconnect_interval_sec: int,
        subscription_topic: str,
        message_callback: MessageCallback,
    ) -> None:
        self._host = host
        self._port = port
        self._keepalive_sec = keepalive_sec
        self._subscription_topic = subscription_topic
        self._message_callback = message_callback
        self._connected = False

        client_options = {
            'client_id': client_id,
            'clean_session': True,
            'protocol': mqtt.MQTTv311,
            'transport': 'tcp',
        }
        if hasattr(mqtt, 'CallbackAPIVersion'):
            self._client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                **client_options,
            )
        else:
            self._client = mqtt.Client(**client_options)

        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(
            min_delay=reconnect_interval_sec,
            max_delay=reconnect_interval_sec,
        )

    @property
    def connected(self) -> bool:
        """Return the connection state maintained by the MQTT loop thread."""
        return self._connected

    def start(self) -> None:
        """Start asynchronous connection and network processing."""
        self._client.connect_async(
            self._host,
            self._port,
            self._keepalive_sec,
        )
        self._client.loop_start()

    def stop(self) -> None:
        """Stop network processing without blocking packet callbacks indefinitely."""
        self._client.disconnect()
        self._client.loop_stop()
        self._connected = False

    def publish(
        self,
        topic: str,
        payload: bytes,
    ):
        """Publish one non-retained QoS 0 binary control message."""
        result = self._client.publish(topic, payload, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f'MQTT publish failed with code {result.rc}')
        return result

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code != 0:
            self._connected = False
            return

        result, _ = client.subscribe(
            self._subscription_topic,
            qos=0,
        )
        self._connected = result == mqtt.MQTT_ERR_SUCCESS

    def _on_disconnect(self, client, userdata, *args):
        self._connected = False

    def _on_message(self, client, userdata, message):
        if message.topic == self._subscription_topic:
            self._message_callback(bytes(message.payload))
