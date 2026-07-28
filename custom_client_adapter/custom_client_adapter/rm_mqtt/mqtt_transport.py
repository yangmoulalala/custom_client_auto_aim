"""Low-latency MQTT transport with automatic client ID discovery."""

from dataclasses import dataclass, field
import threading
import time
from typing import Callable

import paho.mqtt.client as mqtt


MessageCallback = Callable[[bytes], None]
CLIENT_IDS = tuple(str(value) for value in range(1, 7)) + tuple(
    str(value) for value in range(101, 107)
)


@dataclass
class _ConnectionAttempt:
    connack_received: threading.Event = field(default_factory=threading.Event)
    disconnected: threading.Event = field(default_factory=threading.Event)
    accepted: bool = False
    rejected: bool = False
    reason: str = ''


class MqttTransport:
    """Own the MQTT connection worker and expose direct receive/transmit paths."""

    def __init__(
        self,
        host: str,
        port: int,
        keepalive_sec: int,
        reconnect_interval_sec: float,
        tcp_connect_timeout_sec: float,
        connack_timeout_sec: float,
        subscription_topic: str,
        message_callback: MessageCallback,
    ) -> None:
        self._host = host
        self._port = port
        self._keepalive_sec = keepalive_sec
        self._reconnect_interval_sec = reconnect_interval_sec
        self._tcp_connect_timeout_sec = tcp_connect_timeout_sec
        self._connack_timeout_sec = connack_timeout_sec
        self._subscription_topic = subscription_topic
        self._message_callback = message_callback

        self._state_lock = threading.Lock()
        self._connected = False
        self._client_id = CLIENT_IDS[0]
        self._last_error = 'Connection not started'
        self._client = None
        self._stop_event = threading.Event()
        self._worker = None

    @property
    def connected(self) -> bool:
        """Return the connection state maintained by the MQTT threads."""
        with self._state_lock:
            return self._connected

    @property
    def client_id(self) -> str:
        """Return the client ID used by the current connection attempt."""
        with self._state_lock:
            return self._client_id

    @property
    def last_error(self) -> str:
        """Return the latest connection failure without emitting high-rate logs."""
        with self._state_lock:
            return self._last_error

    def start(self) -> None:
        """Start the connection worker without blocking the caller."""
        if self._worker is not None and self._worker.is_alive():
            return

        self._stop_event.clear()
        self._worker = threading.Thread(
            target=self._run,
            name='rm_mqtt_connection',
            daemon=True,
        )
        self._worker.start()

    def stop(self) -> None:
        """Stop connection processing and release the active Paho client."""
        self._stop_event.set()
        with self._state_lock:
            client = self._client
        if client is not None:
            client.disconnect()

        worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join()

        with self._state_lock:
            self._connected = False
            self._client = None
        self._worker = None

    def publish(
        self,
        topic: str,
        payload: bytes,
    ):
        """Publish one non-retained QoS 0 binary control message."""
        with self._state_lock:
            client = self._client if self._connected else None
        if client is None:
            raise RuntimeError('MQTT client is not connected')

        result = client.publish(topic, payload, qos=0, retain=False)
        if result.rc != mqtt.MQTT_ERR_SUCCESS:
            raise RuntimeError(f'MQTT publish failed with code {result.rc}')
        return result

    def _run(self) -> None:
        client_id_index = 0
        while not self._stop_event.is_set():
            client_id = CLIENT_IDS[client_id_index]
            try:
                rejected = self._connect_once(client_id)
            except Exception as exc:
                self._set_disconnected(f'Connection worker failed: {exc}')
                rejected = False

            if rejected:
                client_id_index = (client_id_index + 1) % len(CLIENT_IDS)

            if self._stop_event.wait(self._reconnect_interval_sec):
                break

    def _connect_once(self, client_id: str) -> bool:
        attempt = _ConnectionAttempt()
        client = self._create_client(client_id, attempt)
        network_loop_started = False

        with self._state_lock:
            self._client_id = client_id
            self._client = client
            self._connected = False
            self._last_error = 'Connecting'

        try:
            try:
                result = client.connect(
                    self._host,
                    self._port,
                    self._keepalive_sec,
                )
            except OSError as exc:
                self._set_disconnected(f'TCP connection failed: {exc}')
                return False
            if result != mqtt.MQTT_ERR_SUCCESS:
                self._set_disconnected(
                    f'TCP connection failed with code {result}'
                )
                return False

            deadline = time.monotonic() + self._connack_timeout_sec
            while not attempt.connack_received.is_set():
                if self._stop_event.is_set():
                    return False

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self._set_disconnected(
                        f'MQTT CONNACK timeout after '
                        f'{self._connack_timeout_sec:g}s'
                    )
                    return False

                result = client.loop(timeout=remaining)
                if attempt.connack_received.is_set():
                    break
                if result == mqtt.MQTT_ERR_CONN_REFUSED:
                    self._set_disconnected('MQTT connection refused')
                    return True
                if result == mqtt.MQTT_ERR_PROTOCOL:
                    self._set_disconnected('MQTT handshake rejected')
                    return True
                if result != mqtt.MQTT_ERR_SUCCESS:
                    self._set_disconnected(
                        f'MQTT handshake failed: {mqtt.error_string(result)}'
                    )
                    return False

            if attempt.rejected:
                self._set_disconnected(attempt.reason)
                return True
            if not attempt.accepted:
                self._set_disconnected(attempt.reason or 'MQTT subscription failed')
                return False

            loop_result = client.loop_start()
            if loop_result not in (None, mqtt.MQTT_ERR_SUCCESS):
                self._set_disconnected(
                    f'MQTT network loop failed with code {loop_result}'
                )
                return False
            network_loop_started = True

            while not self._stop_event.is_set():
                if attempt.disconnected.wait(0.1):
                    break
            return False
        finally:
            client.disconnect()
            if network_loop_started:
                client.loop_stop()
            with self._state_lock:
                if self._client is client:
                    self._connected = False
                    self._client = None

    def _create_client(
        self,
        client_id: str,
        attempt: _ConnectionAttempt,
    ):
        client_options = {
            'client_id': client_id,
            'clean_session': True,
            'protocol': mqtt.MQTTv311,
            'transport': 'tcp',
            'reconnect_on_failure': False,
        }
        if hasattr(mqtt, 'CallbackAPIVersion'):
            client = mqtt.Client(
                mqtt.CallbackAPIVersion.VERSION2,
                **client_options,
            )
        else:
            client = mqtt.Client(**client_options)

        # Paho 1.6.1 has no public TCP connection timeout setter.
        client._connect_timeout = self._tcp_connect_timeout_sec
        client.on_connect = (
            lambda mqtt_client, userdata, flags, reason_code, properties=None:
            self._on_connect(
                mqtt_client,
                attempt,
                reason_code,
            )
        )
        client.on_disconnect = (
            lambda mqtt_client, userdata, *args:
            self._on_disconnect(attempt, args)
        )
        client.on_message = self._on_message
        return client

    def _on_connect(
        self,
        client,
        attempt: _ConnectionAttempt,
        reason_code,
    ) -> None:
        reason_value = self._reason_code_value(reason_code)
        if reason_value != 0:
            attempt.rejected = True
            attempt.reason = self._connack_error(reason_code, reason_value)
            attempt.connack_received.set()
            return

        result, _ = client.subscribe(self._subscription_topic, qos=0)
        if result != mqtt.MQTT_ERR_SUCCESS:
            attempt.reason = f'MQTT subscribe failed with code {result}'
            attempt.connack_received.set()
            return

        attempt.accepted = True
        with self._state_lock:
            if self._client is client:
                self._connected = True
                self._last_error = ''
        attempt.connack_received.set()

    def _on_disconnect(
        self,
        attempt: _ConnectionAttempt,
        callback_args,
    ) -> None:
        if attempt.accepted and not self._stop_event.is_set():
            reason_code = callback_args[1] if len(callback_args) >= 2 else (
                callback_args[0] if callback_args else None
            )
            detail = f': {reason_code}' if reason_code not in (None, 0) else ''
            self._set_disconnected(f'MQTT connection lost{detail}')
        attempt.disconnected.set()

    def _on_message(self, client, userdata, message) -> None:
        if message.topic == self._subscription_topic:
            self._message_callback(bytes(message.payload))

    def _set_disconnected(self, error: str) -> None:
        with self._state_lock:
            self._connected = False
            self._last_error = error

    @staticmethod
    def _reason_code_value(reason_code) -> int:
        value = getattr(reason_code, 'value', reason_code)
        return int(value)

    @staticmethod
    def _connack_error(reason_code, reason_value: int) -> str:
        if 0 < reason_value < 6:
            return mqtt.connack_string(reason_value)
        return f'MQTT connection refused: {reason_code}'
