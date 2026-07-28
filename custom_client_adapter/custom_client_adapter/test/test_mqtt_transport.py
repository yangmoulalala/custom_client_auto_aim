"""Tests for MQTT client ID discovery, reconnection, and publication."""

from types import SimpleNamespace
import threading
import time

import pytest

from rm_mqtt import mqtt_transport


class FakeClient:
    """Paho client double with configurable connection outcomes."""

    def __init__(
        self,
        reason_code=0,
        connect_error=None,
        connect_result=None,
        loop_result=None,
        send_connack=True,
    ):
        self.reason_code = reason_code
        self.connect_error = connect_error
        self.connect_result = connect_result
        self.loop_result = loop_result
        self.send_connack = send_connack
        self.on_connect = None
        self.on_disconnect = None
        self.on_message = None
        self.connect_arguments = None
        self.loop_started = False
        self.loop_stopped = False
        self.disconnected = False
        self.subscription = None
        self.publication = None
        self.connack_sent = False
        self._connect_timeout = None

    def connect(self, host, port, keepalive):
        self.connect_arguments = (host, port, keepalive)
        if self.connect_error is not None:
            raise self.connect_error
        if self.connect_result is not None:
            return self.connect_result
        return mqtt_transport.mqtt.MQTT_ERR_SUCCESS

    def loop(self, timeout):
        if self.send_connack and not self.connack_sent:
            self.connack_sent = True
            self.on_connect(self, None, None, self.reason_code)
        if self.loop_result is not None:
            return self.loop_result
        if not self.send_connack:
            time.sleep(min(timeout, 0.002))
        return mqtt_transport.mqtt.MQTT_ERR_SUCCESS

    def loop_start(self):
        self.loop_started = True
        message = SimpleNamespace(
            topic='CustomByteBlock',
            payload=b'telemetry',
        )
        self.on_message(self, None, message)
        self.on_disconnect(
            self,
            None,
            mqtt_transport.mqtt.MQTT_ERR_CONN_LOST,
        )

    def loop_stop(self):
        self.loop_stopped = True

    def disconnect(self):
        self.disconnected = True

    def subscribe(self, topic, qos):
        self.subscription = (topic, qos)
        return mqtt_transport.mqtt.MQTT_ERR_SUCCESS, 1

    def publish(self, topic, payload, qos, retain):
        self.publication = (topic, payload, qos, retain)
        return SimpleNamespace(rc=mqtt_transport.mqtt.MQTT_ERR_SUCCESS)


def make_transport(**overrides):
    options = {
        'host': '192.168.12.1',
        'port': 3333,
        'keepalive_sec': 10,
        'reconnect_interval_sec': 0.1,
        'tcp_connect_timeout_sec': 1.0,
        'connack_timeout_sec': 1.0,
        'subscription_topic': 'CustomByteBlock',
        'message_callback': lambda payload: None,
    }
    options.update(overrides)
    return mqtt_transport.MqttTransport(**options)


def install_fake_client(monkeypatch, fake_client):
    created_with = []

    def factory(*args, **kwargs):
        created_with.append((args, kwargs))
        return fake_client

    monkeypatch.setattr(mqtt_transport.mqtt, 'Client', factory)
    return created_with


def test_rejections_cycle_through_all_client_ids():
    transport = make_transport(reconnect_interval_sec=0.0)
    attempted_ids = []

    def reject(client_id):
        attempted_ids.append(client_id)
        if len(attempted_ids) == len(mqtt_transport.CLIENT_IDS) + 1:
            transport._stop_event.set()
        return True

    transport._connect_once = reject
    transport._run()

    assert attempted_ids == list(mqtt_transport.CLIENT_IDS) + ['1']


def test_network_failures_retry_the_same_client_id():
    transport = make_transport(reconnect_interval_sec=0.0)
    attempted_ids = []

    def fail(client_id):
        attempted_ids.append(client_id)
        if len(attempted_ids) == 3:
            transport._stop_event.set()
        return False

    transport._connect_once = fail
    transport._run()

    assert attempted_ids == ['1', '1', '1']


@pytest.mark.parametrize('reason_code', [1, 2, 3, 4, 5, 128])
def test_any_nonzero_connack_rejects_the_client_id(monkeypatch, reason_code):
    fake_client = FakeClient(reason_code=reason_code)
    created_with = install_fake_client(monkeypatch, fake_client)
    transport = make_transport()

    assert transport._connect_once('3') is True
    assert transport.client_id == '3'
    assert not transport.connected
    assert fake_client.connect_arguments == ('192.168.12.1', 3333, 10)
    assert fake_client._connect_timeout == 1.0
    assert created_with[0][1]['client_id'] == '3'
    assert created_with[0][1]['reconnect_on_failure'] is False


def test_protocol_rejection_without_callback_advances_id(monkeypatch):
    fake_client = FakeClient(
        loop_result=mqtt_transport.mqtt.MQTT_ERR_PROTOCOL,
        send_connack=False,
    )
    install_fake_client(monkeypatch, fake_client)
    transport = make_transport()

    assert transport._connect_once('4') is True
    assert transport.last_error == 'MQTT handshake rejected'


def test_tcp_failure_keeps_id_and_uses_one_second_timeout(monkeypatch):
    fake_client = FakeClient(connect_error=TimeoutError('timed out'))
    install_fake_client(monkeypatch, fake_client)
    transport = make_transport()

    assert transport._connect_once('5') is False
    assert fake_client._connect_timeout == 1.0
    assert transport.client_id == '5'
    assert transport.last_error == 'TCP connection failed: timed out'


def test_tcp_client_error_keeps_id(monkeypatch):
    fake_client = FakeClient(
        connect_result=mqtt_transport.mqtt.MQTT_ERR_INVAL,
    )
    install_fake_client(monkeypatch, fake_client)
    transport = make_transport()

    assert transport._connect_once('5') is False
    assert transport.last_error == 'TCP connection failed with code 3'


def test_connack_timeout_keeps_id(monkeypatch):
    fake_client = FakeClient(send_connack=False)
    install_fake_client(monkeypatch, fake_client)
    transport = make_transport(connack_timeout_sec=0.005)

    assert transport._connect_once('6') is False
    assert transport.client_id == '6'
    assert transport.last_error == 'MQTT CONNACK timeout after 0.005s'


def test_success_subscribes_receives_and_retries_same_id_after_disconnect(
    monkeypatch,
):
    fake_client = FakeClient(reason_code=0)
    install_fake_client(monkeypatch, fake_client)
    received = []
    transport = make_transport(message_callback=received.append)

    assert transport._connect_once('101') is False
    assert fake_client.subscription == ('CustomByteBlock', 0)
    assert fake_client.loop_started
    assert fake_client.loop_stopped
    assert fake_client.disconnected
    assert received == [b'telemetry']


def test_publish_uses_qos_zero_without_retain():
    fake_client = FakeClient()
    transport = make_transport()
    with transport._state_lock:
        transport._client = fake_client
        transport._connected = True

    transport.publish('CustomControl', b'command')

    assert fake_client.publication == ('CustomControl', b'command', 0, False)


def test_stop_terminates_connection_worker(monkeypatch):
    connect_called = threading.Event()
    fake_client = FakeClient(connect_error=TimeoutError('timed out'))

    def factory(*args, **kwargs):
        connect_called.set()
        return fake_client

    monkeypatch.setattr(mqtt_transport.mqtt, 'Client', factory)
    transport = make_transport(reconnect_interval_sec=0.01)

    transport.start()
    assert connect_called.wait(1.0)
    transport.stop()

    assert transport._worker is None
    assert not transport.connected
    assert fake_client.disconnected
