"""Tests for MQTT connection, subscription, and control publication."""

from types import SimpleNamespace

from rm_mqtt import mqtt_transport


class FakeClient:
    """Minimal paho client double used by transport tests."""

    def __init__(self, *args, **kwargs):
        self.on_connect = None
        self.on_disconnect = None
        self.on_message = None
        self.reconnect_delays = None
        self.connect_arguments = None
        self.loop_started = False
        self.loop_stopped = False
        self.disconnected = False
        self.subscription = None
        self.publication = None

    def reconnect_delay_set(self, min_delay, max_delay):
        self.reconnect_delays = (min_delay, max_delay)

    def connect_async(self, host, port, keepalive):
        self.connect_arguments = (host, port, keepalive)

    def loop_start(self):
        self.loop_started = True

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


def test_transport_reconnects_subscribes_receives_and_publishes(monkeypatch):
    fake_client = FakeClient()
    monkeypatch.setattr(
        mqtt_transport.mqtt,
        'Client',
        lambda *args, **kwargs: fake_client,
    )
    received = []
    transport = mqtt_transport.MqttTransport(
        host='192.168.12.1',
        port=3333,
        client_id='3',
        keepalive_sec=10,
        reconnect_interval_sec=1,
        subscription_topic='CustomByteBlock',
        message_callback=received.append,
    )

    transport.start()
    assert fake_client.reconnect_delays == (1, 1)
    assert fake_client.connect_arguments == ('192.168.12.1', 3333, 10)
    assert fake_client.loop_started

    fake_client.on_connect(fake_client, None, None, 0)
    assert transport.connected
    assert fake_client.subscription == ('CustomByteBlock', 0)

    message = SimpleNamespace(topic='CustomByteBlock', payload=b'telemetry')
    fake_client.on_message(fake_client, None, message)
    assert received == [b'telemetry']

    transport.publish('CustomControl', b'command')
    assert fake_client.publication == ('CustomControl', b'command', 0, False)

    transport.stop()
    assert fake_client.disconnected
    assert fake_client.loop_stopped
    assert not transport.connected
