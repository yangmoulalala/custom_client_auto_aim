"""Tests for packet-rate and cumulative-error counters."""

from rm_mqtt.statistics import PacketStatistics


def test_snapshot_resets_packets_but_keeps_cumulative_errors():
    statistics = PacketStatistics()
    statistics.record_rx_packet()
    statistics.record_rx_packet()
    statistics.record_rx_error()
    statistics.record_tx_packet()
    statistics.record_tx_error()

    first = statistics.take_snapshot()
    second = statistics.take_snapshot()

    assert first.rx_packets == 2
    assert first.rx_errors == 1
    assert first.tx_packets == 1
    assert first.tx_errors == 1
    assert second.rx_packets == 0
    assert second.rx_errors == 1
    assert second.tx_packets == 0
    assert second.tx_errors == 1
