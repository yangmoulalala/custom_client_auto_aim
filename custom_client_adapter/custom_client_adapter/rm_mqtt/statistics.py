"""Thread-safe bidirectional statistics shared by MQTT and ROS threads."""

from dataclasses import dataclass
import threading


@dataclass(frozen=True)
class StatisticsSnapshot:
    """One bidirectional rate window and cumulative error counts."""

    rx_packets: int
    rx_errors: int
    tx_packets: int
    tx_errors: int


class PacketStatistics:
    """Track per-window packet counts and cumulative errors by direction."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._window_rx_packets = 0
        self._total_rx_errors = 0
        self._window_tx_packets = 0
        self._total_tx_errors = 0

    def record_rx_packet(self) -> None:
        """Record one received CustomByteBlock message."""
        with self._lock:
            self._window_rx_packets += 1

    def record_rx_error(self) -> None:
        """Record one receive-side processing or ROS publication error."""
        with self._lock:
            self._total_rx_errors += 1

    def record_tx_packet(self) -> None:
        """Record one control packet accepted by the MQTT client."""
        with self._lock:
            self._window_tx_packets += 1

    def record_tx_error(self) -> None:
        """Record one control parsing, encoding, or MQTT publication error."""
        with self._lock:
            self._total_tx_errors += 1

    def take_snapshot(self) -> StatisticsSnapshot:
        """Return current values and begin a new packet-rate window."""
        with self._lock:
            snapshot = StatisticsSnapshot(
                rx_packets=self._window_rx_packets,
                rx_errors=self._total_rx_errors,
                tx_packets=self._window_tx_packets,
                tx_errors=self._total_tx_errors,
            )
            self._window_rx_packets = 0
            self._window_tx_packets = 0
        return snapshot
