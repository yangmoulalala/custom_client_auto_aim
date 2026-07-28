#!/usr/bin/python3 -s
"""Record camera and IMU messages for offline latency analysis."""

from datetime import datetime
import os
from pathlib import Path
import shutil
import signal
import site
import subprocess
import sys
import tempfile
import time


# ------------------------------ Configuration ------------------------------

IMAGE_TOPIC = '/rm_video/image_processed'
IMU_TOPIC = '/rm_mqtt/imu'
BAG_NAME_PREFIX = 'imu_camera_'
STATUS_INTERVAL_SEC = 1.0
MAX_CACHE_SIZE_BYTES = 32 * 1024 * 1024
RECORDER_SHUTDOWN_TIMEOUT_SEC = 15.0
QOS_DEPTH = 1

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
LOG_DIRECTORY = REPOSITORY_ROOT / 'log'

# ---------------------------------------------------------------------------


# ROS, OpenCV, and NumPy are installed as Ubuntu packages. Keep user-site
# packages from shadowing their ABI-compatible system versions.
user_site_packages = site.getusersitepackages()
if isinstance(user_site_packages, str):
    user_site_packages = [user_site_packages]
user_site_packages = set(user_site_packages)
sys.path[:] = [
    path for path in sys.path if path not in user_site_packages
]

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu


class TopicMonitor(Node):
    """Count incoming messages without decoding or retaining their payloads."""

    def __init__(self):
        super().__init__('imu_camera_bag_monitor')
        self.image_count = 0
        self.imu_count = 0
        self.last_image_time = None
        self.last_imu_time = None

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            CompressedImage, IMAGE_TOPIC, self._handle_image, qos
        )
        self.create_subscription(Imu, IMU_TOPIC, self._handle_imu, qos)

    def _handle_image(self, _message):
        self.image_count += 1
        self.last_image_time = time.monotonic()

    def _handle_imu(self, _message):
        self.imu_count += 1
        self.last_imu_time = time.monotonic()


def make_output_path():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    return LOG_DIRECTORY / f'{BAG_NAME_PREFIX}{timestamp}'


def make_qos_override_file():
    contents = ''.join(
        f'{topic}:\n'
        '  history: keep_last\n'
        f'  depth: {QOS_DEPTH}\n'
        '  reliability: best_effort\n'
        '  durability: volatile\n'
        for topic in (IMAGE_TOPIC, IMU_TOPIC)
    )
    handle = tempfile.NamedTemporaryFile(
        mode='w', prefix='imu_camera_qos_', suffix='.yaml', delete=False
    )
    try:
        handle.write(contents)
        return Path(handle.name)
    finally:
        handle.close()


def directory_size(path):
    if not path.exists():
        return 0
    return sum(
        entry.stat().st_size
        for entry in path.rglob('*')
        if entry.is_file()
    )


def format_age(last_time, now):
    if last_time is None:
        return '---'
    return f'{max(0.0, now - last_time) * 1000.0:.0f}ms'


def print_status(monitor, previous_counts, interval, output_path):
    now = time.monotonic()
    image_delta = monitor.image_count - previous_counts[0]
    imu_delta = monitor.imu_count - previous_counts[1]
    image_rate = image_delta / interval
    imu_rate = imu_delta / interval
    state = 'RECORDING' if image_delta > 0 and imu_delta > 0 else 'WAITING'
    size_mib = directory_size(output_path) / (1024.0 * 1024.0)
    print(
        f'[{state}] image={image_rate:.1f}msg/s total={monitor.image_count} '
        f'age={format_age(monitor.last_image_time, now)} | '
        f'imu={imu_rate:.1f}msg/s total={monitor.imu_count} '
        f'age={format_age(monitor.last_imu_time, now)} | '
        f'bag={size_mib:.1f}MiB',
        flush=True,
    )
    return monitor.image_count, monitor.imu_count


def stop_recorder(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=RECORDER_SHUTDOWN_TIMEOUT_SEC)
    except subprocess.TimeoutExpired:
        print('[WARN] Recorder did not stop in time; sending SIGTERM.', flush=True)
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5.0)


def recorded_topic_counts(output_path):
    metadata = rosbag2_py.Info().read_metadata(str(output_path), 'mcap')
    return {
        item.topic_metadata.name: int(item.message_count)
        for item in metadata.topics_with_message_count
    }


def run_recorder(output_path, qos_path):
    command = [
        'ros2',
        'bag',
        'record',
        '--output',
        str(output_path),
        '--storage',
        'mcap',
        '--topics',
        IMAGE_TOPIC,
        IMU_TOPIC,
        '--qos-profile-overrides-path',
        str(qos_path),
        '--max-cache-size',
        str(MAX_CACHE_SIZE_BYTES),
        '--disable-keyboard-controls',
        '--node-name',
        'imu_camera_bag_recorder',
    ]
    return subprocess.Popen(command, start_new_session=True)


def main():
    if len(sys.argv) != 1:
        print('Usage: record_imu_camera_bag.py', file=sys.stderr)
        return 2
    if shutil.which('ros2') is None:
        print(
            'ros2 was not found. Source the ROS 2 environment first.',
            file=sys.stderr,
        )
        return 2

    LOG_DIRECTORY.mkdir(parents=True, exist_ok=True)
    output_path = make_output_path()
    if output_path.exists():
        print(f'Output already exists: {output_path}', file=sys.stderr)
        return 2

    qos_path = make_qos_override_file()
    process = None
    monitor = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        monitor = TopicMonitor()
        process = run_recorder(output_path, qos_path)
        print(f'Recording camera and IMU to {output_path}', flush=True)
        print('Press Ctrl+C to stop and finalize the bag.', flush=True)

        previous_counts = (0, 0)
        previous_status_time = time.monotonic()
        while process.poll() is None:
            rclpy.spin_once(monitor, timeout_sec=0.1)
            now = time.monotonic()
            interval = now - previous_status_time
            if interval >= STATUS_INTERVAL_SEC:
                previous_counts = print_status(
                    monitor, previous_counts, interval, output_path
                )
                previous_status_time = now
    except KeyboardInterrupt:
        print('\nStopping recorder and finalizing MCAP...', flush=True)
    finally:
        if process is not None:
            stop_recorder(process)
        if monitor is not None:
            monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        qos_path.unlink(missing_ok=True)

    try:
        counts = recorded_topic_counts(output_path)
    except Exception as error:
        print(f'[INVALID] Could not read bag metadata: {error}', file=sys.stderr)
        return 1

    image_count = counts.get(IMAGE_TOPIC, 0)
    imu_count = counts.get(IMU_TOPIC, 0)
    if image_count <= 0 or imu_count <= 0:
        print(
            f'[INVALID] Recorded image={image_count}, imu={imu_count}. '
            'Both topics must contain messages.',
            file=sys.stderr,
        )
        return 1

    print(
        f'[VALID] Recorded image={image_count}, imu={imu_count}, '
        f'path={output_path}',
        flush=True,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
