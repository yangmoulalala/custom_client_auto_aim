#!/usr/bin/python3 -s
"""Display low-latency camera optical flow and IMU motion curves."""

from collections import deque
import math
import signal
import site
import sys
import threading
import time

# ROS、OpenCV 和 Matplotlib 使用 Ubuntu 系统包，避免用户目录中的 NumPy
# 覆盖系统 NumPy 后产生二进制 ABI 冲突。该处理同时覆盖 python3 直接启动。
user_site_packages = site.getusersitepackages()
if isinstance(user_site_packages, str):
    user_site_packages = [user_site_packages]
user_site_packages = set(user_site_packages)
sys.path[:] = [
    path for path in sys.path if path not in user_site_packages
]

import cv2
import matplotlib
import numpy as np

matplotlib.use('Qt5Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT
from matplotlib.figure import Figure
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu


# ------------------------------ 可调常量 ------------------------------

IMAGE_TOPIC = '/rm_video/image_processed'
IMU_TOPIC = '/rm_mqtt/imu'
QOS_DEPTH = 1

HISTORY_SECONDS = 20.0
PLOT_REFRESH_HZ = 15.0
GUI_POLL_INTERVAL_MS = 2
CAMERA_WINDOW_WIDTH = 960
CAMERA_WINDOW_HEIGHT = 540

# 光流只在缩小后的灰度图上计算，最终速度会换算回原图像素每秒。
FLOW_PROCESSING_MAX_WIDTH = 640
FLOW_MAX_FEATURES = 250
FLOW_QUALITY_LEVEL = 0.01
FLOW_MIN_DISTANCE = 8.0
FLOW_BLOCK_SIZE = 7
FLOW_LK_WINDOW_SIZE = 21
FLOW_LK_MAX_LEVEL = 3
FLOW_LK_CRITERIA_COUNT = 20
FLOW_LK_CRITERIA_EPS = 0.01
FLOW_MIN_TRACKS = 12
FLOW_REDETECT_INTERVAL = 10
FLOW_MAX_TRACK_ERROR = 30.0
FLOW_MAX_DISPLAY_VECTORS = 80
FLOW_VECTOR_DISPLAY_GAIN = 2.0

MIN_SAMPLE_INTERVAL_SEC = 1.0e-5
CAMERA_MAX_INTERVAL_SEC = 0.5
IMU_MAX_INTERVAL_SEC = 0.25
WARNING_INTERVAL_SEC = 2.0

# ---------------------------------------------------------------------


class SignalBuffer:
    """Thread-safe, time-bounded storage for a pair of signals."""

    def __init__(self, history_seconds):
        self._history_seconds = history_seconds
        self._samples = deque()
        self._lock = threading.Lock()

    def append(self, sample_time, value_x, value_y):
        with self._lock:
            self._samples.append((sample_time, value_x, value_y))
            self._prune_locked(sample_time - self._history_seconds)

    def snapshot(self, now):
        with self._lock:
            self._prune_locked(now - self._history_seconds)
            if not self._samples:
                empty = np.empty(0, dtype=np.float64)
                return empty, empty.copy(), empty.copy()
            samples = np.asarray(self._samples, dtype=np.float64)
        return samples[:, 0], samples[:, 1], samples[:, 2]

    def _prune_locked(self, cutoff):
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()


def quaternion_multiply(left, right):
    """Multiply two xyzw quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )


def quaternion_angular_velocity(previous, current, delta_time):
    """Return body-frame angular velocity from two normalized quaternions."""
    if np.dot(previous, current) < 0.0:
        current = -current

    conjugate = np.array(
        [-previous[0], -previous[1], -previous[2], previous[3]],
        dtype=np.float64,
    )
    delta = quaternion_multiply(conjugate, current)
    delta_norm = np.linalg.norm(delta)
    if not math.isfinite(delta_norm) or delta_norm < 1.0e-12:
        return None
    delta /= delta_norm
    if delta[3] < 0.0:
        delta = -delta

    vector_norm = np.linalg.norm(delta[:3])
    if vector_norm < 1.0e-10:
        rotation_vector = 2.0 * delta[:3]
    else:
        angle = 2.0 * math.atan2(vector_norm, delta[3])
        rotation_vector = delta[:3] * (angle / vector_norm)
    return rotation_vector / delta_time


class LatencyViewerNode(Node):
    """Receive IMU messages and feed a latest-only image worker."""

    def __init__(self, start_time_ns, flow_worker, imu_buffer):
        super().__init__('imu_camera_latency_viewer')
        self._start_time_ns = start_time_ns
        self._flow_worker = flow_worker
        self._imu_buffer = imu_buffer
        self._image_sequence = 0
        self._previous_imu = None
        self._previous_imu_time_ns = None
        self._last_warning_time = {}

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=QOS_DEPTH,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        image_group = MutuallyExclusiveCallbackGroup()
        imu_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self._handle_image,
            qos,
            callback_group=image_group,
        )
        self.create_subscription(
            Imu,
            IMU_TOPIC,
            self._handle_imu,
            qos,
            callback_group=imu_group,
        )

    def _handle_image(self, message):
        receipt_time_ns = time.monotonic_ns()
        self._image_sequence += 1
        self._flow_worker.submit(
            self._image_sequence, receipt_time_ns, message
        )

    def _handle_imu(self, message):
        receipt_time_ns = time.monotonic_ns()
        quaternion = np.array(
            [
                message.orientation.x,
                message.orientation.y,
                message.orientation.z,
                message.orientation.w,
            ],
            dtype=np.float64,
        )
        norm = np.linalg.norm(quaternion)
        if not math.isfinite(norm) or norm < 1.0e-9:
            self.warn_throttled('imu_quaternion', 'Invalid IMU quaternion')
            self._previous_imu = None
            self._previous_imu_time_ns = None
            return
        quaternion /= norm

        if self._previous_imu is not None:
            delta_time = (
                receipt_time_ns - self._previous_imu_time_ns
            ) * 1.0e-9
            if MIN_SAMPLE_INTERVAL_SEC <= delta_time <= IMU_MAX_INTERVAL_SEC:
                angular_velocity = quaternion_angular_velocity(
                    self._previous_imu, quaternion, delta_time
                )
                if angular_velocity is not None and np.all(
                    np.isfinite(angular_velocity)
                ):
                    midpoint_ns = (
                        self._previous_imu_time_ns + receipt_time_ns
                    ) // 2
                    sample_time = (
                        midpoint_ns - self._start_time_ns
                    ) * 1.0e-9
                    self._imu_buffer.append(
                        sample_time,
                        float(angular_velocity[0]),
                        float(angular_velocity[1]),
                    )

        self._previous_imu = quaternion
        self._previous_imu_time_ns = receipt_time_ns

    def warn_throttled(self, key, message):
        now = time.monotonic()
        previous = self._last_warning_time.get(key, -math.inf)
        if now - previous >= WARNING_INTERVAL_SEC:
            self._last_warning_time[key] = now
            self.get_logger().warning(message)


class OpticalFlowWorker:
    """Compute optical flow on a latest-only image slot."""

    def __init__(self, start_time_ns, flow_buffer):
        self._start_time_ns = start_time_ns
        self._flow_buffer = flow_buffer
        self._condition = threading.Condition()
        self._pending = None
        self._stopping = False
        self._thread = threading.Thread(
            target=self._run, name='optical-flow', daemon=False
        )
        self._display_lock = threading.Lock()
        self._display_sequence = 0
        self._latest_display = None
        self._warning_callback = None
        self._previous_gray = None
        self._previous_points = None
        self._previous_time_ns = None
        self._tracking_frame_count = 0

    def set_warning_callback(self, callback):
        self._warning_callback = callback

    def start(self):
        self._thread.start()

    def stop(self):
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._thread.join()

    def submit(self, sequence, receipt_time_ns, message):
        with self._condition:
            self._pending = (sequence, receipt_time_ns, message)
            self._condition.notify()

    def latest_display(self, after_sequence):
        with self._display_lock:
            if self._display_sequence == after_sequence:
                return None
            return self._display_sequence, self._latest_display

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._stopping:
                    self._condition.wait()
                if self._stopping:
                    return
                sequence, receipt_time_ns, message = self._pending
                self._pending = None

            try:
                display = self._process_image(receipt_time_ns, message)
            except Exception as error:
                self._reset_tracking()
                self._warn('image_processing', f'Image processing failed: {error}')
                continue

            with self._display_lock:
                self._display_sequence = sequence
                self._latest_display = display

    def _process_image(self, receipt_time_ns, message):
        image = self._message_to_bgr(message)
        source_height, source_width = image.shape[:2]
        processing_scale = min(
            1.0, FLOW_PROCESSING_MAX_WIDTH / float(source_width)
        )
        processing_width = max(1, round(source_width * processing_scale))
        processing_height = max(1, round(source_height * processing_scale))
        if processing_scale < 1.0:
            processing_image = cv2.resize(
                image,
                (processing_width, processing_height),
                interpolation=cv2.INTER_AREA,
            )
        else:
            processing_image = image
        gray = cv2.cvtColor(processing_image, cv2.COLOR_BGR2GRAY)
        display = image.copy()

        velocity_x = None
        velocity_y = None
        valid_tracks = 0
        vectors = None

        same_shape = (
            self._previous_gray is not None
            and self._previous_gray.shape == gray.shape
        )
        if not same_shape:
            self._reset_tracking()

        if self._previous_gray is not None and self._previous_points is not None:
            delta_time = (
                receipt_time_ns - self._previous_time_ns
            ) * 1.0e-9
            if MIN_SAMPLE_INTERVAL_SEC <= delta_time <= CAMERA_MAX_INTERVAL_SEC:
                next_points, status, errors = cv2.calcOpticalFlowPyrLK(
                    self._previous_gray,
                    gray,
                    self._previous_points,
                    None,
                    winSize=(FLOW_LK_WINDOW_SIZE, FLOW_LK_WINDOW_SIZE),
                    maxLevel=FLOW_LK_MAX_LEVEL,
                    criteria=(
                        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                        FLOW_LK_CRITERIA_COUNT,
                        FLOW_LK_CRITERIA_EPS,
                    ),
                )
                if next_points is not None and status is not None:
                    valid = status.reshape(-1).astype(bool)
                    valid &= np.all(np.isfinite(next_points.reshape(-1, 2)), axis=1)
                    if errors is not None and FLOW_MAX_TRACK_ERROR > 0.0:
                        valid &= errors.reshape(-1) <= FLOW_MAX_TRACK_ERROR
                    old_points = self._previous_points.reshape(-1, 2)[valid]
                    new_points = next_points.reshape(-1, 2)[valid]
                    valid_tracks = len(new_points)
                    vectors = (old_points, new_points)
                    if valid_tracks >= FLOW_MIN_TRACKS:
                        displacement = np.median(
                            new_points - old_points, axis=0
                        )
                        scale_x = source_width / float(processing_width)
                        scale_y = source_height / float(processing_height)
                        velocity_x = float(displacement[0] * scale_x / delta_time)
                        velocity_y = float(displacement[1] * scale_y / delta_time)
                        midpoint_ns = (
                            self._previous_time_ns + receipt_time_ns
                        ) // 2
                        sample_time = (
                            midpoint_ns - self._start_time_ns
                        ) * 1.0e-9
                        self._flow_buffer.append(
                            sample_time, velocity_x, velocity_y
                        )
                    self._previous_points = new_points.reshape(-1, 1, 2)
            else:
                self._previous_points = None

        self._tracking_frame_count += 1
        needs_features = (
            self._previous_points is None
            or len(self._previous_points) < FLOW_MIN_TRACKS
            or self._tracking_frame_count >= FLOW_REDETECT_INTERVAL
        )
        if needs_features:
            self._previous_points = cv2.goodFeaturesToTrack(
                gray,
                maxCorners=FLOW_MAX_FEATURES,
                qualityLevel=FLOW_QUALITY_LEVEL,
                minDistance=FLOW_MIN_DISTANCE,
                blockSize=FLOW_BLOCK_SIZE,
            )
            self._tracking_frame_count = 0

        if vectors is not None:
            self._draw_vectors(
                display,
                vectors[0],
                vectors[1],
                source_width / float(processing_width),
                source_height / float(processing_height),
            )
        self._draw_status(display, velocity_x, velocity_y, valid_tracks)

        self._previous_gray = gray
        self._previous_time_ns = receipt_time_ns
        return display

    @staticmethod
    def _message_to_bgr(message):
        if message.encoding.lower() != 'bgr8':
            raise ValueError(f'expected bgr8, got {message.encoding!r}')
        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        if height <= 0 or width <= 0 or step < width * 3:
            raise ValueError('invalid image dimensions or step')
        required_size = (height - 1) * step + width * 3
        if len(message.data) < required_size:
            raise ValueError('image data is shorter than height and step require')
        return np.ndarray(
            shape=(height, width, 3),
            dtype=np.uint8,
            buffer=message.data,
            strides=(step, 3, 1),
        )

    @staticmethod
    def _draw_vectors(display, old_points, new_points, scale_x, scale_y):
        if len(new_points) == 0:
            return
        stride = max(1, math.ceil(len(new_points) / FLOW_MAX_DISPLAY_VECTORS))
        for old_point, new_point in zip(old_points[::stride], new_points[::stride]):
            start = (
                round(old_point[0] * scale_x),
                round(old_point[1] * scale_y),
            )
            end = (
                round(
                    (old_point[0] + (new_point[0] - old_point[0])
                     * FLOW_VECTOR_DISPLAY_GAIN)
                    * scale_x
                ),
                round(
                    (old_point[1] + (new_point[1] - old_point[1])
                     * FLOW_VECTOR_DISPLAY_GAIN)
                    * scale_y
                ),
            )
            cv2.arrowedLine(
                display, start, end, (50, 220, 50), 1, cv2.LINE_AA, tipLength=0.25
            )

    @staticmethod
    def _draw_status(display, velocity_x, velocity_y, valid_tracks):
        if velocity_x is None or velocity_y is None:
            text = f'flow unavailable  tracks={valid_tracks}'
        else:
            text = (
                f'vx={velocity_x:+.1f}px/s  vy={velocity_y:+.1f}px/s  '
                f'tracks={valid_tracks}'
            )
        cv2.putText(
            display,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            display,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def _reset_tracking(self):
        self._previous_gray = None
        self._previous_points = None
        self._previous_time_ns = None
        self._tracking_frame_count = 0

    def _warn(self, key, message):
        if self._warning_callback is not None:
            self._warning_callback(key, message)


class CameraWindow(QMainWindow):
    close_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Camera - optical flow')
        self.resize(CAMERA_WINDOW_WIDTH, CAMERA_WINDOW_HEIGHT)
        self._source_pixmap = None
        self._label = QLabel(f'Waiting for {IMAGE_TOPIC}')
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setStyleSheet('background: #111; color: #ddd;')
        self.setCentralWidget(self._label)

    def set_frame(self, frame):
        height, width = frame.shape[:2]
        image = QImage(
            frame.data,
            width,
            height,
            frame.strides[0],
            QImage.Format_BGR888,
        )
        self._source_pixmap = QPixmap.fromImage(image.copy())
        self._render_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_pixmap()

    def closeEvent(self, event):
        self.close_requested.emit()
        super().closeEvent(event)

    def _render_pixmap(self):
        if self._source_pixmap is None:
            return
        self._label.setPixmap(
            self._source_pixmap.scaled(
                self._label.size(),
                Qt.KeepAspectRatio,
                Qt.FastTransformation,
            )
        )


class LatencyToolbar(NavigationToolbar2QT):
    """Notify the plot when manual navigation changes axis limits."""

    def __init__(self, canvas, parent, manual_callback, home_callback):
        self._manual_callback = manual_callback
        self._home_callback = home_callback
        super().__init__(canvas, parent)

    def pan(self, *args):
        self._manual_callback()
        super().pan(*args)

    def zoom(self, *args):
        self._manual_callback()
        super().zoom(*args)

    def home(self, *args):
        super().home(*args)
        self._home_callback()


class PlotWindow(QMainWindow):
    close_requested = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle('IMU and camera latency curves')
        self.resize(1100, 760)
        self._auto_follow = True
        self._flow_snapshot = self._empty_snapshot()
        self._imu_snapshot = self._empty_snapshot()
        self._cursor_time = None
        self._marker_a = None
        self._marker_b = None
        self._marker_lines = []

        self._figure = Figure(figsize=(10, 7), tight_layout=True)
        self._canvas = FigureCanvasQTAgg(self._figure)
        self._flow_axis, self._imu_axis = self._figure.subplots(
            2, 1, sharex=True
        )
        self._flow_x_line, = self._flow_axis.plot(
            [], [], color='#d1495b', label='camera vx'
        )
        self._flow_y_line, = self._flow_axis.plot(
            [], [], color='#0077b6', label='camera vy'
        )
        self._imu_x_line, = self._imu_axis.plot(
            [], [], color='#e76f51', label='imu wx'
        )
        self._imu_y_line, = self._imu_axis.plot(
            [], [], color='#2a9d8f', label='imu wy'
        )
        self._flow_axis.set_ylabel('Optical flow (px/s)')
        self._imu_axis.set_ylabel('Angular velocity (rad/s)')
        self._imu_axis.set_xlabel('Local monotonic time (s)')
        for axis in (self._flow_axis, self._imu_axis):
            axis.grid(True, alpha=0.25)
            axis.legend(loc='upper left')

        self._cursor_lines = [
            axis.axvline(0.0, color='#555', linewidth=0.8, visible=False)
            for axis in (self._flow_axis, self._imu_axis)
        ]
        self._readout = QLabel('Waiting for camera and IMU samples')
        self._readout.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._readout.setStyleSheet('padding: 4px 8px; font-family: monospace;')
        toolbar = LatencyToolbar(
            self._canvas,
            self,
            self._disable_auto_follow,
            self._enable_auto_follow,
        )
        self._toolbar = toolbar

        container = QWidget()
        layout = QVBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(toolbar)
        layout.addWidget(self._canvas, 1)
        layout.addWidget(self._readout)
        self.setCentralWidget(container)

        self._canvas.mpl_connect('motion_notify_event', self._handle_motion)
        self._canvas.mpl_connect('button_press_event', self._handle_click)

    def refresh(self, flow_snapshot, imu_snapshot, now):
        self._flow_snapshot = flow_snapshot
        self._imu_snapshot = imu_snapshot
        flow_t, flow_x, flow_y = flow_snapshot
        imu_t, imu_x, imu_y = imu_snapshot
        self._flow_x_line.set_data(flow_t, flow_x)
        self._flow_y_line.set_data(flow_t, flow_y)
        self._imu_x_line.set_data(imu_t, imu_x)
        self._imu_y_line.set_data(imu_t, imu_y)

        if self._auto_follow:
            right = max(5.0, now + 0.25)
            left = max(0.0, right - HISTORY_SECONDS)
            self._flow_axis.set_xlim(left, right)
            self._set_auto_y(self._flow_axis, flow_t, flow_x, flow_y, left, right)
            self._set_auto_y(self._imu_axis, imu_t, imu_x, imu_y, left, right)

        if self._cursor_time is not None:
            self._update_readout(self._cursor_time)
        self._canvas.draw_idle()

    def closeEvent(self, event):
        self.close_requested.emit()
        super().closeEvent(event)

    def _handle_motion(self, event):
        if event.inaxes not in (self._flow_axis, self._imu_axis):
            return
        self._cursor_time = float(event.xdata)
        for line in self._cursor_lines:
            line.set_xdata([self._cursor_time, self._cursor_time])
            line.set_visible(True)
        self._update_readout(self._cursor_time)
        self._canvas.draw_idle()

    def _handle_click(self, event):
        if event.inaxes not in (self._flow_axis, self._imu_axis):
            return
        if event.button != 1 or self._toolbar.mode:
            return
        click_time = float(event.xdata)
        if self._marker_a is None or self._marker_b is not None:
            self._marker_a = click_time
            self._marker_b = None
        else:
            self._marker_b = click_time
        self._rebuild_marker_lines()
        self._update_readout(click_time)
        self._canvas.draw_idle()

    def _rebuild_marker_lines(self):
        for line in self._marker_lines:
            line.remove()
        self._marker_lines.clear()
        for marker, color in (
            (self._marker_a, '#6a4c93'),
            (self._marker_b, '#f4a261'),
        ):
            if marker is None:
                continue
            for axis in (self._flow_axis, self._imu_axis):
                self._marker_lines.append(
                    axis.axvline(marker, color=color, linestyle='--', linewidth=1.2)
                )

    def _update_readout(self, cursor_time):
        flow_x = self._nearest_value(self._flow_snapshot[0], self._flow_snapshot[1], cursor_time)
        flow_y = self._nearest_value(self._flow_snapshot[0], self._flow_snapshot[2], cursor_time)
        imu_x = self._nearest_value(self._imu_snapshot[0], self._imu_snapshot[1], cursor_time)
        imu_y = self._nearest_value(self._imu_snapshot[0], self._imu_snapshot[2], cursor_time)
        text = (
            f't={cursor_time:8.3f}s   camera vx={flow_x} px/s   '
            f'vy={flow_y} px/s   imu wx={imu_x} rad/s   wy={imu_y} rad/s'
        )
        if self._marker_a is not None:
            text += f'   A={self._marker_a:.3f}s'
        if self._marker_b is not None:
            delta_ms = (self._marker_b - self._marker_a) * 1000.0
            text += f'   B={self._marker_b:.3f}s   delta={delta_ms:+.1f}ms'
        self._readout.setText(text)

    @staticmethod
    def _nearest_value(times, values, target):
        if len(times) == 0:
            return '---'
        index = int(np.searchsorted(times, target))
        if index >= len(times):
            index = len(times) - 1
        elif index > 0 and target - times[index - 1] <= times[index] - target:
            index -= 1
        return f'{values[index]:+8.3f}'

    @staticmethod
    def _set_auto_y(axis, times, values_x, values_y, left, right):
        if len(times) == 0:
            axis.set_ylim(-1.0, 1.0)
            return
        visible = (times >= left) & (times <= right)
        values = np.concatenate((values_x[visible], values_y[visible]))
        values = values[np.isfinite(values)]
        if len(values) == 0:
            axis.set_ylim(-1.0, 1.0)
            return
        low = float(np.min(values))
        high = float(np.max(values))
        if math.isclose(low, high, rel_tol=1.0e-6, abs_tol=1.0e-9):
            padding = max(1.0, abs(low) * 0.1)
        else:
            padding = (high - low) * 0.1
        axis.set_ylim(low - padding, high + padding)

    def _disable_auto_follow(self):
        self._auto_follow = False

    def _enable_auto_follow(self):
        self._auto_follow = True

    @staticmethod
    def _empty_snapshot():
        empty = np.empty(0, dtype=np.float64)
        return empty, empty.copy(), empty.copy()


class ViewerController:
    """Move the latest processed frame and samples into the Qt widgets."""

    def __init__(
        self,
        app,
        start_time_ns,
        flow_worker,
        flow_buffer,
        imu_buffer,
        camera_window,
        plot_window,
    ):
        self._app = app
        self._start_time_ns = start_time_ns
        self._flow_worker = flow_worker
        self._flow_buffer = flow_buffer
        self._imu_buffer = imu_buffer
        self._camera_window = camera_window
        self._plot_window = plot_window
        self._last_display_sequence = 0
        self._stopping = False

        self._camera_timer = QTimer()
        self._camera_timer.setInterval(GUI_POLL_INTERVAL_MS)
        self._camera_timer.timeout.connect(self._update_camera)
        self._camera_timer.start()

        self._plot_timer = QTimer()
        self._plot_timer.setInterval(max(1, round(1000.0 / PLOT_REFRESH_HZ)))
        self._plot_timer.timeout.connect(self._update_plot)
        self._plot_timer.start()

    def request_shutdown(self):
        if self._stopping:
            return
        self._stopping = True
        self._camera_timer.stop()
        self._plot_timer.stop()
        self._app.quit()

    def _update_camera(self):
        result = self._flow_worker.latest_display(self._last_display_sequence)
        if result is None:
            return
        self._last_display_sequence, frame = result
        self._camera_window.set_frame(frame)

    def _update_plot(self):
        now = (time.monotonic_ns() - self._start_time_ns) * 1.0e-9
        self._plot_window.refresh(
            self._flow_buffer.snapshot(now),
            self._imu_buffer.snapshot(now),
            now,
        )


def main(args=None):
    rclpy.init(args=args)
    start_time_ns = time.monotonic_ns()
    flow_buffer = SignalBuffer(HISTORY_SECONDS)
    imu_buffer = SignalBuffer(HISTORY_SECONDS)
    flow_worker = OpticalFlowWorker(start_time_ns, flow_buffer)
    node = LatencyViewerNode(start_time_ns, flow_worker, imu_buffer)
    flow_worker.set_warning_callback(node.warn_throttled)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    flow_worker.start()
    executor_thread = threading.Thread(
        target=executor.spin, name='ros-executor', daemon=False
    )
    executor_thread.start()

    app = QApplication([sys.argv[0]])
    app.setQuitOnLastWindowClosed(False)
    camera_window = CameraWindow()
    plot_window = PlotWindow()
    controller = ViewerController(
        app,
        start_time_ns,
        flow_worker,
        flow_buffer,
        imu_buffer,
        camera_window,
        plot_window,
    )
    camera_window.close_requested.connect(controller.request_shutdown)
    plot_window.close_requested.connect(controller.request_shutdown)
    signal.signal(signal.SIGINT, lambda _signum, _frame: controller.request_shutdown())
    camera_window.show()
    plot_window.show()

    try:
        return app.exec_()
    finally:
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        flow_worker.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        camera_window.close()
        plot_window.close()


if __name__ == '__main__':
    sys.exit(main())
