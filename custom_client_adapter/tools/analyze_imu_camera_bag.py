#!/usr/bin/python3 -s
"""Analyze camera optical flow and IMU motion from a recorded ROS bag."""

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import site
import sys
import time


# ------------------------------ Configuration ------------------------------

IMAGE_TOPIC = '/rm_video/image_processed'
IMU_TOPIC = '/rm_mqtt/imu'
IMAGE_TYPE = 'sensor_msgs/msg/CompressedImage'
IMU_TYPE = 'sensor_msgs/msg/Imu'
BAG_NAME_PATTERN = 'imu_camera_*'

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
PREVIEW_MAX_HZ = 30.0

NORMALIZATION_LOW_PERCENTILE = 5.0
NORMALIZATION_HIGH_PERCENTILE = 95.0
CORRELATION_MAX_LAG_SEC = 0.5
CORRELATION_STEP_SEC = 0.01
CORRELATION_MIN_OVERLAP_SEC = 1.0
CORRELATION_MIN_COEFFICIENT = 0.3

INITIAL_WINDOW_SEC = 5.0
INITIAL_MAJOR_TICK_SEC = 0.5
INITIAL_MINOR_TICK_SEC = 0.1
MINIMUM_X_WINDOW_SEC = 0.2
SCROLL_ZOOM_FACTOR = 0.8

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

import cv2
import matplotlib
import numpy as np

matplotlib.use('Qt5Agg')

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT
from matplotlib.figure import Figure
from matplotlib.ticker import MultipleLocator
from PyQt5.QtCore import Qt
from PyQt5.QtCore import QThread
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QImage
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QLabel
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtWidgets import QProgressBar
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5.QtWidgets import QWidget
from rclpy.serialization import deserialize_message
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu


@dataclass(frozen=True)
class LatencyEstimate:
    lag_sec: float
    coefficient: float


@dataclass(frozen=True)
class AnalysisData:
    flow_times: np.ndarray
    flow_motion: np.ndarray
    flow_normalized: np.ndarray
    imu_times: np.ndarray
    imu_motion: np.ndarray
    imu_normalized: np.ndarray
    estimate: object
    initial_limits: tuple


def stamp_to_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def quaternion_multiply(left, right):
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
    """Return body-frame angular velocity from two xyzw quaternions."""
    if np.dot(previous, current) < 0.0:
        current = -current

    conjugate = np.array(
        [-previous[0], -previous[1], -previous[2], previous[3]],
        dtype=np.float64,
    )
    delta = quaternion_multiply(conjugate, current)
    norm = np.linalg.norm(delta)
    if not math.isfinite(norm) or norm < 1.0e-12:
        return None
    delta /= norm
    if delta[3] < 0.0:
        delta = -delta

    vector_norm = np.linalg.norm(delta[:3])
    if vector_norm < 1.0e-10:
        rotation_vector = 2.0 * delta[:3]
    else:
        angle = 2.0 * math.atan2(vector_norm, delta[3])
        rotation_vector = delta[:3] * (angle / vector_norm)
    return rotation_vector / delta_time


def robust_normalize(values):
    values = np.asarray(values, dtype=np.float64)
    if len(values) == 0:
        return values.copy()
    low, high = np.percentile(
        values,
        [NORMALIZATION_LOW_PERCENTILE, NORMALIZATION_HIGH_PERCENTILE],
    )
    scale = float(high - low)
    if not math.isfinite(scale) or scale <= 1.0e-12:
        return np.zeros_like(values)
    return np.clip((values - low) / scale, 0.0, 1.0)


def estimate_latency(flow_times, flow_values, imu_times, imu_values):
    """Estimate image lag; positive means flow occurs after IMU motion."""
    if len(flow_times) < 2 or len(imu_times) < 2:
        return None
    if np.std(flow_values) <= 1.0e-9 or np.std(imu_values) <= 1.0e-9:
        return None

    lags = np.arange(
        -CORRELATION_MAX_LAG_SEC,
        CORRELATION_MAX_LAG_SEC + CORRELATION_STEP_SEC * 0.5,
        CORRELATION_STEP_SEC,
    )
    minimum_samples = max(
        3,
        round(CORRELATION_MIN_OVERLAP_SEC / CORRELATION_STEP_SEC),
    )
    best = None
    for lag in lags:
        left = max(float(imu_times[0]), float(flow_times[0] - lag))
        right = min(float(imu_times[-1]), float(flow_times[-1] - lag))
        if right - left < CORRELATION_MIN_OVERLAP_SEC:
            continue
        grid = np.arange(left, right, CORRELATION_STEP_SEC)
        if len(grid) < minimum_samples:
            continue
        imu_samples = np.interp(grid, imu_times, imu_values)
        flow_samples = np.interp(grid + lag, flow_times, flow_values)
        if np.std(imu_samples) <= 1.0e-9 or np.std(flow_samples) <= 1.0e-9:
            continue
        coefficient = float(np.corrcoef(imu_samples, flow_samples)[0, 1])
        if not math.isfinite(coefficient):
            continue
        if best is None or coefficient > best.coefficient:
            best = LatencyEstimate(float(lag), coefficient)

    if best is None or best.coefficient < CORRELATION_MIN_COEFFICIENT:
        return None
    return best


def decode_compressed_image(message):
    encoded = np.frombuffer(message.data, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None or image.size == 0:
        raise ValueError('JPEG decode failed')
    return image


class ImuMotionTracker:
    def __init__(self):
        self._previous_quaternion = None
        self._previous_time = None

    def process(self, timestamp, message):
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
            self.reset()
            return None
        quaternion /= norm

        sample = None
        if self._previous_quaternion is not None:
            delta_time = timestamp - self._previous_time
            if MIN_SAMPLE_INTERVAL_SEC <= delta_time <= IMU_MAX_INTERVAL_SEC:
                velocity = quaternion_angular_velocity(
                    self._previous_quaternion, quaternion, delta_time
                )
                if velocity is not None and np.all(np.isfinite(velocity)):
                    sample = (
                        (self._previous_time + timestamp) * 0.5,
                        float(math.hypot(velocity[0], velocity[1])),
                    )

        self._previous_quaternion = quaternion
        self._previous_time = timestamp
        return sample

    def reset(self):
        self._previous_quaternion = None
        self._previous_time = None


class OpticalFlowTracker:
    def __init__(self):
        self._previous_gray = None
        self._previous_points = None
        self._previous_time = None
        self._tracking_frame_count = 0

    def process(self, timestamp, image):
        source_height, source_width = image.shape[:2]
        scale = min(1.0, FLOW_PROCESSING_MAX_WIDTH / float(source_width))
        processing_width = max(1, round(source_width * scale))
        processing_height = max(1, round(source_height * scale))
        if scale < 1.0:
            processing_image = cv2.resize(
                image,
                (processing_width, processing_height),
                interpolation=cv2.INTER_AREA,
            )
        else:
            processing_image = image
        gray = cv2.cvtColor(processing_image, cv2.COLOR_BGR2GRAY)
        display = image.copy()

        if self._previous_gray is not None and self._previous_gray.shape != gray.shape:
            self.reset()

        motion = None
        sample_time = None
        vectors = None
        valid_tracks = 0
        if self._previous_gray is not None and self._previous_points is not None:
            delta_time = timestamp - self._previous_time
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
                    valid &= np.all(
                        np.isfinite(next_points.reshape(-1, 2)), axis=1
                    )
                    if errors is not None:
                        valid &= errors.reshape(-1) <= FLOW_MAX_TRACK_ERROR
                    old_points = self._previous_points.reshape(-1, 2)[valid]
                    new_points = next_points.reshape(-1, 2)[valid]
                    valid_tracks = len(new_points)
                    vectors = (old_points, new_points)
                    if valid_tracks >= FLOW_MIN_TRACKS:
                        displacement = np.median(
                            new_points - old_points, axis=0
                        )
                        velocity_x = (
                            displacement[0]
                            * source_width
                            / float(processing_width)
                            / delta_time
                        )
                        velocity_y = (
                            displacement[1]
                            * source_height
                            / float(processing_height)
                            / delta_time
                        )
                        motion = float(math.hypot(velocity_x, velocity_y))
                        sample_time = (self._previous_time + timestamp) * 0.5
                    self._previous_points = new_points.reshape(-1, 1, 2)
            else:
                self._previous_points = None

        self._tracking_frame_count += 1
        if (
            self._previous_points is None
            or len(self._previous_points) < FLOW_MIN_TRACKS
            or self._tracking_frame_count >= FLOW_REDETECT_INTERVAL
        ):
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
        self._draw_status(display, motion, valid_tracks)
        self._previous_gray = gray
        self._previous_time = timestamp
        return display, sample_time, motion

    def reset(self):
        self._previous_gray = None
        self._previous_points = None
        self._previous_time = None
        self._tracking_frame_count = 0

    @staticmethod
    def _draw_vectors(display, old_points, new_points, scale_x, scale_y):
        if len(new_points) == 0:
            return
        stride = max(1, math.ceil(len(new_points) / FLOW_MAX_DISPLAY_VECTORS))
        for old_point, new_point in zip(
            old_points[::stride], new_points[::stride]
        ):
            start = (
                round(old_point[0] * scale_x),
                round(old_point[1] * scale_y),
            )
            delta = (new_point - old_point) * FLOW_VECTOR_DISPLAY_GAIN
            end = (
                round((old_point[0] + delta[0]) * scale_x),
                round((old_point[1] + delta[1]) * scale_y),
            )
            cv2.arrowedLine(
                display,
                start,
                end,
                (50, 220, 50),
                1,
                cv2.LINE_AA,
                tipLength=0.25,
            )

    @staticmethod
    def _draw_status(display, motion, valid_tracks):
        if motion is None:
            text = f'motion unavailable  tracks={valid_tracks}'
        else:
            text = f'motion={motion:.1f}px/s  tracks={valid_tracks}'
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


def sort_samples(times, values):
    times = np.asarray(times, dtype=np.float64)
    values = np.asarray(values, dtype=np.float64)
    order = np.argsort(times)
    times = times[order]
    values = values[order]
    unique_times, unique_indices = np.unique(times, return_index=True)
    return unique_times, values[unique_indices]


def choose_initial_limits(flow_times, flow_values, imu_times, imu_values):
    full_left = min(float(flow_times[0]), float(imu_times[0]))
    full_right = max(float(flow_times[-1]), float(imu_times[-1]))
    duration = full_right - full_left
    if duration <= INITIAL_WINDOW_SEC:
        return full_left, max(full_right, full_left + MINIMUM_X_WINDOW_SEC)

    step = 0.05
    grid = np.arange(full_left, full_right, step)
    flow_activity = np.interp(grid, flow_times, flow_values, left=0.0, right=0.0)
    imu_activity = np.interp(grid, imu_times, imu_values, left=0.0, right=0.0)
    window_samples = max(1, round(INITIAL_WINDOW_SEC / step))
    scores = np.convolve(
        flow_activity + imu_activity,
        np.ones(window_samples, dtype=np.float64),
        mode='same',
    )
    center = float(grid[int(np.argmax(scores))])
    left = center - INITIAL_WINDOW_SEC * 0.5
    left = min(max(left, full_left), full_right - INITIAL_WINDOW_SEC)
    return left, left + INITIAL_WINDOW_SEC


def build_analysis_data(flow_times, flow_motion, imu_times, imu_motion):
    flow_times, flow_motion = sort_samples(flow_times, flow_motion)
    imu_times, imu_motion = sort_samples(imu_times, imu_motion)
    if len(flow_times) < 2:
        raise ValueError('Not enough valid optical-flow samples')
    if len(imu_times) < 2:
        raise ValueError('Not enough valid IMU samples')

    origin = min(float(flow_times[0]), float(imu_times[0]))
    flow_times = flow_times - origin
    imu_times = imu_times - origin
    flow_normalized = robust_normalize(flow_motion)
    imu_normalized = robust_normalize(imu_motion)
    estimate = estimate_latency(
        flow_times,
        flow_normalized,
        imu_times,
        imu_normalized,
    )
    initial_limits = choose_initial_limits(
        flow_times,
        flow_normalized,
        imu_times,
        imu_normalized,
    )
    return AnalysisData(
        flow_times,
        flow_motion,
        flow_normalized,
        imu_times,
        imu_motion,
        imu_normalized,
        estimate,
        initial_limits,
    )


def resolve_bag_path(argument):
    if argument is not None:
        path = Path(argument).expanduser().resolve()
    else:
        candidates = [
            path
            for path in LOG_DIRECTORY.glob(BAG_NAME_PATTERN)
            if path.is_dir() and (path / 'metadata.yaml').is_file()
        ]
        if not candidates:
            raise ValueError(f'No {BAG_NAME_PATTERN} bag found in {LOG_DIRECTORY}')
        path = max(candidates, key=lambda candidate: candidate.name)

    if not path.is_dir() or not (path / 'metadata.yaml').is_file():
        raise ValueError(f'Not a ROS bag directory: {path}')
    return path


def validate_bag(path):
    metadata = rosbag2_py.Info().read_metadata(str(path), 'mcap')
    if metadata.storage_identifier != 'mcap':
        raise ValueError('Only MCAP bags are supported')
    topics = {
        item.topic_metadata.name: item.topic_metadata.type
        for item in metadata.topics_with_message_count
    }
    expected = {IMAGE_TOPIC: IMAGE_TYPE, IMU_TOPIC: IMU_TYPE}
    for topic, expected_type in expected.items():
        actual_type = topics.get(topic)
        if actual_type is None:
            raise ValueError(f'Required topic is missing: {topic}')
        if actual_type != expected_type:
            raise ValueError(
                f'{topic} has type {actual_type!r}, expected {expected_type!r}'
            )
    image_count = next(
        int(item.message_count)
        for item in metadata.topics_with_message_count
        if item.topic_metadata.name == IMAGE_TOPIC
    )
    return image_count


class AnalysisWorker(QThread):
    frame_ready = pyqtSignal(QImage)
    progress_changed = pyqtSignal(int, int, str)
    analysis_ready = pyqtSignal(object)
    analysis_failed = pyqtSignal(str)

    def __init__(self, bag_path, total_images):
        super().__init__()
        self._bag_path = bag_path
        self._total_images = total_images

    def run(self):
        try:
            data = self._analyze()
        except Exception as error:
            self.analysis_failed.emit(str(error))
            return
        if not self.isInterruptionRequested():
            self.analysis_ready.emit(data)

    def _analyze(self):
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(
                uri=str(self._bag_path), storage_id='mcap'
            ),
            rosbag2_py.ConverterOptions('', ''),
        )
        reader.set_filter(
            rosbag2_py.StorageFilter(topics=[IMAGE_TOPIC, IMU_TOPIC])
        )

        flow_tracker = OpticalFlowTracker()
        imu_tracker = ImuMotionTracker()
        flow_times = []
        flow_motion = []
        imu_times = []
        imu_motion = []
        image_index = 0
        skipped_images = 0
        last_preview_time = -math.inf

        while reader.has_next():
            if self.isInterruptionRequested():
                raise RuntimeError('Analysis cancelled')
            topic, serialized, _record_time = reader.read_next()
            if topic == IMAGE_TOPIC:
                message = deserialize_message(serialized, CompressedImage)
                image_index += 1
                try:
                    timestamp = stamp_to_seconds(message.header.stamp)
                    image = decode_compressed_image(message)
                    display, sample_time, motion = flow_tracker.process(
                        timestamp, image
                    )
                except Exception:
                    skipped_images += 1
                    flow_tracker.reset()
                    continue
                if sample_time is not None and motion is not None:
                    flow_times.append(sample_time)
                    flow_motion.append(motion)

                now = time.monotonic()
                if now - last_preview_time >= 1.0 / PREVIEW_MAX_HZ:
                    rgb = cv2.cvtColor(display, cv2.COLOR_BGR2RGB)
                    height, width = rgb.shape[:2]
                    preview = QImage(
                        rgb.data,
                        width,
                        height,
                        rgb.strides[0],
                        QImage.Format_RGB888,
                    ).copy()
                    self.frame_ready.emit(preview)
                    self.progress_changed.emit(
                        image_index,
                        self._total_images,
                        f'Analyzing frame {image_index}/{self._total_images}  '
                        f'flow samples={len(flow_times)}  skipped={skipped_images}',
                    )
                    last_preview_time = now
            elif topic == IMU_TOPIC:
                message = deserialize_message(serialized, Imu)
                timestamp = stamp_to_seconds(message.header.stamp)
                sample = imu_tracker.process(timestamp, message)
                if sample is not None:
                    imu_times.append(sample[0])
                    imu_motion.append(sample[1])

        self.progress_changed.emit(
            self._total_images,
            self._total_images,
            f'Building curves from {len(flow_times)} optical-flow and '
            f'{len(imu_times)} IMU samples',
        )
        return build_analysis_data(
            flow_times, flow_motion, imu_times, imu_motion
        )


class PlotWidget(QWidget):
    def __init__(self, data):
        super().__init__()
        self._data = data
        self._cursor_time = None
        self._marker_a = None
        self._marker_b = None
        self._marker_lines = []
        self._full_limits = (
            min(float(data.flow_times[0]), float(data.imu_times[0])),
            max(float(data.flow_times[-1]), float(data.imu_times[-1])),
        )

        figure = Figure(figsize=(10, 7), tight_layout=True)
        self._axis = figure.subplots(1, 1)
        self._canvas = FigureCanvasQTAgg(figure)
        toolbar = NavigationToolbar2QT(self._canvas, self)
        self._axis.plot(
            data.flow_times,
            data.flow_normalized,
            color='#d1495b',
            linewidth=1.4,
            label='Optical-flow XY magnitude',
        )
        self._axis.plot(
            data.imu_times,
            data.imu_normalized,
            color='#0077b6',
            linewidth=1.2,
            label='IMU XY angular-speed magnitude',
        )
        self._axis.set_xlabel('Time from first sample (s)')
        self._axis.set_ylabel('Normalized motion')
        self._axis.set_ylim(-0.05, 1.05)
        self._axis.set_xlim(*data.initial_limits)
        self._axis.grid(True, which='major', alpha=0.3)
        self._axis.grid(True, which='minor', alpha=0.12)
        self._axis.legend(loc='upper right')
        self._configure_ticks(data.initial_limits[1] - data.initial_limits[0])

        self._cursor_line = self._axis.axvline(
            0.0, color='#555555', linewidth=0.9, visible=False
        )
        self._estimate_label = QLabel(self._format_estimate(data.estimate))
        self._estimate_label.setStyleSheet('padding: 6px 8px; font-weight: 600;')
        self._readout = QLabel('Move the cursor over the plot to inspect values.')
        self._readout.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self._readout.setStyleSheet('padding: 5px 8px; font-family: monospace;')

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(toolbar)
        layout.addWidget(self._estimate_label)
        layout.addWidget(self._canvas, 1)
        layout.addWidget(self._readout)

        self._canvas.mpl_connect('motion_notify_event', self._handle_motion)
        self._canvas.mpl_connect('button_press_event', self._handle_click)
        self._canvas.mpl_connect('scroll_event', self._handle_scroll)
        self._axis.callbacks.connect('xlim_changed', self._handle_limits_changed)
        self._canvas.draw_idle()

    @staticmethod
    def _format_estimate(estimate):
        if estimate is None:
            return 'Estimated image lag: unavailable (insufficient correlation)'
        return (
            f'Estimated image lag: {estimate.lag_sec * 1000.0:+.0f} ms  '
            f'correlation={estimate.coefficient:.3f}'
        )

    def _handle_motion(self, event):
        if event.inaxes is not self._axis or event.xdata is None:
            return
        self._cursor_time = float(event.xdata)
        self._cursor_line.set_xdata([self._cursor_time, self._cursor_time])
        self._cursor_line.set_visible(True)
        self._update_readout(self._cursor_time)
        self._canvas.draw_idle()

    def _handle_click(self, event):
        if (
            event.inaxes is not self._axis
            or event.xdata is None
            or event.button != 1
            or self._canvas.toolbar.mode
        ):
            return
        click_time = float(event.xdata)
        if self._marker_a is None or self._marker_b is not None:
            self._marker_a = click_time
            self._marker_b = None
        else:
            self._marker_b = click_time
        self._rebuild_markers()
        self._update_readout(click_time)
        self._canvas.draw_idle()

    def _handle_scroll(self, event):
        if event.inaxes is not self._axis or event.xdata is None:
            return
        left, right = self._axis.get_xlim()
        current_span = right - left
        factor = (
            SCROLL_ZOOM_FACTOR
            if event.button == 'up'
            else 1.0 / SCROLL_ZOOM_FACTOR
        )
        full_span = max(
            MINIMUM_X_WINDOW_SEC,
            self._full_limits[1] - self._full_limits[0],
        )
        new_span = min(
            full_span,
            max(MINIMUM_X_WINDOW_SEC, current_span * factor),
        )
        relative = (float(event.xdata) - left) / current_span
        new_left = float(event.xdata) - relative * new_span
        new_right = new_left + new_span
        full_left, full_right = self._full_limits
        if new_left < full_left:
            new_left = full_left
            new_right = new_left + new_span
        if new_right > full_right:
            new_right = full_right
            new_left = new_right - new_span
        self._axis.set_xlim(new_left, new_right)
        self._canvas.draw_idle()

    def _handle_limits_changed(self, axis):
        left, right = axis.get_xlim()
        self._configure_ticks(right - left)

    def _configure_ticks(self, span):
        if span <= 10.0:
            major = INITIAL_MAJOR_TICK_SEC
            minor = INITIAL_MINOR_TICK_SEC
        elif span <= 30.0:
            major, minor = 2.0, 0.5
        elif span <= 120.0:
            major, minor = 10.0, 2.0
        else:
            major, minor = 30.0, 10.0
        self._axis.xaxis.set_major_locator(MultipleLocator(major))
        self._axis.xaxis.set_minor_locator(MultipleLocator(minor))

    def _rebuild_markers(self):
        for line in self._marker_lines:
            line.remove()
        self._marker_lines.clear()
        for marker, color in (
            (self._marker_a, '#6a4c93'),
            (self._marker_b, '#f4a261'),
        ):
            if marker is not None:
                self._marker_lines.append(
                    self._axis.axvline(
                        marker, color=color, linestyle='--', linewidth=1.2
                    )
                )

    def _update_readout(self, cursor_time):
        flow_index = self._nearest_index(self._data.flow_times, cursor_time)
        imu_index = self._nearest_index(self._data.imu_times, cursor_time)
        text = (
            f't={cursor_time:8.3f}s  '
            f'flow={self._data.flow_motion[flow_index]:8.2f}px/s '
            f'norm={self._data.flow_normalized[flow_index]:.3f}  '
            f'imu={self._data.imu_motion[imu_index]:8.4f}rad/s '
            f'norm={self._data.imu_normalized[imu_index]:.3f}'
        )
        if self._marker_a is not None:
            text += f'  A={self._marker_a:.3f}s'
        if self._marker_b is not None:
            delta_ms = (self._marker_b - self._marker_a) * 1000.0
            text += f'  B={self._marker_b:.3f}s  delta={delta_ms:+.1f}ms'
        self._readout.setText(text)

    @staticmethod
    def _nearest_index(times, target):
        index = int(np.searchsorted(times, target))
        if index >= len(times):
            return len(times) - 1
        if index > 0 and target - times[index - 1] <= times[index] - target:
            return index - 1
        return index


class AnalysisWindow(QMainWindow):
    def __init__(self, bag_path, total_images):
        super().__init__()
        self.setWindowTitle(f'Analyzing {bag_path.name}')
        self.resize(1100, 760)
        self._latest_pixmap = None

        self._image_label = QLabel('Waiting for the first image...')
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setStyleSheet('background: #111111; color: #dddddd;')
        self._status_label = QLabel('Opening bag...')
        self._status_label.setStyleSheet('padding: 4px 8px;')
        self._progress = QProgressBar()
        self._progress.setRange(0, max(1, total_images))

        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(self._image_label, 1)
        layout.addWidget(self._status_label)
        layout.addWidget(self._progress)
        self.setCentralWidget(page)

        self._worker = AnalysisWorker(bag_path, total_images)
        self._worker.frame_ready.connect(self._set_frame)
        self._worker.progress_changed.connect(self._set_progress)
        self._worker.analysis_ready.connect(self._show_result)
        self._worker.analysis_failed.connect(self._show_error)
        self._worker.start()

    def _set_frame(self, image):
        self._latest_pixmap = QPixmap.fromImage(image)
        self._render_pixmap()

    def _set_progress(self, current, total, text):
        self._progress.setRange(0, max(1, total))
        self._progress.setValue(current)
        self._status_label.setText(text)

    def _show_result(self, data):
        if data.estimate is None:
            estimate_text = 'unavailable'
        else:
            estimate_text = (
                f'{data.estimate.lag_sec * 1000.0:+.0f}ms '
                f'(correlation={data.estimate.coefficient:.3f})'
            )
        print(
            f'Analysis complete: flow={len(data.flow_times)}, '
            f'imu={len(data.imu_times)}, image_lag={estimate_text}',
            flush=True,
        )
        self.setWindowTitle('IMU and camera motion analysis')
        self.setCentralWidget(PlotWidget(data))
        self._image_label = None
        self._latest_pixmap = None

    def _show_error(self, message):
        if message == 'Analysis cancelled':
            return
        print(f'Analysis failed: {message}', file=sys.stderr)
        QMessageBox.critical(self, 'Analysis failed', message)
        self.close()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._render_pixmap()

    def closeEvent(self, event):
        if self._worker.isRunning():
            self._worker.requestInterruption()
            self._worker.wait(3000)
        super().closeEvent(event)

    def _render_pixmap(self):
        if self._latest_pixmap is None or self._image_label is None:
            return
        self._image_label.setPixmap(
            self._latest_pixmap.scaled(
                self._image_label.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description='Analyze camera/IMU motion from an MCAP ROS bag.'
    )
    parser.add_argument(
        'bag',
        nargs='?',
        help='ROS bag directory; defaults to the latest log/imu_camera_* bag.',
    )
    return parser.parse_args()


def main():
    arguments = parse_args()
    try:
        bag_path = resolve_bag_path(arguments.bag)
        total_images = validate_bag(bag_path)
    except Exception as error:
        print(f'Cannot start analysis: {error}', file=sys.stderr)
        return 2

    print(f'Analyzing bag: {bag_path}', flush=True)
    app = QApplication([sys.argv[0]])
    window = AnalysisWindow(bag_path, total_images)
    signal.signal(signal.SIGINT, lambda _signum, _frame: window.close())
    signal_timer = QTimer()
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start(100)
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
