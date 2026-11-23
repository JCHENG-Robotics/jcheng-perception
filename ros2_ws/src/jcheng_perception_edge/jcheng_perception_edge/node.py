"""
Pi 5 edge node: capture, compress, time-match, back-project, fuse.

The edge node intentionally has **no detector import**. It publishes JPEG
frames with their acquisition stamps, receives detections from the GPU host,
finds the depth frame and TF sample for that exact stamp, converts boxes to 3-D
base-frame observations and runs the distance-weighted fusion + Kalman update.
"""

import collections
import faulthandler
import os
import queue
import signal
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovariance
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from jcheng_perception_core import (
    Detection,
    Keypoint,
    PerceptionPipeline,
    Phase,
    load_camera_specs,
    load_fusion_config,
    load_static_frames,
    make_transform,
    matrix_from_quaternion,
)
from jcheng_perception_msgs.msg import (
    Detection2DArray,
    FusedTarget,
    FusedTargetArray,
    PerceptionStatus,
    TargetObservation as TargetObservationMsg,
)


def _sensor_qos():
    """Best-effort QoS for camera streams."""
    return QoSProfile(
        depth=2,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _state_qos(depth=10):
    """Best-effort QoS for periodic perception state.

    Targets, observations and detections are periodic snapshots, not commands.
    A slow or dead subscriber must never block the publisher or the edge
    executor; best effort keeps the latest value and drops stale ones.
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _latched_qos():
    """Transient-local QoS for CameraInfo."""
    return QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _default_share_path(filename):
    """Return an installed config file, with a source-tree fallback."""
    try:
        share = get_package_share_directory('jcheng_perception_edge')
        candidate = os.path.join(share, 'config', filename)
        if os.path.isfile(candidate):
            return candidate
    except Exception:  # noqa: BLE001 - source checkout without ROS index
        pass
    candidate = os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'config', filename))
    return candidate


def transform_to_matrix(transform_stamped):
    """Convert a ``TransformStamped`` into a 4x4 numpy matrix."""
    rotation = transform_stamped.transform.rotation
    translation = transform_stamped.transform.translation
    matrix = matrix_from_quaternion(
        [rotation.x, rotation.y, rotation.z, rotation.w])
    return make_transform(
        matrix, [translation.x, translation.y, translation.z])


def header_stamp_ns(header, fallback_ns):
    """Return a header stamp in nanoseconds, falling back when unset."""
    stamp = Time.from_msg(header.stamp).nanoseconds
    return stamp if stamp > 0 else int(fallback_ns)


def covariance6x6(covariance3, yaw_sigma_rad, has_orientation):
    """Build the 6x6 pose covariance with unobserved roll/pitch marked."""
    covariance = np.zeros(36, dtype=np.float64)
    covariance[:9] = np.asarray(covariance3, dtype=np.float64).reshape(9)
    covariance[21] = 1.0
    covariance[28] = 1.0
    covariance[35] = max(1e-4, float(yaw_sigma_rad) ** 2) \
        if has_orientation else 1.0
    return covariance.tolist()


class UplinkWorker:
    """Encode and publish one camera's JPEG stream off the ROS executor.

    JPEG compression and DDS publication are deliberately moved off the image
    callback. The callback only copies the frame into a bounded queue (depth 2),
    so a slow network or a JPEG stall cannot delay depth ingestion, TF sampling
    or the fusion timer. A full queue drops the oldest frame; stale images are
    never useful to an asynchronous detector.
    """

    def __init__(self, camera_id, spec, publisher, pipeline, lock, logger):
        """Create the queue and start the encoder thread."""
        self.camera_id = camera_id
        self.spec = spec
        self.publisher = publisher
        self.pipeline = pipeline
        self.lock = lock
        self.logger = logger
        self.queue = queue.Queue(maxsize=2)
        self._stop = threading.Event()
        self.thread = threading.Thread(
            target=self._run, name='uplink-{}'.format(camera_id), daemon=True)
        self.thread.start()

    def submit(self, image, stamp_ns, frame_id):
        """Queue a frame for compression, dropping the oldest when full."""
        payload = (image, int(stamp_ns), str(frame_id))
        try:
            self.queue.put_nowait(payload)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            with self.lock:
                self.pipeline.note_rate_limited(self.camera_id)
            try:
                self.queue.put_nowait(payload)
            except queue.Full:
                pass

    def _run(self):
        """Compress and publish until stopped."""
        while not self._stop.is_set():
            try:
                image, stamp_ns, frame_id = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            started = time.perf_counter()
            try:
                ok, encoded = cv2.imencode(
                    '.jpg', image,
                    [int(cv2.IMWRITE_JPEG_QUALITY),
                     int(self.spec.jpeg_quality)])
            except cv2.error as error:
                with self.lock:
                    self.pipeline.note_encode_failure(self.camera_id)
                self.logger.warn(
                    '{}: JPEG encode failed: {}'.format(
                        self.camera_id, error),
                    throttle_duration_sec=5.0)
                continue
            if not ok:
                with self.lock:
                    self.pipeline.note_encode_failure(self.camera_id)
                continue
            message = CompressedImage()
            message.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
            message.header.frame_id = frame_id
            message.format = 'jpeg'
            message.data = encoded.tobytes()
            self.publisher.publish(message)
            with self.lock:
                self.pipeline.note_uplink(self.camera_id)
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            if elapsed_ms > 50.0:
                self.logger.warn(
                    '{}: JPEG encode+publish took {:.1f} ms'.format(
                        self.camera_id, elapsed_ms),
                    throttle_duration_sec=5.0)

    def stop(self):
        """Stop the encoder thread."""
        self._stop.set()
        self.thread.join(timeout=2.0)


class PerceptionEdgeNode(Node):
    """ROS 2 adapter around :class:`PerceptionPipeline`."""

    def __init__(self):
        """Load configuration, create subscriptions and start timers."""
        super().__init__('jcheng_perception_edge')
        self.declare_parameter(
            'cameras_config', _default_share_path('cameras.yaml'))
        self.declare_parameter(
            'fusion_config', _default_share_path('fusion.yaml'))
        self.declare_parameter(
            'frames_config', _default_share_path('frames.yaml'))
        self.declare_parameter('target_frame', '')
        self.declare_parameter('publish_rate_hz', 0.0)
        self.declare_parameter('status_rate_hz', 0.0)
        self.declare_parameter('enable_uplink', True)
        self.declare_parameter('timing_log', False)
        cv2.setNumThreads(1)

        cameras_path = str(
            self.get_parameter('cameras_config').value)
        fusion_path = str(self.get_parameter('fusion_config').value)
        frames_path = str(self.get_parameter('frames_config').value)
        self.specs = load_camera_specs(cameras_path)
        self.fusion = load_fusion_config(fusion_path)
        target_frame = str(self.get_parameter('target_frame').value).strip()
        if target_frame:
            self.fusion.target_frame = target_frame
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate > 0.0:
            self.fusion.publish_rate_hz = publish_rate
        status_rate = float(self.get_parameter('status_rate_hz').value)
        if status_rate > 0.0:
            self.fusion.status_rate_hz = status_rate
        self.enable_uplink = bool(
            self.get_parameter('enable_uplink').value)
        self.timing_log = bool(self.get_parameter('timing_log').value)
        self.static_fallbacks = load_static_frames(frames_path)
        for camera_id, spec in self.specs.items():
            if spec.static_extrinsic is not None:
                self.static_fallbacks.setdefault(
                    camera_id, spec.static_extrinsic)

        self.pipeline = PerceptionPipeline(
            self.specs, self.fusion, self.static_fallbacks)
        self.bridge = CvBridge()
        self._lock = threading.RLock()
        self._camera_info = {}
        self._seq = {camera_id: 0 for camera_id in self.specs}
        self._last_uplink_ns = {camera_id: 0 for camera_id in self.specs}
        self._frame_stamps = {
            camera_id: collections.deque(maxlen=90)
            for camera_id in self.specs
        }
        self._last_status_ns = 0
        self._last_tf_warn_ns = 0
        self._last_static_sample_ns = {}
        self._timing = collections.defaultdict(
            lambda: collections.deque(maxlen=120))

        sensor_group = ReentrantCallbackGroup()
        detection_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()
        tf_group = MutuallyExclusiveCallbackGroup()

        self.uplink_publishers = {}
        self.uplink_workers = {}
        self.observation_publishers = {}
        self.detection_subscriptions = {}
        for camera_id, spec in self.specs.items():
            self.uplink_publishers[camera_id] = self.create_publisher(
                CompressedImage, spec.uplink_topic, _sensor_qos())
            if self.enable_uplink:
                self.uplink_workers[camera_id] = UplinkWorker(
                    camera_id, spec, self.uplink_publishers[camera_id],
                    self.pipeline, self._lock, self.get_logger())
            self.observation_publishers[camera_id] = self.create_publisher(
                TargetObservationMsg,
                '/perception/observations/{}'.format(camera_id),
                _state_qos(),
            )
            rgb_type = CompressedImage if spec.rgb_compressed else Image
            self.create_subscription(
                rgb_type, spec.rgb_topic,
                lambda message, camera=camera_id: self.on_image(
                    camera, message),
                _sensor_qos(), callback_group=sensor_group)
            depth_type = CompressedImage if spec.depth_compressed else Image
            self.create_subscription(
                depth_type, spec.depth_topic,
                lambda message, camera=camera_id: self.on_depth(
                    camera, message),
                _sensor_qos(), callback_group=sensor_group)
            self.create_subscription(
                CameraInfo, spec.camera_info_topic,
                lambda message, camera=camera_id: self.on_camera_info(
                    camera, message),
                _latched_qos(), callback_group=sensor_group)
            self.detection_subscriptions[camera_id] = self.create_subscription(
                Detection2DArray, spec.detection_topic,
                lambda message, camera=camera_id: self.on_detections(
                    camera, message),
                _state_qos(), callback_group=detection_group)

        self.target_publisher = self.create_publisher(
            FusedTargetArray, '/perception/targets', _state_qos())
        self.status_publisher = self.create_publisher(
            PerceptionStatus, '/perception/status', _state_qos(5))
        self.create_service(
            Trigger, '~/drop_recheck', self.on_drop_recheck,
            callback_group=detection_group)
        self.create_service(
            Trigger, '~/reset', self.on_reset,
            callback_group=detection_group)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(
            1.0 / max(1.0, float(self.fusion.pose_sample_hz)),
            self.on_tf_timer,
            callback_group=tf_group,
        )
        self.create_timer(
            1.0 / max(1.0, float(self.fusion.publish_rate_hz)),
            self.on_publish_timer,
            callback_group=timer_group,
        )
        self.get_logger().info(
            'edge ready: cameras={} target_frame={} uplink={}'.format(
                ','.join(self.specs), self.fusion.target_frame,
                self.enable_uplink))

    # ------------------------------------------------------------------ input
    def _now_ns(self):
        """Return the current ROS clock in nanoseconds."""
        return self.get_clock().now().nanoseconds

    def _camera_model(self, camera_id, width, height):
        """Return ``(K, D)`` for the current image size, or ``(None, None)``."""
        info = self._camera_info.get(camera_id)
        spec = self.specs[camera_id]
        if info is not None and info.width == width and info.height == height:
            return np.asarray(info.k, dtype=np.float64).reshape(3, 3), \
                np.asarray(info.d, dtype=np.float64)
        if spec.intrinsics is not None:
            return np.asarray(spec.intrinsics, dtype=np.float64).copy(), \
                np.asarray(
                    spec.distortion
                    if spec.distortion is not None
                    else np.zeros(5), dtype=np.float64)
        return None, None

    def on_image(self, camera_id, message):
        """Time one image callback around the implementation."""
        started = time.perf_counter()
        try:
            self._on_image_impl(camera_id, message)
        finally:
            self._timing['image'].append(
                (time.perf_counter() - started) * 1000.0)

    def _on_image_impl(self, camera_id, message):
        """Store metadata for every frame and queue an uplink frame."""
        spec = self.specs[camera_id]
        now_ns = self._now_ns()
        stamp_ns = header_stamp_ns(message.header, now_ns)
        try:
            if isinstance(message, CompressedImage):
                buffer = np.frombuffer(message.data, dtype=np.uint8)
                image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            else:
                image = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding='bgr8')
        except Exception as error:  # noqa: BLE001 - malformed camera frame
            self.get_logger().warn(
                '{}: image decode failed: {}'.format(camera_id, error),
                throttle_duration_sec=5.0)
            return
        if image is None or image.ndim != 3:
            return
        height, width = image.shape[:2]
        camera_matrix, distortion = self._camera_model(
            camera_id, width, height)
        if camera_matrix is None:
            self.get_logger().warn(
                '{}: waiting for CameraInfo or configured intrinsics'.format(
                    camera_id), throttle_duration_sec=5.0)
            return
        self._seq[camera_id] += 1
        seq = self._seq[camera_id]
        frame_id = str(message.header.frame_id or spec.optical_frame)
        with self._lock:
            self.pipeline.ingest_frame(
                camera_id, seq, stamp_ns, now_ns, width, height,
                frame_id, camera_matrix, distortion)
        self._frame_stamps[camera_id].append(now_ns)
        if not self.enable_uplink:
            return
        # A hard 1/fps interval would drop every second frame when the camera
        # jitters around its nominal period (e.g. 95-104 ms at 10 Hz). The 15%
        # tolerance keeps the nominal rate while still bounding the bandwidth.
        min_interval_ns = int(1e9 / max(spec.max_uplink_fps, 1e-6) * 0.85)
        if now_ns - self._last_uplink_ns[camera_id] < min_interval_ns:
            with self._lock:
                self.pipeline.note_rate_limited(camera_id)
            return
        self._last_uplink_ns[camera_id] = now_ns
        self.uplink_workers[camera_id].submit(image, stamp_ns, frame_id)

    def on_depth(self, camera_id, message):
        """Time one depth callback around the implementation."""
        started = time.perf_counter()
        try:
            self._on_depth_impl(camera_id, message)
        finally:
            self._timing['depth'].append(
                (time.perf_counter() - started) * 1000.0)

    def _on_depth_impl(self, camera_id, message):
        """Store one depth frame keyed by its capture timestamp."""
        spec = self.specs[camera_id]
        now_ns = self._now_ns()
        stamp_ns = header_stamp_ns(message.header, now_ns)
        try:
            if isinstance(message, CompressedImage):
                buffer = np.frombuffer(message.data, dtype=np.uint8)
                depth = cv2.imdecode(buffer, cv2.IMREAD_UNCHANGED)
                encoding = ''
            else:
                depth = self.bridge.imgmsg_to_cv2(
                    message, desired_encoding='passthrough')
                encoding = str(message.encoding)
        except Exception as error:  # noqa: BLE001 - malformed depth frame
            self.get_logger().warn(
                '{}: depth decode failed: {}'.format(camera_id, error),
                throttle_duration_sec=5.0)
            return
        if depth is None or depth.ndim != 2:
            return
        if depth.dtype == np.float32 or encoding == '32FC1':
            metric = True
            scale = 1.0
        elif depth.dtype == np.uint16 or encoding in ('16UC1', 'mono16'):
            metric = False
            scale = float(spec.depth_scale)
        else:
            self.get_logger().warn(
                '{}: unsupported depth encoding {} / {}'.format(
                    camera_id, encoding, depth.dtype),
                throttle_duration_sec=5.0)
            return
        with self._lock:
            self.pipeline.ingest_depth(
                camera_id, stamp_ns, depth, scale_m=scale, metric=metric)

    def on_camera_info(self, camera_id, message):
        """Cache the latest intrinsics."""
        self._camera_info[camera_id] = message

    def on_detections(self, camera_id, message):
        """Time one detection/fusion callback around the implementation."""
        started = time.perf_counter()
        try:
            self._on_detections_impl(camera_id, message)
        finally:
            self._timing['detections'].append(
                (time.perf_counter() - started) * 1000.0)

    def _on_detections_impl(self, camera_id, message):
        """Convert detections and run the time-matched fusion."""
        now_ns = self._now_ns()
        stamp_ns = header_stamp_ns(message.header, now_ns)
        detections = []
        for item in message.detections:
            keypoints = []
            values = list(item.keypoints)
            for index in range(0, len(values) - 2, 3):
                keypoints.append(Keypoint(
                    x=float(values[index]),
                    y=float(values[index + 1]),
                    confidence=float(values[index + 2]),
                ))
            detections.append(Detection(
                label=str(item.label),
                score=float(item.score),
                bbox_xyxy=tuple(float(value) for value in item.bbox_xyxy),
                class_id=int(item.class_id),
                keypoints=keypoints,
                track_hint=int(item.track_hint),
            ))
        with self._lock:
            self.pipeline.ingest_detections(
                camera_id, int(message.frame_seq), stamp_ns, now_ns,
                detections, model_id=str(message.model_id))
            targets = self.pipeline.flush(now_ns)
            observations = self.pipeline.take_observations()
        if observations:
            self.publish_observations(observations)
        self.publish_targets(targets, now_ns)

    def on_tf_timer(self):
        """Time the TF sampling callback."""
        started = time.perf_counter()
        try:
            self._on_tf_timer_impl()
        finally:
            self._timing['tf'].append(
                (time.perf_counter() - started) * 1000.0)

    def _on_tf_timer_impl(self):
        """Sample the moving base<-camera transform into the pose ring.

        A rigidly mounted camera is sampled at 0.5 Hz: its transform cannot
        change, and a 100 Hz lookup for both cameras was measured to saturate
        one edge core. The moving wrist camera is sampled at the configured
        rate (30 Hz by default), which is enough for linear/slerp interpolation
        between 8-15 Hz depth frames.
        """
        now_ns = self._now_ns()
        for camera_id, spec in self.specs.items():
            if spec.pose_static:
                last = self._last_static_sample_ns.get(camera_id, 0)
                if now_ns - last < 2_000_000_000:
                    continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.fusion.target_frame, spec.optical_frame, Time())
            except Exception as error:  # noqa: BLE001 - TF is best effort
                with self._lock:
                    self.pipeline.note_tf_failure(camera_id)
                if now_ns - self._last_tf_warn_ns > 5_000_000_000:
                    self._last_tf_warn_ns = now_ns
                    self.get_logger().warn(
                        'TF unavailable {} -> {}: {}'.format(
                            self.fusion.target_frame, spec.optical_frame,
                            error))
                continue
            matrix = transform_to_matrix(transform)
            with self._lock:
                self.pipeline.ingest_pose(camera_id, now_ns, matrix)
            if spec.pose_static:
                self._last_static_sample_ns[camera_id] = now_ns

    # --------------------------------------------------------------- output
    def on_publish_timer(self):
        """Time the periodic prediction/publication callback."""
        started = time.perf_counter()
        try:
            self._on_publish_timer_impl()
        finally:
            self._timing['publish'].append(
                (time.perf_counter() - started) * 1000.0)

    def _on_publish_timer_impl(self):
        """Publish predicted targets at a fixed rate."""
        now_ns = self._now_ns()
        with self._lock:
            targets = self.pipeline.flush(now_ns)
            observations = self.pipeline.take_observations()
        if observations:
            self.publish_observations(observations)
        self.publish_targets(targets, now_ns)
        if now_ns - self._last_status_ns >= int(
                1e9 / max(self.fusion.status_rate_hz, 1e-6)):
            with self._lock:
                status = self.pipeline.status(now_ns)
            self.publish_status(status, now_ns)
            self._last_status_ns = now_ns

    def publish_observations(self, observations):
        """Publish the per-camera 3-D observations (debug and evidence)."""
        for observation in observations:
            spec = self.specs[observation.camera_id]
            message = TargetObservationMsg()
            message.header.stamp = Time(
                nanoseconds=int(observation.capture_ns)).to_msg()
            message.header.frame_id = self.fusion.target_frame
            message.camera_id = observation.camera_id
            message.role = observation.role.message_value
            message.label = observation.label
            message.class_id = int(observation.class_id)
            message.detection_score = float(observation.detection_score)
            message.keypoint_score = float(observation.keypoint_score)
            message.pose = self._pose_with_covariance(
                observation.position_base,
                observation.orientation_base,
                observation.covariance_base,
                observation.orientation_sigma_rad,
            )
            message.depth_m = float(observation.depth_m)
            message.valid_depth_fraction = float(
                observation.valid_depth_fraction)
            message.occlusion_ratio = float(observation.occlusion_ratio)
            message.view_weight = float(observation.view_weight)
            message.frame_seq = int(observation.frame_seq)
            message.camera_frame = spec.optical_frame
            self.observation_publishers[observation.camera_id].publish(
                message)

    def publish_targets(self, targets, now_ns):
        """Publish the fused target array predicted to ``now_ns``."""
        external_ids = [
            camera_id for camera_id, spec in self.specs.items()
            if spec.role.value == 'external']
        wrist_ids = [
            camera_id for camera_id, spec in self.specs.items()
            if spec.role.value == 'wrist']
        message = FusedTargetArray()
        message.header.stamp = Time(nanoseconds=int(now_ns)).to_msg()
        message.header.frame_id = self.fusion.target_frame
        for target in targets:
            item = FusedTarget()
            item.header.stamp = Time(nanoseconds=int(now_ns)).to_msg()
            item.header.frame_id = self.fusion.target_frame
            item.target_id = target.target_id
            item.label = target.label
            item.class_id = int(target.class_id)
            item.pose = self._pose_with_covariance(
                target.position_base,
                target.orientation_base,
                target.covariance_6x6[:3, :3],
                float(np.sqrt(max(1e-6, target.covariance_6x6[5, 5]))),
                covariance6=target.covariance_6x6,
            )
            item.confidence = float(target.confidence)
            item.phase = int(target.phase)
            item.status = int(target.status)
            item.weight_external = float(sum(
                target.weights.get(camera_id, 0.0)
                for camera_id in external_ids))
            item.weight_wrist = float(sum(
                target.weights.get(camera_id, 0.0)
                for camera_id in wrist_ids))
            item.depth_external_m = float(max(
                [target.depth_by_camera.get(camera_id, 0.0)
                 for camera_id in external_ids] or [0.0]))
            item.depth_wrist_m = float(max(
                [target.depth_by_camera.get(camera_id, 0.0)
                 for camera_id in wrist_ids] or [0.0]))
            item.occluded = bool(target.occluded)
            item.recheck_required = bool(target.recheck_required)
            item.source_cameras = list(target.source_cameras)
            item.age_s = float(target.age_s)
            message.targets.append(item)
        self.target_publisher.publish(message)

    def publish_status(self, status, now_ns):
        """Publish the edge health snapshot."""
        message = PerceptionStatus()
        message.header.stamp = Time(nanoseconds=int(now_ns)).to_msg()
        message.header.frame_id = self.fusion.target_frame
        message.camera_ids = list(self.specs)
        message.camera_fps = [
            float(self._fps(camera_id)) for camera_id in self.specs]
        message.total_frames = int(self.pipeline.total_frames)
        message.total_detections = int(self.pipeline.total_detections)
        message.detections_matched = int(self.pipeline.total_matched)
        message.detections_dropped = int(self.pipeline.total_dropped)
        total = max(1, self.pipeline.total_detections)
        message.match_success_rate = float(
            self.pipeline.total_matched / total)
        latencies = [
            health.mean_latency_ms
            for health in self.pipeline.health.values()
            if health.mean_latency_ms > 0.0
        ]
        message.mean_detection_latency_ms = float(
            np.mean(latencies) if latencies else 0.0)
        ages = [
            health.last_depth_age_ms
            for health in self.pipeline.health.values()
        ]
        message.mean_depth_match_age_ms = float(
            np.mean(ages) if ages else 0.0)
        message.phase = int(status.phase)
        message.gpu_alive = bool(status.gpu_alive)
        message.active_tracks = int(min(255, status.active_tracks))
        message.confirmed_tracks = int(
            min(255, status.confirmed_tracks))
        message.notes = list(status.notes)
        self.status_publisher.publish(message)
        if self.timing_log:
            parts = []
            for key in sorted(self._timing):
                values = self._timing[key]
                if values:
                    parts.append('{}={:.1f}/{:.1f}'.format(
                        key, float(np.mean(values)), max(values)))
            self.get_logger().info(
                'timing ms(mean/max) {} pending={}'.format(
                    ' '.join(parts), len(self.pipeline.pending)))

    def _pose_with_covariance(self, position, orientation, covariance3,
                              yaw_sigma, covariance6=None):
        """Build a ``PoseWithCovariance`` with a 6x6 row-major covariance."""
        message = PoseWithCovariance()
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        if orientation is None:
            message.pose.orientation.w = 1.0
        else:
            message.pose.orientation.x = float(orientation[0])
            message.pose.orientation.y = float(orientation[1])
            message.pose.orientation.z = float(orientation[2])
            message.pose.orientation.w = float(orientation[3])
        if covariance6 is None:
            message.covariance = covariance6x6(
                covariance3, yaw_sigma, orientation is not None)
        else:
            message.covariance = [
                float(value) for value in np.asarray(
                    covariance6, dtype=np.float64).reshape(36)
            ]
        return message

    def _fps(self, camera_id):
        """Return the recent camera frame rate."""
        stamps = self._frame_stamps[camera_id]
        if len(stamps) < 2:
            return 0.0
        span = stamps[-1] - stamps[0]
        if span <= 0:
            return 0.0
        return (len(stamps) - 1) * 1e9 / float(span)

    # -------------------------------------------------------------- services
    def on_drop_recheck(self, request, response):
        """Force the external-view drop recheck phase for a bounded time."""
        now_ns = self._now_ns()
        with self._lock:
            self.pipeline.set_phase_override(Phase.DROP_RECHECK, now_ns)
        response.success = True
        response.message = 'DROP_RECHECK for {:.1f} s'.format(
            self.fusion.recheck_duration_s)
        return response

    def on_reset(self, request, response):
        """Drop all tracks and pending detections."""
        with self._lock:
            self.pipeline.reset()
        response.success = True
        response.message = 'perception tracks reset'
        return response

    def destroy_node(self):
        """Stop the uplink encoder threads before destroying the node."""
        for worker in self.uplink_workers.values():
            worker.stop()
        super().destroy_node()


def main(args=None):
    """Run the edge node."""
    faulthandler.enable()
    try:
        faulthandler.register(signal.SIGUSR1, all_threads=True)
    except (AttributeError, ValueError):
        pass
    rclpy.init(args=args)
    node = PerceptionEdgeNode()
    # A single executor thread plus the two uplink encoder threads is both
    # lower-jitter and easier to reason about than a multi-threaded executor.
    # Every callback is kept short; JPEG compression is already off-thread.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
