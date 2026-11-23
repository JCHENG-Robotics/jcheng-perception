"""
Laptop GPU node: asynchronous two-stream YOLO detection.

Each camera has its own bounded queue and worker thread. The ROS callback only
copies the compressed bytes and the capture timestamp; decoding and inference
happen off the executor. A full queue drops the oldest frame instead of
back-pressuring the network or the edge node. The capture timestamp is copied
unchanged into ``Detection2DArray.header.stamp`` — that is the contract that
lets the Pi 5 find the matching depth frame and TF sample.
"""

import os
import queue
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage

from jcheng_perception_core import load_gpu_config
from jcheng_perception_msgs.msg import (
    Detection2D,
    Detection2DArray,
    GpuStatus,
)
from .detectors import DetectorError, build_detector


def _sensor_qos():
    """Best-effort QoS for compressed camera streams."""
    return QoSProfile(
        depth=2,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _state_qos(depth=10):
    """Best-effort QoS for periodic detection state.

    Detections are snapshots: a stale detection is worse than a dropped one.
    Best effort also prevents a slow edge subscriber from blocking a GPU
    worker thread inside a reliable DDS write.
    """
    return QoSProfile(
        depth=depth,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _default_config_path(filename):
    """Return the installed config or a source-tree fallback."""
    try:
        share = get_package_share_directory('jcheng_perception_gpu')
        candidate = os.path.join(share, 'config', filename)
        if os.path.isfile(candidate):
            return candidate
    except Exception:  # noqa: BLE001 - source checkout without ROS index
        pass
    return os.path.abspath(os.path.join(
        os.path.dirname(__file__), '..', 'config', filename))


class CameraWorker:
    """One bounded queue, one detector and one worker thread per camera."""

    def __init__(self, camera_id, entry, node, queue_size, debug_publish,
                 debug_interval_s):
        """Create the queue, publishers and worker thread."""
        self.camera_id = camera_id
        self.entry = entry
        self.node = node
        self.queue = queue.Queue(maxsize=max(1, int(queue_size)))
        self.debug_publish = bool(debug_publish)
        self.debug_interval_ns = int(debug_interval_s * 1e9)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._detector = None
        self._model_id = 'loading'
        self._last_error = ''
        self._last_debug_ns = 0
        self.frames_received = 0
        self.frames_dropped = 0
        self.frames_failed = 0
        self.frames_published = 0
        self.inference_ms = 0.0
        self._inference_window = []
        self._stamp_window = []
        self.detection_publisher = node.create_publisher(
            Detection2DArray, entry['detections_topic'], _state_qos())
        self.debug_publisher = node.create_publisher(
            CompressedImage, entry['debug_topic'], _sensor_qos())
        self.thread = threading.Thread(
            target=self._run, name='yolo-{}'.format(camera_id), daemon=True)
        self.thread.start()

    def submit(self, payload):
        """Enqueue one compressed frame, dropping the oldest when full."""
        with self._lock:
            self.frames_received += 1
        try:
            self.queue.put_nowait(payload)
        except queue.Full:
            try:
                self.queue.get_nowait()
            except queue.Empty:
                pass
            with self._lock:
                self.frames_dropped += 1
            try:
                self.queue.put_nowait(payload)
            except queue.Full:
                pass

    def _ensure_detector(self):
        """Load the detector once, retrying with backoff after failures."""
        if self._detector is not None:
            return True
        try:
            self._detector = build_detector(
                self.entry['model'], logger=self.node.get_logger())
            self._model_id = self._detector.describe()
            self._last_error = ''
            return True
        except Exception as error:  # noqa: BLE001 - model/runtime dependent
            message = str(error)
            if message != self._last_error:
                self._last_error = message
                self.node.get_logger().error(
                    '{}: detector not loaded: {}'.format(
                        self.camera_id, error))
            time.sleep(1.0)
            return False

    def _run(self):
        """Decode, infer and publish until stopped."""
        while not self._stop.is_set():
            if not self._ensure_detector():
                continue
            try:
                payload = self.queue.get(timeout=0.2)
            except queue.Empty:
                continue
            data, stamp_ns, frame_id = payload
            buffer = np.frombuffer(data, dtype=np.uint8)
            image = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            if image is None:
                with self._lock:
                    self.frames_failed += 1
                continue
            started = time.perf_counter()
            try:
                detections = self._detector.detect(image)
            except DetectorError as error:
                with self._lock:
                    self.frames_failed += 1
                self.node.get_logger().warn(
                    '{}: inference failed: {}'.format(
                        self.camera_id, error),
                    throttle_duration_sec=5.0)
                continue
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            with self._lock:
                self.inference_ms = elapsed_ms
                self._inference_window.append(elapsed_ms)
                if len(self._inference_window) > 120:
                    self._inference_window.pop(0)
                self._stamp_window.append(stamp_ns)
                if len(self._stamp_window) > 120:
                    self._stamp_window.pop(0)
                self.frames_published += 1
                self._model_id = self._detector.describe()
            self._publish_detections(
                detections, image, stamp_ns, frame_id, elapsed_ms)
            if self.debug_publish:
                self._maybe_publish_debug(image, detections, stamp_ns,
                                          frame_id)

    def _publish_detections(self, detections, image, stamp_ns, frame_id,
                            elapsed_ms):
        """Publish one detection batch with the original capture stamp."""
        message = Detection2DArray()
        message.header.stamp = Time(nanoseconds=int(stamp_ns)).to_msg()
        message.header.frame_id = frame_id
        message.camera_id = self.camera_id
        message.model_id = self._model_id
        message.frame_seq = int(stamp_ns)
        message.image_width = int(image.shape[1])
        message.image_height = int(image.shape[0])
        message.inference_ms = float(elapsed_ms)
        for detection in detections:
            item = Detection2D()
            item.label = detection.label
            item.class_id = int(detection.class_id)
            item.score = float(detection.score)
            item.bbox_xyxy = [float(value) for value in detection.bbox_xyxy]
            keypoints = []
            for keypoint in detection.keypoints:
                keypoints.extend([
                    float(keypoint.x),
                    float(keypoint.y),
                    float(keypoint.confidence),
                ])
            item.keypoints = keypoints
            item.track_hint = int(detection.track_hint)
            message.detections.append(item)
        self.detection_publisher.publish(message)

    def _maybe_publish_debug(self, image, detections, stamp_ns, frame_id):
        """Publish an annotated debug frame at a bounded rate."""
        now_ns = self.node.get_clock().now().nanoseconds
        if now_ns - self._last_debug_ns < self.debug_interval_ns:
            return
        self._last_debug_ns = now_ns
        canvas = image.copy()
        for detection in detections:
            x0, y0, x1, y1 = [int(round(value))
                              for value in detection.bbox_xyxy]
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 255, 0), 2)
            label = '{} {:.2f}'.format(detection.label, detection.score)
            cv2.putText(canvas, label, (x0, max(12, y0 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            for keypoint in detection.keypoints:
                cv2.circle(
                    canvas, (int(round(keypoint.x)),
                             int(round(keypoint.y))),
                    4, (0, 0, 255), -1)
        ok, encoded = cv2.imencode(
            '.jpg', canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return
        message = CompressedImage()
        message.header.stamp = Time(nanoseconds=int(stamp_ns)).to_msg()
        message.header.frame_id = frame_id
        message.format = 'jpeg'
        message.data = encoded.tobytes()
        self.debug_publisher.publish(message)

    def status(self):
        """Return a snapshot of counters used by the status timer."""
        with self._lock:
            inference = float(np.mean(self._inference_window)) \
                if self._inference_window else 0.0
            stamps = list(self._stamp_window)
            fps = 0.0
            if len(stamps) >= 2 and stamps[-1] > stamps[0]:
                fps = (len(stamps) - 1) * 1e9 / float(
                    stamps[-1] - stamps[0])
            return {
                'frames_received': self.frames_received,
                'frames_dropped': self.frames_dropped,
                'frames_failed': self.frames_failed,
                'frames_published': self.frames_published,
                'inference_ms': inference,
                'fps': fps,
                'model_id': self._model_id,
            }

    def stop(self):
        """Stop the worker thread."""
        self._stop.set()
        self.thread.join(timeout=2.0)


class PerceptionGpuNode(Node):
    """ROS 2 adapter around the per-camera YOLO workers."""

    def __init__(self):
        """Load the GPU config and start one worker per camera."""
        super().__init__('jcheng_perception_gpu')
        cv2.setNumThreads(1)
        self.declare_parameter(
            'gpu_config', _default_config_path('gpu.yaml'))
        self.declare_parameter('debug_publish', False)
        self.declare_parameter('queue_size', 0)
        self.declare_parameter('status_topic', '/perception/gpu_status')
        config_path = str(self.get_parameter('gpu_config').value)
        self.config = load_gpu_config(config_path)
        debug_override = bool(
            self.get_parameter('debug_publish').value)
        queue_override = int(self.get_parameter('queue_size').value)
        node_config = self.config['node']
        self.workers = {}
        for camera_id, entry in self.config['cameras'].items():
            queue_size = queue_override or int(
                node_config.get('queue_size', 2))
            worker = CameraWorker(
                camera_id, entry, self,
                queue_size=queue_size,
                debug_publish=debug_override or bool(
                    node_config.get('debug_publish', False)),
                debug_interval_s=float(
                    node_config.get('debug_interval_s', 1.0)),
            )
            self.workers[camera_id] = worker
            self.create_subscription(
                CompressedImage, entry['input_topic'],
                lambda message, camera=camera_id: self.on_image(camera,
                                                                message),
                _sensor_qos())
        self.status_publisher = self.create_publisher(
            GpuStatus, str(self.get_parameter('status_topic').value),
            _state_qos(5))
        self.create_timer(
            float(node_config.get('stats_interval_s', 2.0)),
            self.publish_status)
        self.get_logger().info(
            'GPU detector ready: {}'.format(','.join(self.workers)))

    def on_image(self, camera_id, message):
        """Hand the compressed frame to the camera worker."""
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = Time.from_msg(message.header.stamp).nanoseconds
        if stamp_ns <= 0:
            stamp_ns = now_ns
        self.workers[camera_id].submit((
            bytes(message.data),
            int(stamp_ns),
            str(message.header.frame_id),
        ))

    def publish_status(self):
        """Publish aggregate worker health."""
        message = GpuStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        for camera_id, worker in self.workers.items():
            status = worker.status()
            message.camera_ids.append(camera_id)
            message.fps.append(float(status['fps']))
            message.inference_ms.append(float(status['inference_ms']))
            message.frames_received.append(
                int(status['frames_received']))
            message.frames_dropped.append(int(status['frames_dropped']))
            message.frames_failed.append(int(status['frames_failed']))
            message.model_ids.append(str(status['model_id']))
        self.status_publisher.publish(message)

    def destroy_node(self):
        """Stop workers before destroying the node."""
        for worker in self.workers.values():
            worker.stop()
        super().destroy_node()


def main(args=None):
    """Run the GPU node."""
    rclpy.init(args=args)
    node = PerceptionGpuNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
