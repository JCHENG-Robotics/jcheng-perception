"""
Detector adapters for the GPU host.

The vendor ArmPi Ultra YOLO packages (``yolov8_detect`` and the
``yolo_detect`` examples) load Ultralytics models, including OpenVINO exports,
and publish boxes with class names. This module keeps that model format but
returns transport-neutral ``Detection`` objects; the ROS node adds the capture
timestamp and camera identity.

The mock detector is a colour threshold. It exists so the complete two-machine
pipeline can be exercised without a model file, and so CI never needs a GPU.
"""

import math
import os

import cv2
import numpy as np

from jcheng_perception_core import Detection, Keypoint


class DetectorError(RuntimeError):
    """Raised when a detector cannot be loaded or used."""


def _to_numpy(value):
    """Convert an Ultralytics tensor/array to a numpy array."""
    if hasattr(value, 'cpu'):
        value = value.cpu().numpy()
    return np.asarray(value)


class BaseDetector:
    """Detector interface used by the GPU worker threads."""

    def describe(self):
        """Return a short model identity for logs and messages."""
        raise NotImplementedError

    def detect(self, image_bgr):
        """Return a list of :class:`Detection` for one BGR image."""
        raise NotImplementedError


class MockColorDetector(BaseDetector):
    """Detect the largest red blob; used for offline end-to-end tests."""

    def __init__(self, config):
        """Store the HSV thresholds and label."""
        self.label = str(config.get('label', 'target'))
        self.min_area = int(config.get('min_area', 80))
        self.lower1 = np.array(
            config.get('hsv_lower', [0, 120, 80]), dtype=np.uint8)
        self.upper1 = np.array(
            config.get('hsv_upper', [10, 255, 255]), dtype=np.uint8)
        self.lower2 = np.array(
            config.get('hsv_lower2', [170, 120, 80]), dtype=np.uint8)
        self.upper2 = np.array(
            config.get('hsv_upper2', [180, 255, 255]), dtype=np.uint8)

    def describe(self):
        """Return the fixed mock identity."""
        return 'mock-color-red'

    def detect(self, image_bgr):
        """Return the largest red contour as a box plus centre keypoint."""
        hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower1, self.upper1)
        mask |= cv2.inRange(hsv, self.lower2, self.upper2)
        kernel = np.ones((3, 3), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return []
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area:
            return []
        x, y, width, height = cv2.boundingRect(contour)
        centre_x = x + width / 2.0
        centre_y = y + height / 2.0
        return [Detection(
            label=self.label,
            score=1.0,
            bbox_xyxy=(float(x), float(y),
                       float(x + width), float(y + height)),
            class_id=0,
            keypoints=[Keypoint(centre_x, centre_y, 1.0)],
        )]


class UltralyticsDetector(BaseDetector):
    """Ultralytics YOLO detector (``.pt``, ``.onnx`` or OpenVINO export)."""

    def __init__(self, config):
        """Load the model and store inference parameters."""
        path = str(config.get('path', '')).strip()
        if not path:
            raise DetectorError('model.path is empty')
        if not os.path.exists(path):
            raise DetectorError('model path does not exist: {}'.format(path))
        try:
            from ultralytics import YOLO
        except ImportError as error:
            raise DetectorError(
                'ultralytics is not installed; install the GPU extras or '
                'use backend: mock') from error
        self.path = path
        self.task = str(config.get('task', 'detect'))
        self.imgsz = int(config.get('imgsz', 640))
        self.conf = float(config.get('conf', 0.35))
        self.iou = float(config.get('iou', 0.45))
        self.device = str(config.get('device', '0'))
        self.half = bool(config.get('half', False))
        self.max_detections = int(config.get('max_detections', 20))
        self.labels = list(config.get('labels') or [])
        self.with_keypoints = bool(config.get('keypoints', False))
        try:
            self.model = YOLO(path, task=self.task)
        except Exception as error:  # noqa: BLE001 - model/runtime dependent
            raise DetectorError(
                'failed to load {}: {}'.format(path, error)) from error
        self._mtime_ns = self._stat_mtime_ns(path)
        self.model_id = '{}:{}'.format(
            os.path.basename(path.rstrip('/')), self._mtime_ns)

    @staticmethod
    def _stat_mtime_ns(path):
        """Return the newest mtime in a model file or directory."""
        try:
            if os.path.isdir(path):
                stamps = [
                    os.path.getmtime(os.path.join(path, name))
                    for name in os.listdir(path)
                    if os.path.isfile(os.path.join(path, name))
                ]
                return int(max(stamps) * 1e9) if stamps else 0
            return int(os.path.getmtime(path) * 1e9)
        except OSError:
            return 0

    def describe(self):
        """Return the model identity including its mtime."""
        return self.model_id

    def _label(self, class_id, names):
        """Resolve a class id to a configured or model-provided label."""
        if 0 <= class_id < len(self.labels):
            return str(self.labels[class_id])
        if isinstance(names, dict):
            return str(names.get(class_id, class_id))
        if 0 <= class_id < len(names):
            return str(names[class_id])
        return str(class_id)

    def detect(self, image_bgr):
        """Run inference and return boxes plus optional keypoints."""
        try:
            results = self.model.predict(
                image_bgr,
                imgsz=self.imgsz,
                conf=self.conf,
                iou=self.iou,
                device=self.device,
                half=self.half,
                verbose=False,
                max_det=self.max_detections,
            )
        except Exception as error:  # noqa: BLE001 - runtime dependent
            raise DetectorError('inference failed: {}'.format(error)) from error
        if not results:
            return []
        result = results[0]
        names = getattr(result, 'names', {})
        detections = []

        keypoint_arrays = None
        if self.with_keypoints and getattr(result, 'keypoints', None) is not None:
            keypoint_arrays = result.keypoints

        boxes = getattr(result, 'boxes', None)
        if boxes is not None and len(boxes) > 0:
            xyxy = _to_numpy(boxes.xyxy)
            scores = _to_numpy(boxes.conf)
            classes = _to_numpy(boxes.cls).astype(int)
            for index in range(len(boxes)):
                class_id = int(classes[index])
                keypoints = []
                if keypoint_arrays is not None and index < len(keypoint_arrays):
                    xy = _to_numpy(keypoint_arrays.xy[index])
                    conf = _to_numpy(keypoint_arrays.conf[index]) \
                        if keypoint_arrays.conf is not None else np.ones(
                            len(xy))
                    for point_index in range(len(xy)):
                        keypoints.append(Keypoint(
                            float(xy[point_index][0]),
                            float(xy[point_index][1]),
                            float(conf[point_index]),
                        ))
                detections.append(Detection(
                    label=self._label(class_id, names),
                    score=float(scores[index]),
                    bbox_xyxy=tuple(float(value) for value in xyxy[index]),
                    class_id=class_id,
                    keypoints=keypoints,
                ))
            return detections

        obb = getattr(result, 'obb', None)
        if obb is not None and len(obb) > 0:
            xywhr = _to_numpy(obb.xywhr)
            scores = _to_numpy(obb.conf)
            classes = _to_numpy(obb.cls).astype(int)
            for index in range(len(obb)):
                class_id = int(classes[index])
                centre_x, centre_y, width, height, angle = [
                    float(value) for value in xywhr[index]
                ]
                cosine = abs(math.cos(angle))
                sine = abs(math.sin(angle))
                box_width = width * cosine + height * sine
                box_height = width * sine + height * cosine
                detections.append(Detection(
                    label=self._label(class_id, names),
                    score=float(scores[index]),
                    bbox_xyxy=(
                        centre_x - box_width / 2.0,
                        centre_y - box_height / 2.0,
                        centre_x + box_width / 2.0,
                        centre_y + box_height / 2.0,
                    ),
                    class_id=class_id,
                ))
        return detections


class HotReloadingDetector:
    """Reload a detector when its model file changes on disk."""

    def __init__(self, config, logger=None):
        """Build the initial detector."""
        self.config = dict(config)
        self.logger = logger
        self.backend = str(config.get('backend', 'ultralytics'))
        self.hot_reload = bool(config.get('hot_reload', True))
        self._detector = None
        self._mtime_ns = 0
        self._last_error = ''
        self._load()

    def _factory(self):
        """Build a detector from the configuration."""
        backend = self.backend
        if backend == 'mock':
            return MockColorDetector(self.config)
        if backend == 'ultralytics':
            return UltralyticsDetector(self.config)
        raise DetectorError('unknown backend: {}'.format(backend))

    def _load(self):
        """Load or reload, keeping the previous detector on failure."""
        detector = self._factory()
        self._detector = detector
        self._mtime_ns = self._model_mtime_ns()
        self._last_error = ''
        if self.logger is not None:
            self.logger.info(
                'detector loaded: {}'.format(detector.describe()))

    def _model_mtime_ns(self):
        """Return the current model mtime for change detection."""
        path = str(self.config.get('path', '')).strip()
        if not path:
            return 0
        try:
            if os.path.isdir(path):
                stamps = [
                    os.path.getmtime(os.path.join(path, name))
                    for name in os.listdir(path)
                    if os.path.isfile(os.path.join(path, name))
                ]
                return int(max(stamps) * 1e9) if stamps else 0
            return int(os.path.getmtime(path) * 1e9)
        except OSError:
            return 0

    def detect(self, image_bgr):
        """Run detection, reloading first if the model changed."""
        if self.hot_reload and self.backend != 'mock':
            mtime = self._model_mtime_ns()
            if mtime and mtime != self._mtime_ns:
                try:
                    self._load()
                except Exception as error:  # noqa: BLE001 - keep serving
                    self._last_error = str(error)
                    if self.logger is not None:
                        self.logger.warn(
                            'model reload failed, keeping previous: {}'.format(
                                error))
        if self._detector is None:
            raise DetectorError(
                self._last_error or 'detector is not loaded')
        return self._detector.detect(image_bgr)

    def describe(self):
        """Return the active model identity."""
        if self._detector is None:
            return 'unloaded'
        return self._detector.describe()


def build_detector(config, logger=None):
    """Build a hot-reloading detector from a GPU camera entry."""
    return HotReloadingDetector(config, logger=logger)
