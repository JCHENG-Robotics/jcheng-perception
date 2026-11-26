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

import base64
import json
import math
import os
import urllib.request

import cv2
import numpy as np
import yaml

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


class ArucoTagDetector(BaseDetector):
    """Detect an ArUco/AprilTag marker and return its centre as a keypoint.

    This is the **defined reference point** mode. The marker centre is the same
    physical point in every view, so two-view fusion no longer sees the
    object-dependent bbox-centre offset that produced the 57-67 mm cup
    difference and the first-pair reference step. The centre keypoint is the
    primary keypoint; the four marker corners are included with a lower
    confidence for future pose/keypoint use.

    The 3-D point still comes from the edge's keypoint depth path (or the bbox
    fallback), not from RGB-only PnP: the edge would need the marker size and
    camera matrix, and ``scripts/fixed_point_check.py`` already measures the
    depth-vs-PnP residual for absolute checks.
    """

    def __init__(self, config):
        """Parse the dictionary, marker filter and label."""
        if not hasattr(cv2, 'aruco'):
            raise DetectorError(
                'cv2.aruco is missing; install opencv-contrib-python')
        self.dictionary_name = str(
            config.get('dictionary', 'DICT_APRILTAG_36h11'))
        dictionary_id = getattr(cv2.aruco, self.dictionary_name, None)
        if dictionary_id is None:
            raise DetectorError(
                'unknown ArUco dictionary: {}'.format(self.dictionary_name))
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        # OpenCV 4.6's Python binding segfaults when assigning
        # ``cornerRefinementMethod`` on a DetectorParameters object (verified
        # in this environment). The default detector parameters find the tag
        # centre to ~0.5 px, which is enough for the reference-point mode; do
        # not add the assignment without testing it on the target OpenCV build.
        self.marker_ids = {
            int(value) for value in (config.get('marker_ids') or [])
        }
        self.label = str(config.get('label', 'reference_tag'))
        self.include_id = bool(config.get('include_id', True))
        self.marker_size_m = float(config.get('marker_size_m', 0.06))
        self.confidence = float(config.get('confidence', 1.0))
        self.corner_confidence = float(
            config.get('corner_confidence', 0.9))

    def describe(self):
        """Return the dictionary, marker filter and physical size."""
        marker_text = ','.join(
            str(value) for value in sorted(self.marker_ids)) or 'all'
        return 'aruco:{}:{}:{:.3f}m'.format(
            self.dictionary_name, marker_text, self.marker_size_m)

    def detect(self, image_bgr):
        """Return one detection per matched marker, centre keypoint first."""
        gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(self.dictionary)
            corners, identifiers, _ = detector.detectMarkers(gray)
        else:
            corners, identifiers, _ = cv2.aruco.detectMarkers(
                gray, self.dictionary)
        if identifiers is None:
            return []
        detections = []
        for marker_corners, marker_id in zip(
                corners, identifiers.reshape(-1)):
            marker_id = int(marker_id)
            if self.marker_ids and marker_id not in self.marker_ids:
                continue
            points = np.asarray(
                marker_corners, dtype=np.float64).reshape(4, 2)
            centre = points.mean(axis=0)
            label = self.label if not self.include_id \
                else '{}_{}'.format(self.label, marker_id)
            keypoints = [
                Keypoint(float(centre[0]), float(centre[1]),
                         self.confidence)
            ]
            keypoints.extend(
                Keypoint(float(point[0]), float(point[1]),
                         self.corner_confidence)
                for point in points)
            detections.append(Detection(
                label=label,
                score=float(self.confidence),
                bbox_xyxy=(
                    float(points[:, 0].min()), float(points[:, 1].min()),
                    float(points[:, 0].max()), float(points[:, 1].max()),
                ),
                class_id=marker_id,
                keypoints=keypoints,
            ))
        return detections


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
        arguments = {
            'imgsz': self.imgsz,
            'conf': self.conf,
            'iou': self.iou,
            'device': self.device,
            'verbose': False,
            'max_det': self.max_detections,
        }
        if self.half:
            # Passing half=False still triggers the deprecation warning in
            # current Ultralytics; omit the key entirely when unused.
            arguments['half'] = True
        try:
            results = self.model.predict(image_bgr, **arguments)
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


def _decode_yolo_v8(raw, conf, iou, labels, imgsz, width, height,
                    max_detections):
    """Decode a standard YOLOv8 detect output into original-image boxes."""
    values = np.asarray(raw)
    if values.ndim == 3 and values.shape[0] == 1:
        values = values[0]
    if values.ndim != 2:
        raise DetectorError(
            'unexpected YOLO output shape {}'.format(values.shape))
    if values.shape[0] < values.shape[1]:
        values = values.T
    if values.shape[1] < 5:
        raise DetectorError(
            'unexpected YOLO output channels {}'.format(values.shape))
    boxes = []
    scores = []
    class_ids = []
    for row in values:
        class_scores = row[4:]
        if class_scores.size == 0:
            continue
        class_id = int(np.argmax(class_scores))
        score = float(class_scores[class_id])
        if score < conf:
            continue
        centre_x, centre_y, box_width, box_height = [
            float(value) for value in row[:4]
        ]
        boxes.append([
            centre_x - box_width / 2.0,
            centre_y - box_height / 2.0,
            box_width,
            box_height,
        ])
        scores.append(score)
        class_ids.append(class_id)
    if not boxes:
        return []
    indices = cv2.dnn.NMSBoxes(boxes, scores, conf, iou)
    indices = np.asarray(indices).reshape(-1).tolist()
    scale = float(max(width, height)) / float(imgsz)
    detections = []
    for index in indices[:max_detections]:
        x, y, box_width, box_height = boxes[index]
        x0 = max(0.0, min(float(width - 1), x * scale))
        y0 = max(0.0, min(float(height - 1), y * scale))
        x1 = max(0.0, min(float(width - 1), (x + box_width) * scale))
        y1 = max(0.0, min(float(height - 1), (y + box_height) * scale))
        if x1 <= x0 or y1 <= y0:
            continue
        class_id = class_ids[index]
        detections.append(Detection(
            label=labels.get(class_id, str(class_id)),
            score=float(scores[index]),
            bbox_xyxy=(x0, y0, x1, y1),
            class_id=class_id,
        ))
    return detections


class OpenVinoDetector(BaseDetector):
    """OpenVINO IR detector for standard YOLOv8-style detect models.

    This backend is for Intel CPU/GPU hosts and for the vendor's pre-exported
    ``*_openvino_model`` directories. It deliberately supports detect models
    only (not pose/OBB); use Ultralytics for keypoints or oriented boxes.

    The pre/post-processing mirrors the vendor ArmPi YOLOv8 example: the image
    is placed top-left on a square canvas, scaled to the model input, and boxes
    are mapped back by ``max(width, height) / imgsz``. That keeps results
    comparable with the vendor deployment.
    """

    def __init__(self, config):
        """Load the IR model and metadata."""
        path = str(config.get('path', '')).strip()
        if not path:
            raise DetectorError('model.path is empty')
        if not os.path.exists(path):
            raise DetectorError('model path does not exist: {}'.format(path))
        self.model_path = self._resolve_model_path(path)
        self.metadata = self._load_metadata(self.model_path)
        task = str(config.get('task') or self.metadata.get('task', 'detect'))
        if task not in ('detect', 'detection'):
            raise DetectorError(
                'openvino backend supports detect models only, got task={}; '
                'use backend: ultralytics for pose/obb'.format(task))
        try:
            import openvino as ov
        except ImportError as error:
            raise DetectorError(
                'openvino is not installed; install the openvino wheel or '
                'use backend: mock/ultralytics') from error
        self.core = ov.Core()
        self.device = self._openvino_device(config.get('device', 'CPU'))
        try:
            self.model = self.core.read_model(self.model_path)
            self.compiled = self.core.compile_model(self.model, self.device)
        except Exception as error:  # noqa: BLE001 - runtime dependent
            raise DetectorError(
                'failed to compile {}: {}'.format(
                    self.model_path, error)) from error
        self.input = self.compiled.input(0)
        self.output = self.compiled.output(0)
        shape = list(self.input.shape)
        self.imgsz = int(shape[2]) if len(shape) == 4 and isinstance(
            shape[2], int) else int(config.get('imgsz', 640))
        self.conf = float(config.get('conf', 0.35))
        self.iou = float(config.get('iou', 0.45))
        self.max_detections = int(config.get('max_detections', 20))
        names = config.get('labels') or self.metadata.get('names') or {}
        if isinstance(names, dict):
            self.labels = {
                int(key): str(value) for key, value in names.items()
            }
        else:
            self.labels = {
                index: str(value) for index, value in enumerate(names)
            }
        self.model_id = '{}:{}'.format(
            os.path.basename(self.model_path),
            int(os.path.getmtime(self.model_path) * 1e9))

    @staticmethod
    def _resolve_model_path(path):
        """Return an .xml/.onnx file, or the first XML in a model directory."""
        if os.path.isdir(path):
            candidates = sorted(
                os.path.join(path, name)
                for name in os.listdir(path)
                if name.endswith('.xml')
            )
            if not candidates:
                raise DetectorError(
                    'no .xml model inside {}'.format(path))
            return candidates[0]
        if not path.endswith(('.xml', '.onnx')):
            raise DetectorError(
                'openvino expects .xml/.onnx or a model directory: {}'.format(
                    path))
        return path

    @staticmethod
    def _load_metadata(model_path):
        """Read Ultralytics metadata next to the IR, when present."""
        directory = os.path.dirname(model_path)
        for name in ('metadata.yaml', 'metadata.yml'):
            candidate = os.path.join(directory, name)
            if not os.path.isfile(candidate):
                continue
            try:
                with open(candidate, 'r', encoding='utf-8') as handle:
                    return yaml.safe_load(handle) or {}
            except (OSError, yaml.YAMLError):
                return {}
        return {}

    @staticmethod
    def _openvino_device(value):
        """Map Ultralytics-style device strings to OpenVINO devices."""
        text = str(value or 'CPU').strip()
        if text in ('0', 'cuda', 'cuda:0', 'gpu', 'GPU.0'):
            return 'CPU'
        return text or 'CPU'

    def describe(self):
        """Return the IR model identity including its mtime."""
        return self.model_id

    def detect(self, image_bgr):
        """Run OpenVINO inference and return boxes in original coordinates."""
        height, width = image_bgr.shape[:2]
        length = max(height, width)
        canvas = np.zeros((length, length, 3), dtype=np.uint8)
        canvas[:height, :width] = image_bgr
        blob = cv2.dnn.blobFromImage(
            canvas, scalefactor=1.0 / 255.0,
            size=(self.imgsz, self.imgsz), swapRB=True)
        try:
            raw = self.compiled([blob])[self.output]
        except Exception as error:  # noqa: BLE001 - runtime dependent
            raise DetectorError(
                'openvino inference failed: {}'.format(error)) from error
        return _decode_yolo_v8(
            raw, self.conf, self.iou, self.labels, self.imgsz,
            width, height, self.max_detections)


class OpenCvDnnDetector(BaseDetector):
    """YOLOv8 ONNX detector using OpenCV DNN.

    This is the no-extra-runtime fallback for CPU-only hosts and the vendor
    ``yolov8s.onnx`` / ``best_traffic.onnx`` exports. It supports detect models
    only; use Ultralytics for pose/OBB. OpenCV DNN ignores ``device``.
    """

    def __init__(self, config):
        """Load the ONNX graph."""
        path = str(config.get('path', '')).strip()
        if not path:
            raise DetectorError('model.path is empty')
        if not os.path.exists(path):
            raise DetectorError('model path does not exist: {}'.format(path))
        if not path.endswith('.onnx'):
            raise DetectorError(
                'opencv backend expects an .onnx model: {}'.format(path))
        task = str(config.get('task', 'detect'))
        if task not in ('detect', 'detection'):
            raise DetectorError(
                'opencv backend supports detect models only, got task={}; '
                'use backend: ultralytics for pose/obb'.format(task))
        try:
            self.net = cv2.dnn.readNetFromONNX(path)
        except cv2.error as error:
            raise DetectorError(
                'failed to load {}: {}'.format(path, error)) from error
        self.path = path
        self.imgsz = int(config.get('imgsz', 640))
        self.conf = float(config.get('conf', 0.35))
        self.iou = float(config.get('iou', 0.45))
        self.max_detections = int(config.get('max_detections', 20))
        names = config.get('labels') or []
        if isinstance(names, dict):
            self.labels = {
                int(key): str(value) for key, value in names.items()
            }
        else:
            self.labels = {
                index: str(value) for index, value in enumerate(names)
            }
        self.model_id = '{}:{}'.format(
            os.path.basename(path), int(os.path.getmtime(path) * 1e9))

    def describe(self):
        """Return the ONNX model identity including its mtime."""
        return self.model_id

    def detect(self, image_bgr):
        """Run OpenCV DNN inference and return original-image boxes."""
        height, width = image_bgr.shape[:2]
        length = max(height, width)
        canvas = np.zeros((length, length, 3), dtype=np.uint8)
        canvas[:height, :width] = image_bgr
        blob = cv2.dnn.blobFromImage(
            canvas, scalefactor=1.0 / 255.0,
            size=(self.imgsz, self.imgsz), swapRB=True)
        self.net.setInput(blob)
        try:
            raw = self.net.forward()
        except cv2.error as error:
            raise DetectorError(
                'opencv inference failed: {}'.format(error)) from error
        return _decode_yolo_v8(
            raw, self.conf, self.iou, self.labels, self.imgsz,
            width, height, self.max_detections)


class OnnxRuntimeDetector(BaseDetector):
    """YOLOv8 ONNX detector using ONNX Runtime (CPU by default).

    ONNX Runtime parses the YOLOv8 DFL graph correctly, unlike OpenCV 4.6 DNN.
    It is a compact fallback for hosts without Ultralytics/torch. GPU providers
    can be selected with ``providers`` once onnxruntime-gpu is installed.
    """

    def __init__(self, config):
        """Create the ONNX Runtime session."""
        path = str(config.get('path', '')).strip()
        if not path:
            raise DetectorError('model.path is empty')
        if not os.path.exists(path):
            raise DetectorError('model path does not exist: {}'.format(path))
        if not path.endswith('.onnx'):
            raise DetectorError(
                'onnxruntime backend expects an .onnx model: {}'.format(path))
        task = str(config.get('task', 'detect'))
        if task not in ('detect', 'detection'):
            raise DetectorError(
                'onnxruntime backend supports detect models only, got '
                'task={}; use backend: ultralytics for pose/obb'.format(task))
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise DetectorError(
                'onnxruntime is not installed; install the onnxruntime wheel '
                'or use backend: mock/ultralytics') from error
        providers = config.get('providers') or ['CPUExecutionProvider']
        try:
            self.session = ort.InferenceSession(
                path, providers=list(providers))
        except Exception as error:  # noqa: BLE001 - runtime dependent
            raise DetectorError(
                'failed to load {}: {}'.format(path, error)) from error
        self.path = path
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name
        input_shape = self.session.get_inputs()[0].shape
        self.imgsz = int(config.get('imgsz', 640))
        if len(input_shape) == 4 and isinstance(input_shape[2], int):
            self.imgsz = int(input_shape[2])
        self.conf = float(config.get('conf', 0.35))
        self.iou = float(config.get('iou', 0.45))
        self.max_detections = int(config.get('max_detections', 20))
        names = config.get('labels') or []
        if isinstance(names, dict):
            self.labels = {
                int(key): str(value) for key, value in names.items()
            }
        else:
            self.labels = {
                index: str(value) for index, value in enumerate(names)
            }
        self.model_id = '{}:{}'.format(
            os.path.basename(path), int(os.path.getmtime(path) * 1e9))

    def describe(self):
        """Return the ONNX model identity including its mtime."""
        return self.model_id

    def detect(self, image_bgr):
        """Run ONNX Runtime inference and return original-image boxes."""
        height, width = image_bgr.shape[:2]
        length = max(height, width)
        canvas = np.zeros((length, length, 3), dtype=np.uint8)
        canvas[:height, :width] = image_bgr
        blob = cv2.dnn.blobFromImage(
            canvas, scalefactor=1.0 / 255.0,
            size=(self.imgsz, self.imgsz), swapRB=True)
        try:
            raw = self.session.run(
                [self.output_name], {self.input_name: blob})[0]
        except Exception as error:  # noqa: BLE001 - runtime dependent
            raise DetectorError(
                'onnxruntime inference failed: {}'.format(error)) from error
        return _decode_yolo_v8(
            raw, self.conf, self.iou, self.labels, self.imgsz,
            width, height, self.max_detections)


class AdvisorDetector(BaseDetector):
    """SAM3 + Qwen advisory service detector over HTTP.

    The remote service (AGX Orin) returns the best mask after the fixed
    ``T1 + SAM3 score >= 0.8`` policy. Only a ``JUDGE`` reply becomes a
    detection; ``SKIP`` and ``NO_MASK`` are successful empty results. Any
    transport, protocol or malformed-reply error raises ``DetectorError`` so the
    camera worker counts a failed frame and never publishes a fabricated box.

    The mask itself is not exposed: ``Detection2D`` carries boxes only. Adding a
    mask field is a contract change for ``jcheng-interfaces``.
    """

    def __init__(self, config, transport=None):
        """Validate the endpoint and prompt; keep the transport injectable."""
        endpoint = str(config.get('endpoint', '')).strip()
        if not endpoint:
            raise DetectorError('advisor.endpoint is empty')
        if not endpoint.startswith(('http://', 'https://')):
            raise DetectorError(
                'advisor.endpoint must be http(s): {}'.format(endpoint))
        self.endpoint = endpoint
        self.prompt = str(config.get('prompt', '')).strip()
        if not self.prompt:
            raise DetectorError('advisor.prompt is empty')
        self.timeout_s = float(config.get('timeout_s', 10.0))
        if self.timeout_s <= 0:
            raise DetectorError('advisor.timeout_s must be positive')
        self.jpeg_quality = int(config.get('jpeg_quality', 90))
        if not 1 <= self.jpeg_quality <= 100:
            raise DetectorError('advisor.jpeg_quality must be 1..100')
        self.min_score = float(config.get('min_score', 0.0))
        self._transport = transport or self._post
        self.model_id = 'sam3_advisor:{}:{}'.format(endpoint, self.prompt)

    def describe(self):
        """Return a stable identity for logs and status messages."""
        return self.model_id

    @staticmethod
    def _post(endpoint, payload, timeout_s):
        """POST JSON and return the decoded reply."""
        request = urllib.request.Request(
            endpoint,
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return json.load(response)

    def detect(self, image_bgr):
        """Return at most one judged detection for the configured prompt."""
        encoded_ok, encoded = cv2.imencode(
            '.jpg', image_bgr,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not encoded_ok:
            raise DetectorError('advisor failed to encode the frame as JPEG')
        payload = {
            'image_b64': base64.b64encode(encoded.tobytes()).decode('ascii'),
            'image_format': 'jpg',
            'prompt': self.prompt,
        }
        try:
            reply = self._transport(self.endpoint, payload, self.timeout_s)
        except Exception as error:  # noqa: BLE001 - transport dependent
            raise DetectorError(
                'advisor request failed: {}'.format(error)) from error
        if not isinstance(reply, dict) or reply.get('ok') is not True:
            raise DetectorError(
                'advisor returned an error reply: {!r}'.format(reply))
        gate = str(reply.get('gate', '')).upper()
        if gate in ('SKIP', 'NO_MASK'):
            return []
        if gate != 'JUDGE':
            raise DetectorError(
                'advisor returned an unknown gate: {!r}'.format(gate))
        try:
            score = float(reply['score'])
            x0, y0, x1, y1 = (float(value) for value in reply['bbox'])
        except (KeyError, TypeError, ValueError) as error:
            raise DetectorError(
                'advisor JUDGE reply has no valid score/bbox: {!r}'.format(
                    reply)) from error
        if not all(math.isfinite(value) for value in (score, x0, y0, x1, y1)):
            raise DetectorError('advisor returned non-finite score/bbox')
        if x1 < x0 or y1 < y0:
            raise DetectorError('advisor returned an inverted bbox')
        if score < self.min_score:
            return []
        return [Detection(
            label='sam3:{}'.format(self.prompt),
            score=score,
            bbox_xyxy=(x0, y0, x1, y1))]


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
        if backend in ('aruco', 'apriltag'):
            return ArucoTagDetector(self.config)
        if backend == 'ultralytics':
            return UltralyticsDetector(self.config)
        if backend == 'openvino':
            return OpenVinoDetector(self.config)
        if backend == 'opencv':
            return OpenCvDnnDetector(self.config)
        if backend == 'onnxruntime':
            return OnnxRuntimeDetector(self.config)
        if backend == 'sam3_advisor':
            return AdvisorDetector(self.config)
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
