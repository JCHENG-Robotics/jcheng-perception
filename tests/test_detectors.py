"""Detector adapter tests that do not require Ultralytics or a GPU."""

import cv2
import numpy as np
import pytest

from jcheng_perception_gpu.detectors import (
    DetectorError,
    MockColorDetector,
    build_detector,
)


def _red_square():
    image = np.full((240, 320, 3), 50, dtype=np.uint8)
    image[90:150, 120:200] = (0, 0, 235)
    return image


def _tag_image(marker_id=0, size=200, border=40):
    """Return a BGR image with an AprilTag and its expected centre pixel."""
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        marker = cv2.aruco.generateImageMarker(
            dictionary, marker_id, size, borderBits=1)
    else:
        marker = cv2.aruco.drawMarker(dictionary, marker_id, size, 1)
    image = np.full((size + 2 * border, size + 2 * border), 255, np.uint8)
    image[border:border + size, border:border + size] = marker
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), border + size / 2.0


def test_mock_detector_finds_red_square():
    detector = MockColorDetector({'label': 'block', 'min_area': 50})
    detections = detector.detect(_red_square())
    assert len(detections) == 1
    detection = detections[0]
    assert detection.label == 'block'
    assert detection.score == 1.0
    x0, y0, x1, y1 = detection.bbox_xyxy
    assert 118 <= x0 <= 122
    assert 88 <= y0 <= 92
    assert 198 <= x1 <= 202
    assert 148 <= y1 <= 152
    assert detection.keypoints[0].x == pytest.approx(160.0, abs=0.6)
    assert detection.keypoints[0].y == pytest.approx(120.0, abs=0.6)


def test_mock_detector_returns_empty_for_grey_image():
    detector = MockColorDetector({})
    assert detector.detect(np.full((100, 100, 3), 60, dtype=np.uint8)) == []


def test_build_mock_detector():
    detector = build_detector({
        'backend': 'mock',
        'hot_reload': False,
        'label': 'target',
    })
    assert detector.describe() == 'mock-color-red'
    assert len(detector.detect(_red_square())) == 1


def test_unknown_backend_raises():
    with pytest.raises(DetectorError):
        build_detector({'backend': 'not-a-backend', 'hot_reload': False})


def test_ultralytics_missing_path_raises_without_import():
    with pytest.raises(DetectorError) as error:
        build_detector({
            'backend': 'ultralytics',
            'path': '/no/such/model.pt',
            'hot_reload': False,
        })
    assert 'does not exist' in str(error.value)


def test_openvino_missing_path_raises_without_import():
    with pytest.raises(DetectorError) as error:
        build_detector({
            'backend': 'openvino',
            'path': '/no/such/model.xml',
            'hot_reload': False,
        })
    assert 'does not exist' in str(error.value)


def test_opencv_missing_path_raises_without_model_load():
    with pytest.raises(DetectorError) as error:
        build_detector({
            'backend': 'opencv',
            'path': '/no/such/model.onnx',
            'hot_reload': False,
        })
    assert 'does not exist' in str(error.value)


def test_onnxruntime_missing_path_raises_without_import():
    with pytest.raises(DetectorError) as error:
        build_detector({
            'backend': 'onnxruntime',
            'path': '/no/such/model.onnx',
            'hot_reload': False,
        })
    assert 'does not exist' in str(error.value)


def test_aruco_detector_returns_centre_keypoint_and_bbox():
    image, expected = _tag_image(marker_id=0)
    detector = build_detector({
        'backend': 'aruco',
        'hot_reload': False,
        'dictionary': 'DICT_APRILTAG_36h11',
        'marker_size_m': 0.06,
        'marker_ids': [0],
        'label': 'reference_tag',
    })
    detections = detector.detect(image)
    assert len(detections) == 1
    detection = detections[0]
    assert detection.label == 'reference_tag_0'
    assert detection.class_id == 0
    assert detection.score == 1.0
    assert len(detection.keypoints) == 5
    # The centre keypoint is first and has the highest confidence: the edge's
    # keypoint path uses it as the defined 3-D point.
    assert detection.keypoints[0].confidence == 1.0
    assert detection.keypoints[0].x == pytest.approx(expected, abs=2.0)
    assert detection.keypoints[0].y == pytest.approx(expected, abs=2.0)
    assert all(keypoint.confidence == 0.9
               for keypoint in detection.keypoints[1:])
    x0, y0, x1, y1 = detection.bbox_xyxy
    assert 35.0 <= x0 <= 45.0
    assert 35.0 <= y0 <= 45.0
    assert 235.0 <= x1 <= 245.0
    assert 235.0 <= y1 <= 245.0


def test_aruco_detector_filters_marker_ids_and_labels():
    image, _ = _tag_image(marker_id=0)
    filtered = build_detector({
        'backend': 'aruco',
        'hot_reload': False,
        'marker_ids': [1],
    })
    assert filtered.detect(image) == []
    no_id = build_detector({
        'backend': 'aruco',
        'hot_reload': False,
        'include_id': False,
    })
    detections = no_id.detect(image)
    assert len(detections) == 1
    assert detections[0].label == 'reference_tag'


def test_aruco_detector_returns_empty_without_a_marker():
    detector = build_detector({
        'backend': 'aruco',
        'hot_reload': False,
    })
    assert detector.detect(
        np.full((120, 160, 3), 127, dtype=np.uint8)) == []


def test_aruco_unknown_dictionary_raises():
    with pytest.raises(DetectorError) as error:
        build_detector({
            'backend': 'aruco',
            'hot_reload': False,
            'dictionary': 'DICT_NOT_A_DICTIONARY',
        })
    assert 'unknown ArUco dictionary' in str(error.value)
