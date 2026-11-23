"""Detector adapter tests that do not require Ultralytics or a GPU."""

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
