"""Detector adapter tests that do not require Ultralytics or a GPU."""

import base64

import cv2
import numpy as np
import pytest

from jcheng_perception_gpu.detectors import (
    AdvisorDetector,
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


def _advisor_config(**overrides):
    config = {
        'backend': 'sam3_advisor',
        'hot_reload': False,
        'endpoint': 'http://127.0.0.1:8090/infer',
        'prompt': 'bottle',
        'timeout_s': 3.0,
    }
    config.update(overrides)
    return config


def test_advisor_judge_returns_one_detection():
    seen = {}

    def transport(endpoint, payload, timeout_s):
        seen['endpoint'] = endpoint
        seen['payload'] = payload
        seen['timeout_s'] = timeout_s
        return {
            'ok': True,
            'gate': 'JUDGE',
            'score': 0.93,
            'bbox': [10, 20, 110, 220],
            'judge': {'answer': 'yes', 'p_yes': 0.99},
        }

    detector = AdvisorDetector(_advisor_config(), transport=transport)
    detections = detector.detect(_red_square())

    assert seen['endpoint'] == 'http://127.0.0.1:8090/infer'
    assert seen['timeout_s'] == 3.0
    assert seen['payload']['prompt'] == 'bottle'
    assert seen['payload']['image_format'] == 'jpg'
    encoded = base64.b64decode(seen['payload']['image_b64'])
    assert encoded[:2] == b'\xff\xd8'
    assert base64.b64encode(encoded).decode('ascii') == \
        seen['payload']['image_b64']
    assert len(detections) == 1
    assert detections[0].label == 'sam3:bottle'
    assert detections[0].score == pytest.approx(0.93)
    assert detections[0].bbox_xyxy == (10.0, 20.0, 110.0, 220.0)
    assert detector.describe().startswith('sam3_advisor:')


@pytest.mark.parametrize('gate', ['SKIP', 'NO_MASK'])
def test_advisor_non_judge_gate_is_an_empty_result(gate):
    detector = AdvisorDetector(
        _advisor_config(),
        transport=lambda endpoint, payload, timeout_s: {
            'ok': True, 'gate': gate, 'score': 0.4})
    assert detector.detect(_red_square()) == []


@pytest.mark.parametrize('reply', [
    {'ok': False, 'error': 'boom'},
    {'ok': True, 'gate': 'UNKNOWN'},
    {'ok': True, 'gate': 'JUDGE', 'score': 0.9},
    {'ok': True, 'gate': 'JUDGE', 'score': 0.9, 'bbox': [1, 2, 3]},
    {'ok': True, 'gate': 'JUDGE', 'score': 0.9, 'bbox': [9, 2, 3, 4]},
    {'ok': True, 'gate': 'JUDGE', 'score': 0.9, 'bbox': [1, 2, 3, 4, 5]},
    {'ok': True, 'gate': 'JUDGE', 'score': float('nan'),
     'bbox': [1, 2, 3, 4]},
])
def test_advisor_malformed_reply_raises(reply):
    detector = AdvisorDetector(
        _advisor_config(),
        transport=lambda endpoint, payload, timeout_s: reply)
    with pytest.raises(DetectorError):
        detector.detect(_red_square())


def test_advisor_transport_error_raises_detector_error():
    def transport(endpoint, payload, timeout_s):
        raise TimeoutError('service did not answer')

    detector = AdvisorDetector(_advisor_config(), transport=transport)
    with pytest.raises(DetectorError) as error:
        detector.detect(_red_square())
    assert 'advisor request failed' in str(error.value)


def test_advisor_min_score_can_drop_a_judged_box():
    detector = AdvisorDetector(
        _advisor_config(min_score=0.95),
        transport=lambda endpoint, payload, timeout_s: {
            'ok': True, 'gate': 'JUDGE', 'score': 0.9,
            'bbox': [1, 2, 3, 4]})
    assert detector.detect(_red_square()) == []


def test_advisor_requires_endpoint_and_prompt():
    with pytest.raises(DetectorError):
        AdvisorDetector(_advisor_config(endpoint=''))
    with pytest.raises(DetectorError):
        AdvisorDetector(_advisor_config(prompt=''))
    with pytest.raises(DetectorError):
        AdvisorDetector(_advisor_config(endpoint='ftp://example/x'))
    with pytest.raises(DetectorError):
        AdvisorDetector(_advisor_config(timeout_s=0))


def test_build_advisor_detector_from_config():
    detector = build_detector(_advisor_config())
    assert detector.describe() == (
        'sam3_advisor:http://127.0.0.1:8090/infer:bottle')
