"""Depth sampling, uncertainty and back-projection tests."""

import dataclasses
import math

import numpy as np

from helpers import K, depth_array, load_fusion, load_specs, make_detection
from jcheng_perception_core import (
    DepthFrame,
    FrameMeta,
    Keypoint,
    build_observation,
    depth_sigma_m,
    depth_valid_fraction,
    lateral_sigma_m,
    sample_bbox_depth,
    sample_depth,
)


def _frame(width=640, height=400):
    return FrameMeta(
        camera_id='d435i_ext',
        seq=1,
        capture_ns=1_000_000_000,
        received_ns=1_001_000_000,
        width=width,
        height=height,
        frame_id='camera',
        camera_matrix=K,
        distortion=np.zeros(5),
    )


def _depth(data, scale_m=0.001):
    return DepthFrame(data=np.asarray(data), scale_m=scale_m)


def test_sample_depth_median_and_hole():
    data = np.full((20, 20), 1000, dtype=np.uint16)
    data[9:11, 9:11] = 0
    frame = _depth(data)
    sample = sample_depth(frame, 9.5, 9.5, radius=2, min_valid=1)
    assert sample is not None
    depth_m, fraction = sample
    assert abs(depth_m - 1.0) < 1e-9
    assert 0.0 < fraction < 1.0


def test_sample_bbox_depth_grid_fallback():
    data = np.full((100, 100), 1200, dtype=np.uint16)
    data[47:53, 47:53] = 0
    frame = _depth(data)
    sample = sample_bbox_depth(
        frame, (20.0, 20.0, 80.0, 80.0), radius=2, min_valid=1)
    assert sample is not None
    assert sample['source'] == 'grid'
    assert abs(sample['depth_m'] - 1.2) < 1e-9


def test_depth_noise_scales_quadratically():
    assert abs(depth_sigma_m(0.0, 0.0025, 0.0015) - 0.0015) < 1e-12
    expected = math.sqrt(0.0025 ** 2 + 0.0015 ** 2)
    assert abs(depth_sigma_m(1.0, 0.0025, 0.0015) - expected) < 1e-12
    assert lateral_sigma_m(1.0, 500.0, 1.5) == 0.003


def test_depth_valid_fraction():
    data = np.full((40, 40), 1000, dtype=np.uint16)
    data[:, :20] = 0
    frame = _depth(data)
    fraction = depth_valid_fraction(frame, (0.0, 0.0, 40.0, 40.0), stride=1)
    assert abs(fraction - 0.5) < 1e-9


def test_backproject_identity_transform():
    specs = load_specs()
    fusion = load_fusion()
    observation = build_observation(
        make_detection(), _frame(), _depth(depth_array(1.0)),
        np.eye(4), specs['d435i_ext'], fusion)
    assert observation is not None
    assert np.allclose(observation.position_base, [0.0, 0.0, 1.0],
                       atol=1e-6)
    assert observation.depth_m == 1.0
    assert observation.valid_depth_fraction == 1.0
    assert observation.occlusion_ratio == 0.0
    assert observation.orientation_base is None
    assert np.all(np.diag(observation.covariance_base) > 0.0)


def test_backproject_applies_base_transform():
    specs = load_specs()
    fusion = load_fusion()
    transform = np.eye(4)
    transform[:3, 3] = [0.1, 0.2, 0.3]
    observation = build_observation(
        make_detection(), _frame(), _depth(depth_array(1.0)),
        transform, specs['d435i_ext'], fusion)
    assert observation is not None
    assert np.allclose(observation.position_base, [0.1, 0.2, 1.3],
                       atol=1e-6)


def test_backproject_prefers_offset_keypoint_depth():
    specs = load_specs()
    fusion = load_fusion()
    detection = make_detection(
        keypoints=[Keypoint(400.0, 200.0, 0.95)])
    observation = build_observation(
        detection, _frame(), _depth(depth_array(1.0)),
        np.eye(4), specs['d435i_ext'], fusion)
    assert observation is not None
    assert abs(observation.position_base[0] - 0.16) < 1e-6
    assert observation.keypoint_score == 0.95


def test_backproject_keypoint_axis_yaw():
    specs = load_specs()
    spec = dataclasses.replace(
        specs['aurora_wrist'], keypoint_axis_pairs=[[0, 1]])
    fusion = load_fusion()
    detection = make_detection(keypoints=[
        Keypoint(300.0, 200.0, 0.9),
        Keypoint(400.0, 200.0, 0.9),
    ])
    observation = build_observation(
        detection, _frame(), _depth(depth_array(1.0)),
        np.eye(4), spec, fusion)
    assert observation is not None
    assert observation.orientation_base is not None
    from jcheng_perception_core import quaternion_yaw

    assert abs(quaternion_yaw(observation.orientation_base)) < 1e-6


def test_backproject_missing_depth_or_size_mismatch():
    specs = load_specs()
    fusion = load_fusion()
    empty = np.zeros((400, 640), dtype=np.uint16)
    assert build_observation(
        make_detection(), _frame(), _depth(empty),
        np.eye(4), specs['d435i_ext'], fusion) is None
    small = np.full((240, 320), 1000, dtype=np.uint16)
    assert build_observation(
        make_detection(), _frame(), _depth(small),
        np.eye(4), specs['d435i_ext'], fusion) is None


def test_backproject_marks_thin_depth_as_occluded():
    specs = load_specs()
    fusion = load_fusion()
    data = np.zeros((400, 640), dtype=np.uint16)
    data[197:203, 317:323] = 1000
    observation = build_observation(
        make_detection(keypoints=[]), _frame(), _depth(data),
        np.eye(4), specs['d435i_ext'], fusion)
    assert observation is not None
    assert observation.occlusion_ratio > 0.5
    assert observation.is_occluded
