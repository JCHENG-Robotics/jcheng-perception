"""Shared builders for the core tests."""

import os
from pathlib import Path

import numpy as np

from jcheng_perception_core import (
    Detection,
    Keypoint,
    TargetObservation,
    load_camera_specs,
    load_fusion_config,
    load_static_frames,
)
from jcheng_perception_core.types import Role

REPO_ROOT = Path(__file__).resolve().parents[1]
WIDTH = 640
HEIGHT = 400
K = np.array([
    [500.0, 0.0, 320.0],
    [0.0, 500.0, 200.0],
    [0.0, 0.0, 1.0],
])


def transform(x=0.0, y=0.0, z=0.0, rotation=None):
    """Return a 4x4 transform."""
    matrix = np.eye(4, dtype=np.float64)
    if rotation is not None:
        matrix[:3, :3] = rotation
    matrix[:3, 3] = [x, y, z]
    return matrix


def depth_array(depth_m, width=WIDTH, height=HEIGHT):
    """Return a uint16 millimetre depth image filled with one value."""
    return np.full(
        (height, width), int(round(depth_m * 1000.0)), dtype=np.uint16)


def make_detection(bbox=None, label='target', score=0.9, keypoints=None):
    """Return a detection centred on the principal point by default."""
    if bbox is None:
        bbox = (300.0, 180.0, 340.0, 220.0)
    if keypoints is None:
        keypoints = [Keypoint(320.0, 200.0, 0.95)]
    return Detection(
        label=label,
        score=score,
        bbox_xyxy=tuple(float(value) for value in bbox),
        class_id=0,
        keypoints=list(keypoints),
    )


def load_specs():
    """Load the real camera definitions."""
    return load_camera_specs(str(REPO_ROOT / 'config' / 'cameras.yaml'))


def load_fusion(name='fusion.yaml'):
    """Load a fusion policy by file name."""
    return load_fusion_config(str(REPO_ROOT / 'config' / name))


def load_frames():
    """Load the static fallbacks."""
    return load_static_frames(str(REPO_ROOT / 'config' / 'frames.yaml'))


def feed_frame(pipeline, camera_id, stamp_ns, pose, depth_m, seq=1,
               camera_matrix=None, width=WIDTH, height=HEIGHT,
               detection=None, received_ns=None):
    """Store a frame, depth and pose; return the detection for that frame."""
    if camera_matrix is None:
        camera_matrix = K
    if received_ns is None:
        received_ns = int(stamp_ns) + 1_000_000
    pipeline.ingest_frame(
        camera_id, seq, stamp_ns, received_ns, width, height,
        'test_camera_frame', camera_matrix,
        np.zeros(5, dtype=np.float64))
    pipeline.ingest_depth(
        camera_id, stamp_ns, depth_array(depth_m, width, height),
        scale_m=0.001, metric=False)
    pipeline.ingest_pose(camera_id, stamp_ns, pose)
    if detection is None:
        detection = make_detection()
    return detection


def observation(camera_id='d435i_ext', role=Role.EXTERNAL, position=(0.0, 0.0, 0.4),
                sigma=0.002, depth_m=0.4, score=0.9, keypoint_score=0.0,
                occlusion=0.0, capture_ns=1_000_000_000):
    """Build a TargetObservation with an isotropic position covariance."""
    return TargetObservation(
        camera_id=camera_id,
        role=role,
        label='target',
        class_id=0,
        capture_ns=int(capture_ns),
        received_ns=int(capture_ns) + 10_000_000,
        frame_seq=1,
        frame_id_camera='camera',
        detection_score=float(score),
        keypoint_score=float(keypoint_score),
        position_base=np.array(position, dtype=np.float64),
        covariance_base=np.eye(3) * float(sigma) ** 2,
        depth_m=float(depth_m),
        valid_depth_fraction=1.0 - float(occlusion),
        occlusion_ratio=float(occlusion),
    )
