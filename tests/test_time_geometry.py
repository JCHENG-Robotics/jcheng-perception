"""Timestamp ring, geometry and quaternion tests."""

import math

import numpy as np

from jcheng_perception_core import (
    PoseRingBuffer,
    TimestampRingBuffer,
    deproject_pixel,
    invert_transform,
    make_transform,
    matrix_from_quaternion,
    object_yaw_from_points,
    plane_pose_from_points,
    project_point,
    quaternion_from_matrix,
    quaternion_slerp,
    quaternion_yaw,
    transform_points,
    yaw_quaternion,
)


def _angular_difference(first, second):
    return abs(math.atan2(math.sin(first - second), math.cos(first - second)))


def test_timestamp_buffer_sorted_and_nearest():
    buffer = TimestampRingBuffer(max_items=10, max_age_ns=10_000)
    buffer.add(300, 'c')
    buffer.add(100, 'a')
    buffer.add(200, 'b')
    assert buffer.stamps == [100, 200, 300]
    assert buffer.nearest(140) == (100, 'a')
    assert buffer.nearest(170) == (200, 'b')
    assert buffer.nearest(900, max_delta_ns=100) is None
    assert buffer.latest() == (300, 'c')


def test_timestamp_buffer_replaces_duplicate_and_prunes():
    buffer = TimestampRingBuffer(max_items=2, max_age_ns=100)
    buffer.add(0, 'old')
    buffer.add(10, 'a')
    buffer.add(10, 'replaced')
    assert buffer.nearest(10) == (10, 'replaced')
    buffer.add(20, 'b')
    assert buffer.stamps == [10, 20]
    buffer.prune(150)
    assert buffer.stamps == []


def test_pose_ring_interpolates_translation_and_rotation():
    ring = PoseRingBuffer(max_items=8, max_age_ns=10_000_000_000)
    start = make_transform(np.eye(3), [0.0, 0.0, 0.0])
    end_rotation = matrix_from_quaternion(yaw_quaternion(math.pi / 2.0))
    end = make_transform(end_rotation, [1.0, 0.0, 0.0])
    ring.add(0, start)
    ring.add(1_000_000_000, end)
    pose = ring.pose_at(
        500_000_000, max_gap_ns=1_000_000_000,
        max_delta_ns=200_000_000)
    assert pose is not None
    assert np.allclose(pose[:3, 3], [0.5, 0.0, 0.0], atol=1e-9)
    assert abs(quaternion_yaw(quaternion_from_matrix(pose[:3, :3])) -
               math.pi / 4.0) < 1e-6


def test_pose_ring_rejects_wide_gap_and_far_edge():
    ring = PoseRingBuffer(max_items=8, max_age_ns=10_000_000_000)
    ring.add(0, np.eye(4))
    ring.add(1_000_000_000, np.eye(4))
    assert ring.pose_at(
        500_000_000, max_gap_ns=100_000_000) is None
    assert ring.pose_at(
        2_000_000_000, max_gap_ns=1_000_000_000,
        max_delta_ns=100_000_000) is None


def test_quaternion_roundtrip_and_slerp():
    yaw = 0.7
    quaternion = yaw_quaternion(yaw)
    matrix = matrix_from_quaternion(quaternion)
    recovered = quaternion_from_matrix(matrix)
    assert abs(quaternion_yaw(recovered) - yaw) < 1e-9
    halfway = quaternion_slerp(yaw_quaternion(0.0), quaternion, 0.5)
    assert abs(quaternion_yaw(halfway) - yaw / 2.0) < 1e-9


def test_deproject_project_roundtrip():
    camera_matrix = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 480.0, 200.0],
        [0.0, 0.0, 1.0],
    ])
    point = deproject_pixel(400.0, 150.0, 2.0, camera_matrix)
    assert np.allclose(point, [0.32, -0.2083333333, 2.0], atol=1e-9)
    pixel = project_point(point, camera_matrix)
    assert np.allclose(pixel, [400.0, 150.0], atol=1e-9)


def test_transform_inverse_and_points():
    rotation = matrix_from_quaternion(yaw_quaternion(0.4))
    transform = make_transform(rotation, [0.1, -0.2, 0.3])
    points = np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]])
    transformed = transform_points(transform, points)
    restored = transform_points(invert_transform(transform), transformed)
    assert np.allclose(restored, points, atol=1e-12)


def test_plane_pose_from_points_recovers_yaw():
    yaw = math.radians(30.0)
    axis_x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    axis_y = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    samples = []
    for x in np.linspace(-0.10, 0.10, 20):
        for y in np.linspace(-0.03, 0.03, 10):
            samples.append(
                np.array([0.3, 0.1, 0.5]) + x * axis_x + y * axis_y)
    result = plane_pose_from_points(np.asarray(samples))
    assert result is not None
    orientation, normal, ratio = result
    assert abs(normal[2] - 1.0) < 0.01
    assert ratio > 0.95
    assert _angular_difference(quaternion_yaw(orientation), yaw) < 0.01


def test_object_yaw_ignores_table_plane():
    yaw = math.radians(25.0)
    axis_x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    axis_y = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    table = []
    for x in np.linspace(-0.20, 0.20, 30):
        for y in np.linspace(-0.20, 0.20, 30):
            table.append([x, y, 0.0])
    top = []
    for x in np.linspace(-0.05, 0.05, 12):
        for y in np.linspace(-0.025, 0.025, 8):
            top.append(
                np.array([0.0, 0.0, 0.03]) + x * axis_x + y * axis_y)
    result = object_yaw_from_points(np.asarray(table + top))
    assert result is not None
    orientation, height, support = result
    assert 0.025 < height < 0.035
    assert 0.05 < support < 0.3
    assert _angular_difference(quaternion_yaw(orientation), yaw) < 0.02
