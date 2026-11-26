"""
Visible-only 3-D geometry for one instance mask.

The output deliberately names every geometric field ``visible_*``. The
centroid/extent describe only the observed surface; the true centroid, full
extent, volume and mass remain unknown. This module is ROS-free and uses the
same pinhole/deprojection conventions as ``jcheng_perception_core``.
"""

import dataclasses

import numpy as np

from jcheng_perception_core.geometry import (
    deproject_pixel,
    quaternion_from_matrix,
    transform_points,
)


@dataclasses.dataclass(frozen=True)
class GeometryParams:
    """Sampling and validity limits for visible geometry."""

    min_depth_m: float = 0.05
    max_depth_m: float = 2.0
    stride: int = 2
    min_points: int = 20

    def __post_init__(self):
        """Validate the parameters."""
        if not 0.0 < float(self.min_depth_m) < float(self.max_depth_m):
            raise ValueError('depth limits must satisfy 0 < min < max')
        if int(self.stride) < 1:
            raise ValueError('stride must be >= 1')
        if int(self.min_points) < 1:
            raise ValueError('min_points must be >= 1')


def _failure(status, reason, depth_valid_fraction=0.0, point_count=0,
             frame_id='base_link'):
    return {
        'status': status,
        'reason': reason,
        'frame_id': frame_id,
        'visible_point_count': int(point_count),
        'depth_valid_fraction': float(depth_valid_fraction),
        'visible_centroid_base': None,
        'visible_centroid_covariance': None,
        'visible_aabb': None,
        'visible_obb': None,
        'field_status': {
            'visible_centroid_base': 'unknown',
            'visible_centroid_covariance': 'unknown',
            'visible_aabb': 'unknown',
            'visible_obb': 'unknown',
            'true_centroid': 'unknown',
            'full_extent': 'unknown',
            'volume': 'unknown',
        },
        'unknowns': [
            'true_centroid', 'full_extent', 'volume', 'mass',
            'material_composition', 'ore_grade', 'economic_value',
        ],
    }


def _aabb(points, frame_id):
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    center = (minimum + maximum) / 2.0
    extent = maximum - minimum
    return {
        'center': [float(v) for v in center],
        'extent': [float(v) for v in extent],
        'frame_id': frame_id,
    }


def _obb(points, frame_id):
    """Return a PCA OBB; orientation sign is chosen for a right-handed frame."""
    centroid = points.mean(axis=0)
    centered = points - centroid
    covariance = np.cov(centered, rowvar=False, ddof=1)
    if covariance.shape != (3, 3) or not np.all(np.isfinite(covariance)):
        return None
    values, vectors = np.linalg.eigh(covariance)
    order = np.argsort(values)[::-1]
    axes = vectors[:, order]
    if np.linalg.det(axes) < 0.0:
        axes[:, 2] *= -1.0
    # Re-orthonormalise after the sign choice.
    axes, _ = np.linalg.qr(axes)
    if np.linalg.det(axes) < 0.0:
        axes[:, 2] *= -1.0
    projections = centered @ axes
    minimum = projections.min(axis=0)
    maximum = projections.max(axis=0)
    extent = maximum - minimum
    center = centroid + axes @ ((minimum + maximum) / 2.0)
    quaternion = quaternion_from_matrix(axes)
    return {
        'center': [float(v) for v in center],
        'quaternion_xyzw': [float(v) for v in quaternion],
        'extent': [float(v) for v in extent],
        'frame_id': frame_id,
        'axes': [[float(v) for v in row] for row in axes],
    }


def visible_geometry(mask, depth_m, camera_matrix, transform_base_camera,
                     params=None, frame_id='base_link'):
    """Compute visible point cloud, centroid, covariance, AABB and OBB.

    ``depth_m`` must already be metric and aligned to ``mask``. Invalid depth
    is not interpolated: a mask with less than ``min_points`` supported pixels
    returns ``status='insufficient_depth'``.
    """
    params = params or GeometryParams()
    mask = np.asarray(mask).astype(bool, copy=False)
    depth = np.asarray(depth_m, dtype=np.float64)
    if mask.ndim != 2 or depth.ndim != 2:
        raise ValueError('mask and depth must be 2-D')
    if mask.shape != depth.shape:
        raise ValueError('mask and depth shapes differ')
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    if intrinsics.shape != (3, 3):
        raise ValueError('camera_matrix must be 3x3')
    transform = np.asarray(transform_base_camera, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError('transform_base_camera must be 4x4')

    rows, columns = np.nonzero(mask)
    if rows.size == 0:
        return _failure('invalid_mask', 'mask is empty', frame_id=frame_id)
    stride = int(params.stride)
    if stride > 1:
        row_origin = int(rows.min())
        column_origin = int(columns.min())
        keep = (
            ((rows - row_origin) % stride == 0)
            & ((columns - column_origin) % stride == 0)
        )
        rows = rows[keep]
        columns = columns[keep]
    depths = depth[rows, columns]
    valid = (
        np.isfinite(depths)
        & (depths >= float(params.min_depth_m))
        & (depths <= float(params.max_depth_m))
    )
    point_count = int(np.count_nonzero(valid))
    depth_valid_fraction = point_count / float(rows.size)
    if point_count < int(params.min_points):
        return _failure(
            'insufficient_depth',
            'valid depth samples %d < min_points %d' % (
                point_count, int(params.min_points)),
            depth_valid_fraction, point_count, frame_id)

    camera_points = np.empty((point_count, 3), dtype=np.float64)
    valid_rows = rows[valid]
    valid_columns = columns[valid]
    valid_depths = depths[valid]
    for index in range(point_count):
        camera_points[index] = deproject_pixel(
            valid_columns[index], valid_rows[index], valid_depths[index],
            intrinsics)
    base_points = transform_points(transform, camera_points)

    centroid = base_points.mean(axis=0)
    covariance = np.cov(base_points, rowvar=False, ddof=1)
    if covariance.shape != (3, 3) or not np.all(np.isfinite(covariance)):
        covariance = np.zeros((3, 3), dtype=np.float64)
    aabb = _aabb(base_points, frame_id)
    obb = _obb(base_points, frame_id)
    return {
        'status': 'computed',
        'reason': None,
        'frame_id': frame_id,
        'visible_point_count': int(point_count),
        'depth_valid_fraction': float(depth_valid_fraction),
        'visible_centroid_base': [float(v) for v in centroid],
        'visible_centroid_covariance': [
            [float(v) for v in row] for row in covariance
        ],
        'visible_aabb': aabb,
        'visible_obb': obb,
        'field_status': {
            'visible_centroid_base': 'algorithm_estimate',
            'visible_centroid_covariance': 'algorithm_estimate',
            'visible_aabb': 'algorithm_estimate',
            'visible_obb': 'algorithm_estimate',
            'true_centroid': 'unknown',
            'full_extent': 'unknown',
            'volume': 'unknown',
        },
        'unknowns': [
            'true_centroid', 'full_extent', 'volume', 'mass',
            'material_composition', 'ore_grade', 'economic_value',
        ],
    }
