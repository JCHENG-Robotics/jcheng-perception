"""
Turn a 2-D detection plus a time-matched depth frame and TF sample into a
3-D base-frame observation.

This is the only place where pixels become metres. Failures are explicit: a
missing depth sample, a mismatched image size or a non-invertible transform
returns ``None`` and the edge node counts the drop. The pipeline never
fabricates a depth or reuses the latest frame.
"""

import math

import numpy as np

from .depth import (
    border_occlusion_ratio,
    depth_sigma_m,
    depth_valid_fraction,
    lateral_sigma_m,
    sample_bbox_depth,
    sample_depth,
)
from .geometry import (
    deproject_pixel,
    object_yaw_from_points,
    transform_points,
    yaw_quaternion,
)
from .types import (
    Detection,
    DepthFrame,
    FrameMeta,
    Role,
    TargetObservation,
)


def _keypoint_candidates(detection, width, height, min_confidence, max_count):
    """Return in-image keypoints above the confidence threshold."""
    candidates = []
    for keypoint in detection.keypoints:
        confidence = float(keypoint.confidence)
        if confidence < float(min_confidence):
            continue
        u = float(keypoint.x)
        v = float(keypoint.y)
        if not (0.0 <= u < width and 0.0 <= v < height):
            continue
        candidates.append((confidence, u, v))
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[:max(1, int(max_count))]


def _keypoint_axis_yaw(detection, depth_frame, camera_matrix,
                       transform, min_depth_m, max_depth_m, pairs):
    """Return a yaw from the first valid keypoint axis pair, or None."""
    keypoints = detection.keypoints
    for pair in pairs:
        if len(pair) != 2:
            continue
        first_index, second_index = pair
        if max(first_index, second_index) >= len(keypoints):
            continue
        points = []
        for index in (first_index, second_index):
            keypoint = keypoints[index]
            sample = sample_depth(
                depth_frame, keypoint.x, keypoint.y, radius=1, min_valid=1,
                min_depth_m=min_depth_m, max_depth_m=max_depth_m)
            if sample is None:
                break
            point_camera = deproject_pixel(
                keypoint.x, keypoint.y, sample[0], camera_matrix)
            points.append(point_camera)
        if len(points) != 2:
            continue
        base_points = transform_points(transform, np.asarray(points))
        direction = base_points[1] - base_points[0]
        length = float(np.linalg.norm(direction[:2]))
        if length < 0.005:
            continue
        yaw = math.atan2(float(direction[1]), float(direction[0]))
        return yaw_quaternion(yaw), min(0.30, 0.05 / max(length, 1e-3))
    return None


def _pointcloud_orientation(bbox, depth_frame, transform, camera_matrix,
                            min_depth_m, max_depth_m, fusion):
    """Estimate a tabletop yaw from the depth points inside a box."""
    x0, y0, x1, y1 = bbox
    width = x1 - x0
    height = y1 - y0
    if width < 6.0 or height < 6.0:
        return None
    inner = (
        x0 + 0.10 * width,
        y0 + 0.10 * height,
        x1 - 0.10 * width,
        y1 - 0.10 * height,
    )
    pixel_points = depth_frame.roi_metres(
        inner[0], inner[1], inner[2], inner[3], stride=2)
    if pixel_points.shape[0] < 20:
        return None
    rows, columns, depths = (
        pixel_points[:, 1], pixel_points[:, 0], pixel_points[:, 2])
    valid = (depths >= min_depth_m) & (depths <= max_depth_m)
    if not np.any(valid):
        return None
    rows = rows[valid]
    columns = columns[valid]
    depths = depths[valid]
    intrinsics = np.asarray(camera_matrix, dtype=np.float64)
    fx, fy = intrinsics[0, 0], intrinsics[1, 1]
    cx, cy = intrinsics[0, 2], intrinsics[1, 2]
    camera_points = np.column_stack([
        (columns - cx) * depths / fx,
        (rows - cy) * depths / fy,
        depths,
    ])
    base_points = transform_points(transform, camera_points)
    result = object_yaw_from_points(
        base_points,
        table_clearance_m=fusion.table_clearance_m,
        max_tilt_deg=fusion.orientation_max_tilt_deg,
    )
    if result is None:
        return None
    orientation, height, support_ratio = result
    sigma = max(
        0.12,
        float(fusion.orientation_sigma_rad) / max(support_ratio, 0.05),
    )
    return orientation, min(sigma, 0.8), float(height)


def build_observation(detection, frame_meta, depth_frame, transform_base_camera,
                      spec, fusion):
    """
    Build a :class:`TargetObservation` or return ``None``.

    ``transform_base_camera`` maps camera-optical coordinates into
    ``fusion.target_frame`` (nominally ``base_link``).
    """
    if not isinstance(detection, Detection):
        raise TypeError('detection must be a Detection')
    if not isinstance(frame_meta, FrameMeta):
        raise TypeError('frame_meta must be a FrameMeta')
    if not isinstance(depth_frame, DepthFrame):
        raise TypeError('depth_frame must be a DepthFrame')
    transform = np.asarray(transform_base_camera, dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError('transform_base_camera must be 4x4')
    if depth_frame.height != frame_meta.height or \
            depth_frame.width != frame_meta.width:
        return None

    width = frame_meta.width
    height = frame_meta.height
    bbox = tuple(float(value) for value in detection.bbox_xyxy)
    if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        return None
    clipped_bbox = (
        max(0.0, bbox[0]),
        max(0.0, bbox[1]),
        min(float(width - 1), bbox[2]),
        min(float(height - 1), bbox[3]),
    )
    if clipped_bbox[2] <= clipped_bbox[0] or \
            clipped_bbox[3] <= clipped_bbox[1]:
        return None

    keypoints = _keypoint_candidates(
        detection, width, height,
        spec.keypoint_min_confidence, spec.max_keypoints)
    depth_sample = None
    keypoint_confidence = None
    source = 'bbox'
    for confidence, u, v in keypoints:
        sample = sample_depth(
            depth_frame, u, v, radius=1, min_valid=1,
            min_depth_m=spec.min_depth_m, max_depth_m=spec.max_depth_m)
        if sample is not None:
            depth_sample = {
                'depth_m': sample[0],
                'valid_fraction': sample[1],
                'pixel': (u, v),
                'source': 'keypoint',
            }
            keypoint_confidence = confidence
            source = 'keypoint'
            break
    if depth_sample is None:
        depth_sample = sample_bbox_depth(
            depth_frame, clipped_bbox, radius=2, min_valid=2,
            min_depth_m=spec.min_depth_m, max_depth_m=spec.max_depth_m)
    if depth_sample is None:
        return None

    depth_m = float(depth_sample['depth_m'])
    if not (spec.min_depth_m <= depth_m <= spec.max_depth_m):
        return None
    u, v = depth_sample['pixel']
    point_camera = deproject_pixel(
        u, v, depth_m, frame_meta.camera_matrix)
    point_base = transform_points(transform, point_camera)[0]

    intrinsics = np.asarray(frame_meta.camera_matrix, dtype=np.float64)
    fx, fy = float(intrinsics[0, 0]), float(intrinsics[1, 1])
    pixel_sigma = float(spec.pixel_sigma_px)
    if keypoint_confidence is not None:
        pixel_sigma /= max(float(keypoint_confidence), 0.15)
    sigma_x = lateral_sigma_m(depth_m, fx, pixel_sigma)
    sigma_y = lateral_sigma_m(depth_m, fy, pixel_sigma)
    sigma_z = depth_sigma_m(
        depth_m, spec.depth_noise_quadratic, spec.depth_noise_floor_m)
    covariance_camera = np.diag([
        sigma_x * sigma_x + fusion.measurement_floor_m ** 2,
        sigma_y * sigma_y + fusion.measurement_floor_m ** 2,
        sigma_z * sigma_z + fusion.measurement_floor_m ** 2,
    ])
    rotation = transform[:3, :3]
    covariance_base = rotation @ covariance_camera @ rotation.T

    valid_fraction = depth_valid_fraction(
        depth_frame, clipped_bbox, stride=3,
        min_depth_m=spec.min_depth_m, max_depth_m=spec.max_depth_m)
    depth_occlusion = 1.0 - min(
        1.0, valid_fraction / max(0.5, spec.min_valid_depth_fraction * 2.0))
    border_occlusion = border_occlusion_ratio(
        clipped_bbox, width, height, margin_px=2)
    occlusion_ratio = max(
        max(0.0, depth_occlusion), 0.5 * border_occlusion)
    if source == 'keypoint':
        occlusion_ratio *= 0.5

    orientation = None
    orientation_sigma = 0.0
    axis = None
    if spec.keypoint_axis_pairs:
        axis = _keypoint_axis_yaw(
            detection, depth_frame,
            frame_meta.camera_matrix, transform,
            spec.min_depth_m, spec.max_depth_m,
            spec.keypoint_axis_pairs)
    if axis is not None:
        orientation, orientation_sigma = axis
    else:
        cloud = _pointcloud_orientation(
            clipped_bbox, depth_frame, transform,
            frame_meta.camera_matrix, spec.min_depth_m, spec.max_depth_m,
            fusion)
        if cloud is not None:
            orientation, orientation_sigma, _ = cloud

    observation = TargetObservation(
        camera_id=spec.camera_id,
        role=spec.role,
        label=str(detection.label),
        class_id=int(detection.class_id),
        capture_ns=int(frame_meta.capture_ns),
        received_ns=int(frame_meta.received_ns),
        frame_seq=int(frame_meta.seq),
        frame_id_camera=frame_meta.frame_id,
        detection_score=float(detection.score),
        keypoint_score=float(
            keypoint_confidence if keypoint_confidence is not None else 0.0),
        position_base=point_base,
        covariance_base=covariance_base,
        depth_m=depth_m,
        valid_depth_fraction=valid_fraction,
        occlusion_ratio=float(min(1.0, occlusion_ratio)),
        orientation_base=orientation,
        orientation_sigma_rad=float(orientation_sigma),
        pixel=(float(u), float(v)),
    )
    return observation
