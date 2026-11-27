"""
Offline dual-view instance association prototype (S4.2).

The matcher is deliberately ROS-free and uses only geometry that the S4.1
generator (or a future recorded dual-view bundle) provides: per-view masks,
metric depth, camera intrinsics/extrinsics and visible 3-D points. It never
uses the ground-truth ``global_id`` while matching; GT pairs are used only by
:func:`evaluate_association`.

This is a prototype, not a runtime capability: no DDS, device access or motion.
"""

import dataclasses
import copy
import json
import math
import os
import time
from typing import Dict, List, Tuple

import cv2
import numpy as np

from .synthetic_dual import Camera

ASSOCIATION_SCHEMA_VERSION = 'jcheng.scene_association.v1'
ASSOCIATION_EVAL_SCHEMA_VERSION = 'jcheng.ore_association_eval.v1'

_VIEW_IDS = ('ext', 'wrist')


@dataclasses.dataclass(frozen=True)
class AssociationParams:
    """Thresholds and weights for the offline matcher."""

    strategy: str = 'mutual_projection'
    association_margin_m: float = 0.02
    min_mutual_support: float = 0.15
    min_support_pixels: int = 8
    depth_tolerance_m: float = 0.01
    extrinsic_depth_sigma_m: float = 0.0
    depth_sigma_multiplier: float = 2.0
    max_chamfer_ratio: float = 1.0
    size_ratio_max: float = 2.5
    ambiguity_margin: float = 0.05
    max_points: int = 300
    max_candidates: int = 512
    sync_window_ns: int = 50_000_000
    weight_support: float = 1.0
    weight_chamfer: float = 0.35
    weight_size: float = 0.15
    weight_prior: float = 0.10

    def __post_init__(self):
        """Validate the parameter ranges."""
        if self.strategy not in ('mutual_projection', 'centroid'):
            raise ValueError('unsupported association strategy: %s' %
                             self.strategy)
        if self.association_margin_m < 0:
            raise ValueError('association_margin_m must be non-negative')
        if not 0.0 <= self.min_mutual_support <= 1.0:
            raise ValueError('min_mutual_support must be in [0, 1]')
        if self.min_support_pixels < 1:
            raise ValueError('min_support_pixels must be positive')
        if self.depth_tolerance_m < 0:
            raise ValueError('depth_tolerance_m must be non-negative')
        if self.extrinsic_depth_sigma_m < 0:
            raise ValueError('extrinsic_depth_sigma_m must be non-negative')
        if self.depth_sigma_multiplier < 0:
            raise ValueError('depth_sigma_multiplier must be non-negative')
        if self.max_chamfer_ratio <= 0 or self.size_ratio_max < 1.0:
            raise ValueError('invalid chamfer/size limits')
        if not 0.0 <= self.ambiguity_margin <= 10.0:
            raise ValueError('ambiguity_margin out of range')
        if self.max_points < 8 or self.max_candidates < 1:
            raise ValueError('max_points/max_candidates too small')
        if self.sync_window_ns < 0:
            raise ValueError('sync_window_ns must be non-negative')

    @property
    def effective_depth_tolerance_m(self):
        """Return the depth gate widened by the declared calibration sigma.

        The mutual-projection support test compares a projected depth against
        the target-view depth map. A per-view depth-axis extrinsic uncertainty
        biases that comparison one-sidedly, so the declared sigma is folded
        into the gate. With the default ``extrinsic_depth_sigma_m=0`` the gate
        is exactly ``depth_tolerance_m`` and previous results are unchanged.
        """
        return (self.depth_tolerance_m +
                self.depth_sigma_multiplier * self.extrinsic_depth_sigma_m)

    def to_dict(self):
        """Return a JSON-serialisable parameter record."""
        return {
            field.name: getattr(self, field.name)
            for field in dataclasses.fields(self)
        }


@dataclasses.dataclass(frozen=True)
class AssociationInstance:
    """One visible instance with its per-view 3-D points."""

    view_id: str
    instance_id: str
    mask_index: int
    bbox_xyxy: Tuple[int, int, int, int]
    points_base: np.ndarray
    centroid_base: np.ndarray
    covariance_base: np.ndarray
    aabb_min: np.ndarray
    aabb_max: np.ndarray
    visible_area_px: int
    occlusion_ratio: float

    def __post_init__(self):
        """Normalise arrays and validate the instance."""
        if self.view_id not in _VIEW_IDS:
            raise ValueError('unsupported view_id: %s' % self.view_id)
        if not self.instance_id:
            raise ValueError('instance_id must be non-empty')
        points = np.asarray(self.points_base, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or points.shape[0] == 0:
            raise ValueError('points_base must be non-empty Nx3')
        if not np.all(np.isfinite(points)):
            raise ValueError('points_base must be finite')
        object.__setattr__(self, 'points_base', points)
        for name in ('centroid_base', 'aabb_min', 'aabb_max'):
            value = np.asarray(getattr(self, name), dtype=np.float64).reshape(3)
            object.__setattr__(self, name, value)
        covariance = np.asarray(
            self.covariance_base, dtype=np.float64).reshape(3, 3)
        object.__setattr__(self, 'covariance_base', covariance)

    @property
    def diagonal_m(self):
        """Return the visible AABB diagonal length."""
        return float(np.linalg.norm(self.aabb_max - self.aabb_min))


@dataclasses.dataclass
class ViewFrame:
    """One camera's label image, depth and visible instances."""

    view_id: str
    camera: Camera
    label: np.ndarray
    depth_m: np.ndarray
    instances: List[AssociationInstance]
    timestamp_ns: int = 0


@dataclasses.dataclass
class AssociationScene:
    """Both views of one scene plus scene-level attributes."""

    scene_id: str
    frames: Dict[str, ViewFrame]
    attributes: Dict[str, object]


@dataclasses.dataclass
class AssociationDataset:
    """A loaded dual-view dataset with ground-truth pairs."""

    root: str
    manifest: Dict[str, object]
    cameras: Dict[str, Camera]
    scenes: List[AssociationScene]
    pairs: Dict[str, List[Dict[str, object]]]
    singletons: Dict[str, List[Dict[str, object]]]


def _backproject(camera, columns, rows, depth):
    """Backproject pixel coordinates with metric z-depth into base_link."""
    depth = np.asarray(depth, dtype=np.float64)
    x = (np.asarray(columns, dtype=np.float64) - camera.cx) / camera.fx * depth
    y = (np.asarray(rows, dtype=np.float64) - camera.cy) / camera.fy * depth
    points_camera = np.stack([x, y, depth], axis=-1)
    rotation = camera.rotation_world_to_camera
    return points_camera.reshape(-1, 3) @ rotation + camera.position_base


def _project(points, camera):
    """Project base_link points into a camera; return u, v, z."""
    points = np.asarray(points, dtype=np.float64)
    points_camera = (points - camera.position_base) @ \
        camera.rotation_world_to_camera.T
    z = points_camera[:, 2]
    safe = np.where(np.abs(z) > 1e-12, z, 1e-12)
    columns = camera.fx * points_camera[:, 0] / safe + camera.cx
    rows = camera.fy * points_camera[:, 1] / safe + camera.cy
    return columns, rows, z


def _instance_from_view(view_id, camera, label, depth, instance_record,
                        max_points):
    bbox = [int(value) for value in instance_record['bbox_xyxy']]
    mask_index = int(instance_record['mask_index'])
    x0, y0, x1, y1 = bbox
    local = label[y0:y1 + 1, x0:x1 + 1] == mask_index
    rows, columns = np.nonzero(local)
    if rows.size == 0:
        return None
    rows = rows + y0
    columns = columns + x0
    depth_values = depth[rows, columns].astype(np.float64)
    valid = depth_values > 0
    if not np.any(valid):
        return None
    points = _backproject(camera, columns[valid], rows[valid],
                          depth_values[valid])
    points = _subsample(points, max_points)
    centroid = points.mean(axis=0)
    if points.shape[0] >= 3:
        covariance = np.cov(points.T, bias=True)
    else:
        covariance = np.zeros((3, 3), dtype=np.float64)
    covariance = covariance + np.eye(3) * 1e-8
    return AssociationInstance(
        view_id=view_id,
        instance_id=str(instance_record['instance_id']),
        mask_index=mask_index,
        bbox_xyxy=(x0, y0, x1, y1),
        points_base=points,
        centroid_base=centroid,
        covariance_base=covariance,
        aabb_min=points.min(axis=0),
        aabb_max=points.max(axis=0),
        visible_area_px=int(rows.size),
        occlusion_ratio=float(instance_record.get('occlusion_ratio', 0.0)),
    )


def _subsample(points, max_points):
    if points.shape[0] <= max_points:
        return points
    indices = np.linspace(
        0, points.shape[0] - 1, int(max_points)).astype(np.int64)
    return points[np.unique(indices)]


def load_dual_view_dataset(root, max_points=300):
    """Load a S4.1 dual-view dataset for association."""
    manifest_path = os.path.join(root, 'dataset.json')
    with open(manifest_path, 'r', encoding='utf-8') as handle:
        manifest = json.load(handle)
    if manifest.get('schema_version') != 'jcheng.ore_dual_view_dataset.v1':
        raise ValueError('unsupported dual-view dataset schema: %s' %
                         manifest.get('schema_version'))
    cameras = {}
    for view_id in _VIEW_IDS:
        record = manifest['cameras'][view_id]
        cameras[view_id] = Camera(
            view_id=view_id,
            width=int(record['width']),
            height=int(record['height']),
            fx=float(record['fx']),
            fy=float(record['fy']),
            cx=float(record['cx']),
            cy=float(record['cy']),
            camera_to_base=np.asarray(
                record['camera_to_base'], dtype=np.float64),
            fov_deg=float(record.get('fov_deg', 55.0)),
        )
    pairs = {}
    singletons = {}
    pairs_path = os.path.join(root, manifest.get('pairs_path', 'pairs.jsonl'))
    with open(pairs_path, 'r', encoding='utf-8') as handle:
        for line in handle:
            if not line.strip():
                continue
            record = json.loads(line)
            scene_id = str(record['scene_id'])
            pairs[scene_id] = list(record.get('pairs', []))
            singletons[scene_id] = list(record.get('singletons', []))
    scenes = []
    for scene_record in manifest['scenes']:
        frames = {}
        for view_id in _VIEW_IDS:
            view_record = scene_record['views'][view_id]
            label = cv2.imread(
                os.path.join(root, view_record['mask']), cv2.IMREAD_UNCHANGED)
            depth = np.load(os.path.join(root, view_record['depth']))
            if label is None or depth is None:
                raise ValueError('cannot read %s view arrays' % view_id)
            if label.shape != depth.shape:
                raise ValueError('label/depth shape mismatch for %s' % view_id)
            if label.shape != (cameras[view_id].height,
                               cameras[view_id].width):
                raise ValueError('array shape does not match camera for %s' %
                                 view_id)
            instances = []
            for instance_record in scene_record['instances']:
                record = instance_record['views'].get(view_id)
                if record is None:
                    continue
                instance = _instance_from_view(
                    view_id, cameras[view_id], label, depth, record,
                    max_points)
                if instance is not None:
                    instances.append(instance)
            frames[view_id] = ViewFrame(
                view_id=view_id,
                camera=cameras[view_id],
                label=label,
                depth_m=depth.astype(np.float64),
                instances=instances,
                timestamp_ns=int(view_record.get('timestamp_ns', 0)),
            )
        scenes.append(AssociationScene(
            scene_id=str(scene_record['scene_id']),
            frames=frames,
            attributes=dict(scene_record.get('attributes', {})),
        ))
    return AssociationDataset(
        root=os.path.abspath(root),
        manifest=manifest,
        cameras=cameras,
        scenes=scenes,
        pairs=pairs,
        singletons=singletons,
    )


def _rotation_matrix(axis, angle_deg):
    angle = math.radians(float(angle_deg))
    cosine = math.cos(angle)
    sine = math.sin(angle)
    if axis == 'x':
        return np.asarray([
            [1.0, 0.0, 0.0],
            [0.0, cosine, -sine],
            [0.0, sine, cosine],
        ])
    if axis == 'y':
        return np.asarray([
            [cosine, 0.0, sine],
            [0.0, 1.0, 0.0],
            [-sine, 0.0, cosine],
        ])
    if axis == 'z':
        return np.asarray([
            [cosine, -sine, 0.0],
            [sine, cosine, 0.0],
            [0.0, 0.0, 1.0],
        ])
    raise ValueError('unsupported rotation axis: %s' % axis)


def perturb_scene_extrinsics(scene, view_id, translation_m=(0, 0, 0),
                             rotation_deg=0.0, axis='z'):
    """Apply an extrinsic error to one view's points and camera.

    The label/depth arrays stay at their original rendering; this deliberately
    simulates a wrong calibration estimate, not a new observation.
    """
    if view_id not in _VIEW_IDS:
        raise ValueError('unsupported view_id: %s' % view_id)
    translation = np.asarray(translation_m, dtype=np.float64).reshape(3)
    if not np.all(np.isfinite(translation)):
        raise ValueError('translation_m must be finite')
    rotation = _rotation_matrix(axis, rotation_deg)
    delta = np.eye(4, dtype=np.float64)
    delta[:3, :3] = rotation
    delta[:3, 3] = translation
    frames = dict(scene.frames)
    frame = frames[view_id]
    instances = []
    for instance in frame.instances:
        points = instance.points_base @ rotation.T + translation
        centroid = instance.centroid_base @ rotation.T + translation
        covariance = rotation @ instance.covariance_base @ rotation.T
        instances.append(dataclasses.replace(
            instance,
            points_base=points,
            centroid_base=centroid,
            covariance_base=covariance,
            aabb_min=points.min(axis=0),
            aabb_max=points.max(axis=0),
        ))
    camera = frame.camera
    camera_to_base = delta @ camera.camera_to_base
    frames[view_id] = dataclasses.replace(
        frame,
        camera=dataclasses.replace(camera, camera_to_base=camera_to_base),
        instances=instances,
    )
    return dataclasses.replace(scene, frames=frames)


def perturb_dataset_extrinsics(dataset, view_id, translation_m=(0, 0, 0),
                               rotation_deg=0.0, axis='z'):
    """Return a dataset copy with one view's extrinsic calibration perturbed."""
    scenes = [
        perturb_scene_extrinsics(
            scene, view_id, translation_m, rotation_deg, axis)
        for scene in dataset.scenes
    ]
    return dataclasses.replace(dataset, scenes=scenes)


def shift_dataset_timestamps(dataset, view_id, offset_ns):
    """Return a dataset copy with one view's timestamps shifted."""
    if view_id not in _VIEW_IDS:
        raise ValueError('unsupported view_id: %s' % view_id)
    offset_ns = int(offset_ns)
    scenes = []
    for scene in dataset.scenes:
        frames = dict(scene.frames)
        frame = frames[view_id]
        frames[view_id] = dataclasses.replace(
            frame, timestamp_ns=frame.timestamp_ns + offset_ns)
        scenes.append(dataclasses.replace(scene, frames=frames))
    return dataclasses.replace(dataset, scenes=scenes)


def _support_points(source, target_frame, target_instance,
                    params):
    """Return shared-point support and visibility statistics.

    A source point counts only when it projects in front of (or on) the target
    surface, so a point hidden behind the target object is treated as no
    information rather than as negative evidence.
    """
    columns, rows, z = _project(
        _subsample(source.points_base, params.max_points),
        target_frame.camera)
    height, width = target_frame.label.shape
    inside = (
        (z > 1e-9) &
        (columns >= 0) & (columns <= width - 1) &
        (rows >= 0) & (rows <= height - 1)
    )
    if not np.any(inside):
        return 0.0, 0, 0, 0
    columns = np.rint(columns[inside]).astype(np.int64)
    rows = np.rint(rows[inside]).astype(np.int64)
    z = z[inside]
    target_depth = target_frame.depth_m[rows, columns]
    target_label = target_frame.label[rows, columns]
    visible = (
        (target_depth > 0) &
        (z <= target_depth + params.effective_depth_tolerance_m)
    )
    denominator = int(np.count_nonzero(visible))
    if denominator == 0:
        return 0.0, 0, 0, int(np.count_nonzero(inside))
    matches = int(np.count_nonzero(
        visible & (target_label == target_instance.mask_index)))
    return matches / float(denominator), matches, denominator, int(
        np.count_nonzero(inside))


def _chamfer(first, second, max_points):
    """Symmetric mean nearest-neighbour distance between two point sets."""
    first = _subsample(np.asarray(first, dtype=np.float64), max_points)
    second = _subsample(np.asarray(second, dtype=np.float64), max_points)
    diff = first[:, None, :] - second[None, :, :]
    distances = np.linalg.norm(diff, axis=2)
    return 0.5 * (float(distances.min(axis=1).mean()) +
                  float(distances.min(axis=0).mean()))


def _mahalanobis(first, second):
    difference = first.centroid_base - second.centroid_base
    covariance = first.covariance_base + second.covariance_base
    try:
        solved = np.linalg.solve(covariance, difference)
    except np.linalg.LinAlgError:
        solved = np.linalg.pinv(covariance) @ difference
    value = float(difference @ solved)
    return math.sqrt(max(value, 0.0))


def _aabb_gate(first, second, margin):
    return not (
        first.aabb_max[0] + margin < second.aabb_min[0] or
        second.aabb_max[0] + margin < first.aabb_min[0] or
        first.aabb_max[1] + margin < second.aabb_min[1] or
        second.aabb_max[1] + margin < first.aabb_min[1] or
        first.aabb_max[2] + margin < second.aabb_min[2] or
        second.aabb_max[2] + margin < first.aabb_min[2]
    )


def _candidate_evidence(first, second, first_frame, second_frame, params):
    """Compute all evidence for one ordered candidate pair."""
    size_floor = max(1e-6, 0.5 * (
        first.diagonal_m + second.diagonal_m))
    size_ratio = max(first.diagonal_m, second.diagonal_m) / max(
        1e-6, min(first.diagonal_m, second.diagonal_m))
    chamfer = _chamfer(first.points_base, second.points_base,
                       params.max_points)
    chamfer_ratio = chamfer / size_floor
    mahalanobis = _mahalanobis(first, second) / max(size_floor, 1e-6)
    evidence = {
        'size_ratio': round(float(size_ratio), 6),
        'chamfer_m': round(float(chamfer), 6),
        'chamfer_ratio': round(float(chamfer_ratio), 6),
        'depth_tolerance_m_effective': round(
            float(params.effective_depth_tolerance_m), 6),
        'centroid_distance_m': round(float(np.linalg.norm(
            first.centroid_base - second.centroid_base)), 6),
        'mahalanobis_ratio': round(float(mahalanobis), 6),
        'visible_pixels_first': int(first.visible_area_px),
        'visible_pixels_second': int(second.visible_area_px),
    }
    if params.strategy == 'centroid':
        evidence['support_first_to_second'] = None
        evidence['support_second_to_first'] = None
        evidence['symmetric_support'] = None
        gate_reason = None
        if size_ratio > params.size_ratio_max:
            gate_reason = 'size_gate'
        return evidence, gate_reason

    support_ab, matches_ab, denominator_ab, inside_ab = _support_points(
        first, second_frame, second, params)
    support_ba, matches_ba, denominator_ba, inside_ba = _support_points(
        second, first_frame, first, params)
    symmetric = min(support_ab, support_ba)
    evidence.update({
        'support_first_to_second': round(float(support_ab), 6),
        'support_second_to_first': round(float(support_ba), 6),
        'symmetric_support': round(float(symmetric), 6),
        'matches_first_to_second': int(matches_ab),
        'matches_second_to_first': int(matches_ba),
        'support_pixels_first_to_second': int(denominator_ab),
        'support_pixels_second_to_first': int(denominator_ba),
        'projected_inside_pixels_first_to_second': int(inside_ab),
        'projected_inside_pixels_second_to_first': int(inside_ba),
    })
    gate_reason = None
    if size_ratio > params.size_ratio_max:
        gate_reason = 'size_gate'
    elif chamfer_ratio > params.max_chamfer_ratio:
        gate_reason = 'chamfer_gate'
    elif denominator_ab < params.min_support_pixels or \
            denominator_ba < params.min_support_pixels:
        gate_reason = 'insufficient_visible'
    elif symmetric < params.min_mutual_support:
        gate_reason = 'low_mutual_support'
    return evidence, gate_reason


def _pair_cost(evidence, params):
    if params.strategy == 'centroid':
        return (
            params.weight_size * max(0.0, evidence['size_ratio'] - 1.0) +
            params.weight_prior * evidence['mahalanobis_ratio']
        )
    support = 1.0 - float(evidence['symmetric_support'])
    return (
        params.weight_support * support +
        params.weight_chamfer * evidence['chamfer_ratio'] +
        params.weight_size * max(0.0, evidence['size_ratio'] - 1.0) +
        params.weight_prior * evidence['mahalanobis_ratio']
    )


def _hungarian(cost):
    """Return a minimum-cost rectangular assignment (pure Python)."""
    cost = np.asarray(cost, dtype=np.float64)
    rows, columns = cost.shape
    if rows == 0 or columns == 0:
        return []
    size = max(rows, columns)
    padded = np.full((size, size), 1e9, dtype=np.float64)
    padded[:rows, :columns] = cost
    u = np.zeros(size + 1, dtype=np.float64)
    v = np.zeros(size + 1, dtype=np.float64)
    match = np.zeros(size + 1, dtype=np.int64)
    way = np.zeros(size + 1, dtype=np.int64)
    for row in range(1, size + 1):
        match[0] = row
        column0 = 0
        minimum = np.full(size + 1, np.inf, dtype=np.float64)
        used = np.zeros(size + 1, dtype=bool)
        while True:
            used[column0] = True
            row0 = match[column0]
            delta = np.inf
            column1 = 0
            for column in range(1, size + 1):
                if used[column]:
                    continue
                current = padded[row0 - 1, column - 1] - u[row0] - v[column]
                if current < minimum[column]:
                    minimum[column] = current
                    way[column] = column0
                if minimum[column] < delta:
                    delta = minimum[column]
                    column1 = column
            for column in range(size + 1):
                if used[column]:
                    u[match[column]] += delta
                    v[column] -= delta
                else:
                    minimum[column] -= delta
            column0 = column1
            if match[column0] == 0:
                break
        while True:
            column1 = way[column0]
            match[column0] = match[column1]
            column0 = column1
            if column0 == 0:
                break
    result = []
    for column in range(1, size + 1):
        row = int(match[column])
        if row <= rows and column <= columns:
            result.append((row - 1, column - 1))
    return result


def _singleton_reason(candidates, assigned_elsewhere, ambiguous_elsewhere):
    """Explain why one instance has no committed partner."""
    if not candidates:
        return 'no_candidate'
    if any(item['instance_id'] in ambiguous_elsewhere
           for item in candidates):
        return 'ambiguous_elsewhere'
    if any(item['instance_id'] in assigned_elsewhere
           for item in candidates):
        return 'matched_elsewhere'
    if all(item['gate_reason'] is not None for item in candidates):
        return str(candidates[0]['gate_reason'])
    return 'not_selected'


def associate_scene(scene, params=None, first_view='ext'):
    """Associate the two views of one scene; returns an association record."""
    params = params or AssociationParams()
    if first_view not in _VIEW_IDS:
        raise ValueError('unsupported first_view: %s' % first_view)
    second_view = 'wrist' if first_view == 'ext' else 'ext'
    first_frame = scene.frames[first_view]
    second_frame = scene.frames[second_view]
    first_instances = first_frame.instances
    second_instances = second_frame.instances
    if abs(first_frame.timestamp_ns - second_frame.timestamp_ns) > \
            params.sync_window_ns:
        return {
            'schema_version': ASSOCIATION_SCHEMA_VERSION,
            'scene_id': scene.scene_id,
            'first_view': first_view,
            'second_view': second_view,
            'params': params.to_dict(),
            'candidate_pairs': 0,
            'candidate_truncated': False,
            'clusters': [],
            'singletons': [
                {
                    'view_id': instance.view_id,
                    'instance_id': instance.instance_id,
                    'reason': 'temporal_mismatch',
                }
                for instance in list(first_instances) + list(second_instances)
            ],
        }
    if not first_instances or not second_instances:
        return {
            'schema_version': ASSOCIATION_SCHEMA_VERSION,
            'scene_id': scene.scene_id,
            'first_view': first_view,
            'second_view': second_view,
            'params': params.to_dict(),
            'candidate_pairs': 0,
            'candidate_truncated': False,
            'clusters': [],
            'singletons': [
                {
                    'view_id': instance.view_id,
                    'instance_id': instance.instance_id,
                    'reason': 'other_view_empty',
                }
                for instance in list(first_instances) + list(second_instances)
            ],
        }

    candidates = {first.instance_id: [] for first in first_instances}
    candidate_count = 0
    truncated_count = 0
    for first in first_instances:
        for second in second_instances:
            if not _aabb_gate(first, second, params.association_margin_m):
                continue
            evidence, gate_reason = _candidate_evidence(
                first, second, first_frame, second_frame, params)
            cost = _pair_cost(evidence, params)
            if gate_reason is not None:
                cost = 1e9
            candidates[first.instance_id].append({
                'instance_id': second.instance_id,
                'view_id': second.view_id,
                'mask_index': second.mask_index,
                'cost': round(float(cost), 9),
                'evidence': evidence,
                'gate_reason': gate_reason,
            })
            candidate_count += 1
    for first in first_instances:
        candidates[first.instance_id].sort(
            key=lambda item: (item['cost'], item['instance_id']))
        if len(candidates[first.instance_id]) > params.max_candidates:
            truncated_count += len(
                candidates[first.instance_id]) - params.max_candidates
            candidates[first.instance_id] = candidates[
                first.instance_id][:params.max_candidates]

    index_by_second = {
        second.instance_id: index
        for index, second in enumerate(second_instances)
    }
    cost_matrix = np.full(
        (len(first_instances), len(second_instances)), 1e9, dtype=np.float64)
    for row, first in enumerate(first_instances):
        for item in candidates[first.instance_id]:
            if item['gate_reason'] is not None:
                continue
            column = index_by_second[item['instance_id']]
            cost_matrix[row, column] = item['cost']
    assignment = _hungarian(cost_matrix)

    assigned_first = set()
    assigned_second = set()
    assigned_pair = {}
    cluster_records = []
    for row, column in assignment:
        if cost_matrix[row, column] >= 1e8:
            continue
        first = first_instances[row]
        second = second_instances[column]
        assigned_pair[first.instance_id] = second.instance_id
        assigned_first.add(first.instance_id)
        assigned_second.add(second.instance_id)

    for row, first in enumerate(first_instances):
        if first.instance_id not in assigned_first:
            continue
        second_id = assigned_pair[first.instance_id]
        item = next(
            candidate for candidate in candidates[first.instance_id]
            if candidate['instance_id'] == second_id)
        alternatives = [
            candidate for candidate in candidates[first.instance_id]
            if candidate['instance_id'] != second_id and
            candidate['gate_reason'] is None
        ]
        margin = None
        if alternatives:
            margin = float(alternatives[0]['cost'] - item['cost'])
        if margin is not None and margin < params.ambiguity_margin:
            cluster_records.append({
                'status': 'ambiguous',
                'global_id': None,
                'members': [
                    {'view_id': first.view_id,
                     'instance_id': first.instance_id},
                    {'view_id': second_frame.view_id,
                     'instance_id': second_id},
                ],
                'candidates': [
                    {
                        'instance_id': item['instance_id'],
                        'cost': item['cost'],
                        'evidence': item['evidence'],
                    },
                    {
                        'instance_id': alternatives[0]['instance_id'],
                        'cost': alternatives[0]['cost'],
                        'evidence': alternatives[0]['evidence'],
                    },
                ],
                'confidence': None,
                'confidence_kind': 'heuristic_uncalibrated',
                'unmatched_reason': None,
            })
        else:
            cluster_records.append({
                'status': 'associated',
                'global_id': 'assoc:%s:%s' % (
                    scene.scene_id, first.instance_id),
                'members': [
                    {'view_id': first.view_id,
                     'instance_id': first.instance_id},
                    {'view_id': second_frame.view_id,
                     'instance_id': second_id},
                ],
                'confidence': round(1.0 / (1.0 + float(item['cost'])), 6),
                'confidence_kind': 'heuristic_uncalibrated',
                'evidence': item['evidence'],
                'unmatched_reason': None,
            })

    ambiguous_first = {
        cluster['members'][0]['instance_id']
        for cluster in cluster_records
        if cluster['status'] == 'ambiguous'
    }
    ambiguous_second = {
        cluster['members'][1]['instance_id']
        for cluster in cluster_records
        if cluster['status'] == 'ambiguous'
    }
    singleton_records = []
    for first in first_instances:
        if first.instance_id in assigned_first:
            continue
        reason = _singleton_reason(
            candidates[first.instance_id],
            assigned_second, ambiguous_second)
        singleton_records.append({
            'view_id': first.view_id,
            'instance_id': first.instance_id,
            'reason': reason,
        })
    second_candidates = {}
    for second in second_instances:
        second_candidates[second.instance_id] = []
        for first in first_instances:
            for item in candidates[first.instance_id]:
                if item['instance_id'] == second.instance_id:
                    second_candidates[second.instance_id].append({
                        'instance_id': first.instance_id,
                        'gate_reason': item['gate_reason'],
                    })
    for second in second_instances:
        if second.instance_id in assigned_second:
            continue
        reason = _singleton_reason(
            second_candidates[second.instance_id],
            assigned_first, ambiguous_first)
        singleton_records.append({
            'view_id': second.view_id,
            'instance_id': second.instance_id,
            'reason': reason,
        })

    return {
        'schema_version': ASSOCIATION_SCHEMA_VERSION,
        'scene_id': scene.scene_id,
        'first_view': first_view,
        'second_view': second_view,
        'params': params.to_dict(),
        'candidate_pairs': candidate_count,
        'candidate_truncated': truncated_count > 0,
        'clusters': cluster_records,
        'singletons': singleton_records,
    }


def _pair_key(members):
    return frozenset(
        (str(member['view_id']), str(member['instance_id']))
        for member in members)


def _primitive_metrics(correct, predicted, truth):
    precision = correct / float(predicted) if predicted else 0.0
    recall = correct / float(truth) if truth else 0.0
    f1 = (2.0 * precision * recall / (precision + recall)
          if precision + recall > 0 else 0.0)
    return {
        'precision': round(precision, 6),
        'recall': round(recall, 6),
        'f1': round(f1, 6),
    }


def evaluate_association(dataset, params=None, first_view='ext'):
    """Run the matcher on every scene and score it against constructed pairs."""
    params = params or AssociationParams()
    records = []
    gt_count = 0
    predicted_count = 0
    correct_count = 0
    ambiguous_count = 0
    singleton_reasons = {}
    forced_errors = 0
    latency_ms = []
    per_density = {}
    per_occlusion = {}
    for scene in dataset.scenes:
        started = time.perf_counter()
        record = associate_scene(scene, params, first_view=first_view)
        latency_ms.append((time.perf_counter() - started) * 1000.0)
        records.append(record)
        truth = {
            _pair_key(pair['members'])
            for pair in dataset.pairs.get(scene.scene_id, [])
        }
        predicted = set()
        correct = 0
        for cluster in record['clusters']:
            members = cluster['members']
            if len(members) != 2:
                forced_errors += 1
                continue
            if cluster['status'] == 'ambiguous':
                ambiguous_count += 1
                continue
            key = _pair_key(members)
            if key in predicted:
                forced_errors += 1
            predicted.add(key)
            if key in truth:
                correct += 1
        for singleton in record['singletons']:
            reason = str(singleton['reason'])
            singleton_reasons[reason] = singleton_reasons.get(reason, 0) + 1
        gt_count += len(truth)
        predicted_count += len(predicted)
        correct_count += correct
        density = _density_bucket(scene)
        bucket = per_density.setdefault(density, {
            'scenes': 0, 'truth': 0, 'predicted': 0, 'correct': 0,
        })
        bucket['scenes'] += 1
        bucket['truth'] += len(truth)
        bucket['predicted'] += len(predicted)
        bucket['correct'] += correct
        occlusion = _occlusion_bucket(scene)
        occl_bucket = per_occlusion.setdefault(occlusion, {
            'scenes': 0, 'truth': 0, 'predicted': 0, 'correct': 0,
        })
        occl_bucket['scenes'] += 1
        occl_bucket['truth'] += len(truth)
        occl_bucket['predicted'] += len(predicted)
        occl_bucket['correct'] += correct
    overall = _primitive_metrics(correct_count, predicted_count, gt_count)
    density_metrics = {}
    for density, bucket in sorted(per_density.items()):
        density_metrics[density] = {
            'scenes': bucket['scenes'],
            'truth': bucket['truth'],
            'predicted': bucket['predicted'],
            'correct': bucket['correct'],
            **_primitive_metrics(
                bucket['correct'], bucket['predicted'], bucket['truth']),
        }
    occlusion_metrics = {}
    for occlusion, bucket in sorted(per_occlusion.items()):
        occlusion_metrics[occlusion] = {
            'scenes': bucket['scenes'],
            'truth': bucket['truth'],
            'predicted': bucket['predicted'],
            'correct': bucket['correct'],
            **_primitive_metrics(
                bucket['correct'], bucket['predicted'], bucket['truth']),
        }
    latencies = np.asarray(latency_ms, dtype=np.float64) if latency_ms else \
        np.zeros(1)
    return {
        'schema_version': ASSOCIATION_EVAL_SCHEMA_VERSION,
        'dataset': {
            'root': dataset.root,
            'dataset_id': dataset.manifest.get('dataset_id'),
            'scenes': len(dataset.scenes),
        },
        'params': params.to_dict(),
        'first_view': first_view,
        'results': {
            'truth_pairs': gt_count,
            'predicted_pairs': predicted_count,
            'correct_pairs': correct_count,
            **overall,
            'ambiguous_clusters': ambiguous_count,
            'forced_id_errors': forced_errors,
            'singleton_reason_counts': dict(sorted(
                singleton_reasons.items())),
            'per_density': density_metrics,
            'per_occlusion': occlusion_metrics,
            'latency_ms': {
                'p50': round(float(np.percentile(latencies, 50)), 3),
                'p95': round(float(np.percentile(latencies, 95)), 3),
                'max': round(float(latencies.max()), 3),
            },
        },
        'scenes': records,
        'hardware_claimed': False,
        'real_ore_validation': False,
    }


def _density_bucket(scene):
    count = int(scene.attributes.get('n_instances', 0))
    if count == 0:
        return 'empty'
    if count <= 8:
        return 'sparse'
    if count <= 14:
        return 'medium'
    return 'dense'


def _occlusion_bucket(scene):
    ratios = [
        instance.occlusion_ratio
        for frame in scene.frames.values()
        for instance in frame.instances
    ]
    if not ratios:
        return 'empty'
    typical = float(np.median(ratios))
    if typical < 0.15:
        return 'low'
    if typical < 0.40:
        return 'medium'
    return 'high'


_FORBIDDEN_FUSED_KEYS = {
    'true_centroid', 'true_center', 'full_extent', 'volume', 'mass',
    'material_composition', 'ore_grade', 'economic_value',
}


def _quaternion_from_axes(axes):
    """Return ``[x, y, z, w]`` for a rotation matrix whose columns are axes."""
    matrix = np.asarray(axes, dtype=np.float64)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1e-12:
        return [0.0, 0.0, 0.0, 1.0]
    return [round(float(value / norm), 9) for value in quaternion]


def _visible_obb(points):
    """Return a PCA oriented box for the visible point union."""
    points = np.asarray(points, dtype=np.float64)
    centroid = points.mean(axis=0)
    centered = points - centroid
    if points.shape[0] < 3:
        axes = np.eye(3)
    else:
        covariance = np.cov(centered.T, bias=True)
        values, vectors = np.linalg.eigh(covariance)
        order = np.argsort(values)[::-1]
        axes = vectors[:, order]
        if np.linalg.det(axes) < 0:
            axes[:, 2] *= -1.0
    projected = centered @ axes
    minimum = projected.min(axis=0)
    maximum = projected.max(axis=0)
    center = centroid + axes @ ((minimum + maximum) / 2.0)
    extent = maximum - minimum
    return {
        'center_base_m': [round(float(value), 6) for value in center],
        'quaternion_xyzw': _quaternion_from_axes(axes),
        'extent_m': [round(float(value), 6) for value in extent],
        'frame_id': 'base_link',
        'field_status': 'algorithm_estimate',
    }


def _fused_visible_geometry(scene, cluster):
    members = cluster['members']
    member_geometry = []
    centroids = []
    covariances = []
    point_sets = []
    for member in members:
        view_id = member['view_id']
        instance_id = member['instance_id']
        frame = scene.frames[view_id]
        instance = next(
            (item for item in frame.instances
             if item.instance_id == instance_id), None)
        if instance is None:
            raise ValueError('association member not found: %s/%s' % (
                view_id, instance_id))
        centroids.append(instance.centroid_base)
        covariances.append(instance.covariance_base)
        point_sets.append(instance.points_base)
        member_geometry.append({
            'view_id': view_id,
            'instance_id': instance_id,
            'visible_centroid_base_m': [
                round(float(value), 6)
                for value in instance.centroid_base],
            'visible_points': int(instance.points_base.shape[0]),
            'covariance_m2': [
                [round(float(value), 9) for value in row]
                for row in instance.covariance_base],
            'occlusion_ratio': round(float(instance.occlusion_ratio), 6),
        })
    information = np.zeros((3, 3), dtype=np.float64)
    weighted = np.zeros(3, dtype=np.float64)
    for centroid, covariance in zip(centroids, covariances):
        try:
            precision = np.linalg.inv(covariance)
        except np.linalg.LinAlgError:
            precision = np.linalg.pinv(covariance)
        information += precision
        weighted += precision @ centroid
    try:
        fused_covariance = np.linalg.inv(information)
    except np.linalg.LinAlgError:
        fused_covariance = np.linalg.pinv(information)
    fused_covariance = 0.5 * (fused_covariance + fused_covariance.T)
    fused_centroid = fused_covariance @ weighted
    points = np.vstack(point_sets)
    aabb_min = points.min(axis=0)
    aabb_max = points.max(axis=0)
    return {
        'frame_id': 'base_link',
        'method': 'information_weighted_visible_centroids',
        'fused_visible_centroid_base_m': [
            round(float(value), 6) for value in fused_centroid],
        'covariance_m2': [
            [round(float(value), 9) for value in row]
            for row in fused_covariance],
        'visible_aabb': {
            'min_base_m': [round(float(value), 6) for value in aabb_min],
            'max_base_m': [round(float(value), 6) for value in aabb_max],
            'frame_id': 'base_link',
            'field_status': 'algorithm_estimate',
        },
        'visible_obb': _visible_obb(points),
        'visible_points': int(points.shape[0]),
        'member_geometry': member_geometry,
        'field_status': 'algorithm_estimate',
        'unknowns': [
            'true_centroid', 'full_extent', 'volume', 'mass', 'ore_grade',
            'economic_value',
        ],
        'note': 'visible surfaces only; not a complete object or true centroid',
    }


def fuse_association(scene, record):
    """Attach visible-only fused geometry to associated clusters."""
    fused = copy.deepcopy(record)
    for cluster in fused.get('clusters', []):
        if cluster.get('status') == 'associated':
            cluster['fused_visible_geometry'] = _fused_visible_geometry(
                scene, cluster)
        else:
            cluster['fused_visible_geometry'] = None
    return fused


def _walk_forbidden_keys(value, path=''):
    if isinstance(value, dict):
        for key, child in value.items():
            child_path = '%s.%s' % (path, key) if path else str(key)
            if key in _FORBIDDEN_FUSED_KEYS:
                yield child_path
            for found in _walk_forbidden_keys(child, child_path):
                yield found
    elif isinstance(value, list):
        for index, child in enumerate(value):
            for found in _walk_forbidden_keys(
                    child, '%s[%d]' % (path, index)):
                yield found


def validate_association_record(record, scene=None, require_fused=False):
    """Validate a ``jcheng.scene_association.v1`` record; raise on failure."""
    if not isinstance(record, dict):
        raise ValueError('association record must be an object')
    if record.get('schema_version') != ASSOCIATION_SCHEMA_VERSION:
        raise ValueError('association schema must be %s' %
                         ASSOCIATION_SCHEMA_VERSION)
    if not record.get('scene_id'):
        raise ValueError('association record needs scene_id')
    first_view = record.get('first_view')
    second_view = record.get('second_view')
    if first_view not in _VIEW_IDS or second_view not in _VIEW_IDS or \
            first_view == second_view:
        raise ValueError('association views are invalid')
    seen_global_ids = set()
    for index, cluster in enumerate(record.get('clusters', [])):
        status = cluster.get('status')
        if status not in ('associated', 'ambiguous'):
            raise ValueError('cluster %d has invalid status' % index)
        members = cluster.get('members')
        if not isinstance(members, list) or len(members) != 2:
            raise ValueError('cluster %d must have two members' % index)
        views = {member.get('view_id') for member in members}
        if views != {first_view, second_view}:
            raise ValueError('cluster %d members must cover both views' % index)
        for member in members:
            if not member.get('instance_id'):
                raise ValueError('cluster %d has empty instance_id' % index)
            if scene is not None:
                frame = scene.frames.get(member['view_id'])
                if frame is None or not any(
                        item.instance_id == member['instance_id']
                        for item in frame.instances):
                    raise ValueError('cluster %d references unknown instance' %
                                     index)
        global_id = cluster.get('global_id')
        if status == 'associated':
            if not global_id:
                raise ValueError('associated cluster %d needs global_id' %
                                 index)
            if global_id in seen_global_ids:
                raise ValueError('duplicate global_id: %s' % global_id)
            seen_global_ids.add(global_id)
            if require_fused and not cluster.get('fused_visible_geometry'):
                raise ValueError('associated cluster %d is not fused' % index)
        else:
            if global_id is not None:
                raise ValueError('ambiguous cluster %d has global_id' % index)
            candidates = cluster.get('candidates') or []
            if len(candidates) < 2:
                raise ValueError('ambiguous cluster %d needs two candidates' %
                                 index)
        fused = cluster.get('fused_visible_geometry')
        if fused is not None:
            if status != 'associated':
                raise ValueError('only associated clusters may be fused')
            forbidden = sorted(set(_walk_forbidden_keys(fused)))
            if forbidden:
                raise ValueError('fused geometry has forbidden fields: %s' %
                                 ', '.join(forbidden))
            if fused.get('frame_id') != 'base_link':
                raise ValueError('fused geometry must be in base_link')
            if fused.get('field_status') != 'algorithm_estimate':
                raise ValueError('fused geometry must be an algorithm estimate')
            centroid = np.asarray(
                fused.get('fused_visible_centroid_base_m'), dtype=np.float64)
            if centroid.shape != (3,) or not np.all(np.isfinite(centroid)):
                raise ValueError('fused visible centroid is invalid')
            covariance = np.asarray(
                fused.get('covariance_m2'), dtype=np.float64)
            if covariance.shape != (3, 3) or \
                    not np.all(np.isfinite(covariance)):
                raise ValueError('fused covariance is invalid')
            if not np.allclose(covariance, covariance.T, atol=1e-9):
                raise ValueError('fused covariance is not symmetric')
            if float(np.linalg.eigvalsh(covariance).min()) < -1e-9:
                raise ValueError('fused covariance is not PSD')
            aabb = fused.get('visible_aabb') or {}
            aabb_min = np.asarray(aabb.get('min_base_m'), dtype=np.float64)
            aabb_max = np.asarray(aabb.get('max_base_m'), dtype=np.float64)
            if aabb_min.shape != (3,) or aabb_max.shape != (3,) or \
                    np.any(aabb_min > aabb_max):
                raise ValueError('fused visible AABB is invalid')
            if np.any(centroid < aabb_min - 1e-9) or \
                    np.any(centroid > aabb_max + 1e-9):
                raise ValueError('fused visible centroid is outside AABB')
            members_geometry = fused.get('member_geometry') or []
            if len(members_geometry) != 2:
                raise ValueError('fused geometry must retain two members')
            if int(fused.get('visible_points', 0)) <= 0:
                raise ValueError('fused geometry has no visible points')
            obb = fused.get('visible_obb') or {}
            obb_center = np.asarray(
                obb.get('center_base_m'), dtype=np.float64)
            obb_extent = np.asarray(obb.get('extent_m'), dtype=np.float64)
            obb_quaternion = np.asarray(
                obb.get('quaternion_xyzw'), dtype=np.float64)
            if obb_center.shape != (3,) or obb_extent.shape != (3,) or \
                    obb_quaternion.shape != (4,) or \
                    not np.all(np.isfinite(obb_center)) or \
                    not np.all(np.isfinite(obb_extent)) or \
                    not np.all(np.isfinite(obb_quaternion)):
                raise ValueError('fused visible OBB is invalid')
            if np.any(obb_extent < 0):
                raise ValueError('fused visible OBB extent is negative')
            if abs(float(np.linalg.norm(obb_quaternion)) - 1.0) > 1e-6:
                raise ValueError('fused visible OBB quaternion is not unit')
    for singleton in record.get('singletons', []):
        if not singleton.get('instance_id') or not singleton.get('reason'):
            raise ValueError('singleton needs instance_id and reason')
        if scene is not None:
            view_id = singleton.get('view_id')
            frame = scene.frames.get(view_id)
            if frame is None or not any(
                    item.instance_id == singleton['instance_id']
                    for item in frame.instances):
                raise ValueError('singleton references unknown instance')
    return True


def write_association_jsonl(path, records):
    """Write association records as UTF-8 JSON Lines."""
    directory = os.path.dirname(os.path.abspath(path))
    os.makedirs(directory, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        for record in records:
            handle.write(json.dumps(record, sort_keys=True))
            handle.write('\n')


def read_association_jsonl(path, validate=False):
    """Read association JSONL; optionally validate each record's structure."""
    records = []
    with open(path, 'r', encoding='utf-8') as handle:
        for line_number, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            record = json.loads(line)
            if validate:
                try:
                    validate_association_record(record)
                except ValueError as error:
                    raise ValueError('line %d: %s' % (line_number, error))
            records.append(record)
    return records
