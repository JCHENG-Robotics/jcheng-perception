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
    max_chamfer_ratio: float = 1.0
    size_ratio_max: float = 2.5
    ambiguity_margin: float = 0.05
    max_points: int = 300
    max_candidates: int = 512
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
        if self.max_chamfer_ratio <= 0 or self.size_ratio_max < 1.0:
            raise ValueError('invalid chamfer/size limits')
        if not 0.0 <= self.ambiguity_margin <= 10.0:
            raise ValueError('ambiguity_margin out of range')
        if self.max_points < 8 or self.max_candidates < 1:
            raise ValueError('max_points/max_candidates too small')

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


def load_dual_view_dataset(root, max_points=1000):
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
    visible = (target_depth > 0) & (z <= target_depth + params.depth_tolerance_m)
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
