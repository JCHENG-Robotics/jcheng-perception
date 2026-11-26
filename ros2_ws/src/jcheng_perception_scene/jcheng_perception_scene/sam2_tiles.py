"""
Convert exported SAM 2.1 tile masks into versioned scene-state records.

This module is deliberately torch-free: the remote SAM 2.1 process writes one
``result.json`` plus binary masks per tile, and this converter only reads those
artifacts. It never imports SAM 2, opens a GPU or touches a device.

A tile-set configuration makes the scene-type assumptions explicit:

    schema_version: 1
    backend:
      id: sam2.1_hiera_tiny_tiled
      version: "1.0.0"
      score_field: predicted_iou
      min_area_px: 50
      min_new_fraction: 0.30
    tile_sets:
      default:
        root: /path/to/out_1008
        tile_size: 1008
        max_area_fraction: 0.05
      overrides:
        "40_2":
          root: /path/to/out_504_40_2
          tile_size: 504
          max_area_fraction: 0.10
"""

import dataclasses
import json
import os
import re
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

from .config import config_sha256, load_config
from .dataset import load_dataset, write_jsonl
from .metrics import evaluate
from .types import (
    SCENE_FIELD_STATUS,
    SCENE_SCHEMA_VERSION,
    UNKNOWNS,
    sha256_bytes,
    sha256_file,
)

TILES_CONFIG_SCHEMA_VERSION = 1
DEFAULT_BACKEND_ID = 'sam2.1_hiera_tiny_tiled'
DEFAULT_BACKEND_VERSION = '1.0.0'

_TILE_RE = re.compile(r'^(?P<image>.+)_x(?P<x>\d+)_y(?P<y>\d+)$')
_TOP_KEYS = {'schema_version', 'backend', 'tile_sets'}
_BACKEND_KEYS = {
    'id', 'version', 'score_field', 'min_area_px', 'min_new_fraction',
    'mask_threshold',
}
_TILE_SET_KEYS = {'root', 'tile_size', 'max_area_fraction'}
_TILE_SETS_KEYS = {'default', 'overrides'}


def _require_keys(mapping, allowed, name):
    unknown = sorted(set(mapping) - set(allowed))
    if unknown:
        raise ValueError('%s has unknown keys: %s' %
                         (name, ', '.join(unknown)))


def _positive_int(value, name):
    if isinstance(value, bool) or int(value) != value or int(value) <= 0:
        raise ValueError('%s must be a positive integer' % name)
    return int(value)


def _fraction(value, name, allow_zero=False):
    number = float(value)
    if not np.isfinite(number):
        raise ValueError('%s must be finite' % name)
    lower_ok = number >= 0.0 if allow_zero else number > 0.0
    if not lower_ok or number > 1.0:
        raise ValueError('%s must be in %s[0, 1]' % (
            name, '[0, 1]' if allow_zero else '(0, 1]'))
    return number


@dataclasses.dataclass(frozen=True)
class TileSet:
    """One tile grid: where the masks are and how they are filtered."""

    root: str
    tile_size: int
    max_area_fraction: float = 0.05

    def __post_init__(self):
        """Validate the tile set."""
        if not isinstance(self.root, str) or not self.root:
            raise ValueError('tile set root must be a non-empty string')
        _positive_int(self.tile_size, 'tile_size')
        _fraction(self.max_area_fraction, 'max_area_fraction')


@dataclasses.dataclass(frozen=True)
class TilesConfig:
    """Tile sets plus merge/filter policy."""

    default: TileSet
    overrides: Dict[str, TileSet]
    backend_id: str = DEFAULT_BACKEND_ID
    backend_version: str = DEFAULT_BACKEND_VERSION
    score_field: str = 'predicted_iou'
    min_area_px: int = 50
    min_new_fraction: float = 0.30
    mask_threshold: int = 127

    def for_image(self, image_stem):
        """Return the override tile set or the default."""
        return self.overrides.get(image_stem, self.default)


def _parse_tile_set(value, name):
    if not isinstance(value, dict):
        raise ValueError('%s must be a mapping' % name)
    _require_keys(value, _TILE_SET_KEYS, name)
    for key in ('root', 'tile_size'):
        if key not in value:
            raise ValueError('%s missing %s' % (name, key))
    return TileSet(
        root=str(value['root']),
        tile_size=int(value['tile_size']),
        max_area_fraction=float(value.get('max_area_fraction', 0.05)),
    )


def load_tiles_config(path):
    """Load and validate a SAM2 tiles YAML/JSON configuration."""
    with open(path, 'r', encoding='utf-8') as handle:
        loaded = yaml.safe_load(handle)
    if not isinstance(loaded, dict):
        raise ValueError('tiles configuration root must be a mapping')
    _require_keys(loaded, _TOP_KEYS, 'tiles configuration')
    if int(loaded.get('schema_version', 0)) != TILES_CONFIG_SCHEMA_VERSION:
        raise ValueError('tiles schema_version must be %d' %
                         TILES_CONFIG_SCHEMA_VERSION)

    backend = loaded.get('backend', {})
    if not isinstance(backend, dict):
        raise ValueError('backend must be a mapping')
    _require_keys(backend, _BACKEND_KEYS, 'backend')
    score_field = str(backend.get('score_field', 'predicted_iou'))
    if not score_field:
        raise ValueError('backend.score_field must be non-empty')
    min_area = _positive_int(backend.get('min_area_px', 50), 'min_area_px')
    min_new = _fraction(backend.get('min_new_fraction', 0.30),
                        'min_new_fraction', allow_zero=True)
    threshold = _positive_int(backend.get('mask_threshold', 127),
                              'mask_threshold')
    if threshold > 255:
        raise ValueError('mask_threshold must be <= 255')

    tile_sets = loaded.get('tile_sets')
    if not isinstance(tile_sets, dict):
        raise ValueError('tile_sets must be a mapping')
    _require_keys(tile_sets, _TILE_SETS_KEYS, 'tile_sets')
    if 'default' not in tile_sets:
        raise ValueError('tile_sets.default is required')
    default = _parse_tile_set(tile_sets['default'], 'tile_sets.default')
    overrides = {}
    raw_overrides = tile_sets.get('overrides', {})
    if not isinstance(raw_overrides, dict):
        raise ValueError('tile_sets.overrides must be a mapping')
    for key, value in raw_overrides.items():
        overrides[str(key)] = _parse_tile_set(
            value, 'tile_sets.overrides.%s' % key)

    return TilesConfig(
        default=default,
        overrides=overrides,
        backend_id=str(backend.get('id', DEFAULT_BACKEND_ID)),
        backend_version=str(
            backend.get('version', DEFAULT_BACKEND_VERSION)),
        score_field=score_field,
        min_area_px=min_area,
        min_new_fraction=min_new,
        mask_threshold=threshold,
    )


def tiles_config_sha256(config):
    """Return a stable hash of the effective tile-set policy."""
    payload = {
        'backend_id': config.backend_id,
        'backend_version': config.backend_version,
        'score_field': config.score_field,
        'min_area_px': config.min_area_px,
        'min_new_fraction': config.min_new_fraction,
        'mask_threshold': config.mask_threshold,
        'default': {
            'root': config.default.root,
            'tile_size': config.default.tile_size,
            'max_area_fraction': config.default.max_area_fraction,
        },
        'overrides': {
            key: {
                'root': value.root,
                'tile_size': value.tile_size,
                'max_area_fraction': value.max_area_fraction,
            }
            for key, value in sorted(config.overrides.items())
        },
    }
    return sha256_bytes(
        json.dumps(payload, sort_keys=True, separators=(',', ':')).encode())


def _load_tile_items(tile_set, image_stem, config):
    """Load all tile masks for one image, sorted by score descending."""
    if not os.path.isdir(tile_set.root):
        raise ValueError('tile root does not exist: %s' % tile_set.root)
    items = []
    wall_seconds = 0.0
    found = 0
    for name in sorted(os.listdir(tile_set.root)):
        match = _TILE_RE.match(name)
        if match is None or match.group('image') != image_stem:
            continue
        found += 1
        tile_root = os.path.join(tile_set.root, name)
        result_path = os.path.join(tile_root, 'result.json')
        if not os.path.isfile(result_path):
            raise ValueError('missing result.json: %s' % result_path)
        with open(result_path, 'r', encoding='utf-8') as handle:
            result = json.load(handle)
        wall_seconds += float(result.get('wall_seconds', 0.0))
        masks = result.get('masks')
        if not isinstance(masks, list):
            raise ValueError('result.json has no masks list: %s' %
                             result_path)
        x = int(match.group('x'))
        y = int(match.group('y'))
        for item in masks:
            if not isinstance(item, dict):
                raise ValueError('invalid mask entry in %s' % result_path)
            mask_name = item.get('mask')
            if not isinstance(mask_name, str) or not mask_name:
                raise ValueError('mask entry without file in %s' %
                                 result_path)
            mask_path = os.path.join(tile_root, 'masks', mask_name)
            mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
            if mask is None:
                raise ValueError('cannot read tile mask: %s' % mask_path)
            mask = mask > int(config.mask_threshold)
            area = int(np.count_nonzero(mask))
            if area < int(config.min_area_px):
                continue
            if config.score_field not in item:
                raise ValueError('mask entry missing %s in %s' % (
                    config.score_field, result_path))
            score = float(item[config.score_field])
            if not np.isfinite(score):
                raise ValueError('non-finite score in %s' % result_path)
            stability = float(item.get('stability_score', 0.0))
            items.append((score, x, y, mask, area, stability, name))
    if found == 0:
        raise ValueError('no tiles for image %s under %s' % (
            image_stem, tile_set.root))
    items.sort(key=lambda value: -value[0])
    return items, wall_seconds, found


def merge_tile_set(tile_set, image_stem, width, height, config):
    """Merge one image's tile masks into a full-resolution label map.

    Returns ``(label_image, instances, stats)``. ``instances`` are metadata
    dicts with inclusive bounding boxes and a stable scene-local order.
    """
    items, wall_seconds, tile_count = _load_tile_items(
        tile_set, image_stem, config)
    label = np.zeros((int(height), int(width)), dtype=np.uint16)
    assigned = np.zeros((int(height), int(width)), dtype=bool)
    instances = []
    max_area = float(tile_set.max_area_fraction) * \
        int(tile_set.tile_size) * int(tile_set.tile_size)
    index = 1
    skipped_area = 0
    skipped_overlap = 0
    for score, x, y, mask, area, stability, tile_name in items:
        if area > max_area:
            skipped_area += 1
            continue
        x1 = min(int(width), x + int(tile_set.tile_size))
        y1 = min(int(height), y + int(tile_set.tile_size))
        if x1 <= x or y1 <= y:
            skipped_area += 1
            continue
        local = mask[:y1 - y, :x1 - x]
        label_roi = label[y:y1, x:x1]
        assigned_roi = assigned[y:y1, x:x1]
        new = local & ~assigned_roi
        new_area = int(np.count_nonzero(new))
        if new_area < float(config.min_new_fraction) * area:
            skipped_overlap += 1
            continue
        label_roi[new] = index
        assigned_roi |= new
        rows, columns = np.nonzero(new)
        instances.append({
            'instance_id': '%s:%04d' % (image_stem, index),
            'detection_confidence': score,
            'confidence_kind': 'sam2_predicted_iou_uncalibrated',
            'bbox_xyxy': [
                int(columns.min() + x), int(rows.min() + y),
                int(columns.max() + x), int(rows.max() + y),
            ],
            'mask_index': index,
            'mask_area_px': new_area,
            'visible_centroid_px': [
                float(columns.mean() + x), float(rows.mean() + y),
            ],
            'stability_score': stability,
            'tile': tile_name,
            'tile_size': int(tile_set.tile_size),
        })
        index += 1
    stats = {
        'tiles': tile_count,
        'raw_masks': len(items),
        'kept_instances': len(instances),
        'skipped_area': skipped_area,
        'skipped_overlap': skipped_overlap,
        'wall_seconds': round(wall_seconds, 4),
        'tile_size': int(tile_set.tile_size),
    }
    return label, instances, stats


def _scene_record(scene, instances, label_relative, label_sha, dataset,
                  tiles_config, stats, frame_id, semantic_label):
    """Build one schema-conforming scene-state record."""
    observation_source = {
        'kind': dataset.source,
        'dataset_id': dataset.dataset_id,
        'annotation_method': dataset.annotation_method,
        'license': dataset.license,
        'dataset_manifest_sha256': sha256_file(dataset.manifest_path),
    }
    backend = {
        'id': tiles_config.backend_id,
        'version': tiles_config.backend_version,
        'family': 'promptable_sam2_tiled',
        'calibrated': False,
        'description': 'SAM 2.1 automatic masks merged from exported tiles',
    }
    field_status = dict(SCENE_FIELD_STATUS)
    field_status['adjacency'] = 'not_computed_in_s1'
    instance_records = []
    for item in instances:
        x0, y0, x1, y1 = item['bbox_xyxy']
        instance_records.append({
            'instance_id': item['instance_id'],
            'semantic_label': semantic_label,
            'detection_confidence': item['detection_confidence'],
            'confidence_kind': item['confidence_kind'],
            'bbox_xyxy': list(item['bbox_xyxy']),
            'mask_path': label_relative,
            'mask_sha256': label_sha,
            'mask_index': item['mask_index'],
            'mask_area_px': item['mask_area_px'],
            'visible_centroid_px': item['visible_centroid_px'],
            'visible_extent_px': [x1 - x0 + 1, y1 - y0 + 1],
            'geometry': {
                'status': 'not_computed_in_s1',
                'reason': 'rgb_only_pipeline',
            },
            'attributes': {
                'tile': item['tile'],
                'tile_size': item['tile_size'],
                'stability_score': item['stability_score'],
            },
        })
    return {
        'schema_version': SCENE_SCHEMA_VERSION,
        'scene_id': scene.scene_id,
        'timestamp_ns': int(scene.timestamp_ns),
        'frame_id': frame_id,
        'image': {
            'path': scene.image_path,
            'sha256': sha256_file(scene.image_path),
            'width': int(scene.width),
            'height': int(scene.height),
        },
        'observation_source': observation_source,
        'backend': backend,
        'instances': instance_records,
        'relations': [],
        'field_status': field_status,
        'validity': {
            'valid': True,
            'empty_scene': len(instance_records) == 0,
            'missing_fields': [],
            'failure_reason': None,
            'unknowns': list(UNKNOWNS),
        },
        'errors': [],
        'timing': {
            'segmentation_ms': float(stats['wall_seconds']) * 1000.0,
        },
    }


def run_sam2_tiles(dataset, tiles_config, out_dir, scene_config=None,
                   limit=None, write_metrics=False):
    """Convert all dataset scenes using exported SAM2 tile masks."""
    scene_config = scene_config or load_config(None)
    os.makedirs(os.path.join(out_dir, 'masks'), exist_ok=True)
    scenes = list(dataset.scenes)
    if limit is not None:
        scenes = scenes[:max(0, int(limit))]
    records = []
    summaries = []
    for scene in scenes:
        image_stem = os.path.splitext(
            os.path.basename(scene.image_path))[0]
        tile_set = tiles_config.for_image(image_stem)
        label, instances, stats = merge_tile_set(
            tile_set, image_stem, scene.width, scene.height, tiles_config)
        label_name = scene.scene_id + '.png'
        label_path = os.path.join(out_dir, 'masks', label_name)
        if not cv2.imwrite(label_path, label):
            raise RuntimeError('failed to write label image: %s' % label_path)
        label_relative = os.path.join('masks', label_name)
        record = _scene_record(
            scene, instances, label_relative, sha256_file(label_path),
            dataset, tiles_config, stats, scene_config['scene']['frame_id'],
            scene_config['scene']['semantic_label'])
        records.append(record)
        summaries.append({
            'scene_id': scene.scene_id,
            'image_stem': image_stem,
            'tile_root': tile_set.root,
            **stats,
        })
        print('%s kept=%d skipped_overlap=%d skipped_area=%d tiles=%d' % (
            image_stem, stats['kept_instances'], stats['skipped_overlap'],
            stats['skipped_area'], stats['tiles']), flush=True)

    scenes_path = os.path.join(out_dir, 'scenes.jsonl')
    instances_path = os.path.join(out_dir, 'instances.jsonl')
    write_jsonl(scenes_path, records)
    instance_rows = []
    for record in records:
        for item in record['instances']:
            row = dict(item)
            row['scene_id'] = record['scene_id']
            row['source'] = dataset.source
            instance_rows.append(row)
    write_jsonl(instances_path, instance_rows)

    summary = {
        'schema_version': 'jcheng.sam2_tile_run.v1',
        'dataset_id': dataset.dataset_id,
        'dataset_source': dataset.source,
        'dataset_license': dataset.license,
        'tile_config_sha256': tiles_config_sha256(tiles_config),
        'backend': {
            'id': tiles_config.backend_id,
            'version': tiles_config.backend_version,
            'score_field': tiles_config.score_field,
        },
        'scenes': summaries,
        'scene_records': len(records),
        'instance_records': len(instance_rows),
        'hardware_validation': False,
        'real_ore_validation': False,
        'note': 'Offline conversion only; masks come from exported SAM2 tiles.',
    }
    summary_path = os.path.join(out_dir, 'summary.json')
    with open(summary_path, 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write('\n')
    if write_metrics:
        metrics = evaluate(dataset, records, out_dir, scene_config)
        with open(os.path.join(out_dir, 'metrics.json'), 'w',
                  encoding='utf-8') as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)
            handle.write('\n')
    return summary
