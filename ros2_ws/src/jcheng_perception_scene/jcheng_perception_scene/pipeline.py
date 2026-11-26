"""
Offline run loop: dataset scene -> backend -> scene state -> JSONL + masks.

The pipeline is deliberately small and explicit. It never fabricates a mask,
never treats a missing image as an empty scene, and records every failure as a
structured error in the output instead of silently dropping it.
"""

import datetime
import json
import os
import platform
import time
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from .backends.classical import build_backend
from .config import config_sha256
from .dataset import read_image, write_jsonl
from .scene import build_scene_state
from .types import (
    ERROR_BACKEND,
    ERROR_INPUT_IMAGE,
    SCENE_FIELD_STATUS,
    SCENE_SCHEMA_VERSION,
    SceneError,
    sha256_file,
)
from .visualize import render_scene


def _safe_name(value):
    """Return a filesystem-safe scene stem."""
    return ''.join(
        character if character.isalnum() or character in '._-' else '_'
        for character in str(value))


def _observation_source(dataset):
    return {
        'kind': dataset.source,
        'dataset_id': dataset.dataset_id,
        'annotation_method': dataset.annotation_method,
        'license': dataset.license,
        'dataset_manifest_sha256': sha256_file(dataset.manifest_path),
    }


def _failure_record(scene_id, width, height, errors, backend_info,
                    observation_source, timing):
    return {
        'schema_version': SCENE_SCHEMA_VERSION,
        'scene_id': str(scene_id),
        'timestamp_ns': 0,
        'frame_id': 'camera_color_optical_frame',
        'image': {
            'path': None,
            'sha256': None,
            'width': int(width),
            'height': int(height),
        },
        'observation_source': dict(observation_source),
        'backend': backend_info.to_dict(),
        'instances': [],
        'relations': [],
        'field_status': dict(SCENE_FIELD_STATUS),
        'validity': {
            'valid': False,
            'empty_scene': False,
            'missing_fields': [],
            'failure_reason': errors[0].code if errors else 'UNKNOWN',
            'unknowns': [],
        },
        'errors': [error.to_dict() for error in errors],
        'timing': {key: float(value) for key, value in timing.items()},
    }


def _write_label_image(path, instances):
    """Write one uint16 label image and verify the mask partition.

    Summing individual mask areas and comparing with the covered label-image
    area detects overlap in O(sum(mask)) rather than O(N^2).
    """
    if len(instances) > 65535:
        raise ValueError('too many instances for a uint16 label image')
    if not instances:
        return None, 0, 0
    height, width = instances[0].mask.shape
    label_image = np.zeros((height, width), dtype=np.uint16)
    total_area = 0
    for instance in instances:
        x0, y0, x1, y1 = instance.bbox_xyxy
        roi = label_image[y0:y1 + 1, x0:x1 + 1]
        roi[instance.mask[y0:y1 + 1, x0:x1 + 1]] = int(instance.mask_index)
        total_area += instance.area_px
    covered = int(np.count_nonzero(label_image))
    if total_area != covered:
        raise ValueError(
            'instance masks overlap: summed area %d != covered %d' % (
                total_area, covered))
    if not cv2.imwrite(path, label_image):
        raise RuntimeError('failed to write label image: %s' % path)
    return sha256_file(path), total_area, covered


def run_dataset(dataset, config, out_dir, backend=None, limit=None,
                write_overlays=True):
    """Run the configured backend over every scene in ``dataset``.

    Returns a summary dictionary. Per-scene failures are written to the output
    and reflected in ``summary['failed_scenes']``; they are not raised.
    """
    if backend is None:
        backend = build_backend(config['backend']['id'])
    if backend.info.backend_id != config['backend']['id']:
        raise ValueError('backend/config id mismatch')
    os.makedirs(os.path.join(out_dir, 'masks'), exist_ok=True)
    overlay_dir = os.path.join(out_dir, 'overlays')
    if write_overlays:
        os.makedirs(overlay_dir, exist_ok=True)

    started = time.perf_counter()
    scenes = list(dataset.scenes)
    if limit is not None:
        scenes = scenes[:max(0, int(limit))]
    records = []
    instance_records = []
    failures = []
    backend_info = backend.info
    source = _observation_source(dataset)
    config_digest = config_sha256(config)

    for scene in scenes:
        scene_started = time.perf_counter()
        image, image_error = read_image(scene.image_path)
        if image_error is not None:
            failure = _failure_record(
                scene.scene_id, scene.width, scene.height, [image_error],
                backend_info, source, {})
            records.append(failure)
            failures.append({
                'scene_id': scene.scene_id,
                'code': image_error.code,
            })
            continue
        height, width = image.shape[:2]
        if width != scene.width or height != scene.height:
            error = SceneError(
                code=ERROR_INPUT_IMAGE,
                message='image size %dx%d != manifest %dx%d' % (
                    width, height, scene.width, scene.height),
                stage='input',
                context={'scene_id': scene.scene_id},
            )
            records.append(_failure_record(
                scene.scene_id, scene.width, scene.height, [error],
                backend_info, source, {}))
            failures.append({
                'scene_id': scene.scene_id,
                'code': error.code,
            })
            continue

        segmentation_started = time.perf_counter()
        try:
            instances = backend.segment(image, config, scene.scene_id)
        except Exception as error:  # noqa: BLE001 - recorded, then continued
            wrapped = SceneError(
                code=ERROR_BACKEND,
                message='%s: %s' % (type(error).__name__, error),
                stage='segmentation',
                context={'scene_id': scene.scene_id,
                         'backend': backend_info.backend_id},
            )
            records.append(_failure_record(
                scene.scene_id, scene.width, scene.height, [wrapped],
                backend_info, source, {}))
            failures.append({
                'scene_id': scene.scene_id,
                'code': wrapped.code,
            })
            continue
        segmentation_ms = (
            time.perf_counter() - segmentation_started) * 1000.0

        # The label image is the single source of instance indices. Sorting by
        # score gives a deterministic, metric-friendly order.
        instances = sorted(instances, key=lambda item: (-item.score,
                                                         item.instance_id))
        for index, instance in enumerate(instances, start=1):
            instance.mask_index = index
        label_relative = os.path.join(
            'masks', _safe_name(scene.scene_id) + '.png')
        label_path = os.path.join(out_dir, label_relative)
        try:
            label_sha, _, _ = _write_label_image(label_path, instances)
        except (ValueError, RuntimeError) as error:
            wrapped = SceneError(
                code=ERROR_BACKEND,
                message=str(error),
                stage='export',
                context={'scene_id': scene.scene_id},
            )
            records.append(_failure_record(
                scene.scene_id, scene.width, scene.height, [wrapped],
                backend_info, source, {}))
            failures.append({
                'scene_id': scene.scene_id,
                'code': wrapped.code,
            })
            continue
        for instance in instances:
            instance.mask_path = label_relative
            instance.mask_sha256 = label_sha

        total_ms = (time.perf_counter() - scene_started) * 1000.0
        try:
            scene_state = build_scene_state(
                scene_id=scene.scene_id,
                timestamp_ns=scene.timestamp_ns,
                frame_id=config['scene']['frame_id'],
                image=image,
                image_path=scene.image_path,
                image_sha256=sha256_file(scene.image_path),
                instances=instances,
                config=config,
                backend_info=backend_info,
                observation_source=source,
                errors=[],
                timing={
                    'segmentation_ms': round(segmentation_ms, 4),
                    'total_ms': round(total_ms, 4),
                },
                disjoint_verified=True,
            )
        except ValueError as error:
            wrapped = SceneError(
                code=ERROR_BACKEND,
                message=str(error),
                stage='scene_assembly',
                context={'scene_id': scene.scene_id},
            )
            records.append(_failure_record(
                scene.scene_id, scene.width, scene.height, [wrapped],
                backend_info, source,
                {'segmentation_ms': round(segmentation_ms, 4)}))
            failures.append({
                'scene_id': scene.scene_id,
                'code': wrapped.code,
            })
            continue

        record = scene_state.to_dict()
        records.append(record)
        for instance in scene_state.instances:
            instance_records.append({
                'schema_version': 'jcheng.ore_instance.v1',
                'scene_id': scene.scene_id,
                'instance_id': instance.instance_id,
                'semantic_label': instance.semantic_label,
                'detection_confidence': instance.score,
                'confidence_kind': instance.confidence_kind,
                'bbox_xyxy': list(instance.bbox_xyxy),
                'mask_area_px': instance.area_px,
                'mask_index': instance.mask_index,
                'mask_path': instance.mask_path,
                'mask_sha256': instance.mask_sha256,
                'visible_centroid_px': list(instance.visible_centroid_px),
                'visible_extent_px': list(instance.visible_extent_px),
                'source': dataset.source,
                'attributes': dict(instance.attributes),
            })
        if write_overlays and config['output']['overlays']:
            overlay_path = os.path.join(
                overlay_dir, _safe_name(scene.scene_id) + '.png')
            try:
                render_scene(image, scene_state.instances, overlay_path)
            except RuntimeError as error:
                failures.append({
                    'scene_id': scene.scene_id,
                    'code': 'OVERLAY_WRITE_FAILED',
                    'message': str(error),
                })

    scenes_path = os.path.join(out_dir, 'scenes.jsonl')
    instances_path = os.path.join(out_dir, 'instances.jsonl')
    write_jsonl(scenes_path, records)
    write_jsonl(instances_path, instance_records)

    elapsed_s = time.perf_counter() - started
    summary = {
        'schema_version': 'jcheng.ore_run.v1',
        'run_id': '%s-%s' % (
            _safe_name(dataset.dataset_id),
            datetime.datetime.now(
                datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')),
        'created_at': datetime.datetime.now(
            datetime.timezone.utc).replace(microsecond=0).isoformat(),
        'dataset': {
            'dataset_id': dataset.dataset_id,
            'source': dataset.source,
            'annotation_method': dataset.annotation_method,
            'license': dataset.license,
            'manifest_sha256': sha256_file(dataset.manifest_path),
            'scenes': len(scenes),
        },
        'backend': backend_info.to_dict(),
        'config_sha256': config_digest,
        'scene_records': len(records),
        'instance_records': len(instance_records),
        'failed_scenes': failures,
        'failed_scene_count': len(failures),
        'empty_scenes': sum(
            1 for record in records
            if record.get('validity', {}).get('empty_scene')),
        'elapsed_s': round(elapsed_s, 4),
        'environment': {
            'python': platform.python_version(),
            'opencv': cv2.__version__,
            'numpy': np.__version__,
            'platform': platform.platform(),
        },
        'hardware_validation': False,
        'real_ore_validation': False,
        'outputs': {
            'scenes': scenes_path,
            'instances': instances_path,
            'masks': os.path.join(out_dir, 'masks'),
            'overlays': overlay_dir if write_overlays else None,
        },
    }
    summary_path = os.path.join(out_dir, 'summary.json')
    with open(summary_path, 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write('\n')
    summary['summary_path'] = summary_path
    return summary


def run_unlabelled(images_dir, config, out_dir, backend=None,
                   write_overlays=True, limit=None):
    """Run the backend on a directory of images without ground truth.

    This produces qualitative overlays and counts only. It cannot produce
    accuracy metrics and the summary says so explicitly.
    """
    if backend is None:
        backend = build_backend(config['backend']['id'])
    extensions = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')
    names = sorted(
        name for name in os.listdir(images_dir)
        if os.path.splitext(name)[1].lower() in extensions)
    if limit is not None:
        names = names[:max(0, int(limit))]
    if not names:
        raise ValueError('no images found in %s' % images_dir)
    os.makedirs(os.path.join(out_dir, 'masks'), exist_ok=True)
    overlay_dir = os.path.join(out_dir, 'overlays')
    if write_overlays:
        os.makedirs(overlay_dir, exist_ok=True)
    records = []
    instance_records = []
    failures = []
    backend_info = backend.info
    source = {
        'kind': 'unlabelled_real',
        'dataset_id': os.path.basename(os.path.abspath(images_dir)),
        'annotation_method': 'none',
        'license': 'unknown',
        'dataset_manifest_sha256': None,
    }
    started = time.perf_counter()
    for name in names:
        scene_id = os.path.splitext(name)[0]
        image_path = os.path.join(images_dir, name)
        image, error = read_image(image_path)
        if error is not None:
            failures.append({'scene_id': scene_id, 'code': error.code})
            records.append(_failure_record(
                scene_id, 0, 0, [error], backend_info, source, {}))
            continue
        height, width = image.shape[:2]
        segmentation_started = time.perf_counter()
        try:
            instances = backend.segment(image, config, scene_id)
        except Exception as raised:  # noqa: BLE001
            wrapped = SceneError(
                code=ERROR_BACKEND,
                message='%s: %s' % (type(raised).__name__, raised),
                stage='segmentation',
                context={'scene_id': scene_id},
            )
            records.append(_failure_record(
                scene_id, width, height, [wrapped], backend_info, source, {}))
            failures.append({'scene_id': scene_id, 'code': wrapped.code})
            continue
        segmentation_ms = (
            time.perf_counter() - segmentation_started) * 1000.0
        instances = sorted(instances, key=lambda item: (-item.score,
                                                         item.instance_id))
        for index, instance in enumerate(instances, start=1):
            instance.mask_index = index
        label_relative = os.path.join(
            'masks', _safe_name(scene_id) + '.png')
        try:
            label_sha, _, _ = _write_label_image(
                os.path.join(out_dir, label_relative), instances)
            for instance in instances:
                instance.mask_path = label_relative
                instance.mask_sha256 = label_sha
            state = build_scene_state(
                scene_id=scene_id,
                timestamp_ns=0,
                frame_id=config['scene']['frame_id'],
                image=image,
                image_path=image_path,
                image_sha256=sha256_file(image_path),
                instances=instances,
                config=config,
                backend_info=backend_info,
                observation_source=source,
                timing={'segmentation_ms': round(segmentation_ms, 4)},
                disjoint_verified=True,
            )
        except (ValueError, RuntimeError) as error:
            wrapped = SceneError(
                code=ERROR_BACKEND,
                message=str(error),
                stage='export',
                context={'scene_id': scene_id},
            )
            records.append(_failure_record(
                scene_id, width, height, [wrapped], backend_info, source,
                {'segmentation_ms': round(segmentation_ms, 4)}))
            failures.append({'scene_id': scene_id, 'code': wrapped.code})
            continue
        records.append(state.to_dict())
        for instance in state.instances:
            instance_records.append({
                'schema_version': 'jcheng.ore_instance.v1',
                'scene_id': scene_id,
                'instance_id': instance.instance_id,
                'detection_confidence': instance.score,
                'bbox_xyxy': list(instance.bbox_xyxy),
                'mask_area_px': instance.area_px,
                'mask_index': instance.mask_index,
                'mask_path': instance.mask_path,
                'source': 'unlabelled_real',
            })
        if write_overlays and config['output']['overlays']:
            render_scene(
                image, state.instances,
                os.path.join(overlay_dir, _safe_name(scene_id) + '.png'))

    scenes_path = os.path.join(out_dir, 'scenes.jsonl')
    instances_path = os.path.join(out_dir, 'instances.jsonl')
    write_jsonl(scenes_path, records)
    write_jsonl(instances_path, instance_records)
    summary = {
        'schema_version': 'jcheng.ore_run.v1',
        'run_id': 'unlabelled-%s' % datetime.datetime.now(
            datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'),
        'dataset': {
            'dataset_id': source['dataset_id'],
            'source': source['kind'],
            'annotation_method': source['annotation_method'],
            'license': source['license'],
            'scenes': len(records),
        },
        'backend': backend_info.to_dict(),
        'config_sha256': config_sha256(config),
        'scene_records': len(records),
        'instance_records': len(instance_records),
        'failed_scenes': failures,
        'failed_scene_count': len(failures),
        'empty_scenes': sum(
            1 for record in records
            if record.get('validity', {}).get('empty_scene')),
        'elapsed_s': round(time.perf_counter() - started, 4),
        'environment': {
            'python': platform.python_version(),
            'opencv': cv2.__version__,
            'numpy': np.__version__,
            'platform': platform.platform(),
        },
        'hardware_validation': False,
        'real_ore_validation': False,
        'accuracy_metrics': None,
        'note': (
            'Unlabelled images: counts and overlays are qualitative only. '
            'No precision, recall, AP or accuracy is computed.'),
        'outputs': {
            'scenes': scenes_path,
            'instances': instances_path,
            'masks': os.path.join(out_dir, 'masks'),
            'overlays': overlay_dir if write_overlays else None,
        },
    }
    summary_path = os.path.join(out_dir, 'summary.json')
    with open(summary_path, 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write('\n')
    return summary
