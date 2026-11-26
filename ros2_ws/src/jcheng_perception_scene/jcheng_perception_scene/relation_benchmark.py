"""
Controlled synthetic relation benchmark for Scene State v2.

The ground-truth pairs are defined by construction (the placement gap), not by
running the relation detector. Evaluation uses the dataset's GT masks as
instances, so it measures the relation algorithm with oracle segmentation, not
end-to-end detection. It is still synthetic: no real-ore relation accuracy is
claimed.
"""

import dataclasses
import datetime
import json
import os
from typing import Dict, List

import cv2
import numpy as np

from .dataset import load_dataset
from .relations import RelationParams, build_relation_graph, compare_relation_sets
from .types import (
    DATASET_SCHEMA_VERSION,
    InstanceMask,
    SEMANTIC_LABEL,
    sha256_file,
)

BENCHMARK_SCHEMA_VERSION = 'jcheng.relation_benchmark.v1'
GENERATOR_NAME = 'jcheng_perception_scene.relation_benchmark'
GENERATOR_VERSION = '1.0.0'


@dataclasses.dataclass(frozen=True)
class BenchmarkSpec:
    """One synthetic two-object relation case."""

    relation_class: str
    first_x: int
    second_x: int
    y: int = 100
    size: int = 40


def _draw_case(image, mask, spec, scene_id):
    """Draw two rectangles; return manifest instances and truth pairs."""
    first_x0 = spec.first_x
    second_x0 = spec.second_x
    y0 = spec.y
    size = spec.size
    colour = (90, 110, 140)
    cv2.rectangle(image, (first_x0, y0),
                  (first_x0 + size - 1, y0 + size - 1), colour, -1)
    cv2.rectangle(image, (second_x0, y0),
                  (second_x0 + size - 1, y0 + size - 1),
                  (140, 120, 90), -1)
    mask[y0:y0 + size, first_x0:first_x0 + size] = 1
    mask[y0:y0 + size, second_x0:second_x0 + size] = 2
    instances = []
    for index, x0 in enumerate((first_x0, second_x0), start=1):
        instances.append({
            'instance_id': '%s:%04d' % (scene_id, index),
            'mask_index': index,
            'area_px': int(size * size),
            'bbox_xyxy': [x0, y0, x0 + size - 1, y0 + size - 1],
            'stratum': 'unlabelled',
        })
    truth = []
    if spec.relation_class in ('contact', 'near'):
        truth.append({
            'subject_id': instances[0]['instance_id'],
            'object_id': instances[1]['instance_id'],
            'predicate': 'contact',
        })
    return instances, truth


def generate_relation_benchmark(out_dir, per_class=4, seed=20261126,
                                created_at=None):
    """Write a controlled relation dataset plus its independent truth file.

    Classes:
      * ``contact``: two 40 px rectangles with a 0 px gap (adjacent);
      * ``near``: two rectangles with a 4 px gap (still within a 5 px search);
      * ``clear``: two rectangles with an 80 px gap (no edge).
    """
    per_class = max(0, int(per_class))
    os.makedirs(os.path.join(out_dir, 'images'), exist_ok=True)
    os.makedirs(os.path.join(out_dir, 'masks'), exist_ok=True)
    specs = {
        'contact': BenchmarkSpec('contact', 60, 100),
        'near': BenchmarkSpec('near', 60, 104),
        'clear': BenchmarkSpec('clear', 40, 160),
    }
    scenes = []
    truth = {'contact': [], 'near': [], 'clear': []}
    index = 0
    for relation_class, spec in specs.items():
        for _ in range(per_class):
            scene_id = 'rel-%d-%s-%03d' % (int(seed), relation_class, index)
            image = np.full((240, 240, 3), 28, dtype=np.uint8)
            image += np.random.default_rng(
                int(seed) + index).integers(
                    0, 8, (240, 240, 3), dtype=np.uint8)
            mask = np.zeros((240, 240), dtype=np.uint16)
            instances, pairs = _draw_case(image, mask, spec, scene_id)
            image_name = 'images/%s.png' % scene_id
            mask_name = 'masks/%s.png' % scene_id
            if not cv2.imwrite(os.path.join(out_dir, image_name), image):
                raise RuntimeError('failed to write %s' % image_name)
            if not cv2.imwrite(
                    os.path.join(out_dir, mask_name),
                    mask.astype(np.uint16)):
                raise RuntimeError('failed to write %s' % mask_name)
            scenes.append({
                'scene_id': scene_id,
                'image': image_name,
                'mask': mask_name,
                'width': 240,
                'height': 240,
                'timestamp_ns': 0,
                'frame_id': 'camera_color_optical_frame',
                'attributes': {
                    'relation_class': relation_class,
                    'source': 'synthetic',
                    'n_instances': len(instances),
                    'mask_sha256': sha256_file(
                        os.path.join(out_dir, mask_name)),
                },
                'instances': instances,
            })
            for pair in pairs:
                truth[relation_class].append({
                    'scene_id': scene_id,
                    **pair,
                })
            index += 1

    if created_at is None:
        created_at = datetime.datetime.now(
            datetime.timezone.utc).replace(microsecond=0).isoformat()
    manifest = {
        'schema_version': DATASET_SCHEMA_VERSION,
        'dataset_id': 'relation_benchmark_%d' % int(seed),
        'source': 'synthetic',
        'annotation_method': 'constructed_pair_truth',
        'license': 'CC0-1.0',
        'citation': None,
        'created_at': created_at,
        'generator': {
            'name': GENERATOR_NAME,
            'version': GENERATOR_VERSION,
            'seed': int(seed),
            'per_class': per_class,
        },
        'category_names': ['ore'],
        'scenes': scenes,
    }
    dataset_path = os.path.join(out_dir, 'dataset.json')
    with open(dataset_path, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle, indent=2, sort_keys=True)
        handle.write('\n')
    truth_document = {
        'schema_version': BENCHMARK_SCHEMA_VERSION,
        'dataset_id': manifest['dataset_id'],
        'classes': truth,
        'note': (
            'Truth pairs are defined by the construction gap, not by the '
            'relation detector; evaluation uses GT masks (oracle segmentation).'),
    }
    truth_path = os.path.join(out_dir, 'relations_truth.json')
    with open(truth_path, 'w', encoding='utf-8') as handle:
        json.dump(truth_document, handle, indent=2, sort_keys=True)
        handle.write('\n')
    return dataset_path, truth_path


def _gt_instances(scene):
    """Build InstanceMask objects from the dataset's GT label image."""
    label = cv2.imread(scene.mask_path, cv2.IMREAD_UNCHANGED)
    if label is None:
        raise ValueError('cannot read GT mask: %s' % scene.mask_path)
    instances = []
    for item in scene.instances:
        mask = label == int(item['mask_index'])
        if not np.any(mask):
            continue
        instances.append(InstanceMask(
            instance_id=str(item['instance_id']),
            mask=mask,
            semantic_label=SEMANTIC_LABEL,
            score=1.0,
        ))
    return instances


def evaluate_relation_benchmark(dataset, truth_path,
                                params=None):
    """Evaluate relation edges on GT masks against constructed pair truth."""
    params = params or RelationParams()
    with open(truth_path, 'r', encoding='utf-8') as handle:
        truth_document = json.load(handle)
    truth_by_scene = {}
    for relation_class, pairs in truth_document.get('classes', {}).items():
        for pair in pairs:
            truth_by_scene.setdefault(pair['scene_id'], []).append({
                'subject_id': pair['subject_id'],
                'object_id': pair['object_id'],
                'predicate': 'contact',
                'class': relation_class,
            })
    predicted_all = []
    truth_all = []
    per_scene = {}
    for scene in dataset.scenes:
        instances = _gt_instances(scene)
        edges = build_relation_graph(instances, params)
        predicted = [{
            'subject_id': edge.subject_id,
            'object_id': edge.object_id,
            'predicate': 'contact',
            'predicate_2d': edge.predicate,
        } for edge in edges]
        truth = truth_by_scene.get(scene.scene_id, [])
        predicted_all.extend(predicted)
        truth_all.extend(truth)
        per_scene[scene.scene_id] = compare_relation_sets(predicted, truth)
    result = compare_relation_sets(predicted_all, truth_all)
    result.update({
        'schema_version': 'jcheng.relation_evaluation.v1',
        'dataset_id': dataset.dataset_id,
        'data_source': dataset.source,
        'evaluated_scenes': len(dataset.scenes),
        'per_scene': per_scene,
        'params': {
            'adjacency_px': params.adjacency_px,
            'touching_min_px': params.touching_min_px,
            'max_instances': params.max_instances,
        },
        'hardware_validation': False,
        'real_ore_validation': False,
        'limitations': [
            'Synthetic constructed pairs; not real ore relations.',
            'Evaluation uses GT masks (oracle segmentation), not predicted masks.',
            'Contact collapses touching/near into one predicate.',
            'No real relation-pair labels or hardware validation.',
        ],
    })
    return result
