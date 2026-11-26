"""Offline ore scene understanding tests.

These tests use synthetic data only. They prove that the pipeline, scene
schema, failure paths and metrics behave as specified; they do not prove any
real-ore accuracy.
"""

import json
import os
import shutil

import cv2
import numpy as np
import pytest

from jcheng_perception_scene.backends.classical import (
    ClassicalWatershedBackend,
    build_backend,
)
from jcheng_perception_scene.cli import main
from jcheng_perception_scene.config import load_config, validate_config
from jcheng_perception_scene.dataset import (
    import_coco_dataset,
    load_dataset,
    load_ground_truth,
    read_predictions,
    validate_dataset_files,
)
from jcheng_perception_scene.metrics import evaluate
from jcheng_perception_scene.pipeline import run_dataset, run_unlabelled
from jcheng_perception_scene.scene import build_scene_state
from jcheng_perception_scene.synthetic import generate_dataset
from jcheng_perception_scene.types import (
    ERROR_MASK_OVERLAP,
    InstanceMask,
    SEMANTIC_LABEL,
    sha256_file,
)


CREATED_AT = '2026-11-26T00:00:00+00:00'


@pytest.fixture(scope='module')
def dataset_root(tmp_path_factory):
    root = tmp_path_factory.mktemp('ore-synthetic')
    generate_dataset(
        str(root), scene_count=6, empty_count=2, seed=20261126,
        width=320, height=240, created_at=CREATED_AT)
    return str(root)


@pytest.fixture(scope='module')
def dataset(dataset_root):
    return load_dataset(dataset_root)


def _write_perfect_predictions(dataset, out_dir):
    """Copy ground-truth masks as perfect predictions."""
    os.makedirs(os.path.join(out_dir, 'masks'), exist_ok=True)
    records = []
    for scene in dataset.scenes:
        label = cv2.imread(scene.mask_path, cv2.IMREAD_UNCHANGED)
        relative = os.path.join('masks', scene.scene_id + '.png')
        destination = os.path.join(out_dir, relative)
        cv2.imwrite(destination, label)
        instances = []
        for instance in scene.instances:
            instances.append({
                'instance_id': instance['instance_id'],
                'detection_confidence': 0.9,
                'mask_index': instance['mask_index'],
                'mask_path': relative,
                'bbox_xyxy': instance['bbox_xyxy'],
                'mask_area_px': instance['area_px'],
            })
        records.append({
            'schema_version': 'jcheng.scene_state.v1',
            'scene_id': scene.scene_id,
            'image': {'path': scene.image_path, 'width': scene.width,
                      'height': scene.height},
            'instances': instances,
            'relations': [],
            'validity': {
                'valid': True,
                'empty_scene': len(instances) == 0,
                'missing_fields': [],
                'failure_reason': None,
                'unknowns': [],
            },
            'timing': {'segmentation_ms': 1.0},
        })
    path = os.path.join(out_dir, 'scenes.jsonl')
    with open(path, 'w', encoding='utf-8') as handle:
        for record in records:
            handle.write(json.dumps(record) + '\n')
    return path


def test_synthetic_generation_is_deterministic(tmp_path):
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    generate_dataset(str(first), scene_count=3, empty_count=1, seed=7,
                     width=160, height=120, created_at=CREATED_AT)
    generate_dataset(str(second), scene_count=3, empty_count=1, seed=7,
                     width=160, height=120, created_at=CREATED_AT)
    with open(first / 'dataset.json', encoding='utf-8') as handle:
        first_manifest = json.load(handle)
    with open(second / 'dataset.json', encoding='utf-8') as handle:
        second_manifest = json.load(handle)
    assert first_manifest == second_manifest
    for scene in first_manifest['scenes']:
        assert sha256_file(str(first / scene['mask'])) == \
            sha256_file(str(second / scene['mask']))


def test_manifest_provenance_and_hashes(dataset):
    assert dataset.source == 'synthetic'
    assert dataset.annotation_method == 'procedural_ground_truth'
    assert dataset.license == 'CC0-1.0'
    assert dataset.manifest['generator']['name'] == (
        'jcheng_perception_scene.synthetic')
    assert validate_dataset_files(dataset) == []
    for scene in dataset.scenes:
        assert scene.attributes['source'] == 'synthetic'
        assert scene.attributes['mask_sha256']
        assert scene.attributes['seed'] >= 0


def test_empty_scenes_produce_zero_instances(dataset, tmp_path):
    summary = run_dataset(
        dataset, load_config(), str(tmp_path / 'run'), write_overlays=False)
    assert summary['failed_scene_count'] == 0
    records = read_predictions(str(tmp_path / 'run' / 'scenes.jsonl'))
    empty = [record for record in records
             if record['validity']['empty_scene']]
    assert len(empty) == 2
    for record in empty:
        assert record['instances'] == []
        assert record['validity']['valid'] is True
        assert record['relations'] == []


def test_pipeline_and_metrics_smoke(dataset, tmp_path):
    out = str(tmp_path / 'run')
    summary = run_dataset(dataset, load_config(), out, write_overlays=True)
    assert summary['failed_scene_count'] == 0
    assert summary['scene_records'] == 8
    records = read_predictions(os.path.join(out, 'scenes.jsonl'))
    assert len(records) == 8
    metrics = evaluate(dataset, records, out, load_config())
    assert metrics['schema_version'] == 'jcheng.ore_evaluation.v1'
    assert metrics['scenes'] == 8
    assert metrics['gt_instances'] > 0
    assert metrics['mask_ap50'] is not None
    assert 0.0 <= metrics['mask_ap50'] <= 1.0
    assert metrics['latency_ms']['p50'] is not None
    assert metrics['hardware_validation'] is False
    assert metrics['real_ore_validation'] is False
    assert metrics['data_source'] == 'synthetic'


def test_perfect_predictions_reach_ap_one(dataset, tmp_path):
    out = str(tmp_path / 'perfect')
    path = _write_perfect_predictions(dataset, out)
    metrics = evaluate(dataset, read_predictions(path), out, load_config())
    assert metrics['mask_ap50'] == pytest.approx(1.0, abs=1e-9)
    assert metrics['mask_ap50_95'] == pytest.approx(1.0, abs=1e-9)
    assert metrics['instance_count_mae'] == pytest.approx(0.0)
    assert metrics['precision_at_operating_point'] == pytest.approx(1.0)
    assert metrics['recall_at_operating_point'] == pytest.approx(1.0)


def test_overlapping_masks_are_rejected():
    first = InstanceMask(
        instance_id='scene:0001',
        mask=np.ones((10, 10), dtype=bool),
        semantic_label=SEMANTIC_LABEL,
        score=0.5)
    second = InstanceMask(
        instance_id='scene:0002',
        mask=np.ones((10, 10), dtype=bool),
        semantic_label=SEMANTIC_LABEL,
        score=0.5)
    with pytest.raises(ValueError) as error:
        build_scene_state(
            'scene', 0, 'camera_color_optical_frame',
            np.zeros((10, 10, 3), dtype=np.uint8), None, None,
            [first, second], load_config(),
            ClassicalWatershedBackend.info, {})
    assert ERROR_MASK_OVERLAP in str(error.value)


def test_adjacency_relation_has_evidence():
    image = np.zeros((32, 32, 3), dtype=np.uint8)
    first = np.zeros((32, 32), dtype=bool)
    second = np.zeros((32, 32), dtype=bool)
    first[8:16, 4:12] = True
    second[8:16, 13:21] = True
    scene = build_scene_state(
        'adjacent', 0, 'camera_color_optical_frame', image, None, None,
        [
            InstanceMask('adjacent:0001', first, SEMANTIC_LABEL, 0.8),
            InstanceMask('adjacent:0002', second, SEMANTIC_LABEL, 0.7),
        ],
        load_config(), ClassicalWatershedBackend.info, {})
    assert len(scene.relations) == 1
    relation = scene.relations[0]
    assert relation.predicate == 'adjacent'
    assert relation.evidence['contact_px'] > 0
    assert relation.evidence['depth_order'] == 'unknown'


def test_bad_image_produces_structured_failure(tmp_path):
    bad_dir = tmp_path / 'bad'
    bad_dir.mkdir()
    with open(bad_dir / 'broken.png', 'wb') as handle:
        handle.write(b'\x89PNG\r\n\x1a\n')
    summary = run_unlabelled(
        str(bad_dir), load_config(), str(tmp_path / 'bad-run'),
        write_overlays=False)
    assert summary['failed_scene_count'] == 1
    assert summary['accuracy_metrics'] is None
    records = read_predictions(
        str(tmp_path / 'bad-run' / 'scenes.jsonl'))
    assert records[0]['validity']['valid'] is False
    assert records[0]['validity']['failure_reason'] == (
        'INPUT_IMAGE_UNREADABLE')


def test_missing_dataset_is_an_input_error(tmp_path):
    code = main([
        'run', '--dataset', str(tmp_path / 'missing'),
        '--out', str(tmp_path / 'out'),
    ])
    assert code == 2


def test_config_rejects_unknown_keys():
    config = load_config()
    config['backend']['foreground']['typo_threshold'] = 1.0
    with pytest.raises(ValueError):
        validate_config(config)


def test_config_rejects_unsupported_backend():
    config = load_config()
    config['backend']['id'] = 'sam2'
    with pytest.raises(ValueError):
        validate_config(config)


def test_cli_validate_accepts_generated_scene(dataset, tmp_path):
    out = str(tmp_path / 'run')
    run_dataset(dataset, load_config(), out, write_overlays=False)
    scenes = os.path.join(out, 'scenes.jsonl')
    assert main(['validate', '--scenes', scenes]) == 0
    record = json.loads(open(scenes, encoding='utf-8').readline())
    record['instances'][0]['volume'] = 1.0
    tampered = tmp_path / 'tampered.jsonl'
    with open(tampered, 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(record) + '\n')
    assert main(['validate', '--scenes', str(tampered)]) == 4


def test_cli_selftest_passes(tmp_path):
    code = main(['selftest', '--out', str(tmp_path / 'selftest')])
    assert code == 0
    report = json.load(
        open(tmp_path / 'selftest' / 'selftest.json', encoding='utf-8'))
    assert report['passed'] is True
    assert report['checks']['empty_scene_ok'] is True
    assert report['checks']['bad_input_structured'] is True
    assert report['checks']['overlap_rejected'] is True


def test_scene_package_has_no_ros_or_device_path():
    package = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'ros2_ws', 'src', 'jcheng_perception_scene',
        'jcheng_perception_scene')
    forbidden = (
        'import rclpy', 'from rclpy', 'ExecuteMotion', 'cv2.VideoCapture',
        'import serial', 'import socket', 'import torch', 'import ultralytics',
        'from ultralytics', 'subprocess',
    )
    for root, _, files in os.walk(package):
        for name in files:
            if not name.endswith('.py'):
                continue
            path = os.path.join(root, name)
            with open(path, encoding='utf-8') as handle:
                source = handle.read()
            for token in forbidden:
                assert token not in source, '%s contains %s' % (path, token)


def test_coco_import_preserves_visible_partition(tmp_path):
    image_dir = tmp_path / 'images'
    image_dir.mkdir()
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    image[10:30, 10:30] = (90, 90, 90)
    image[20:40, 20:40] = (140, 140, 140)
    assert cv2.imwrite(str(image_dir / 'a.png'), image)
    coco = {
        'images': [{
            'id': 1, 'file_name': 'a.png', 'width': 64, 'height': 64,
        }],
        'categories': [{'id': 1, 'name': 'stone'}],
        'annotations': [
            {
                'id': 1, 'image_id': 1, 'category_id': 1, 'iscrowd': 0,
                'segmentation': [[10, 10, 30, 10, 30, 30, 10, 30]],
                'area': 400, 'bbox': [10, 10, 20, 20],
            },
            {
                'id': 2, 'image_id': 1, 'category_id': 1, 'iscrowd': 0,
                'segmentation': [[20, 20, 40, 20, 40, 40, 20, 40]],
                'area': 400, 'bbox': [20, 20, 20, 20],
            },
        ],
    }
    coco_path = tmp_path / 'instances.json'
    with open(coco_path, 'w', encoding='utf-8') as handle:
        json.dump(coco, handle)
    out = tmp_path / 'imported'
    import_coco_dataset(
        str(coco_path), str(image_dir), str(out), 'toy',
        license_id='CC-BY-4.0', image_limit=1,
        created_at=CREATED_AT)
    imported = load_dataset(str(out))
    assert imported.source == 'public'
    assert imported.license == 'CC-BY-4.0'
    assert imported.annotation_method == 'dataset_provided_coco_polygons'
    assert len(imported.scenes) == 1
    scene = imported.scenes[0]
    assert len(scene.instances) == 2
    label = load_ground_truth(scene)
    visible = int(np.count_nonzero(label))
    assert visible == sum(
        int(np.count_nonzero(label == instance['mask_index']))
        for instance in scene.instances)
    # The second polygon overwrites the overlap; the visible partition is
    # still consistent with the manifest areas.
    assert visible == sum(instance['area_px'] for instance in scene.instances)


def test_backend_registry():
    assert isinstance(build_backend('classical_watershed'),
                      ClassicalWatershedBackend)
    with pytest.raises(ValueError):
        build_backend('mask_rcnn')
