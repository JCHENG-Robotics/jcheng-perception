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
    Dataset,
    import_coco_dataset,
    load_dataset,
    load_ground_truth,
    read_predictions,
    validate_dataset_files,
)
from jcheng_perception_scene.geometry import GeometryParams, visible_geometry
from jcheng_perception_scene.metrics import evaluate
from jcheng_perception_scene.pipeline import run_dataset, run_unlabelled
from jcheng_perception_scene.relation_benchmark import (
    evaluate_relation_benchmark,
    generate_relation_benchmark,
)
from jcheng_perception_scene.relation_eval import (
    RelationEvalParams,
    evaluate_predicted_relations,
)
from jcheng_perception_scene.relations import (
    RelationParams,
    build_relation_graph,
    compare_relation_sets,
)
from jcheng_perception_scene.sam2_tiles import (
    TileSet,
    TilesConfig,
    load_tiles_config,
    merge_tile_set,
    run_sam2_tiles,
)
from jcheng_perception_scene.scene import build_scene_state
from jcheng_perception_scene.scene_v2 import (
    build_scene_v2,
    geometry_for_mask,
    validate_scene_v2,
)
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


def _plane_geometry_inputs():
    mask = np.zeros((480, 640), dtype=bool)
    mask[190:290, 270:370] = True
    depth = np.ones((480, 640), dtype=np.float64)
    camera = np.array([[500.0, 0.0, 320.0],
                       [0.0, 500.0, 240.0],
                       [0.0, 0.0, 1.0]])
    return mask, depth, camera


def test_visible_geometry_plane():
    mask, depth, camera = _plane_geometry_inputs()
    result = visible_geometry(
        mask, depth, camera, np.eye(4), GeometryParams(stride=2))
    assert result['status'] == 'computed'
    assert result['visible_point_count'] == 2500
    assert result['depth_valid_fraction'] == pytest.approx(1.0)
    centroid = np.asarray(result['visible_centroid_base'])
    assert centroid[2] == pytest.approx(1.0, abs=1e-9)
    assert abs(float(centroid[0])) < 0.01
    assert abs(float(centroid[1])) < 0.01
    aabb = result['visible_aabb']
    assert aabb['extent'][2] == pytest.approx(0.0, abs=1e-9)
    assert aabb['extent'][0] == pytest.approx(0.196, abs=0.01)
    assert result['visible_obb'] is not None
    assert np.asarray(result['visible_centroid_covariance']).shape == (3, 3)
    assert result['field_status']['true_centroid'] == 'unknown'
    assert 'true_centroid' in result['unknowns']


def test_visible_geometry_transform_and_failure_paths():
    mask, depth, camera = _plane_geometry_inputs()
    transform = np.eye(4)
    transform[:3, 3] = [0.1, 0.2, 0.3]
    result = visible_geometry(
        mask, depth, camera, transform, GeometryParams(stride=2))
    assert result['status'] == 'computed'
    centroid = np.asarray(result['visible_centroid_base'])
    assert centroid[0] == pytest.approx(0.1, abs=0.01)
    assert centroid[1] == pytest.approx(0.2, abs=0.01)
    assert centroid[2] == pytest.approx(1.3, abs=1e-9)

    bad = visible_geometry(
        mask, np.zeros_like(depth), camera, np.eye(4))
    assert bad['status'] == 'insufficient_depth'
    assert bad['visible_centroid_base'] is None
    assert bad['depth_valid_fraction'] == 0.0

    empty = np.zeros_like(mask)
    invalid = visible_geometry(empty, depth, camera, np.eye(4))
    assert invalid['status'] == 'invalid_mask'

    with pytest.raises(ValueError):
        visible_geometry(mask, depth[:, :-1], camera, np.eye(4))


def _relation_instances():
    first = np.zeros((32, 48), dtype=bool)
    second = np.zeros((32, 48), dtype=bool)
    third = np.zeros((32, 48), dtype=bool)
    first[8:18, 8:18] = True
    second[8:18, 18:28] = True
    third[8:18, 40:48] = True
    return [
        InstanceMask('rel:1', first, SEMANTIC_LABEL, 0.9),
        InstanceMask('rel:2', second, SEMANTIC_LABEL, 0.9),
        InstanceMask('rel:3', third, SEMANTIC_LABEL, 0.9),
    ]


def test_relation_graph_touching_near_and_density_cap():
    instances = _relation_instances()
    edges = build_relation_graph(
        instances, RelationParams(adjacency_px=5, touching_min_px=8,
                                  max_instances=10))
    assert len(edges) == 1
    assert edges[0].predicate == 'touching'
    assert edges[0].evidence['contact_px'] > 0
    assert edges[0].evidence['depth_order'] == 'unknown'

    near = build_relation_graph(
        instances[:2], RelationParams(adjacency_px=5, touching_min_px=1000,
                                      max_instances=10))
    assert len(near) == 1
    assert near[0].predicate == 'near'

    capped = build_relation_graph(
        instances, RelationParams(adjacency_px=5, touching_min_px=8,
                                  max_instances=1))
    assert capped == []


def test_relation_set_comparison_and_v2_attachment():
    instances = _relation_instances()[:2]
    edges = build_relation_graph(
        instances, RelationParams(adjacency_px=5, touching_min_px=8,
                                  max_instances=10))
    same = compare_relation_sets(edges, edges)
    assert same['precision'] == pytest.approx(1.0)
    assert same['recall'] == pytest.approx(1.0)
    extra = edges + [type(edges[0])(
        subject_id='rel:1', predicate='near', object_id='rel:2',
        confidence=0.1, evidence={})]
    worse = compare_relation_sets(extra, edges)
    assert worse['precision'] == pytest.approx(0.5)

    image = np.zeros((32, 48, 3), dtype=np.uint8)
    scene_v1 = build_scene_state(
        'rel', 0, 'base_link', image, None, None, instances,
        load_config(), ClassicalWatershedBackend.info, {})
    v2 = build_scene_v2(scene_v1.to_dict(), {}, relations=edges)
    assert validate_scene_v2(v2) == []
    assert v2['relations'][0]['predicate'] == 'touching'
    with pytest.raises(ValueError):
        build_scene_v2(
            scene_v1.to_dict(), {}, relations=edges + edges)


def test_relation_benchmark_generate_and_evaluate(tmp_path):
    dataset_path, truth_path = generate_relation_benchmark(
        str(tmp_path / 'relation-benchmark'), per_class=2, seed=7,
        created_at=CREATED_AT)
    dataset = load_dataset(str(tmp_path / 'relation-benchmark'))
    result = evaluate_relation_benchmark(
        dataset, truth_path, RelationParams(
            adjacency_px=5, touching_min_px=8, max_instances=128))
    assert result['precision'] == pytest.approx(1.0)
    assert result['recall'] == pytest.approx(1.0)
    assert result['f1'] == pytest.approx(1.0)
    assert result['predicted_edges'] == 4
    assert result['truth_edges'] == 4
    assert result['evaluated_scenes'] == 6
    assert result['data_source'] == 'synthetic'
    assert result['hardware_validation'] is False
    assert result['real_ore_validation'] is False


def test_predicted_relation_evaluation_with_perfect_masks(tmp_path):
    root = tmp_path / 'relation-pred'
    generate_relation_benchmark(
        str(root), per_class=1, seed=11, created_at=CREATED_AT)
    dataset = load_dataset(str(root))
    prediction_root = root / 'pred'
    (prediction_root / 'masks').mkdir(parents=True)
    records = []
    for scene in dataset.scenes:
        shutil.copyfile(
            scene.mask_path,
            prediction_root / 'masks' / (scene.scene_id + '.png'))
        records.append({
            'schema_version': 'jcheng.scene_state.v1',
            'scene_id': scene.scene_id,
            'image': {
                'path': scene.image_path,
                'width': scene.width,
                'height': scene.height,
            },
            'instances': [{
                'instance_id': item['instance_id'],
                'mask_index': item['mask_index'],
                'mask_path': 'masks/%s.png' % scene.scene_id,
                'bbox_xyxy': item['bbox_xyxy'],
                'detection_confidence': 1.0,
            } for item in scene.instances],
            'relations': [],
            'validity': {
                'valid': True,
                'empty_scene': False,
                'missing_fields': [],
                'failure_reason': None,
                'unknowns': [],
            },
            'timing': {},
        })
    result = evaluate_predicted_relations(
        dataset, records, str(prediction_root),
        RelationEvalParams(RelationParams(5, 8, 128), 0.5))
    assert result['precision'] == pytest.approx(1.0)
    assert result['recall'] == pytest.approx(1.0)
    assert result['evaluated_scenes'] == 3
    assert result['instance_matching']['matched'] == 6
    assert result['instance_matching']['match_rate'] == pytest.approx(1.0)
    assert result['truth_edges'] == 2


def test_scene_v2_attaches_visible_geometry():
    image = np.zeros((64, 64, 3), dtype=np.uint8)
    mask = np.zeros((64, 64), dtype=bool)
    mask[20:40, 20:40] = True
    instance = InstanceMask('v2:0001', mask, SEMANTIC_LABEL, 0.9)
    scene_v1 = build_scene_state(
        'v2', 0, 'base_link', image, None, None, [instance],
        load_config(), ClassicalWatershedBackend.info, {})
    camera = np.array([[100.0, 0.0, 32.0],
                       [0.0, 100.0, 32.0],
                       [0.0, 0.0, 1.0]])
    geometry = geometry_for_mask(
        mask, np.ones((64, 64), dtype=np.float64), camera, np.eye(4),
        frame_id='base_link')
    v2 = build_scene_v2(
        scene_v1.to_dict(), {'v2:0001': geometry})
    assert v2['schema_version'] == 'jcheng.scene_state.v2'
    assert scene_v1.to_dict()['schema_version'] == 'jcheng.scene_state.v1'
    assert v2['instances'][0]['geometry']['status'] == 'computed'
    assert validate_scene_v2(v2) == []

    missing = build_scene_v2(scene_v1.to_dict(), {})
    assert missing['instances'][0]['geometry']['status'] == 'not_computed'
    assert validate_scene_v2(missing) == []

    tampered = json.loads(json.dumps(v2))
    tampered['instances'][0]['true_centroid'] = [0.0, 0.0, 0.0]
    assert validate_scene_v2(tampered)


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


def _write_sam2_tiles(tile_root, scene, label):
    """Create one SAM2-style tile result from a synthetic GT label image."""
    stem = os.path.splitext(os.path.basename(scene.image_path))[0]
    tile_dir = os.path.join(tile_root, '%s_x0000_y0000' % stem)
    masks_dir = os.path.join(tile_dir, 'masks')
    os.makedirs(masks_dir)
    entries = []
    for order, instance in enumerate(scene.instances):
        mask = (label == int(instance['mask_index'])).astype(np.uint8) * 255
        name = '%02d_ore.png' % order
        assert cv2.imwrite(os.path.join(masks_dir, name), mask)
        entries.append({
            'mask': name,
            'predicted_iou': 0.9,
            'stability_score': 0.9,
        })
    # Duplicate the first mask at a lower score; the converter must skip it.
    if entries:
        entries.append(dict(entries[0], predicted_iou=0.8))
    with open(os.path.join(tile_dir, 'result.json'), 'w',
              encoding='utf-8') as handle:
        json.dump({
            'image': scene.image_path,
            'shape': [scene.height, scene.width],
            'wall_seconds': 1.0,
            'masks': entries,
        }, handle)
    return tile_root


def test_sam2_tile_conversion_reconstructs_partition(dataset, tmp_path):
    scene = dataset.scenes[0]
    label = cv2.imread(scene.mask_path, cv2.IMREAD_UNCHANGED)
    tile_root = _write_sam2_tiles(
        str(tmp_path / 'tiles'), scene, label)
    config = TilesConfig(
        default=TileSet(tile_root, tile_size=max(scene.width, scene.height),
                        max_area_fraction=1.0),
        overrides={},
    )
    out = str(tmp_path / 'sam2-out')
    summary = run_sam2_tiles(
        dataset, config, out, scene_config=load_config(), limit=1)
    assert summary['scene_records'] == 1
    assert summary['instance_records'] == len(scene.instances)
    records = read_predictions(os.path.join(out, 'scenes.jsonl'))
    assert len(records) == 1
    assert len(records[0]['instances']) == len(scene.instances)
    subset = Dataset(dataset.root, dataset.manifest_path, dataset.manifest,
                     [scene])
    metrics = evaluate(
        subset, records, out, load_config())
    assert metrics['mask_ap50'] == pytest.approx(1.0, abs=1e-9)
    assert metrics['instance_count_mae'] == pytest.approx(0.0)


def test_sam2_tiles_config_and_override(tmp_path):
    config_path = tmp_path / 'tiles.yaml'
    config_path.write_text(
        'schema_version: 1\n'
        'backend:\n'
        '  id: sam2.test\n'
        '  version: "1"\n'
        '  score_field: predicted_iou\n'
        '  min_area_px: 10\n'
        '  min_new_fraction: 0.3\n'
        '  mask_threshold: 127\n'
        'tile_sets:\n'
        '  default:\n'
        '    root: /tmp/default\n'
        '    tile_size: 1008\n'
        '    max_area_fraction: 0.05\n'
        '  overrides:\n'
        '    "40_2":\n'
        '      root: /tmp/small\n'
        '      tile_size: 504\n'
        '      max_area_fraction: 0.10\n',
        encoding='utf-8')
    config = load_tiles_config(str(config_path))
    assert config.backend_id == 'sam2.test'
    assert config.for_image('40_2').tile_size == 504
    assert config.for_image('40_3').tile_size == 1008

    config_path.write_text(
        'schema_version: 1\n'
        'tile_sets:\n'
        '  default:\n'
        '    root: /tmp/default\n'
        '    tile_size: 1008\n'
        '    typo: 1\n',
        encoding='utf-8')
    with pytest.raises(ValueError):
        load_tiles_config(str(config_path))


def test_sam2_tiles_missing_root_fails(dataset, tmp_path):
    config = TilesConfig(
        default=TileSet(str(tmp_path / 'missing'), tile_size=64,
                        max_area_fraction=0.5),
        overrides={},
    )
    scene = dataset.scenes[0]
    label = cv2.imread(scene.mask_path, cv2.IMREAD_UNCHANGED)
    with pytest.raises(ValueError):
        merge_tile_set(
            config.default,
            os.path.splitext(os.path.basename(scene.image_path))[0],
            scene.width, scene.height, config)
    assert label is not None


def test_sam2_tiles_cli(dataset, tmp_path):
    scene = dataset.scenes[0]
    label = cv2.imread(scene.mask_path, cv2.IMREAD_UNCHANGED)
    tile_root = _write_sam2_tiles(
        str(tmp_path / 'tiles'), scene, label)
    config_path = tmp_path / 'tiles.yaml'
    config_path.write_text(
        'schema_version: 1\n'
        'tile_sets:\n'
        '  default:\n'
        '    root: "%s"\n'
        '    tile_size: %d\n'
        '    max_area_fraction: 1.0\n' % (
            tile_root, max(scene.width, scene.height)),
        encoding='utf-8')
    code = main([
        'import-sam2-tiles', '--dataset', dataset.root,
        '--out', str(tmp_path / 'cli-out'), '--tiles', str(config_path),
        '--limit', '1', '--metrics',
    ])
    assert code == 0
    assert os.path.isfile(tmp_path / 'cli-out' / 'metrics.json')


def test_backend_registry():
    assert isinstance(build_backend('classical_watershed'),
                      ClassicalWatershedBackend)
    with pytest.raises(ValueError):
        build_backend('mask_rcnn')
