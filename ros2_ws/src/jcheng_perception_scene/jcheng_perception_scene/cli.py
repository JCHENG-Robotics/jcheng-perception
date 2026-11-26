"""
Command-line interface for the offline ore scene baseline.

Exit codes:
  0  success
  2  input/configuration/dataset error
  3  backend/runtime failure while processing a scene
  4  output validation failure
"""

import argparse
import copy
import json
import os
import tempfile

from .backends.classical import build_backend
from .config import load_config, validate_config
from .dataset import (
    import_coco_dataset,
    load_dataset,
    read_predictions,
    validate_dataset_files,
)
from .metrics import evaluate
from .pipeline import run_dataset, run_unlabelled
from .synthetic import generate_dataset
from .types import (
    EVALUATION_SCHEMA_VERSION,
    SCENE_SCHEMA_VERSION,
    UNKNOWNS,
)

EXIT_OK = 0
EXIT_INPUT = 2
EXIT_BACKEND = 3
EXIT_VALIDATION = 4

_FORBIDDEN_INSTANCE_KEYS = {
    'true_centroid', 'true_center', 'full_extent', 'volume', 'mass',
    'material_composition', 'ore_grade', 'economic_value',
}


def _load_config(path, backend_id=None):
    config = load_config(path)
    if backend_id:
        config = copy.deepcopy(config)
        config['backend']['id'] = backend_id
        validate_config(config)
    return config


def _print_json(payload):
    print(json.dumps(payload, indent=2, sort_keys=True))


def _validate_record(record, line_number):
    errors = []
    if record.get('schema_version') != SCENE_SCHEMA_VERSION:
        errors.append('line %d: schema_version must be %s' % (
            line_number, SCENE_SCHEMA_VERSION))
    for key in ('scene_id', 'image', 'instances', 'validity'):
        if key not in record:
            errors.append('line %d: missing %s' % (line_number, key))
    instances = record.get('instances', [])
    if not isinstance(instances, list):
        errors.append('line %d: instances must be a list' % line_number)
        instances = []
    seen = set()
    for instance in instances:
        instance_id = instance.get('instance_id')
        if not instance_id:
            errors.append('line %d: instance without instance_id' %
                          line_number)
        elif instance_id in seen:
            errors.append('line %d: duplicate instance_id %s' % (
                line_number, instance_id))
        seen.add(instance_id)
        confidence = instance.get('detection_confidence')
        if not isinstance(confidence, (int, float)) or not (
                0.0 <= float(confidence) <= 1.0):
            errors.append('line %d: invalid confidence for %s' % (
                line_number, instance_id))
        if int(instance.get('mask_index', 0)) <= 0:
            errors.append('line %d: invalid mask_index for %s' % (
                line_number, instance_id))
        forbidden = sorted(set(instance) & _FORBIDDEN_INSTANCE_KEYS)
        if forbidden:
            errors.append('line %d: forbidden fields %s' % (
                line_number, ', '.join(forbidden)))
    relations = record.get('relations', [])
    for relation in relations:
        if relation.get('subject_id') not in seen or \
                relation.get('object_id') not in seen:
            errors.append('line %d: relation references unknown instance' %
                          line_number)
    validity = record.get('validity', {})
    if not isinstance(validity, dict) or 'valid' not in validity:
        errors.append('line %d: validity.valid missing' % line_number)
    if validity.get('valid') is False and not validity.get('failure_reason'):
        errors.append('line %d: invalid scene without failure_reason' %
                      line_number)
    for unknown in validity.get('unknowns', []):
        if unknown not in UNKNOWNS and unknown not in (
                'occlusion_direction',):
            errors.append('line %d: unexpected unknown %s' % (
                line_number, unknown))
    return errors


def _command_generate(args):
    manifest_path = generate_dataset(
        args.out, scene_count=args.scenes, empty_count=args.empty_scenes,
        seed=args.seed, width=args.width, height=args.height)
    _print_json({
        'dataset_manifest': manifest_path,
        'scenes': args.scenes,
        'empty_scenes': args.empty_scenes,
        'seed': args.seed,
        'source': 'synthetic',
        'license': 'CC0-1.0',
    })
    return EXIT_OK


def _command_import_coco(args):
    manifest = import_coco_dataset(
        args.coco, args.images, args.out, args.dataset_id,
        source='public', license_id=args.license, citation=args.citation,
        source_url=args.source_url, category_names=args.categories,
        image_limit=args.image_limit, image_offset=args.image_offset)
    _print_json({
        'dataset_manifest': manifest,
        'dataset_id': args.dataset_id,
        'source': 'public',
        'license': args.license,
        'image_limit': args.image_limit,
        'image_offset': args.image_offset,
    })
    return EXIT_OK


def _command_run(args):
    config = _load_config(args.config, args.backend)
    dataset = load_dataset(args.dataset)
    file_errors = validate_dataset_files(dataset, check_hashes=True)
    if file_errors and not args.allow_dataset_errors:
        _print_json({
            'error': 'dataset validation failed',
            'details': [error.to_dict() for error in file_errors[:20]],
            'total': len(file_errors),
        })
        return EXIT_INPUT
    summary = run_dataset(
        dataset, config, args.out, limit=args.limit,
        write_overlays=not args.no_overlays)
    metrics = None
    if not args.no_metrics:
        metrics = evaluate(
            dataset,
            read_predictions(os.path.join(args.out, 'scenes.jsonl')),
            args.out, config)
        metrics_path = os.path.join(args.out, 'metrics.json')
        with open(metrics_path, 'w', encoding='utf-8') as handle:
            json.dump(metrics, handle, indent=2, sort_keys=True)
            handle.write('\n')
    _print_json({
        'summary': summary['summary_path'],
        'scenes': summary['scene_records'],
        'instances': summary['instance_records'],
        'failed_scenes': summary['failed_scene_count'],
        'metrics': os.path.join(args.out, 'metrics.json')
        if metrics is not None else None,
        'data_source': dataset.source,
        'hardware_validation': False,
    })
    if summary['failed_scene_count'] and not args.allow_failures:
        return EXIT_BACKEND
    return EXIT_OK


def _command_evaluate(args):
    config = _load_config(args.config)
    dataset = load_dataset(args.dataset)
    prediction_root = args.prediction_root or os.path.dirname(
        os.path.abspath(args.predictions))
    records = read_predictions(args.predictions)
    metrics = evaluate(dataset, records, prediction_root, config)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as handle:
        json.dump(metrics, handle, indent=2, sort_keys=True)
        handle.write('\n')
    _print_json({
        'metrics': os.path.abspath(args.out),
        'schema_version': EVALUATION_SCHEMA_VERSION,
        'mask_ap50': metrics['mask_ap50'],
        'mask_ap50_95': metrics['mask_ap50_95'],
        'failed_scenes': len(metrics['failed_scenes']),
        'data_source': dataset.source,
        'hardware_validation': False,
    })
    return EXIT_OK


def _command_validate(args):
    records = read_predictions(args.scenes)
    errors = []
    for line_number, record in enumerate(records, start=1):
        errors.extend(_validate_record(record, line_number))
    if errors:
        _print_json({'valid': False, 'errors': errors})
        return EXIT_VALIDATION
    _print_json({
        'valid': True,
        'records': len(records),
        'schema_version': SCENE_SCHEMA_VERSION,
    })
    return EXIT_OK


def _command_run_images(args):
    config = _load_config(args.config, args.backend)
    summary = run_unlabelled(
        args.images, config, args.out, limit=args.limit,
        write_overlays=not args.no_overlays)
    _print_json({
        'summary': summary['outputs'],
        'scenes': summary['scene_records'],
        'instances': summary['instance_records'],
        'failed_scenes': summary['failed_scene_count'],
        'accuracy_metrics': None,
        'note': summary['note'],
    })
    return EXIT_OK


def _command_selftest(args):
    """Exercise empty, overlapping, bad-input and failure paths."""
    out_dir = args.out or tempfile.mkdtemp(prefix='jcheng-ore-selftest-')
    os.makedirs(out_dir, exist_ok=True)
    config = _load_config(args.config)
    dataset_root = os.path.join(out_dir, 'synthetic')
    generate_dataset(
        dataset_root, scene_count=4, empty_count=2, seed=20261126,
        width=320, height=240, created_at='2026-11-26T00:00:00+00:00')
    dataset = load_dataset(dataset_root)
    run_summary = run_dataset(
        dataset, config, os.path.join(out_dir, 'run'),
        write_overlays=True)
    metrics = evaluate(
        dataset,
        read_predictions(os.path.join(out_dir, 'run', 'scenes.jsonl')),
        os.path.join(out_dir, 'run'), config)

    # Bad input: a truncated image must produce a structured failure.
    bad_dir = os.path.join(out_dir, 'bad-input')
    os.makedirs(bad_dir, exist_ok=True)
    bad_path = os.path.join(bad_dir, 'truncated.png')
    with open(bad_path, 'wb') as handle:
        handle.write(b'\x89PNG\r\n\x1a\n')
    bad_summary = run_unlabelled(
        bad_dir, config, os.path.join(out_dir, 'bad-run'),
        write_overlays=False)

    # Overlap: a backend that returns overlapping masks must be rejected.
    overlap_rejected = False
    try:
        from .scene import build_scene_state
        from .types import InstanceMask, SEMANTIC_LABEL
        import numpy as np
        from .backends.classical import ClassicalWatershedBackend
        first = InstanceMask(
            instance_id='overlap:1',
            mask=np.ones((8, 8), dtype=bool),
            semantic_label=SEMANTIC_LABEL,
            score=0.5)
        second = InstanceMask(
            instance_id='overlap:2',
            mask=np.ones((8, 8), dtype=bool),
            semantic_label=SEMANTIC_LABEL,
            score=0.5)
        build_scene_state(
            'overlap', 0, 'camera', np.zeros((8, 8, 3), dtype=np.uint8),
            None, None, [first, second], config,
            ClassicalWatershedBackend.info, {})
    except ValueError:
        overlap_rejected = True

    report = {
        'schema_version': 'jcheng.ore_selftest.v1',
        'out_dir': os.path.abspath(out_dir),
        'synthetic': {
            'scenes': run_summary['scene_records'],
            'empty_scenes': run_summary['empty_scenes'],
            'failed_scenes': run_summary['failed_scene_count'],
            'mask_ap50': metrics['mask_ap50'],
            'mask_ap50_95': metrics['mask_ap50_95'],
        },
        'bad_input': {
            'failed_scenes': bad_summary['failed_scene_count'],
            'accuracy_metrics': bad_summary['accuracy_metrics'],
        },
        'overlap_rejected': overlap_rejected,
        'checks': {
            'empty_scene_ok': run_summary['empty_scenes'] >= 2,
            'bad_input_structured': bad_summary['failed_scene_count'] == 1,
            'overlap_rejected': overlap_rejected,
        },
        'hardware_validation': False,
        'real_ore_validation': False,
    }
    report['passed'] = all(report['checks'].values())
    report_path = os.path.join(out_dir, 'selftest.json')
    with open(report_path, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
        handle.write('\n')
    _print_json(report)
    return EXIT_OK if report['passed'] else EXIT_VALIDATION


def build_parser():
    """Build the argument parser."""
    parser = argparse.ArgumentParser(
        prog='ore_scene',
        description='Offline ore instance segmentation and scene state. '
        'Advisory and offline only; no robot connection.')
    subparsers = parser.add_subparsers(dest='command', required=True)

    generate = subparsers.add_parser(
        'generate-synthetic', help='generate deterministic synthetic data')
    generate.add_argument('--out', required=True)
    generate.add_argument('--scenes', type=int, default=36)
    generate.add_argument('--empty-scenes', type=int, default=3)
    generate.add_argument('--seed', type=int, default=20261126)
    generate.add_argument('--width', type=int, default=640)
    generate.add_argument('--height', type=int, default=480)
    generate.set_defaults(func=_command_generate)

    coco = subparsers.add_parser(
        'import-coco',
        help='convert COCO polygon instance annotations to the internal layout')
    coco.add_argument('--coco', required=True)
    coco.add_argument('--images', required=True)
    coco.add_argument('--out', required=True)
    coco.add_argument('--dataset-id', required=True)
    coco.add_argument('--license', default='unknown')
    coco.add_argument('--citation')
    coco.add_argument('--source-url')
    coco.add_argument('--categories', nargs='*')
    coco.add_argument('--image-limit', type=int)
    coco.add_argument('--image-offset', type=int, default=0)
    coco.set_defaults(func=_command_import_coco)

    run = subparsers.add_parser(
        'run', help='run the backend over a labelled dataset')
    run.add_argument('--dataset', required=True)
    run.add_argument('--out', required=True)
    run.add_argument('--config')
    run.add_argument('--backend', choices=('classical_watershed',
                                           'classical_cc'))
    run.add_argument('--limit', type=int)
    run.add_argument('--no-overlays', action='store_true')
    run.add_argument('--no-metrics', action='store_true')
    run.add_argument('--allow-failures', action='store_true')
    run.add_argument('--allow-dataset-errors', action='store_true')
    run.set_defaults(func=_command_run)

    evaluate_parser = subparsers.add_parser(
        'evaluate', help='evaluate an existing scenes.jsonl')
    evaluate_parser.add_argument('--dataset', required=True)
    evaluate_parser.add_argument('--predictions', required=True)
    evaluate_parser.add_argument('--prediction-root')
    evaluate_parser.add_argument('--out', required=True)
    evaluate_parser.add_argument('--config')
    evaluate_parser.set_defaults(func=_command_evaluate)

    validate = subparsers.add_parser(
        'validate', help='validate a scenes.jsonl export')
    validate.add_argument('--scenes', required=True)
    validate.set_defaults(func=_command_validate)

    images = subparsers.add_parser(
        'run-images', help='qualitative run on unlabelled images')
    images.add_argument('--images', required=True)
    images.add_argument('--out', required=True)
    images.add_argument('--config')
    images.add_argument('--backend', choices=('classical_watershed',
                                              'classical_cc'))
    images.add_argument('--limit', type=int)
    images.add_argument('--no-overlays', action='store_true')
    images.set_defaults(func=_command_run_images)

    selftest = subparsers.add_parser(
        'selftest', help='exercise empty, bad-input and overlap paths')
    selftest.add_argument('--out')
    selftest.add_argument('--config')
    selftest.set_defaults(func=_command_selftest)
    return parser


def main(argv=None):
    """Run the CLI and return a process exit code."""
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (ValueError, OSError) as error:
        _print_json({
            'error': type(error).__name__,
            'message': str(error),
        })
        return EXIT_INPUT
