"""
S4.4 sensitivity/ablation runner for the offline dual-view matcher.

The configuration is pre-registered (YAML) before the run: baseline parameters,
ablation overrides, extrinsic perturbations and timestamp offsets. Results are
summary metrics only; this module never touches DDS, devices or motion.
"""

import dataclasses
import datetime
import hashlib
import json
import os
from typing import Dict, List

import yaml

from .association import (
    AssociationParams,
    evaluate_association,
    perturb_dataset_extrinsics,
    shift_dataset_timestamps,
)

SENSITIVITY_SCHEMA_VERSION = 'jcheng.association_sensitivity.v1'
SENSITIVITY_EVAL_SCHEMA_VERSION = 'jcheng.ore_association_sensitivity.v1'

_PARAM_FIELDS = {
    field.name for field in dataclasses.fields(AssociationParams)}
_REQUIRED = ('schema_version', 'baseline', 'ablations', 'perturbations',
             'time_offsets_ns')


def _sha256_file(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def load_sensitivity_config(path):
    """Load and validate the pre-registered sensitivity configuration."""
    with open(path, 'r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    if not isinstance(config, dict):
        raise ValueError('sensitivity config must be an object')
    for key in _REQUIRED:
        if key not in config:
            raise ValueError('sensitivity config missing %s' % key)
    if config['schema_version'] != SENSITIVITY_SCHEMA_VERSION:
        raise ValueError('unsupported sensitivity schema: %s' %
                         config['schema_version'])
    baseline = config['baseline']
    if not isinstance(baseline, dict):
        raise ValueError('baseline must be an object')
    unknown = set(baseline) - _PARAM_FIELDS
    if unknown:
        raise ValueError('unknown baseline fields: %s' %
                         ', '.join(sorted(unknown)))
    _params_from(baseline)
    ablations = config['ablations']
    if not isinstance(ablations, list) or not ablations:
        raise ValueError('ablations must be a non-empty list')
    names = set()
    for entry in ablations:
        if not isinstance(entry, dict) or not entry.get('name'):
            raise ValueError('each ablation needs a name')
        if entry['name'] in names:
            raise ValueError('duplicate ablation: %s' % entry['name'])
        names.add(entry['name'])
        unknown = set(entry) - _PARAM_FIELDS - {'name'}
        if unknown:
            raise ValueError('unknown ablation fields: %s' %
                             ', '.join(sorted(unknown)))
        _params_from(entry, base=baseline)
    perturbations = config['perturbations']
    if not isinstance(perturbations, list):
        raise ValueError('perturbations must be a list')
    for entry in perturbations:
        for key in ('name', 'view_id', 'translation_m'):
            if key not in entry:
                raise ValueError('perturbation missing %s' % key)
        if entry['view_id'] not in ('ext', 'wrist'):
            raise ValueError('unsupported perturbation view: %s' %
                             entry['view_id'])
        translation = entry['translation_m']
        if not isinstance(translation, list) or len(translation) != 3:
            raise ValueError('translation_m must be a three-value list')
        if entry.get('axis', 'z') not in ('x', 'y', 'z'):
            raise ValueError('unsupported rotation axis')
    offsets = config['time_offsets_ns']
    if not isinstance(offsets, list) or any(
            not isinstance(value, int) or value < 0 for value in offsets):
        raise ValueError('time_offsets_ns must be non-negative integers')
    return config


def _params_from(overrides, base=None):
    values = dict(base or {})
    values.update({
        key: value for key, value in overrides.items()
        if key != 'name' and key in _PARAM_FIELDS
    })
    return AssociationParams(**values)


def _summary(metrics):
    result = metrics['results']
    return {
        key: result[key]
        for key in (
            'truth_pairs', 'predicted_pairs', 'correct_pairs', 'precision',
            'recall', 'f1', 'ambiguous_clusters', 'forced_id_errors',
            'singleton_reason_counts', 'per_density', 'per_occlusion',
            'latency_ms')
    }


def _translation_norm_m(translation):
    return float(sum(float(value) ** 2 for value in translation) ** 0.5)


def run_sensitivity(dataset, config, config_path=None):
    """Run baseline, ablations, extrinsic perturbations and timestamp offsets."""
    baseline_params = _params_from(config['baseline'])
    baseline = evaluate_association(dataset, baseline_params)
    baseline_summary = _summary(baseline)
    baseline_f1 = float(baseline_summary['f1'])

    ablations = []
    for entry in config['ablations']:
        params = _params_from(entry, base=config['baseline'])
        metrics = evaluate_association(dataset, params)
        summary = _summary(metrics)
        ablations.append({
            'name': entry['name'],
            'strategy': params.strategy,
            'overrides': {
                key: value for key, value in entry.items() if key != 'name'},
            **_summary(metrics),
            'delta_f1': round(float(summary['f1']) - baseline_f1, 6),
        })

    perturbations = []
    for entry in config['perturbations']:
        translation = [float(value) for value in entry['translation_m']]
        rotation_deg = float(entry.get('rotation_deg', 0.0))
        axis = entry.get('axis', 'z')
        perturbed = perturb_dataset_extrinsics(
            dataset, entry['view_id'], translation, rotation_deg, axis)
        metrics = evaluate_association(perturbed, baseline_params)
        summary = _summary(metrics)
        perturbations.append({
            'name': entry['name'],
            'view_id': entry['view_id'],
            'translation_m': translation,
            'translation_norm_mm': round(_translation_norm_m(translation) * 1000, 3),
            'rotation_deg': rotation_deg,
            'axis': axis,
            **_summary(metrics),
            'delta_f1': round(float(summary['f1']) - baseline_f1, 6),
        })

    time_offsets = []
    for offset in config['time_offsets_ns']:
        shifted = shift_dataset_timestamps(dataset, 'wrist', int(offset))
        metrics = evaluate_association(shifted, baseline_params)
        summary = _summary(metrics)
        time_offsets.append({
            'view_id': 'wrist',
            'offset_ns': int(offset),
            **_summary(metrics),
            'delta_f1': round(float(summary['f1']) - baseline_f1, 6),
        })

    acceptance = config.get('acceptance') or {}
    checks = {}
    if 'baseline_f1_min' in acceptance:
        checks['baseline_f1_min'] = bool(
            baseline_f1 >= float(acceptance['baseline_f1_min']))
    translation_drops = [
        baseline_f1 - item['f1'] for item in perturbations
        if item['translation_norm_mm'] >= 10.0 - 1e-9]
    if 'max_translation_10mm_f1_drop' in acceptance:
        checks['max_translation_10mm_f1_drop'] = bool(
            all(drop <= float(acceptance['max_translation_10mm_f1_drop'])
                for drop in translation_drops))
    rotation_drops = [
        baseline_f1 - item['f1'] for item in perturbations
        if item['rotation_deg'] >= 1.0 - 1e-9]
    if 'max_rotation_1deg_f1_drop' in acceptance:
        checks['max_rotation_1deg_f1_drop'] = bool(
            all(drop <= float(acceptance['max_rotation_1deg_f1_drop'])
                for drop in rotation_drops))
    if 'time_offset_outside_window_f1_max' in acceptance:
        window = baseline_params.sync_window_ns
        checks['time_offset_outside_window_f1_max'] = bool(
            all(item['f1'] <= float(
                acceptance['time_offset_outside_window_f1_max'])
                for item in time_offsets if item['offset_ns'] > window))

    return {
        'schema_version': SENSITIVITY_EVAL_SCHEMA_VERSION,
        'created_at': datetime.datetime.now(
            datetime.timezone.utc).replace(microsecond=0).isoformat(),
        'config_path': os.path.abspath(config_path) if config_path else None,
        'config_sha256': _sha256_file(config_path) if config_path else None,
        'baseline': {
            'params': baseline_params.to_dict(),
            **_summary(baseline),
        },
        'ablations': ablations,
        'perturbations': perturbations,
        'time_offsets': time_offsets,
        'acceptance': acceptance,
        'checks': checks,
        'hardware_claimed': False,
        'real_ore_validation': False,
    }
