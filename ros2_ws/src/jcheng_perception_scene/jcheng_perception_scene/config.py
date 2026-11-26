"""
Strict configuration loading for the offline ore scene pipeline.

Unknown keys are rejected: a typo in a threshold must not silently fall back
to a default that changes the reported metrics.
"""

import copy
import json
import math

import yaml

from .types import sha256_bytes

SUPPORTED_BACKENDS = ('classical_watershed', 'classical_cc')

DEFAULT_CONFIG = {
    'schema_version': 1,
    'scene': {
        'frame_id': 'camera_color_optical_frame',
        'semantic_label': 'ore',
    },
    'backend': {
        'id': 'classical_watershed',
        'version': '1.0.0',
        'foreground': {
            'mode': 'otsu_gray',
            'polarity': 'bright',
            'border_fraction': 0.05,
            'border_mad_max': 18.0,
            'threshold_scale': 0.85,
            'clahe_clip_limit': 2.0,
            'clahe_tile_grid': 8,
            'blur_kernel': 5,
            'open_kernel': 3,
            'close_kernel': 7,
            'min_area_px': 120,
            'max_area_fraction': 0.20,
            'min_foreground_fraction': 0.01,
            'min_contrast': 10.0,
        },
        'watershed': {
            'peak_mode': 'global',
            'peak_ratio': 0.25,
            'peak_window': 7,
            'min_peak_px': 1.5,
            'gradient_blur': 5,
            'adjacency_px': 5,
            'adjacency_max_instances': 64,
        },
        'score': {
            'area_reference_fraction': 0.01,
        },
    },
    'evaluation': {
        'iou_min': 0.50,
        'iou_max': 0.95,
        'iou_step': 0.05,
        'score_threshold': 0.05,
        'pr_score_threshold': 0.50,
        'pr_iou': 0.50,
    },
    'output': {
        'overlays': True,
    },
}

_FOREGROUND_KEYS = {
    'mode', 'polarity', 'border_fraction', 'border_mad_max',
    'threshold_scale', 'clahe_clip_limit', 'clahe_tile_grid', 'blur_kernel',
    'open_kernel', 'close_kernel', 'min_area_px', 'max_area_fraction',
    'min_foreground_fraction', 'min_contrast',
}
_WATERSHED_KEYS = {
    'peak_mode', 'peak_ratio', 'peak_window', 'min_peak_px',
    'gradient_blur', 'adjacency_px', 'adjacency_max_instances',
}
_SCORE_KEYS = {'area_reference_fraction'}
_BACKEND_KEYS = {'id', 'version', 'foreground', 'watershed', 'score'}
_EVALUATION_KEYS = {
    'iou_min', 'iou_max', 'iou_step', 'score_threshold',
    'pr_score_threshold', 'pr_iou',
}
_OUTPUT_KEYS = {'overlays'}
_SCENE_KEYS = {'frame_id', 'semantic_label'}
_TOP_KEYS = {
    'schema_version', 'scene', 'backend', 'evaluation', 'output',
}


def _deep_merge(base, override):
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def _require_mapping(value, name):
    if not isinstance(value, dict):
        raise ValueError('%s must be a mapping' % name)


def _require_keys(mapping, allowed, name):
    unknown = sorted(set(mapping) - set(allowed))
    if unknown:
        raise ValueError('%s has unknown keys: %s' %
                          (name, ', '.join(unknown)))


def _number(value, name):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('%s must be finite' % name)
    return number


def _positive_int(value, name):
    if isinstance(value, bool) or int(value) != value or int(value) <= 0:
        raise ValueError('%s must be a positive integer' % name)
    return int(value)


def validate_config(config):
    """Validate a merged configuration and raise ``ValueError`` on error."""
    _require_mapping(config, 'config')
    _require_keys(config, _TOP_KEYS, 'config')
    if int(config.get('schema_version', 0)) != 1:
        raise ValueError('schema_version must be 1')

    scene = config.get('scene')
    _require_mapping(scene, 'scene')
    _require_keys(scene, _SCENE_KEYS, 'scene')
    if not isinstance(scene.get('frame_id'), str) or not scene['frame_id']:
        raise ValueError('scene.frame_id must be a non-empty string')
    if not isinstance(scene.get('semantic_label'), str) or \
            not scene['semantic_label']:
        raise ValueError('scene.semantic_label must be a non-empty string')

    backend = config.get('backend')
    _require_mapping(backend, 'backend')
    _require_keys(backend, _BACKEND_KEYS, 'backend')
    if backend.get('id') not in SUPPORTED_BACKENDS:
        raise ValueError('unsupported backend id: %s' % backend.get('id'))
    if not isinstance(backend.get('version'), str) or not backend['version']:
        raise ValueError('backend.version must be a non-empty string')

    foreground = backend.get('foreground')
    _require_mapping(foreground, 'backend.foreground')
    _require_keys(foreground, _FOREGROUND_KEYS, 'backend.foreground')
    if foreground.get('mode') not in ('border_color', 'otsu_gray'):
        raise ValueError(
            'backend.foreground.mode must be border_color or otsu_gray')
    if foreground.get('polarity') not in ('bright', 'dark'):
        raise ValueError('backend.foreground.polarity must be bright or dark')
    border_fraction = _number(foreground.get('border_fraction'),
                              'border_fraction')
    if not 0.01 <= border_fraction <= 0.25:
        raise ValueError('border_fraction must be in [0.01, 0.25]')
    if _number(foreground.get('border_mad_max'), 'border_mad_max') <= 0.0:
        raise ValueError('border_mad_max must be positive')
    threshold_scale = _number(foreground.get('threshold_scale'),
                              'threshold_scale')
    if not 0.5 <= threshold_scale <= 1.5:
        raise ValueError('threshold_scale must be in [0.5, 1.5]')
    if _number(foreground.get('clahe_clip_limit'),
               'clahe_clip_limit') < 0.0:
        raise ValueError('clahe_clip_limit must be non-negative')
    for key in ('blur_kernel', 'open_kernel', 'close_kernel'):
        kernel = _positive_int(foreground.get(key), key)
        if kernel % 2 == 0:
            raise ValueError('%s must be odd' % key)
    _positive_int(foreground.get('clahe_tile_grid'), 'clahe_tile_grid')
    _positive_int(foreground.get('min_area_px'), 'min_area_px')
    max_fraction = _number(foreground.get('max_area_fraction'),
                           'max_area_fraction')
    if not 0.0 < max_fraction <= 1.0:
        raise ValueError('max_area_fraction must be in (0, 1]')
    min_fraction = _number(foreground.get('min_foreground_fraction'),
                           'min_foreground_fraction')
    if not 0.0 <= min_fraction < 1.0:
        raise ValueError('min_foreground_fraction must be in [0, 1)')
    if _number(foreground.get('min_contrast'), 'min_contrast') < 0.0:
        raise ValueError('min_contrast must be non-negative')

    watershed = backend.get('watershed')
    _require_mapping(watershed, 'backend.watershed')
    _require_keys(watershed, _WATERSHED_KEYS, 'backend.watershed')
    if watershed.get('peak_mode') not in ('global', 'local'):
        raise ValueError('backend.watershed.peak_mode must be global or local')
    peak_ratio = _number(watershed.get('peak_ratio'), 'peak_ratio')
    if not 0.0 < peak_ratio <= 1.0:
        raise ValueError('peak_ratio must be in (0, 1]')
    peak_window = _positive_int(watershed.get('peak_window'), 'peak_window')
    if peak_window % 2 == 0 or peak_window < 3:
        raise ValueError('peak_window must be odd and at least 3')
    if _number(watershed.get('min_peak_px'), 'min_peak_px') < 0.0:
        raise ValueError('min_peak_px must be non-negative')
    gradient_blur = _positive_int(watershed.get('gradient_blur'),
                                  'gradient_blur')
    if gradient_blur % 2 == 0:
        raise ValueError('gradient_blur must be odd')
    _positive_int(watershed.get('adjacency_px'), 'adjacency_px')
    _positive_int(watershed.get('adjacency_max_instances'),
                  'adjacency_max_instances')

    score = backend.get('score')
    _require_mapping(score, 'backend.score')
    _require_keys(score, _SCORE_KEYS, 'backend.score')
    area_reference = _number(score.get('area_reference_fraction'),
                             'area_reference_fraction')
    if not 0.0 < area_reference <= 1.0:
        raise ValueError('area_reference_fraction must be in (0, 1]')

    evaluation = config.get('evaluation')
    _require_mapping(evaluation, 'evaluation')
    _require_keys(evaluation, _EVALUATION_KEYS, 'evaluation')
    iou_min = _number(evaluation.get('iou_min'), 'iou_min')
    iou_max = _number(evaluation.get('iou_max'), 'iou_max')
    iou_step = _number(evaluation.get('iou_step'), 'iou_step')
    if not (0.0 < iou_min <= iou_max <= 1.0):
        raise ValueError('iou thresholds must satisfy 0 < min <= max <= 1')
    if iou_step <= 0.0:
        raise ValueError('iou_step must be positive')
    score_threshold = _number(evaluation.get('score_threshold'),
                              'score_threshold')
    pr_score = _number(evaluation.get('pr_score_threshold'),
                       'pr_score_threshold')
    if not 0.0 <= score_threshold <= 1.0:
        raise ValueError('score_threshold must be in [0, 1]')
    if not 0.0 <= pr_score <= 1.0:
        raise ValueError('pr_score_threshold must be in [0, 1]')
    pr_iou = _number(evaluation.get('pr_iou'), 'pr_iou')
    if not 0.0 < pr_iou <= 1.0:
        raise ValueError('pr_iou must be in (0, 1]')

    output = config.get('output')
    _require_mapping(output, 'output')
    _require_keys(output, _OUTPUT_KEYS, 'output')
    if not isinstance(output.get('overlays'), bool):
        raise ValueError('output.overlays must be a boolean')
    return config


def load_config(path=None):
    """Load, merge and validate a YAML configuration.

    ``None`` returns the validated built-in default configuration.
    """
    if path is None:
        return validate_config(copy.deepcopy(DEFAULT_CONFIG))
    with open(path, 'rb') as handle:
        loaded = yaml.safe_load(handle)
    if loaded is None:
        loaded = {}
    if not isinstance(loaded, dict):
        raise ValueError('configuration root must be a mapping')
    merged = _deep_merge(DEFAULT_CONFIG, loaded)
    return validate_config(merged)


def config_sha256(config):
    """Return a stable SHA-256 over the effective configuration."""
    payload = json.dumps(
        config, sort_keys=True, separators=(',', ':'), ensure_ascii=True)
    return sha256_bytes(payload.encode('utf-8'))
