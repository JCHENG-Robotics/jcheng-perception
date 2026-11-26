"""
Scene State v2 prototype: attach visible-geometry evidence to a v1 scene.

v1 stays frozen and readable. v2 changes only the per-instance ``geometry``
object (plus the schema version). Relations, instance identity and 3-D scene
graphs remain S3 work; this module is the geometry hand-off, not a complete
scene graph.
"""

import copy

from .geometry import visible_geometry

SCENE_V2_SCHEMA_VERSION = 'jcheng.scene_state.v2'

_GEOMETRY_STATUSES = (
    'computed', 'insufficient_depth', 'invalid_mask', 'not_computed',
)


def not_computed_geometry(frame_id='base_link', reason='geometry_not_run'):
    """Return an explicit no-geometry record; never a zero measurement."""
    return {
        'status': 'not_computed',
        'reason': reason,
        'frame_id': frame_id,
        'visible_point_count': None,
        'depth_valid_fraction': None,
        'visible_centroid_base': None,
        'visible_centroid_covariance': None,
        'visible_aabb': None,
        'visible_obb': None,
        'field_status': {
            'visible_centroid_base': 'unknown',
            'visible_centroid_covariance': 'unknown',
            'visible_aabb': 'unknown',
            'visible_obb': 'unknown',
            'true_centroid': 'unknown',
            'full_extent': 'unknown',
            'volume': 'unknown',
        },
        'unknowns': [
            'true_centroid', 'full_extent', 'volume', 'mass',
            'material_composition', 'ore_grade', 'economic_value',
        ],
    }


def geometry_for_mask(mask, depth_m, camera_matrix, transform_base_camera,
                      params=None, frame_id='base_link'):
    """Shortcut for the per-instance geometry call."""
    return visible_geometry(
        mask, depth_m, camera_matrix, transform_base_camera,
        params=params, frame_id=frame_id)


def build_scene_v2(scene_v1, geometry_by_instance, frame_id='base_link'):
    """Return a v2 copy with geometry attached per instance.

    ``geometry_by_instance`` maps instance_id -> the output of
    :func:`visible_geometry`. Missing instances get an explicit
    ``not_computed`` geometry, never zeros.
    """
    if not isinstance(scene_v1, dict):
        raise TypeError('scene_v1 must be a mapping')
    if scene_v1.get('schema_version') != 'jcheng.scene_state.v1':
        raise ValueError('input must be a jcheng.scene_state.v1 record')
    if not isinstance(geometry_by_instance, dict):
        raise TypeError('geometry_by_instance must be a mapping')
    record = copy.deepcopy(scene_v1)
    record['schema_version'] = SCENE_V2_SCHEMA_VERSION
    for instance in record.get('instances', []):
        instance_id = instance.get('instance_id')
        geometry = geometry_by_instance.get(instance_id)
        if geometry is None:
            geometry = not_computed_geometry(
                frame_id=frame_id, reason='no_geometry_for_instance')
        instance['geometry'] = copy.deepcopy(geometry)
    record.setdefault('field_status', {})['geometry'] = 'algorithm_estimate'
    return record


def validate_scene_v2(record):
    """Return a list of structural errors for a v2 record."""
    errors = []
    if record.get('schema_version') != SCENE_V2_SCHEMA_VERSION:
        errors.append('schema_version must be %s' % SCENE_V2_SCHEMA_VERSION)
    instances = record.get('instances')
    if not isinstance(instances, list):
        return errors + ['instances must be a list']
    seen = set()
    for instance in instances:
        instance_id = instance.get('instance_id')
        if not instance_id or instance_id in seen:
            errors.append('missing/duplicate instance_id: %r' % instance_id)
        seen.add(instance_id)
        geometry = instance.get('geometry')
        if not isinstance(geometry, dict):
            errors.append('%s: geometry missing' % instance_id)
            continue
        status = geometry.get('status')
        if status not in _GEOMETRY_STATUSES:
            errors.append('%s: invalid geometry status %r' % (
                instance_id, status))
        for forbidden in ('true_centroid', 'full_extent', 'volume', 'mass',
                          'material_composition', 'ore_grade',
                          'economic_value'):
            if forbidden in instance:
                errors.append('%s: forbidden field %s' % (
                    instance_id, forbidden))
            if forbidden in geometry and geometry[forbidden] is not None:
                errors.append('%s: forbidden geometry field %s' % (
                    instance_id, forbidden))
        if status == 'computed':
            centroid = geometry.get('visible_centroid_base')
            covariance = geometry.get('visible_centroid_covariance')
            if not (isinstance(centroid, list) and len(centroid) == 3):
                errors.append('%s: invalid visible centroid' % instance_id)
            if not (isinstance(covariance, list) and len(covariance) == 3):
                errors.append('%s: invalid visible covariance' % instance_id)
            if geometry.get('visible_aabb') is None or \
                    geometry.get('visible_obb') is None:
                errors.append('%s: computed geometry lacks AABB/OBB' %
                              instance_id)
        if status != 'computed':
            if geometry.get('visible_centroid_base') is not None:
                errors.append('%s: non-computed geometry has centroid' %
                              instance_id)
    return errors
