"""
Scene-state assembly: instances plus evidence-backed 2-D relations.

S1 only emits adjacency. It deliberately does not emit an occlusion direction
or an ``occludes`` relation: a single RGB image does not contain enough
evidence to order two overlapping fragments. The field stays ``unknown`` until
S2 adds depth/geometry and S4 adds multi-view association.
"""

from .backends.base import BackendInfo
from .maskops import boundary_pixels, contact_pixels, mask_iou
from .types import (
    ERROR_MASK_SHAPE,
    SCENE_FIELD_STATUS,
    Relation,
    SceneState,
)


def _adjacency_relations(instances, radius):
    """Return undirected adjacency relations for close instances."""
    relations = []
    boundary = {
        instance.instance_id: boundary_pixels(instance.mask)
        for instance in instances
    }
    for first_index in range(len(instances)):
        first = instances[first_index]
        for second_index in range(first_index + 1, len(instances)):
            second = instances[second_index]
            contact = contact_pixels(first.mask, second.mask, radius=radius)
            if contact <= 0:
                continue
            denom = max(boundary[first.instance_id], 1)
            confidence = min(1.0, contact / float(denom))
            relations.append(Relation(
                subject_id=first.instance_id,
                predicate='adjacent',
                object_id=second.instance_id,
                confidence=confidence,
                evidence={
                    'contact_px': contact,
                    'subject_boundary_px': boundary[first.instance_id],
                    'object_boundary_px': boundary[second.instance_id],
                    'search_radius_px': int(radius),
                    'mask_iou': mask_iou(first.mask, second.mask),
                    'depth_order': 'unknown',
                    'note': 'undirected 2-D adjacency; no occlusion direction',
                },
                source='2d_mask',
            ))
    return relations


def build_scene_state(scene_id, timestamp_ns, frame_id, image, image_path,
                      image_sha256, instances, config, backend_info,
                      observation_source, errors=None, timing=None,
                      disjoint_verified=False):
    """Assemble a :class:`SceneState` from backend masks.

    The masks are checked for shape. ``disjoint_verified`` is set by the
    pipeline after it has compared the summed mask areas with the covered
    label-image area; this avoids an expensive per-pair full-resolution check.
    Without that flag, :class:`SceneState` performs the disjointness check.
    """
    if not isinstance(backend_info, BackendInfo):
        raise TypeError('backend_info must be a BackendInfo')
    height, width = image.shape[:2]
    for instance in instances:
        if instance.mask.shape != (height, width):
            raise ValueError(ERROR_MASK_SHAPE + ': mask shape mismatch')

    adjacency_radius = int(config['backend']['watershed']['adjacency_px'])
    adjacency_limit = int(
        config['backend']['watershed']['adjacency_max_instances'])
    field_status = dict(SCENE_FIELD_STATUS)
    if len(instances) <= adjacency_limit:
        relations = _adjacency_relations(instances, adjacency_radius)
    else:
        relations = []
        field_status['adjacency'] = 'not_computed_density_limit'
    scene = SceneState(
        scene_id=scene_id,
        timestamp_ns=int(timestamp_ns),
        frame_id=frame_id,
        image_width=width,
        image_height=height,
        instances=list(instances),
        relations=relations,
        observation_source=dict(observation_source),
        backend=backend_info.to_dict(),
        image_path=image_path,
        image_sha256=image_sha256,
        errors=list(errors or []),
        timing=dict(timing or {}),
        field_status=field_status,
        disjoint_verified=bool(disjoint_verified),
    )
    return scene
