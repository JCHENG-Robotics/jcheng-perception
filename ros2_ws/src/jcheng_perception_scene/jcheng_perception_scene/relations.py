"""
Evidence-backed 2-D relation graph for Scene State v2.

S3 prototype predicates:

* ``touching`` — visible masks have at least ``touching_min_px`` contact pixels
  within ``adjacency_px``;
* ``near`` — masks are within ``adjacency_px`` but below the touching threshold.

``occludes`` is deliberately not emitted: a single RGB view has no reliable
depth order. Every edge carries its contact evidence; no edge is invented from
proximity alone.
"""

import dataclasses
from typing import Tuple

import numpy as np

from .maskops import (
    boundary_pixels,
    boundary_pixels_local,
    bbox_intersects,
    contact_pixels,
    contact_pixels_local,
    iou_local,
    mask_iou,
)
from .types import Relation


@dataclasses.dataclass(frozen=True)
class LocalInstance:
    """An instance stored as a tight bbox plus a local boolean mask."""

    instance_id: str
    bbox_xyxy: Tuple[int, int, int, int]
    mask: np.ndarray

    def __post_init__(self):
        """Validate the local mask and bbox."""
        mask = np.asarray(self.mask).astype(bool, copy=False)
        if mask.ndim != 2 or not np.any(mask):
            raise ValueError('local instance mask must be non-empty 2-D')
        if len(self.bbox_xyxy) != 4:
            raise ValueError('bbox_xyxy must have four values')
        if mask.shape != (
                self.bbox_xyxy[3] - self.bbox_xyxy[1] + 1,
                self.bbox_xyxy[2] - self.bbox_xyxy[0] + 1):
            raise ValueError('local mask shape does not match bbox')
        object.__setattr__(self, 'mask', mask)


@dataclasses.dataclass(frozen=True)
class RelationParams:
    """Search radius, contact threshold and density bound."""

    adjacency_px: int = 5
    touching_min_px: int = 8
    max_instances: int = 128

    def __post_init__(self):
        """Validate parameters."""
        if int(self.adjacency_px) < 1:
            raise ValueError('adjacency_px must be >= 1')
        if int(self.touching_min_px) < 0:
            raise ValueError('touching_min_px must be >= 0')
        if int(self.max_instances) < 1:
            raise ValueError('max_instances must be >= 1')


def build_relation_graph(instances, params=None):
    """Return touching/near edges for a list of disjoint instance masks.

    Above ``max_instances`` the graph is not computed (empty list); callers
    must label that explicitly rather than interpreting it as "no relations".
    """
    params = params or RelationParams()
    if len(instances) > int(params.max_instances):
        return []
    boundary = {
        instance.instance_id: boundary_pixels(instance.mask)
        for instance in instances
    }
    edges = []
    for first_index in range(len(instances)):
        first = instances[first_index]
        for second_index in range(first_index + 1, len(instances)):
            second = instances[second_index]
            contact = contact_pixels(
                first.mask, second.mask, radius=int(params.adjacency_px))
            if contact <= 0:
                continue
            smaller_boundary = max(
                1, min(boundary[first.instance_id],
                       boundary[second.instance_id]))
            contact_ratio = contact / float(smaller_boundary)
            predicate = (
                'touching'
                if contact >= int(params.touching_min_px) else 'near'
            )
            confidence = min(1.0, 2.0 * contact_ratio)
            edges.append(Relation(
                subject_id=first.instance_id,
                predicate=predicate,
                object_id=second.instance_id,
                confidence=confidence,
                evidence={
                    'contact_px': int(contact),
                    'subject_boundary_px': boundary[first.instance_id],
                    'object_boundary_px': boundary[second.instance_id],
                    'contact_ratio': round(contact_ratio, 6),
                    'search_radius_px': int(params.adjacency_px),
                    'touching_min_px': int(params.touching_min_px),
                    'mask_iou': mask_iou(first.mask, second.mask),
                    'depth_order': 'unknown',
                    'directed': False,
                    'note': (
                        'undirected 2-D contact; no occlusion direction'),
                },
                source='2d_mask',
            ))
    return edges


def build_relation_graph_local(instances, params=None):
    """Dense-scene variant: bbox prefilter plus local mask operations.

    ``instances`` must be :class:`LocalInstance` objects. The full-image
    variant is kept for small/simple inputs and existing tests.
    """
    params = params or RelationParams()
    if len(instances) > int(params.max_instances):
        return []
    boundary = {
        instance.instance_id: boundary_pixels_local(
            instance.bbox_xyxy, instance.mask)
        for instance in instances
    }
    edges = []
    for first_index in range(len(instances)):
        first = instances[first_index]
        for second_index in range(first_index + 1, len(instances)):
            second = instances[second_index]
            if not bbox_intersects(
                    first.bbox_xyxy, second.bbox_xyxy,
                    margin=int(params.adjacency_px)):
                continue
            contact = contact_pixels_local(
                first.bbox_xyxy, first.mask,
                second.bbox_xyxy, second.mask,
                radius=int(params.adjacency_px))
            if contact <= 0:
                continue
            smaller_boundary = max(
                1, min(boundary[first.instance_id],
                       boundary[second.instance_id]))
            contact_ratio = contact / float(smaller_boundary)
            predicate = (
                'touching'
                if contact >= int(params.touching_min_px) else 'near')
            edges.append(Relation(
                subject_id=first.instance_id,
                predicate=predicate,
                object_id=second.instance_id,
                confidence=min(1.0, 2.0 * contact_ratio),
                evidence={
                    'contact_px': int(contact),
                    'subject_boundary_px': boundary[first.instance_id],
                    'object_boundary_px': boundary[second.instance_id],
                    'contact_ratio': round(contact_ratio, 6),
                    'search_radius_px': int(params.adjacency_px),
                    'touching_min_px': int(params.touching_min_px),
                    'mask_iou': iou_local(
                        first.bbox_xyxy, first.mask,
                        second.bbox_xyxy, second.mask),
                    'depth_order': 'unknown',
                    'directed': False,
                    'note': (
                        'undirected 2-D contact; no occlusion direction'),
                },
                source='2d_mask',
            ))
    return edges


def relation_key(relation):
    """Return a stable, order-insensitive key for an edge."""
    if isinstance(relation, Relation):
        subject, predicate, obj = (
            relation.subject_id, relation.predicate, relation.object_id)
    else:
        subject, predicate, obj = (
            relation['subject_id'], relation['predicate'],
            relation['object_id'])
    first, second = sorted((str(subject), str(obj)))
    return (first, str(predicate), second)


def compare_relation_sets(predicted, truth):
    """Return precision/recall/F1 for two iterables of relations.

    Edges are matched by unordered instance pair + predicate; duplicate edges
    are collapsed. This is a utility for independent pair labels, not a claim
    that the synthetic label is independent.
    """
    predicted_keys = {relation_key(edge) for edge in predicted}
    truth_keys = {relation_key(edge) for edge in truth}
    true_positive = len(predicted_keys & truth_keys)
    precision = (
        true_positive / float(len(predicted_keys))
        if predicted_keys else None)
    recall = (
        true_positive / float(len(truth_keys))
        if truth_keys else None)
    if precision is None or recall is None or precision + recall == 0.0:
        f1 = None
    else:
        f1 = 2.0 * precision * recall / (precision + recall)
    return {
        'predicted_edges': len(predicted_keys),
        'truth_edges': len(truth_keys),
        'true_positive': true_positive,
        'precision': precision,
        'recall': recall,
        'f1': f1,
    }


def relation_keys_by_predicate(relations):
    """Return predicate -> set of unordered pair keys."""
    grouped = {}
    for edge in relations:
        key = relation_key(edge)
        grouped.setdefault(key[1], set()).add(key)
    return grouped
