"""
End-to-end relation evaluation with predicted masks.

GT relation edges are derived from GT masks; predicted edges are derived from
the backend's masks after greedy IoU matching to GT instances. This measures
segmentation-error propagation into relations, but it is still synthetic and
uses a mask-derived GT definition, not independent human relation labels.
"""

import dataclasses
import os

import cv2
import numpy as np

from .dataset import load_ground_truth
from .maskops import mask_iou
from .relations import RelationParams, build_relation_graph, compare_relation_sets
from .types import InstanceMask, SEMANTIC_LABEL


@dataclasses.dataclass(frozen=True)
class RelationEvalParams:
    """Relation search plus instance-matching threshold."""

    relation: RelationParams = dataclasses.field(
        default_factory=RelationParams)
    match_iou: float = 0.5

    def __post_init__(self):
        """Validate the matching threshold."""
        if not 0.0 < float(self.match_iou) <= 1.0:
            raise ValueError('match_iou must be in (0, 1]')


def _instances_from_label(label, items):
    instances = []
    for item in items:
        mask = label == int(item['mask_index'])
        if not np.any(mask):
            continue
        instances.append(InstanceMask(
            instance_id=str(item['instance_id']),
            mask=mask,
            semantic_label=SEMANTIC_LABEL,
            score=float(item.get('detection_confidence', 1.0)),
        ))
    return instances


def _match_instances(predicted, truth, threshold):
    """Greedy one-to-one matching by mask IoU; returns predicted->truth ids."""
    used = [False] * len(truth)
    mapping = {}
    matches = []
    for pred in sorted(predicted, key=lambda item: item.instance_id):
        best_iou = 0.0
        best_index = -1
        for index, gt in enumerate(truth):
            if used[index]:
                continue
            value = mask_iou(pred.mask, gt.mask)
            if value > best_iou:
                best_iou = value
                best_index = index
        if best_index >= 0 and best_iou >= threshold:
            used[best_index] = True
            mapping[pred.instance_id] = truth[best_index].instance_id
            matches.append({
                'predicted_id': pred.instance_id,
                'truth_id': truth[best_index].instance_id,
                'iou': round(best_iou, 6),
            })
    return mapping, matches


def _contact_edges(edges, mapping=None):
    """Map touching/near to contact, optionally remapping instance IDs."""
    result = []
    for edge in edges:
        subject = edge.subject_id
        obj = edge.object_id
        if mapping is not None:
            subject = mapping.get(subject)
            obj = mapping.get(obj)
            if subject is None or obj is None:
                continue
        result.append({
            'subject_id': subject,
            'object_id': obj,
            'predicate': 'contact',
            'predicate_2d': edge.predicate,
        })
    return result


def evaluate_predicted_relations(dataset, records, prediction_root,
                                 params=None):
    """Evaluate predicted-mask relation edges against GT-mask edges."""
    params = params or RelationEvalParams()
    by_id = {str(record['scene_id']): record for record in records}
    predicted_all = []
    truth_all = []
    failed = []
    per_scene = {}
    total_matches = 0
    total_gt = 0
    total_pred = 0
    for scene in dataset.scenes:
        record = by_id.get(scene.scene_id)
        if record is None or not record.get('validity', {}).get('valid', False):
            failed.append({'scene_id': scene.scene_id,
                           'reason': 'missing_or_invalid_prediction'})
            continue
        try:
            gt_label = load_ground_truth(scene)
            gt_instances = _instances_from_label(gt_label, scene.instances)
            if record['instances']:
                pred_label = cv2.imread(
                    os.path.join(
                        prediction_root,
                        record['instances'][0]['mask_path']),
                    cv2.IMREAD_UNCHANGED)
                if pred_label is None:
                    raise ValueError('cannot read predicted label')
                if pred_label.shape != (scene.height, scene.width):
                    raise ValueError(
                        'predicted label shape does not match scene')
                pred_instances = _instances_from_label(
                    pred_label, record['instances'])
            else:
                pred_instances = []
        except (ValueError, OSError) as error:
            failed.append({'scene_id': scene.scene_id, 'reason': str(error)})
            continue
        mapping, matches = _match_instances(
            pred_instances, gt_instances, params.match_iou)
        predicted_edges = build_relation_graph(
            pred_instances, params.relation)
        truth_edges = build_relation_graph(gt_instances, params.relation)
        predicted_contact = _contact_edges(predicted_edges, mapping)
        truth_contact = _contact_edges(truth_edges)
        predicted_all.extend(predicted_contact)
        truth_all.extend(truth_contact)
        per_scene[scene.scene_id] = compare_relation_sets(
            predicted_contact, truth_contact)
        per_scene[scene.scene_id]['matches'] = matches
        total_matches += len(matches)
        total_gt += len(gt_instances)
        total_pred += len(pred_instances)
    result = compare_relation_sets(predicted_all, truth_all)
    result.update({
        'schema_version': 'jcheng.relation_predicted_eval.v1',
        'dataset_id': dataset.dataset_id,
        'data_source': dataset.source,
        'evaluated_scenes': len(dataset.scenes) - len(failed),
        'failed_scenes': failed,
        'instance_matching': {
            'matched': total_matches,
            'gt_instances': total_gt,
            'predicted_instances': total_pred,
            'match_rate': total_matches / float(max(total_gt, 1)),
            'match_iou': params.match_iou,
        },
        'relation_params': {
            'adjacency_px': params.relation.adjacency_px,
            'touching_min_px': params.relation.touching_min_px,
            'max_instances': params.relation.max_instances,
        },
        'per_scene': per_scene,
        'hardware_validation': False,
        'real_ore_validation': False,
        'limitations': [
            'Synthetic data; GT edges are derived from GT masks, not '
            'independent human relation labels.',
            'Greedy IoU matching, not Hungarian; unmatched predictions are '
            'excluded from relation scoring.',
            'touching/near are collapsed to contact.',
            'No real-ore or hardware validation.',
        ],
    })
    return result
