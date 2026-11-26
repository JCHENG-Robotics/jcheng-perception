"""
COCO-style mask evaluation for the offline ore pipeline.

Masks are kept as ``(bbox, local_mask)`` pairs so a 5 MP image with >1000
instances does not allocate a full-resolution boolean mask per instance. The
IoU matrix is computed once per scene and reused for every threshold.

Reported numbers carry their provenance. A synthetic result is never labelled
as real-ore accuracy, and a missing ground truth produces ``None`` rather than
a fabricated perfect score.
"""

import os

import cv2
import numpy as np

from .dataset import load_ground_truth
from .maskops import bbox_intersects, iou_local
from .types import EVALUATION_SCHEMA_VERSION, sha256_file

LIMITATIONS = [
    'Synthetic ground truth measures pipeline mechanics, not real-ore accuracy.',
    'Scores are heuristic and uncalibrated; precision/recall at a fixed score '
    'threshold are not operating-point guarantees.',
    'Occlusion strata are scene-level labels supplied by the synthetic '
    'generator; public datasets without such labels are not stratified.',
    'Fully occluded zero-area instances are excluded from the ground-truth '
    'count because they have no observable mask.',
]


def iou_thresholds(config):
    """Return the inclusive IoU threshold list from the configuration."""
    minimum = float(config['evaluation']['iou_min'])
    maximum = float(config['evaluation']['iou_max'])
    step = float(config['evaluation']['iou_step'])
    values = []
    value = minimum
    guard = 0
    while value <= maximum + 1e-9 and guard < 1000:
        values.append(round(value, 6))
        value += step
        guard += 1
    if not values:
        raise ValueError('empty IoU threshold list')
    return values


def _clamp_bbox(bbox, height, width):
    if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
        raise ValueError('instance bbox must have four values')
    x0, y0, x1, y1 = (int(value) for value in bbox)
    x0 = max(0, min(width - 1, x0))
    y0 = max(0, min(height - 1, y0))
    x1 = max(x0, min(width - 1, x1))
    y1 = max(y0, min(height - 1, y1))
    return (x0, y0, x1, y1)


def _prediction_masks(record, prediction_root):
    """Load predicted masks as ``(score, bbox, local_mask)`` triples."""
    instances = record.get('instances', [])
    if not instances:
        return [], 0, 0
    label_images = {}
    predictions = []
    height = int(record.get('image', {}).get('height', 0))
    width = int(record.get('image', {}).get('width', 0))
    for instance in instances:
        path = instance.get('mask_path')
        index = int(instance.get('mask_index', 0))
        if not path:
            raise ValueError('instance %s has no mask_path' %
                             instance.get('instance_id'))
        if index <= 0:
            raise ValueError('instance %s has invalid mask_index' %
                             instance.get('instance_id'))
        if height <= 0 or width <= 0:
            raise ValueError('prediction record has no image dimensions')
        bbox = _clamp_bbox(instance.get('bbox_xyxy'), height, width)
        absolute = path if os.path.isabs(path) else os.path.join(
            prediction_root, path)
        if absolute not in label_images:
            label_image = cv2.imread(absolute, cv2.IMREAD_UNCHANGED)
            if label_image is None:
                raise ValueError('cannot read predicted mask: %s' % absolute)
            if label_image.ndim != 2:
                raise ValueError('predicted mask must be single-channel')
            if label_image.shape != (height, width):
                raise ValueError(
                    'predicted mask shape %s != image %dx%d' % (
                        label_image.shape, width, height))
            label_images[absolute] = label_image
        local = label_images[absolute][
            bbox[1]:bbox[3] + 1, bbox[0]:bbox[2] + 1] == index
        if not np.any(local):
            raise ValueError(
                'predicted mask %s has no pixels for index %d in bbox %s' %
                (absolute, index, bbox))
        predictions.append((
            float(instance.get('detection_confidence', 0.0)), bbox, local))
    return predictions, height, width


def _gt_masks(scene):
    """Return ``(truths, height, width)`` for observable GT instances."""
    label_image = load_ground_truth(scene)
    height, width = label_image.shape[:2]
    truths = []
    for instance in scene.instances:
        index = int(instance['mask_index'])
        bbox = _clamp_bbox(instance.get('bbox_xyxy'), height, width)
        local = label_image[
            bbox[1]:bbox[3] + 1, bbox[0]:bbox[2] + 1] == index
        if not np.any(local):
            # Fully occluded / empty annotation: no observable mask.
            continue
        truths.append((bbox, local, str(
            instance.get('stratum', 'unlabelled'))))
    return truths, height, width


def _scene_ious(predictions, truths):
    """Compute IoUs only for prediction/truth pairs whose boxes intersect."""
    matrix = {}
    for prediction_index, (_, prediction_bbox, _) in enumerate(predictions):
        for truth_index, (truth_bbox, _, _) in enumerate(truths):
            if not bbox_intersects(prediction_bbox, truth_bbox):
                continue
            value = iou_local(
                prediction_bbox, predictions[prediction_index][2],
                truth_bbox, truths[truth_index][1])
            if value > 0.0:
                matrix[(prediction_index, truth_index)] = value
    return matrix


def _match(prediction_indices, predictions, truths, threshold, matrix):
    """Greedily match prediction indices to truths at one IoU threshold."""
    used = [False] * len(truths)
    entries = []
    ordered = sorted(
        prediction_indices,
        key=lambda index: -predictions[index][0])
    for prediction_index in ordered:
        best_iou = 0.0
        best_index = -1
        for truth_index in range(len(truths)):
            if used[truth_index]:
                continue
            value = matrix.get((prediction_index, truth_index), 0.0)
            if value > best_iou:
                best_iou = value
                best_index = truth_index
        if best_index >= 0 and best_iou >= threshold:
            used[best_index] = True
            entries.append((predictions[prediction_index][0], True))
        else:
            entries.append((predictions[prediction_index][0], False))
    return entries


def _average_precision(entries, gt_count):
    """Return all-point interpolated AP, or ``None`` when there is no GT."""
    if gt_count <= 0:
        return None
    if not entries:
        return 0.0
    ordered = sorted(entries, key=lambda item: -item[0])
    true_positive = 0
    false_positive = 0
    precisions = []
    recalls = []
    for _, is_true_positive in ordered:
        if is_true_positive:
            true_positive += 1
        else:
            false_positive += 1
        precisions.append(true_positive / float(
            true_positive + false_positive))
        recalls.append(true_positive / float(gt_count))
    mrec = [0.0] + recalls + [1.0]
    mpre = [0.0] + precisions + [0.0]
    for index in range(len(mpre) - 2, -1, -1):
        mpre[index] = max(mpre[index], mpre[index + 1])
    area = 0.0
    for index in range(1, len(mrec)):
        if mrec[index] != mrec[index - 1]:
            area += (mrec[index] - mrec[index - 1]) * mpre[index]
    return float(area)


def _percentile(values, percentile):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64),
                               percentile))


class _Accumulator:
    """Accumulate per-scene matching statistics for one scene subset."""

    def __init__(self, thresholds, pr_iou, pr_score):
        self.thresholds = thresholds
        self.pr_iou = pr_iou
        self.pr_score = pr_score
        self.per_threshold = {threshold: [] for threshold in thresholds}
        self.gt_count = 0
        self.pred_count = 0
        self.count_errors = []
        self.scenes = 0
        self.pr_tp = 0
        self.pr_fp = 0
        self.pr_fn = 0

    def add(self, predictions, truths):
        self.scenes += 1
        self.gt_count += len(truths)
        self.pred_count += len(predictions)
        self.count_errors.append(abs(len(predictions) - len(truths)))
        matrix = _scene_ious(predictions, truths)
        all_indices = list(range(len(predictions)))
        for threshold in self.thresholds:
            self.per_threshold[threshold].extend(
                _match(all_indices, predictions, truths, threshold, matrix))

        # Operating point: score >= pr_score, IoU >= pr_iou.
        selected = [
            index for index, prediction in enumerate(predictions)
            if prediction[0] >= self.pr_score
        ]
        matched = _match(selected, predictions, truths, self.pr_iou, matrix)
        hits = sum(1 for _, hit in matched if hit)
        self.pr_tp += hits
        self.pr_fp += len(matched) - hits
        self.pr_fn += len(truths) - hits

    def result(self):
        ap_by_threshold = {}
        for threshold, entries in self.per_threshold.items():
            ap_by_threshold['%.2f' % threshold] = _average_precision(
                entries, self.gt_count)
        ap50 = ap_by_threshold.get('0.50')
        ap_values = [
            value for value in ap_by_threshold.values()
            if value is not None
        ]
        ap50_95 = float(np.mean(ap_values)) if ap_values else None
        precision = None
        recall = None
        if self.pr_tp + self.pr_fp > 0:
            precision = self.pr_tp / float(self.pr_tp + self.pr_fp)
        if self.pr_tp + self.pr_fn > 0:
            recall = self.pr_tp / float(self.pr_tp + self.pr_fn)
        count_mae = (
            float(np.mean(self.count_errors)) if self.count_errors else None)
        count_relative = (
            float(sum(self.count_errors) / float(max(self.gt_count, 1)))
            if self.count_errors else None)
        return {
            'scenes': self.scenes,
            'gt_instances': self.gt_count,
            'predicted_instances': self.pred_count,
            'mask_ap50': ap50,
            'mask_ap50_95': ap50_95,
            'mask_ap_by_threshold': ap_by_threshold,
            'precision_at_operating_point': precision,
            'recall_at_operating_point': recall,
            'instance_count_mae': count_mae,
            'instance_count_relative_mae': count_relative,
        }


def evaluate(dataset, records, prediction_root, config):
    """Evaluate scene-state records against a dataset's ground truth.

    Scenes whose prediction record is missing, invalid or unreadable are
    excluded and reported in ``failed_scenes``. They are never counted as
    correct.
    """
    thresholds = iou_thresholds(config)
    pr_iou = float(config['evaluation']['pr_iou'])
    pr_score = float(config['evaluation']['pr_score_threshold'])
    score_threshold = float(config['evaluation']['score_threshold'])
    by_id = {str(record['scene_id']): record for record in records}
    subset_definitions = {
        'by_illumination': ('illumination', ('dark', 'normal', 'bright')),
        'by_density': ('density', ('sparse', 'medium', 'dense')),
        'by_occlusion': ('occlusion_class', ('none', 'touching', 'occluded')),
    }

    overall = _Accumulator(thresholds, pr_iou, pr_score)
    subsets = {
        name: {value: _Accumulator(thresholds, pr_iou, pr_score)
               for value in values}
        for name, (_, values) in subset_definitions.items()
    }
    failed = []
    latencies = []
    missing_scenes = []
    for scene in dataset.scenes:
        record = by_id.get(scene.scene_id)
        if record is None:
            missing_scenes.append(scene.scene_id)
            continue
        validity = record.get('validity', {})
        if not validity.get('valid', False):
            failed.append({
                'scene_id': scene.scene_id,
                'reason': validity.get('failure_reason'),
            })
            continue
        try:
            predictions, _, _ = _prediction_masks(record, prediction_root)
            predictions = [
                prediction for prediction in predictions
                if prediction[0] >= score_threshold
            ]
            truths, _, _ = _gt_masks(scene)
        except ValueError as error:
            failed.append({
                'scene_id': scene.scene_id,
                'reason': str(error),
            })
            continue
        overall.add(predictions, truths)
        timing = record.get('timing') or {}
        if 'segmentation_ms' in timing:
            latencies.append(float(timing['segmentation_ms']))
        if truths:
            attributes = scene.attributes
            for name, (key, values) in subset_definitions.items():
                value = attributes.get(key)
                if value in subsets[name]:
                    subsets[name][value].add(predictions, truths)

    result = overall.result()
    result.update({
        'schema_version': EVALUATION_SCHEMA_VERSION,
        'dataset_id': dataset.dataset_id,
        'data_source': dataset.source,
        'annotation_method': dataset.annotation_method,
        'dataset_license': dataset.license,
        'dataset_manifest_sha256': sha256_file(dataset.manifest_path),
        'iou_thresholds': thresholds,
        'score_threshold_for_ap': score_threshold,
        'operating_point': {
            'iou': pr_iou,
            'score_threshold': pr_score,
        },
        'failed_scenes': failed,
        'missing_prediction_scenes': missing_scenes,
        'latency_ms': {
            'p50': _percentile(latencies, 50),
            'p95': _percentile(latencies, 95),
            'max': max(latencies) if latencies else None,
            'n': len(latencies),
        },
        'by_illumination': {
            value: accumulator.result()
            for value, accumulator in subsets['by_illumination'].items()
            if accumulator.scenes > 0
        },
        'by_density': {
            value: accumulator.result()
            for value, accumulator in subsets['by_density'].items()
            if accumulator.scenes > 0
        },
        'by_occlusion': {
            value: accumulator.result()
            for value, accumulator in subsets['by_occlusion'].items()
            if accumulator.scenes > 0
        },
        'hardware_validation': False,
        'real_ore_validation': False,
        'limitations': list(LIMITATIONS),
    })
    return result
