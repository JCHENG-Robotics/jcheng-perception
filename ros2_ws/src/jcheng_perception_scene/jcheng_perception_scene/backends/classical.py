"""
Classical, weight-free ore-instance baselines.

``classical_watershed`` separates touching fragments with a distance-transform
marker watershed. ``classical_cc`` is the same foreground stage without
splitting; it exists only as an ablation that shows what the watershed adds.

Both are heuristic. Their scores are not calibrated probabilities, and no
accuracy claim on real ore is made or implied.
"""

import math

import cv2
import numpy as np

from ..types import InstanceMask
from .base import BackendInfo, SegmentationBackend


def _border_color_score(image_bgr, params):
    """Return a Lab colour-distance map and the border median absolute deviation.

    The belt is modelled from the top and bottom image bands. A large border
    MAD means the band is probably not belt (for example a pile that fills the
    frame), in which case the caller falls back to grayscale Otsu.
    """
    lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB).astype(np.float64)
    height = lab.shape[0]
    band = max(2, int(round(height * float(params['border_fraction']))))
    band = min(band, max(1, height // 3))
    border = np.concatenate(
        [lab[:band].reshape(-1, 3), lab[-band:].reshape(-1, 3)], axis=0)
    background = np.median(border, axis=0)
    distances = np.linalg.norm(lab - background[None, None, :], axis=2)
    spread = float(np.median(np.abs(border - background[None, :])))
    return np.clip(distances, 0.0, 255.0).astype(np.uint8), spread


def _gate(score, mask, params):
    """Return True when a candidate mask has enough contrast and support."""
    if not np.any(mask) or np.all(mask):
        return False
    mean_foreground = float(score[mask].mean())
    mean_background = float(score[~mask].mean())
    if abs(mean_foreground - mean_background) < float(
            params['min_contrast']):
        return False
    if float(np.mean(mask)) < float(params['min_foreground_fraction']):
        return False
    return True


def _clean(mask, params):
    """Apply morphology and remove components below ``min_area_px``."""
    work = mask.astype(np.uint8) * 255
    open_kernel = int(params['open_kernel'])
    if open_kernel > 1:
        work = cv2.morphologyEx(
            work, cv2.MORPH_OPEN,
            np.ones((open_kernel, open_kernel), np.uint8))
    close_kernel = int(params['close_kernel'])
    if close_kernel > 1:
        work = cv2.morphologyEx(
            work, cv2.MORPH_CLOSE,
            np.ones((close_kernel, close_kernel), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(work, 8)
    cleaned = np.zeros_like(work)
    min_area = int(params['min_area_px'])
    for label in range(1, count):
        if int(stats[label, cv2.CC_STAT_AREA]) >= min_area:
            cleaned[labels == label] = 255
    return cleaned > 0


def _foreground(image_bgr, config):
    """Return a cleaned foreground mask and the threshold used."""
    params = config['backend']['foreground']
    mode = params['mode']
    if mode == 'border_color':
        score, spread = _border_color_score(image_bgr, params)
        if spread <= float(params['border_mad_max']):
            threshold, _ = cv2.threshold(
                score, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            mask = score > threshold
            if not _gate(score, mask, params):
                return np.zeros(mask.shape, dtype=bool), float(threshold)
            return _clean(mask, params), float(threshold)

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    clip = float(params['clahe_clip_limit'])
    grid = int(params['clahe_tile_grid'])
    if clip > 0.0:
        clahe = cv2.createCLAHE(clipLimit=clip, tileGridSize=(grid, grid))
        enhanced = clahe.apply(gray)
    else:
        enhanced = gray
    kernel = int(params['blur_kernel'])
    blurred = cv2.GaussianBlur(enhanced, (kernel, kernel), 0)
    threshold, _ = cv2.threshold(
        blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    scale = float(params['threshold_scale'])
    if params['polarity'] == 'bright':
        threshold = float(threshold) * scale
        mask = blurred > threshold
    else:
        threshold = 255.0 - (255.0 - float(threshold)) * scale
        mask = blurred < threshold
    if not _gate(blurred, mask, params):
        return np.zeros(mask.shape, dtype=bool), float(threshold)
    return _clean(mask, params), float(threshold)


def _score_mask(mask, distance, config):
    """Return a heuristic confidence in ``[0.01, 0.99]`` and its components."""
    params = config['backend']['score']
    area = int(np.count_nonzero(mask))
    if area == 0:
        return 0.01, {'area_px': 0}
    mean_distance = float(distance[mask].mean())
    equivalent_radius = math.sqrt(area / math.pi)
    circularity = mean_distance / max(equivalent_radius, 1e-6)
    image_area = float(mask.shape[0] * mask.shape[1])
    reference = max(
        1.0, float(params['area_reference_fraction']) * image_area)
    area_term = min(1.0, area / reference)
    shape_term = min(1.0, circularity / 0.667)
    score = 0.65 * shape_term + 0.35 * area_term
    score = float(min(0.99, max(0.01, score)))
    return score, {
        'area_px': area,
        'mean_distance_px': round(mean_distance, 4),
        'circularity': round(circularity, 4),
        'shape_term': round(shape_term, 4),
        'area_term': round(area_term, 4),
    }


def _instances_from_labels(labels, foreground, distance, config, scene_id):
    """Convert a label image into filtered, scored instance masks."""
    params = config['backend']['foreground']
    min_area = int(params['min_area_px'])
    max_area = float(params['max_area_fraction']) * labels.size
    masks = []
    for label in sorted(set(int(value) for value in np.unique(labels))):
        if label <= 0:
            continue
        mask = (labels == label) & foreground
        area = int(np.count_nonzero(mask))
        if area < min_area or area > max_area:
            continue
        score, attributes = _score_mask(mask, distance, config)
        attributes['label'] = label
        masks.append(InstanceMask(
            instance_id='%s:%04d' % (scene_id, len(masks) + 1),
            mask=mask,
            score=score,
            attributes=attributes,
        ))
    masks.sort(key=lambda instance: -instance.score)
    return masks


class ClassicalWatershedBackend(SegmentationBackend):
    """Distance-marker watershed baseline for touching ore fragments."""

    info = BackendInfo(
        backend_id='classical_watershed',
        version='1.0.0',
        family='classical_cv',
        calibrated=False,
        description=(
            'Otsu foreground, distance-transform markers, watershed split; '
            'heuristic score, no learned weights'),
    )

    def segment(self, image_bgr, config, scene_id):
        """Return watershed-split instance masks."""
        foreground, _ = _foreground(image_bgr, config)
        if not np.any(foreground):
            return []
        distance = cv2.distanceTransform(
            foreground.astype(np.uint8), cv2.DIST_L2, 5)
        params = config['backend']['watershed']
        if params['peak_mode'] == 'local':
            window = int(params['peak_window'])
            local_max = cv2.dilate(
                distance, np.ones((window, window), np.uint8))
            peaks = (
                foreground
                & (distance >= local_max - 1e-6)
                & (distance >= float(params['min_peak_px']))
            )
        else:
            peak_floor = max(
                float(params['min_peak_px']),
                float(params['peak_ratio']) * float(distance.max()))
            peaks = foreground & (distance >= peak_floor)
        count, seed_labels = cv2.connectedComponents(
            peaks.astype(np.uint8), 8)
        if count <= 1:
            return []
        markers = seed_labels.astype(np.int32)
        markers[~foreground] = count
        gradient_blur = int(params['gradient_blur'])
        gradient_image = image_bgr
        if gradient_blur > 1:
            gradient_image = cv2.GaussianBlur(
                image_bgr, (gradient_blur, gradient_blur), 0)
        cv2.watershed(gradient_image, markers)
        return _instances_from_labels(
            markers, foreground, distance, config, scene_id)


class ClassicalConnectedComponentsBackend(SegmentationBackend):
    """Connected-component baseline; touching fragments stay merged."""

    info = BackendInfo(
        backend_id='classical_cc',
        version='1.0.0',
        family='classical_cv',
        calibrated=False,
        description=(
            'Otsu foreground and connected components; no touching-fragment '
            'split; heuristic score, no learned weights'),
    )

    def segment(self, image_bgr, config, scene_id):
        """Return one instance per connected foreground component."""
        foreground, _ = _foreground(image_bgr, config)
        if not np.any(foreground):
            return []
        distance = cv2.distanceTransform(
            foreground.astype(np.uint8), cv2.DIST_L2, 5)
        _, labels = cv2.connectedComponents(
            foreground.astype(np.uint8), 8)
        return _instances_from_labels(
            labels, foreground, distance, config, scene_id)


def build_backend(backend_id):
    """Return a backend instance for a supported identifier."""
    if backend_id == 'classical_watershed':
        return ClassicalWatershedBackend()
    if backend_id == 'classical_cc':
        return ClassicalConnectedComponentsBackend()
    raise ValueError('unsupported backend: %s' % backend_id)
